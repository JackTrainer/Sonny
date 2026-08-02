//! # Hard-Watchdog
//!
//! Thread asincrono dedicato ad altissima priorità che sorveglia il segnale
//! "sono vivo" (heartbeat) del modulo di controllo. Se l'heartbeat non arriva
//! entro la finestra di sicurezza (10 ms di default), il watchdog scavalca il
//! software, azzera gli azionamenti con un `CommandVector` di zeri e aggancia
//! l'estop: qualunque cosa accada all'IA o a una Skill WASM di terze parti,
//! il robot si ferma in sicurezza.
//!
//! Il watchdog vive su un thread OS dedicato con runtime Tokio mono-thread
//! privato: è il kernel, non Tokio, a schedularlo. Così resta vivo anche se il
//! runtime principale del robot è bloccato da un ciclo infinito.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};

/// Finestra di default entro cui il modulo di controllo deve confermare
/// l'heartbeat: 10 ms come da specifica.
pub const DEFAULT_HEARTBEAT_TIMEOUT_MS: u64 = 10;

/// Granularità del controllo periodico: 1 ms → ~10 campioni per finestra.
pub const DEFAULT_CHECK_INTERVAL_MS: u64 = 1;

/// Intervallo con cui, a watchdog scattato, viene ripubblicato il vettore di
/// zeri verso i registri fisici dell'hardware.
pub const DEFAULT_REASSERT_INTERVAL_MS: u64 = 10;

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

/// Registro condiviso del segnale "sono vivo".
///
/// Il modulo di controllo chiama [`Heartbeat::ping`] a ogni ciclo di comando;
/// il watchdog legge solo l'età dell'ultimo battito. Niente lock: un singolo
/// `AtomicU64` è sufficiente e resta non bloccante per il loop di controllo.
#[derive(Debug)]
pub struct Heartbeat {
    last_ping_ms: AtomicU64,
}

impl Heartbeat {
    /// Inizializza l'heartbeat "appena battuto": il watchdog dà per vivo il
    /// controllo già da subito, senza falso scatto all'avvio.
    pub fn new() -> Self {
        Self {
            last_ping_ms: AtomicU64::new(now_ms()),
        }
    }

    /// Da chiamare dal modulo di controllo a ogni ciclo di comando (≥ 100 Hz).
    pub fn ping(&self) {
        self.last_ping_ms.store(now_ms(), Ordering::Relaxed);
    }

    /// Età dell'ultimo battito in millisecondi.
    fn age_ms(&self) -> u64 {
        now_ms().saturating_sub(self.last_ping_ms.load(Ordering::Relaxed))
    }
}

impl Default for Heartbeat {
    fn default() -> Self {
        Self::new()
    }
}

/// Parametri di intervento dell'Hard-Watchdog.
#[derive(Debug, Clone)]
pub struct WatchdogConfig {
    /// Ritardo massimo accettato tra due heartbeat prima dell'intervento.
    pub heartbeat_timeout_ms: u64,
    /// Granularità del polling del registro heartbeat.
    pub check_interval_ms: u64,
    /// Intervallo di ripubblicazione del vettore di zeri mentre il watchdog è
    /// scattato (mantiene forzati a zero i registri fisici).
    pub reassert_interval_ms: u64,
}

impl Default for WatchdogConfig {
    fn default() -> Self {
        Self {
            heartbeat_timeout_ms: DEFAULT_HEARTBEAT_TIMEOUT_MS,
            check_interval_ms: DEFAULT_CHECK_INTERVAL_MS,
            reassert_interval_ms: DEFAULT_REASSERT_INTERVAL_MS,
        }
    }
}

/// Sorvegliante di sicurezza hardware.
///
/// Sul bus pubblica:
/// - `alpha/cmd/<id>`            → `CommandVector` di zeri;
/// - `alpha/cmd/<id>/estop`      → 1 = aggancia il latch, 0 = rilascia;
/// - `alpha/watchdog/status/<id>`→ telemetria di stato del watchdog.
pub struct HardWatchdog {
    session: Arc<zenoh::Session>,
    hardware_id: String,
    joint_count: usize,
    config: WatchdogConfig,
    heartbeat: Arc<Heartbeat>,
    tripped: AtomicBool,
    trips: AtomicU64,
    stop: AtomicBool,
}

impl HardWatchdog {
    pub fn new(
        session: Arc<zenoh::Session>,
        heartbeat: Arc<Heartbeat>,
        hardware_id: &str,
        joint_count: usize,
        config: WatchdogConfig,
    ) -> Self {
        Self {
            session,
            hardware_id: hardware_id.to_string(),
            joint_count,
            config,
            heartbeat,
            tripped: AtomicBool::new(false),
            trips: AtomicU64::new(0),
            stop: AtomicBool::new(false),
        }
    }

    /// Numero di interventi di sicurezza eseguiti dall'avvio.
    pub fn trips(&self) -> u64 {
        self.trips.load(Ordering::Relaxed)
    }

    /// `true` se il watchdog è attualmente scattato (azionamenti a zero).
    pub fn is_tripped(&self) -> bool {
        self.tripped.load(Ordering::Relaxed)
    }

    /// Richiede l'arresto ordinato del loop (smontaggio supervisionato e test).
    /// In produzione il watchdog resta armato per tutta la vita del robot.
    pub fn shutdown(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }

    /// Avvia il thread dedicato Hard-Watchdog ad altissima priorità.
    ///
    /// Il thread possiede un runtime Tokio mono-thread privato: se il runtime
    /// principale viene bloccato da una Skill WASM malvagia, questo thread
    /// continua comunque a battere perché è l'OS a schedularlo.
    pub fn start(self: Arc<Self>) -> std::io::Result<std::thread::JoinHandle<()>> {
        let handle = std::thread::Builder::new()
            .name("hard-watchdog".into())
            .spawn(move || {
                boost_thread_priority();
                let rt = match tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                {
                    Ok(rt) => rt,
                    Err(e) => {
                        eprintln!("[HARD-WATCHDOG] Runtime dedicato non disponibile: {}", e);
                        return;
                    }
                };
                rt.block_on(self.monitor_loop());
            })?;
        Ok(handle)
    }

    async fn monitor_loop(self: Arc<Self>) {
        let cmd_topic = format!("alpha/cmd/{}", self.hardware_id);
        let estop_topic = format!("alpha/cmd/{}/estop", self.hardware_id);
        let status_topic = format!("alpha/watchdog/status/{}", self.hardware_id);

        // Vettore di sicurezza pre-allocato: zeri serializzati una sola volta
        // in un buffer statico a stack, riutilizzato a ogni intervento.
        let zero_cmd = CommandVector::zeros(self.joint_count);
        let mut zero_buf = [0u8; MAX_JOINTS * 4];
        let zero_len = zero_cmd.write_to(&mut zero_buf);
        let zero_bytes = &zero_buf[..zero_len];

        let mut last_force_at = tokio::time::Instant::now();

        loop {
            if self.stop.load(Ordering::Relaxed) {
                if self.tripped.swap(false, Ordering::Relaxed) {
                    self.release_estop(&estop_topic).await;
                }
                break;
            }

            let age = self.heartbeat.age_ms();

            if age > self.config.heartbeat_timeout_ms {
                if !self.tripped.swap(true, Ordering::Relaxed) {
                    // Transizione vivo → morto: intervento di sicurezza.
                    self.trips.fetch_add(1, Ordering::Relaxed);
                    self.intervene(&cmd_topic, &estop_topic, &status_topic, zero_bytes)
                        .await;
                    last_force_at = tokio::time::Instant::now();
                    eprintln!(
                        "[HARD-WATCHDOG] SCATTO su {}: heartbeat muto da {} ms (timeout {} ms). Azionamenti azzerati.",
                        self.hardware_id, age, self.config.heartbeat_timeout_ms
                    );
                } else if last_force_at.elapsed()
                    >= Duration::from_millis(self.config.reassert_interval_ms)
                {
                    // Continua a mantenere i registri fisici a zero finché il
                    // controllo non torna vivo.
                    self.reassert_zeros(&cmd_topic, zero_bytes).await;
                    last_force_at = tokio::time::Instant::now();
                }
            } else if self.tripped.swap(false, Ordering::Relaxed) {
                // Transizione morto → vivo: rilascio del latch di sicurezza.
                self.release_estop(&estop_topic).await;
                println!(
                    "[HARD-WATCHDOG] Heartbeat ripristinato su {}: latch di sicurezza rilasciato.",
                    self.hardware_id
                );
            }

            tokio::time::sleep(Duration::from_millis(self.config.check_interval_ms)).await;
        }
    }

    /// Intervento completo: vettore di zeri ai registri + aggancio estop.
    async fn intervene(
        &self,
        cmd_topic: &str,
        estop_topic: &str,
        status_topic: &str,
        zero_bytes: &[u8],
    ) {
        if let Err(e) = self.session.put(cmd_topic, zero_bytes.to_vec()).await {
            eprintln!("[HARD-WATCHDOG] Invio zeri a {}: {}", cmd_topic, e);
        }
        if let Err(e) = self.session.put(estop_topic, vec![1u8]).await {
            eprintln!("[HARD-WATCHDOG] Aggancio estop a {}: {}", estop_topic, e);
        }
        self.publish_status(status_topic).await;
    }

    /// Ripubblicazione periodica del solo vettore di zeri (lo stato di estop
    /// resta latched dall'intervento iniziale).
    async fn reassert_zeros(&self, cmd_topic: &str, zero_bytes: &[u8]) {
        if let Err(e) = self.session.put(cmd_topic, zero_bytes.to_vec()).await {
            eprintln!("[HARD-WATCHDOG] Invio zeri a {}: {}", cmd_topic, e);
        }
    }

    /// Rilascia il latch di sicurezza quando l'heartbeat torna regolare.
    async fn release_estop(&self, estop_topic: &str) {
        if let Err(e) = self.session.put(estop_topic, vec![0u8]).await {
            eprintln!("[HARD-WATCHDOG] Rilascio estop a {}: {}", estop_topic, e);
        }
    }

    async fn publish_status(&self, status_topic: &str) {
        let status = serde_json::json!({
            "hardware_id": self.hardware_id,
            "tripped": self.tripped.load(Ordering::Relaxed),
            "trips": self.trips.load(Ordering::Relaxed),
            "timeout_ms": self.config.heartbeat_timeout_ms,
        });
        if let Err(e) = self
            .session
            .put(status_topic, status.to_string().into_bytes())
            .await
        {
            eprintln!("[HARD-WATCHDOG] Pubblicazione stato watchdog: {}", e);
        }
    }
}

// ---------------------------------------------------------------------------
// Priorità del thread dedicato: THREAD_PRIORITY_TIME_CRITICAL su Windows
// ---------------------------------------------------------------------------
#[cfg(target_os = "windows")]
#[link(name = "kernel32")]
extern "system" {
    fn GetCurrentThread() -> *mut core::ffi::c_void;
    fn SetThreadPriority(thread: *mut core::ffi::c_void, priority: i32) -> i32;
}

#[cfg(target_os = "windows")]
fn boost_thread_priority() {
    // winbase.h: THREAD_PRIORITY_TIME_CRITICAL = 15
    const THREAD_PRIORITY_TIME_CRITICAL: i32 = 15;
    unsafe {
        let _ = SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
    }
}

#[cfg(not(target_os = "windows"))]
fn boost_thread_priority() {
    // Su POSIX la priorità real-time richiede privilegi dedicati (SCHED_RR) e
    // `std::thread::Builder` non offre l'hook: qui resta un no-op voluto.
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_spec() {
        let cfg = WatchdogConfig::default();
        assert_eq!(cfg.heartbeat_timeout_ms, DEFAULT_HEARTBEAT_TIMEOUT_MS);
        assert_eq!(cfg.check_interval_ms, DEFAULT_CHECK_INTERVAL_MS);
        assert_eq!(cfg.reassert_interval_ms, DEFAULT_REASSERT_INTERVAL_MS);
    }

    #[test]
    fn heartbeat_ages_when_silent_and_resets_on_ping() {
        let hb = Heartbeat::new();
        hb.ping();
        assert!(hb.age_ms() <= 1);

        std::thread::sleep(Duration::from_millis(15));
        let age = hb.age_ms();
        assert!(age >= 15, "l'heartbeat deve invecchiare: età {}", age);

        hb.ping();
        assert!(hb.age_ms() <= 1, "il ping deve azzerare l'età");
    }

    #[test]
    fn zero_vector_is_encoded_as_all_zero_bytes() {
        let cmd = CommandVector::zeros(3);
        let mut buf = [0u8; MAX_JOINTS * 4];
        let n = cmd.write_to(&mut buf);
        assert_eq!(&buf[..n], &[0u8; 12][..]);
    }
}
