use std::ptr;
use std::sync::atomic::{AtomicPtr, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};
use crate::io_bridge::vector_sanitizer::VectorSanitizer;
use crate::io_bridge::watchdog_timer::Heartbeat;

/// Core dedicato al loop di controllo real-time a 100 Hz.
///
/// Su questo core vivono la lettura del buffer lock-free, il
/// [`VectorSanitizer`] e la pubblicazione del comando verso l'HAL: nessun
/// altro thread condiviso, cache dati calda, zero cambi di contesto.
pub const RT_CONTROL_CORE: usize = 1;

/// Core dedicato al runtime Wasmtime.
///
/// Il sandbox WASM esegue in isolamento su questo core: anche se una Skill di
/// terze parti impiega centinaia di microsecondi, il Core 1 non lo aspetta
/// mai perché legge dal buffer lock-free.
pub const WASM_RUNTIME_CORE: usize = 2;

/// Rapporto di sfasamento rispetto al periodo teorico del frame di controllo.
#[derive(Debug, Clone, Copy)]
pub struct JitterReport {
    pub expected_us: u64,
    pub measured_us: u64,
    pub diff_us: u64,
}

impl JitterReport {
    /// Deviazione percentuale rispetto al periodo teorico.
    pub fn deviation_pct(&self) -> u64 {
        if self.expected_us == 0 {
            0
        } else {
            self.diff_us * 100 / self.expected_us
        }
    }

    /// Sfasamento quantificato in millisecondi.
    pub fn jitter_ms(&self) -> f64 {
        self.diff_us as f64 / 1000.0
    }
}

pub struct FrequencyEnforcer {
    session: Arc<zenoh::Session>,
    hardware_id: String,
    expected_interval_us: u64,
    max_jitter_us: u64,
}

impl FrequencyEnforcer {
    pub fn new(
        session: Arc<zenoh::Session>,
        hardware_id: &str,
        expected_hz: f64,
        max_jitter_us: u64,
    ) -> Self {
        let expected_interval_us = (1_000_000.0 / expected_hz) as u64;
        Self {
            session,
            hardware_id: hardware_id.to_string(),
            expected_interval_us,
            max_jitter_us,
        }
    }

    pub async fn start_monitoring(&self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let topic = format!("alpha/telemetry/{}", self.hardware_id);
        let subscriber = self.session.declare_subscriber(&topic).await?;

        let mut last_timestamp: Option<Instant> = None;

        while let Ok(_sample) = subscriber.recv_async().await {
            let now = Instant::now();

            if let Some(last) = last_timestamp {
                let elapsed_us = now.duration_since(last).as_micros() as u64;
                if let Some(report) = self.analyze_interval(elapsed_us) {
                    eprintln!(
                        "[WARN][frequency_enforcer] Jitter superato sul nodo {}: intervallo atteso {} us, misurato {} us, differenza {} us",
                        self.hardware_id,
                        report.expected_us,
                        report.measured_us,
                        report.diff_us,
                    );
                }
            }

            last_timestamp = Some(now);
        }

        Ok(())
    }

    /// Analisi pura dell'intervallo tra due frame successivi usando l'orologio
    /// ad alta precisione della CPU: calcola la differenza rispetto al periodo
    /// teorico e, se supera la soglia di tolleranza industriale, restituisce il
    /// rapporto di jitter. Non blocca mai il programma: il chiamante decide.
    pub fn analyze_interval(&self, elapsed_us: u64) -> Option<JitterReport> {
        compute_jitter(self.expected_interval_us, self.max_jitter_us, elapsed_us)
    }
}

/// Calcolo puro del rapporto di jitter rispetto al periodo teorico.
///
/// Separato dal chiamante così il nucleo di analisi è testabile senza sessioni
/// di rete: nessuna allocazione, nessun I/O, pensato per girare sul Core 1.
pub fn compute_jitter(
    expected_interval_us: u64,
    max_jitter_us: u64,
    elapsed_us: u64,
) -> Option<JitterReport> {
    let diff_us = elapsed_us.abs_diff(expected_interval_us);
    if diff_us > max_jitter_us {
        Some(JitterReport {
            expected_us: expected_interval_us,
            measured_us: elapsed_us,
            diff_us,
        })
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// Buffer circolare lock-free a capacità 1 (ultimo valore vince)
// ---------------------------------------------------------------------------

/// Registro lock-free a singolo slot: il produttore (WASM, Core 2) pubblica
/// l'ultimo vettore di coordinate geometriche, il consumatore (loop real-time,
/// Core 1) lo legge senza mai bloccarsi.
///
/// La capacità è 1 per design: un buffer più profondo aggiungerebbe latenza
/// tra la Skill e i motori. Con un solo slot, se il produttore è più veloce
/// del loop il valore più recente vince; se il produttore è in ritardo (picco
/// di overhead del runtime Wasmtime), il consumatore trova lo slot vuoto e
/// riusa l'ultimo vettore valido: il ciclo a 100 Hz non salta mai un frame e
/// l'heartbeat al watchdog non ha mai buchi.
///
/// La sincronizzazione è un singolo `AtomicPtr`: `AcqRel` su entrambe le
/// operazioni garantisce che i dati scritti dal produttore siano visibili al
/// consumatore. Ogni puntatore vive in esattamente una delle due parti.
pub struct LatestCommand {
    slot: AtomicPtr<CommandVector>,
}

impl LatestCommand {
    pub const fn new() -> Self {
        Self {
            slot: AtomicPtr::new(ptr::null_mut()),
        }
    }

    /// Pubblicazione non bloccante dell'ultimo comando prodotto dal sandbox.
    /// Il vecchio valore, se presente, viene rilasciato qui dal produttore:
    /// il consumatore non può più possederlo perché è stato rimosso dallo slot.
    pub fn publish(&self, cmd: CommandVector) {
        let new = Box::into_raw(Box::new(cmd));
        let old = self.slot.swap(new, Ordering::AcqRel);
        if !old.is_null() {
            unsafe { drop(Box::from_raw(old)) };
        }
    }

    /// Lettura non bloccante: restituisce l'ultimo comando pubblicato oppure
    /// `fallback` se il produttore è in ritardo. Il consumatore svuota lo slot
    /// così il frame successivo riparte dall'ultimo valore valido.
    pub fn take_or(&self, fallback: CommandVector) -> CommandVector {
        let ptr = self.slot.swap(ptr::null_mut(), Ordering::AcqRel);
        if ptr.is_null() {
            fallback
        } else {
            unsafe { *Box::from_raw(ptr) }
        }
    }

    /// Lettura non bloccante opzionale: `None` se lo slot è vuoto.
    pub fn take(&self) -> Option<CommandVector> {
        let ptr = self.slot.swap(ptr::null_mut(), Ordering::AcqRel);
        if ptr.is_null() {
            None
        } else {
            Some(unsafe { *Box::from_raw(ptr) })
        }
    }
}

impl Default for LatestCommand {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Loop di controllo real-time a 100 Hz (Core 1)
// ---------------------------------------------------------------------------

/// Ciclo di comando ad alta frequenza isolato sul Core 1.
///
/// Ad ogni tick da 10 ms il loop, senza attendere mai il runtime WASM:
///
/// 1. legge l'ultimo vettore dal buffer lock-free (o riusa l'ultimo valido);
/// 2. sanifica il vettore con [`VectorSanitizer`] (NaN/Inf/clamp);
/// 3. batte l'heartbeat verso l'Hard-Watchdog → mai uno scatto indebito;
/// 4. pubblica il comando verso l'HAL su Zenoh;
/// 5. misura lo jitter del frame rispetto al periodo teorico.
///
/// Il thread possiede un runtime Tokio mono-thread privato: è il kernel a
/// schedularlo sul Core 1, con la cache calda, e nessun'altra task gli ruba
/// tempo.
pub struct RealTimeControlLoop {
    session: Arc<zenoh::Session>,
    hardware_id: String,
    heartbeat: Arc<Heartbeat>,
    buffer: Arc<LatestCommand>,
    sanitizer: VectorSanitizer,
    joint_count: usize,
    limits: Vec<(f32, f32)>,
    enforcer: FrequencyEnforcer,
    core: usize,
}

impl RealTimeControlLoop {
    pub fn new(
        session: Arc<zenoh::Session>,
        hardware_id: &str,
        heartbeat: Arc<Heartbeat>,
        buffer: Arc<LatestCommand>,
        joint_count: usize,
        limits: Vec<(f32, f32)>,
        expected_hz: f64,
        max_jitter_us: u64,
    ) -> Self {
        Self {
            enforcer: FrequencyEnforcer::new(
                session.clone(),
                hardware_id,
                expected_hz,
                max_jitter_us,
            ),
            session,
            hardware_id: hardware_id.to_string(),
            heartbeat,
            buffer,
            sanitizer: VectorSanitizer::new(),
            joint_count,
            limits,
            core: RT_CONTROL_CORE,
        }
    }

    /// Avvia il thread dedicato del loop e lo vincola al Core 1.
    pub fn start(self) -> std::io::Result<std::thread::JoinHandle<()>> {
        let handle = std::thread::Builder::new()
            .name("rt-control-100hz".into())
            .spawn(move || {
                if core_affinity::set_for_current(core_affinity::CoreId { id: self.core }) {
                    println!(
                        "[RT] Loop di controllo a 100 Hz pinnato sul Core {}.",
                        self.core
                    );
                } else {
                    eprintln!(
                        "[WARN - RT] Pinning del loop sul Core {} non riuscito.",
                        self.core
                    );
                }

                let rt = match tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                {
                    Ok(rt) => rt,
                    Err(e) => {
                        eprintln!("[RT] Runtime dedicato non disponibile: {}", e);
                        return;
                    }
                };
                rt.block_on(self.run());
            })?;
        Ok(handle)
    }

    async fn run(mut self) {
        let cmd_topic = format!("alpha/cmd/{}", self.hardware_id);
        let period_ms = (self.enforcer.expected_interval_us / 1000).max(1);

        // Ultimo vettore valido: se lo slot è vuoto viene riusato tale e
        // quale, quindi la catena cinematica non vede mai un buco.
        let mut last_valid = CommandVector::zeros(self.joint_count);

        let mut ticker = tokio::time::interval(Duration::from_millis(period_ms));
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

        loop {
            ticker.tick().await;
            let t0 = Instant::now();

            // 1. Lettura non bloccante: WASM in ritardo? → ultimo valido.
            let mut cmd = self.buffer.take_or(last_valid);

            // 2. Sanitizzazione sul Core 1 (NaN/Inf ripristino, clamp fisico).
            self.sanitizer.sanitize_and_clamp(&mut cmd, &self.limits);
            last_valid = cmd;

            // 3. Heartbeat: il watchdog non vede mai un frame mancante.
            self.heartbeat.ping();

            // 4. Pubblicazione del comando verso l'HAL.
            let mut buf = [0u8; MAX_JOINTS * 4];
            let n = cmd.write_to(&mut buf);
            if let Err(e) = self.session.put(&cmd_topic, buf[..n].to_vec()).await {
                eprintln!("[RT] Invio comando a {}: {}", cmd_topic, e);
            }

            // 5. Jitter del frame rispetto al periodo teorico.
            let elapsed_us = t0.elapsed().as_micros() as u64;
            if let Some(report) = self.enforcer.analyze_interval(elapsed_us) {
                eprintln!(
                    "[WARN - RT] Frame fuori finestra: atteso {} us, misurato {} us (deviazione {}%)",
                    report.expected_us,
                    report.measured_us,
                    report.deviation_pct(),
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_slot_falls_back_to_last_valid() {
        let buf = LatestCommand::new();
        let last = CommandVector::from_slice(&[1.0, 2.0, 3.0]);
        let got = buf.take_or(last);
        assert_eq!(got.as_slice(), &[1.0, 2.0, 3.0][..]);
        assert!(buf.take().is_none());
    }

    #[test]
    fn published_command_is_consumed_once() {
        let buf = LatestCommand::new();
        buf.publish(CommandVector::from_slice(&[0.5, -0.5]));
        assert_eq!(
            buf.take().unwrap().as_slice(),
            &[0.5, -0.5][..],
            "il primo take deve restituire il comando pubblicato"
        );
        assert!(buf.take().is_none(), "il secondo take deve trovare slot vuoto");
    }

    #[test]
    fn newest_publish_wins_and_frees_previous() {
        let buf = LatestCommand::new();
        buf.publish(CommandVector::from_slice(&[1.0]));
        buf.publish(CommandVector::from_slice(&[2.0]));
        buf.publish(CommandVector::from_slice(&[3.0]));
        assert_eq!(buf.take().unwrap().as_slice(), &[3.0][..]);
    }

    #[test]
    fn producer_consumer_across_threads_without_loss() {
        const FRAMES: usize = 10_000;
        let buf = Arc::new(LatestCommand::new());

        let producer = {
            let buf = buf.clone();
            std::thread::spawn(move || {
                for i in 0..FRAMES {
                    buf.publish(CommandVector::from_slice(&[i as f32]));
                }
            })
        };

        let consumer = {
            let buf = buf.clone();
            std::thread::spawn(move || {
                let mut last = CommandVector::zeros(1);
                for _ in 0..FRAMES {
                    let cmd = buf.take_or(last);
                    assert_eq!(cmd.len, 1);
                    last = cmd;
                }
            })
        };

        producer.join().unwrap();
        consumer.join().unwrap();
    }

    #[test]
    fn analysis_reports_jitter_only_outside_window() {
        assert!(compute_jitter(10_000, 1_000, 10_000).is_none());
        assert!(compute_jitter(10_000, 1_000, 10_100).is_none());
        let report = compute_jitter(10_000, 1_000, 11_100).unwrap();
        assert_eq!(report.expected_us, 10_000);
        assert_eq!(report.diff_us, 1_100);
        assert_eq!(report.deviation_pct(), 11);
    }
}
