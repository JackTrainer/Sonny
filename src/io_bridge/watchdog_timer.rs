//! # Hard-Watchdog
//!
//! Dedicated ultra-high-priority asynchronous thread that watches the control
//! module's "alive" signal (heartbeat). If the heartbeat does not arrive
//! within the safety window (10 ms by default), the watchdog bypasses the
//! software, zeros the actuators with a `CommandVector` of zeros, and latches
//! the estop: no matter what happens to the AI or a third-party WASM Skill,
//! the robot stops safely.
//!
//! The watchdog lives on a dedicated OS thread with a private single-threaded
//! Tokio runtime: it is the kernel, not Tokio, that schedules it. It therefore
//! stays alive even if the robot's main runtime is stuck in an infinite loop.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};

/// Default window within which the control module must confirm the heartbeat:
/// 10 ms as per specification.
pub const DEFAULT_HEARTBEAT_TIMEOUT_MS: u64 = 10;

/// Periodic check granularity: 1 ms → ~10 samples per window.
pub const DEFAULT_CHECK_INTERVAL_MS: u64 = 1;

/// Interval at which the vector of zeros is re-published to the physical
/// hardware registers while the watchdog is tripped.
pub const DEFAULT_REASSERT_INTERVAL_MS: u64 = 10;

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

/// Shared register for the "alive" signal.
///
/// The control module calls [`Heartbeat::ping`] every command cycle; the
/// watchdog only reads the age of the last beat. No locks: a single
/// `AtomicU64` is sufficient and remains non-blocking for the control loop.
#[derive(Debug)]
pub struct Heartbeat {
    last_ping_ms: AtomicU64,
}

impl Heartbeat {
    /// Initializes the heartbeat as "just beaten": the watchdog considers the
    /// control alive right away, without a false trip at startup.
    pub fn new() -> Self {
        Self {
            last_ping_ms: AtomicU64::new(now_ms()),
        }
    }

    /// To be called by the control module every command cycle (≥ 100 Hz).
    pub fn ping(&self) {
        self.last_ping_ms.store(now_ms(), Ordering::Relaxed);
    }

    /// Age of the last beat in milliseconds.
    fn age_ms(&self) -> u64 {
        now_ms().saturating_sub(self.last_ping_ms.load(Ordering::Relaxed))
    }
}

impl Default for Heartbeat {
    fn default() -> Self {
        Self::new()
    }
}

/// Hard-Watchdog intervention parameters.
#[derive(Debug, Clone)]
pub struct WatchdogConfig {
    /// Maximum accepted delay between two heartbeats before intervention.
    pub heartbeat_timeout_ms: u64,
    /// Polling granularity of the heartbeat register.
    pub check_interval_ms: u64,
    /// Interval for re-publishing the vector of zeros while the watchdog is
    /// tripped (keeps the physical registers forced to zero).
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

/// Hardware safety supervisor.
///
/// On the bus it publishes:
/// - `alpha/cmd/<id>`            → `CommandVector` of zeros;
/// - `alpha/cmd/<id>/estop`      → 1 = engage the latch, 0 = release;
/// - `alpha/watchdog/status/<id>`→ watchdog status telemetry.
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

    /// Number of safety interventions performed since startup.
    pub fn trips(&self) -> u64 {
        self.trips.load(Ordering::Relaxed)
    }

    /// `true` if the watchdog is currently tripped (actuators zeroed).
    pub fn is_tripped(&self) -> bool {
        self.tripped.load(Ordering::Relaxed)
    }

    /// Requests an orderly shutdown of the loop (supervised teardown and tests).
    /// In production the watchdog stays armed for the robot's whole life.
    pub fn shutdown(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }

    /// Starts the dedicated ultra-high-priority Hard-Watchdog thread.
    ///
    /// The thread owns a private single-threaded Tokio runtime: if the main
    /// runtime gets blocked by a malicious WASM Skill, this thread keeps beating
    /// anyway because the OS schedules it.
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
                        eprintln!("[HARD-WATCHDOG] Dedicated runtime unavailable: {}", e);
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

        // Pre-allocated safety vector: zeros serialized once into a static stack
        // buffer, reused on every intervention.
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
                    // Alive → dead transition: safety intervention.
                    self.trips.fetch_add(1, Ordering::Relaxed);
                    self.intervene(&cmd_topic, &estop_topic, &status_topic, zero_bytes)
                        .await;
                    last_force_at = tokio::time::Instant::now();
                    eprintln!(
                        "[HARD-WATCHDOG] TRIP on {}: heartbeat silent for {} ms (timeout {} ms). Actuators zeroed.",
                        self.hardware_id, age, self.config.heartbeat_timeout_ms
                    );
                } else if last_force_at.elapsed()
                    >= Duration::from_millis(self.config.reassert_interval_ms)
                {
                    // Keep forcing the physical registers to zero until the
                    // control comes back alive.
                    self.reassert_zeros(&cmd_topic, zero_bytes).await;
                    last_force_at = tokio::time::Instant::now();
                }
            } else if self.tripped.swap(false, Ordering::Relaxed) {
                // Dead → alive transition: release the safety latch.
                self.release_estop(&estop_topic).await;
                println!(
                    "[HARD-WATCHDOG] Heartbeat restored on {}: safety latch released.",
                    self.hardware_id
                );
            }

            tokio::time::sleep(Duration::from_millis(self.config.check_interval_ms)).await;
        }
    }

    /// Full intervention: vector of zeros to the registers + engage estop.
    async fn intervene(
        &self,
        cmd_topic: &str,
        estop_topic: &str,
        status_topic: &str,
        zero_bytes: &[u8],
    ) {
        if let Err(e) = self.session.put(cmd_topic, zero_bytes.to_vec()).await {
            eprintln!("[HARD-WATCHDOG] Sending zeros to {}: {}", cmd_topic, e);
        }
        if let Err(e) = self.session.put(estop_topic, vec![1u8]).await {
            eprintln!("[HARD-WATCHDOG] Engaging estop on {}: {}", estop_topic, e);
        }
        self.publish_status(status_topic).await;
    }

    /// Periodic re-publication of the vector of zeros only (the estop state
    /// stays latched from the initial intervention).
    async fn reassert_zeros(&self, cmd_topic: &str, zero_bytes: &[u8]) {
        if let Err(e) = self.session.put(cmd_topic, zero_bytes.to_vec()).await {
            eprintln!("[HARD-WATCHDOG] Sending zeros to {}: {}", cmd_topic, e);
        }
    }

    /// Releases the safety latch when the heartbeat returns to normal.
    async fn release_estop(&self, estop_topic: &str) {
        if let Err(e) = self.session.put(estop_topic, vec![0u8]).await {
            eprintln!("[HARD-WATCHDOG] Releasing estop on {}: {}", estop_topic, e);
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
            eprintln!("[HARD-WATCHDOG] Watchdog status publication: {}", e);
        }
    }
}

// ---------------------------------------------------------------------------
// Dedicated thread priority: THREAD_PRIORITY_TIME_CRITICAL on Windows
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
    // On POSIX real-time priority requires dedicated privileges (SCHED_RR) and
    // `std::thread::Builder` offers no hook: this remains a deliberate no-op.
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
        assert!(age >= 15, "heartbeat must age: age {}", age);

        hb.ping();
        assert!(hb.age_ms() <= 1, "ping must reset the age");
    }

    #[test]
    fn zero_vector_is_encoded_as_all_zero_bytes() {
        let cmd = CommandVector::zeros(3);
        let mut buf = [0u8; MAX_JOINTS * 4];
        let n = cmd.write_to(&mut buf);
        assert_eq!(&buf[..n], &[0u8; 12][..]);
    }
}
