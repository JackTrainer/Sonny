use std::ptr;
use std::sync::atomic::{AtomicPtr, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};
use crate::io_bridge::vector_sanitizer::VectorSanitizer;
use crate::io_bridge::watchdog_timer::Heartbeat;

/// Core dedicated to the 100 Hz real-time control loop.
///
/// This core hosts the lock-free buffer reads, the [`VectorSanitizer`] and
/// the publication of the command to the HAL: no other shared thread, warm
/// data cache, zero context switches.
pub const RT_CONTROL_CORE: usize = 1;

/// Core dedicated to the Wasmtime runtime.
///
/// The WASM sandbox runs in isolation on this core: even if a third-party
/// Skill takes hundreds of microseconds, Core 1 never waits on it because it
/// reads from the lock-free buffer.
pub const WASM_RUNTIME_CORE: usize = 2;

/// Phase deviation report relative to the theoretical control frame period.
#[derive(Debug, Clone, Copy)]
pub struct JitterReport {
    pub expected_us: u64,
    pub measured_us: u64,
    pub diff_us: u64,
}

impl JitterReport {
    /// Percentage deviation relative to the theoretical period.
    pub fn deviation_pct(&self) -> u64 {
        if self.expected_us == 0 {
            0
        } else {
            self.diff_us * 100 / self.expected_us
        }
    }

    /// Phase deviation quantified in milliseconds.
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
                        "[WARN][frequency_enforcer] Jitter exceeded on node {}: expected interval {} us, measured {} us, difference {} us",
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

    /// Pure analysis of the interval between two successive frames using the
    /// CPU high-precision clock: computes the difference against the theoretical
    /// period and, if it exceeds the industrial tolerance threshold, returns the
    /// jitter report. Never blocks the program: the caller decides.
    pub fn analyze_interval(&self, elapsed_us: u64) -> Option<JitterReport> {
        compute_jitter(self.expected_interval_us, self.max_jitter_us, elapsed_us)
    }
}

/// Pure computation of the jitter report relative to the theoretical period.
///
/// Kept separate from the caller so the analysis core is testable without
/// network sessions: no allocation, no I/O, designed to run on Core 1.
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
// Lock-free circular buffer with capacity 1 (last value wins)
// ---------------------------------------------------------------------------

/// Single-slot lock-free register: the producer (WASM, Core 2) publishes the
/// latest geometric coordinate vector, the consumer (real-time loop, Core 1)
/// reads it without ever blocking.
///
/// Capacity is 1 by design: a deeper buffer would add latency between the
/// Skill and the motors. With a single slot, if the producer is faster than
/// the loop the newest value wins; if the producer is late (peak Wasmtime
/// runtime overhead), the consumer finds an empty slot and reuses the last
/// valid vector: the 100 Hz loop never skips a frame and the watchdog
/// heartbeat never has gaps.
///
/// Synchronization is a single `AtomicPtr`: `AcqRel` on both operations
/// guarantees that data written by the producer is visible to the consumer.
/// Each pointer lives in exactly one of the two sides.
pub struct LatestCommand {
    slot: AtomicPtr<CommandVector>,
}

impl LatestCommand {
    pub const fn new() -> Self {
        Self {
            slot: AtomicPtr::new(ptr::null_mut()),
        }
    }

    /// Non-blocking publication of the latest command produced by the sandbox.
    /// The old value, if present, is released here by the producer: the
    /// consumer can no longer own it because it has been removed from the slot.
    pub fn publish(&self, cmd: CommandVector) {
        let new = Box::into_raw(Box::new(cmd));
        let old = self.slot.swap(new, Ordering::AcqRel);
        if !old.is_null() {
            unsafe { drop(Box::from_raw(old)) };
        }
    }

    /// Non-blocking read: returns the latest published command or `fallback`
    /// if the producer is late. The consumer empties the slot so the next
    /// frame starts again from the last valid value.
    pub fn take_or(&self, fallback: CommandVector) -> CommandVector {
        let ptr = self.slot.swap(ptr::null_mut(), Ordering::AcqRel);
        if ptr.is_null() {
            fallback
        } else {
            unsafe { *Box::from_raw(ptr) }
        }
    }

    /// Optional non-blocking read: `None` if the slot is empty.
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
// 100 Hz real-time control loop (Core 1)
// ---------------------------------------------------------------------------

/// High-frequency command loop isolated on Core 1.
///
/// On every 10 ms tick the loop, without ever waiting on the WASM runtime:
///
/// 1. reads the latest vector from the lock-free buffer (or reuses the last valid one);
/// 2. sanitizes the vector with [`VectorSanitizer`] (NaN/Inf/clamp);
/// 3. beats the heartbeat to the Hard-Watchdog → never an unintended trip;
/// 4. publishes the command to the HAL over Zenoh;
/// 5. measures the frame jitter against the theoretical period.
///
/// The thread owns a private single-threaded Tokio runtime: the kernel
/// schedules it on Core 1 with a warm cache, and no other task steals time
/// from it.
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

    /// Starts the dedicated loop thread and pins it to Core 1.
    pub fn start(self) -> std::io::Result<std::thread::JoinHandle<()>> {
        let handle = std::thread::Builder::new()
            .name("rt-control-100hz".into())
            .spawn(move || {
                if core_affinity::set_for_current(core_affinity::CoreId { id: self.core }) {
                    println!(
                        "[RT] 100 Hz control loop pinned to Core {}.",
                        self.core
                    );
                } else {
                    eprintln!(
                        "[WARN - RT] Failed to pin the loop to Core {}.",
                        self.core
                    );
                }

                let rt = match tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                {
                    Ok(rt) => rt,
                    Err(e) => {
                        eprintln!("[RT] Dedicated runtime unavailable: {}", e);
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

        // Last valid vector: if the slot is empty it is reused as-is, so the
        // kinematic chain never sees a gap.
        let mut last_valid = CommandVector::zeros(self.joint_count);

        let mut ticker = tokio::time::interval(Duration::from_millis(period_ms));
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

        loop {
            ticker.tick().await;
            let t0 = Instant::now();

            // 1. Non-blocking read: WASM late? → last valid.
            let mut cmd = self.buffer.take_or(last_valid);

            // 2. Sanitization on Core 1 (NaN/Inf recovery, physical clamp).
            self.sanitizer.sanitize_and_clamp(&mut cmd, &self.limits);
            last_valid = cmd;

            // 3. Heartbeat: the watchdog never sees a missing frame.
            self.heartbeat.ping();

            // 4. Publication of the command to the HAL.
            let mut buf = [0u8; MAX_JOINTS * 4];
            let n = cmd.write_to(&mut buf);
            if let Err(e) = self.session.put(&cmd_topic, buf[..n].to_vec()).await {
                eprintln!("[RT] Sending command to {}: {}", cmd_topic, e);
            }

            // 5. Frame jitter against the theoretical period.
            let elapsed_us = t0.elapsed().as_micros() as u64;
            if let Some(report) = self.enforcer.analyze_interval(elapsed_us) {
                eprintln!(
                    "[WARN - RT] Frame out of window: expected {} us, measured {} us (deviation {}%)",
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
            "first take must return the published command"
        );
        assert!(buf.take().is_none(), "second take must find an empty slot");
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
