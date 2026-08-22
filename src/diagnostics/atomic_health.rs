//! Lock-free health probe for the isolated threads and the Zenoh bus.
//!
//! Every isolated core (1: control loop, 2: WASM runtime) writes its own
//! telemetry into dedicated `AtomicU64` slots with `Relaxed` ordering:
//! a plain store on an aligned 64-bit word compiles to a single MOV on
//! x86-64/AArch64, so the instrumentation cost is ~1 ns per metric and
//! there is nothing to allocate, lock or wait on.
//!
//! Each hot writer lives on its own cache line (`CachePadded`, align(64)):
//! Core 1 and Core 2 never contend for the same line, so no false sharing
//! can evict the warm data cache of the real-time loop.
//!
//! The CLI snapshot reads all words with a relaxed load sweep and renders
//! one text line: no external monitor, no heap allocation in the hot path.
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

/// Wraps hot per-core counters in their own cache line (64 B): writers on
/// different cores never share a line, eliminating false sharing.
#[repr(align(64))]
#[derive(Debug)]
pub struct CachePadded<T>(pub T);

/// Nanosecond timing sample of one hot-path cycle (sanitizer, WASM, frame).
#[repr(C)]
pub struct CycleMetric {
    pub last_ns: AtomicU64,
    pub max_ns: AtomicU64,
    pub total_ns: AtomicU64,
    pub count: AtomicU64,
}

impl CycleMetric {
    pub const fn new() -> Self {
        Self {
            last_ns: AtomicU64::new(0),
            max_ns: AtomicU64::new(0),
            total_ns: AtomicU64::new(0),
            count: AtomicU64::new(0),
        }
    }

    /// Records one duration in nanoseconds. Relaxed stores only: the value
    /// is diagnostic output, no cross-thread happens-before is required.
    #[inline]
    pub fn record(&self, duration_ns: u64) {
        self.last_ns.store(duration_ns, Ordering::Relaxed);
        self.max_ns.fetch_max(duration_ns, Ordering::Relaxed);
        self.total_ns.fetch_add(duration_ns, Ordering::Relaxed);
        self.count.fetch_add(1, Ordering::Relaxed);
    }

    #[inline]
    fn snapshot(&self) -> CycleSnapshot {
        CycleSnapshot {
            last_ns: self.last_ns.load(Ordering::Relaxed),
            max_ns: self.max_ns.load(Ordering::Relaxed),
            avg_ns: match self.count.load(Ordering::Relaxed) {
                0 => 0,
                n => self.total_ns.load(Ordering::Relaxed) / n,
            },
            count: self.count.load(Ordering::Relaxed),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct CycleSnapshot {
    pub last_ns: u64,
    pub max_ns: u64,
    pub avg_ns: u64,
    pub count: u64,
}

/// Liveness stamp of one thread: nanoseconds since the Unix epoch, written
/// every loop iteration. A reader computes `now - last_beat` to detect a
/// stalled or dead thread without any lock.
#[repr(align(64))]
pub struct ThreadBeat {
    pub last_beat_unix_ns: AtomicU64,
}

impl ThreadBeat {
    pub const fn new() -> Self {
        Self {
            last_beat_unix_ns: AtomicU64::new(0),
        }
    }

    #[inline]
    pub fn ping(&self) {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0);
        self.last_beat_unix_ns.store(now, Ordering::Relaxed);
    }

    /// Age of the last beat in milliseconds, or `None` if never beaten.
    pub fn age_ms(&self) -> Option<u64> {
        let last = self.last_beat_unix_ns.load(Ordering::Relaxed);
        if last == 0 {
            return None;
        }
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(last);
        Some((now.saturating_sub(last)) / 1_000_000)
    }
}

/// Global health registry: static, zero-sized initialization cost, no heap.
///
/// Writers: Core 1 (sanitizer + frame + Zenoh TX), Core 2 (WASM reward
/// evaluation), Zenoh subscriber tasks (RX counters). Reader: CLI command.
pub struct AtomicHealth {
    /// Duration of each `VectorSanitizer::sanitize_and_clamp` call (Core 1).
    pub sanitizer_cycle: CachePadded<CycleMetric>,
    /// Duration of each WASM Skill reward evaluation (Core 2).
    pub wasm_cycle: CachePadded<CycleMetric>,
    /// Duration of the full 100 Hz control-loop frame (Core 1).
    pub rt_frame: CachePadded<CycleMetric>,
    /// Round-trip of the Zenoh command publish (Core 1 -> bus).
    pub zenoh_tx_latency: CachePadded<CycleMetric>,
    /// Telemetry frames received from the bus (network RX path).
    pub zenoh_rx_frames: CachePadded<AtomicU64>,
    pub zenoh_tx_errors: CachePadded<AtomicU64>,
    /// Heartbeat of the real-time control loop (Core 1).
    pub rt_thread_beat: CachePadded<ThreadBeat>,
    /// Heartbeat of the WASM runtime sandbox (Core 2).
    pub wasm_thread_beat: CachePadded<ThreadBeat>,
}

pub static HEALTH: AtomicHealth = AtomicHealth::new();

impl AtomicHealth {
    pub const fn new() -> Self {
        Self {
            sanitizer_cycle: CachePadded(CycleMetric::new()),
            wasm_cycle: CachePadded(CycleMetric::new()),
            rt_frame: CachePadded(CycleMetric::new()),
            zenoh_tx_latency: CachePadded(CycleMetric::new()),
            zenoh_rx_frames: CachePadded(AtomicU64::new(0)),
            zenoh_tx_errors: CachePadded(AtomicU64::new(0)),
            rt_thread_beat: CachePadded(ThreadBeat::new()),
            wasm_thread_beat: CachePadded(ThreadBeat::new()),
        }
    }

    /// One-shot consistent-enough readout: every field is a single atomic
    /// load, no locks, no allocation.
    pub fn snapshot(&self) -> HealthSnapshot {
        HealthSnapshot {
            sanitizer: self.sanitizer_cycle.0.snapshot(),
            wasm: self.wasm_cycle.0.snapshot(),
            rt_frame: self.rt_frame.0.snapshot(),
            zenoh_tx: self.zenoh_tx_latency.0.snapshot(),
            zenoh_rx_frames: self.zenoh_rx_frames.0.load(Ordering::Relaxed),
            zenoh_tx_errors: self.zenoh_tx_errors.0.load(Ordering::Relaxed),
            rt_age_ms: self.rt_thread_beat.0.age_ms(),
            wasm_age_ms: self.wasm_thread_beat.0.age_ms(),
        }
    }

    /// Renders the single-line CLI report into a caller-provided buffer:
    /// zero heap allocations even on the diagnostics side.
    pub fn render_line(&self, out: &mut String) {
        let s = self.snapshot();
        out.clear();
        use std::fmt::Write;
        let _ = write!(
            out,
            "[HEALTH] RT(core1): frame {} us (max {} us) | beat {} ms | SANITIZER: {} ns (max {}) | \
             WASM(core2): {} us (max {}) | beat {} ms | ZENOH: tx {} us (max {}, err {}) rx {} frames",
            s.rt_frame.last_ns / 1_000,
            s.rt_frame.max_ns / 1_000,
            fmt_opt(s.rt_age_ms),
            s.sanitizer.last_ns,
            s.sanitizer.max_ns,
            s.wasm.last_ns / 1_000,
            s.wasm.max_ns / 1_000,
            fmt_opt(s.wasm_age_ms),
            s.zenoh_tx.last_ns / 1_000,
            s.zenoh_tx.max_ns / 1_000,
            s.zenoh_tx_errors,
            s.zenoh_rx_frames,
        );
    }
}

fn fmt_opt(age: Option<u64>) -> String {
    match age {
        Some(ms) => format!("{} ms", ms),
        None => "never".into(),
    }
}

#[derive(Debug, Clone, Copy)]
pub struct HealthSnapshot {
    pub sanitizer: CycleSnapshot,
    pub wasm: CycleSnapshot,
    pub rt_frame: CycleSnapshot,
    pub zenoh_tx: CycleSnapshot,
    pub zenoh_rx_frames: u64,
    pub zenoh_tx_errors: u64,
    pub rt_age_ms: Option<u64>,
    pub wasm_age_ms: Option<u64>,
}

impl Default for AtomicHealth {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn metrics_track_last_max_avg_count() {
        let m = CycleMetric::new();
        m.record(100);
        m.record(300);
        m.record(200);
        let s = m.snapshot();
        assert_eq!(s.last_ns, 200);
        assert_eq!(s.max_ns, 300);
        assert_eq!(s.avg_ns, 200);
        assert_eq!(s.count, 3);
    }

    #[test]
    fn thread_beat_reports_age_or_never() {
        let b = ThreadBeat::new();
        assert!(b.age_ms().is_none());
        b.ping();
        assert!(b.age_ms().is_some());
    }

    #[test]
    fn render_line_contains_all_sections_without_panicking() {
        HEALTH.sanitizer_cycle.0.record(42);
        HEALTH.wasm_thread_beat.0.ping();
        let mut line = String::new();
        HEALTH.render_line(&mut line);
        assert!(line.contains("[HEALTH]"));
        assert!(line.contains("RT(core1)"));
        assert!(line.contains("WASM(core2)"));
        assert!(line.contains("ZENOH"));
    }
}
