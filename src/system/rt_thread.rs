/// Isolation of the critical control thread: CPU affinity and real-time priority.
///
/// The SONNY core runs on embedded boards (Jetson, Raspberry Pi) where the
/// operating system shares the CPU with background services (log, SSH,
/// containers). If the 100 Hz control loop were evicted by an external
/// process, the execution jitter would exceed the frequency enforcer window
/// and the HardWatchdog would trigger an unwanted E-stop.
///
/// This module reduces the risk in two steps:
///
/// 1. Affinity: the thread is pinned to a dedicated/isolated core:
///    context switches to other cores disappear and the data cache stays
///    warm. Core 0 is typically reserved for the kernel, so the default
///    choice falls on the first available logical core.
/// 2. Priority: the thread is promoted to SCHED_FIFO (real-time, not
///    preemptible by normal-priority processes). If the permissions are
///    missing (requires CAP_SYS_NICE or RLIMIT_RTPRIO), it falls back to
///    niceness -20.
///
/// All operations are best-effort and never fatal: a failure produces a
/// warning but does not prevent the core from starting.
///
/// On non-Linux platforms the module is a transparent no-op, so local
/// development (Windows/macOS) compiles and runs without changing behavior.
#[cfg(target_os = "linux")]
pub fn pin_critical_thread(core: usize) {
    use std::mem::size_of;

    // 1. Verify that the requested core actually exists.
    let online = unsafe { libc::sysconf(libc::_SC_NPROCESSORS_ONLN) };
    if online <= 0 {
        eprintln!("[WARN - RT] Cannot count online cores (sysconf)");
        return;
    }
    if (core as libc::c_long) >= online {
        eprintln!(
            "[WARN - RT] Core {} unavailable (only {} cores online)",
            core, online
        );
        return;
    }

    // 2. Affinity mask containing exclusively the requested core.
    let mut set: libc::cpu_set_t = unsafe { std::mem::zeroed() };
    unsafe { libc::CPU_SET(core, &mut set) };
    let rc = unsafe { libc::sched_setaffinity(0, size_of::<libc::cpu_set_t>(), &set) };
    if rc != 0 {
        eprintln!(
            "[WARN - RT] sched_setaffinity(core {}) rejected: {}",
            core,
            std::io::Error::last_os_error()
        );
        return;
    }

    // Post-check: the mask applied by the kernel must contain the requested
    // core (protects against kernels that ignore invalid requests).
    let mut verify: libc::cpu_set_t = unsafe { std::mem::zeroed() };
    let rc = unsafe { libc::sched_getaffinity(0, size_of::<libc::cpu_set_t>(), &mut verify) };
    if rc == 0 && unsafe { libc::CPU_ISSET(core, &verify) } {
        println!("[RT] Critical thread pinned to core {} (affinity confirmed)", core);
    } else {
        eprintln!("[WARN - RT] Affinity not confirmed on core {}", core);
    }

    // 3. Promotion to real-time priority with fallback to niceness -20.
    let prio_min = unsafe { libc::sched_get_priority_min(libc::SCHED_FIFO) };
    let prio_max = unsafe { libc::sched_get_priority_max(libc::SCHED_FIFO) };
    if prio_min < 0 || prio_max < 0 {
        eprintln!("[WARN - RT] sched_get_priority() unavailable");
        return;
    }

    let mut param: libc::sched_param = unsafe { std::mem::zeroed() };
    param.sched_priority = prio_max;
    let rc = unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) };
    if rc == 0 {
        println!(
            "[RT] Real-time policy SCHED_FIFO active (priority {}/{})",
            prio_max, prio_max
        );
    } else {
        let sched_err = std::io::Error::last_os_error();
        let rc = unsafe { libc::setpriority(libc::PRIO_PROCESS, 0, -20) };
        if rc == 0 {
            eprintln!(
                "[WARN - RT] SCHED_FIFO rejected ({}): niceness -20 fallback active",
                sched_err
            );
        } else {
            let nice_err = std::io::Error::last_os_error();
            eprintln!(
                "[WARN - RT] No elevated priority obtainable (SCHED_FIFO: {}, nice: {})",
                sched_err, nice_err
            );
        }
    }
}

/// No-op on non-Linux platforms: affinity and real-time scheduling are
/// Linux-kernel-specific concepts with no portable equivalents.
#[cfg(not(target_os = "linux"))]
pub fn pin_critical_thread(_core: usize) {}
