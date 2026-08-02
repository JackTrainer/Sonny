/// Isolamento del thread critico di controllo: CPU affinity e priorità real-time.
///
/// Il nucleo SONNY gira su board embedded (Jetson, Raspberry Pi) dove il
/// sistema operativo condivide la CPU con servizi di background (log, SSH,
/// container). Se il ciclo di controllo a 100 Hz venisse cacciato da un
/// processo estraneo, il jitter di esecuzione supererebbe la finestra della
/// frequenza enforcer e l'HardWatchdog scatenerebbe un E-stop non desiderato.
///
/// Questo modulo riduce il rischio in due mosse:
///
/// 1. **Affinity** — il thread viene bloccato su un core dedicato/isolato:
///    i cambi di contesto verso altri core spariscono e la cache dati resta
///    calda. Il core 0 è tipicamente riservato al kernel, quindi la scelta
///    predefinita cade sul primo core logico disponibile.
/// 2. **Priorità** — il thread viene promosso a SCHED_FIFO (real-time,
///    non preemptibile da processi a priorità normale). Se i permessi mancano
///    (serve CAP_SYS_NICE o RLIMIT_RTPRIO), si ripiega sul niceness -20.
///
/// Tutte le operazioni sono best-effort e mai fatali: un fallimento produce
/// un warning ma non impedisce l'avvio del nucleo.
///
/// Su piattaforme non-Linux il modulo è un no-op trasparente, così lo sviluppo
/// locale (Windows/macOS) compila e gira senza cambiare comportamento.
#[cfg(target_os = "linux")]
pub fn pin_critical_thread(core: usize) {
    use std::mem::size_of;

    // 1. Verifica che il core richiesto esista davvero.
    let online = unsafe { libc::sysconf(libc::_SC_NPROCESSORS_ONLN) };
    if online <= 0 {
        eprintln!("[WARN - RT] Impossibile contare i core online (sysconf)");
        return;
    }
    if (core as libc::c_long) >= online {
        eprintln!(
            "[WARN - RT] Core {} non disponibile (solo {} core online)",
            core, online
        );
        return;
    }

    // 2. Maschera di affinity contenente esclusivamente il core richiesto.
    let mut set: libc::cpu_set_t = unsafe { std::mem::zeroed() };
    unsafe { libc::CPU_SET(core, &mut set) };
    let rc = unsafe { libc::sched_setaffinity(0, size_of::<libc::cpu_set_t>(), &set) };
    if rc != 0 {
        eprintln!(
            "[WARN - RT] sched_setaffinity(core {}) rifiutata: {}",
            core,
            std::io::Error::last_os_error()
        );
        return;
    }

    // Verifica a posteriori: la maschera applicata dal kernel deve contenere
    // il core richiesto (protegge da kernel che ignorano richieste errate).
    let mut verify: libc::cpu_set_t = unsafe { std::mem::zeroed() };
    let rc = unsafe { libc::sched_getaffinity(0, size_of::<libc::cpu_set_t>(), &mut verify) };
    if rc == 0 && unsafe { libc::CPU_ISSET(core, &verify) } {
        println!("[RT] Thread critico pinnato sul core {} (affinity confermata)", core);
    } else {
        eprintln!("[WARN - RT] Affinity non confermata sul core {}", core);
    }

    // 3. Promozione a priorità real-time con fallback sul niceness -20.
    let prio_min = unsafe { libc::sched_get_priority_min(libc::SCHED_FIFO) };
    let prio_max = unsafe { libc::sched_get_priority_max(libc::SCHED_FIFO) };
    if prio_min < 0 || prio_max < 0 {
        eprintln!("[WARN - RT] sched_get_priority() non disponibile");
        return;
    }

    let mut param: libc::sched_param = unsafe { std::mem::zeroed() };
    param.sched_priority = prio_max;
    let rc = unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) };
    if rc == 0 {
        println!(
            "[RT] Politica real-time SCHED_FIFO attiva (priorità {}/{})",
            prio_max, prio_max
        );
    } else {
        let sched_err = std::io::Error::last_os_error();
        let rc = unsafe { libc::setpriority(libc::PRIO_PROCESS, 0, -20) };
        if rc == 0 {
            eprintln!(
                "[WARN - RT] SCHED_FIFO rifiutata ({}): fallback niceness -20 attivo",
                sched_err
            );
        } else {
            let nice_err = std::io::Error::last_os_error();
            eprintln!(
                "[WARN - RT] Nessuna priorità elevata ottenibile (SCHED_FIFO: {}, nice: {})",
                sched_err, nice_err
            );
        }
    }
}

/// No-op su piattaforme non-Linux: l'affinity e il real-time scheduling sono
/// concetti specifici del kernel Linux e non hanno equivalenti portabili.
#[cfg(not(target_os = "linux"))]
pub fn pin_critical_thread(_core: usize) {}
