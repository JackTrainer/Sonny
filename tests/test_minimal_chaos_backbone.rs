use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use SONNY::diagnostics::frequency_enforcer::FrequencyEnforcer;
use SONNY::io_bridge::state_vector::StateVector;

// ---------------------------------------------------------------------------
// PRNG deterministico — zero dipendenze extra, caos riproducibile
// ---------------------------------------------------------------------------
struct ChaosRng(u64);

impl ChaosRng {
    fn new(seed: u64) -> Self {
        Self(seed)
    }

    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.0
    }

    fn range_u64(&mut self, lo: u64, hi: u64) -> u64 {
        lo + (self.next() % (hi - lo + 1))
    }
}

// ---------------------------------------------------------------------------
// Metriche atomiche condivise tra i task di caos
// ---------------------------------------------------------------------------
struct ChaosMetrics {
    comau_published:   AtomicU64,
    amr_published:     AtomicU64,
    comau_interrupted: AtomicU64,
    amr_interrupted:   AtomicU64,
    bytes_allocated:   AtomicU64,
    zenoh_errors:      AtomicU64,
}

impl ChaosMetrics {
    fn new() -> Self {
        Self {
            comau_published:   AtomicU64::new(0),
            amr_published:     AtomicU64::new(0),
            comau_interrupted: AtomicU64::new(0),
            amr_interrupted:   AtomicU64::new(0),
            bytes_allocated:   AtomicU64::new(0),
            zenoh_errors:      AtomicU64::new(0),
        }
    }

    fn report(&self) {
        let cp = self.comau_published.load(Ordering::Relaxed);
        let ap = self.amr_published.load(Ordering::Relaxed);
        let ci = self.comau_interrupted.load(Ordering::Relaxed);
        let ai = self.amr_interrupted.load(Ordering::Relaxed);
        let ba = self.bytes_allocated.load(Ordering::Relaxed);
        let ze = self.zenoh_errors.load(Ordering::Relaxed);
        println!("/====================================================\\");
        println!("|              CHAOS BACKBONE — REPORT                |");
        println!("|====================================================|");
        println!("| COMAU pubblicati        : {:>20} |", cp);
        println!("| COMAU interruzioni      : {:>20} |", ci);
        println!("| AMR pubblicati          : {:>20} |", ap);
        println!("| AMR interruzioni        : {:>20} |", ai);
        println!("| Bytes allocati (cum.)   : {:>20} |", ba);
        println!("| Errori Zenoh            : {:>20} |", ze);
        println!("\\====================================================/");
    }
}

// ---------------------------------------------------------------------------
// SIMULATORE 1 — Braccio industriale Comau  500 Hz · 6 DOF
// ---------------------------------------------------------------------------
async fn comau_arm_chaos(
    session: Arc<zenoh::Session>,
    metrics: Arc<ChaosMetrics>,
    stop: Arc<AtomicBool>,
    mut rng: ChaosRng,
) {
    let hardware_id = "COMAU-6DOF-01";
    let topic = format!("alpha/telemetry/{}", hardware_id);
    let mut tick: f64 = 0.0;
    let mut cycle: u64 = 0;

    while !stop.load(Ordering::Relaxed) {
        let values: Vec<f32> = (0..6)
            .map(|j| (tick + j as f64 * 0.5).sin() as f32)
            .collect();
        let state = StateVector::new(hardware_id, values);
        let payload = state.to_bytes();

        if let Err(e) = session.put(&topic, payload).await {
            eprintln!("[CHAOS:COMAU] Errore Zenoh: {:?}", e);
            metrics.zenoh_errors.fetch_add(1, Ordering::Relaxed);
        } else {
            metrics.comau_published.fetch_add(1, Ordering::Relaxed);
            metrics.bytes_allocated.fetch_add(6 * 4, Ordering::Relaxed);
        }

        tick += 0.1;
        cycle += 1;

        // Base: 2000 µs = 500 Hz
        let base = Duration::from_micros(2000);

        // Micro-interruzione elettrica: ogni 10 cicli, gap extra 0.5–3 ms
        if cycle % 10 == 0 {
            let extra = rng.range_u64(500, 3000);
            tokio::time::sleep(base + Duration::from_micros(extra)).await;
            metrics.comau_interrupted.fetch_add(1, Ordering::Relaxed);
        } else {
            tokio::time::sleep(base).await;
        }
    }
}

// ---------------------------------------------------------------------------
// SIMULATORE 2 — Base mobile AMR  20 Hz · 2 DOF
// ---------------------------------------------------------------------------
async fn amr_base_chaos(
    session: Arc<zenoh::Session>,
    metrics: Arc<ChaosMetrics>,
    stop: Arc<AtomicBool>,
    mut rng: ChaosRng,
) {
    let hardware_id = "AMR-MOBILE-01";
    let topic = format!("alpha/telemetry/{}", hardware_id);
    let mut tick: f64 = 0.0;
    let mut cycle: u64 = 0;

    while !stop.load(Ordering::Relaxed) {
        let values: Vec<f32> = (0..2)
            .map(|j| (tick + j as f64 * 0.3).sin() as f32)
            .collect();
        let state = StateVector::new(hardware_id, values);
        let payload = state.to_bytes();

        if let Err(e) = session.put(&topic, payload).await {
            eprintln!("[CHAOS:AMR] Errore Zenoh: {:?}", e);
            metrics.zenoh_errors.fetch_add(1, Ordering::Relaxed);
        } else {
            metrics.amr_published.fetch_add(1, Ordering::Relaxed);
            metrics.bytes_allocated.fetch_add(2 * 4, Ordering::Relaxed);
        }

        tick += 0.1;
        cycle += 1;

        // Base: 50 ms = 20 Hz
        let base = Duration::from_millis(50);

        // Colpo elettrico violento: ogni 3 messaggi, gap extra 5–20 ms
        if cycle % 3 == 0 {
            let extra = rng.range_u64(5_000, 20_000);
            tokio::time::sleep(base + Duration::from_micros(extra)).await;
            metrics.amr_interrupted.fetch_add(1, Ordering::Relaxed);
        } else {
            tokio::time::sleep(base).await;
        }
    }
}

// ---------------------------------------------------------------------------
// STRESS ALLOCATORE — rapida allocazione/rilascio vettori grandi
// ---------------------------------------------------------------------------
async fn vector_memory_stress(
    metrics: Arc<ChaosMetrics>,
    stop: Arc<AtomicBool>,
    mut rng: ChaosRng,
) {
    let mut heap: Vec<Vec<f32>> = Vec::with_capacity(128);

    while !stop.load(Ordering::Relaxed) {
        let count = rng.range_u64(1, 6) as usize;
        for _ in 0..count {
            let size = rng.range_u64(512, 4096) as usize;
            let v: Vec<f32> = (0..size).map(|i| i as f32).collect();
            metrics
                .bytes_allocated
                .fetch_add((v.len() * 4) as u64, Ordering::Relaxed);
            heap.push(v);
        }

        if !heap.is_empty() {
            let drain = rng.range_u64(1, heap.len().min(8) as u64) as usize;
            let new_len = heap.len().saturating_sub(drain);
            heap.truncate(new_len);
        }

        tokio::time::sleep(Duration::from_micros(rng.range_u64(100, 2000))).await;
    }
}

// ---------------------------------------------------------------------------
// TEST — Chaos Backbone Minimal
// ---------------------------------------------------------------------------
#[tokio::test(flavor = "multi_thread")]
async fn minimal_chaos_backbone() {
    const TEST_DURATION_SECS: u64 = 5;

    println!("/====================================================\\");
    println!("|         MINIMAL CHAOS BACKBONE — TEST               |");
    println!("|====================================================|");
    println!("| Flotta: COMAU (500 Hz) + AMR (20 Hz)               |");
    println!("| Durata: {:>44} |", format!("{} secondi", TEST_DURATION_SECS));
    println!("| Stress: StateVector alloc · Zenoh · FrequencyEnf.  |");
    println!("\\====================================================/");

    // ---- 1. Sessione Zenoh ------------------------------------------------
    let session = match zenoh::open(zenoh::Config::default()).await {
        Ok(s) => Arc::new(s),
        Err(e) => {
            eprintln!("[CHAOS] Impossibile aprire Zenoh: {:?}", e);
            eprintln!("[CHAOS] Test saltato — nessun router/discovery disponibile.");
            return;
        }
    };
    println!("[CHAOS] Zenoh agganciato.");

    // ---- 2. Metriche e stop flag -----------------------------------------
    let metrics = Arc::new(ChaosMetrics::new());
    let stop = Arc::new(AtomicBool::new(false));

    // ---- 3. Task: Comau arm (500 Hz, 6 DOF) ------------------------------
    let (s1, m1, st1, r1) = (
        session.clone(),
        metrics.clone(),
        stop.clone(),
        ChaosRng::new(42),
    );
    let _h_comau = tokio::spawn(async move { comau_arm_chaos(s1, m1, st1, r1).await });

    // ---- 4. Task: AMR base (20 Hz, 2 DOF) --------------------------------
    let (s2, m2, st2, r2) = (
        session.clone(),
        metrics.clone(),
        stop.clone(),
        ChaosRng::new(12345),
    );
    let _h_amr = tokio::spawn(async move { amr_base_chaos(s2, m2, st2, r2).await });

    // ---- 5. FrequencyEnforcer — COMAU ------------------------------------
    let fe_comau = FrequencyEnforcer::new(session.clone(), "COMAU-6DOF-01", 500.0, 500);
    let _h_fe_c = tokio::spawn(async move {
        if let Err(e) = fe_comau.start_monitoring().await {
            eprintln!("[CHAOS] FE[COMAU] uscito: {:?}", e);
        }
    });

    // ---- 6. FrequencyEnforcer — AMR ------------------------------------
    let fe_amr = FrequencyEnforcer::new(session.clone(), "AMR-MOBILE-01", 20.0, 5_000);
    let _h_fe_a = tokio::spawn(async move {
        if let Err(e) = fe_amr.start_monitoring().await {
            eprintln!("[CHAOS] FE[AMR] uscito: {:?}", e);
        }
    });

    // ---- 7. Stress memoria vettori --------------------------------------
    let (m3, st3, r3) = (metrics.clone(), stop.clone(), ChaosRng::new(999));
    let _h_mem = tokio::spawn(async move { vector_memory_stress(m3, st3, r3).await });

    // ---- 8. Esecuzione --------------------------------------------------
    let start = Instant::now();
    tokio::time::sleep(Duration::from_secs(TEST_DURATION_SECS)).await;
    let elapsed = start.elapsed();
    stop.store(true, Ordering::Relaxed);

    // ---- 9. Report ------------------------------------------------------
    println!("\n[CHAOS] Tempo reale: {:?}", elapsed);
    metrics.report();

    // ---- 10. Verifiche --------------------------------------------------

    let comau_count = metrics.comau_published.load(Ordering::Relaxed);
    let amr_count = metrics.amr_published.load(Ordering::Relaxed);
    let zenoh_errors = metrics.zenoh_errors.load(Ordering::Relaxed);

    // Attesi ~2500 a 500 Hz; micro-interruzioni + overhead runtime riducono
    assert!(
        comau_count >= 100,
        "COMAU: solo {} messaggi in {}s — rete o scheduler collassati",
        comau_count,
        TEST_DURATION_SECS,
    );

    // Attesi ~100 a 20 Hz; interruzioni ogni 3 messaggi
    assert!(
        amr_count >= 15,
        "AMR: solo {} messaggi in {}s — troppo pochi per essere sani",
        amr_count,
        TEST_DURATION_SECS,
    );

    assert_eq!(
        zenoh_errors, 0,
        "{} errori Zenoh durante il chaos — la rete ha perso colpi",
        zenoh_errors,
    );

    println!("[CHAOS] Il backbone ha retto il caos eterogeneo. ✓");
}
