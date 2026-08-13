use std::env;
use std::error::Error;
use std::sync::Arc;
use std::time::Duration;

use SONNY::diagnostics;
use SONNY::diagnostics::frequency_enforcer::{LatestCommand, RealTimeControlLoop};
use SONNY::io_bridge;
use SONNY::io_bridge::hal_loader::HardwareConfig;
use SONNY::io_bridge::hardware_abstraction::{FieldbusConfig, FieldbusTransport};
use SONNY::io_bridge::robot_brand::RobotBrand;
use SONNY::io_bridge::watchdog_timer::{HardWatchdog, Heartbeat, WatchdogConfig};
use SONNY::mocks;
use SONNY::registry;
use SONNY::ros2_bridge;
use SONNY::system;

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
    println!("=== AVVIO NUCLEO - SONNY ===");

    // Il ciclo di controllo vive interamente su questo thread: lo si isola su
    // un core dedicato con priorità real-time prima di avviare qualunque task.
    system::rt_thread::pin_critical_thread(0);

    let config_path =
        env::var("ROBOT_CONFIG_PATH").or_else(|_| env::args().nth(1).ok_or("Nessun argomento"));

    // ------------------------------------------------------------------
    // 1. BOOT: configurazione hardware (file JSON o modalità mock)
    // ------------------------------------------------------------------
    let (config, robot_name, expected_hz, max_jitter_us, dof) = match config_path {
        Ok(path) => {
            let config = io_bridge::hal_loader::HalLoader::from_file(&path)?;
            println!(
                "[BOOT] Config caricata: {} ({} DOF) di {}, profilo compatibilità: {}",
                config.robot_name,
                config.dof,
                config.manufacturer,
                config.resolved_brand().label()
            );
            for joint in &config.joints {
                println!(
                    "  -> Giunto {}: [{:.2}, {:.2}] rad, home {:.2}",
                    joint.name, joint.min_angle_rad, joint.max_angle_rad, joint.home_angle_rad
                );
            }
            println!(
                "[BOOT] Frequenza attesa: {} Hz, jitter max: {} us",
                config.expected_hz, config.max_jitter_us
            );
            (
                Some(config.clone()),
                config.robot_name,
                config.expected_hz,
                config.max_jitter_us,
                config.dof,
            )
        }
        Err(_) => {
            println!("[BOOT] Nessuna configurazione fornita. Avvio modalità mock.");
            (None, "MockRobot".into(), 100.0, 1000, 6)
        }
    };

    let joint_names: Vec<String> = config
        .as_ref()
        .map(|c| c.joints.iter().map(|j| j.name.clone()).collect())
        .unwrap_or_else(|| (0..dof).map(|i| format!("J{}", i + 1)).collect());

    let joint_limits: Vec<(f32, f32)> = config
        .as_ref()
        .map(|c| {
            c.joints
                .iter()
                .map(|j| (j.min_angle_rad, j.max_angle_rad))
                .collect()
        })
        .unwrap_or_default();

    let zenoh_session = Arc::new(zenoh::open(zenoh::Config::default()).await?);
    println!("[BOOT] Bus Zenoh agganciato correttamente.");

    // Canale di comunicazione tra il runtime WASM (Core 2) e il loop di
    // controllo (Core 1): buffer circolare lock-free a capacità 1. Il Core 1
    // legge senza mai attendere WASM; se lo slot è vuoto riusa l'ultimo
    // vettore valido e il ciclo a 100 Hz non salta mai.
    let command_buffer = Arc::new(LatestCommand::new());

    // Heartbeat unico condiviso tra loop di controllo e Hard-Watchdog: è il
    // Core 1 a batterlo a ogni frame, il watchdog sorveglia solo l'età.
    let control_heartbeat = Arc::new(Heartbeat::new());

    // ------------------------------------------------------------------
    // 2. Configurazione fieldbus: file JSON > variabili d'ambiente
    // ------------------------------------------------------------------
    let fieldbus_config = resolve_fieldbus(&config).await?;

    let name_for_mock = robot_name.clone();
    let session_for_mock = zenoh_session.clone();
    tokio::spawn(async move {
        let mut mock = mocks::virtual_hardware_mock::VirtualHardwareMock::new(
            session_for_mock,
            &name_for_mock,
            dof,
        );

        if let Some(config) = fieldbus_config {
            mock = mock.with_fieldbus(config);
        }

        if let Err(e) = mock.start_loop().await {
            eprintln!("[ERROR] Mock hardware: {:?}", e);
        }
    });

    let session_for_skills = zenoh_session.clone();
    let buffer_for_skills = command_buffer.clone();
    tokio::spawn(async move {
        if let Err(e) = registry::wasm_runner::listen_for_skills(session_for_skills, buffer_for_skills).await {
            eprintln!("[ERROR] Registry Skill: {:?}", e);
        }
    });

    let session_for_ros2 = zenoh_session.clone();
    tokio::spawn(async move {
        ros2_bridge::topic_converter::start_bridge(session_for_ros2).await;
    });

    let name_for_freq = robot_name.clone();
    let session_for_freq = zenoh_session.clone();
    tokio::spawn(async move {
        let enforcer = diagnostics::frequency_enforcer::FrequencyEnforcer::new(
            session_for_freq,
            &name_for_freq,
            expected_hz,
            max_jitter_us,
        );
        if let Err(e) = enforcer.start_monitoring().await {
            eprintln!("[ERROR] FrequencyEnforcer: {:?}", e);
        }
    });

    // ------------------------------------------------------------------
    // Hard-Watchdog: thread ad altissima priorità che sorveglia il cuore
    // del controllo. Se il modulo di controllo non batte l'heartbeat ogni
    // 10 ms, azzera gli azionamenti e aggancia l'estop, scavalcando
    // qualunque Skill WASM bloccata.
    // ------------------------------------------------------------------
    let session_for_wd = zenoh_session.clone();
    let name_for_wd = robot_name.clone();
    let heartbeat_for_wd = control_heartbeat.clone();
    tokio::spawn(async move {
        let watchdog = Arc::new(HardWatchdog::new(
            session_for_wd.clone(),
            heartbeat_for_wd.clone(),
            &name_for_wd,
            dof,
            WatchdogConfig::default(),
        ));
        if let Err(e) = watchdog.start() {
            eprintln!("[ERROR] Hard-Watchdog: {:?}", e);
            return;
        }
        println!(
            "[BOOT] Hard-Watchdog armato su {} (timeout heartbeat {} ms).",
            name_for_wd,
            WatchdogConfig::default().heartbeat_timeout_ms
        );

        // Il battito lo genera il loop di controllo a 100 Hz sul Core 1: se
        // quel thread muore o si blocca, il watchdog scatta. Il task qui si
        // limita a mantenere vivo il monitor del watchdog (nessun pinger di
        // ripiego: il cuore vero batte solo dal Core 1).
        loop {
            tokio::time::sleep(Duration::from_secs(3600)).await;
        }
    });

    // ------------------------------------------------------------------
    // Loop di controllo real-time a 100 Hz (Core 1): legge dal buffer
    // lock-free senza attendere WASM, sanifica il vettore, batte
    // l'heartbeat e pubblica il comando verso l'HAL.
    // ------------------------------------------------------------------
    let session_for_rt = zenoh_session.clone();
    let name_for_rt = robot_name.clone();
    let hb_for_rt = control_heartbeat.clone();
    let buffer_for_rt = command_buffer.clone();
    let limits_for_rt = joint_limits.clone();
    tokio::spawn(async move {
        let rt_loop = RealTimeControlLoop::new(
            session_for_rt,
            &name_for_rt,
            hb_for_rt,
            buffer_for_rt,
            dof,
            limits_for_rt,
            expected_hz,
            max_jitter_us,
        );
        match rt_loop.start() {
            Ok(_) => println!("[BOOT] Loop di controllo 100 Hz avviato sul Core 1."),
            Err(e) => eprintln!("[ERROR] Loop real-time 100 Hz: {:?}", e),
        }
    });

    let ui = diagnostics::terminal_ui::TerminalUi::new(
        zenoh_session,
        &robot_name,
        expected_hz,
        joint_names,
        joint_limits,
    );
    tokio::spawn(async move {
        ui.start_render_loop().await;
    });

    let brand_label = config
        .as_ref()
        .map(|c| c.resolved_brand().label())
        .unwrap_or("GENERIC");
    println!(
        "[BOOT] Sistema pronto su hardware: {} (profilo: {})",
        robot_name, brand_label
    );
    tokio::signal::ctrl_c().await?;
    println!("=== SPEGNIMENTO ===");
    Ok(())
}

/// Risolve il fieldbus reale: prima dal file di config, poi dalle variabili
/// d'ambiente, infine nessuno (mock simulato puro).
async fn resolve_fieldbus(
    config: &Option<HardwareConfig>,
) -> Result<Option<FieldbusConfig>, Box<dyn Error + Send + Sync>> {
    if let Some(config) = config {
        match config.to_fieldbus_config() {
            Ok(Some(cfg)) => {
                println!("[BOOT] Fieldbus dal file di config: {:?}", cfg.transport);
                return Ok(Some(cfg));
            }
            Ok(None) => {}
            Err(e) => eprintln!("[BOOT] Config fieldbus ignorata: {}", e),
        }
    }

    let joint_limits: Vec<(f32, f32)> = config
        .as_ref()
        .map(|c| {
            c.joints
                .iter()
                .map(|j| (j.min_angle_rad, j.max_angle_rad))
                .collect()
        })
        .unwrap_or_default();

    let expected_hz = config.as_ref().map(|c| c.expected_hz).unwrap_or(100.0);
    let max_speeds: Vec<f32> = config
        .as_ref()
        .map(|c| c.joints.iter().map(|j| j.max_speed_rad_s).collect())
        .unwrap_or_default();
    let max_accels: Vec<f32> = config
        .as_ref()
        .map(|c| c.joints.iter().map(|j| j.max_accel()).collect())
        .unwrap_or_default();
    let max_jerks: Vec<f32> = config
        .as_ref()
        .map(|c| c.joints.iter().map(|j| j.max_jerk()).collect())
        .unwrap_or_default();

    if let Ok(port) = env::var("ROBOT_SERIAL_PORT") {
        let baud: u32 = env::var("ROBOT_SERIAL_BAUD")
            .unwrap_or_else(|_| "115200".into())
            .parse()
            .unwrap_or(115200);
        let hardware_id = config
            .as_ref()
            .map(|c| c.robot_name.clone())
            .unwrap_or_else(|| env::var("ROBOT_NAME").unwrap_or_else(|_| "Robot".into()));
        let joint_count = config.as_ref().map(|c| c.dof).unwrap_or_else(|| {
            env::var("ROBOT_DOF")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(6)
        });
        let fieldbus = FieldbusConfig {
            transport: FieldbusTransport::Serial {
                port,
                baud_rate: baud,
            },
            hardware_id,
            joint_count,
            frame_timeout_ms: 100,
            reconnect_delay_ms: 2000,
            joint_limits_rad: joint_limits,
            expected_hz,
            max_speed_rad_s: max_speeds,
            max_accel_rad_s2: max_accels,
            max_jerk_rad_s3: max_jerks,
            brand: config
                .as_ref()
                .map(|c| c.resolved_brand())
                .unwrap_or(RobotBrand::Generic),
        };
        println!("[BOOT] Fieldbus seriale configurato da ambiente.");
        return Ok(Some(fieldbus));
    }

    if let Ok(bind) = env::var("ROBOT_UDP_BIND") {
        let remote =
            env::var("ROBOT_UDP_REMOTE").expect("ROBOT_UDP_REMOTE necessario con ROBOT_UDP_BIND");
        let hardware_id = config
            .as_ref()
            .map(|c| c.robot_name.clone())
            .unwrap_or_else(|| env::var("ROBOT_NAME").unwrap_or_else(|_| "Robot".into()));
        let joint_count = config.as_ref().map(|c| c.dof).unwrap_or(6);
        let fieldbus = FieldbusConfig {
            transport: FieldbusTransport::Udp {
                bind_addr: bind,
                remote_addr: remote,
            },
            hardware_id,
            joint_count,
            frame_timeout_ms: 100,
            reconnect_delay_ms: 2000,
            joint_limits_rad: joint_limits,
            expected_hz,
            max_speed_rad_s: max_speeds,
            max_accel_rad_s2: max_accels,
            max_jerk_rad_s3: max_jerks,
            brand: config
                .as_ref()
                .map(|c| c.resolved_brand())
                .unwrap_or(RobotBrand::Generic),
        };
        println!("[BOOT] Fieldbus UDP configurato da ambiente.");
        return Ok(Some(fieldbus));
    }

    println!("[BOOT] Nessun fieldbus configurato. Uso mock simulato.");
    Ok(None)
}
