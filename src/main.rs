use std::env;
use std::error::Error;
use std::sync::Arc;

use SONNY::diagnostics;
use SONNY::io_bridge;
use SONNY::io_bridge::hardware_abstraction::{FieldbusConfig, FieldbusTransport};
use SONNY::mocks;
use SONNY::registry;
use SONNY::ros2_bridge;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
    println!("=== AVVIO NUCLEO - SONNY ===");

    let config_path = env::var("ROBOT_CONFIG_PATH")
        .or_else(|_| env::args().nth(1).ok_or("Nessun argomento"));

    let (robot_name, expected_hz, max_jitter_us, dof) = match config_path {
        Ok(path) => {
            let config = io_bridge::hal_loader::HalLoader::from_file(&path)?;
            println!(
                "[BOOT] Config caricata: {} ({} DOF) di {}",
                config.robot_name, config.dof, config.manufacturer
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
                config.robot_name,
                config.expected_hz,
                config.max_jitter_us,
                config.dof,
            )
        }
        Err(_) => {
            println!("[BOOT] Nessuna configurazione fornita. Avvio modalità mock.");
            ("MockRobot".into(), 100.0, 1000, 6)
        }
    };

    let zenoh_session = Arc::new(zenoh::open(zenoh::Config::default()).await?);
    println!("[BOOT] Bus Zenoh agganciato correttamente.");

    let name_for_mock = robot_name.clone();
    let session_for_mock = zenoh_session.clone();
    tokio::spawn(async move {
        let mut mock = mocks::virtual_hardware_mock::VirtualHardwareMock::new(
            session_for_mock,
            &name_for_mock,
            dof,
        );

        if let Ok(port) = env::var("ROBOT_SERIAL_PORT") {
            let baud: u32 = env::var("ROBOT_SERIAL_BAUD")
                .unwrap_or_else(|_| "115200".into())
                .parse()
                .unwrap_or(115200);
            let config = FieldbusConfig {
                transport: FieldbusTransport::Serial { port, baud_rate: baud },
                hardware_id: name_for_mock.clone(),
                joint_count: dof,
                frame_timeout_ms: 100,
                reconnect_delay_ms: 2000,
            };
            mock = mock.with_fieldbus(config);
            println!("[BOOT] Fieldbus seriale configurato.");
        } else if let Ok(bind) = env::var("ROBOT_UDP_BIND") {
            let remote = env::var("ROBOT_UDP_REMOTE")
                .expect("ROBOT_UDP_REMOTE necessario con ROBOT_UDP_BIND");
            let config = FieldbusConfig {
                transport: FieldbusTransport::Udp { bind_addr: bind, remote_addr: remote },
                hardware_id: name_for_mock.clone(),
                joint_count: dof,
                frame_timeout_ms: 100,
                reconnect_delay_ms: 2000,
            };
            mock = mock.with_fieldbus(config);
            println!("[BOOT] Fieldbus UDP configurato.");
        } else {
            println!("[BOOT] Nessun fieldbus configurato. Uso mock simulato.");
        }

        if let Err(e) = mock.start_loop().await {
            eprintln!("[ERROR] Mock hardware: {:?}", e);
        }
    });

    let session_for_skills = zenoh_session.clone();
    tokio::spawn(async move {
        if let Err(e) = registry::wasm_runner::listen_for_skills(session_for_skills).await {
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

    let ui = diagnostics::terminal_ui::TerminalUi::new(&robot_name);
    tokio::spawn(async move {
        ui.start_render_loop().await;
    });

    println!("[BOOT] Sistema pronto su hardware: {}", robot_name);
    tokio::signal::ctrl_c().await?;
    println!("=== SPEGNIMENTO ===");
    Ok(())
}
