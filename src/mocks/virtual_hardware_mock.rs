use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};
use crate::io_bridge::hardware_abstraction::{FieldbusConfig, HardwareAbstraction};
use crate::io_bridge::state_vector::StateVector;
use std::sync::Arc;
use std::time::Duration;
use tokio::time::{sleep_until, Instant};

pub struct VirtualHardwareMock {
    session: Arc<zenoh::Session>,
    hardware_id: String,
    joint_count: usize,
    fieldbus: Option<FieldbusConfig>,
}

impl VirtualHardwareMock {
    pub fn new(session: Arc<zenoh::Session>, hardware_id: &str, joint_count: usize) -> Self {
        Self {
            session,
            hardware_id: hardware_id.to_string(),
            joint_count,
            fieldbus: None,
        }
    }

    pub fn with_fieldbus(mut self, config: FieldbusConfig) -> Self {
        self.fieldbus = Some(config);
        self
    }

    pub async fn start_loop(&self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        match &self.fieldbus {
            Some(config) => {
                let ha = HardwareAbstraction::new(self.session.clone(), config.clone());
                ha.start_loop().await
            }
            None => self.run_simulated_loop().await,
        }
    }

    async fn run_simulated_loop(&self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let telemetry_topic = format!("alpha/telemetry/{}", self.hardware_id);
        let status_topic = format!("alpha/status/{}", self.hardware_id);
        let cmd_topic = format!("alpha/cmd/{}", self.hardware_id);
        let estop_topic = format!("alpha/cmd/{}/estop", self.hardware_id);

        let cmd_sub = self.session.declare_subscriber(&cmd_topic).await?;
        let estop_sub = self.session.declare_subscriber(&estop_topic).await?;

        let mut targets = [0.0f32; MAX_JOINTS];
        let mut current = [0.0f32; MAX_JOINTS];
        let mut values = [0.0f32; MAX_JOINTS];
        let mut command_received = false;
        let mut estop = false;
        let mut frames: u64 = 0;
        let mut tick: f64 = 0.0;
        let interval = Duration::from_millis(10);
        let mut next_tick = Instant::now();

        // Pre-allocazione all'avvio del robot: StateVector e buffer di
        // serializzazione creati UNA volta e riutilizzati a ogni tick.
        // Durante il loop a 100 Hz non avviene alcuna allocazione dinamica.
        let mut state = StateVector::new(&self.hardware_id, &[]);
        let mut tx_buf = [0u8; MAX_JOINTS * 4];

        loop {
            tokio::select! {
                _ = sleep_until(next_tick) => {
                    let active = self.joint_count.min(MAX_JOINTS);
                    for j in 0..active {
                        let val = if command_received {
                            // Convergenza veloce verso il target di comando.
                            let c = current[j] + (targets[j] - current[j]) * 0.2;
                            current[j] = c;
                            c
                        } else {
                            let phase = j as f64 * 0.5;
                            (tick + phase).sin() as f32
                        };
                        values[j] = if estop { 0.0 } else { val };
                    }

                    state.set_values(&values[..active]);
                    let written = state.write_to(&mut tx_buf);
                    self.session.put(&telemetry_topic, &tx_buf[..written]).await?;

                    frames += 1;
                    if frames % 100 == 0 {
                        self.publish_status(&status_topic, frames, estop).await?;
                    }

                    tick += 0.1;
                    next_tick += interval;
                }
                cmd = cmd_sub.recv_async() => {
                    if let Ok(sample) = cmd {
                        if !estop {
                            let bytes = sample.payload().to_bytes();
                            if bytes.len() % 4 == 0 {
                                let command = CommandVector::from_bytes(bytes.as_ref());
                                command_received = true;
                                for (t, v) in targets.iter_mut().zip(command.as_slice()) {
                                    *t = *v;
                                }
                                println!("[MOCK] Comando ricevuto: {:?}", command.as_slice());
                            }
                        }
                    }
                }
                estop_sample = estop_sub.recv_async() => {
                    if let Ok(sample) = estop_sample {
                        let active = sample.payload().to_bytes().first().copied().unwrap_or(0) != 0;
                        if active != estop {
                            estop = active;
                            targets = [0.0f32; MAX_JOINTS];
                            current = [0.0f32; MAX_JOINTS];
                            println!(
                                "[MOCK] ESTOP {}",
                                if estop { "ATTIVO" } else { "disattivato" }
                            );
                            self.publish_status(&status_topic, frames, estop).await?;
                        }
                    }
                }
            }
        }
    }

    async fn publish_status(
        &self,
        status_topic: &str,
        frames: u64,
        estop: bool,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let status = serde_json::json!({
            "online": true,
            "transport": "SIMULATED",
            "frames": frames,
            "estop": estop,
        });
        self.session
            .put(status_topic, status.to_string().into_bytes())
            .await?;
        Ok(())
    }
}
