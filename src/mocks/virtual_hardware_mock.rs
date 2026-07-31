use std::sync::Arc;
use std::time::Duration;
use tokio::time::sleep;
use crate::io_bridge::hardware_abstraction::{FieldbusConfig, HardwareAbstraction};
use crate::io_bridge::state_vector::StateVector;

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
        let mut tick: f64 = 0.0;
        loop {
            let mut values = Vec::with_capacity(self.joint_count);
            for j in 0..self.joint_count {
                let phase = j as f64 * 0.5;
                let val = (tick + phase).sin() as f32;
                values.push(val);
            }

            let state = StateVector::new(&self.hardware_id, values);
            let topic = format!("alpha/telemetry/{}", self.hardware_id);
            self.session.put(&topic, state.to_bytes()).await?;

            tick += 0.1;
            sleep(Duration::from_millis(10)).await;
        }
    }
}
