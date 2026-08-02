use crate::io_bridge::command_vector::MAX_JOINTS;
use crate::io_bridge::state_vector::StateVector;
use std::sync::Arc;

pub struct AlphaZenohBus {
    session: Arc<zenoh::Session>,
}

impl AlphaZenohBus {
    pub fn new(session: Arc<zenoh::Session>) -> Self {
        Self { session }
    }

    pub async fn transmit_telemetry(
        &self,
        state: &StateVector,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let routing_key = format!("alpha/telemetry/{}", state.hardware_id);
        let mut buf = [0u8; MAX_JOINTS * 4];
        let n = state.write_to(&mut buf);
        self.session.put(&routing_key, &buf[..n]).await?;
        Ok(())
    }
}
