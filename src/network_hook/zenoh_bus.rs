use std::sync::Arc;
use crate::io_bridge::state_vector::StateVector;

pub struct AlphaZenohBus {
    session: Arc<zenoh::Session>,
}

impl AlphaZenohBus {
    pub fn new(session: Arc<zenoh::Session>) -> Self {
        Self { session }
    }

    pub async fn transmit_telemetry(&self, state: &StateVector) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let routing_key = format!("alpha/telemetry/{}", state.hardware_id);
        let payload = state.to_bytes();
        self.session.put(&routing_key, payload).await?;
        Ok(())
    }
}
