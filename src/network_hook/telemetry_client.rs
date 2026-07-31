use std::sync::Arc;
use crate::io_bridge::state_vector::StateVector;

pub struct TelemetryClient {
    zenoh_session: Arc<zenoh::Session>,
}

impl TelemetryClient {
    pub fn new(session: Arc<zenoh::Session>) -> Self {
        Self { zenoh_session: session }
    }

    pub async fn push_to_brain(&self, state: &StateVector) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let brain_topic = format!("alpha/brain/{}/input", state.hardware_id);
        let payload = state.to_bytes();
        self.zenoh_session.put(&brain_topic, payload).await?;
        Ok(())
    }
}
