use crate::io_bridge::command_vector::MAX_JOINTS;
use crate::io_bridge::state_vector::StateVector;
use std::sync::Arc;

pub struct TelemetryClient {
    zenoh_session: Arc<zenoh::Session>,
}

impl TelemetryClient {
    pub fn new(session: Arc<zenoh::Session>) -> Self {
        Self {
            zenoh_session: session,
        }
    }

    pub async fn push_to_brain(
        &self,
        state: &StateVector,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let brain_topic = format!("alpha/brain/{}/input", state.hardware_id);
        let mut buf = [0u8; MAX_JOINTS * 4];
        let n = state.write_to(&mut buf);
        self.zenoh_session.put(&brain_topic, &buf[..n]).await?;
        Ok(())
    }
}
