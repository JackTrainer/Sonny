use std::sync::Arc;
use std::time::Instant;

pub struct FrequencyEnforcer {
    session: Arc<zenoh::Session>,
    hardware_id: String,
    expected_interval_us: u64,
    max_jitter_us: u64,
}

impl FrequencyEnforcer {
    pub fn new(
        session: Arc<zenoh::Session>,
        hardware_id: &str,
        expected_hz: f64,
        max_jitter_us: u64,
    ) -> Self {
        let expected_interval_us = (1_000_000.0 / expected_hz) as u64;
        Self {
            session,
            hardware_id: hardware_id.to_string(),
            expected_interval_us,
            max_jitter_us,
        }
    }

    pub async fn start_monitoring(&self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let topic = format!("alpha/telemetry/{}", self.hardware_id);
        let subscriber = self.session.declare_subscriber(&topic).await?;

        let mut last_timestamp: Option<Instant> = None;

        while let Ok(_sample) = subscriber.recv_async().await {
            let now = Instant::now();

            if let Some(last) = last_timestamp {
                let elapsed_us = now.duration_since(last).as_micros() as u64;

                let diff_us = if elapsed_us > self.expected_interval_us {
                    elapsed_us - self.expected_interval_us
                } else {
                    self.expected_interval_us - elapsed_us
                };

                if diff_us > self.max_jitter_us {
                    eprintln!(
                        "[WARN][frequency_enforcer] Jitter superato sul nodo {}: intervallo atteso {} us, misurato {} us, differenza {} us",
                        self.hardware_id,
                        self.expected_interval_us,
                        elapsed_us,
                        diff_us,
                    );
                }
            }

            last_timestamp = Some(now);
        }

        Ok(())
    }
}
