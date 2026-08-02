use crate::io_bridge::state_vector::StateVector;
use std::fs::OpenOptions;
use std::io::Write;

pub struct TelemetryLogger {
    file_path: String,
}

impl TelemetryLogger {
    pub fn new(file_path: &str) -> Self {
        Self {
            file_path: file_path.to_string(),
        }
    }

    /// Registra il vettore di stato in un file di log locale in modo asincrono
    pub fn log_state(&self, state: &StateVector) -> Result<(), Box<dyn std::error::Error>> {
        let mut file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&self.file_path)?;

        // Formatta i dati in una stringa JSON compatta per non pesare sulla CPU
        let log_line = format!(
            "{{\"timestamp\":{},\"hardware\":\"{}\",\"values\":{:?}}}\n",
            state.timestamp,
            state.hardware_id,
            state.as_slice()
        );

        file.write_all(log_line.as_bytes())?;
        Ok(())
    }
}
