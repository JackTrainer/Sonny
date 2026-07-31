/// Rappresentazione matematica standardizzata della telemetria in ingresso
#[derive(Debug, Clone)]
pub struct StateVector {
    pub hardware_id: String,
    pub timestamp: u64,
    pub values: Vec<f32>, // Sensori, giunti e contatti linearizzati
}

impl StateVector {
    pub fn new(hardware_id: &str, values: Vec<f32>) -> Self {
        Self {
            hardware_id: hardware_id.to_string(),
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64,
            values,
        }
    }

    /// Serializzazione a basso costo computazionale per il bus di rete Zenoh
    pub fn to_bytes(&self) -> Vec<u8> {
        // Converte il vettore f32 in un buffer binario pulito e compatto
        let mut buffer = Vec::with_capacity(self.values.len() * 4);
        for val in &self.values {
            buffer.extend_from_slice(&val.to_le_bytes());
        }
        buffer
    }
}
