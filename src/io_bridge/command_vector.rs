/// Rappresentazione matematica standardizzata dei comandi diretti ai motori
#[derive(Debug, Clone)]
pub struct CommandVector {
    pub target_actuators: Vec<f32>, // Voltaggi o coppie per i giunti
}

impl CommandVector {
    pub fn from_bytes(bytes: &[u8]) -> Self {
        // Converte i byte estratti da Zenoh nuovamente in comandi f32 per il robot
        let chunks = bytes.chunks_exact(4);
        let target_actuators = chunks
            .map(|chunk| f32::from_le_bytes(chunk.try_into().unwrap()))
            .collect();

        Self { target_actuators }
    }
}
