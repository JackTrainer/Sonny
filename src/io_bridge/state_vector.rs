use std::time::{SystemTime, UNIX_EPOCH};

use crate::io_bridge::command_vector::MAX_JOINTS;

/// Rappresentazione matematica standardizzata della telemetria in ingresso.
///
/// Vettore a dimensione **fissa** `[f32; 32]`: il `StateVector` viene
/// pre-allocato una sola volta all'avvio del robot e aggiornato *in-place*
/// a ogni frame telemetrico con [`StateVector::set_values`], senza alcuna
/// allocazione sull'heap durante il loop a 100 Hz.
#[derive(Debug, Clone)]
pub struct StateVector {
    pub hardware_id: String,
    pub timestamp: u64,
    pub values: [f32; MAX_JOINTS], // Sensori, giunti e contatti linearizzati
    pub len: usize,
}

impl StateVector {
    pub fn new(hardware_id: &str, values: &[f32]) -> Self {
        let mut v = Self {
            hardware_id: hardware_id.to_string(),
            timestamp: now_ms(),
            values: [0.0; MAX_JOINTS],
            len: 0,
        };
        v.set_values(values);
        v
    }

    /// Aggiorna il vettore *in-place* (nessuna allocazione): ricopia i valori
    /// nell'array statico e timbra un nuovo timestamp di telemetria.
    pub fn set_values(&mut self, values: &[f32]) {
        let n = values.len().min(MAX_JOINTS);
        self.values[..n].copy_from_slice(&values[..n]);
        self.len = n;
        self.timestamp = now_ms();
    }

    pub fn as_slice(&self) -> &[f32] {
        &self.values[..self.len]
    }

    pub fn as_mut_slice(&mut self) -> &mut [f32] {
        &mut self.values[..self.len]
    }

    /// Serializzazione binaria a basso costo **senza allocazione** (f32 LE),
    /// stesso formato del `CommandVector`: basta un unico parser lato
    /// microcontrollore. Restituisce i byte scritti in `out`.
    pub fn write_to(&self, out: &mut [u8]) -> usize {
        let n = self.len.min(out.len() / 4);
        let mut written = 0;
        for i in 0..n {
            let bytes = self.values[i].to_le_bytes();
            out[written..written + 4].copy_from_slice(&bytes);
            written += 4;
        }
        written
    }
}

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}
