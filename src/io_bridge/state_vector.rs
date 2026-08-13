use std::time::{SystemTime, UNIX_EPOCH};

use crate::io_bridge::command_vector::MAX_JOINTS;

/// Standardized mathematical representation of the incoming telemetry.
///
/// Fixed-size **vector** `[f32; 32]`: the `StateVector` is
/// pre-allocated only once at robot startup and updated *in-place*
/// on every telemetry frame with [`StateVector::set_values`], with no heap
/// allocation during the 100 Hz loop.
#[derive(Debug, Clone)]
pub struct StateVector {
    pub hardware_id: String,
    pub timestamp: u64,
    pub values: [f32; MAX_JOINTS], // Linearized sensors, joints, and contacts
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

    /// Updates the vector *in-place* (no allocation): copies the values
    /// back into the static array and stamps a new telemetry timestamp.
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

    /// Low-cost binary serialization **without allocation** (f32 LE),
    /// same format as `CommandVector`: a single parser on the
    /// microcontroller side is enough. Returns the bytes written into `out`.
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
