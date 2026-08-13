/// Fixed maximum size of the state and command vectors.
///
/// 32 joints comfortably cover any robot (arms, AMRs, drones,
/// humanoids) and ensure that `StateVector`/`CommandVector` are static
/// pre-allocated arrays: no `malloc` during the control loop at
/// 100 Hz, hence zero heap fragmentation.
pub const MAX_JOINTS: usize = 32;

/// Standardized mathematical representation of the commands sent to the motors.
///
/// Fixed-size **vector** `[f32; 32]` + active length `len`: the RAM
/// occupied is identical from the first second to 10 years of continuous use.
#[derive(Debug, Clone, Copy)]
pub struct CommandVector {
    pub target_actuators: [f32; MAX_JOINTS], // Voltages or torques for the joints
    pub len: usize,
}

impl CommandVector {
    /// Empty vector, all zeros. No allocation.
    pub const fn new() -> Self {
        Self {
            target_actuators: [0.0; MAX_JOINTS],
            len: 0,
        }
    }

    /// Zero vector of length `joint_count`: used by estop and Hard-Watchdog.
    pub fn zeros(joint_count: usize) -> Self {
        Self {
            target_actuators: [0.0; MAX_JOINTS],
            len: joint_count.min(MAX_JOINTS),
        }
    }

    /// Copies `values` into the static array (truncated to `MAX_JOINTS`).
    pub fn from_slice(values: &[f32]) -> Self {
        let mut v = Self::new();
        let n = values.len().min(MAX_JOINTS);
        v.target_actuators[..n].copy_from_slice(&values[..n]);
        v.len = n;
        v
    }

    /// Converts the bytes retrieved from Zenoh back into f32 commands for the robot.
    pub fn from_bytes(bytes: &[u8]) -> Self {
        let mut v = Self::new();
        let mut n = 0;
        for chunk in bytes.chunks_exact(4) {
            if n >= MAX_JOINTS {
                break;
            }
            v.target_actuators[n] = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            n += 1;
        }
        v.len = n;
        v
    }

    pub fn as_slice(&self) -> &[f32] {
        &self.target_actuators[..self.len]
    }

    pub fn as_mut_slice(&mut self) -> &mut [f32] {
        &mut self.target_actuators[..self.len]
    }

    /// Compact binary serialization **without allocation**: writes the f32 LE
    /// values into `out` and returns the bytes written. Same format used by
    /// `StateVector`: a single parser on the microcontroller side for telemetry
    /// and commands. `out` must have capacity `MAX_JOINTS * 4` (128 bytes).
    pub fn write_to(&self, out: &mut [u8]) -> usize {
        let n = self.len.min(out.len() / 4);
        let mut written = 0;
        for i in 0..n {
            let bytes = self.target_actuators[i].to_le_bytes();
            out[written..written + 4].copy_from_slice(&bytes);
            written += 4;
        }
        written
    }
}

impl Default for CommandVector {
    fn default() -> Self {
        Self::new()
    }
}
