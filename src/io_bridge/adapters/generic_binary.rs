use crate::io_bridge::adapters::{BrandAdapter, FrameStrategy};
use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};
use crate::io_bridge::robot_brand::RobotBrand;

/// Generic binary codec: f32 little-endian frames, 4 bytes per joint.
/// It is the native format of SONNY vectors (`StateVector`/`CommandVector`):
/// a single parser on the microcontroller side for telemetry and commands.
pub struct GenericBinary;

impl BrandAdapter for GenericBinary {
    fn brand(&self) -> RobotBrand {
        RobotBrand::Generic
    }

    fn protocol_label(&self) -> &'static str {
        "Fixed f32 LE"
    }

    fn frame_strategy(&self) -> FrameStrategy {
        FrameStrategy::Fixed { bytes_per_joint: 4 }
    }

    fn decode_telemetry(
        &self,
        acc: &[u8],
        joint_count: usize,
    ) -> Option<(usize, [f32; MAX_JOINTS], usize)> {
        let frame_size = joint_count.saturating_mul(4);
        if frame_size == 0 || acc.len() < frame_size {
            return None;
        }
        let n = joint_count.min(MAX_JOINTS);
        let mut values = [0.0f32; MAX_JOINTS];
        for i in 0..n {
            let off = i * 4;
            values[i] = f32::from_le_bytes([acc[off], acc[off + 1], acc[off + 2], acc[off + 3]]);
        }
        Some((frame_size, values, n))
    }

    fn encode_command(&self, command: &CommandVector, out: &mut [u8]) -> usize {
        command.write_to(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_fixed_f32_le_frame() {
        let mut frame = Vec::new();
        for v in [1.0f32, -2.5, 3.25] {
            frame.extend_from_slice(&v.to_le_bytes());
        }
        let adapter = GenericBinary;
        let (consumed, values, n) = adapter.decode_telemetry(&frame, 3).unwrap();
        assert_eq!(consumed, 12);
        assert_eq!(n, 3);
        assert_eq!(&values[..n], &[1.0, -2.5, 3.25][..]);
    }

    #[test]
    fn waits_for_more_bytes_when_incomplete() {
        let adapter = GenericBinary;
        let partial = 1.0f32.to_le_bytes();
        assert!(adapter.decode_telemetry(&partial, 3).is_none());
    }

    #[test]
    fn command_roundtrip() {
        let adapter = GenericBinary;
        let cmd = CommandVector::from_slice(&[0.1, -0.2, 0.3]);
        let mut out = [0u8; 256];
        let n = adapter.encode_command(&cmd, &mut out);
        let (_, values, count) = adapter.decode_telemetry(&out[..n], 3).unwrap();
        assert_eq!(&values[..count], cmd.as_slice());
    }
}
