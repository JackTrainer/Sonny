use crate::io_bridge::adapters::{BrandAdapter, FrameStrategy};
use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};
use crate::io_bridge::robot_brand::RobotBrand;

/// Codec per **piattaforme AMR standard**.
///
/// Le AMR standard integrano ROS 2 (`/cmd_vel`, `/odom`): la compatibilità
/// passa dal `ros2_bridge` che converte i topic in vettori SONNY. Sulla rete
/// SONNY il vettore di giunti della base mobile è **velocità ruote in rad/s**
/// (differenziale: `[v_left, v_right, ...]`), trasportato con il frame binario
/// f32 LE nativo (stesso formato del profilo generico).
///
/// Le funzioni [`twist_to_wheel_speeds`] e [`wheel_speeds_to_twist`]
/// implementano il mapping differenziale standard `Twist ⇄ ruote` usato dal
/// bridge ROS2, così una AMR si pilota in termini di `linear.x`/`angular.z`
/// mentre sul bus SONNY viaggiano le velocità delle ruote.
#[derive(Default)]
pub struct AmrRos2;

/// Interasse predefinito [m] per il mapping differenziale.
pub const DEFAULT_WHEEL_BASE_M: f32 = 0.5;

impl BrandAdapter for AmrRos2 {
    fn brand(&self) -> RobotBrand {
        RobotBrand::Amr
    }

    fn protocol_label(&self) -> &'static str {
        "ROS2 Twist/Odom (f32 LE)"
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

/// Mapping `Twist (v, ω) → velocità ruote (rad/s)` per base differenziale.
///
/// `v` velocità lineare lungo x, `ω` velocità angolare attorno a z,
/// `wheel_base` interasse in metri.
pub fn twist_to_wheel_speeds(linear: f32, angular: f32, wheel_base: f32) -> (f32, f32) {
    if wheel_base <= 0.0 {
        return (linear, linear);
    }
    let half = angular * wheel_base / 2.0;
    (linear - half, linear + half)
}

/// Mapping inverso `velocità ruote → Twist (v, ω)`.
pub fn wheel_speeds_to_twist(left: f32, right: f32, wheel_base: f32) -> (f32, f32) {
    let v = (left + right) / 2.0;
    let omega = if wheel_base <= 0.0 {
        0.0
    } else {
        (right - left) / wheel_base
    };
    (v, omega)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn twist_to_wheels_and_back() {
        let (l, r) = twist_to_wheel_speeds(0.5, 0.2, 0.5);
        let (v, w) = wheel_speeds_to_twist(l, r, 0.5);
        assert!((v - 0.5).abs() < 1e-5);
        assert!((w - 0.2).abs() < 1e-5);
    }

    #[test]
    fn straight_line_moves_both_wheels_equal() {
        let (l, r) = twist_to_wheel_speeds(0.8, 0.0, 0.5);
        assert!((l - r).abs() < 1e-6);
        assert!((l - 0.8).abs() < 1e-6);
    }

    #[test]
    fn rotation_in_place() {
        let (l, r) = twist_to_wheel_speeds(0.0, 1.0, 0.5);
        assert!((l + 0.25).abs() < 1e-6);
        assert!((r - 0.25).abs() < 1e-6);
    }

    #[test]
    fn decodes_fixed_wheel_vector() {
        let mut frame = Vec::new();
        for v in [0.5f32, -0.5] {
            frame.extend_from_slice(&v.to_le_bytes());
        }
        let adapter = AmrRos2;
        let (consumed, values, n) = adapter.decode_telemetry(&frame, 2).unwrap();
        assert_eq!(consumed, 8);
        assert_eq!(&values[..n], &[0.5, -0.5][..]);
    }
}
