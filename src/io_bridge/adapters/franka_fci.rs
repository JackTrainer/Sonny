use crate::io_bridge::adapters::{BrandAdapter, FrameStrategy};
use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};
use crate::io_bridge::robot_brand::RobotBrand;

/// Codec Franka Emika — **FCI** (Franka Control Interface), UDP 30401.
///
/// La telemetria è il `RobotState` FCI (documentazione libfranka):
/// ```text
/// uint32  message_size      (big-endian)
/// double  time              (offset body 0)
/// double  q[7]              (offset body 8)   <-- posizioni reali giunti
/// double  dq[7]             (offset body 64)
/// double  tau_J[7]          (offset body 120)
/// ...
/// ```
/// I comandi sono il `RobotCommand` (192 byte, modello ad addon da 32 byte):
/// ```text
/// uint32 time_ms (LE)      [0..4)
/// uint16 reserved          [4..6)
/// uint32 frame_type = 1    [6..10)  (motion generator: JointPose)
/// uint32 mode      = 0     [10..14) (kJointPosition)
/// uint32 addon     = 0     [14..18)
/// int32  controller = 0    [18..22) (controller interno → impedenza giunti)
/// float32 q[7]             [22..50) (posizioni target giunti, LE)
/// padding a zero           [50..192)
/// ```
/// NB: gli offset del comando vanno validati contro il manuale FCI durante
/// il primo bring-up su Panda reale; il codec è il punto unico di modifica.
pub struct FrankaFci;

const FCI_JOINTS: usize = 7;
const FCI_Q_BODY_OFFSET: usize = 8;
const FCI_CMD_TOTAL: usize = 192;

impl BrandAdapter for FrankaFci {
    fn brand(&self) -> RobotBrand {
        RobotBrand::FrankaEmika
    }

    fn protocol_label(&self) -> &'static str {
        "FCI (UDP 30401)"
    }

    fn frame_strategy(&self) -> FrameStrategy {
        FrameStrategy::LengthPrefixed { header_bytes: 4 }
    }

    fn buffer_hint(&self) -> usize {
        2048
    }

    fn decode_telemetry(
        &self,
        acc: &[u8],
        joint_count: usize,
    ) -> Option<(usize, [f32; MAX_JOINTS], usize)> {
        if acc.len() < 4 {
            return None;
        }
        let size = u32::from_be_bytes([acc[0], acc[1], acc[2], acc[3]]) as usize;
        if size < 4 || acc.len() < size {
            return None;
        }

        let body = &acc[4..size];
        let n = joint_count.min(FCI_JOINTS).min(MAX_JOINTS);
        let mut values = [0.0f32; MAX_JOINTS];

        if body.len() < FCI_Q_BODY_OFFSET + n * 8 {
            return None;
        }

        for i in 0..n {
            let off = FCI_Q_BODY_OFFSET + i * 8;
            values[i] = read_f64_be(&body[off..off + 8]) as f32;
        }

        Some((size, values, n))
    }

    fn encode_command(&self, command: &CommandVector, out: &mut [u8]) -> usize {
        let mut frame = [0u8; FCI_CMD_TOTAL];

        frame[0..4].copy_from_slice(&0u32.to_le_bytes());
        frame[4..6].copy_from_slice(&0u16.to_le_bytes());
        frame[6..10].copy_from_slice(&1u32.to_le_bytes());
        frame[10..14].copy_from_slice(&0u32.to_le_bytes());
        frame[14..18].copy_from_slice(&0u32.to_le_bytes());
        frame[18..22].copy_from_slice(&0i32.to_le_bytes());

        let n = command.as_slice().len().min(FCI_JOINTS);
        for (i, v) in command.as_slice()[..n].iter().enumerate() {
            let off = 22 + i * 4;
            frame[off..off + 4].copy_from_slice(&v.to_le_bytes());
        }

        let n = FCI_CMD_TOTAL.min(out.len());
        out[..n].copy_from_slice(&frame[..n]);
        n
    }
}

fn read_f64_be(bytes: &[u8]) -> f64 {
    let mut buf = [0u8; 8];
    buf.copy_from_slice(bytes);
    f64::from_be_bytes(buf)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn build_fci_state(q: [f64; 7], dq: [f64; 7]) -> Vec<u8> {
        let mut body = vec![0u8; 192];
        for (i, v) in q.iter().enumerate() {
            let off = FCI_Q_BODY_OFFSET + i * 8;
            body[off..off + 8].copy_from_slice(&v.to_be_bytes());
        }
        for (i, v) in dq.iter().enumerate() {
            let off = FCI_Q_BODY_OFFSET + 7 * 8 + i * 8;
            body[off..off + 8].copy_from_slice(&v.to_be_bytes());
        }
        let size = (body.len() + 4) as u32;
        let mut packet = Vec::with_capacity(body.len() + 4);
        packet.extend_from_slice(&size.to_be_bytes());
        packet.extend_from_slice(&body);
        packet
    }

    #[test]
    fn decodes_fci_robot_state() {
        let q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7];
        let dq = [0.0; 7];
        let packet = build_fci_state(q, dq);
        let adapter = FrankaFci;
        let (consumed, values, n) = adapter.decode_telemetry(&packet, 7).unwrap();
        assert_eq!(consumed, packet.len());
        assert_eq!(n, 7);
        for i in 0..7 {
            assert!((values[i] - q[i] as f32).abs() < 1e-4);
        }
    }

    #[test]
    fn waits_for_full_state_packet() {
        let packet = build_fci_state([0.0; 7], [0.0; 7]);
        let adapter = FrankaFci;
        assert!(adapter.decode_telemetry(&packet[..4], 7).is_none());
    }

    #[test]
    fn encodes_fixed_192_byte_command() {
        let adapter = FrankaFci;
        let cmd = CommandVector::from_slice(&[0.5, -0.5, 1.0, -1.0, 2.0, -2.0, 0.0]);
        let mut out = [0u8; 512];
        let n = adapter.encode_command(&cmd, &mut out);
        assert_eq!(n, 192);
        let q0 = f32::from_le_bytes([out[22], out[23], out[24], out[25]]);
        assert!((q0 - 0.5).abs() < 1e-6);
    }
}
