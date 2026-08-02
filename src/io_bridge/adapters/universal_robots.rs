use std::io::Write;

use crate::io_bridge::adapters::{BrandAdapter, FrameStrategy};
use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};
use crate::io_bridge::robot_brand::RobotBrand;

/// Codec Universal Robots: **Real-Time Client** (porta 30003, binario)
/// per la telemetria + **URScript** (porta 30002) per i comandi.
///
/// Formato Real-Time Client (documentazione UR, trasmesso a 125 Hz):
/// ```text
/// int32   message_size      (big-endian)
/// double  time
/// double  q_target[6]       (offset body 8)
/// double  qd_target[6]      (offset body 56)
/// double  qdd_target[6]     (offset body 104)
/// double  I_target[6]       (offset body 152)
/// double  M_target[6]       (offset body 200)
/// double  q_actual[6]       (offset body 248)  <-- posizioni reali dei giunti
/// double  qd_actual[6]      (offset body 296)
/// ...
/// ```
/// Comandi in URScript: `servoj([q1,..,q6], 0, 0, 0.008, 0.05, 100)` — servo
/// in tempo reale con i giunti target del vettore di comando.
pub struct UniversalRobots;

const UR_JOINTS: usize = 6;
const UR_Q_ACTUAL_BODY_OFFSET: usize = 248;

impl BrandAdapter for UniversalRobots {
    fn brand(&self) -> RobotBrand {
        RobotBrand::UniversalRobots
    }

    fn protocol_label(&self) -> &'static str {
        "UR RT-Client + URScript"
    }

    fn frame_strategy(&self) -> FrameStrategy {
        FrameStrategy::LengthPrefixed { header_bytes: 4 }
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
        let n = joint_count.min(UR_JOINTS).min(MAX_JOINTS);
        let mut values = [0.0f32; MAX_JOINTS];

        if body.len() < UR_Q_ACTUAL_BODY_OFFSET + n * 8 {
            return None;
        }

        for i in 0..n {
            let off = UR_Q_ACTUAL_BODY_OFFSET + i * 8;
            values[i] = read_f64_be(&body[off..off + 8]) as f32;
        }

        Some((size, values, n))
    }

    fn encode_command(&self, command: &CommandVector, out: &mut [u8]) -> usize {
        let mut sink = SliceWriter { buf: out, pos: 0 };

        let _ = write!(sink, "servoj([");
        let n = command.as_slice().len().min(UR_JOINTS);
        for (i, v) in command.as_slice()[..n].iter().enumerate() {
            if i > 0 {
                let _ = sink.write_all(b", ");
            }
            let _ = write!(sink, "{:.6}", v);
        }
        let _ = write!(sink, "], 0, 0, 0.008, 0.05, 100)\n");

        sink.pos
    }
}

struct SliceWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl Write for SliceWriter<'_> {
    fn write(&mut self, data: &[u8]) -> std::io::Result<usize> {
        let space = self.buf.len().saturating_sub(self.pos);
        let n = data.len().min(space);
        self.buf[self.pos..self.pos + n].copy_from_slice(&data[..n]);
        self.pos += n;
        Ok(n)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
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

    fn build_ur_packet(q_actual: [f64; 6]) -> Vec<u8> {
        let mut body = vec![0u8; 320];
        for (i, v) in q_actual.iter().enumerate() {
            let off = UR_Q_ACTUAL_BODY_OFFSET + i * 8;
            body[off..off + 8].copy_from_slice(&v.to_be_bytes());
        }
        let size = (body.len() + 4) as u32;
        let mut packet = Vec::with_capacity(body.len() + 4);
        packet.extend_from_slice(&size.to_be_bytes());
        packet.extend_from_slice(&body);
        packet
    }

    #[test]
    fn decodes_q_actual_from_rt_client_packet() {
        let q_actual = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6];
        let packet = build_ur_packet(q_actual);
        let adapter = UniversalRobots;
        let (consumed, values, n) = adapter.decode_telemetry(&packet, 6).unwrap();
        assert_eq!(consumed, packet.len());
        assert_eq!(n, 6);
        for i in 0..6 {
            assert!((values[i] - q_actual[i] as f32).abs() < 1e-4);
        }
    }

    #[test]
    fn waits_for_full_message() {
        let packet = build_ur_packet([0.0; 6]);
        let adapter = UniversalRobots;
        assert!(adapter
            .decode_telemetry(&packet[..packet.len() - 4], 6)
            .is_none());
    }

    #[test]
    fn encodes_urscript_servoj() {
        let adapter = UniversalRobots;
        let cmd = CommandVector::from_slice(&[0.5, -1.5, 0.0, 1.0, -0.5, 0.25]);
        let mut out = [0u8; 256];
        let n = adapter.encode_command(&cmd, &mut out);
        let msg = std::str::from_utf8(&out[..n]).unwrap();
        assert!(msg.starts_with("servoj(["));
        assert!(msg.contains("0.500000, -1.500000"));
        assert!(msg.ends_with("], 0, 0, 0.008, 0.05, 100)\n"));
    }
}
