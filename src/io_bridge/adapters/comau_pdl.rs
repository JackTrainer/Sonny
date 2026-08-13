use std::io::Write;

use crate::io_bridge::adapters::{BrandAdapter, FrameStrategy};
use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};
use crate::io_bridge::robot_brand::RobotBrand;

/// Comau codec — **PDL2** socket bridge.
///
/// Comau does not expose a public binary protocol: the C5G/C6G controllers
/// talk over custom sockets using PDL2 programs. This codec uses a
/// deterministic textual protocol (comma-separated float values, frames
/// terminated by newline) that a PDL2 program can read/write with the
/// `SOCKETRECV`/`SOCKETSEND` primitives:
/// ```text
/// 0.1234,-1.5708,3.1416,0.0,0.0,0.0
/// ```
/// Incoming telemetry and outgoing commands share the same format; the
/// direction (robot→SONNY for state, SONNY→robot for commands) is defined
/// by the supervising PDL2 program.
pub struct ComauPdl2;

const COMAU_JOINTS: usize = 6;

impl BrandAdapter for ComauPdl2 {
    fn brand(&self) -> RobotBrand {
        RobotBrand::Comau
    }

    fn protocol_label(&self) -> &'static str {
        "PDL2 Socket (CSV/TCP)"
    }

    fn frame_strategy(&self) -> FrameStrategy {
        FrameStrategy::Delimited { terminator: b"\n" }
    }

    fn decode_telemetry(
        &self,
        acc: &[u8],
        joint_count: usize,
    ) -> Option<(usize, [f32; MAX_JOINTS], usize)> {
        let term = b"\n";
        let end = acc.windows(1).position(|w| w[0] == term[0])?;
        let frame_end = end + 1;

        let mut values = [0.0f32; MAX_JOINTS];
        let mut n = 0usize;
        let limit = joint_count.min(MAX_JOINTS);

        let line = &acc[..end];
        for (i, field) in line
            .split(|b| *b == b',' || b.is_ascii_whitespace())
            .enumerate()
        {
            if i >= limit || field.is_empty() {
                continue;
            }
            let text = std::str::from_utf8(field).ok()?;
            let value: f32 = text.trim().parse().ok()?;
            values[i] = value;
            n = i + 1;
        }

        Some((frame_end, values, n))
    }

    fn encode_command(&self, command: &CommandVector, out: &mut [u8]) -> usize {
        let mut sink = SliceWriter { buf: out, pos: 0 };
        let n = command.as_slice().len().min(COMAU_JOINTS);
        for (i, v) in command.as_slice()[..n].iter().enumerate() {
            if i > 0 {
                let _ = sink.write_all(b",");
            }
            let _ = write!(sink, "{:.6}", v);
        }
        let _ = sink.write_all(b"\n");
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_csv_line() {
        let frame = b"0.1234,-1.5708,3.1416\n";
        let adapter = ComauPdl2;
        let (consumed, values, n) = adapter.decode_telemetry(frame, 6).unwrap();
        assert_eq!(consumed, frame.len());
        assert_eq!(n, 3);
        assert!((values[0] - 0.1234).abs() < 1e-5);
        assert!((values[1] + 1.5708).abs() < 1e-5);
        assert!((values[2] - 3.1416).abs() < 1e-5);
    }

    #[test]
    fn waits_for_newline() {
        let adapter = ComauPdl2;
        assert!(adapter.decode_telemetry(b"0.5,1.0", 6).is_none());
    }

    #[test]
    fn encodes_csv_command() {
        let adapter = ComauPdl2;
        let cmd = CommandVector::from_slice(&[0.5, -1.0]);
        let mut out = [0u8; 256];
        let n = adapter.encode_command(&cmd, &mut out);
        let msg = std::str::from_utf8(&out[..n]).unwrap();
        assert_eq!(msg, "0.500000,-1.000000\n");
    }
}
