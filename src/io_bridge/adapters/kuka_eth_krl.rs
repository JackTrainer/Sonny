use std::io::Write;

use crate::io_bridge::adapters::{BrandAdapter, FrameStrategy};
use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};
use crate::io_bridge::robot_brand::RobotBrand;

/// KUKA codec over **Ethernet KRL** (EKI): XML on TCP, port 7911 of the KRC.
///
/// Telemetry (KUKA → SONNY): the KRL program arrays are exposed as `SEND[n]`
/// variables:
/// ```xml
/// <?xml version="1.0" encoding="UTF-8"?><Rob><Data>
///   <Var Name="SEND[1]">-0.1234</Var><Var Name="SEND[2]">1.5708</Var>
/// </Data></Rob>
/// ```
/// Commands (SONNY → KUKA): `A[n]` variables read on the KRL side:
/// ```xml
/// <?xml version="1.0" encoding="UTF-8"?><Rob><Data>
///   <Var Name="CMD">SONNY</Var><Var Name="A1">0.0000</Var>
/// </Data></Rob>
/// ```
/// The KRL program must declare the `SEND[]` and `A[]` arrays in the EKI config.
pub struct KukaEthernetKrl;

const XML_HEADER: &[u8] = b"<?xml version=\"1.0\" encoding=\"UTF-8\"?><Rob><Data>";
const XML_FOOTER: &[u8] = b"</Data></Rob>";

impl BrandAdapter for KukaEthernetKrl {
    fn brand(&self) -> RobotBrand {
        RobotBrand::Kuka
    }

    fn protocol_label(&self) -> &'static str {
        "Ethernet KRL (XML/TCP)"
    }

    fn frame_strategy(&self) -> FrameStrategy {
        FrameStrategy::Delimited {
            terminator: b"</Rob>",
        }
    }

    fn decode_telemetry(
        &self,
        acc: &[u8],
        joint_count: usize,
    ) -> Option<(usize, [f32; MAX_JOINTS], usize)> {
        let term = b"</Rob>";
        let end = find_subslice(acc, term)?;
        let frame_end = end + term.len();
        let body = &acc[..end];

        let mut values = [0.0f32; MAX_JOINTS];
        let mut n = 0usize;
        let mut search_from = 0usize;

        while let Some(var) = parse_var(body, search_from) {
            if n >= joint_count.min(MAX_JOINTS) {
                break;
            }
            if var.name.starts_with(b"SEND") {
                let index = parse_bracket_index(var.name);
                let idx = match index {
                    Some(i) if i >= 1 && i <= MAX_JOINTS => i - 1,
                    _ => n,
                };
                values[idx] = var.value;
                n = n.max(idx + 1);
            }
            search_from = var.next_offset;
        }

        Some((frame_end, values, n))
    }

    fn encode_command(&self, command: &CommandVector, out: &mut [u8]) -> usize {
        let mut sink = SliceWriter { buf: out, pos: 0 };

        let _ = sink.write_all(XML_HEADER);
        let _ = write!(sink, "<Var Name=\"CMD\">SONNY</Var>");
        for (i, v) in command.as_slice().iter().enumerate() {
            let _ = write!(sink, "<Var Name=\"A{}\">{:.6}</Var>", i + 1, v);
        }
        let _ = sink.write_all(XML_FOOTER);

        sink.pos
    }
}

/// Write buffer without heap allocation.
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

struct ParsedVar<'a> {
    name: &'a [u8],
    value: f32,
    next_offset: usize,
}

/// Extracts a `<Var Name="...">value</Var>` element starting at `from`.
fn parse_var(body: &[u8], from: usize) -> Option<ParsedVar<'_>> {
    let name_tag = b"<Var Name=\"";
    let start = find_from(body, name_tag, from)?;
    let name_begin = start + name_tag.len();
    let name_end = find_from(body, b"\"", name_begin)?;
    let close = find_from(body, b">", name_end + 1)?;
    let value_end = find_from(body, b"</Var>", close + 1)?;

    let name = &body[name_begin..name_end];
    let value = std::str::from_utf8(&body[close + 1..value_end]).ok()?;
    let value: f32 = value.trim().parse().ok()?;

    Some(ParsedVar {
        name,
        value,
        next_offset: value_end + 6,
    })
}

fn parse_bracket_index(name: &[u8]) -> Option<usize> {
    let open = name.iter().position(|&b| b == b'[')?;
    let close = name.iter().position(|&b| b == b']')?;
    if close <= open {
        return None;
    }
    std::str::from_utf8(&name[open + 1..close])
        .ok()?
        .trim()
        .parse()
        .ok()
}

fn find_subslice(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    haystack.windows(needle.len()).position(|w| w == needle)
}

fn find_from(haystack: &[u8], needle: &[u8], from: usize) -> Option<usize> {
    if from > haystack.len() {
        return None;
    }
    find_subslice(&haystack[from..], needle).map(|i| i + from)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_kuka_send_frame() {
        let frame = br#"<?xml version="1.0" encoding="UTF-8"?><Rob><Data>
            <Var Name="SEND[1]">-0.1234</Var><Var Name="SEND[2]">1.5708</Var>
            <Var Name="SEND[3]">3.1416</Var></Data></Rob>"#;
        let adapter = KukaEthernetKrl;
        let (consumed, values, n) = adapter.decode_telemetry(frame, 6).unwrap();
        assert_eq!(consumed, frame.len());
        assert_eq!(n, 3);
        assert!((values[0] - (-0.1234)).abs() < 1e-5);
        assert!((values[1] - 1.5708).abs() < 1e-5);
        assert!((values[2] - 3.1416).abs() < 1e-5);
    }

    #[test]
    fn waits_for_complete_xml_frame() {
        let partial = br#"<?xml version="1.0"?><Rob><Data><Var Name="SEND[1]">0.5</Var>"#;
        let adapter = KukaEthernetKrl;
        assert!(adapter.decode_telemetry(partial, 6).is_none());
    }

    #[test]
    fn encodes_a_commands_xml() {
        let adapter = KukaEthernetKrl;
        let cmd = CommandVector::from_slice(&[0.25, -1.0]);
        let mut out = [0u8; 512];
        let n = adapter.encode_command(&cmd, &mut out);
        let msg = std::str::from_utf8(&out[..n]).unwrap();
        assert!(msg.contains("<Var Name=\"A1\">0.250000</Var>"));
        assert!(msg.contains("<Var Name=\"A2\">-1.000000</Var>"));
        assert!(msg.ends_with("</Rob>"));
    }
}
