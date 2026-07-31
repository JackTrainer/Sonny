use std::sync::Arc;

pub async fn start_bridge(session: Arc<zenoh::Session>) {
    println!("[ROS2 BRIDGE] Modulo di conversione topic ROS2 avviato.");

    let subscriber = match session.declare_subscriber("rt/joint_states").await {
        Ok(sub) => sub,
        Err(_) => return,
    };

    while let Ok(sample) = subscriber.recv_async().await {
        let ros2_payload = sample.payload().to_bytes().to_vec();
        let converted = fieldbus_parse_modbus_frame(&ros2_payload);
        let _ = session.put("alpha/local/sensors", converted).await;
    }
}

fn fieldbus_parse_modbus_frame(raw: &[u8]) -> Vec<u8> {
    if raw.len() < 4 {
        return vec![0u8; 16];
    }

    let mut out = Vec::with_capacity(raw.len());

    match raw[0] {
        0x03 | 0x04 => {
            let byte_count = raw[1] as usize;
            let payload = if raw.len() >= 2 + byte_count + 2 {
                &raw[2..2 + byte_count]
            } else {
                return vec![0u8; 16];
            };

            for chunk in payload.chunks(4) {
                let mut buf = [0u8; 4];
                let len = chunk.len().min(4);
                buf[..len].copy_from_slice(&chunk[..len]);
                out.extend_from_slice(&f32::from_le_bytes(buf).to_le_bytes());
            }
        }
        0x10 => {
            let _register_count = u16::from_be_bytes([raw[4], raw[5]]) as usize;
            let byte_count = raw[6] as usize;
            let payload = if raw.len() >= 7 + byte_count {
                &raw[7..7 + byte_count]
            } else {
                return vec![0u8; 16];
            };

            for chunk in payload.chunks(2) {
                let mut buf = [0u8; 2];
                buf.copy_from_slice(chunk);
                let reg_val = u16::from_be_bytes(buf);
                let f = reg_val as f32;
                out.extend_from_slice(&f.to_le_bytes());
            }
        }
        _ => {
            let len = raw.len().min(16);
            let padded = if len < 16 {
                let mut v = raw[..len].to_vec();
                v.resize(16, 0);
                v
            } else {
                raw[..16].to_vec()
            };
            return padded;
        }
    }

    out
}
