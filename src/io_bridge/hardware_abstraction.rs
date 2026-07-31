use std::sync::Arc;
use std::time::Duration;
use tokio::io::AsyncReadExt;
use tokio::net::UdpSocket;
use tokio::time::sleep;
use tokio_serial::{SerialPortBuilderExt, SerialStream};
use crate::io_bridge::state_vector::StateVector;

#[derive(Debug, Clone)]
pub enum FieldbusTransport {
    Serial { port: String, baud_rate: u32 },
    Udp { bind_addr: String, remote_addr: String },
}

#[derive(Debug, Clone)]
pub struct FieldbusConfig {
    pub transport: FieldbusTransport,
    pub hardware_id: String,
    pub joint_count: usize,
    pub frame_timeout_ms: u64,
    pub reconnect_delay_ms: u64,
}

/// Punto di contatto elettrico: byte grezzi dal fieldbus industriale → StateVector
pub struct HardwareAbstraction {
    session: Arc<zenoh::Session>,
    config: FieldbusConfig,
}

impl HardwareAbstraction {
    pub fn new(session: Arc<zenoh::Session>, config: FieldbusConfig) -> Self {
        Self { session, config }
    }

    pub async fn start_loop(&self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        match &self.config.transport {
            FieldbusTransport::Serial { port, baud_rate } => {
                self.run_serial_loop(port, *baud_rate).await
            }
            FieldbusTransport::Udp { bind_addr, remote_addr } => {
                self.run_udp_loop(bind_addr, remote_addr).await
            }
        }
    }

    async fn run_serial_loop(
        &self,
        port: &str,
        baud_rate: u32,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let frame_size = self.config.joint_count * 4;
        let mut acc = Vec::new();

        loop {
            let mut serial = match Self::open_serial(port, baud_rate).await {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("[HARDWARE] Errore apertura {}: {}", port, e);
                    sleep(Duration::from_millis(self.config.reconnect_delay_ms)).await;
                    continue;
                }
            };

            println!("[HARDWARE] Seriale {} @ {} baud, frame={}B", port, baud_rate, frame_size);

            let mut buf = [0u8; 1024];
            loop {
                match serial.read(&mut buf).await {
                    Ok(0) => break,
                    Ok(n) => {
                        acc.extend_from_slice(&buf[..n]);
                        self.drain_frames(&mut acc, frame_size).await?;
                    }
                    Err(e) => {
                        eprintln!("[HARDWARE] Lettura seriale: {}", e);
                        break;
                    }
                }
            }

            sleep(Duration::from_millis(self.config.reconnect_delay_ms)).await;
        }
    }

    async fn run_udp_loop(
        &self,
        bind_addr: &str,
        remote_addr: &str,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let frame_size = self.config.joint_count * 4;
        let socket = UdpSocket::bind(bind_addr).await?;
        socket.connect(remote_addr).await?;

        println!("[HARDWARE] UDP {} → {}, frame={}B", bind_addr, remote_addr, frame_size);

        let mut buf = [0u8; 65535];
        loop {
            let n = socket.recv(&mut buf).await?;
            let mut acc = buf[..n].to_vec();
            self.drain_frames(&mut acc, frame_size).await?;
        }
    }

    async fn drain_frames(
        &self,
        acc: &mut Vec<u8>,
        frame_size: usize,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        while acc.len() >= frame_size {
            let frame: Vec<u8> = acc.drain(..frame_size).collect();
            let values = Self::parse_frame(&frame, self.config.joint_count);
            let state = StateVector::new(&self.config.hardware_id, values);
            let topic = format!("alpha/telemetry/{}", self.config.hardware_id);
            self.session.put(&topic, state.to_bytes()).await?;
        }
        Ok(())
    }

    fn parse_frame(frame: &[u8], joint_count: usize) -> Vec<f32> {
        frame
            .chunks_exact(4)
            .take(joint_count)
            .map(|c| f32::from_le_bytes(c.try_into().unwrap()))
            .collect()
    }

    async fn open_serial(port: &str, baud_rate: u32) -> Result<SerialStream, tokio_serial::Error> {
        tokio_serial::new(port, baud_rate)
            .data_bits(tokio_serial::DataBits::Eight)
            .flow_control(tokio_serial::FlowControl::None)
            .parity(tokio_serial::Parity::None)
            .stop_bits(tokio_serial::StopBits::One)
            .open_native_async()
    }
}
