use crate::io_bridge::adapters::{adapter_for, BrandAdapter};
use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};
use crate::io_bridge::jerk_limiter::JerkLimiter;
use crate::io_bridge::robot_brand::RobotBrand;
use crate::io_bridge::state_vector::StateVector;
use crate::io_bridge::vector_sanitizer::VectorSanitizer;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::io::{AsyncReadExt, AsyncWrite, AsyncWriteExt};
use tokio::net::{TcpStream, UdpSocket};
use tokio::time::sleep;
use tokio_serial::{SerialPortBuilderExt, SerialStream};

/// Buffer massimo per la codifica di un comando nel protocollo nativo del
/// marchio (XML KUKA, URScript, JSON, binario f32 LE).
const COMMAND_BUF: usize = 2048;

#[derive(Debug, Clone)]
pub enum FieldbusTransport {
    Serial {
        port: String,
        baud_rate: u32,
    },
    Udp {
        bind_addr: String,
        remote_addr: String,
    },
    Tcp {
        remote_addr: String,
    },
}

#[derive(Debug, Clone)]
pub struct FieldbusConfig {
    pub transport: FieldbusTransport,
    pub hardware_id: String,
    pub joint_count: usize,
    pub frame_timeout_ms: u64,
    pub reconnect_delay_ms: u64,
    /// Limiti fisici [min, max] in rad per ogni giunto: clamp di sicurezza
    /// applicato a OGNI comando prima che raggiunga gli azionamenti.
    pub joint_limits_rad: Vec<(f32, f32)>,
    /// Frequenza nominale del loop di controllo (Hz): periodo del tick usato
    /// dal limitatore di jerk.
    pub expected_hz: f64,
    /// Velocità massima per giunto (rad/s): limita la derivata prima.
    pub max_speed_rad_s: Vec<f32>,
    /// Accelerazione massima per giunto (rad/s²): limita la derivata seconda.
    pub max_accel_rad_s2: Vec<f32>,
    /// Jerk massimo per giunto (rad/s³): limita la derivata terza e smussa la
    /// curva di comando contro gli shock meccanici.
    pub max_jerk_rad_s3: Vec<f32>,
    /// Profilo di compatibilità del marchio: seleziona il codec nativo.
    pub brand: RobotBrand,
}

impl Default for FieldbusConfig {
    fn default() -> Self {
        Self {
            transport: FieldbusTransport::Udp {
                bind_addr: String::new(),
                remote_addr: String::new(),
            },
            hardware_id: String::new(),
            joint_count: 6,
            frame_timeout_ms: 100,
            reconnect_delay_ms: 2000,
            joint_limits_rad: Vec::new(),
            expected_hz: 100.0,
            max_speed_rad_s: Vec::new(),
            max_accel_rad_s2: Vec::new(),
            max_jerk_rad_s3: Vec::new(),
            brand: RobotBrand::Generic,
        }
    }
}

/// Punto di contatto elettrico: byte grezzi dal fieldbus industriale → StateVector
/// e StateVector → comandi attuatori (bidirezionale). Il codec dei byte grezzi
/// dipende dal marchio (`RobotBrand`), selezionato tramite [`BrandAdapter`].
pub struct HardwareAbstraction {
    session: Arc<zenoh::Session>,
    config: FieldbusConfig,
    adapter: Box<dyn BrandAdapter>,
    /// Intercettore di sicurezza: blocca NaN/Infinity e clampa i comandi
    /// prima che raggiungano gli azionamenti. Protetto da Mutex perché il
    /// loop di I/O è `&self`; il lock non viene mai tenuto attraverso un
    /// `.await`.
    sanitizer: Mutex<VectorSanitizer>,
    /// Limitatore di shock meccanico: smussa la curva di comando limitando
    /// velocità, accelerazione e jerk (derivata terza).
    jerk_limiter: Mutex<JerkLimiter>,
}

/// Clamp di sicurezza: nessun comando può superare i limiti fisici del giunto.
/// Opera su array statici pre-allocati: nessuna allocazione durante il loop.
pub fn clamp_to_limits(values: &[f32], limits: &[(f32, f32)]) -> CommandVector {
    let mut command = CommandVector::new();
    let n = values.len().min(MAX_JOINTS);
    for (i, v) in values[..n].iter().enumerate() {
        command.target_actuators[i] = limits
            .get(i)
            .map(|(min, max)| v.clamp(*min, *max))
            .unwrap_or(*v);
    }
    command.len = n;
    command
}

impl HardwareAbstraction {
    pub fn new(session: Arc<zenoh::Session>, config: FieldbusConfig) -> Self {
        let adapter = adapter_for(config.brand);

        let dt = if config.expected_hz > 0.0 {
            1.0 / config.expected_hz as f32
        } else {
            0.01
        };
        let mut jerk_limiter = JerkLimiter::new(dt);
        jerk_limiter.set_limits(
            &config.max_speed_rad_s,
            &config.max_accel_rad_s2,
            &config.max_jerk_rad_s3,
        );

        Self {
            session,
            config,
            adapter,
            sanitizer: Mutex::new(VectorSanitizer::new()),
            jerk_limiter: Mutex::new(jerk_limiter),
        }
    }

    pub fn transport_name(&self) -> &'static str {
        match &self.config.transport {
            FieldbusTransport::Serial { .. } => "SERIAL",
            FieldbusTransport::Udp { .. } => "UDP",
            FieldbusTransport::Tcp { .. } => "TCP",
        }
    }

    pub fn brand_label(&self) -> &'static str {
        self.adapter.brand().label()
    }

    pub async fn start_loop(&self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        println!(
            "[HARDWARE] Marchio: {} ({}), framing: {}",
            self.adapter.brand().label(),
            self.adapter.protocol_label(),
            self.adapter.frame_strategy().label()
        );
        match &self.config.transport {
            FieldbusTransport::Serial { port, baud_rate } => {
                self.run_serial_loop(port, *baud_rate).await
            }
            FieldbusTransport::Udp {
                bind_addr,
                remote_addr,
            } => self.run_udp_loop(bind_addr, remote_addr).await,
            FieldbusTransport::Tcp { remote_addr } => self.run_tcp_loop(remote_addr).await,
        }
    }

    // -----------------------------------------------------------------------
    // Fieldbus SERIALE
    // -----------------------------------------------------------------------
    async fn run_serial_loop(
        &self,
        port: &str,
        baud_rate: u32,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let cmd_sub = self
            .session
            .declare_subscriber(&format!("alpha/cmd/{}", self.config.hardware_id))
            .await?;
        let estop_sub = self
            .session
            .declare_subscriber(&format!("alpha/cmd/{}/estop", self.config.hardware_id))
            .await?;
        let telemetry_topic = format!("alpha/telemetry/{}", self.config.hardware_id);
        let mut estop_active = false;

        // Pre-allocazione all'avvio: StateVector e buffer di serializzazione
        // creati una sola volta e riutilizzati per tutta la vita del processo.
        let mut state = StateVector::new(&self.config.hardware_id, &[]);
        let mut tx_buf = [0u8; MAX_JOINTS * 4];

        loop {
            let serial = match Self::open_serial(port, baud_rate).await {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("[HARDWARE] Errore apertura {}: {}", port, e);
                    self.publish_status(false, 0, false).await?;
                    sleep(Duration::from_millis(self.config.reconnect_delay_ms)).await;
                    continue;
                }
            };

            println!(
                "[HARDWARE] Seriale {} @ {} baud (comandi su alpha/cmd/{})",
                port, baud_rate, self.config.hardware_id
            );

            let (mut reader, mut writer) = tokio::io::split(serial);
            let mut acc = Vec::with_capacity(self.adapter.buffer_hint() * 2);
            let mut buf = [0u8; 1024];
            let mut frames: u64 = 0;

            self.publish_status(true, frames, estop_active).await?;

            loop {
                tokio::select! {
                    r = reader.read(&mut buf) => {
                        match r {
                            Ok(0) => break,
                            Ok(n) => {
                                acc.extend_from_slice(&buf[..n]);
                                if let Err(e) = self
                                    .drain_frames(&mut acc, &mut frames, &mut state, &telemetry_topic, &mut tx_buf)
                                    .await
                                {
                                    eprintln!("[HARDWARE] Parsing frame: {}", e);
                                }
                            }
                            Err(e) => {
                                eprintln!("[HARDWARE] Lettura seriale: {}", e);
                                break;
                            }
                        }
                    }
                    cmd = cmd_sub.recv_async() => {
                        match cmd {
                            Ok(sample) => {
                                if estop_active { continue; }
                                let bytes = sample.payload().to_bytes();
                                if !bytes.is_empty() {
                                    let command = self.clamp_command(CommandVector::from_bytes(bytes.as_ref()));
                                    if let Err(e) = Self::write_command(&mut writer, self.adapter.as_ref(), &command).await {
                                        eprintln!("[HARDWARE] Invio comando seriale: {}", e);
                                    }
                                }
                            }
                            Err(_) => break,
                        }
                    }
                    estop = estop_sub.recv_async() => {
                        match estop {
                            Ok(sample) => {
                                let active = sample.payload().to_bytes().first().copied().unwrap_or(0) != 0;
                                if active != estop_active {
                                    estop_active = active;
                                    if active {
                                        if let Err(e) = Self::write_estop(&mut writer, self.adapter.as_ref(), self.config.joint_count).await {
                                            eprintln!("[SAFETY] Invio ESTOP: {}", e);
                                        }
                                        eprintln!("[SAFETY] ESTOP ATTIVO su {}", self.config.hardware_id);
                                    } else {
                                        eprintln!("[SAFETY] ESTOP disattivato su {}", self.config.hardware_id);
                                    }
                                    self.publish_status(true, frames, estop_active).await?;
                                }
                            }
                            Err(_) => break,
                        }
                    }
                }
            }

            self.publish_status(false, frames, estop_active).await?;
            sleep(Duration::from_millis(self.config.reconnect_delay_ms)).await;
        }
    }

    // -----------------------------------------------------------------------
    // Fieldbus UDP (es. Franka FCI)
    // -----------------------------------------------------------------------
    async fn run_udp_loop(
        &self,
        bind_addr: &str,
        remote_addr: &str,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let cmd_sub = self
            .session
            .declare_subscriber(&format!("alpha/cmd/{}", self.config.hardware_id))
            .await?;
        let estop_sub = self
            .session
            .declare_subscriber(&format!("alpha/cmd/{}/estop", self.config.hardware_id))
            .await?;
        let telemetry_topic = format!("alpha/telemetry/{}", self.config.hardware_id);
        let mut estop_active = false;

        // Pre-allocazione all'avvio: nessuna allocazione durante il loop.
        let mut state = StateVector::new(&self.config.hardware_id, &[]);
        let mut tx_buf = [0u8; MAX_JOINTS * 4];
        let mut cmd_buf = [0u8; COMMAND_BUF];

        loop {
            let socket = match UdpSocket::bind(bind_addr).await {
                Ok(s) => Arc::new(s),
                Err(e) => {
                    eprintln!("[HARDWARE] Errore bind UDP {}: {}", bind_addr, e);
                    self.publish_status(false, 0, estop_active).await?;
                    sleep(Duration::from_millis(self.config.reconnect_delay_ms)).await;
                    continue;
                }
            };
            if let Err(e) = socket.connect(remote_addr).await {
                eprintln!("[HARDWARE] Errore connect UDP {}: {}", remote_addr, e);
                self.publish_status(false, 0, estop_active).await?;
                sleep(Duration::from_millis(self.config.reconnect_delay_ms)).await;
                continue;
            }

            println!(
                "[HARDWARE] UDP {} → {}, (comandi su alpha/cmd/{})",
                bind_addr, remote_addr, self.config.hardware_id
            );

            let mut acc = Vec::with_capacity(self.adapter.buffer_hint() * 2);
            let mut buf = [0u8; 65535];
            let mut frames: u64 = 0;

            self.publish_status(true, frames, estop_active).await?;

            loop {
                tokio::select! {
                    r = socket.recv(&mut buf) => {
                        match r {
                            Ok(0) => break,
                            Ok(n) => {
                                acc.extend_from_slice(&buf[..n]);
                                if let Err(e) = self
                                    .drain_frames(&mut acc, &mut frames, &mut state, &telemetry_topic, &mut tx_buf)
                                    .await
                                {
                                    eprintln!("[HARDWARE] Parsing frame: {}", e);
                                }
                            }
                            Err(e) => {
                                eprintln!("[HARDWARE] Ricezione UDP: {}", e);
                                break;
                            }
                        }
                    }
                    cmd = cmd_sub.recv_async() => {
                        match cmd {
                            Ok(sample) => {
                                if estop_active { continue; }
                                let bytes = sample.payload().to_bytes();
                                if !bytes.is_empty() {
                                    let command = self.clamp_command(CommandVector::from_bytes(bytes.as_ref()));
                                    let n = self.adapter.encode_command(&command, &mut cmd_buf);
                                    if let Err(e) = socket.send(&cmd_buf[..n]).await {
                                        eprintln!("[HARDWARE] Invio comando UDP: {}", e);
                                    }
                                }
                            }
                            Err(_) => break,
                        }
                    }
                    estop = estop_sub.recv_async() => {
                        match estop {
                            Ok(sample) => {
                                let active = sample.payload().to_bytes().first().copied().unwrap_or(0) != 0;
                                if active != estop_active {
                                    estop_active = active;
                                    if active {
                                        let n = self.adapter.encode_estop(self.config.joint_count, &mut cmd_buf);
                                        if let Err(e) = socket.send(&cmd_buf[..n]).await {
                                            eprintln!("[SAFETY] Invio ESTOP: {}", e);
                                        }
                                        eprintln!("[SAFETY] ESTOP ATTIVO su {}", self.config.hardware_id);
                                    } else {
                                        eprintln!("[SAFETY] ESTOP disattivato su {}", self.config.hardware_id);
                                    }
                                    self.publish_status(true, frames, estop_active).await?;
                                }
                            }
                            Err(_) => break,
                        }
                    }
                }
            }

            self.publish_status(false, frames, estop_active).await?;
            sleep(Duration::from_millis(self.config.reconnect_delay_ms)).await;
        }
    }

    // -----------------------------------------------------------------------
    // Fieldbus TCP (KUKA Ethernet KRL, UR RT client, Comau PDL2)
    // -----------------------------------------------------------------------
    async fn run_tcp_loop(
        &self,
        remote_addr: &str,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let cmd_sub = self
            .session
            .declare_subscriber(&format!("alpha/cmd/{}", self.config.hardware_id))
            .await?;
        let estop_sub = self
            .session
            .declare_subscriber(&format!("alpha/cmd/{}/estop", self.config.hardware_id))
            .await?;
        let telemetry_topic = format!("alpha/telemetry/{}", self.config.hardware_id);
        let mut estop_active = false;

        let mut state = StateVector::new(&self.config.hardware_id, &[]);
        let mut tx_buf = [0u8; MAX_JOINTS * 4];

        loop {
            let stream = match TcpStream::connect(remote_addr).await {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("[HARDWARE] Errore connect TCP {}: {}", remote_addr, e);
                    self.publish_status(false, 0, estop_active).await?;
                    sleep(Duration::from_millis(self.config.reconnect_delay_ms)).await;
                    continue;
                }
            };

            println!(
                "[HARDWARE] TCP {} (comandi su alpha/cmd/{})",
                remote_addr, self.config.hardware_id
            );

            let (mut reader, mut writer) = tokio::io::split(stream);
            let mut acc = Vec::with_capacity(self.adapter.buffer_hint() * 2);
            let mut buf = [0u8; 65535];
            let mut frames: u64 = 0;

            self.publish_status(true, frames, estop_active).await?;

            loop {
                tokio::select! {
                    r = reader.read(&mut buf) => {
                        match r {
                            Ok(0) => break,
                            Ok(n) => {
                                acc.extend_from_slice(&buf[..n]);
                                if let Err(e) = self
                                    .drain_frames(&mut acc, &mut frames, &mut state, &telemetry_topic, &mut tx_buf)
                                    .await
                                {
                                    eprintln!("[HARDWARE] Parsing frame: {}", e);
                                }
                            }
                            Err(e) => {
                                eprintln!("[HARDWARE] Ricezione TCP: {}", e);
                                break;
                            }
                        }
                    }
                    cmd = cmd_sub.recv_async() => {
                        match cmd {
                            Ok(sample) => {
                                if estop_active { continue; }
                                let bytes = sample.payload().to_bytes();
                                if !bytes.is_empty() {
                                    let command = self.clamp_command(CommandVector::from_bytes(bytes.as_ref()));
                                    if let Err(e) = Self::write_command(&mut writer, self.adapter.as_ref(), &command).await {
                                        eprintln!("[HARDWARE] Invio comando TCP: {}", e);
                                    }
                                }
                            }
                            Err(_) => break,
                        }
                    }
                    estop = estop_sub.recv_async() => {
                        match estop {
                            Ok(sample) => {
                                let active = sample.payload().to_bytes().first().copied().unwrap_or(0) != 0;
                                if active != estop_active {
                                    estop_active = active;
                                    if active {
                                        if let Err(e) = Self::write_estop(&mut writer, self.adapter.as_ref(), self.config.joint_count).await {
                                            eprintln!("[SAFETY] Invio ESTOP: {}", e);
                                        }
                                        eprintln!("[SAFETY] ESTOP ATTIVO su {}", self.config.hardware_id);
                                    } else {
                                        eprintln!("[SAFETY] ESTOP disattivato su {}", self.config.hardware_id);
                                    }
                                    self.publish_status(true, frames, estop_active).await?;
                                }
                            }
                            Err(_) => break,
                        }
                    }
                }
            }

            self.publish_status(false, frames, estop_active).await?;
            sleep(Duration::from_millis(self.config.reconnect_delay_ms)).await;
        }
    }

    // -----------------------------------------------------------------------
    // Pipeline telemetria
    // -----------------------------------------------------------------------
    async fn drain_frames(
        &self,
        acc: &mut Vec<u8>,
        frames: &mut u64,
        state: &mut StateVector,
        telemetry_topic: &str,
        tx_buf: &mut [u8; MAX_JOINTS * 4],
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        loop {
            let Some((consumed, values, n)) =
                self.adapter.decode_telemetry(acc, self.config.joint_count)
            else {
                break;
            };
            state.set_values(&values[..n]);
            let written = state.write_to(tx_buf);
            if let Err(e) = self.session.put(telemetry_topic, &tx_buf[..written]).await {
                eprintln!("[HARDWARE] Pubblicazione telemetria: {}", e);
            }
            acc.drain(..consumed);
            *frames += 1;
        }
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Pipeline comandi (sicurezza)
    // -----------------------------------------------------------------------

    /// Pipeline comandi (sicurezza): prima sanifica i valori illegali
    /// (NaN/Inf → ultimo valore sicuro, clamp geometrico), poi smussa la
    /// curva limitando velocità, accelerazione e jerk (derivata terza).
    fn clamp_command(&self, mut command: CommandVector) -> CommandVector {
        {
            let mut sanitizer = self
                .sanitizer
                .lock()
                .expect("VectorSanitizer poisoned");
            sanitizer.sanitize_and_clamp(&mut command, &self.config.joint_limits_rad);
        }
        self.jerk_limiter
            .lock()
            .expect("JerkLimiter poisoned")
            .limit(&mut command, &self.config.joint_limits_rad);
        command
    }

    async fn write_command(
        writer: &mut (impl AsyncWrite + Unpin),
        adapter: &dyn BrandAdapter,
        command: &CommandVector,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let mut buf = [0u8; COMMAND_BUF];
        let n = adapter.encode_command(command, &mut buf);
        writer.write_all(&buf[..n]).await?;
        writer.flush().await?;
        Ok(())
    }

    async fn write_estop(
        writer: &mut (impl AsyncWrite + Unpin),
        adapter: &dyn BrandAdapter,
        joint_count: usize,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let mut buf = [0u8; COMMAND_BUF];
        let n = adapter.encode_estop(joint_count, &mut buf);
        writer.write_all(&buf[..n]).await?;
        writer.flush().await?;
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Health/status
    // -----------------------------------------------------------------------
    async fn publish_status(
        &self,
        online: bool,
        frames: u64,
        estop: bool,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let status = serde_json::json!({
            "online": online,
            "transport": self.transport_name(),
            "brand": self.adapter.brand().label(),
            "protocol": self.adapter.protocol_label(),
            "frames": frames,
            "estop": estop,
        });
        let topic = format!("alpha/status/{}", self.config.hardware_id);
        self.session
            .put(&topic, status.to_string().into_bytes())
            .await?;
        Ok(())
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamp_respects_limits_and_passes_through() {
        let limits = vec![(-1.0, 1.0), (-2.0, 2.0), (0.0, 0.5)];
        let cmd = clamp_to_limits(&[5.0, -7.0, 0.25], &limits);
        assert_eq!(cmd.as_slice(), &[1.0, -2.0, 0.25][..]);
    }

    #[test]
    fn clamp_ignores_missing_limits() {
        let cmd = clamp_to_limits(&[42.0, -42.0], &[]);
        assert_eq!(cmd.as_slice(), &[42.0, -42.0][..]);
    }

    #[test]
    fn command_vector_roundtrip() {
        let cmd = CommandVector::from_slice(&[1.5, -2.25, 3.0]);
        let mut buf = [0u8; MAX_JOINTS * 4];
        let n = cmd.write_to(&mut buf);
        let parsed = CommandVector::from_bytes(&buf[..n]);
        assert_eq!(parsed.as_slice(), cmd.as_slice());
    }

    #[test]
    fn fixed_vector_occupies_constant_memory() {
        // [f32; 32] + len: la RAM è fissa indipendentemente dai DOF.
        let small = CommandVector::zeros(2);
        let large = CommandVector::zeros(32);
        assert_eq!(std::mem::size_of_val(&small), std::mem::size_of_val(&large));
    }

    #[test]
    fn adapter_selection_matches_brand() {
        assert_eq!(adapter_for(RobotBrand::Kuka).brand(), RobotBrand::Kuka);
        assert_eq!(
            adapter_for(RobotBrand::UniversalRobots).brand(),
            RobotBrand::UniversalRobots
        );
        assert_eq!(
            adapter_for(RobotBrand::FrankaEmika).brand(),
            RobotBrand::FrankaEmika
        );
        assert_eq!(adapter_for(RobotBrand::Comau).brand(), RobotBrand::Comau);
        assert_eq!(adapter_for(RobotBrand::Amr).brand(), RobotBrand::Amr);
        assert_eq!(
            adapter_for(RobotBrand::Generic).brand(),
            RobotBrand::Generic
        );
    }

    #[test]
    fn frame_strategies_are_consistent() {
        use crate::io_bridge::adapters::FrameStrategy;
        assert_eq!(
            adapter_for(RobotBrand::Kuka).frame_strategy(),
            FrameStrategy::Delimited {
                terminator: b"</Rob>"
            }
        );
        assert_eq!(
            adapter_for(RobotBrand::UniversalRobots).frame_strategy(),
            FrameStrategy::LengthPrefixed { header_bytes: 4 }
        );
        assert_eq!(
            adapter_for(RobotBrand::FrankaEmika).frame_strategy(),
            FrameStrategy::LengthPrefixed { header_bytes: 4 }
        );
        assert_eq!(
            adapter_for(RobotBrand::Comau).frame_strategy(),
            FrameStrategy::Delimited { terminator: b"\n" }
        );
        assert_eq!(
            adapter_for(RobotBrand::Amr).frame_strategy(),
            FrameStrategy::Fixed { bytes_per_joint: 4 }
        );
    }
}
