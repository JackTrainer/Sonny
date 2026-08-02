//! Adattatori di protocollo per i codec nativi dei marchi di robot.
//!
//! Ogni adattatore implementa [`BrandAdapter`]: converte i byte grezzi del
//! fieldbus del marchio in un [`StateVector`] (telemetria) e un
//! [`CommandVector`] in byte nativi del marchio (comandi). Il nucleo SONNY
//! resta identico: cambia solo il codec al bordo hardware.
//!
//! Strategie di framing supportate:
//! - **Fixed**: frame a dimensione fissa (es. f32 LE generico, 4 byte/giunto);
//! - **LengthPrefixed**: header di lunghezza (es. UR RT client, FCI);
//! - **Delimited**: messaggi terminati da un delimitatore (XML KUKA, PDL2, JSON).

pub mod amr_ros2;
pub mod comau_pdl;
pub mod franka_fci;
pub mod generic_binary;
pub mod kuka_eth_krl;
pub mod universal_robots;

use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};
use crate::io_bridge::robot_brand::RobotBrand;

/// Strategia di framing del protocollo nativo.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameStrategy {
    /// Frame a dimensione fissa: `joint_count * bytes_per_joint`.
    Fixed { bytes_per_joint: usize },
    /// Header di lunghezza (big-endian) seguito dal payload.
    LengthPrefixed { header_bytes: usize },
    /// Messaggi delimitati da una sequenza finale (XML, testo, JSON).
    Delimited { terminator: &'static [u8] },
}

impl FrameStrategy {
    pub fn label(&self) -> &'static str {
        match self {
            FrameStrategy::Fixed { .. } => "FIXED",
            FrameStrategy::LengthPrefixed { .. } => "LENGTH-PREFIXED",
            FrameStrategy::Delimited { .. } => "DELIMITED",
        }
    }
}

/// Codec nativo di un marchio. Deve essere `Send + Sync`: viene usato dagli
/// stessi loop asincroni della `HardwareAbstraction`.
pub trait BrandAdapter: Send + Sync {
    fn brand(&self) -> RobotBrand;

    fn protocol_label(&self) -> &'static str;

    fn frame_strategy(&self) -> FrameStrategy;

    /// Decodifica UN messaggio di telemetria completo da `acc`.
    ///
    /// Restituisce `(byte_consumed, valori, giunti_attivi)` quando è presente
    /// un frame completo, `None` quando servono altri byte. Non deve mai
    /// allocare: i valori vengono scritti nell'array statico pre-allocato.
    fn decode_telemetry(
        &self,
        acc: &[u8],
        joint_count: usize,
    ) -> Option<(usize, [f32; MAX_JOINTS], usize)>;

    /// Codifica un comando nei byte nativi del marchio in `out`.
    /// Restituisce i byte scritti. Non deve mai allocare.
    fn encode_command(&self, command: &CommandVector, out: &mut [u8]) -> usize;

    /// Codifica il comando di ESTOP (tutti i giunti a zero) in forma nativa.
    fn encode_estop(&self, joint_count: usize, out: &mut [u8]) -> usize {
        let zeros = CommandVector::zeros(joint_count);
        self.encode_command(&zeros, out)
    }

    /// Costo di pre-allocazione suggerito per il buffer di ricezione.
    fn buffer_hint(&self) -> usize {
        1024
    }
}

/// Seleziona il codec nativo in base al marchio.
pub fn adapter_for(brand: RobotBrand) -> Box<dyn BrandAdapter> {
    match brand {
        RobotBrand::Kuka => Box::new(kuka_eth_krl::KukaEthernetKrl),
        RobotBrand::Comau => Box::new(comau_pdl::ComauPdl2),
        RobotBrand::UniversalRobots => Box::new(universal_robots::UniversalRobots),
        RobotBrand::FrankaEmika => Box::new(franka_fci::FrankaFci),
        RobotBrand::Amr => Box::new(amr_ros2::AmrRos2),
        RobotBrand::Generic => Box::new(generic_binary::GenericBinary),
    }
}
