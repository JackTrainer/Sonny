//! Protocol adapters for the native codecs of robot brands.
//!
//! Each adapter implements [`BrandAdapter`]: it converts the raw bytes of the
//! brand's fieldbus into a [`StateVector`] (telemetry) and a
//! [`CommandVector`] into brand-native bytes (commands). The SONNY core
//! stays identical: only the codec at the hardware edge changes.
//!
//! Supported framing strategies:
//! - **Fixed**: fixed-size frames (e.g. generic f32 LE, 4 bytes/joint);
//! - **LengthPrefixed**: length header (e.g. UR RT client, FCI);
//! - **Delimited**: messages terminated by a delimiter (KUKA XML, PDL2, JSON).

pub mod amr_ros2;
pub mod comau_pdl;
pub mod franka_fci;
pub mod generic_binary;
pub mod kuka_eth_krl;
pub mod universal_robots;

use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};
use crate::io_bridge::robot_brand::RobotBrand;

/// Framing strategy of the native protocol.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameStrategy {
    /// Fixed-size frame: `joint_count * bytes_per_joint`.
    Fixed { bytes_per_joint: usize },
    /// Length header (big-endian) followed by the payload.
    LengthPrefixed { header_bytes: usize },
    /// Messages terminated by a trailing sequence (XML, text, JSON).
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

/// Native codec of a brand. Must be `Send + Sync`: it is used by the same
/// async loops as the `HardwareAbstraction`.
pub trait BrandAdapter: Send + Sync {
    fn brand(&self) -> RobotBrand;

    fn protocol_label(&self) -> &'static str;

    fn frame_strategy(&self) -> FrameStrategy;

    /// Decodes ONE complete telemetry message from `acc`.
    ///
    /// Returns `(byte_consumed, values, active_joints)` when a complete frame
    /// is present, `None` when more bytes are needed. It must never allocate:
    /// values are written into the pre-allocated static array.
    fn decode_telemetry(
        &self,
        acc: &[u8],
        joint_count: usize,
    ) -> Option<(usize, [f32; MAX_JOINTS], usize)>;

    /// Encodes a command into the brand's native bytes in `out`.
    /// Returns the bytes written. It must never allocate.
    fn encode_command(&self, command: &CommandVector, out: &mut [u8]) -> usize;

    /// Encodes the ESTOP command (all joints at zero) in native form.
    fn encode_estop(&self, joint_count: usize, out: &mut [u8]) -> usize {
        let zeros = CommandVector::zeros(joint_count);
        self.encode_command(&zeros, out)
    }

    /// Suggested pre-allocation size for the receive buffer.
    fn buffer_hint(&self) -> usize {
        1024
    }
}

/// Selects the native codec based on the brand.
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
