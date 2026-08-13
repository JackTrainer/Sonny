/// Compatibility profiles of the main industrial robot brands.
///
/// SONNY is "agnostic by construction": the `manufacturer` field (or the
/// optional `brand` field) of the HAL JSON file determines the robot's
/// compatibility profile. Each brand exposes its own native codec through the
/// `adapters` module, without touching the microkernel core.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RobotBrand {
    /// Generic profile with f32 LE binary frames (no specific adapter).
    #[default]
    Generic,
    /// KUKA — KRC4/KRC5 controllers over Ethernet KRL (XML on TCP, port 7911).
    Kuka,
    /// Comau — C5G/C6G controllers over PDL2 socket bridge (text on TCP).
    Comau,
    /// Universal Robots — Real-Time Client (port 30003) + URScript (30002).
    UniversalRobots,
    /// Franka Emika — Panda over FCI (Franka Control Interface, UDP 30401).
    FrankaEmika,
    /// Standard AMR platforms — twist/odometry mapping in ROS2 style.
    Amr,
}

impl RobotBrand {
    /// Recognizes the brand from a free-form string (`manufacturer` or `brand`).
    pub fn from_manufacturer(manufacturer: &str) -> RobotBrand {
        let s = manufacturer.to_ascii_lowercase();
        if s.contains("kuka") {
            RobotBrand::Kuka
        } else if s.contains("comau") {
            RobotBrand::Comau
        } else if s.contains("franka") || s.contains("emika") || s.contains("panda") {
            RobotBrand::FrankaEmika
        } else if s.contains("universal") || s.contains("ur") {
            RobotBrand::UniversalRobots
        } else if s.contains("amr") || s.contains("agv") || s.contains("mobile") {
            RobotBrand::Amr
        } else {
            RobotBrand::Generic
        }
    }

    /// Readable label for logs and UI.
    pub fn label(&self) -> &'static str {
        match self {
            RobotBrand::Generic => "GENERIC",
            RobotBrand::Kuka => "KUKA",
            RobotBrand::Comau => "COMAU",
            RobotBrand::UniversalRobots => "UNIVERSAL-ROBOTS",
            RobotBrand::FrankaEmika => "FRANKA-EMIKA",
            RobotBrand::Amr => "AMR",
        }
    }

    /// Native DOF of the brand (used as a fallback when the config does not provide it).
    pub fn native_dof(&self) -> usize {
        match self {
            RobotBrand::FrankaEmika => 7,
            RobotBrand::Amr => 4,
            RobotBrand::Kuka
            | RobotBrand::Comau
            | RobotBrand::UniversalRobots
            | RobotBrand::Generic => 6,
        }
    }

    /// Native protocol used by the adapter codec.
    pub fn native_protocol(&self) -> &'static str {
        match self {
            RobotBrand::Generic => "Fixed-f32-LE",
            RobotBrand::Kuka => "Ethernet-KRL-XML",
            RobotBrand::Comau => "PDL2-Socket",
            RobotBrand::UniversalRobots => "UR-RTClient/URScript",
            RobotBrand::FrankaEmika => "FCI-UDP",
            RobotBrand::Amr => "ROS2-Twist/Odom",
        }
    }

    /// Transport suggested by the brand (for the default fieldbus config).
    pub fn default_transport(&self) -> &'static str {
        match self {
            RobotBrand::FrankaEmika => "Udp",
            RobotBrand::Kuka | RobotBrand::Comau | RobotBrand::UniversalRobots => "Tcp",
            RobotBrand::Generic | RobotBrand::Amr => "Serial",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detects_every_brand_from_manufacturer_string() {
        assert_eq!(RobotBrand::from_manufacturer("KUKA AG"), RobotBrand::Kuka);
        assert_eq!(
            RobotBrand::from_manufacturer("Comau Robotics"),
            RobotBrand::Comau
        );
        assert_eq!(
            RobotBrand::from_manufacturer("Universal Robots A/S"),
            RobotBrand::UniversalRobots
        );
        assert_eq!(
            RobotBrand::from_manufacturer("Franka Emika GmbH"),
            RobotBrand::FrankaEmika
        );
        assert_eq!(
            RobotBrand::from_manufacturer("Panda"),
            RobotBrand::FrankaEmika
        );
        assert_eq!(
            RobotBrand::from_manufacturer("AMR platform"),
            RobotBrand::Amr
        );
        assert_eq!(
            RobotBrand::from_manufacturer("ThirdParty-Robotics"),
            RobotBrand::Generic
        );
    }

    #[test]
    fn native_dofs_are_sensible() {
        assert_eq!(RobotBrand::FrankaEmika.native_dof(), 7);
        assert_eq!(RobotBrand::UniversalRobots.native_dof(), 6);
        assert_eq!(RobotBrand::Amr.native_dof(), 4);
    }
}
