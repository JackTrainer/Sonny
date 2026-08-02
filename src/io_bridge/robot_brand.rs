/// Profili di compatibilità dei principali marchi di robot industriali.
///
/// SONNY è "agnostico per costruzione": il campo `manufacturer` (o il campo
/// opzionale `brand`) del file JSON HAL determina il profilo di compatibilità
/// del robot. Ogni marchio espone il proprio codec nativo tramite il modulo
/// `adapters`, senza modificare il nucleo del microkernel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RobotBrand {
    /// Profilo generico a frame binario f32 LE (nessun adattatore specifico).
    #[default]
    Generic,
    /// KUKA — controllori KRC4/KRC5 via Ethernet KRL (XML su TCP, porta 7911).
    Kuka,
    /// Comau — controllori C5G/C6G via bridge socket PDL2 (testo su TCP).
    Comau,
    /// Universal Robots — Real-Time Client (porta 30003) + URScript (30002).
    UniversalRobots,
    /// Franka Emika — Panda via FCI (Franka Control Interface, UDP 30401).
    FrankaEmika,
    /// Piattaforme AMR standard — mapping twist/odometry stile ROS2.
    Amr,
}

impl RobotBrand {
    /// Riconosce il marchio da una stringa libera (`manufacturer` o `brand`).
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

    /// Etichetta leggibile per log e interfaccia.
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

    /// DOF nativi del marchio (usati come fallback quando la config non li dà).
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

    /// Protocollo nativo usato dal codec dell'adattatore.
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

    /// Trasporto suggerito dal marchio (per la config fieldbus predefinita).
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
