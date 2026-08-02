use serde::Deserialize;
use std::fs;
use std::path::Path;

use crate::io_bridge::hardware_abstraction::{FieldbusConfig, FieldbusTransport};
use crate::io_bridge::robot_brand::RobotBrand;

#[derive(Debug, Clone, Deserialize)]
pub struct JointConfig {
    pub name: String,
    pub min_angle_rad: f32,
    pub max_angle_rad: f32,
    pub max_speed_rad_s: f32,
    pub home_angle_rad: f32,
    pub gear_ratio: f32,
    /// Accelerazione massima dichiarata (rad/s²). Se assente, derivata dalla
    /// velocità massima: raggiunge `max_speed_rad_s` in ~100 ms.
    #[serde(default)]
    pub max_accel_rad_s2: Option<f32>,
    /// Jerk massimo dichiarato (rad/s³), la tolleranza fisica contro gli
    /// shock meccanici. Se assente, derivato dall'accelerazione.
    #[serde(default)]
    pub max_jerk_rad_s3: Option<f32>,
}

/// Rampa di accelerazione implicita: v_max in ~0.1 s.
const DEFAULT_ACCEL_PER_SPEED: f32 = 10.0;
/// Rampa di jerk implicita: a_max in ~0.1 s.
const DEFAULT_JERK_PER_ACCEL: f32 = 10.0;

impl JointConfig {
    /// Accelerazione massima effettiva: valore esplicito del JSON oppure
    /// derivato dalla velocità massima del giunto.
    pub fn max_accel(&self) -> f32 {
        self.max_accel_rad_s2
            .unwrap_or(self.max_speed_rad_s * DEFAULT_ACCEL_PER_SPEED)
    }

    /// Jerk massimo effettivo: valore esplicito del JSON oppure derivato.
    pub fn max_jerk(&self) -> f32 {
        self.max_jerk_rad_s3
            .unwrap_or(self.max_accel() * DEFAULT_JERK_PER_ACCEL)
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct KinematicLink {
    pub parent: String,
    pub child: String,
    pub translation_m: [f32; 3],
    pub rotation_axis: [f32; 3],
}

#[derive(Debug, Clone, Deserialize)]
pub struct FieldbusJsonConfig {
    /// "Serial" / "RS232" / "RS485", "Udp" / "Ethernet" oppure "Tcp"
    pub protocol: String,
    #[serde(default)]
    pub port: Option<String>,
    #[serde(default)]
    pub baud_rate: Option<u32>,
    #[serde(default)]
    pub bind_addr: Option<String>,
    #[serde(default)]
    pub remote_addr: Option<String>,
    #[serde(default = "default_timeout_ms")]
    pub frame_timeout_ms: u64,
    #[serde(default = "default_reconnect_ms")]
    pub reconnect_delay_ms: u64,
}

fn default_timeout_ms() -> u64 {
    100
}

fn default_reconnect_ms() -> u64 {
    2000
}

#[derive(Debug, Clone, Deserialize)]
pub struct HardwareConfig {
    pub robot_name: String,
    pub manufacturer: String,
    pub dof: usize,
    pub expected_hz: f64,
    pub max_jitter_us: u64,
    pub joints: Vec<JointConfig>,
    pub kinematic_chain: Vec<KinematicLink>,
    #[serde(default)]
    pub fieldbus: Option<FieldbusJsonConfig>,
    /// Profilo di compatibilità esplicito (opzionale). Se assente viene
    /// dedotto dal campo `manufacturer`.
    #[serde(default)]
    pub brand: Option<String>,
}

impl HardwareConfig {
    /// Marchio risolto: `brand` esplicito > deduzione da `manufacturer`.
    pub fn resolved_brand(&self) -> RobotBrand {
        match &self.brand {
            Some(b) if !b.trim().is_empty() => RobotBrand::from_manufacturer(b),
            _ => RobotBrand::from_manufacturer(&self.manufacturer),
        }
    }

    /// Traduce la sezione `fieldbus` del file di config in un FieldbusConfig
    /// pronto per l'HardwareAbstraction, con i limiti dei giunti già caricati
    /// per il clamp di sicurezza sui comandi.
    pub fn to_fieldbus_config(&self) -> Result<Option<FieldbusConfig>, String> {
        let Some(fb) = &self.fieldbus else {
            return Ok(None);
        };

        let transport = match fb.protocol.to_ascii_lowercase().as_str() {
            "serial" | "rs232" | "rs485" => {
                let port = fb
                    .port
                    .clone()
                    .ok_or("Fieldbus 'Serial' richiede il campo 'port'")?;
                let baud_rate = fb
                    .baud_rate
                    .ok_or("Fieldbus 'Serial' richiede il campo 'baud_rate'")?;
                FieldbusTransport::Serial { port, baud_rate }
            }
            "udp" | "ethernet" => {
                let bind_addr = fb
                    .bind_addr
                    .clone()
                    .ok_or("Fieldbus 'Udp' richiede il campo 'bind_addr'")?;
                let remote_addr = fb
                    .remote_addr
                    .clone()
                    .ok_or("Fieldbus 'Udp' richiede il campo 'remote_addr'")?;
                FieldbusTransport::Udp {
                    bind_addr,
                    remote_addr,
                }
            }
            "tcp" => {
                let remote_addr = fb
                    .remote_addr
                    .clone()
                    .ok_or("Fieldbus 'Tcp' richiede il campo 'remote_addr'")?;
                FieldbusTransport::Tcp { remote_addr }
            }
            other => return Err(format!("Protocollo fieldbus non supportato: '{}'", other)),
        };

        let joint_limits_rad = self
            .joints
            .iter()
            .map(|j| (j.min_angle_rad, j.max_angle_rad))
            .collect();
        let max_speed_rad_s = self.joints.iter().map(|j| j.max_speed_rad_s).collect();
        let max_accel_rad_s2 = self.joints.iter().map(|j| j.max_accel()).collect();
        let max_jerk_rad_s3 = self.joints.iter().map(|j| j.max_jerk()).collect();

        Ok(Some(FieldbusConfig {
            transport,
            hardware_id: self.robot_name.clone(),
            joint_count: self.dof,
            frame_timeout_ms: fb.frame_timeout_ms,
            reconnect_delay_ms: fb.reconnect_delay_ms,
            joint_limits_rad,
            expected_hz: self.expected_hz,
            max_speed_rad_s,
            max_accel_rad_s2,
            max_jerk_rad_s3,
            brand: self.resolved_brand(),
        }))
    }
}

pub struct HalLoader;

impl HalLoader {
    pub fn from_file<P: AsRef<Path>>(
        path: P,
    ) -> Result<HardwareConfig, Box<dyn std::error::Error + Send + Sync>> {
        let content = fs::read_to_string(path)?;
        let config: HardwareConfig = serde_json::from_str(&content)?;
        Ok(config)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_brand_config_resolves_its_own_profile() {
        let cases = [
            ("configs/kuka_kr6_r900.json", RobotBrand::Kuka),
            ("configs/comau_racer5.json", RobotBrand::Comau),
            (
                "configs/universal_robots_ur5e.json",
                RobotBrand::UniversalRobots,
            ),
            ("configs/franka_emika_panda.json", RobotBrand::FrankaEmika),
            ("configs/amr_standard_base.json", RobotBrand::Amr),
            (
                "configs/robot_6dof_anthropomorphic.json",
                RobotBrand::Generic,
            ),
        ];
        for (path, expected) in cases {
            let config = HalLoader::from_file(path).unwrap_or_else(|e| panic!("{}: {}", path, e));
            assert_eq!(config.resolved_brand(), expected, "{}", path);
            assert_eq!(config.dof, config.joints.len());
        }
    }

    #[test]
    fn tcp_fieldbus_maps_to_tcp_transport() {
        let config = HalLoader::from_file("configs/kuka_kr6_r900.json").unwrap();
        let fb = config.to_fieldbus_config().unwrap().unwrap();
        assert!(matches!(fb.transport, FieldbusTransport::Tcp { .. }));
        assert_eq!(fb.brand, RobotBrand::Kuka);
    }
}
