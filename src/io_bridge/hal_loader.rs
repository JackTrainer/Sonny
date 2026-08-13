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
    /// Declared maximum acceleration (rad/s²). If absent, derived from the
    /// maximum speed: reaches `max_speed_rad_s` in ~100 ms.
    #[serde(default)]
    pub max_accel_rad_s2: Option<f32>,
    /// Declared maximum jerk (rad/s³), the physical tolerance against
    /// mechanical shocks. If absent, derived from the acceleration.
    #[serde(default)]
    pub max_jerk_rad_s3: Option<f32>,
}

/// Implicit acceleration ramp: v_max in ~0.1 s.
const DEFAULT_ACCEL_PER_SPEED: f32 = 10.0;
/// Implicit jerk ramp: a_max in ~0.1 s.
const DEFAULT_JERK_PER_ACCEL: f32 = 10.0;

impl JointConfig {
    /// Effective maximum acceleration: explicit JSON value or derived from
    /// the joint's maximum speed.
    pub fn max_accel(&self) -> f32 {
        self.max_accel_rad_s2
            .unwrap_or(self.max_speed_rad_s * DEFAULT_ACCEL_PER_SPEED)
    }

    /// Effective maximum jerk: explicit JSON value or derived.
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
    /// "Serial" / "RS232" / "RS485", "Udp" / "Ethernet" or "Tcp"
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

/// Optional gripper section declared by the HAL JSON.
///
/// The gripper is a real electrical register of the machine: it lives in the
/// `StateVector` at index `pressure_slot` (measured in Newtons) and is
/// therefore mapped in memory with its own physical limits, exactly like a
/// joint.
#[derive(Debug, Clone, Deserialize)]
pub struct GripperConfig {
    /// Index of the pressure register in the `StateVector` (e.g. 6).
    pub pressure_slot: usize,
    pub min_pressure_n: f32,
    pub max_pressure_n: f32,
    /// Nominal safe grip pressure (e.g. 28 N on glass).
    #[serde(default)]
    pub nominal_grip_n: f32,
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
    /// Explicit compatibility profile (optional). If absent, inferred from
    /// the `manufacturer` field.
    #[serde(default)]
    pub brand: Option<String>,
    /// Gripper declared by the hardware (optional): pressure register index
    /// in the `StateVector` and physical safety limits in Newtons.
    #[serde(default)]
    pub gripper: Option<GripperConfig>,
}

impl HardwareConfig {
    /// Resolved brand: explicit `brand` > inference from `manufacturer`.
    pub fn resolved_brand(&self) -> RobotBrand {
        match &self.brand {
            Some(b) if !b.trim().is_empty() => RobotBrand::from_manufacturer(b),
            _ => RobotBrand::from_manufacturer(&self.manufacturer),
        }
    }

    /// Translates the `fieldbus` section of the config file into a
    /// FieldbusConfig ready for HardwareAbstraction, with the joint limits
    /// already loaded for the safety clamp on commands.
    pub fn to_fieldbus_config(&self) -> Result<Option<FieldbusConfig>, String> {
        let Some(fb) = &self.fieldbus else {
            return Ok(None);
        };

        let transport = match fb.protocol.to_ascii_lowercase().as_str() {
            "serial" | "rs232" | "rs485" => {
                let port = fb
                    .port
                    .clone()
                    .ok_or("Fieldbus 'Serial' requires the 'port' field")?;
                let baud_rate = fb
                    .baud_rate
                    .ok_or("Fieldbus 'Serial' requires the 'baud_rate' field")?;
                FieldbusTransport::Serial { port, baud_rate }
            }
            "udp" | "ethernet" => {
                let bind_addr = fb
                    .bind_addr
                    .clone()
                    .ok_or("Fieldbus 'Udp' requires the 'bind_addr' field")?;
                let remote_addr = fb
                    .remote_addr
                    .clone()
                    .ok_or("Fieldbus 'Udp' requires the 'remote_addr' field")?;
                FieldbusTransport::Udp {
                    bind_addr,
                    remote_addr,
                }
            }
            "tcp" => {
                let remote_addr = fb
                    .remote_addr
                    .clone()
                    .ok_or("Fieldbus 'Tcp' requires the 'remote_addr' field")?;
                FieldbusTransport::Tcp { remote_addr }
            }
            other => return Err(format!("Unsupported fieldbus protocol: '{}'", other)),
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

    /// Builds the map of the machine's electrical registers: one physical
    /// limit for every slot of the `StateVector`, in register order.
    ///
    /// The joints occupy all the slots except the one reserved for the
    /// gripper: if `gripper.pressure_slot` is present, the pressure register
    /// is inserted at that position and the following joints shift by one.
    pub fn register_limits(&self) -> Vec<(f32, f32)> {
        let mut limits = Vec::with_capacity(self.dof + usize::from(self.gripper.is_some()));
        let pressure_slot = self.gripper.as_ref().map(|g| g.pressure_slot);
        for (i, joint) in self.joints.iter().enumerate() {
            if Some(i) == pressure_slot {
                let g = self.gripper.as_ref().expect("pressure_slot requires the gripper");
                limits.push((g.min_pressure_n, g.max_pressure_n));
            }
            limits.push((joint.min_angle_rad, joint.max_angle_rad));
        }
        if let Some(slot) = pressure_slot {
            if slot >= self.joints.len() {
                let g = self.gripper.as_ref().expect("pressure_slot requires the gripper");
                limits.push((g.min_pressure_n, g.max_pressure_n));
            }
        }
        limits
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

    /// Injects the HAL JSON as a string: the test does not know which machine
    /// it faces until the parser maps registers and physical limits in memory.
    pub fn from_str(content: &str) -> Result<HardwareConfig, Box<dyn std::error::Error + Send + Sync>> {
        let config: HardwareConfig = serde_json::from_str(content)?;
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

    #[test]
    fn from_str_maps_gripper_register_between_joints() {
        let json = r#"{
            "robot_name": "TEST-HYBRID",
            "manufacturer": "Test Corp",
            "dof": 2,
            "expected_hz": 100.0,
            "max_jitter_us": 1500,
            "joints": [
                {"name": "j1", "min_angle_rad": -1.0, "max_angle_rad": 1.0, "max_speed_rad_s": 2.0, "home_angle_rad": 0.0, "gear_ratio": 1.0},
                {"name": "j2", "min_angle_rad": -2.0, "max_angle_rad": 2.0, "max_speed_rad_s": 2.0, "home_angle_rad": 0.0, "gear_ratio": 1.0}
            ],
            "kinematic_chain": [
                {"parent": "base", "child": "j1", "translation_m": [0.0, 0.0, 0.1], "rotation_axis": [0.0, 0.0, 1.0]},
                {"parent": "j1", "child": "j2", "translation_m": [0.0, 0.0, 0.1], "rotation_axis": [0.0, 1.0, 0.0]}
            ],
            "gripper": {"pressure_slot": 1, "min_pressure_n": 0.0, "max_pressure_n": 100.0, "nominal_grip_n": 28.0}
        }"#;
        let config = HalLoader::from_str(json).unwrap();
        let limits = config.register_limits();
        assert_eq!(limits, vec![(-1.0, 1.0), (0.0, 100.0), (-2.0, 2.0)]);
        assert_eq!(config.gripper.unwrap().nominal_grip_n, 28.0);
    }
}
