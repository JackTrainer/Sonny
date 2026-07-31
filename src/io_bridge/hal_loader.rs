use serde::Deserialize;
use std::fs;
use std::path::Path;

#[derive(Debug, Clone, Deserialize)]
pub struct JointConfig {
    pub name: String,
    pub min_angle_rad: f32,
    pub max_angle_rad: f32,
    pub max_speed_rad_s: f32,
    pub home_angle_rad: f32,
    pub gear_ratio: f32,
}

#[derive(Debug, Clone, Deserialize)]
pub struct KinematicLink {
    pub parent: String,
    pub child: String,
    pub translation_m: [f32; 3],
    pub rotation_axis: [f32; 3],
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
}

pub struct HalLoader;

impl HalLoader {
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<HardwareConfig, Box<dyn std::error::Error + Send + Sync>> {
        let content = fs::read_to_string(path)?;
        let config: HardwareConfig = serde_json::from_str(&content)?;
        Ok(config)
    }
}
