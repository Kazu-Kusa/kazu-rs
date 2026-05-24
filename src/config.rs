//! Application and run configuration types for kazu-rs.

use log::{info, warn};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

// ── App state ───────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AppConfig {
    #[serde(default)]
    pub motion: MotionSection,
    #[serde(default)]
    pub sensor: SensorSection,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct MotionSection {
    #[serde(default = "default_port")]
    pub port: String,
    #[serde(default = "default_baudrate")]
    pub baudrate: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SensorSection {
    #[serde(default)]
    pub gyro_fsr: u16,
    #[serde(default)]
    pub accel_fsr: u8,
}

pub fn default_port() -> String {
    "/dev/ttyUSB0".into()
}
pub fn default_baudrate() -> u32 {
    115200
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            motion: MotionSection {
                port: default_port(),
                baudrate: default_baudrate(),
            },
            sensor: SensorSection {
                gyro_fsr: 2000,
                accel_fsr: 8,
            },
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RunConfig {
    #[serde(default = "default_team")]
    pub team_color: String,
    #[serde(default)]
    pub missions: MissionsSection,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct MissionsSection {
    #[serde(default)]
    pub boot: Vec<String>,
    #[serde(default)]
    pub stage: Vec<String>,
    #[serde(default)]
    pub off_stage: Vec<String>,
}

pub fn default_team() -> String {
    "blue".into()
}

pub fn load_app_config(path: &PathBuf) -> AppConfig {
    if path.exists() {
        info!("Loading config from {}", path.display());
        fs::read_to_string(path)
            .ok()
            .and_then(|s| toml::from_str(&s).ok())
            .unwrap_or_default()
    } else {
        warn!(
            "Config file not found: {}, using defaults",
            path.display()
        );
        AppConfig::default()
    }
}

pub fn load_run_config(path: &Path) -> RunConfig {
    if path.exists() {
        info!("Loading run config from {}", path.display());
        fs::read_to_string(path)
            .ok()
            .and_then(|s| toml::from_str(&s).ok())
            .unwrap_or_default()
    } else {
        warn!(
            "Run config not found: {}, using defaults",
            path.display()
        );
        RunConfig::default()
    }
}

impl Default for RunConfig {
    fn default() -> Self {
        Self {
            team_color: default_team(),
            missions: MissionsSection::default(),
        }
    }
}
