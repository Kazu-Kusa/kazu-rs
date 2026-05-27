//! Application and run configuration types for kazu-rs.

//!
//! Port of `kazu/config.py`.

use log::{info, warn};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

// ── Context variable keys ─────────────────────────────────────

/// Context variable names — mirrors Python `ContextVar` enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ContextVar {
    /// Previous salvo speed: (fl, rl, fr, rr)
    PrevSalvoSpeed,
    /// Whether the robot is aligned.
    IsAligned,
    /// Recorded behavior pack.
    RecordedPack,
    /// Gradient-generated speed.
    GradientSpeed,
    /// Unclear zone gray ADC value.
    UnclearZoneGray,
}

impl ContextVar {
    /// Default value for each context variable.
    pub fn default_value(&self) -> serde_json::Value {
        match self {
            ContextVar::PrevSalvoSpeed => {
                serde_json::json!([0, 0, 0, 0])
            }
            ContextVar::IsAligned => serde_json::json!(false),
            ContextVar::RecordedPack => serde_json::json!([]),
            ContextVar::GradientSpeed => serde_json::json!(0),
            ContextVar::UnclearZoneGray => serde_json::json!(0),
        }
    }

    /// Export all context vars and their defaults as a map.
    pub fn export_context() -> HashMap<String, serde_json::Value> {
        [
            ContextVar::PrevSalvoSpeed,
            ContextVar::IsAligned,
            ContextVar::RecordedPack,
            ContextVar::GradientSpeed,
            ContextVar::UnclearZoneGray,
        ]
        .iter()
        .map(|cv| (format!("{:?}", cv), cv.default_value()))
        .collect()
    }
}

// ── Submodules ─────────────────────────────────────────────────

mod behavior;
mod hardware;
mod stage;

pub use behavior::{
    EdgeConfig, FenceConfig, GradientConfig, RandTurn, RandWalk, ScanConfig, SearchConfig,
    SurroundingConfig, TagGroup,
};
pub use hardware::{DebugConfig, MotionConfig, SensorConfig, VisionConfig};
pub use stage::{BackStageConfig, BootConfig, PerformanceConfig, StageConfig, StrategyConfig};

// ── AppConfig ──────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AppConfig {
    #[serde(default)]
    pub motion: MotionConfig,
    #[serde(default)]
    pub vision: VisionConfig,
    #[serde(default)]
    pub debug: DebugConfig,
    #[serde(default)]
    pub sensor: SensorConfig,
    #[serde(default)]
    pub edge: EdgeConfig,
    #[serde(default)]
    pub surrounding: SurroundingConfig,
    #[serde(default)]
    pub gradient: GradientConfig,
    #[serde(default)]
    pub scan: ScanConfig,
    #[serde(default)]
    pub search: SearchConfig,
    #[serde(default)]
    pub rand_turn: RandTurn,
    #[serde(default)]
    pub rand_walk: RandWalk,
    #[serde(default)]
    pub fence: FenceConfig,
    #[serde(default)]
    pub boot: BootConfig,
    #[serde(default)]
    pub backstage: BackStageConfig,
    #[serde(default)]
    pub stage: StageConfig,
    #[serde(default)]
    pub strategy: StrategyConfig,
    #[serde(default)]
    pub perf: PerformanceConfig,
}

// ── RunConfig ──────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RunConfig {
    #[serde(default)]
    pub strategy: StrategyConfig,
    #[serde(default)]
    pub boot: BootConfig,
    #[serde(default)]
    pub backstage: BackStageConfig,
    #[serde(default)]
    pub stage: StageConfig,
    #[serde(default)]
    pub edge: EdgeConfig,
    #[serde(default)]
    pub surrounding: SurroundingConfig,
    #[serde(default)]
    pub search: SearchConfig,
    #[serde(default)]
    pub fence: FenceConfig,
    #[serde(default)]
    pub perf: PerformanceConfig,
    #[serde(default = "default_team_str")]
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

fn default_team_str() -> String {
    "blue".into()
}

impl Default for RunConfig {
    fn default() -> Self {
        Self {
            strategy: StrategyConfig::default(),
            boot: BootConfig::default(),
            backstage: BackStageConfig::default(),
            stage: StageConfig::default(),
            edge: EdgeConfig::default(),
            surrounding: SurroundingConfig::default(),
            search: SearchConfig::default(),
            fence: FenceConfig::default(),
            perf: PerformanceConfig::default(),
            team_color: default_team_str(),
            missions: MissionsSection::default(),
        }
    }
}

// ── I/O helpers ────────────────────────────────────────────────

pub fn default_port() -> String {
    "/dev/ttyUSB0".into()
}
// Used by serial port initialization (ports.rs) and bdmc-rs controller setup.
pub fn default_baudrate() -> u32 {
    115200
}
pub fn load_app_config(path: &PathBuf) -> AppConfig {
    if path.exists() {
        info!("Loading config from {}", path.display());
        fs::read_to_string(path)
            .ok()
            .and_then(|s| toml::from_str(&s).ok())
            .unwrap_or_default()
    } else {
        warn!("Config file not found: {}, using defaults", path.display());
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
        warn!("Run config not found: {}, using defaults", path.display());
        RunConfig::default()
    }
}
