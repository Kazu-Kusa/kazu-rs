use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EdgeConfig {
    #[serde(default = "default_edge_lower")]
    pub lower_threshold: [f64; 4],
    #[serde(default = "default_edge_upper")]
    pub upper_threshold: [f64; 4],
    #[serde(default = "default_fallback_speed")]
    pub fallback_speed: i32,
    #[serde(default = "default_fallback_duration")]
    pub fallback_duration: f64,
    #[serde(default = "default_advance_speed")]
    pub advance_speed: i32,
    #[serde(default = "default_advance_duration")]
    pub advance_duration: f64,
    #[serde(default = "default_turn_speed")]
    pub turn_speed: i32,
    #[serde(default = "default_full_turn_duration")]
    pub full_turn_duration: f64,
    #[serde(default = "default_half_turn_duration")]
    pub half_turn_duration: f64,
    #[serde(default = "default_turn_left_prob")]
    pub turn_left_prob: f64,
    #[serde(default = "default_drift_speed")]
    pub drift_speed: i32,
    #[serde(default = "default_drift_duration")]
    pub drift_duration: f64,
    #[serde(default = "default_use_gray_io")]
    pub use_gray_io: bool,
}

fn default_edge_lower() -> [f64; 4] {
    [1740.0, 1819.0, 1819.0, 1740.0]
}
fn default_edge_upper() -> [f64; 4] {
    [2100.0, 2470.0, 2470.0, 2100.0]
}
fn default_fallback_speed() -> i32 {
    2600
}
fn default_fallback_duration() -> f64 {
    0.2
}
fn default_advance_speed() -> i32 {
    2400
}
fn default_advance_duration() -> f64 {
    0.35
}
fn default_turn_speed() -> i32 {
    2800
}
fn default_full_turn_duration() -> f64 {
    0.45
}
fn default_half_turn_duration() -> f64 {
    0.225
}
fn default_turn_left_prob() -> f64 {
    0.5
}
fn default_drift_speed() -> i32 {
    1500
}
fn default_drift_duration() -> f64 {
    0.13
}
fn default_use_gray_io() -> bool {
    true
}

impl Default for EdgeConfig {
    fn default() -> Self {
        Self {
            lower_threshold: default_edge_lower(),
            upper_threshold: default_edge_upper(),
            fallback_speed: default_fallback_speed(),
            fallback_duration: default_fallback_duration(),
            advance_speed: default_advance_speed(),
            advance_duration: default_advance_duration(),
            turn_speed: default_turn_speed(),
            full_turn_duration: default_full_turn_duration(),
            half_turn_duration: default_half_turn_duration(),
            turn_left_prob: default_turn_left_prob(),
            drift_speed: default_drift_speed(),
            drift_duration: default_drift_duration(),
            use_gray_io: default_use_gray_io(),
        }
    }
}
