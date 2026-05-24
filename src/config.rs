//! Application and run configuration types for kazu-rs.
#![allow(dead_code)]
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

// ── TagGroup ───────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TagGroup {
    pub team_color: String,
    #[serde(default = "default_enemy_tag")]
    pub enemy_tag: i32,
    #[serde(default = "default_ally_tag")]
    pub ally_tag: i32,
    #[serde(default)]
    pub neutral_tag: i32,
    #[serde(default = "default_tag_id")]
    pub default_tag: i32,
}

fn default_enemy_tag() -> i32 { 1 }
fn default_ally_tag() -> i32 { 0 }
fn default_tag_id() -> i32 { 0 }

impl Default for TagGroup {
    fn default() -> Self {
        Self {
            team_color: "blue".into(),
            enemy_tag: 1,
            ally_tag: 0,
            neutral_tag: 0,
            default_tag: 0,
        }
    }
}

impl TagGroup {
    pub fn new(team_color: &str) -> Self {
        let (enemy, ally) = match team_color {
            "online" => (1, 0),
            "yellow" => (1, 2),
            "blue" => (2, 1),
            _ => (1, 0),
        };
        Self {
            team_color: team_color.to_string(),
            enemy_tag: enemy,
            ally_tag: ally,
            neutral_tag: 0,
            default_tag: 0,
        }
    }
}

// ── EdgeConfig ─────────────────────────────────────────────────

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

fn default_edge_lower() -> [f64; 4] { [1740.0, 1819.0, 1819.0, 1740.0] }
fn default_edge_upper() -> [f64; 4] { [2100.0, 2470.0, 2470.0, 2100.0] }
fn default_fallback_speed() -> i32 { 2600 }
fn default_fallback_duration() -> f64 { 0.2 }
fn default_advance_speed() -> i32 { 2400 }
fn default_advance_duration() -> f64 { 0.35 }
fn default_turn_speed() -> i32 { 2800 }
fn default_full_turn_duration() -> f64 { 0.45 }
fn default_half_turn_duration() -> f64 { 0.225 }
fn default_turn_left_prob() -> f64 { 0.5 }
fn default_drift_speed() -> i32 { 1500 }
fn default_drift_duration() -> f64 { 0.13 }
fn default_use_gray_io() -> bool { true }

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

// ── SurroundingConfig ──────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SurroundingConfig {
    #[serde(default)]
    pub io_encounter_object_value: i32,
    #[serde(default = "default_surr_left_adc")]
    pub left_adc_lower_threshold: i32,
    #[serde(default = "default_surr_right_adc")]
    pub right_adc_lower_threshold: i32,
    #[serde(default = "default_surr_front_adc")]
    pub front_adc_lower_threshold: i32,
    #[serde(default = "default_surr_back_adc")]
    pub back_adc_lower_threshold: i32,
    #[serde(default = "default_atk_break_front")]
    pub atk_break_front_lower_threshold: i32,
    #[serde(default = "default_atk_break_use_edge")]
    pub atk_break_use_edge_sensors: bool,
    #[serde(default = "default_atk_speed_enemy_car")]
    pub atk_speed_enemy_car: i32,
    #[serde(default = "default_atk_speed_enemy_box")]
    pub atk_speed_enemy_box: i32,
    #[serde(default = "default_atk_speed_neutral_box")]
    pub atk_speed_neutral_box: i32,
    #[serde(default = "default_fallback_speed_ally_box")]
    pub fallback_speed_ally_box: i32,
    #[serde(default = "default_fallback_speed_edge")]
    pub fallback_speed_edge: i32,
    #[serde(default = "default_atk_enemy_car_duration")]
    pub atk_enemy_car_duration: f64,
    #[serde(default = "default_atk_enemy_box_duration")]
    pub atk_enemy_box_duration: f64,
    #[serde(default = "default_atk_neutral_box_duration")]
    pub atk_neutral_box_duration: f64,
    #[serde(default = "default_fallback_duration_ally")]
    pub fallback_duration_ally_box: f64,
    #[serde(default = "default_fallback_duration_edge")]
    pub fallback_duration_edge: f64,
    #[serde(default = "default_surr_turn_speed")]
    pub turn_speed: i32,
    #[serde(default = "default_turn_left_prob")]
    pub turn_left_prob: f64,
    #[serde(default)]
    pub turn_to_front_use_front_sensor: bool,
    #[serde(default = "default_rand_turn_speeds")]
    pub rand_turn_speeds: Vec<i32>,
    #[serde(default = "default_rand_turn_weights")]
    pub rand_turn_speed_weights: Vec<f64>,
    #[serde(default = "default_full_turn_duration")]
    pub full_turn_duration: f64,
    #[serde(default = "default_half_turn_duration")]
    pub half_turn_duration: f64,
}

fn default_surr_left_adc() -> i32 { 1000 }
fn default_surr_right_adc() -> i32 { 1000 }
fn default_surr_front_adc() -> i32 { 1000 }
fn default_surr_back_adc() -> i32 { 1100 }
fn default_atk_break_front() -> i32 { 1500 }
fn default_atk_break_use_edge() -> bool { true }
fn default_atk_speed_enemy_car() -> i32 { 2300 }
fn default_atk_speed_enemy_box() -> i32 { 2500 }
fn default_atk_speed_neutral_box() -> i32 { 2500 }
fn default_fallback_speed_ally_box() -> i32 { 2900 }
fn default_fallback_speed_edge() -> i32 { 2400 }
fn default_atk_enemy_car_duration() -> f64 { 4.2 }
fn default_atk_enemy_box_duration() -> f64 { 3.6 }
fn default_atk_neutral_box_duration() -> f64 { 3.6 }
fn default_fallback_duration_ally() -> f64 { 0.3 }
fn default_fallback_duration_edge() -> f64 { 0.2 }
fn default_surr_turn_speed() -> i32 { 2900 }
fn default_rand_turn_speeds() -> Vec<i32> { vec![1600, 2100, 3000] }
fn default_rand_turn_weights() -> Vec<f64> { vec![2.0, 3.0, 1.0] }

impl Default for SurroundingConfig {
    fn default() -> Self {
        Self {
            io_encounter_object_value: 0,
            left_adc_lower_threshold: default_surr_left_adc(),
            right_adc_lower_threshold: default_surr_right_adc(),
            front_adc_lower_threshold: default_surr_front_adc(),
            back_adc_lower_threshold: default_surr_back_adc(),
            atk_break_front_lower_threshold: default_atk_break_front(),
            atk_break_use_edge_sensors: default_atk_break_use_edge(),
            atk_speed_enemy_car: default_atk_speed_enemy_car(),
            atk_speed_enemy_box: default_atk_speed_enemy_box(),
            atk_speed_neutral_box: default_atk_speed_neutral_box(),
            fallback_speed_ally_box: default_fallback_speed_ally_box(),
            fallback_speed_edge: default_fallback_speed_edge(),
            atk_enemy_car_duration: default_atk_enemy_car_duration(),
            atk_enemy_box_duration: default_atk_enemy_box_duration(),
            atk_neutral_box_duration: default_atk_neutral_box_duration(),
            fallback_duration_ally_box: default_fallback_duration_ally(),
            fallback_duration_edge: default_fallback_duration_edge(),
            turn_speed: default_surr_turn_speed(),
            turn_left_prob: default_turn_left_prob(),
            turn_to_front_use_front_sensor: false,
            rand_turn_speeds: default_rand_turn_speeds(),
            rand_turn_speed_weights: default_rand_turn_weights(),
            full_turn_duration: default_full_turn_duration(),
            half_turn_duration: default_half_turn_duration(),
        }
    }
}

// ── GradientConfig ─────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GradientConfig {
    #[serde(default = "default_gradient_max_speed")]
    pub max_speed: i32,
    #[serde(default = "default_gradient_min_speed")]
    pub min_speed: i32,
    #[serde(default = "default_gradient_lower")]
    pub lower_bound: i32,
    #[serde(default = "default_gradient_upper")]
    pub upper_bound: i32,
}

fn default_gradient_max_speed() -> i32 { 2800 }
fn default_gradient_min_speed() -> i32 { 500 }
fn default_gradient_lower() -> i32 { 2900 }
fn default_gradient_upper() -> i32 { 3700 }

impl Default for GradientConfig {
    fn default() -> Self {
        Self {
            max_speed: default_gradient_max_speed(),
            min_speed: default_gradient_min_speed(),
            lower_bound: default_gradient_lower(),
            upper_bound: default_gradient_upper(),
        }
    }
}

// ── ScanConfig ─────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScanConfig {
    #[serde(default = "default_tolerance_760")]
    pub front_max_tolerance: i32,
    #[serde(default = "default_tolerance_760")]
    pub rear_max_tolerance: i32,
    #[serde(default = "default_tolerance_760")]
    pub left_max_tolerance: i32,
    #[serde(default = "default_tolerance_760")]
    pub right_max_tolerance: i32,
    #[serde(default)]
    pub io_encounter_object_value: i32,
    #[serde(default = "default_scan_speed")]
    pub scan_speed: i32,
    #[serde(default = "default_scan_duration")]
    pub scan_duration: f64,
    #[serde(default = "default_turn_left_prob")]
    pub scan_turn_left_prob: f64,
    #[serde(default = "default_scan_fallback_speed")]
    pub fall_back_speed: i32,
    #[serde(default = "default_scan_fallback_duration")]
    pub fall_back_duration: f64,
    #[serde(default = "default_scan_turn_speed")]
    pub turn_speed: i32,
    #[serde(default = "default_turn_left_prob")]
    pub turn_left_prob: f64,
    #[serde(default = "default_full_turn_duration")]
    pub full_turn_duration: f64,
    #[serde(default = "default_half_turn_duration")]
    pub half_turn_duration: f64,
    #[serde(default = "default_true")]
    pub check_edge_before_scan: bool,
    #[serde(default = "default_true")]
    pub check_gray_adc_before_scan: bool,
    #[serde(default = "default_gray_adc_lower")]
    pub gray_adc_lower_threshold: i32,
}

fn default_tolerance_760() -> i32 { 760 }
fn default_scan_speed() -> i32 { 300 }
fn default_scan_duration() -> f64 { 4.5 }
fn default_scan_fallback_speed() -> i32 { 3250 }
fn default_scan_fallback_duration() -> f64 { 0.2 }
fn default_scan_turn_speed() -> i32 { 2700 }
fn default_true() -> bool { true }
fn default_gray_adc_lower() -> i32 { 3100 }

impl Default for ScanConfig {
    fn default() -> Self {
        Self {
            front_max_tolerance: default_tolerance_760(),
            rear_max_tolerance: default_tolerance_760(),
            left_max_tolerance: default_tolerance_760(),
            right_max_tolerance: default_tolerance_760(),
            io_encounter_object_value: 0,
            scan_speed: default_scan_speed(),
            scan_duration: default_scan_duration(),
            scan_turn_left_prob: default_turn_left_prob(),
            fall_back_speed: default_scan_fallback_speed(),
            fall_back_duration: default_scan_fallback_duration(),
            turn_speed: default_scan_turn_speed(),
            turn_left_prob: default_turn_left_prob(),
            full_turn_duration: default_full_turn_duration(),
            half_turn_duration: default_half_turn_duration(),
            check_edge_before_scan: default_true(),
            check_gray_adc_before_scan: default_true(),
            gray_adc_lower_threshold: default_gray_adc_lower(),
        }
    }
}

// ── RandTurn ───────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RandTurn {
    #[serde(default = "default_randturn_speed")]
    pub turn_speed: i32,
    #[serde(default = "default_turn_left_prob")]
    pub turn_left_prob: f64,
    #[serde(default = "default_randturn_full_dur")]
    pub full_turn_duration: f64,
    #[serde(default = "default_randturn_half_dur")]
    pub half_turn_duration: f64,
    #[serde(default = "default_true")]
    pub use_turn_to_front: bool,
}

fn default_randturn_speed() -> i32 { 2300 }
fn default_randturn_full_dur() -> f64 { 0.25 }
fn default_randturn_half_dur() -> f64 { 0.15 }

impl Default for RandTurn {
    fn default() -> Self {
        Self {
            turn_speed: default_randturn_speed(),
            turn_left_prob: default_turn_left_prob(),
            full_turn_duration: default_randturn_full_dur(),
            half_turn_duration: default_randturn_half_dur(),
            use_turn_to_front: default_true(),
        }
    }
}

// ── SearchConfig ───────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SearchConfig {
    #[serde(default = "default_true")]
    pub use_gradient_move: bool,
    #[serde(default = "default_gradient_weight")]
    pub gradient_move_weight: f64,
    #[serde(default = "default_true")]
    pub use_scan_move: bool,
    #[serde(default = "default_scan_weight")]
    pub scan_move_weight: f64,
    #[serde(default)]
    pub use_rand_turn: bool,
    #[serde(default = "default_randturn_weight")]
    pub rand_turn_weight: f64,
    #[serde(default)]
    pub gradient_move: GradientConfig,
    #[serde(default)]
    pub scan_move: ScanConfig,
    #[serde(default)]
    pub rand_turn: RandTurn,
}

fn default_gradient_weight() -> f64 { 100.0 }
fn default_scan_weight() -> f64 { 1.96 }
fn default_randturn_weight() -> f64 { 0.05 }

impl Default for SearchConfig {
    fn default() -> Self {
        Self {
            use_gradient_move: default_true(),
            gradient_move_weight: default_gradient_weight(),
            use_scan_move: default_true(),
            scan_move_weight: default_scan_weight(),
            use_rand_turn: false,
            rand_turn_weight: default_randturn_weight(),
            gradient_move: GradientConfig::default(),
            scan_move: ScanConfig::default(),
            rand_turn: RandTurn::default(),
        }
    }
}

// ── RandWalk ───────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RandWalk {
    #[serde(default = "default_true")]
    pub use_straight: bool,
    #[serde(default = "default_straight_weight")]
    pub straight_weight: f64,
    #[serde(default = "default_rand_straight_speeds")]
    pub rand_straight_speeds: Vec<i32>,
    #[serde(default = "default_rand_straight_weights")]
    pub rand_straight_speed_weights: Vec<f64>,
    #[serde(default = "default_true")]
    pub use_turn: bool,
    #[serde(default = "default_one")]
    pub turn_weight: f64,
    #[serde(default = "default_rand_turn_speeds2")]
    pub rand_turn_speeds: Vec<i32>,
    #[serde(default = "default_rand_turn_weights2")]
    pub rand_turn_speed_weights: Vec<f64>,
    #[serde(default = "default_walk_duration")]
    pub walk_duration: f64,
}

fn default_straight_weight() -> f64 { 2.0 }
fn default_one() -> f64 { 1.0 }
fn default_rand_straight_speeds() -> Vec<i32> { vec![-800, -500, 500, 800] }
fn default_rand_straight_weights() -> Vec<f64> { vec![1.0, 3.0, 3.0, 1.0] }
fn default_rand_turn_speeds2() -> Vec<i32> { vec![-1200, -800, 800, 1200] }
fn default_rand_turn_weights2() -> Vec<f64> { vec![1.0, 3.0, 3.0, 1.0] }
fn default_walk_duration() -> f64 { 0.3 }

impl Default for RandWalk {
    fn default() -> Self {
        Self {
            use_straight: default_true(),
            straight_weight: default_straight_weight(),
            rand_straight_speeds: default_rand_straight_speeds(),
            rand_straight_speed_weights: default_rand_straight_weights(),
            use_turn: default_true(),
            turn_weight: default_one(),
            rand_turn_speeds: default_rand_turn_speeds2(),
            rand_turn_speed_weights: default_rand_turn_weights2(),
            walk_duration: default_walk_duration(),
        }
    }
}

// ── FenceConfig ────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FenceConfig {
    #[serde(default = "default_fence_front")]
    pub front_adc_lower_threshold: i32,
    #[serde(default = "default_fence_rear")]
    pub rear_adc_lower_threshold: i32,
    #[serde(default = "default_fence_left")]
    pub left_adc_lower_threshold: i32,
    #[serde(default = "default_fence_right")]
    pub right_adc_lower_threshold: i32,
    #[serde(default)]
    pub io_encounter_fence_value: i32,
    #[serde(default = "default_yaw_tolerance")]
    pub max_yaw_tolerance: f64,
    #[serde(default)]
    pub use_mpu_align_stage: bool,
    #[serde(default)]
    pub use_mpu_align_direction: bool,
    #[serde(default = "default_stage_align_speed")]
    pub stage_align_speed: i32,
    #[serde(default = "default_max_stage_align_dur")]
    pub max_stage_align_duration: f64,
    #[serde(default)]
    pub stage_align_direction: String,
    #[serde(default = "default_direction_align_speed")]
    pub direction_align_speed: i32,
    #[serde(default = "default_max_direction_align_dur")]
    pub max_direction_align_duration: f64,
    #[serde(default)]
    pub direction_align_direction: String,
    #[serde(default = "default_exit_corner_speed")]
    pub exit_corner_speed: i32,
    #[serde(default = "default_max_exit_corner_dur")]
    pub max_exit_corner_duration: f64,
    #[serde(default)]
    pub rand_walk: RandWalk,
}

fn default_fence_front() -> i32 { 900 }
fn default_fence_rear() -> i32 { 1100 }
fn default_fence_left() -> i32 { 900 }
fn default_fence_right() -> i32 { 900 }
fn default_yaw_tolerance() -> f64 { 20.0 }
fn default_stage_align_speed() -> i32 { 850 }
fn default_max_stage_align_dur() -> f64 { 4.5 }
fn default_direction_align_speed() -> i32 { 850 }
fn default_max_direction_align_dur() -> f64 { 4.5 }
fn default_exit_corner_speed() -> i32 { 1200 }
fn default_max_exit_corner_dur() -> f64 { 1.5 }

impl Default for FenceConfig {
    fn default() -> Self {
        Self {
            front_adc_lower_threshold: default_fence_front(),
            rear_adc_lower_threshold: default_fence_rear(),
            left_adc_lower_threshold: default_fence_left(),
            right_adc_lower_threshold: default_fence_right(),
            io_encounter_fence_value: 0,
            max_yaw_tolerance: default_yaw_tolerance(),
            use_mpu_align_stage: false,
            use_mpu_align_direction: false,
            stage_align_speed: default_stage_align_speed(),
            max_stage_align_duration: default_max_stage_align_dur(),
            stage_align_direction: "rand".into(),
            direction_align_speed: default_direction_align_speed(),
            max_direction_align_duration: default_max_direction_align_dur(),
            direction_align_direction: "rand".into(),
            exit_corner_speed: default_exit_corner_speed(),
            max_exit_corner_duration: default_max_exit_corner_dur(),
            rand_walk: RandWalk::default(),
        }
    }
}

// ── Strategy / Perf / Boot / BackStage / Stage ──────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StrategyConfig {
    #[serde(default = "default_true")]
    pub use_edge_component: bool,
    #[serde(default = "default_true")]
    pub use_surrounding_component: bool,
    #[serde(default = "default_true")]
    pub use_normal_component: bool,
}

impl Default for StrategyConfig {
    fn default() -> Self {
        Self {
            use_edge_component: true,
            use_surrounding_component: true,
            use_normal_component: true,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceConfig {
    #[serde(default)]
    pub checking_duration: f64,
}

impl Default for PerformanceConfig {
    fn default() -> Self {
        Self { checking_duration: 0.0 }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BootConfig {
    #[serde(default)]
    pub button_io_activate_case_value: i32,
    #[serde(default = "default_boot_stabilize")]
    pub time_to_stabilize: f64,
    #[serde(default = "default_max_holding")]
    pub max_holding_duration: f64,
    #[serde(default = "default_boot_threshold")]
    pub left_threshold: i32,
    #[serde(default = "default_boot_threshold")]
    pub right_threshold: i32,
    #[serde(default = "default_boot_dash_speed")]
    pub dash_speed: i32,
    #[serde(default = "default_boot_dash_duration")]
    pub dash_duration: f64,
    #[serde(default = "default_boot_turn_speed")]
    pub turn_speed: i32,
    #[serde(default = "default_full_turn_duration")]
    pub full_turn_duration: f64,
    #[serde(default = "default_turn_left_prob")]
    pub turn_left_prob: f64,
}

fn default_boot_stabilize() -> f64 { 0.1 }
fn default_max_holding() -> f64 { 180.0 }
fn default_boot_threshold() -> i32 { 1100 }
fn default_boot_dash_speed() -> i32 { 7000 }
fn default_boot_dash_duration() -> f64 { 0.55 }
fn default_boot_turn_speed() -> i32 { 2150 }

impl Default for BootConfig {
    fn default() -> Self {
        Self {
            button_io_activate_case_value: 0,
            time_to_stabilize: default_boot_stabilize(),
            max_holding_duration: default_max_holding(),
            left_threshold: default_boot_threshold(),
            right_threshold: default_boot_threshold(),
            dash_speed: default_boot_dash_speed(),
            dash_duration: default_boot_dash_duration(),
            turn_speed: default_boot_turn_speed(),
            full_turn_duration: default_full_turn_duration(),
            turn_left_prob: default_turn_left_prob(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BackStageConfig {
    #[serde(default = "default_boot_stabilize")]
    pub time_to_stabilize: f64,
    #[serde(default = "default_backstage_advance_speed")]
    pub small_advance_speed: i32,
    #[serde(default = "default_backstage_advance_dur")]
    pub small_advance_duration: f64,
    #[serde(default = "default_boot_dash_speed")]
    pub dash_speed: i32,
    #[serde(default = "default_boot_dash_duration")]
    pub dash_duration: f64,
    #[serde(default = "default_backstage_turn_speed")]
    pub turn_speed: i32,
    #[serde(default = "default_backstage_full_turn")]
    pub full_turn_duration: f64,
    #[serde(default = "default_turn_left_prob")]
    pub turn_left_prob: f64,
    #[serde(default = "default_true")]
    pub use_is_on_stage_check: bool,
    #[serde(default = "default_true")]
    pub use_side_away_check: bool,
    #[serde(default = "default_check_start_percent")]
    pub check_start_percent: f64,
    #[serde(default = "default_side_away_tolerance")]
    pub side_away_degree_tolerance: f64,
    #[serde(default = "default_exit_side_away_speed")]
    pub exit_side_away_speed: i32,
    #[serde(default = "default_exit_side_away_dur")]
    pub exit_side_away_duration: f64,
}

fn default_backstage_advance_speed() -> i32 { 1500 }
fn default_backstage_advance_dur() -> f64 { 0.6 }
fn default_backstage_turn_speed() -> i32 { 2600 }
fn default_backstage_full_turn() -> f64 { 0.35 }
fn default_check_start_percent() -> f64 { 0.9 }
fn default_side_away_tolerance() -> f64 { 10.0 }
fn default_exit_side_away_speed() -> i32 { 1300 }
fn default_exit_side_away_dur() -> f64 { 0.6 }

impl Default for BackStageConfig {
    fn default() -> Self {
        Self {
            time_to_stabilize: default_boot_stabilize(),
            small_advance_speed: default_backstage_advance_speed(),
            small_advance_duration: default_backstage_advance_dur(),
            dash_speed: default_boot_dash_speed(),
            dash_duration: default_boot_dash_duration(),
            turn_speed: default_backstage_turn_speed(),
            full_turn_duration: default_backstage_full_turn(),
            turn_left_prob: default_turn_left_prob(),
            use_is_on_stage_check: true,
            use_side_away_check: true,
            check_start_percent: default_check_start_percent(),
            side_away_degree_tolerance: default_side_away_tolerance(),
            exit_side_away_speed: default_exit_side_away_speed(),
            exit_side_away_duration: default_exit_side_away_dur(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StageConfig {
    #[serde(default = "default_stage_off_upper")]
    pub gray_adc_off_stage_upper_threshold: i32,
    #[serde(default = "default_stage_on_lower")]
    pub gray_adc_on_stage_lower_threshold: i32,
    #[serde(default = "default_unclear_tolerance")]
    pub unclear_zone_tolerance: i32,
    #[serde(default = "default_unclear_turn_speed")]
    pub unclear_zone_turn_speed: i32,
    #[serde(default = "default_unclear_turn_dur")]
    pub unclear_zone_turn_duration: f64,
    #[serde(default = "default_turn_left_prob")]
    pub unclear_zone_turn_left_prob: f64,
    #[serde(default)]
    pub gray_io_off_stage_case_value: i32,
}

fn default_stage_off_upper() -> i32 { 2630 }
fn default_stage_on_lower() -> i32 { 2830 }
fn default_unclear_tolerance() -> i32 { 90 }
fn default_unclear_turn_speed() -> i32 { 1500 }
fn default_unclear_turn_dur() -> f64 { 0.6 }

impl Default for StageConfig {
    fn default() -> Self {
        Self {
            gray_adc_off_stage_upper_threshold: default_stage_off_upper(),
            gray_adc_on_stage_lower_threshold: default_stage_on_lower(),
            unclear_zone_tolerance: default_unclear_tolerance(),
            unclear_zone_turn_speed: default_unclear_turn_speed(),
            unclear_zone_turn_duration: default_unclear_turn_dur(),
            unclear_zone_turn_left_prob: default_turn_left_prob(),
            gray_io_off_stage_case_value: 0,
        }
    }
}

// ── Sensor / Motion / Vision / Debug configs ───────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorConfig {
    #[serde(default = "default_gyro_fsr")]
    pub gyro_fsr: u16,
    #[serde(default = "default_accel_fsr")]
    pub accel_fsr: u8,
    #[serde(default = "default_adc_interval")]
    pub adc_min_sample_interval: i32,
    // Edge sensor indices
    #[serde(default = "default_edge_fl")]
    pub edge_fl_index: i32,
    #[serde(default = "default_edge_fr")]
    pub edge_fr_index: i32,
    #[serde(default = "default_edge_rl")]
    pub edge_rl_index: i32,
    #[serde(default = "default_edge_rr")]
    pub edge_rr_index: i32,
    // ADC indices
    #[serde(default = "default_left_adc")]
    pub left_adc_index: i32,
    #[serde(default = "default_right_adc")]
    pub right_adc_index: i32,
    #[serde(default = "default_front_adc")]
    pub front_adc_index: i32,
    #[serde(default = "default_rb_adc")]
    pub rb_adc_index: i32,
    #[serde(default = "default_gray_adc")]
    pub gray_adc_index: i32,
    // IO indices
    #[serde(default = "default_gray_io_left")]
    pub gray_io_left_index: i32,
    #[serde(default = "default_gray_io_right")]
    pub gray_io_right_index: i32,
    #[serde(default = "default_fl_io")]
    pub fl_io_index: i32,
    #[serde(default = "default_fr_io")]
    pub fr_io_index: i32,
    #[serde(default = "default_rl_io")]
    pub rl_io_index: i32,
    #[serde(default = "default_rr_io")]
    pub rr_io_index: i32,
    #[serde(default = "default_reboot_btn")]
    pub reboot_button_index: i32,
}

fn default_gyro_fsr() -> u16 { 1000 }
fn default_accel_fsr() -> u8 { 8 }
fn default_adc_interval() -> i32 { 5 }
fn default_edge_fl() -> i32 { 3 }
fn default_edge_fr() -> i32 { 0 }
fn default_edge_rl() -> i32 { 2 }
fn default_edge_rr() -> i32 { 1 }
fn default_left_adc() -> i32 { 6 }
fn default_right_adc() -> i32 { 4 }
fn default_front_adc() -> i32 { 5 }
fn default_rb_adc() -> i32 { 7 }
fn default_gray_adc() -> i32 { 8 }
fn default_gray_io_left() -> i32 { 1 }
fn default_gray_io_right() -> i32 { 0 }
fn default_fl_io() -> i32 { 5 }
fn default_fr_io() -> i32 { 2 }
fn default_rl_io() -> i32 { 4 }
fn default_rr_io() -> i32 { 3 }
fn default_reboot_btn() -> i32 { 6 }

impl Default for SensorConfig {
    fn default() -> Self {
        Self {
            gyro_fsr: default_gyro_fsr(),
            accel_fsr: default_accel_fsr(),
            adc_min_sample_interval: default_adc_interval(),
            edge_fl_index: default_edge_fl(),
            edge_fr_index: default_edge_fr(),
            edge_rl_index: default_edge_rl(),
            edge_rr_index: default_edge_rr(),
            left_adc_index: default_left_adc(),
            right_adc_index: default_right_adc(),
            front_adc_index: default_front_adc(),
            rb_adc_index: default_rb_adc(),
            gray_adc_index: default_gray_adc(),
            gray_io_left_index: default_gray_io_left(),
            gray_io_right_index: default_gray_io_right(),
            fl_io_index: default_fl_io(),
            fr_io_index: default_fr_io(),
            rl_io_index: default_rl_io(),
            rr_io_index: default_rr_io(),
            reboot_button_index: default_reboot_btn(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotionConfig {
    #[serde(default = "default_motor_fr")]
    pub motor_fr: (i32, i32),
    #[serde(default = "default_motor_fl")]
    pub motor_fl: (i32, i32),
    #[serde(default = "default_motor_rr")]
    pub motor_rr: (i32, i32),
    #[serde(default = "default_motor_rl")]
    pub motor_rl: (i32, i32),
    #[serde(default = "default_port_str")]
    pub port: String,
}

fn default_motor_fr() -> (i32, i32) { (1, 1) }
fn default_motor_fl() -> (i32, i32) { (2, 1) }
fn default_motor_rr() -> (i32, i32) { (3, 1) }
fn default_motor_rl() -> (i32, i32) { (4, 1) }
fn default_port_str() -> String { "/dev/ttyUSB0".into() }

impl Default for MotionConfig {
    fn default() -> Self {
        Self {
            motor_fr: default_motor_fr(),
            motor_fl: default_motor_fl(),
            motor_rr: default_motor_rr(),
            motor_rl: default_motor_rl(),
            port: default_port_str(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisionConfig {
    #[serde(default = "default_team_str")]
    pub team_color: String,
    #[serde(default = "default_one_f64")]
    pub resolution_multiplier: f64,
    #[serde(default = "default_true")]
    pub use_camera: bool,
    #[serde(default)]
    pub camera_device_id: i32,
}

fn default_team_str() -> String { "blue".into() }
fn default_one_f64() -> f64 { 1.0 }

impl Default for VisionConfig {
    fn default() -> Self {
        Self {
            team_color: default_team_str(),
            resolution_multiplier: default_one_f64(),
            use_camera: true,
            camera_device_id: 0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DebugConfig {
    #[serde(default = "default_log_level_str")]
    pub log_level: String,
    #[serde(default = "default_true")]
    pub use_siglight: bool,
}

fn default_log_level_str() -> String { "INFO".into() }

impl Default for DebugConfig {
    fn default() -> Self {
        Self {
            log_level: default_log_level_str(),
            use_siglight: true,
        }
    }
}

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
