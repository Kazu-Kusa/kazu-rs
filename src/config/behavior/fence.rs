use serde::{Deserialize, Serialize};

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

fn default_true() -> bool {
    true
}
fn default_straight_weight() -> f64 {
    2.0
}
fn default_one() -> f64 {
    1.0
}
fn default_rand_straight_speeds() -> Vec<i32> {
    vec![-800, -500, 500, 800]
}
fn default_rand_straight_weights() -> Vec<f64> {
    vec![1.0, 3.0, 3.0, 1.0]
}
fn default_rand_turn_speeds2() -> Vec<i32> {
    vec![-1200, -800, 800, 1200]
}
fn default_rand_turn_weights2() -> Vec<f64> {
    vec![1.0, 3.0, 3.0, 1.0]
}
fn default_walk_duration() -> f64 {
    0.3
}

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

fn default_fence_front() -> i32 {
    900
}
fn default_fence_rear() -> i32 {
    1100
}
fn default_fence_left() -> i32 {
    900
}
fn default_fence_right() -> i32 {
    900
}
fn default_yaw_tolerance() -> f64 {
    20.0
}
fn default_stage_align_speed() -> i32 {
    850
}
fn default_max_stage_align_dur() -> f64 {
    4.5
}
fn default_direction_align_speed() -> i32 {
    850
}
fn default_max_direction_align_dur() -> f64 {
    4.5
}
fn default_exit_corner_speed() -> i32 {
    1200
}
fn default_max_exit_corner_dur() -> f64 {
    1.5
}

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
