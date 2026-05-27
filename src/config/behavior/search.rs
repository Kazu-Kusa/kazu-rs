use serde::{Deserialize, Serialize};

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

fn default_gradient_max_speed() -> i32 {
    2800
}
fn default_gradient_min_speed() -> i32 {
    500
}
fn default_gradient_lower() -> i32 {
    2900
}
fn default_gradient_upper() -> i32 {
    3700
}

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

fn default_tolerance_760() -> i32 {
    760
}
fn default_scan_speed() -> i32 {
    300
}
fn default_scan_duration() -> f64 {
    4.5
}
fn default_scan_fallback_speed() -> i32 {
    3250
}
fn default_scan_fallback_duration() -> f64 {
    0.2
}
fn default_scan_turn_speed() -> i32 {
    2700
}
fn default_gray_adc_lower() -> i32 {
    3100
}
fn default_turn_left_prob() -> f64 {
    0.5
}
fn default_full_turn_duration() -> f64 {
    0.45
}
fn default_half_turn_duration() -> f64 {
    0.225
}
fn default_true() -> bool {
    true
}

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

fn default_randturn_speed() -> i32 {
    2300
}
fn default_randturn_full_dur() -> f64 {
    0.25
}
fn default_randturn_half_dur() -> f64 {
    0.15
}

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

fn default_gradient_weight() -> f64 {
    100.0
}
fn default_scan_weight() -> f64 {
    1.96
}
fn default_randturn_weight() -> f64 {
    0.05
}

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
