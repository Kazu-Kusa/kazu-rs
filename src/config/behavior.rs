use serde::{Deserialize, Serialize};

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
