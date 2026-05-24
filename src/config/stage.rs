use serde::{Deserialize, Serialize};

// ── Shared helpers ─────────────────────────────────────────────

fn default_true() -> bool { true }
fn default_turn_left_prob() -> f64 { 0.5 }
fn default_full_turn_duration() -> f64 { 0.45 }
fn default_boot_stabilize() -> f64 { 0.1 }
fn default_boot_dash_speed() -> i32 { 7000 }
fn default_boot_dash_duration() -> f64 { 0.55 }

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

fn default_max_holding() -> f64 { 180.0 }
fn default_boot_threshold() -> i32 { 1100 }
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
