use serde::{Deserialize, Serialize};

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
fn default_turn_left_prob() -> f64 { 0.5 }
fn default_full_turn_duration() -> f64 { 0.45 }
fn default_half_turn_duration() -> f64 { 0.225 }

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
