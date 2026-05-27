use serde::{Deserialize, Serialize};

// ── Shared helpers ─────────────────────────────────────────────

fn default_true() -> bool {
    true
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

fn default_gyro_fsr() -> u16 {
    1000
}
fn default_accel_fsr() -> u8 {
    8
}
fn default_adc_interval() -> i32 {
    5
}
fn default_edge_fl() -> i32 {
    3
}
fn default_edge_fr() -> i32 {
    0
}
fn default_edge_rl() -> i32 {
    2
}
fn default_edge_rr() -> i32 {
    1
}
fn default_left_adc() -> i32 {
    6
}
fn default_right_adc() -> i32 {
    4
}
fn default_front_adc() -> i32 {
    5
}
fn default_rb_adc() -> i32 {
    7
}
fn default_gray_adc() -> i32 {
    8
}
fn default_gray_io_left() -> i32 {
    1
}
fn default_gray_io_right() -> i32 {
    0
}
fn default_fl_io() -> i32 {
    5
}
fn default_fr_io() -> i32 {
    2
}
fn default_rl_io() -> i32 {
    4
}
fn default_rr_io() -> i32 {
    3
}
fn default_reboot_btn() -> i32 {
    6
}

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

fn default_motor_fr() -> (i32, i32) {
    (1, 1)
}
fn default_motor_fl() -> (i32, i32) {
    (2, 1)
}
fn default_motor_rr() -> (i32, i32) {
    (3, 1)
}
fn default_motor_rl() -> (i32, i32) {
    (4, 1)
}
fn default_port_str() -> String {
    "/dev/ttyUSB0".into()
}

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

fn default_team_str() -> String {
    "blue".into()
}
fn default_one_f64() -> f64 {
    1.0
}

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

fn default_log_level_str() -> String {
    "INFO".into()
}

impl Default for DebugConfig {
    fn default() -> Self {
        Self {
            log_level: default_log_level_str(),
            use_siglight: true,
        }
    }
}
