use std::time::Duration;

/// Tag selection method for when multiple tags are detected
#[derive(Debug, Clone, Copy)]
pub enum OrderingMethod {
    /// Select the tag nearest to the frame center
    Nearest,
    /// Select the first detected tag
    Single,
}

impl Default for OrderingMethod {
    fn default() -> Self {
        OrderingMethod::Nearest
    }
}

/// Configuration parameters for TagDetector behavior
#[derive(Debug, Clone)]
pub struct Config {
    /// Whether to operate in single tag detection mode
    pub single_tag_mode: bool,
    /// Multiplier for camera resolution scaling
    pub resolution_multiplier: f64,
    /// Method for selecting tags when multiple are detected
    pub ordering_method: OrderingMethod,
    /// Time interval between halt status checks during detection loop
    pub halt_check_interval: Duration,
    /// Tag ID returned when no tags are detected
    pub default_tag_id: i32,
    /// Tag ID returned when camera errors occur
    pub error_tag_id: i32,
    /// Camera buffer size for real-time performance
    pub buffer_size: i32,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            single_tag_mode: true,
            resolution_multiplier: 0.5,
            ordering_method: OrderingMethod::Nearest,
            halt_check_interval: Duration::from_millis(400),
            default_tag_id: -1,
            error_tag_id: -10,
            buffer_size: 2,
        }
    }
}
