//! Breaker predicate factories — port of `kazu/judgers.py`.
//!
//! Each method on [`Breakers`] returns a closure suitable for
//! [`MovingTransition::with_breaker`] that evaluates sensor data against
//! configured thresholds and returns a [`BreakerResult`].

mod align;
mod edge;
mod fence;
mod misc;
mod stage;
mod surrounding;

use crate::config::{AppConfig, RunConfig, TagGroup};
use mentabotix_rs::transition::BreakerResult;
use std::sync::Arc;

// ── Sampler trait ─────────────────────────────────────────────

/// Abstract sensor data access — decouples Breakers from specific hardware.
pub trait SensorData: Send + Sync {
    /// Read all ADC channels (10 values, 0–4095).
    fn adc_all(&self) -> Vec<f64>;
    /// Read all IO channels (8 values, 0/1).
    fn io_all(&self) -> Vec<f64>;
    /// Read MPU Yaw (degrees).
    // Blocked on: MPU6500 hardware via uptechstar-rs.
    #[allow(dead_code)]
    fn mpu_yaw(&self) -> f64;
}

/// A no-op sensor source returning zeros — for graph viz / offline use.
pub struct NullSensor;

impl SensorData for NullSensor {
    fn adc_all(&self) -> Vec<f64> {
        vec![0.0; 10]
    }
    fn io_all(&self) -> Vec<f64> {
        vec![0.0; 8]
    }
    fn mpu_yaw(&self) -> f64 {
        0.0
    }
}

// ── Breakers ──────────────────────────────────────────────────

/// Factory for breaker closures.
///
/// Holds shared sensor access and configuration so that closures
/// can be `'static`.
pub struct Breakers {
    sensor: Arc<dyn SensorData>,
}

impl Breakers {
    /// Create a new Breakers factory with the given sensor data source.
    pub fn new(sensor: Arc<dyn SensorData>) -> Self {
        Self { sensor }
    }

    /// Convenience: create with null sensor for offline/graph use.
    pub fn null() -> Self {
        Self::new(Arc::new(NullSensor))
    }

    // ── Edge breakers ──────────────────────────────────────────

    /// Rear edge detector: checks RL + RR ADC against thresholds.
    pub fn make_std_edge_rear_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        edge::make_std_edge_rear_breaker(&self.sensor, app_config, run_config)
    }

    /// Front edge detector: checks gray IO + FL/FR ADC.
    pub fn make_std_edge_front_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        edge::make_std_edge_front_breaker(&self.sensor, app_config, run_config)
    }

    /// Full edge detector: returns an EdgeCodeSign discriminant (int).
    /// FL=1, FR=2, RL=4, RR=8.
    pub fn make_std_edge_full_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        edge::make_std_edge_full_breaker(&self.sensor, app_config, run_config)
    }

    // ── Attack / Surrounding breakers ──────────────────────────

    /// Stop-turning breaker: checks front obstacles (ADC + IO).
    pub fn make_std_turn_to_front_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        surrounding::make_std_turn_to_front_breaker(&self.sensor, app_config, run_config)
    }

    /// Standard attack breaker: front obstacle + edge.
    pub fn make_std_atk_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        surrounding::make_std_atk_breaker(&self.sensor, app_config, run_config)
    }

    /// Attack breaker using edge sensors directly.
    pub fn make_atk_breaker_with_edge_sensors(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        surrounding::make_atk_breaker_with_edge_sensors(&self.sensor, app_config, run_config)
    }

    /// Surrounding obstacle detector: returns SurroundingCodeSign discriminant (int).
    pub fn make_surr_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
        tag_group: &TagGroup,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        surrounding::make_surr_breaker(&self.sensor, app_config, run_config, tag_group)
    }

    // ── Fence breaker ──────────────────────────────────────────

    /// Full fence breaker: returns FenceCodeSign discriminant (int).
    pub fn make_std_fence_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        fence::make_std_fence_breaker(&self.sensor, app_config, run_config)
    }

    /// Left-right sides blocked breaker.
    pub fn make_lr_sides_blocked_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        fence::make_lr_sides_blocked_breaker(&self.sensor, app_config, run_config)
    }

    // ── Alignment breakers ─────────────────────────────────────

    /// Stage alignment breaker: checks front + rear obstacles.
    pub fn make_std_stage_align_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        align::make_std_stage_align_breaker(&self.sensor, app_config, run_config)
    }

    /// MPU-based stage alignment breaker.
    // Blocked on: MPU6500 hardware.
    #[allow(dead_code)]
    pub fn make_stage_align_breaker_mpu(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        align::make_stage_align_breaker_mpu(&self.sensor, app_config, run_config)
    }

    /// MPU-based direction alignment breaker.
    // Blocked on: MPU6500 hardware.
    #[allow(dead_code)]
    pub fn make_align_direction_breaker_mpu(
        &self,
        _app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        align::make_align_direction_breaker_mpu(&self.sensor, _app_config, run_config)
    }

    /// Standard (non-MPU) direction alignment breaker.
    pub fn make_std_align_direction_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        align::make_std_align_direction_breaker(&self.sensor, app_config, run_config)
    }

    // ── Stage breakers ─────────────────────────────────────────

    /// Standard stage breaker: returns StageCodeSign discriminant (int).
    ///
    /// Uses weighted-sum logic (StageWeight: STAGE=1, REBOOT=2, UNCLEAR=4):
    ///   0 = ON_STAGE, 1 = OFF_STAGE, 2 = ON+REBOOT, 3 = OFF+REBOOT,
    ///   4 = UNCLEAR, 6 = UNCLEAR+REBOOT
    pub fn make_std_stage_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        stage::make_std_stage_breaker(&self.sensor, app_config, run_config)
    }

    /// Always-on stage breaker: always returns ON_STAGE (0).
    pub fn make_always_on_stage_breaker(
        &self,
        _app_config: &AppConfig,
        _run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        stage::make_always_on_stage_breaker(_app_config, _run_config)
    }

    /// Always-off stage breaker: always returns OFF_STAGE (1).
    pub fn make_always_off_stage_breaker(
        &self,
        _app_config: &AppConfig,
        _run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        stage::make_always_off_stage_breaker(_app_config, _run_config)
    }

    /// On-stage checker using gray ADC.
    pub fn make_is_on_stage_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        stage::make_is_on_stage_breaker(&self.sensor, app_config, run_config)
    }

    // ── Misc breakers ──────────────────────────────────────────

    /// Standard scan breaker: returns ScanCodeSign discriminant (int).
    pub fn make_std_scan_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        misc::make_std_scan_breaker(&self.sensor, app_config, run_config)
    }

    /// Reboot button pressed detector.
    pub fn make_reboot_button_pressed_breaker(
        &self,
        app_config: &AppConfig,
        _run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        misc::make_reboot_button_pressed_breaker(&self.sensor, app_config, _run_config)
    }

    /// Gray ADC check for scan.
    pub fn make_check_gray_adc_for_scan_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        misc::make_check_gray_adc_for_scan_breaker(&self.sensor, app_config, run_config)
    }

    /// Back-stage side-away detector.
    pub fn make_back_stage_side_away_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        misc::make_back_stage_side_away_breaker(&self.sensor, app_config, run_config)
    }
}
