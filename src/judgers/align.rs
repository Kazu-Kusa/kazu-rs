use crate::config::{AppConfig, RunConfig};
use mentabotix_rs::transition::BreakerResult;
use std::sync::Arc;
use super::SensorData;

pub(crate) fn make_std_stage_align_breaker(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let front_adc_idx = app_config.sensor.front_adc_index as usize;
    let rear_adc_idx = app_config.sensor.rb_adc_index as usize;
    let front_threshold = run_config.surrounding.front_adc_lower_threshold as f64;
    let rear_threshold = run_config.surrounding.back_adc_lower_threshold as f64;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let adc = sensor.adc_all();
        let f = *adc.get(front_adc_idx).unwrap_or(&0.0);
        let r = *adc.get(rear_adc_idx).unwrap_or(&0.0);
        BreakerResult::Bool(f > front_threshold || r > rear_threshold)
    })
}

#[allow(dead_code)]
pub(crate) fn make_stage_align_breaker_mpu(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let front_adc_idx = app_config.sensor.front_adc_index as usize;
    let front_threshold = run_config.surrounding.front_adc_lower_threshold as f64;
    let yaw_tolerance = run_config.fence.max_yaw_tolerance;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let adc = sensor.adc_all();
        let f = *adc.get(front_adc_idx).unwrap_or(&0.0);
        let yaw = sensor.mpu_yaw();
        let aligned = yaw.abs() < yaw_tolerance;
        BreakerResult::Bool(f > front_threshold || aligned)
    })
}

#[allow(dead_code)]
pub(crate) fn make_align_direction_breaker_mpu(
    sensor: &Arc<dyn SensorData>,
    _app_config: &AppConfig,
    run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let yaw_tol = run_config.fence.max_yaw_tolerance;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let yaw = sensor.mpu_yaw();
        BreakerResult::Bool(yaw.abs() < yaw_tol)
    })
}

pub(crate) fn make_std_align_direction_breaker(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let front_th = run_config.surrounding.front_adc_lower_threshold as f64;
    let back_th = run_config.surrounding.back_adc_lower_threshold as f64;
    let front_idx = app_config.sensor.front_adc_index as usize;
    let back_idx = app_config.sensor.rb_adc_index as usize;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let adc = sensor.adc_all();
        let f = *adc.get(front_idx).unwrap_or(&0.0);
        let b = *adc.get(back_idx).unwrap_or(&0.0);
        BreakerResult::Bool(f > front_th || b > back_th)
    })
}
