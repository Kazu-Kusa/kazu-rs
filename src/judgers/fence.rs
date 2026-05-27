use super::SensorData;
use crate::config::{AppConfig, RunConfig};
use mentabotix_rs::transition::BreakerResult;
use std::sync::Arc;

pub(crate) fn make_std_fence_breaker(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let f_th = run_config.fence.front_adc_lower_threshold as f64;
    let r_th = run_config.fence.rear_adc_lower_threshold as f64;
    let l_th = run_config.fence.left_adc_lower_threshold as f64;
    let ri_th = run_config.fence.right_adc_lower_threshold as f64;
    let f_idx = app_config.sensor.front_adc_index as usize;
    let r_idx = app_config.sensor.rb_adc_index as usize;
    let l_idx = app_config.sensor.left_adc_index as usize;
    let ri_idx = app_config.sensor.right_adc_index as usize;
    let io_fence_val = run_config.fence.io_encounter_fence_value as f64;
    let fl_io = app_config.sensor.fl_io_index as usize;
    let fr_io = app_config.sensor.fr_io_index as usize;
    let rl_io = app_config.sensor.rl_io_index as usize;
    let rr_io = app_config.sensor.rr_io_index as usize;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let adc = sensor.adc_all();
        let io = sensor.io_all();
        let front = *adc.get(f_idx).unwrap_or(&0.0) > f_th
            || *io.get(fl_io).unwrap_or(&1.0) == io_fence_val
            || *io.get(fr_io).unwrap_or(&1.0) == io_fence_val;
        let rear = *adc.get(r_idx).unwrap_or(&0.0) > r_th
            || *io.get(rl_io).unwrap_or(&1.0) == io_fence_val
            || *io.get(rr_io).unwrap_or(&1.0) == io_fence_val;
        let left = *adc.get(l_idx).unwrap_or(&0.0) > l_th;
        let right = *adc.get(ri_idx).unwrap_or(&0.0) > ri_th;

        let code = (front as i32) + (rear as i32) * 2 + (left as i32) * 4 + (right as i32) * 8;
        BreakerResult::Int(code as i64)
    })
}

pub(crate) fn make_lr_sides_blocked_breaker(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let l_th = run_config.fence.left_adc_lower_threshold as f64;
    let r_th = run_config.fence.right_adc_lower_threshold as f64;
    let l_idx = app_config.sensor.left_adc_index as usize;
    let r_idx = app_config.sensor.right_adc_index as usize;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let adc = sensor.adc_all();
        let l = *adc.get(l_idx).unwrap_or(&0.0) > l_th;
        let r = *adc.get(r_idx).unwrap_or(&0.0) > r_th;
        BreakerResult::Bool(l && r)
    })
}
