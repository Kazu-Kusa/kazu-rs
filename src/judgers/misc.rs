use crate::config::{AppConfig, RunConfig};
use mentabotix_rs::transition::BreakerResult;
use std::sync::Arc;
use super::SensorData;

pub(crate) fn make_std_scan_breaker(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let front_tol = run_config.search.scan_move.front_max_tolerance as f64;
    let rear_tol = run_config.search.scan_move.rear_max_tolerance as f64;
    let left_tol = run_config.search.scan_move.left_max_tolerance as f64;
    let right_tol = run_config.search.scan_move.right_max_tolerance as f64;
    let front_idx = app_config.sensor.front_adc_index as usize;
    let rear_idx = app_config.sensor.rb_adc_index as usize;
    let left_idx = app_config.sensor.left_adc_index as usize;
    let right_idx = app_config.sensor.right_adc_index as usize;
    let io_val = run_config.search.scan_move.io_encounter_object_value as f64;
    let fl_io = app_config.sensor.fl_io_index as usize;
    let fr_io = app_config.sensor.fr_io_index as usize;
    let rl_io = app_config.sensor.rl_io_index as usize;
    let rr_io = app_config.sensor.rr_io_index as usize;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let adc = sensor.adc_all();
        let io = sensor.io_all();
        let front = *adc.get(front_idx).unwrap_or(&0.0) > front_tol
            || *io.get(fl_io).unwrap_or(&1.0) == io_val
            || *io.get(fr_io).unwrap_or(&1.0) == io_val;
        let rear = *adc.get(rear_idx).unwrap_or(&0.0) > rear_tol
            || *io.get(rl_io).unwrap_or(&1.0) == io_val
            || *io.get(rr_io).unwrap_or(&1.0) == io_val;
        let left = *adc.get(left_idx).unwrap_or(&0.0) > left_tol;
        let right = *adc.get(right_idx).unwrap_or(&0.0) > right_tol;
        let code = (front as i32)
            + (rear as i32) * 2
            + (left as i32) * 4
            + (right as i32) * 8;
        BreakerResult::Int(code as i64)
    })
}

pub(crate) fn make_reboot_button_pressed_breaker(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    _run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let btn_idx = app_config.sensor.reboot_button_index as usize;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let io = sensor.io_all();
        let pressed = *io.get(btn_idx).unwrap_or(&1.0) == 0.0;
        BreakerResult::Bool(pressed)
    })
}

pub(crate) fn make_check_gray_adc_for_scan_breaker(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let gray_idx = app_config.sensor.gray_adc_index as usize;
    let threshold = run_config.search.scan_move.gray_adc_lower_threshold as f64;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let adc = sensor.adc_all();
        let g = *adc.get(gray_idx).unwrap_or(&0.0);
        BreakerResult::Bool(g > threshold)
    })
}

pub(crate) fn make_back_stage_side_away_breaker(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let left_th = run_config.boot.left_threshold as f64;
    let right_th = run_config.boot.right_threshold as f64;
    let left_idx = app_config.sensor.left_adc_index as usize;
    let right_idx = app_config.sensor.right_adc_index as usize;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let adc = sensor.adc_all();
        let l = *adc.get(left_idx).unwrap_or(&0.0);
        let r = *adc.get(right_idx).unwrap_or(&0.0);
        BreakerResult::Bool(l > left_th || r > right_th)
    })
}
