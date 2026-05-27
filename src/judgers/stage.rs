use super::SensorData;
use crate::config::{AppConfig, RunConfig};
use mentabotix_rs::transition::BreakerResult;
use std::sync::Arc;

pub(crate) fn make_std_stage_breaker(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let gray_idx = app_config.sensor.gray_adc_index as usize;
    let off_upper = run_config.stage.gray_adc_off_stage_upper_threshold as f64;
    let on_lower = run_config.stage.gray_adc_on_stage_lower_threshold as f64;
    let reboot_idx = app_config.sensor.reboot_button_index as usize;
    let reboot_activate = run_config.boot.button_io_activate_case_value as f64;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let adc = sensor.adc_all();
        let io = sensor.io_all();
        let gray_adc = *adc.get(gray_idx).unwrap_or(&0.0);
        let reboot = *io.get(reboot_idx).unwrap_or(&1.0);

        // StageWeight: STAGE=1, REBOOT=2, UNCLEAR=4
        let off_stage = if gray_adc <= off_upper { 1i32 } else { 0 };
        let unclear = if gray_adc > off_upper && gray_adc < on_lower {
            4i32
        } else {
            0
        };
        let rebooted = if reboot == reboot_activate { 2i32 } else { 0 };

        let code = off_stage + unclear + rebooted;
        BreakerResult::Int(code as i64)
    })
}

pub(crate) fn make_always_on_stage_breaker(
    _app_config: &AppConfig,
    _run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    Arc::new(|| BreakerResult::Int(0))
}

pub(crate) fn make_always_off_stage_breaker(
    _app_config: &AppConfig,
    _run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    Arc::new(|| BreakerResult::Int(1))
}

pub(crate) fn make_is_on_stage_breaker(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let gray_idx = app_config.sensor.gray_adc_index as usize;
    let on_lower = run_config.stage.gray_adc_on_stage_lower_threshold as f64;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let adc = sensor.adc_all();
        let g = *adc.get(gray_idx).unwrap_or(&0.0);
        BreakerResult::Bool(g >= on_lower)
    })
}
