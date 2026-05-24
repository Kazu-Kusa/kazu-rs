use crate::config::{AppConfig, RunConfig};
use mentabotix_rs::transition::BreakerResult;
use std::sync::Arc;
use super::SensorData;

pub(crate) fn make_std_edge_rear_breaker(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let lt = run_config.edge.lower_threshold;
    let ut = run_config.edge.upper_threshold;
    let rl_idx = app_config.sensor.edge_rl_index as usize;
    let rr_idx = app_config.sensor.edge_rr_index as usize;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let adc = sensor.adc_all();
        let s0 = *adc.get(rl_idx).unwrap_or(&0.0);
        let s1 = *adc.get(rr_idx).unwrap_or(&0.0);
        let triggered =
            (lt[1] > s0 || s0 > ut[1]) || (lt[2] > s1 || s1 > ut[2]);
        BreakerResult::Bool(triggered)
    })
}

pub(crate) fn make_std_edge_front_breaker(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let lt = run_config.edge.lower_threshold;
    let ut = run_config.edge.upper_threshold;
    let activate = run_config.stage.gray_io_off_stage_case_value as f64;
    let gray_l = app_config.sensor.gray_io_left_index as usize;
    let gray_r = app_config.sensor.gray_io_right_index as usize;
    let fl_idx = app_config.sensor.edge_fl_index as usize;
    let fr_idx = app_config.sensor.edge_fr_index as usize;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let io = sensor.io_all();
        let adc = sensor.adc_all();
        let s0 = *io.get(gray_l).unwrap_or(&0.0);
        let s1 = *io.get(gray_r).unwrap_or(&0.0);
        let s2 = *adc.get(fl_idx).unwrap_or(&0.0);
        let s3 = *adc.get(fr_idx).unwrap_or(&0.0);
        let triggered = s0 == activate
            || s1 == activate
            || (lt[0] > s2 || s2 > ut[0])
            || (lt[3] > s3 || s3 > ut[3]);
        BreakerResult::Bool(triggered)
    })
}

pub(crate) fn make_std_edge_full_breaker(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let lt = run_config.edge.lower_threshold;
    let ut = run_config.edge.upper_threshold;
    let use_gray = run_config.edge.use_gray_io;
    let activate = run_config.stage.gray_io_off_stage_case_value as f64;
    let fl_idx = app_config.sensor.edge_fl_index as usize;
    let fr_idx = app_config.sensor.edge_fr_index as usize;
    let rl_idx = app_config.sensor.edge_rl_index as usize;
    let rr_idx = app_config.sensor.edge_rr_index as usize;
    let gray_l = app_config.sensor.gray_io_left_index as usize;
    let gray_r = app_config.sensor.gray_io_right_index as usize;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let adc = sensor.adc_all();
        let io = sensor.io_all();

        let fl_v = *adc.get(fl_idx).unwrap_or(&0.0);
        let fr_v = *adc.get(fr_idx).unwrap_or(&0.0);
        let rl_v = *adc.get(rl_idx).unwrap_or(&0.0);
        let rr_v = *adc.get(rr_idx).unwrap_or(&0.0);

        let fl = (lt[0] > fl_v || fl_v > ut[0]) as i32;
        let fr = (lt[3] > fr_v || fr_v > ut[3]) as i32;
        let rl = (lt[1] > rl_v || rl_v > ut[1]) as i32;
        let rr = (lt[2] > rr_v || rr_v > ut[2]) as i32;

        let mut code = fl + fr * 2 + rl * 4 + rr * 8;

        // If gray IO is used and all edges clear, check gray IO
        if use_gray && code == 0 {
            let g0 = *io.get(gray_l).unwrap_or(&1.0);
            let g1 = *io.get(gray_r).unwrap_or(&1.0);
            if g0 == activate || g1 == activate {
                // Front gray triggered — treat as front edge
                code = 1; // FL only
            }
        }

        BreakerResult::Int(code as i64)
    })
}
