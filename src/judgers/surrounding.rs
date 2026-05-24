use crate::config::{AppConfig, RunConfig, TagGroup};
use mentabotix_rs::transition::BreakerResult;
use std::sync::Arc;
use super::SensorData;

pub(crate) fn make_std_turn_to_front_breaker(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let front_adc_idx = app_config.sensor.front_adc_index as usize;
    let front_threshold = run_config.surrounding.front_adc_lower_threshold as f64;
    let fl_io = app_config.sensor.fl_io_index as usize;
    let fr_io = app_config.sensor.fr_io_index as usize;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let adc = sensor.adc_all();
        let io = sensor.io_all();
        let adc_v = *adc.get(front_adc_idx).unwrap_or(&0.0);
        let io_l = *io.get(fl_io).unwrap_or(&1.0);
        let io_r = *io.get(fr_io).unwrap_or(&1.0);
        // Front is clear when no IO trigger AND ADC below threshold
        let blocked = io_l == 0.0 || io_r == 0.0 || adc_v > front_threshold;
        BreakerResult::Bool(blocked)
    })
}

pub(crate) fn make_std_atk_breaker(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let front_adc_idx = app_config.sensor.front_adc_index as usize;
    let front_threshold = run_config.surrounding.atk_break_front_lower_threshold as f64;
    let fl_idx = app_config.sensor.edge_fl_index as usize;
    let fr_idx = app_config.sensor.edge_fr_index as usize;
    let lt = run_config.edge.lower_threshold;
    let ut = run_config.edge.upper_threshold;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let adc = sensor.adc_all();
        let adc_f = *adc.get(front_adc_idx).unwrap_or(&0.0);
        let fl_v = *adc.get(fl_idx).unwrap_or(&0.0);
        let fr_v = *adc.get(fr_idx).unwrap_or(&0.0);
        let obstacle = adc_f > front_threshold;
        let edge = (lt[0] > fl_v || fl_v > ut[0]) || (lt[3] > fr_v || fr_v > ut[3]);
        BreakerResult::Bool(obstacle || edge)
    })
}

pub(crate) fn make_atk_breaker_with_edge_sensors(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let fl_idx = app_config.sensor.edge_fl_index as usize;
    let fr_idx = app_config.sensor.edge_fr_index as usize;
    let lt = run_config.edge.lower_threshold;
    let ut = run_config.edge.upper_threshold;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let adc = sensor.adc_all();
        let fl_v = *adc.get(fl_idx).unwrap_or(&0.0);
        let fr_v = *adc.get(fr_idx).unwrap_or(&0.0);
        let edge = (lt[0] > fl_v || fl_v > ut[0]) || (lt[3] > fr_v || fr_v > ut[3]);
        BreakerResult::Bool(edge)
    })
}

pub(crate) fn make_surr_breaker(
    sensor: &Arc<dyn SensorData>,
    app_config: &AppConfig,
    run_config: &RunConfig,
    tag_group: &TagGroup,
) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
    let front_th = run_config.surrounding.front_adc_lower_threshold as f64;
    let back_th = run_config.surrounding.back_adc_lower_threshold as f64;
    let left_th = run_config.surrounding.left_adc_lower_threshold as f64;
    let right_th = run_config.surrounding.right_adc_lower_threshold as f64;
    let io_obj = run_config.surrounding.io_encounter_object_value as f64;
    let front_idx = app_config.sensor.front_adc_index as usize;
    let rear_idx = app_config.sensor.rb_adc_index as usize;
    let left_idx = app_config.sensor.left_adc_index as usize;
    let right_idx = app_config.sensor.right_adc_index as usize;
    let fl_io = app_config.sensor.fl_io_index as usize;
    let fr_io = app_config.sensor.fr_io_index as usize;
    let rl_io = app_config.sensor.rl_io_index as usize;
    let rr_io = app_config.sensor.rr_io_index as usize;
    let query_table = crate::static_utils::make_query_table(tag_group);
    let ally_tag = tag_group.ally_tag;
    let sensor = Arc::clone(sensor);

    Arc::new(move || {
        let adc = sensor.adc_all();
        let io = sensor.io_all();
        let front = *adc.get(front_idx).unwrap_or(&0.0) > front_th
            || *io.get(fl_io).unwrap_or(&1.0) == io_obj
            || *io.get(fr_io).unwrap_or(&1.0) == io_obj;
        let rear = *adc.get(rear_idx).unwrap_or(&0.0) > back_th
            || *io.get(rl_io).unwrap_or(&1.0) == io_obj
            || *io.get(rr_io).unwrap_or(&1.0) == io_obj;
        let left = *adc.get(left_idx).unwrap_or(&0.0) > left_th;
        let right = *adc.get(right_idx).unwrap_or(&0.0) > right_th;
        let mut code = (front as i32)
            + (rear as i32) * 2
            + (left as i32) * 4
            + (right as i32) * 8;
        // Tag-group based classification: if front object detected and
        // team has ally tags configured, classify as ally (conservative).
        if front && matches!(query_table.get(&(ally_tag, false)), Some(&0)) {
            code += 100; // FRONT_ALLY_BOX range
        }
        BreakerResult::Int(code as i64)
    })
}
