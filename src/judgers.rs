//! Breaker predicate factories — port of `kazu/judgers.py`.

//!
//! Each method on [`Breakers`] returns a closure suitable for
//! [`MovingTransition::with_breaker`] that evaluates sensor data against
//! configured thresholds and returns a [`BreakerResult`].

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
    // TODO: implement via uptechstar-rs MPU6500 when hardware is connected.
    #[allow(dead_code)]
    fn mpu_yaw(&self) -> f64;
}

/// A no-op sensor source returning zeros — for graph viz / offline use.
pub struct NullSensor;

impl SensorData for NullSensor {
    fn adc_all(&self) -> Vec<f64> { vec![0.0; 10] }
    fn io_all(&self) -> Vec<f64> { vec![0.0; 8] }
    fn mpu_yaw(&self) -> f64 { 0.0 }
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
        let lt = run_config.edge.lower_threshold;
        let ut = run_config.edge.upper_threshold;
        let rl_idx = app_config.sensor.edge_rl_index as usize;
        let rr_idx = app_config.sensor.edge_rr_index as usize;
        let sensor = Arc::clone(&self.sensor);

        Arc::new(move || {
            let adc = sensor.adc_all();
            let s0 = *adc.get(rl_idx).unwrap_or(&0.0);
            let s1 = *adc.get(rr_idx).unwrap_or(&0.0);
            let triggered =
                (lt[1] > s0 || s0 > ut[1]) || (lt[2] > s1 || s1 > ut[2]);
            BreakerResult::Bool(triggered)
        })
    }

    /// Front edge detector: checks gray IO + FL/FR ADC.
    pub fn make_std_edge_front_breaker(
        &self,
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
        let sensor = Arc::clone(&self.sensor);

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

    /// Full edge detector: returns an EdgeCodeSign discriminant (int).
    /// FL=1, FR=2, RL=4, RR=8.
    pub fn make_std_edge_full_breaker(
        &self,
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
        let sensor = Arc::clone(&self.sensor);

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

    /// Stop-turning breaker: checks front obstacles (ADC + IO).
    pub fn make_std_turn_to_front_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        let front_adc_idx = app_config.sensor.front_adc_index as usize;
        let front_threshold = run_config.surrounding.front_adc_lower_threshold as f64;
        let fl_io = app_config.sensor.fl_io_index as usize;
        let fr_io = app_config.sensor.fr_io_index as usize;
        let sensor = Arc::clone(&self.sensor);

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

    // ── Attack breakers ────────────────────────────────────────

    /// Standard attack breaker: front obstacle + edge.
    pub fn make_std_atk_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        let front_adc_idx = app_config.sensor.front_adc_index as usize;
        let front_threshold = run_config.surrounding.atk_break_front_lower_threshold as f64;
        let fl_idx = app_config.sensor.edge_fl_index as usize;
        let fr_idx = app_config.sensor.edge_fr_index as usize;
        let lt = run_config.edge.lower_threshold;
        let ut = run_config.edge.upper_threshold;
        let sensor = Arc::clone(&self.sensor);

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

    /// Attack breaker using edge sensors directly.
    // TODO: wire into the handler that needs this breaker.
    #[allow(dead_code)]
    pub fn make_atk_breaker_with_edge_sensors(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        let fl_idx = app_config.sensor.edge_fl_index as usize;
        let fr_idx = app_config.sensor.edge_fr_index as usize;
        let lt = run_config.edge.lower_threshold;
        let ut = run_config.edge.upper_threshold;
        let sensor = Arc::clone(&self.sensor);

        Arc::new(move || {
            let adc = sensor.adc_all();
            let fl_v = *adc.get(fl_idx).unwrap_or(&0.0);
            let fr_v = *adc.get(fr_idx).unwrap_or(&0.0);
            let edge = (lt[0] > fl_v || fl_v > ut[0]) || (lt[3] > fr_v || fr_v > ut[3]);
            BreakerResult::Bool(edge)
        })
    }

    // ── Stage / align breakers ─────────────────────────────────

    /// Stage alignment breaker: checks front + rear obstacles.
    // TODO: wire into the handler that needs this breaker.
    #[allow(dead_code)]
    pub fn make_std_stage_align_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        let front_adc_idx = app_config.sensor.front_adc_index as usize;
        let rear_adc_idx = app_config.sensor.rb_adc_index as usize;
        let front_threshold = run_config.surrounding.front_adc_lower_threshold as f64;
        let rear_threshold = run_config.surrounding.back_adc_lower_threshold as f64;
        let sensor = Arc::clone(&self.sensor);

        Arc::new(move || {
            let adc = sensor.adc_all();
            let f = *adc.get(front_adc_idx).unwrap_or(&0.0);
            let r = *adc.get(rear_adc_idx).unwrap_or(&0.0);
            BreakerResult::Bool(f > front_threshold || r > rear_threshold)
        })
    }

    /// MPU-based stage alignment breaker.
    // TODO: wire into the handler that needs this breaker.
    #[allow(dead_code)]
    pub fn make_stage_align_breaker_mpu(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        let front_adc_idx = app_config.sensor.front_adc_index as usize;
        let front_threshold = run_config.surrounding.front_adc_lower_threshold as f64;
        let yaw_tolerance = run_config.fence.max_yaw_tolerance;
        let sensor = Arc::clone(&self.sensor);

        Arc::new(move || {
            let adc = sensor.adc_all();
            let f = *adc.get(front_adc_idx).unwrap_or(&0.0);
            let yaw = sensor.mpu_yaw();
            let aligned = yaw.abs() < yaw_tolerance;
            BreakerResult::Bool(f > front_threshold || aligned)
        })
    }

    // ── Fence breaker ──────────────────────────────────────────

    /// Full fence breaker: returns FenceCodeSign discriminant (int).
    pub fn make_std_fence_breaker(
        &self,
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
        let sensor = Arc::clone(&self.sensor);

        Arc::new(move || {
            let adc = sensor.adc_all();
            let io = sensor.io_all();
            let front =
                *adc.get(f_idx).unwrap_or(&0.0) > f_th
                    || *io.get(fl_io).unwrap_or(&1.0) == io_fence_val
                    || *io.get(fr_io).unwrap_or(&1.0) == io_fence_val;
            let rear =
                *adc.get(r_idx).unwrap_or(&0.0) > r_th
                    || *io.get(rl_io).unwrap_or(&1.0) == io_fence_val
                    || *io.get(rr_io).unwrap_or(&1.0) == io_fence_val;
            let left = *adc.get(l_idx).unwrap_or(&0.0) > l_th;
            let right = *adc.get(ri_idx).unwrap_or(&0.0) > ri_th;

            let code = (front as i32)
                + (rear as i32) * 2
                + (left as i32) * 4
                + (right as i32) * 8;
            BreakerResult::Int(code as i64)
        })
    }

    /// Left-right sides blocked breaker.
    // TODO: wire into the handler that needs this breaker.
    pub fn make_lr_sides_blocked_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        let l_th = run_config.fence.left_adc_lower_threshold as f64;
        let r_th = run_config.fence.right_adc_lower_threshold as f64;
        let l_idx = app_config.sensor.left_adc_index as usize;
        let r_idx = app_config.sensor.right_adc_index as usize;
        let sensor = Arc::clone(&self.sensor);

        Arc::new(move || {
            let adc = sensor.adc_all();
            let l = *adc.get(l_idx).unwrap_or(&0.0) > l_th;
            let r = *adc.get(r_idx).unwrap_or(&0.0) > r_th;
            BreakerResult::Bool(l && r)
        })
    }

    // ── Direction alignment breakers ───────────────────────────

    /// MPU-based direction alignment breaker.
    // TODO: wire into the handler that needs this breaker.
    #[allow(dead_code)]
    pub fn make_align_direction_breaker_mpu(
        &self,
        _app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        let yaw_tol = run_config.fence.max_yaw_tolerance;
        let sensor = Arc::clone(&self.sensor);

        Arc::new(move || {
            let yaw = sensor.mpu_yaw();
            BreakerResult::Bool(yaw.abs() < yaw_tol)
        })
    }

    /// Standard (non-MPU) direction alignment breaker.
    // TODO: wire into the handler that needs this breaker.
    #[allow(dead_code)]
    pub fn make_std_align_direction_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        let front_th = run_config.surrounding.front_adc_lower_threshold as f64;
        let back_th = run_config.surrounding.back_adc_lower_threshold as f64;
        let front_idx = app_config.sensor.front_adc_index as usize;
        let back_idx = app_config.sensor.rb_adc_index as usize;
        let sensor = Arc::clone(&self.sensor);

        Arc::new(move || {
            let adc = sensor.adc_all();
            let f = *adc.get(front_idx).unwrap_or(&0.0);
            let b = *adc.get(back_idx).unwrap_or(&0.0);
            BreakerResult::Bool(f > front_th || b > back_th)
        })
    }

    // ── Scan breaker ───────────────────────────────────────────

    /// Standard scan breaker: returns ScanCodeSign discriminant (int).
    pub fn make_std_scan_breaker(
        &self,
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
        let sensor = Arc::clone(&self.sensor);

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
        let gray_idx = app_config.sensor.gray_adc_index as usize;
        let off_upper = run_config.stage.gray_adc_off_stage_upper_threshold as f64;
        let on_lower = run_config.stage.gray_adc_on_stage_lower_threshold as f64;
        let reboot_idx = app_config.sensor.reboot_button_index as usize;
        let reboot_activate = run_config.boot.button_io_activate_case_value as f64;
        let sensor = Arc::clone(&self.sensor);

        Arc::new(move || {
            let adc = sensor.adc_all();
            let io = sensor.io_all();
            let gray_adc = *adc.get(gray_idx).unwrap_or(&0.0);
            let reboot = *io.get(reboot_idx).unwrap_or(&1.0);

            // StageWeight: STAGE=1, REBOOT=2, UNCLEAR=4
            let off_stage = if gray_adc <= off_upper { 1i32 } else { 0 };
            let unclear = if gray_adc > off_upper && gray_adc < on_lower { 4i32 } else { 0 };
            let rebooted = if reboot == reboot_activate { 2i32 } else { 0 };

            let code = off_stage + unclear + rebooted;
            BreakerResult::Int(code as i64)
        })
    }

    /// Always-on stage breaker: always returns ON_STAGE (0).
    pub fn make_always_on_stage_breaker(
        &self,
        _app_config: &AppConfig,
        _run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        Arc::new(|| BreakerResult::Int(0))
    }

    /// Always-off stage breaker: always returns OFF_STAGE (1).
    pub fn make_always_off_stage_breaker(
        &self,
        _app_config: &AppConfig,
        _run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        Arc::new(|| BreakerResult::Int(1))
    }

    // ── Surrounding breaker ────────────────────────────────────

    /// Surrounding obstacle detector: returns SurroundingCodeSign discriminant (int).
    pub fn make_surr_breaker(
        &self,
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
        let sensor = Arc::clone(&self.sensor);

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

    // ── Misc breakers ──────────────────────────────────────────

    /// Reboot button pressed detector.
    pub fn make_reboot_button_pressed_breaker(
        &self,
        app_config: &AppConfig,
        _run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        let btn_idx = app_config.sensor.reboot_button_index as usize;
        let sensor = Arc::clone(&self.sensor);

        Arc::new(move || {
            let io = sensor.io_all();
            let pressed = *io.get(btn_idx).unwrap_or(&1.0) == 0.0;
            BreakerResult::Bool(pressed)
        })
    }

    /// Gray ADC check for scan.
    // TODO: wire into the handler that needs this breaker.
    pub fn make_check_gray_adc_for_scan_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        let gray_idx = app_config.sensor.gray_adc_index as usize;
        let threshold = run_config.search.scan_move.gray_adc_lower_threshold as f64;
        let sensor = Arc::clone(&self.sensor);

        Arc::new(move || {
            let adc = sensor.adc_all();
            let g = *adc.get(gray_idx).unwrap_or(&0.0);
            BreakerResult::Bool(g > threshold)
        })
    }

    /// On-stage checker using gray ADC.
    // TODO: wire into the handler that needs this breaker.
    #[allow(dead_code)]
    pub fn make_is_on_stage_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        let gray_idx = app_config.sensor.gray_adc_index as usize;
        let on_lower = run_config.stage.gray_adc_on_stage_lower_threshold as f64;
        let sensor = Arc::clone(&self.sensor);

        Arc::new(move || {
            let adc = sensor.adc_all();
            let g = *adc.get(gray_idx).unwrap_or(&0.0);
            BreakerResult::Bool(g >= on_lower)
        })
    }

    /// Back-stage side-away detector.
    pub fn make_back_stage_side_away_breaker(
        &self,
        app_config: &AppConfig,
        run_config: &RunConfig,
    ) -> Arc<dyn Fn() -> BreakerResult + Send + Sync> {
        let left_th = run_config.boot.left_threshold as f64;
        let right_th = run_config.boot.right_threshold as f64;
        let left_idx = app_config.sensor.left_adc_index as usize;
        let right_idx = app_config.sensor.right_adc_index as usize;
        let sensor = Arc::clone(&self.sensor);

        Arc::new(move || {
            let adc = sensor.adc_all();
            let l = *adc.get(left_idx).unwrap_or(&0.0);
            let r = *adc.get(right_idx).unwrap_or(&0.0);
            BreakerResult::Bool(l > left_th || r > right_th)
        })
    }
}
