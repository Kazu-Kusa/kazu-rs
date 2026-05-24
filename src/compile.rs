//! Robot behavior handlers — compiles run config into Botix state graphs.
//!
//! Each handler function corresponds to a robot behavior (edge detection,
//! search, battle, navigation, etc.) and returns the transitions that define
//! the state machine for that behavior.
//!
//! Port of `kazu/compile.py`.


use crate::config::{AppConfig, RunConfig};
use crate::constant::{
    EdgeCodeSign, FenceCodeSign, ScanCodesign,
    StageCodeSign, SurroundingCodeSign,
};
use crate::judgers::Breakers;
use mentabotix_rs::{
    composer::MovingChainComposer,
    registry::CaseRegistry,
    state::{FixedAxis, MovingState, TurnDirection},
    transition::{BreakerResult, MovingTransition},
};
use std::collections::HashMap;

/// Result from a handler: the transitions defining the behavior graph
/// plus the start, normal-exit, and abnormal-exit states.
#[allow(dead_code)]
pub struct HandlerOutput {
    pub start_state: MovingState,
    pub normal_exit: MovingState,
    pub abnormal_exit: MovingState,
    pub transitions: Vec<MovingTransition>,
}

/// Shorthand: just the transitions (used by viz).
pub type HandlerResult = Vec<MovingTransition>;

// ── Helpers ───────────────────────────────────────────────────

fn halt_state() -> MovingState { MovingState::halt() }
fn continues_state() -> MovingState { MovingState::straight(0) }

fn make_straight(speed: i32) -> MovingState { MovingState::straight(speed) }
fn make_turn_l(speed: i32) -> MovingState { MovingState::turn(TurnDirection::Left, speed) }
fn make_turn_r(speed: i32) -> MovingState { MovingState::turn(TurnDirection::Right, speed) }
fn make_drift_rl(speed: i32) -> MovingState { MovingState::drift(FixedAxis::RearLeft, speed) }
fn make_drift_rr(speed: i32) -> MovingState { MovingState::drift(FixedAxis::RearRight, speed) }

fn make_trans(
    dur: f64,
    breaker: Option<std::sync::Arc<dyn Fn() -> BreakerResult + Send + Sync>>,
) -> MovingTransition {
    let mut t = MovingTransition::new(dur).unwrap();
    if let Some(b) = breaker { t = t.with_arc_breaker(b); }
    t
}

fn make_trans_no_breaker(dur: f64) -> MovingTransition {
    MovingTransition::new(dur).unwrap()
}

// ── Edge handler ──────────────────────────────────────────────

pub fn make_edge_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    start_state: Option<MovingState>,
    normal_exit: Option<MovingState>,
    abnormal_exit: Option<MovingState>,
) -> HandlerOutput {
    let breakers = Breakers::null();
    let start_state = start_state.unwrap_or_else(continues_state);
    let normal_exit = normal_exit.unwrap_or_else(continues_state);
    let abnormal_exit = abnormal_exit.unwrap_or_else(halt_state);

    let ec = &run_config.edge;

    let edge_full_breaker = breakers.make_std_edge_full_breaker(app_config, run_config);
    let edge_front_breaker = breakers.make_std_edge_front_breaker(app_config, run_config);
    let edge_rear_breaker = breakers.make_std_edge_rear_breaker(app_config, run_config);

    let mut composer = MovingChainComposer::new();
    let mut transitions_pool: Vec<MovingTransition> = Vec::new();
    let mut case_reg = CaseRegistry::<EdgeCodeSign>::new();

    case_reg.register(EdgeCodeSign::O_O_O_O, normal_exit.id()).ok();
    case_reg.register(EdgeCodeSign::X_X_X_X, abnormal_exit.id()).ok();

    // Helper: build a chain, register head state, collect transitions.
    // Takes closures that produce states/transitions on each call (no cloning).
    macro_rules! edge_case {
        ($reg:expr, $sign:expr, [$($step:expr),+ $(,)?]) => {{
            let (states, trans) = {
                $(
                    let _ = $step;
                )+
                // Build chain step by step
                let mut c = MovingChainComposer::new();
                $(
                    c = $step(c);
                )+
                c.export()
            };
            transitions_pool.extend(trans);
            if let Some(head) = states.first() {
                $reg.register($sign, head.id()).ok();
            }
        }};
    }

    // Each $step is a closure: |c: MovingChainComposer| -> MovingChainComposer
    // that adds one state or transition and returns the composer.

    let fb_state = || make_straight(-ec.fallback_speed);
    let fb_trans = || make_trans(ec.fallback_duration, Some(edge_rear_breaker.clone()));

    let adv_state = || make_straight(ec.advance_speed);
    let adv_trans = || make_trans(ec.advance_duration, Some(edge_front_breaker.clone()));

    let lt_state = || make_turn_l(ec.turn_speed);
    let rt_state = || make_turn_r(ec.turn_speed);
    let rlt_state = || make_turn_l(ec.turn_speed); // simplified rand

    let full_t = || make_trans_no_breaker(ec.full_turn_duration);
    let half_t = || make_trans_no_breaker(ec.half_turn_duration);
    let drift_t = || make_trans_no_breaker(ec.drift_duration);
    let dlb_state = || make_drift_rl(ec.drift_speed);
    let drb_state = || make_drift_rr(ec.drift_speed);

    let abnormal = || halt_state();

    let add_s = |s: MovingState| move |mut c: MovingChainComposer| { c.add_state(s); c };
    let add_t = |t: MovingTransition| move |mut c: MovingChainComposer| { c.add_transition(t); c };

    // 1-Activation cases
    edge_case!(case_reg, EdgeCodeSign::X_O_O_O, [
        add_s(fb_state()), add_t(fb_trans()), add_s(rt_state()), add_t(full_t()), add_s(abnormal()),
    ]);
    edge_case!(case_reg, EdgeCodeSign::O_O_O_X, [
        add_s(fb_state()), add_t(fb_trans()), add_s(lt_state()), add_t(full_t()), add_s(abnormal()),
    ]);
    edge_case!(case_reg, EdgeCodeSign::O_X_O_O, [
        add_s(adv_state()), add_t(adv_trans()), add_s(rt_state()), add_t(half_t()), add_s(abnormal()),
    ]);
    edge_case!(case_reg, EdgeCodeSign::O_O_X_O, [
        add_s(adv_state()), add_t(adv_trans()), add_s(lt_state()), add_t(half_t()), add_s(abnormal()),
    ]);

    // 2-Activation cases
    edge_case!(case_reg, EdgeCodeSign::X_X_O_O, [
        add_s(rt_state()), add_t(half_t()), add_s(abnormal()),
    ]);
    edge_case!(case_reg, EdgeCodeSign::O_O_X_X, [
        add_s(lt_state()), add_t(half_t()), add_s(abnormal()),
    ]);
    edge_case!(case_reg, EdgeCodeSign::X_O_O_X, [
        add_s(fb_state()), add_t(fb_trans()), add_s(rlt_state()), add_t(full_t()), add_s(abnormal()),
    ]);
    edge_case!(case_reg, EdgeCodeSign::O_X_X_O, [
        add_s(adv_state()), add_t(adv_trans()), add_s(abnormal()),
    ]);
    edge_case!(case_reg, EdgeCodeSign::X_O_X_O, [
        add_s(drb_state()), add_t(drift_t()), add_s(abnormal()),
    ]);
    edge_case!(case_reg, EdgeCodeSign::O_X_O_X, [
        add_s(dlb_state()), add_t(drift_t()), add_s(abnormal()),
    ]);

    // 3-Activation cases
    edge_case!(case_reg, EdgeCodeSign::O_X_X_X, [
        add_s(lt_state()), add_t(half_t()), add_s(adv_state()), add_t(adv_trans()), add_s(abnormal()),
    ]);
    edge_case!(case_reg, EdgeCodeSign::X_X_X_O, [
        add_s(rt_state()), add_t(half_t()), add_s(adv_state()), add_t(adv_trans()), add_s(abnormal()),
    ]);
    edge_case!(case_reg, EdgeCodeSign::X_O_X_X, [
        add_s(rt_state()), add_t(half_t()), add_s(fb_state()), add_t(fb_trans()), add_s(abnormal()),
    ]);
    edge_case!(case_reg, EdgeCodeSign::X_X_O_X, [
        add_s(lt_state()), add_t(half_t()), add_s(fb_state()), add_t(fb_trans()), add_s(abnormal()),
    ]);

    // Top-level branching transition
    composer.init_container();
    composer.add_state(start_state.clone());

    let to_states: HashMap<BreakerResult, usize> = case_reg
        .export()
        .into_iter()
        .map(|(k, v)| (BreakerResult::Int(k as i64), v))
        .collect();

    let mut head = MovingTransition::new(run_config.perf.checking_duration)
        .unwrap()
        .with_arc_breaker(edge_full_breaker);
    for (key, state_id) in &to_states {
        head.to_states.insert(key.clone(), *state_id);
    }
    composer.add_transition(head);

    let (_composer_states, mut composer_trans) = composer.export();
    transitions_pool.append(&mut composer_trans);

    HandlerOutput {
        start_state,
        normal_exit,
        abnormal_exit,
        transitions: transitions_pool,
    }
}

// ── Surrounding handler ───────────────────────────────────────

pub fn make_surrounding_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    start_state: Option<MovingState>,
    normal_exit: Option<MovingState>,
    abnormal_exit: Option<MovingState>,
) -> HandlerOutput {
    let breakers = Breakers::null();
    let start_state = start_state.unwrap_or_else(continues_state);
    let normal_exit = normal_exit.unwrap_or_else(continues_state);
    let abnormal_exit = abnormal_exit.unwrap_or_else(halt_state);

    let sc = &run_config.surrounding;

    let tag_group = crate::config::TagGroup::new(&run_config.team_color);
    let _query_table = crate::static_utils::make_query_table(&tag_group);
    let surr_full_breaker = breakers.make_surr_breaker(
        app_config, run_config,
        &tag_group,
    );
    let atk_breaker = breakers.make_std_atk_breaker(app_config, run_config);
    let edge_rear_breaker = breakers.make_std_edge_rear_breaker(app_config, run_config);
    let turn_to_front_breaker = breakers.make_std_turn_to_front_breaker(app_config, run_config);

    // State factories
    let aec = || make_straight(sc.atk_speed_enemy_car);
    let aeb = || make_straight(sc.atk_speed_enemy_box);
    let anb = || make_straight(sc.atk_speed_neutral_box);
    let afb = || make_straight(-sc.fallback_speed_ally_box);
    let efb = || make_straight(-sc.fallback_speed_edge);
    let lt = || make_turn_l(sc.turn_speed);
    let rt = || make_turn_r(sc.turn_speed);
    let rnd = || make_turn_l(sc.turn_speed); // simplified rand

    // Transition factories
    let aect = || make_trans(sc.atk_enemy_car_duration, Some(atk_breaker.clone()));
    let aebt = || make_trans(sc.atk_enemy_box_duration, Some(atk_breaker.clone()));
    let anbt = || make_trans(sc.atk_neutral_box_duration, Some(atk_breaker.clone()));
    let afbt = || make_trans(sc.fallback_duration_ally_box, Some(edge_rear_breaker.clone()));
    let efbt = || make_trans(sc.fallback_duration_edge, Some(edge_rear_breaker.clone()));
    let ftt = || make_trans(sc.full_turn_duration, Some(turn_to_front_breaker.clone()));
    let htt = || make_trans(sc.half_turn_duration, Some(turn_to_front_breaker.clone()));

    let abn = || halt_state();

    let mut transitions_pool: Vec<MovingTransition> = Vec::new();
    let mut case_reg = CaseRegistry::<SurroundingCodeSign>::new();

    case_reg.register(SurroundingCodeSign::NOTHING, normal_exit.id()).ok();

    macro_rules! surr_case {
        ($reg:expr, $signs:expr, [$($step:expr),+ $(,)?]) => {{
            let mut c = MovingChainComposer::new();
            $(
                c = $step(c);
            )+
            let (states, trans) = c.export();
            transitions_pool.extend(trans);
            if let Some(head) = states.first() {
                for sign in $signs {
                    $reg.register(*sign, head.id()).ok();
                }
            }
        }};
    }

    let add_s = |s: MovingState| move |mut c: MovingChainComposer| { c.add_state(s); c };
    let add_t = |t: MovingTransition| move |mut c: MovingChainComposer| { c.add_transition(t); c };

    // Front enemy car
    surr_case!(case_reg, &[
        SurroundingCodeSign::FRONT_ENEMY_CAR,
        SurroundingCodeSign::FRONT_ENEMY_CAR_RIGHT_OBJECT,
        SurroundingCodeSign::FRONT_ENEMY_CAR_LEFT_OBJECT,
        SurroundingCodeSign::FRONT_ENEMY_CAR_BEHIND_OBJECT,
        SurroundingCodeSign::FRONT_ENEMY_CAR_LEFT_RIGHT_OBJECTS,
        SurroundingCodeSign::FRONT_ENEMY_CAR_RIGHT_BEHIND_OBJECTS,
        SurroundingCodeSign::FRONT_ENEMY_CAR_LEFT_BEHIND_OBJECTS,
        SurroundingCodeSign::FRONT_ENEMY_CAR_LEFT_RIGHT_BEHIND_OBJECTS,
    ], [
        add_s(aec()), add_t(aect()), add_s(efb()), add_t(efbt()),
        add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);

    // Target switch: behind
    surr_case!(case_reg, &[
        SurroundingCodeSign::BEHIND_OBJECT,
        SurroundingCodeSign::LEFT_RIGHT_BEHIND_OBJECTS,
        SurroundingCodeSign::FRONT_ENEMY_BOX_BEHIND_OBJECT,
        SurroundingCodeSign::FRONT_ALLY_BOX_BEHIND_OBJECT,
        SurroundingCodeSign::FRONT_NEUTRAL_BOX_BEHIND_OBJECT,
    ], [
        add_s(rnd()), add_t(ftt()), add_s(aec()), add_t(aect()),
        add_s(efb()), add_t(efbt()), add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);

    // Left object
    surr_case!(case_reg, &[
        SurroundingCodeSign::LEFT_OBJECT,
    ], [
        add_s(lt()), add_t(htt()), add_s(aec()), add_t(aect()),
        add_s(efb()), add_t(efbt()), add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);

    // Right object
    surr_case!(case_reg, &[
        SurroundingCodeSign::RIGHT_OBJECT,
    ], [
        add_s(rt()), add_t(htt()), add_s(aec()), add_t(aect()),
        add_s(efb()), add_t(efbt()), add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);

    // Left+right objects
    surr_case!(case_reg, &[
        SurroundingCodeSign::LEFT_RIGHT_OBJECTS,
    ], [
        add_s(rnd()), add_t(htt()), add_s(aec()), add_t(aect()),
        add_s(efb()), add_t(efbt()), add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);

    // Front box only
    surr_case!(case_reg, &[
        SurroundingCodeSign::FRONT_ENEMY_BOX,
    ], [
        add_s(aeb()), add_t(aebt()), add_s(efb()), add_t(efbt()),
        add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);
    surr_case!(case_reg, &[
        SurroundingCodeSign::FRONT_NEUTRAL_BOX,
    ], [
        add_s(anb()), add_t(anbt()), add_s(efb()), add_t(efbt()),
        add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);
    surr_case!(case_reg, &[
        SurroundingCodeSign::FRONT_ALLY_BOX,
    ], [
        add_s(afb()), add_t(afbt()), add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);

    // Assembly: top-level branching
    let mut composer = MovingChainComposer::new();
    composer.add_state(start_state.clone());

    let to_states: HashMap<BreakerResult, usize> = case_reg
        .export()
        .into_iter()
        .map(|(k, v)| (BreakerResult::Int(k as i64), v))
        .collect();

    let mut head = MovingTransition::new(run_config.perf.checking_duration)
        .unwrap()
        .with_arc_breaker(surr_full_breaker);
    for (key, state_id) in &to_states {
        head.to_states.insert(key.clone(), *state_id);
    }
    composer.add_transition(head);

    let (_, mut composer_trans) = composer.export();
    transitions_pool.append(&mut composer_trans);

    HandlerOutput {
        start_state,
        normal_exit,
        abnormal_exit,
        transitions: transitions_pool,
    }
}

// ── Scan handler ──────────────────────────────────────────────

pub fn make_scan_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    end_state: Option<MovingState>,
) -> HandlerOutput {
    let breakers = Breakers::null();
    let scan_cfg = &run_config.search.scan_move;
    let end_state = end_state.unwrap_or_else(continues_state);

    let scan_breaker = breakers.make_std_scan_breaker(app_config, run_config);
    let edge_rear_breaker = breakers.make_std_edge_rear_breaker(app_config, run_config);
    let gray_adc_breaker = breakers.make_check_gray_adc_for_scan_breaker(app_config, run_config);

    let scan_s = || make_straight(scan_cfg.scan_speed);
    let fb_s = || make_straight(-scan_cfg.fall_back_speed);
    let lt_s = || make_turn_l(scan_cfg.turn_speed);
    let rt_s = || make_turn_r(scan_cfg.turn_speed);
    let rnd_s = || make_turn_l(scan_cfg.turn_speed); // simplified

    let scan_t = || make_trans(scan_cfg.scan_duration, Some(scan_breaker.clone()));
    let fb_t = || make_trans(scan_cfg.fall_back_duration, Some(edge_rear_breaker.clone()));
    let ft_t = || make_trans_no_breaker(scan_cfg.full_turn_duration);
    let ht_t = || make_trans_no_breaker(scan_cfg.half_turn_duration);

    let end = || continues_state();
    let _abn = || halt_state();

    let mut transitions_pool: Vec<MovingTransition> = Vec::new();
    let mut case_reg = CaseRegistry::<ScanCodesign>::new();

    case_reg.register(ScanCodesign::O_O_O_O, end_state.id()).ok();

    macro_rules! scan_case {
        ($reg:expr, $signs:expr, [$($step:expr),+ $(,)?]) => {{
            let mut c = MovingChainComposer::new();
            $(
                c = $step(c);
            )+
            let (states, trans) = c.export();
            transitions_pool.extend(trans);
            if let Some(head) = states.first() {
                for sign in $signs {
                    $reg.register(*sign, head.id()).ok();
                }
            }
        }};
    }

    let add_s = |s: MovingState| move |mut c: MovingChainComposer| { c.add_state(s); c };
    let add_t = |t: MovingTransition| move |mut c: MovingChainComposer| { c.add_transition(t); c };

    // Front only
    scan_case!(case_reg, &[ScanCodesign::X_O_O_O], [
        add_s(rnd_s()), add_t(ft_t()), add_s(scan_s()), add_t(scan_t()), add_s(end()),
    ]);

    // Rear only
    scan_case!(case_reg, &[ScanCodesign::O_X_O_O], [
        add_s(fb_s()), add_t(fb_t()), add_s(scan_s()), add_t(scan_t()), add_s(end()),
    ]);

    // Left/right
    scan_case!(case_reg, &[ScanCodesign::O_O_X_O], [
        add_s(rt_s()), add_t(ht_t()), add_s(scan_s()), add_t(scan_t()), add_s(end()),
    ]);
    scan_case!(case_reg, &[ScanCodesign::O_O_O_X], [
        add_s(lt_s()), add_t(ht_t()), add_s(scan_s()), add_t(scan_t()), add_s(end()),
    ]);

    // Multi-trigger → aggressive
    scan_case!(case_reg, &[
        ScanCodesign::X_X_O_O, ScanCodesign::O_O_X_X,
        ScanCodesign::X_O_X_O, ScanCodesign::O_X_O_X,
        ScanCodesign::X_O_O_X, ScanCodesign::O_X_X_O,
        ScanCodesign::X_X_X_O, ScanCodesign::X_X_O_X,
        ScanCodesign::X_O_X_X, ScanCodesign::O_X_X_X,
        ScanCodesign::X_X_X_X,
    ], [
        add_s(rnd_s()), add_t(ft_t()), add_s(scan_s()), add_t(scan_t()), add_s(end()),
    ]);

    // Assembly
    let mut composer = MovingChainComposer::new();
    // Pre-check: if on stage (gray ADC high), skip scan entirely
    composer.add_state(continues_state());

    let mut pre_head = MovingTransition::new(run_config.perf.checking_duration)
        .unwrap()
        .with_arc_breaker(gray_adc_breaker.clone());
    pre_head.to_states.insert(BreakerResult::Bool(true), end_state.id());
    pre_head.to_states.insert(BreakerResult::Bool(false), scan_s().id());
    composer.add_transition(pre_head);

    // Scan flow: only reached if not on stage
    composer.add_state(scan_s());

    let to_states: HashMap<BreakerResult, usize> = case_reg
        .export()
        .into_iter()
        .map(|(k, v)| (BreakerResult::Int(k as i64), v))
        .collect();

    let mut head = MovingTransition::new(scan_cfg.scan_duration)
        .unwrap()
        .with_arc_breaker(scan_breaker);
    for (key, state_id) in &to_states {
        head.to_states.insert(key.clone(), *state_id);
    }
    composer.add_transition(head);

    let (composer_states, mut composer_trans) = composer.export();
    let start_state = composer_states.into_iter().next().unwrap_or_else(halt_state);
    transitions_pool.append(&mut composer_trans);

    HandlerOutput {
        start_state,
        normal_exit: end_state,
        abnormal_exit: halt_state(),
        transitions: transitions_pool,
    }
}

// ── Search handler (delegates to scan) ────────────────────────

pub fn make_search_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    _start_state: Option<MovingState>,
    normal_exit: Option<MovingState>,
    _abnormal_exit: Option<MovingState>,
) -> HandlerOutput {
    make_scan_handler(app_config, run_config, normal_exit)
}

// ── Fence handler ─────────────────────────────────────────────

pub fn make_fence_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    start_state: Option<MovingState>,
    normal_exit: Option<MovingState>,
    abnormal_exit: Option<MovingState>,
) -> HandlerOutput {
    let breakers = Breakers::null();
    let start_state = start_state.unwrap_or_else(continues_state);
    let normal_exit = normal_exit.unwrap_or_else(continues_state);
    let abnormal_exit = abnormal_exit.unwrap_or_else(halt_state);

    let fc = &run_config.fence;
    let fence_breaker = breakers.make_std_fence_breaker(app_config, run_config);
    let lr_blocked_breaker = breakers.make_lr_sides_blocked_breaker(app_config, run_config);
    // Invert: transition fires when at least one side clears (not both blocked)
    let lr_clear_breaker: std::sync::Arc<dyn Fn() -> BreakerResult + Send + Sync> = {
        let inner = std::sync::Arc::clone(&lr_blocked_breaker);
        std::sync::Arc::new(move || match inner() {
            BreakerResult::Bool(b) => BreakerResult::Bool(!b),
            other => other,
        })
    };

    let lt_s = || make_turn_l(fc.direction_align_speed);
    let rt_s = || make_turn_r(fc.direction_align_speed);
    let rnd_s = || make_turn_l(fc.stage_align_speed); // simplified
    let exit_s = || make_straight(fc.exit_corner_speed);

    let ft_t = || make_trans_no_breaker(fc.max_direction_align_duration);
    let ht_t = || make_trans_no_breaker(fc.max_direction_align_duration);
    let exit_t = || make_trans_no_breaker(fc.max_exit_corner_duration);
    let lr_check_t = || make_trans(fc.max_direction_align_duration, Some(std::sync::Arc::clone(&lr_clear_breaker)));

    let abn = || halt_state();

    let mut transitions_pool: Vec<MovingTransition> = Vec::new();
    let mut case_reg = CaseRegistry::<FenceCodeSign>::new();

    case_reg.register(FenceCodeSign::O_O_O_O, normal_exit.id()).ok();

    macro_rules! fence_case {
        ($reg:expr, $signs:expr, [$($step:expr),+ $(,)?]) => {{
            let mut c = MovingChainComposer::new();
            $(
                c = $step(c);
            )+
            let (states, trans) = c.export();
            transitions_pool.extend(trans);
            if let Some(head) = states.first() {
                for sign in $signs {
                    $reg.register(*sign, head.id()).ok();
                }
            }
        }};
    }

    let add_s = |s: MovingState| move |mut c: MovingChainComposer| { c.add_state(s); c };
    let add_t = |t: MovingTransition| move |mut c: MovingChainComposer| { c.add_transition(t); c };

    fence_case!(case_reg, &[FenceCodeSign::X_O_O_O], [
        add_s(rnd_s()), add_t(ft_t()), add_s(exit_s()), add_t(exit_t()), add_s(abn()),
    ]);
    fence_case!(case_reg, &[FenceCodeSign::O_X_O_O], [
        add_s(rnd_s()), add_t(ft_t()), add_s(exit_s()), add_t(exit_t()), add_s(abn()),
    ]);
    fence_case!(case_reg, &[FenceCodeSign::O_O_X_O, FenceCodeSign::X_O_X_O,
        FenceCodeSign::O_X_X_O, FenceCodeSign::X_X_X_O], [
        add_s(rt_s()), add_t(ht_t()), add_s(exit_s()), add_t(exit_t()), add_s(abn()),
    ]);
    fence_case!(case_reg, &[FenceCodeSign::O_O_O_X, FenceCodeSign::X_O_O_X,
        FenceCodeSign::O_X_O_X, FenceCodeSign::X_X_O_X], [
        add_s(lt_s()), add_t(ht_t()), add_s(exit_s()), add_t(exit_t()), add_s(abn()),
    ]);
    // LR-blocked cases: use lr_check_t to wait until at least one side clears
    fence_case!(case_reg, &[FenceCodeSign::O_O_X_X, FenceCodeSign::X_O_X_X,
        FenceCodeSign::O_X_X_X, FenceCodeSign::X_X_X_X], [
        add_s(rnd_s()), add_t(lr_check_t()), add_s(exit_s()), add_t(exit_t()), add_s(abn()),
    ]);
    // Non-LR-blocked multi-trigger: just random turn and exit
    fence_case!(case_reg, &[FenceCodeSign::X_X_O_O, FenceCodeSign::X_X_O_X], [
        add_s(rnd_s()), add_t(ft_t()), add_s(exit_s()), add_t(exit_t()), add_s(abn()),
    ]);

    // Assembly
    let mut composer = MovingChainComposer::new();
    composer.add_state(start_state.clone());

    let to_states: HashMap<BreakerResult, usize> = case_reg
        .export()
        .into_iter()
        .map(|(k, v)| (BreakerResult::Int(k as i64), v))
        .collect();
    let mut head = MovingTransition::new(run_config.perf.checking_duration)
        .unwrap()
        .with_arc_breaker(fence_breaker);
    for (key, state_id) in &to_states {
        head.to_states.insert(key.clone(), *state_id);
    }
    composer.add_transition(head);

    let (_, mut composer_trans) = composer.export();
    transitions_pool.append(&mut composer_trans);

    HandlerOutput {
        start_state,
        normal_exit,
        abnormal_exit,
        transitions: transitions_pool,
    }
}

// ── Reboot handler ────────────────────────────────────────────

pub fn make_reboot_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    end_state: Option<MovingState>,
) -> HandlerOutput {
    let breakers = Breakers::null();
    let bc = &run_config.boot;
    let end_state = end_state.unwrap_or_else(continues_state);

    let reboot_breaker = breakers.make_reboot_button_pressed_breaker(app_config, run_config);
    let edge_rear_breaker = breakers.make_std_edge_rear_breaker(app_config, run_config);

    let dash_s = || make_straight(bc.dash_speed);
    let fb_s = || make_straight(-bc.dash_speed / 3);
    let turn_s = || make_turn_l(bc.turn_speed);

    let dash_t = || make_trans(bc.dash_duration, Some(edge_rear_breaker.clone()));
    let fb_t = || make_trans(0.2, Some(edge_rear_breaker.clone()));
    let turn_t = || make_trans(bc.full_turn_duration, Some(reboot_breaker));

    let end = || continues_state();

    let mut c = MovingChainComposer::new();
    c.add_state(dash_s());
    c.add_transition(dash_t());
    c.add_state(fb_s());
    c.add_transition(fb_t());
    c.add_state(turn_s());
    c.add_transition(turn_t());
    c.add_state(end());
    let (states, transitions) = c.export();

    HandlerOutput {
        start_state: states.into_iter().next().unwrap_or_else(halt_state),
        normal_exit: end_state,
        abnormal_exit: halt_state(),
        transitions,
    }
}

// ── Back to stage handler ─────────────────────────────────────

pub fn make_back_to_stage_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    end_state: Option<MovingState>,
) -> HandlerOutput {
    let breakers = Breakers::null();
    let bsc = &run_config.backstage;
    let end_state = end_state.unwrap_or_else(continues_state);

    let edge_rear_breaker = breakers.make_std_edge_rear_breaker(app_config, run_config);
    let side_away_breaker = breakers.make_back_stage_side_away_breaker(app_config, run_config);

    let dash_s = || make_straight(bsc.dash_speed);
    let exit_s = || make_straight(bsc.exit_side_away_speed);
    let turn_s = || make_turn_l(bsc.turn_speed);
    let adv_s = || make_straight(bsc.small_advance_speed);

    let dash_t = || make_trans(bsc.dash_duration, Some(edge_rear_breaker.clone()));
    let exit_t = || make_trans(bsc.exit_side_away_duration, Some(side_away_breaker));
    let turn_t = || make_trans_no_breaker(bsc.full_turn_duration);
    let adv_t = || make_trans_no_breaker(bsc.small_advance_duration);

    let end = || continues_state();

    let mut transitions_pool: Vec<MovingTransition> = Vec::new();

    // Dash → end
    let mut c = MovingChainComposer::new();
    c.add_state(dash_s());
    c.add_transition(dash_t());
    c.add_state(end());
    let (_, trans) = c.export();
    transitions_pool.extend(trans);

    // Side-away recovery
    let mut c = MovingChainComposer::new();
    c.add_state(exit_s());
    c.add_transition(exit_t());
    c.add_state(turn_s());
    c.add_transition(turn_t());
    c.add_state(adv_s());
    c.add_transition(adv_t());
    c.add_state(end());
    let (_, trans) = c.export();
    transitions_pool.extend(trans);

    HandlerOutput {
        start_state: dash_s(),
        normal_exit: end_state,
        abnormal_exit: halt_state(),
        transitions: transitions_pool,
    }
}

// ── Random walk handler ───────────────────────────────────────

pub fn make_rand_walk_handler(
    _app_config: &AppConfig,
    run_config: &RunConfig,
    end_state: Option<MovingState>,
) -> HandlerOutput {
    let rw = &run_config.fence.rand_walk;
    let end_state = end_state.unwrap_or_else(continues_state);

    let s_s = || make_straight(500);
    let t_s = || make_turn_l(800);

    let w_t = || make_trans_no_breaker(rw.walk_duration);
    let tt = || make_trans_no_breaker(rw.walk_duration);

    let end = || continues_state();

    let mut c = MovingChainComposer::new();
    c.add_state(s_s());
    c.add_transition(w_t());
    c.add_state(t_s());
    c.add_transition(tt());
    c.add_state(end());
    let (states, transitions) = c.export();

    HandlerOutput {
        start_state: states.into_iter().next().unwrap_or_else(halt_state),
        normal_exit: end_state,
        abnormal_exit: halt_state(),
        transitions,
    }
}

// ── Standard battle handler ───────────────────────────────────

pub fn make_std_battle_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> HandlerOutput {
    let sc = &run_config.surrounding;
    let breakers = Breakers::null();

    let atk_breaker = breakers.make_std_atk_breaker(app_config, run_config);

    let atk_s = || make_straight(sc.atk_speed_enemy_car);
    let fb_s = || make_straight(-sc.fallback_speed_edge);
    let turn_s = || make_turn_l(sc.turn_speed);

    let atk_t = || make_trans(sc.atk_enemy_car_duration, Some(atk_breaker));
    let fb_t = || make_trans_no_breaker(sc.fallback_duration_edge);
    let turn_t = || make_trans_no_breaker(sc.full_turn_duration);

    let mut c = MovingChainComposer::new();
    c.add_state(atk_s());
    c.add_transition(atk_t());
    c.add_state(fb_s());
    c.add_transition(fb_t());
    c.add_state(turn_s());
    c.add_transition(turn_t());
    c.add_state(atk_s()); // loop
    let (states, transitions) = c.export();

    HandlerOutput {
        start_state: states.into_iter().next().unwrap_or_else(halt_state),
        normal_exit: halt_state(),
        abnormal_exit: halt_state(),
        transitions,
    }
}

// ── Always-on-stage / Always-off-stage battle handlers ────────

pub fn make_always_on_stage_battle_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> HandlerOutput {
    make_std_battle_handler(app_config, run_config)
}

pub fn make_always_off_stage_battle_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> HandlerOutput {
    make_std_battle_handler(app_config, run_config)
}

// ── Rand turn handler ─────────────────────────────────────────

#[allow(dead_code)]
pub fn make_rand_turn_handler(
    _app_config: &AppConfig,
    run_config: &RunConfig,
    end_state: Option<MovingState>,
) -> HandlerOutput {
    let sc = &run_config.search.rand_turn;
    let end_state = end_state.unwrap_or_else(continues_state);

    let turn_s = || make_turn_l(sc.turn_speed);
    let turn_t = || make_trans_no_breaker(sc.full_turn_duration);
    let end = || continues_state();

    let mut c = MovingChainComposer::new();
    c.add_state(turn_s());
    c.add_transition(turn_t());
    c.add_state(end());
    let (states, transitions) = c.export();

    HandlerOutput {
        start_state: states.into_iter().next().unwrap_or_else(halt_state),
        normal_exit: end_state,
        abnormal_exit: halt_state(),
        transitions,
    }
}

// ── Align direction handler ───────────────────────────────────

// TODO: wire into fence_handler for MPU- or sensor-based direction alignment.
#[allow(dead_code)]
pub fn make_align_direction_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    not_aligned_state: Option<MovingState>,
) -> HandlerOutput {
    let breakers = Breakers::null();
    let fc = &run_config.fence;
    let not_aligned = not_aligned_state.unwrap_or_else(continues_state);

    let align_breaker = breakers.make_std_align_direction_breaker(app_config, run_config);

    let rt_s = || make_turn_r(fc.direction_align_speed);
    let turn_t = || make_trans(fc.max_direction_align_duration, Some(align_breaker));

    let mut c = MovingChainComposer::new();
    c.add_state(rt_s());
    c.add_transition(turn_t());
    c.add_state(not_aligned.clone());
    let (states, transitions) = c.export();

    HandlerOutput {
        start_state: states.into_iter().next().unwrap_or_else(halt_state),
        normal_exit: not_aligned,
        abnormal_exit: halt_state(),
        transitions,
    }
}

// ── On-stage handler ──────────────────────────────────────────

// TODO: wire into NGS/FGS assembly for stage-transition logic.
#[allow(dead_code)]
pub fn make_on_stage_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    start_state: Option<MovingState>,
    normal_exit: Option<MovingState>,
    abnormal_exit: Option<MovingState>,
) -> HandlerOutput {
    let breakers = Breakers::null();
    let start_state = start_state.unwrap_or_else(continues_state);
    let normal_exit = normal_exit.unwrap_or_else(continues_state);
    let abnormal_exit = abnormal_exit.unwrap_or_else(halt_state);

    let stage_breaker = breakers.make_std_stage_breaker(app_config, run_config);
    let sc = &run_config.stage;

    let unclear_s = || make_turn_l(sc.unclear_zone_turn_speed);
    let unclear_t = || make_trans_no_breaker(sc.unclear_zone_turn_duration);

    let mut transitions_pool: Vec<MovingTransition> = Vec::new();
    let mut case_reg = CaseRegistry::<StageCodeSign>::new();

    case_reg.register(StageCodeSign::OFF_STAGE, normal_exit.id()).ok();
    case_reg.register(StageCodeSign::ON_STAGE, normal_exit.id()).ok();
    case_reg.register(StageCodeSign::UNCLEAR_ZONE, unclear_s().id()).ok();

    // Unclear zone chain
    let mut c = MovingChainComposer::new();
    c.add_state(unclear_s());
    c.add_transition(unclear_t());
    c.add_state(normal_exit.clone());
    let (_, trans) = c.export();
    transitions_pool.extend(trans);

    // Assembly
    let mut composer = MovingChainComposer::new();
    composer.add_state(start_state.clone());

    let to_states: HashMap<BreakerResult, usize> = case_reg
        .export()
        .into_iter()
        .map(|(k, v)| (BreakerResult::Int(k as i64), v))
        .collect();
    let mut head = MovingTransition::new(run_config.perf.checking_duration)
        .unwrap()
        .with_arc_breaker(stage_breaker);
    for (key, state_id) in &to_states {
        head.to_states.insert(key.clone(), *state_id);
    }
    composer.add_transition(head);

    let (_, mut composer_trans) = composer.export();
    transitions_pool.append(&mut composer_trans);

    HandlerOutput {
        start_state,
        normal_exit,
        abnormal_exit,
        transitions: transitions_pool,
    }
}

// ── Unclear zone handler ──────────────────────────────────────

// TODO: wire live SensorData for gray ADC read; currently uses fixed 0.0.
#[allow(dead_code)]
/// Handler for unclear zone (between on-stage and off-stage).
pub fn make_unclear_zone_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    normal_exit: Option<MovingState>,
) -> HandlerOutput {
    let sc = &run_config.stage;
    let normal_exit = normal_exit.unwrap_or_else(continues_state);

    // TODO: read real gray ADC when Breakers has live SensorData wired.
    // Currently uses fixed 0.0; unclear_zone_tolerance check is a no-op.
    let _gray_idx = app_config.sensor.gray_adc_index as usize;
    let tolerance = sc.unclear_zone_tolerance as f64;

    // Shared state: hook records initial gray, breaker compares against it.
    let recorded_gray: std::sync::Arc<std::sync::Mutex<Option<f64>>> =
        std::sync::Arc::new(std::sync::Mutex::new(None));

    // Hook: capture initial gray ADC value on entry.
    let recorded = std::sync::Arc::clone(&recorded_gray);
    let start_state = continues_state().with_before_entering(move || {
        // TODO: read real gray ADC via sensor when live SensorData available.
        if let Ok(mut guard) = recorded.lock() {
            *guard = Some(0.0);
        }
    });

    // Breaker: exit unclear zone when gray ADC deviates beyond tolerance.
    let recorded = std::sync::Arc::clone(&recorded_gray);
    let unclear_breaker: std::sync::Arc<dyn Fn() -> BreakerResult + Send + Sync> =
        std::sync::Arc::new(move || {
            // TODO: read real gray ADC via sensor when live SensorData available.
            let current_gray = 0.0_f64;
            let triggered = if let Ok(guard) = recorded.lock() {
                match *guard {
                    Some(initial) => (current_gray - initial).abs() > tolerance,
                    None => false,
                }
            } else {
                false
            };
            BreakerResult::Bool(triggered)
        });

    let turn_t = || make_trans(sc.unclear_zone_turn_duration, Some(std::sync::Arc::clone(&unclear_breaker)));

    let mut c = MovingChainComposer::new();
    c.add_state(start_state.clone());       // records gray ADC via hook
    c.add_transition(turn_t());            // turns until tolerance exceeded
    c.add_state(normal_exit.clone());
    let (_, transitions) = c.export();

    HandlerOutput {
        start_state,
        normal_exit,
        abnormal_exit: halt_state(),
        transitions,
    }
}

// ── Dispatch table ────────────────────────────────────────────

/// Signature for handler functions compatible with the dispatch table.
pub type HandlerFn = fn(&AppConfig, &RunConfig) -> HandlerResult;

/// Dispatch table mapping pack names to handler functions.
pub fn get_handler(name: &str) -> Option<HandlerFn> {
    match name {
        "edge" => Some(|ac, rc| {
            make_edge_handler(ac, rc, None, None, None).transitions
        }),
        "surr" => Some(|ac, rc| {
            make_surrounding_handler(ac, rc, None, None, None).transitions
        }),
        "scan" => Some(|ac, rc| {
            make_scan_handler(ac, rc, None).transitions
        }),
        "search" => Some(|ac, rc| {
            make_search_handler(ac, rc, None, None, None).transitions
        }),
        "fence" => Some(|ac, rc| {
            make_fence_handler(ac, rc, None, None, None).transitions
        }),
        "boot" => Some(|ac, rc| {
            make_reboot_handler(ac, rc, None).transitions
        }),
        "bkstage" => Some(|ac, rc| {
            make_back_to_stage_handler(ac, rc, None).transitions
        }),
        "rdwalk" => Some(|ac, rc| {
            make_rand_walk_handler(ac, rc, None).transitions
        }),
        "stdbat" => Some(|ac, rc| {
            make_std_battle_handler(ac, rc).transitions
        }),
        "onstage" => Some(|ac, rc| {
            make_always_on_stage_battle_handler(ac, rc).transitions
        }),
        "angbat" => Some(|ac, rc| {
            make_always_on_stage_battle_handler(ac, rc).transitions
        }),
        "afgbat" => Some(|ac, rc| {
            make_always_off_stage_battle_handler(ac, rc).transitions
        }),
        _ => None,
    }
}

/// All available handler names.
pub fn all_handler_names() -> &'static [&'static str] {
    &[
        "edge", "surr", "scan", "search", "fence", "boot",
        "bkstage", "rdwalk", "stdbat", "onstage", "angbat", "afgbat",
    ]
}
