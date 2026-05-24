use crate::compile::{continues_state, halt_state, make_straight, make_turn_l, make_turn_r, make_trans, make_trans_no_breaker, HandlerOutput};
use crate::config::{AppConfig, RunConfig};
use crate::constant::ScanCodesign;
use crate::judgers::Breakers;
use mentabotix_rs::composer::MovingChainComposer;
use mentabotix_rs::registry::CaseRegistry;
use mentabotix_rs::transition::BreakerResult;
use std::collections::HashMap;

use mentabotix_rs::state::MovingState;
use mentabotix_rs::transition::MovingTransition;
pub fn make_scan_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    end_state: Option<MovingState>,
) -> HandlerOutput {
    use mentabotix_rs::state::MovingState;

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

pub fn make_search_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    _start_state: Option<MovingState>,
    normal_exit: Option<MovingState>,
    _abnormal_exit: Option<MovingState>,
) -> HandlerOutput {
    

    make_scan_handler(app_config, run_config, normal_exit)
}
