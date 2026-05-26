use crate::compile::{
    HandlerOutput, continues_state, halt_state, make_straight, make_trans, make_turn_l, make_turn_r,
};
use crate::config::{AppConfig, RunConfig};
use crate::constant::ScanCodesign;
use crate::judgers::Breakers;
use mentabotix_rs::composer::MovingChainComposer;
use mentabotix_rs::registry::CaseRegistry;
use mentabotix_rs::transition::BreakerResult;
use std::collections::HashMap;
use std::sync::Arc;

use mentabotix_rs::state::MovingState;
use mentabotix_rs::transition::MovingTransition;

pub fn make_scan_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    end_state: Option<MovingState>,
) -> HandlerOutput {
    let breakers = Breakers::null();
    let end_state = end_state.unwrap_or_else(halt_state);
    let scan_cfg = &run_config.search.scan_move;

    // ── Breakers ──────────────────────────────────────────────
    let scan_breaker = breakers.make_std_scan_breaker(app_config, run_config);
    let rear_edge_breaker = breakers.make_std_edge_rear_breaker(app_config, run_config);
    let turn_to_front_breaker = breakers.make_std_turn_to_front_breaker(app_config, run_config);

    // ── State closures (fresh IDs per invocation) ─────────────
    // scan_state: Python rand_dir_turn → Rust make_straight
    let scan_s = || make_straight(scan_cfg.scan_speed);
    // rand_turn_state: Python rand_dir_turn → Rust make_turn_l
    let rnd_s = || make_turn_l(scan_cfg.turn_speed);
    let lt_s = || make_turn_l(scan_cfg.turn_speed);
    let rt_s = || make_turn_r(scan_cfg.turn_speed);
    let fb_s = || make_straight(-scan_cfg.fall_back_speed);

    // ── Transition closures ───────────────────────────────────
    // Python uses run_config.surrounding.full_turn_duration / half_turn_duration
    let ft_t = || {
        make_trans(
            run_config.surrounding.full_turn_duration,
            Some(turn_to_front_breaker.clone()),
        )
    };
    let ht_t = || {
        make_trans(
            run_config.surrounding.half_turn_duration,
            Some(turn_to_front_breaker.clone()),
        )
    };
    let fb_t = || make_trans(scan_cfg.fall_back_duration, Some(rear_edge_breaker.clone()));

    // ── Case chains ───────────────────────────────────────────
    let mut transitions_pool: Vec<MovingTransition> = Vec::new();
    let mut case_reg = CaseRegistry::<ScanCodesign>::new();

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

    let add_s = |s: MovingState| {
        move |mut c: MovingChainComposer| {
            c.add_state(s);
            c
        }
    };
    let add_t = |t: MovingTransition| {
        move |mut c: MovingChainComposer| {
            c.add_transition(t);
            c
        }
    };

    // Case 1: O_O_O_O → end_state (direct, no chain)
    case_reg
        .register(ScanCodesign::O_O_O_O, end_state.id())
        .ok();

    // Case 2: X_O_O_O, X_O_X_X → fall_back → end_state
    scan_case!(
        case_reg,
        &[ScanCodesign::X_O_O_O, ScanCodesign::X_O_X_X],
        [add_s(fb_s()), add_t(fb_t()), add_s(halt_state()),]
    );

    // Case 3: X_X_X_X, X_X_X_O, X_X_O_X, O_X_X_X, X_X_O_O, O_X_X_O, O_X_O_X, O_X_O_O
    //         → rand_turn → full_turn → end_state
    scan_case!(
        case_reg,
        &[
            ScanCodesign::X_X_X_X,
            ScanCodesign::X_X_X_O,
            ScanCodesign::X_X_O_X,
            ScanCodesign::O_X_X_X,
            ScanCodesign::X_X_O_O,
            ScanCodesign::O_X_X_O,
            ScanCodesign::O_X_O_X,
            ScanCodesign::O_X_O_O,
        ],
        [add_s(rnd_s()), add_t(ft_t()), add_s(halt_state()),]
    );

    // Case 4: O_O_X_O, X_O_X_O → turn_left → half_turn → end_state
    scan_case!(
        case_reg,
        &[ScanCodesign::O_O_X_O, ScanCodesign::X_O_X_O],
        [add_s(lt_s()), add_t(ht_t()), add_s(halt_state()),]
    );

    // Case 5: O_O_O_X, X_O_O_X → turn_right → half_turn → end_state
    scan_case!(
        case_reg,
        &[ScanCodesign::O_O_O_X, ScanCodesign::X_O_O_X],
        [add_s(rt_s()), add_t(ht_t()), add_s(halt_state()),]
    );

    // Case 6: O_O_X_X → fall_back → rand_turn → half_turn → end_state
    scan_case!(
        case_reg,
        &[ScanCodesign::O_O_X_X],
        [
            add_s(fb_s()),
            add_t(fb_t()),
            add_s(rnd_s()),
            add_t(ht_t()),
            add_s(halt_state()),
        ]
    );

    // ── Assembly: pre-checks + scan chain ─────────────────────
    let mut composer = MovingChainComposer::new();
    // salvo_end = make_salvo_end_state() in Python; continues_state() in Rust
    let salvo_end = continues_state();

    // Pre-check 1: gray ADC before scan
    if scan_cfg.check_gray_adc_before_scan {
        let gray_adc_breaker =
            breakers.make_check_gray_adc_for_scan_breaker(app_config, run_config);
        let mut pre_trans = MovingTransition::new(0.0)
            .unwrap()
            .with_arc_breaker(gray_adc_breaker);
        pre_trans
            .to_states
            .insert(BreakerResult::Bool(true), salvo_end.id());
        composer.add_state(halt_state());
        composer.add_transition(pre_trans);
    }

    // Pre-check 2: edge before scan
    if scan_cfg.check_edge_before_scan {
        let edge_full_breaker = breakers.make_std_edge_full_breaker(app_config, run_config);
        // Python: bool(check_edge_breaker()) — converts Int → Bool
        let boolean_full_edge_breaker: Arc<dyn Fn() -> BreakerResult + Send + Sync> = {
            let inner = Arc::clone(&edge_full_breaker);
            Arc::new(move || match inner() {
                BreakerResult::Int(v) => BreakerResult::Bool(v != 0),
                other => other,
            })
        };
        let mut pre_trans = MovingTransition::new(0.0)
            .unwrap()
            .with_arc_breaker(boolean_full_edge_breaker);
        pre_trans
            .to_states
            .insert(BreakerResult::Bool(true), salvo_end.id());
        composer.add_state(halt_state());
        composer.add_transition(pre_trans);
    }

    // Scan transition: dispatches to case chain heads based on ScanCodesign
    let to_states: HashMap<BreakerResult, usize> = case_reg
        .export()
        .into_iter()
        .map(|(k, v)| (BreakerResult::Int(k as i64), v))
        .collect();

    let mut scan_trans = MovingTransition::new(scan_cfg.scan_duration)
        .unwrap()
        .with_arc_breaker(scan_breaker);
    for (key, state_id) in &to_states {
        scan_trans.to_states.insert(key.clone(), *state_id);
    }

    composer.add_state(scan_s());
    composer.add_transition(scan_trans);

    let (composer_states, mut composer_trans) = composer.export();
    let start_state = composer_states
        .into_iter()
        .next()
        .unwrap_or_else(halt_state);
    transitions_pool.append(&mut composer_trans);

    HandlerOutput {
        start_state,
        normal_exit: end_state,
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
