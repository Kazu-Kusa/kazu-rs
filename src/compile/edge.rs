use crate::compile::{continues_state, halt_state, make_straight, make_turn_l, make_turn_r, make_drift_rl, make_drift_rr, make_trans, make_trans_no_breaker, HandlerOutput};
use crate::config::{AppConfig, RunConfig};
use crate::constant::EdgeCodeSign;
use crate::judgers::Breakers;
use mentabotix_rs::composer::MovingChainComposer;
use mentabotix_rs::registry::CaseRegistry;
use mentabotix_rs::transition::BreakerResult;
use std::collections::HashMap;
use mentabotix_rs::state::MovingState;
use mentabotix_rs::transition::MovingTransition;

pub fn make_edge_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    start_state: Option<MovingState>,
    normal_exit: Option<MovingState>,
    abnormal_exit: Option<MovingState>,
) -> HandlerOutput {
    use mentabotix_rs::state::MovingState;

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
