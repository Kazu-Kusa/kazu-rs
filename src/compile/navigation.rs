use crate::compile::{continues_state, halt_state, make_straight, make_turn_l, make_turn_r, make_trans, make_trans_no_breaker, HandlerOutput};
use crate::config::{AppConfig, RunConfig};
use crate::judgers::Breakers;
use mentabotix_rs::composer::MovingChainComposer;
use mentabotix_rs::transition::BreakerResult;

use mentabotix_rs::state::MovingState;
use mentabotix_rs::transition::MovingTransition;
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

pub fn make_back_to_stage_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    end_state: Option<MovingState>,
) -> HandlerOutput {
    

    let breakers = Breakers::null();
    let bsc = &run_config.backstage;
    let end_state = end_state.unwrap_or_else(continues_state);

    let side_away_breaker = breakers.make_back_stage_side_away_breaker(app_config, run_config);
    let on_stage_breaker = breakers.make_is_on_stage_breaker(app_config, run_config);

    let dash_s = || make_straight(bsc.dash_speed);
    let exit_s = || make_straight(bsc.exit_side_away_speed);
    let turn_s = || make_turn_l(bsc.turn_speed);
    let adv_s = || make_straight(bsc.small_advance_speed);

    let exit_t = || make_trans(bsc.exit_side_away_duration, Some(side_away_breaker));
    let turn_t = || make_trans_no_breaker(bsc.full_turn_duration);
    let adv_t = || make_trans_no_breaker(bsc.small_advance_duration);

    let end = || continues_state();

    let mut transitions_pool: Vec<MovingTransition> = Vec::new();

    // Dash → on-stage check: if on stage, go to end; else side-away recovery
    let mut c = MovingChainComposer::new();
    c.add_state(dash_s());

    let mut dash_trans = MovingTransition::new(bsc.dash_duration)
        .unwrap()
        .with_arc_breaker(on_stage_breaker.clone());
    dash_trans.to_states.insert(BreakerResult::Bool(true), end_state.id());
    dash_trans.to_states.insert(BreakerResult::Bool(false), exit_s().id());
    c.add_transition(dash_trans);

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
