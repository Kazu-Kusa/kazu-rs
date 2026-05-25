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

    let mut c = MovingChainComposer::new();
    // halt (with siglight before_entering in Python — skipped, NullSensor)
    c.add_state(halt_state());
    // holding transition: wait for reboot button, up to max_holding_duration
    c.add_transition(make_trans(bc.max_holding_duration, Some(reboot_breaker)));
    // dash backward
    c.add_state(make_straight(-bc.dash_speed));
    // dash duration with edge-rear detection
    c.add_transition(make_trans(bc.dash_duration, Some(edge_rear_breaker)));
    // halt after dash
    c.add_state(halt_state());
    // stabilize
    c.add_transition(make_trans_no_breaker(bc.time_to_stabilize));
    // rand_dir_turn simplified: always turn left
    c.add_state(make_turn_l(bc.turn_speed));
    // full turn duration
    c.add_transition(make_trans_no_breaker(bc.full_turn_duration));
    // end
    c.add_state(end_state.clone());
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

    let small_advance = make_straight(bsc.small_advance_speed);
    let dash_backward = make_straight(-bsc.dash_speed);
    let turn_state = make_turn_l(bsc.turn_speed);

    // Build side-away recovery sub-chain first (if both checks enabled).
    // Its first state is `concat_state` referenced by the checking transition.
    let mut extra_transitions: Vec<MovingTransition> = Vec::new();
    let concat_state_id: Option<usize>;

    if bsc.use_is_on_stage_check && bsc.use_side_away_check {
        let side_away_breaker =
            breakers.make_back_stage_side_away_breaker(app_config, run_config);

        let mut side_c = MovingChainComposer::new();
        // concat_state: exit side-away moving forward
        side_c.add_state(make_straight(bsc.exit_side_away_speed));
        side_c.add_transition(make_trans(
            bsc.exit_side_away_duration,
            Some(side_away_breaker),
        ));
        side_c.add_state(turn_state.clone());
        side_c.add_transition(make_trans_no_breaker(bsc.full_turn_duration));
        side_c.add_state(halt_state());

        let (s, mut t) = side_c.export();
        let halt_id = s.last().unwrap().id();

        // Loop-back transition: stabilize, then back to small_advance
        let mut loop_back_t = make_trans_no_breaker(bsc.time_to_stabilize);
        loop_back_t.from_states.push(halt_id);
        loop_back_t
            .to_states
            .insert(BreakerResult::Placeholder, small_advance.id());
        t.push(loop_back_t);

        concat_state_id = Some(s.first().unwrap().id());
        extra_transitions = t;
    } else {
        concat_state_id = None;
    }

    // Main chain
    let mut c = MovingChainComposer::new();
    // small_advance
    c.add_state(small_advance.clone());
    c.add_transition(make_trans_no_breaker(bsc.small_advance_duration));
    // halt
    c.add_state(halt_state());
    // stabilize
    c.add_transition(make_trans_no_breaker(bsc.time_to_stabilize));
    // dash backward
    c.add_state(dash_backward.clone());

    if bsc.use_is_on_stage_check {
        let is_on_stage_breaker =
            breakers.make_is_on_stage_breaker(app_config, run_config);

        // First part of dash
        c.add_transition(make_trans_no_breaker(
            bsc.dash_duration * bsc.check_start_percent,
        ));
        // Continue dashing backward
        c.add_state(dash_backward.clone());

        // Checking transition: remaining dash with on-stage breaker
        let mut checking_t = make_trans(
            bsc.dash_duration * (1.0 - bsc.check_start_percent),
            Some(is_on_stage_breaker),
        );
        // If breaker returns False (not on stage) → concat_state (side-away recovery)
        if let Some(cid) = concat_state_id {
            checking_t
                .to_states
                .insert(BreakerResult::Bool(false), cid);
        }
        c.add_transition(checking_t);
    } else {
        // Full dash without stage check
        c.add_transition(make_trans_no_breaker(bsc.dash_duration));
    }

    // halt (register_case in Python)
    c.add_state(halt_state());
    // stabilize
    c.add_transition(make_trans_no_breaker(bsc.time_to_stabilize));
    // rand_dir_turn simplified: always turn left
    c.add_state(turn_state.clone());
    // full turn
    c.add_transition(make_trans_no_breaker(bsc.full_turn_duration));
    // end
    c.add_state(end_state.clone());

    let (_, mut main_transitions) = c.export();

    // Merge: side-away transitions first, then main chain
    let mut all_transitions = extra_transitions;
    all_transitions.append(&mut main_transitions);

    HandlerOutput {
        start_state: small_advance,
        normal_exit: end_state,
        abnormal_exit: halt_state(),
        transitions: all_transitions,
    }
}

pub fn make_rand_walk_handler(
    _app_config: &AppConfig,
    run_config: &RunConfig,
    end_state: Option<MovingState>,
) -> HandlerOutput {
    let rw = &run_config.fence.rand_walk;
    let end_state = end_state.unwrap_or_else(continues_state);

    // Simplified rand_move_state: straight(500) instead of rand_move()
    let s_s = make_straight(500);
    let t_s = make_turn_l(800);

    let mut c = MovingChainComposer::new();
    c.add_state(s_s);
    c.add_transition(make_trans_no_breaker(rw.walk_duration));
    c.add_state(t_s);
    c.add_transition(make_trans_no_breaker(rw.walk_duration));
    c.add_state(end_state.clone());
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

    let mut c = MovingChainComposer::new();
    c.add_state(make_turn_l(sc.turn_speed));
    c.add_transition(make_trans_no_breaker(sc.full_turn_duration));
    c.add_state(end_state.clone());
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
    let aligned = continues_state();

    let align_breaker = if fc.use_mpu_align_direction {
        breakers.make_align_direction_breaker_mpu(app_config, run_config)
    } else {
        breakers.make_std_align_direction_breaker(app_config, run_config)
    };

    let align_state = match fc.direction_align_direction.as_str() {
        "r" => make_turn_r(fc.direction_align_speed),
        "l" => make_turn_l(fc.direction_align_speed),
        _ => make_turn_l(fc.direction_align_speed), // "rand" or default
    };

    let mut align_transition =
        make_trans(fc.max_direction_align_duration, Some(align_breaker));
    // Branch: aligned(True) → aligned state, not aligned(False) → not_aligned state
    align_transition
        .to_states
        .insert(BreakerResult::Bool(true), aligned.id());
    align_transition
        .to_states
        .insert(BreakerResult::Bool(false), not_aligned.id());

    let mut c = MovingChainComposer::new();
    c.add_state(align_state);
    c.add_transition(align_transition);
    let (states, transitions) = c.export();

    HandlerOutput {
        start_state: states.into_iter().next().unwrap_or_else(halt_state),
        normal_exit: aligned,
        abnormal_exit: halt_state(),
        transitions,
    }
}
