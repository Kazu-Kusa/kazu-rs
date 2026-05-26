use crate::compile::{
    HandlerOutput, continues_state, halt_state, make_straight, make_trans, make_turn_l, make_turn_r,
};
use crate::compile::{
    make_align_direction_handler, make_back_to_stage_handler, make_rand_walk_handler,
};
use crate::config::{AppConfig, RunConfig};
use crate::constant::FenceCodeSign;
use crate::judgers::Breakers;
use mentabotix_rs::composer::MovingChainComposer;
use mentabotix_rs::registry::CaseRegistry;
use mentabotix_rs::transition::BreakerResult;
use std::collections::HashMap;
use std::sync::Arc;

use mentabotix_rs::state::MovingState;
use mentabotix_rs::transition::MovingTransition;

pub fn make_fence_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    start_state: Option<MovingState>,
    stop_state: Option<MovingState>,
) -> HandlerOutput {
    let breakers = Breakers::null();
    let start_state = start_state.unwrap_or_else(continues_state);
    let stop_state = stop_state.unwrap_or_else(halt_state);
    let fc = &run_config.fence;

    // Sub-handlers
    let back_stage_output =
        make_back_to_stage_handler(app_config, run_config, Some(stop_state.clone()));
    let rand_move_output = make_rand_walk_handler(app_config, run_config, Some(stop_state.clone()));
    let rand_move_head_state = rand_move_output.start_state.clone();
    let align_direction_output =
        make_align_direction_handler(app_config, run_config, Some(rand_move_head_state.clone()));

    // Breakers
    let fence_breaker = breakers.make_std_fence_breaker(app_config, run_config);
    let lr_blocked_breaker = breakers.make_lr_sides_blocked_breaker(app_config, run_config);
    let align_stage_breaker = if fc.use_mpu_align_stage {
        breakers.make_stage_align_breaker_mpu(app_config, run_config)
    } else {
        breakers.make_std_stage_align_breaker(app_config, run_config)
    };

    // Shared states
    let rear_exit_corner_state = make_straight(-fc.exit_corner_speed);
    let front_exit_corner_state = make_straight(fc.exit_corner_speed);

    let align_state = match fc.stage_align_direction.as_str() {
        "r" => make_turn_r(fc.stage_align_speed),
        "l" => make_turn_l(fc.stage_align_speed),
        _ => make_turn_l(fc.stage_align_speed), // "rand" simplified
    };

    // Collect all transitions from sub-handlers
    let mut transitions_pool: Vec<MovingTransition> = Vec::new();
    transitions_pool.extend(back_stage_output.transitions);
    transitions_pool.extend(rand_move_output.transitions);
    transitions_pool.extend(align_direction_output.transitions);

    let mut case_reg = CaseRegistry::<FenceCodeSign>::new();

    // Case 1: X_O_O_O → back_stage_pack
    case_reg
        .register(FenceCodeSign::X_O_O_O, back_stage_output.start_state.id())
        .ok();

    // Helper to build an exit_t (must be called once per use since MovingTransition is not Clone)
    let make_exit_t = || -> MovingTransition {
        make_trans(
            fc.max_exit_corner_duration,
            Some(Arc::clone(&lr_blocked_breaker)),
        )
    };

    // Case 2: O_X_O_O, O_O_X_O, O_O_O_X, O_O_X_X, X_X_O_O → align_state → align_stage_transition → back_stage_pack
    {
        let mut align_t = make_trans(fc.max_stage_align_duration, Some(align_stage_breaker));
        // Branch: False → rand_move_head_state, default → back_stage head
        align_t
            .to_states
            .insert(BreakerResult::Bool(false), rand_move_head_state.id());
        align_t.to_states.insert(
            BreakerResult::Placeholder,
            back_stage_output.start_state.id(),
        );

        let mut c = MovingChainComposer::new();
        c.add_state(align_state.clone());
        c.add_transition(align_t);
        let (align_states, align_trans) = c.export();

        if let Some(head) = align_states.first() {
            case_reg
                .batch_register(
                    [
                        FenceCodeSign::O_X_O_O,
                        FenceCodeSign::O_O_X_O,
                        FenceCodeSign::O_O_O_X,
                        FenceCodeSign::O_O_X_X,
                        FenceCodeSign::X_X_O_O,
                    ],
                    head.id(),
                )
                .ok();
        }
        transitions_pool.extend(align_trans);
    }

    // Case 3: O_X_O_X, O_X_X_O → front_exit_corner → exit_t → stop_state
    {
        let mut c = MovingChainComposer::new();
        c.add_state(front_exit_corner_state.clone());
        c.add_transition(make_exit_t());
        c.add_state(stop_state.clone());
        let (states, trans) = c.export();
        if let Some(head) = states.first() {
            case_reg
                .batch_register([FenceCodeSign::O_X_O_X, FenceCodeSign::O_X_X_O], head.id())
                .ok();
        }
        transitions_pool.extend(trans);
    }

    // Case 4: X_O_O_X, X_O_X_O → rear_exit_corner → exit_t → stop_state
    {
        let mut c = MovingChainComposer::new();
        c.add_state(rear_exit_corner_state.clone());
        c.add_transition(make_exit_t());
        c.add_state(stop_state.clone());
        let (states, trans) = c.export();
        if let Some(head) = states.first() {
            case_reg
                .batch_register([FenceCodeSign::X_O_O_X, FenceCodeSign::X_O_X_O], head.id())
                .ok();
        }
        transitions_pool.extend(trans);
    }

    // Case 5: O_X_X_X, X_O_X_X, X_X_O_X, X_X_X_O → align_direction_pack
    case_reg
        .batch_register(
            [
                FenceCodeSign::O_X_X_X,
                FenceCodeSign::X_O_X_X,
                FenceCodeSign::X_X_O_X,
                FenceCodeSign::X_X_X_O,
            ],
            align_direction_output.start_state.id(),
        )
        .ok();

    // Case 6: O_O_O_O, X_X_X_X → rand_move_pack
    case_reg
        .batch_register(
            [FenceCodeSign::O_O_O_O, FenceCodeSign::X_X_X_X],
            rand_move_output.start_state.id(),
        )
        .ok();

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
        normal_exit: stop_state,
        transitions: transitions_pool,
    }
}
