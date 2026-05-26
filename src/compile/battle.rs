use crate::compile::make_on_stage_handler;
use crate::compile::stage::make_unclear_zone_handler;
use crate::compile::{HandlerOutput, continues_state, halt_state};
use crate::compile::{make_fence_handler, make_reboot_handler};
use crate::config::{AppConfig, RunConfig};
use crate::constant::StageCodeSign;
use crate::judgers::Breakers;
use mentabotix_rs::composer::MovingChainComposer;
use mentabotix_rs::registry::CaseRegistry;
use mentabotix_rs::transition::BreakerResult;
use std::collections::HashMap;

use mentabotix_rs::transition::MovingTransition;

pub fn make_std_battle_handler(app_config: &AppConfig, run_config: &RunConfig) -> HandlerOutput {
    let breakers = Breakers::null();
    let end_state = halt_state();
    let start_state = continues_state();

    let stage_breaker = breakers.make_std_stage_breaker(app_config, run_config);

    let reboot_output = make_reboot_handler(app_config, run_config, Some(end_state.clone()));
    let fence_output = make_fence_handler(app_config, run_config, None, Some(end_state.clone()));
    let on_stage_output =
        make_on_stage_handler(app_config, run_config, None, Some(end_state.clone()));
    let unclear_output = make_unclear_zone_handler(app_config, run_config, Some(end_state.clone()));

    let mut transition_pool: Vec<MovingTransition> = Vec::new();
    transition_pool.extend(reboot_output.transitions);
    transition_pool.extend(fence_output.transitions);
    transition_pool.extend(on_stage_output.transitions);
    transition_pool.extend(unclear_output.transitions);

    let mut case_reg = CaseRegistry::<StageCodeSign>::new();
    case_reg
        .batch_register(
            [
                StageCodeSign::ON_STAGE_REBOOT,
                StageCodeSign::OFF_STAGE_REBOOT,
                StageCodeSign::UNCLEAR_ZONE_REBOOT,
            ],
            reboot_output.start_state.id(),
        )
        .ok();
    case_reg
        .register(StageCodeSign::ON_STAGE, on_stage_output.start_state.id())
        .ok();
    case_reg
        .register(StageCodeSign::OFF_STAGE, fence_output.start_state.id())
        .ok();
    case_reg
        .register(StageCodeSign::UNCLEAR_ZONE, unclear_output.start_state.id())
        .ok();

    let to_states: HashMap<BreakerResult, usize> = case_reg
        .export()
        .into_iter()
        .map(|(k, v)| (BreakerResult::Int(k as i64), v))
        .collect();

    let mut composer = MovingChainComposer::new();
    composer.add_state(start_state.clone());
    let mut head = MovingTransition::new(run_config.perf.checking_duration)
        .unwrap()
        .with_arc_breaker(stage_breaker);
    for (key, state_id) in &to_states {
        head.to_states.insert(key.clone(), *state_id);
    }
    composer.add_transition(head);
    let (_, mut composer_trans) = composer.export();
    transition_pool.append(&mut composer_trans);

    HandlerOutput {
        start_state,
        normal_exit: end_state,
        transitions: transition_pool,
    }
}

pub fn make_always_on_stage_battle_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> HandlerOutput {
    let breakers = Breakers::null();
    let end_state = halt_state();
    let start_state = continues_state();

    let stage_breaker = breakers.make_always_on_stage_breaker(app_config, run_config);
    let on_stage_output =
        make_on_stage_handler(app_config, run_config, None, Some(end_state.clone()));

    let mut transition_pool = on_stage_output.transitions;

    let mut composer = MovingChainComposer::new();
    composer.add_state(start_state.clone());
    let mut check_trans = MovingTransition::new(run_config.perf.checking_duration)
        .unwrap()
        .with_arc_breaker(stage_breaker);
    check_trans
        .to_states
        .insert(BreakerResult::Int(0), on_stage_output.start_state.id());
    composer.add_transition(check_trans);
    let (_, mut composer_trans) = composer.export();
    transition_pool.append(&mut composer_trans);

    HandlerOutput {
        start_state,
        normal_exit: end_state,
        transitions: transition_pool,
    }
}

pub fn make_always_off_stage_battle_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> HandlerOutput {
    let breakers = Breakers::null();
    let end_state = halt_state();
    let start_state = continues_state();

    let stage_breaker = breakers.make_always_off_stage_breaker(app_config, run_config);
    let reboot_output = make_reboot_handler(app_config, run_config, Some(end_state.clone()));
    let fence_output = make_fence_handler(app_config, run_config, None, Some(end_state.clone()));

    let mut transition_pool: Vec<MovingTransition> = Vec::new();
    transition_pool.extend(reboot_output.transitions);
    transition_pool.extend(fence_output.transitions);

    let mut composer = MovingChainComposer::new();
    composer.add_state(start_state.clone());
    let mut check_trans = MovingTransition::new(run_config.perf.checking_duration)
        .unwrap()
        .with_arc_breaker(stage_breaker);
    check_trans
        .to_states
        .insert(BreakerResult::Int(1), fence_output.start_state.id()); // OFF_STAGE
    check_trans
        .to_states
        .insert(BreakerResult::Int(3), reboot_output.start_state.id()); // OFF_STAGE_REBOOT
    composer.add_transition(check_trans);
    let (_, mut composer_trans) = composer.export();
    transition_pool.append(&mut composer_trans);

    HandlerOutput {
        start_state,
        normal_exit: end_state,
        transitions: transition_pool,
    }
}
