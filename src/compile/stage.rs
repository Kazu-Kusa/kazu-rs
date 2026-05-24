use crate::compile::{continues_state, halt_state, make_trans, HandlerOutput};
use crate::compile::make_surrounding_handler;
use crate::config::{AppConfig, RunConfig};
use crate::constant::StageCodeSign;
use crate::judgers::Breakers;
use mentabotix_rs::composer::MovingChainComposer;
use mentabotix_rs::registry::CaseRegistry;
use mentabotix_rs::transition::BreakerResult;
use std::collections::HashMap;

use mentabotix_rs::state::MovingState;
use mentabotix_rs::transition::MovingTransition;
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

    // Sub-handlers: battle (ON_STAGE) and unclear zone recovery
    let surrounding_output = make_surrounding_handler(app_config, run_config, None, None, None);
    let unclear_output = make_unclear_zone_handler(app_config, run_config, None);

    let mut transitions_pool: Vec<MovingTransition> = Vec::new();

    // Merge sub-handler transition graphs
    transitions_pool.extend(surrounding_output.transitions);
    transitions_pool.extend(unclear_output.transitions);

    let loop_dur = run_config.perf.checking_duration;

    // Loop-back: after battle ends (nothing detected), check stage again
    let loop_surr = MovingTransition::new(loop_dur)
        .unwrap()
        .with_from_state(surrounding_output.normal_exit.id())
        .with_single_to_state(start_state.id());
    transitions_pool.push(loop_surr);

    // Loop-back: after unclear zone cleared, check stage again
    let loop_unclear = MovingTransition::new(loop_dur)
        .unwrap()
        .with_from_state(unclear_output.normal_exit.id())
        .with_single_to_state(start_state.id());
    transitions_pool.push(loop_unclear);

    // Stage-check branching: map every StageCodeSign variant
    let mut case_reg = CaseRegistry::<StageCodeSign>::new();

    // ON_STAGE (0) and ON_STAGE_REBOOT (2) → enter battle
    case_reg.register(StageCodeSign::ON_STAGE, surrounding_output.start_state.id()).ok();
    case_reg.register(StageCodeSign::ON_STAGE_REBOOT, surrounding_output.start_state.id()).ok();
    // OFF_STAGE (1) and OFF_STAGE_REBOOT (3) → exit the loop
    case_reg.register(StageCodeSign::OFF_STAGE, normal_exit.id()).ok();
    case_reg.register(StageCodeSign::OFF_STAGE_REBOOT, normal_exit.id()).ok();
    // UNCLEAR_ZONE (4) and UNCLEAR_ZONE_REBOOT (6) → unclear zone handler
    case_reg.register(StageCodeSign::UNCLEAR_ZONE, unclear_output.start_state.id()).ok();
    case_reg.register(StageCodeSign::UNCLEAR_ZONE_REBOOT, unclear_output.start_state.id()).ok();

    // Assemble: start_state → head transition (stage check with breaker)
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

// Blocked on: live ADC sensor access; currently uses fixed 0.0.
/// Handler for unclear zone (between on-stage and off-stage).
pub fn make_unclear_zone_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    normal_exit: Option<MovingState>,
) -> HandlerOutput {
    

    let sc = &run_config.stage;
    let normal_exit = normal_exit.unwrap_or_else(continues_state);

    // Blocked on: live ADC sensor access.
    // Currently uses fixed 0.0; unclear_zone_tolerance check is a no-op.
    let _gray_idx = app_config.sensor.gray_adc_index as usize;
    let tolerance = sc.unclear_zone_tolerance as f64;

    // Shared state: hook records initial gray, breaker compares against it.
    let recorded_gray: std::sync::Arc<std::sync::Mutex<Option<f64>>> =
        std::sync::Arc::new(std::sync::Mutex::new(None));

    // Hook: capture initial gray ADC value on entry.
    let recorded = std::sync::Arc::clone(&recorded_gray);
    let start_state = continues_state().with_before_entering(move || {
        // Blocked on: live ADC sensor access.
        if let Ok(mut guard) = recorded.lock() {
            *guard = Some(0.0);
        }
    });

    // Breaker: exit unclear zone when gray ADC deviates beyond tolerance.
    let recorded = std::sync::Arc::clone(&recorded_gray);
    let unclear_breaker: std::sync::Arc<dyn Fn() -> BreakerResult + Send + Sync> =
        std::sync::Arc::new(move || {
            // Blocked on: live ADC sensor access.
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
