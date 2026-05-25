use crate::compile::{continues_state, halt_state, HandlerOutput};
use crate::compile::{make_edge_handler, make_surrounding_handler, make_search_handler};
use crate::config::{AppConfig, RunConfig};

use mentabotix_rs::state::MovingState;
use mentabotix_rs::transition::MovingTransition;
use crate::compile::make_trans;
use mentabotix_rs::composer::MovingChainComposer;
use mentabotix_rs::transition::BreakerResult;

pub fn make_on_stage_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
    start_state: Option<MovingState>,
    abnormal_exit: Option<MovingState>,
) -> HandlerOutput {
    let strategy = &run_config.strategy;
    let start_state = start_state.unwrap_or_else(continues_state);
    let abnormal_exit = abnormal_exit.unwrap_or_else(halt_state);

    let mut transitions: Vec<MovingTransition> = Vec::new();
    let mut concat_state = start_state.clone();

    if strategy.use_edge_component {
        let edge_output = make_edge_handler(
            app_config,
            run_config,
            Some(concat_state.clone()),
            None,
            Some(abnormal_exit.clone()),
        );
        transitions.extend(edge_output.transitions);
        concat_state = edge_output.normal_exit;
    }

    if strategy.use_surrounding_component {
        let surr_output = make_surrounding_handler(
            app_config,
            run_config,
            Some(concat_state.clone()),
            None,
            Some(abnormal_exit.clone()),
        );
        transitions.extend(surr_output.transitions);
        concat_state = surr_output.normal_exit;
    }

    if strategy.use_normal_component {
        let search_output = make_search_handler(
            app_config,
            run_config,
            Some(concat_state.clone()),
            Some(abnormal_exit.clone()),
            None,
        );
        transitions.extend(search_output.transitions);
    }

    HandlerOutput {
        start_state,
        normal_exit: concat_state,
        abnormal_exit,
        transitions,
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
