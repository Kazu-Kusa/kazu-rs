use crate::compile::{continues_state, halt_state, make_straight, make_turn_l, make_trans, make_trans_no_breaker, HandlerOutput};
use crate::config::{AppConfig, RunConfig};
use crate::judgers::Breakers;
use mentabotix_rs::composer::MovingChainComposer;
use mentabotix_rs::transition::BreakerResult;

use mentabotix_rs::transition::MovingTransition;
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

pub fn make_always_on_stage_battle_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> HandlerOutput {
    

    let breakers = Breakers::null();
    let always_on = breakers.make_always_on_stage_breaker(app_config, run_config);
    let battle = make_std_battle_handler(app_config, run_config);

    // Stage gate: always ON_STAGE (0) → enter battle loop
    let gate_start = continues_state();
    let mut gate_trans = MovingTransition::new(run_config.perf.checking_duration)
        .unwrap()
        .with_arc_breaker(always_on);
    gate_trans.to_states.insert(BreakerResult::Int(0), battle.start_state.id()); // ON_STAGE
    gate_trans.from_states.push(gate_start.id());

    let mut transitions = battle.transitions;
    transitions.push(gate_trans);

    HandlerOutput {
        start_state: gate_start,
        normal_exit: halt_state(),
        abnormal_exit: halt_state(),
        transitions,
    }
}

pub fn make_always_off_stage_battle_handler(
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> HandlerOutput {
    

    let breakers = Breakers::null();
    let always_off = breakers.make_always_off_stage_breaker(app_config, run_config);
    let battle = make_std_battle_handler(app_config, run_config);

    // Stage gate: always OFF_STAGE (1); maps both outcomes to battle
    // so that off-stage mode still engages (preserving existing behavior).
    let gate_start = continues_state();
    let mut gate_trans = MovingTransition::new(run_config.perf.checking_duration)
        .unwrap()
        .with_arc_breaker(always_off);
    gate_trans.to_states.insert(BreakerResult::Int(0), battle.start_state.id()); // ON_STAGE
    gate_trans.to_states.insert(BreakerResult::Int(1), battle.start_state.id()); // OFF_STAGE
    gate_trans.from_states.push(gate_start.id());

    let mut transitions = battle.transitions;
    transitions.push(gate_trans);

    HandlerOutput {
        start_state: gate_start,
        normal_exit: halt_state(),
        abnormal_exit: halt_state(),
        transitions,
    }
}

