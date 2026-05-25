use crate::compile::{continues_state, halt_state, make_straight, make_turn_l, make_turn_r, make_trans, HandlerOutput};
use crate::config::{AppConfig, RunConfig};
use crate::constant::SurroundingCodeSign;
use crate::judgers::Breakers;
use mentabotix_rs::composer::MovingChainComposer;
use mentabotix_rs::registry::CaseRegistry;
use mentabotix_rs::transition::BreakerResult;
use std::collections::HashMap;

use mentabotix_rs::state::MovingState;
use mentabotix_rs::transition::MovingTransition;
pub fn make_surrounding_handler(
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

    let sc = &run_config.surrounding;

    let tag_group = crate::config::TagGroup::new(&run_config.team_color);
    let _query_table = crate::static_utils::make_query_table(&tag_group);
    let surr_full_breaker = breakers.make_surr_breaker(
        app_config, run_config,
        &tag_group,
    );
    let atk_breaker = if sc.atk_break_use_edge_sensors {
        breakers.make_atk_breaker_with_edge_sensors(app_config, run_config)
    } else {
        breakers.make_std_atk_breaker(app_config, run_config)
    };
    let edge_rear_breaker = breakers.make_std_edge_rear_breaker(app_config, run_config);
    let turn_to_front_breaker = breakers.make_std_turn_to_front_breaker(app_config, run_config);

    // State factories
    let aec = || make_straight(sc.atk_speed_enemy_car);
    let aeb = || make_straight(sc.atk_speed_enemy_box);
    let anb = || make_straight(sc.atk_speed_neutral_box);
    let afb = || make_straight(-sc.fallback_speed_ally_box);
    let efb = || make_straight(-sc.fallback_speed_edge);
    let lt = || make_turn_l(sc.turn_speed);
    let rt = || make_turn_r(sc.turn_speed);
    let rnd = || make_turn_l(sc.turn_speed); // simplified rand

    // Transition factories
    let aect = || make_trans(sc.atk_enemy_car_duration, Some(atk_breaker.clone()));
    let aebt = || make_trans(sc.atk_enemy_box_duration, Some(atk_breaker.clone()));
    let anbt = || make_trans(sc.atk_neutral_box_duration, Some(atk_breaker.clone()));
    let afbt = || make_trans(sc.fallback_duration_ally_box, Some(edge_rear_breaker.clone()));
    let efbt = || make_trans(sc.fallback_duration_edge, Some(edge_rear_breaker.clone()));
    let ftt = || make_trans(sc.full_turn_duration, Some(turn_to_front_breaker.clone()));
    let htt = || make_trans(sc.half_turn_duration, Some(turn_to_front_breaker.clone()));

    let abn = || halt_state();

    let mut transitions_pool: Vec<MovingTransition> = Vec::new();
    let mut case_reg = CaseRegistry::<SurroundingCodeSign>::new();

    case_reg.register(SurroundingCodeSign::NOTHING, normal_exit.id()).ok();

    macro_rules! surr_case {
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

    // Front enemy car
    surr_case!(case_reg, &[
        SurroundingCodeSign::FRONT_ENEMY_CAR,
        SurroundingCodeSign::FRONT_ENEMY_CAR_RIGHT_OBJECT,
        SurroundingCodeSign::FRONT_ENEMY_CAR_LEFT_OBJECT,
        SurroundingCodeSign::FRONT_ENEMY_CAR_BEHIND_OBJECT,
        SurroundingCodeSign::FRONT_ENEMY_CAR_LEFT_RIGHT_OBJECTS,
        SurroundingCodeSign::FRONT_ENEMY_CAR_RIGHT_BEHIND_OBJECTS,
        SurroundingCodeSign::FRONT_ENEMY_CAR_LEFT_BEHIND_OBJECTS,
        SurroundingCodeSign::FRONT_ENEMY_CAR_LEFT_RIGHT_BEHIND_OBJECTS,
    ], [
        add_s(aec()), add_t(aect()), add_s(efb()), add_t(efbt()),
        add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);

    // Target switch: behind
    surr_case!(case_reg, &[
        SurroundingCodeSign::BEHIND_OBJECT,
        SurroundingCodeSign::LEFT_RIGHT_BEHIND_OBJECTS,
        SurroundingCodeSign::FRONT_ENEMY_BOX_BEHIND_OBJECT,
        SurroundingCodeSign::FRONT_ALLY_BOX_BEHIND_OBJECT,
        SurroundingCodeSign::FRONT_NEUTRAL_BOX_BEHIND_OBJECT,
        SurroundingCodeSign::FRONT_ALLY_BOX_LEFT_RIGHT_BEHIND_OBJECTS,
        SurroundingCodeSign::FRONT_NEUTRAL_BOX_LEFT_RIGHT_BEHIND_OBJECTS,
        SurroundingCodeSign::FRONT_ENEMY_BOX_LEFT_RIGHT_BEHIND_OBJECTS,
    ], [
        add_s(rnd()), add_t(ftt()), add_s(aec()), add_t(aect()),
        add_s(efb()), add_t(efbt()), add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);

    // LEFT_BEHIND_OBJECTS: random spd turn left
    surr_case!(case_reg, &[
        SurroundingCodeSign::LEFT_BEHIND_OBJECTS,
        SurroundingCodeSign::FRONT_ENEMY_BOX_LEFT_BEHIND_OBJECTS,
        SurroundingCodeSign::FRONT_ALLY_BOX_LEFT_BEHIND_OBJECTS,
        SurroundingCodeSign::FRONT_NEUTRAL_BOX_LEFT_BEHIND_OBJECTS,
    ], [
        add_s(rnd()), add_t(ftt()), add_s(aec()), add_t(aect()),
        add_s(efb()), add_t(efbt()), add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);

    // RIGHT_BEHIND_OBJECTS: random spd turn right
    surr_case!(case_reg, &[
        SurroundingCodeSign::RIGHT_BEHIND_OBJECTS,
        SurroundingCodeSign::FRONT_ENEMY_BOX_RIGHT_BEHIND_OBJECTS,
        SurroundingCodeSign::FRONT_ALLY_BOX_RIGHT_BEHIND_OBJECTS,
        SurroundingCodeSign::FRONT_NEUTRAL_BOX_RIGHT_BEHIND_OBJECTS,
    ], [
        add_s(rnd()), add_t(ftt()), add_s(aec()), add_t(aect()),
        add_s(efb()), add_t(efbt()), add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);

    // Left object
    surr_case!(case_reg, &[
        SurroundingCodeSign::LEFT_OBJECT,
        SurroundingCodeSign::FRONT_ENEMY_BOX_LEFT_OBJECT,
        SurroundingCodeSign::FRONT_ALLY_BOX_LEFT_OBJECT,
        SurroundingCodeSign::FRONT_NEUTRAL_BOX_LEFT_OBJECT,
    ], [
        add_s(lt()), add_t(htt()), add_s(aec()), add_t(aect()),
        add_s(efb()), add_t(efbt()), add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);

    // Right object
    surr_case!(case_reg, &[
        SurroundingCodeSign::RIGHT_OBJECT,
        SurroundingCodeSign::FRONT_ENEMY_BOX_RIGHT_OBJECT,
        SurroundingCodeSign::FRONT_ALLY_BOX_RIGHT_OBJECT,
        SurroundingCodeSign::FRONT_NEUTRAL_BOX_RIGHT_OBJECT,
    ], [
        add_s(rt()), add_t(htt()), add_s(aec()), add_t(aect()),
        add_s(efb()), add_t(efbt()), add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);

    // Left+right objects
    surr_case!(case_reg, &[
        SurroundingCodeSign::LEFT_RIGHT_OBJECTS,
        SurroundingCodeSign::FRONT_ENEMY_BOX_LEFT_RIGHT_OBJECTS,
        SurroundingCodeSign::FRONT_ALLY_BOX_LEFT_RIGHT_OBJECTS,
        SurroundingCodeSign::FRONT_NEUTRAL_BOX_LEFT_RIGHT_OBJECTS,
    ], [
        add_s(rnd()), add_t(htt()), add_s(aec()), add_t(aect()),
        add_s(efb()), add_t(efbt()), add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);

    // Front box only
    surr_case!(case_reg, &[
        SurroundingCodeSign::FRONT_ENEMY_BOX,
    ], [
        add_s(aeb()), add_t(aebt()), add_s(efb()), add_t(efbt()),
        add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);
    surr_case!(case_reg, &[
        SurroundingCodeSign::FRONT_NEUTRAL_BOX,
    ], [
        add_s(anb()), add_t(anbt()), add_s(efb()), add_t(efbt()),
        add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);
    surr_case!(case_reg, &[
        SurroundingCodeSign::FRONT_ALLY_BOX,
    ], [
        add_s(afb()), add_t(afbt()), add_s(rnd()), add_t(ftt()), add_s(abn()),
    ]);

    // Assembly: top-level branching
    let mut composer = MovingChainComposer::new();
    composer.add_state(start_state.clone());

    let to_states: HashMap<BreakerResult, usize> = case_reg
        .export()
        .into_iter()
        .map(|(k, v)| (BreakerResult::Int(k as i64), v))
        .collect();

    let mut head = MovingTransition::new(run_config.perf.checking_duration)
        .unwrap()
        .with_arc_breaker(surr_full_breaker);
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
