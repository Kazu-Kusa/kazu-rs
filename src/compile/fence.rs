use crate::compile::{continues_state, halt_state, make_straight, make_turn_l, make_trans, make_trans_no_breaker, HandlerOutput};
use crate::compile::make_align_direction_handler;
use crate::config::{AppConfig, RunConfig};
use crate::constant::FenceCodeSign;
use crate::judgers::Breakers;
use mentabotix_rs::composer::MovingChainComposer;
use mentabotix_rs::registry::CaseRegistry;
use mentabotix_rs::transition::BreakerResult;
use std::collections::HashMap;

use mentabotix_rs::state::MovingState;
use mentabotix_rs::transition::MovingTransition;
pub fn make_fence_handler(
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

    let fc = &run_config.fence;
    let fence_breaker = breakers.make_std_fence_breaker(app_config, run_config);
    let lr_blocked_breaker = breakers.make_lr_sides_blocked_breaker(app_config, run_config);
    // Invert: transition fires when at least one side clears (not both blocked)
    let lr_clear_breaker: std::sync::Arc<dyn Fn() -> BreakerResult + Send + Sync> = {
        let inner = std::sync::Arc::clone(&lr_blocked_breaker);
        std::sync::Arc::new(move || match inner() {
            BreakerResult::Bool(b) => BreakerResult::Bool(!b),
            other => other,
        })
    };

    let rnd_s = || make_turn_l(fc.stage_align_speed); // simplified
    let exit_s = || make_straight(fc.exit_corner_speed);

    let ft_t = || make_trans_no_breaker(fc.max_direction_align_duration);
    let exit_t = || make_trans_no_breaker(fc.max_exit_corner_duration);
    let lr_check_t = || make_trans(fc.max_direction_align_duration, Some(std::sync::Arc::clone(&lr_clear_breaker)));
    let stage_align_breaker: Option<std::sync::Arc<dyn Fn() -> BreakerResult + Send + Sync>> =
        if !fc.use_mpu_align_stage {
            Some(breakers.make_std_stage_align_breaker(app_config, run_config))
        } else {
            None // Blocked on: MPU6500 hardware; no-op breaker for now
        };
    let st_align_t = || make_trans(fc.max_stage_align_duration, stage_align_breaker.clone());

    let abn = || halt_state();
    let align_dir_output =
        make_align_direction_handler(app_config, run_config, Some(abnormal_exit.clone()));

    let mut transitions_pool: Vec<MovingTransition> = Vec::new();
    transitions_pool.extend(align_dir_output.transitions);
    let mut case_reg = CaseRegistry::<FenceCodeSign>::new();

    case_reg.register(FenceCodeSign::O_O_O_O, normal_exit.id()).ok();

    macro_rules! fence_case {
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

    fence_case!(case_reg, &[FenceCodeSign::X_O_O_O, FenceCodeSign::O_X_O_O], [
        add_s(rnd_s()), add_t(st_align_t()), add_s(exit_s()), add_t(exit_t()), add_s(abn()),
    ]);
    // Direction-alignment cases: delegate to align_direction_handler
    for sign in &[
        FenceCodeSign::O_O_X_O, FenceCodeSign::X_O_X_O,
        FenceCodeSign::O_X_X_O, FenceCodeSign::X_X_X_O,
        FenceCodeSign::O_O_O_X, FenceCodeSign::X_O_O_X,
        FenceCodeSign::O_X_O_X, FenceCodeSign::X_X_O_X,
    ] {
        case_reg.register(*sign, align_dir_output.start_state.id()).ok();
    }
    // LR-blocked cases: use lr_check_t to wait until at least one side clears
    fence_case!(case_reg, &[FenceCodeSign::O_O_X_X, FenceCodeSign::X_O_X_X,
        FenceCodeSign::O_X_X_X, FenceCodeSign::X_X_X_X], [
        add_s(rnd_s()), add_t(lr_check_t()), add_s(exit_s()), add_t(exit_t()), add_s(abn()),
    ]);
    // Non-LR-blocked multi-trigger: just random turn and exit
    fence_case!(case_reg, &[FenceCodeSign::X_X_O_O, FenceCodeSign::X_X_O_X], [
        add_s(rnd_s()), add_t(ft_t()), add_s(exit_s()), add_t(exit_t()), add_s(abn()),
    ]);

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
        normal_exit,
        abnormal_exit,
        transitions: transitions_pool,
    }
}
