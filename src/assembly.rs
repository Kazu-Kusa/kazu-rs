//! Mission assembly — port of `kazu/assembly.py`.
//!
//! Wires compile handlers into complete missions for each run mode
//! (FGS, NGS, AFG, ANG, FGDL).
//!

use crate::compile;
use crate::config::{AppConfig, RunConfig};
use mentabotix_rs::transition::MovingTransition;

/// Assemble the Always-Off-Stage (AFG) mission schema.
pub fn assembly_afg_schema(
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Vec<MovingTransition> {
    let output = compile::make_always_off_stage_battle_handler(app_config, run_config);
    output.transitions
}

/// Assemble the Always-On-Stage (ANG) mission schema.
pub fn assembly_ang_schema(
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Vec<MovingTransition> {
    let output = compile::make_always_on_stage_battle_handler(app_config, run_config);
    output.transitions
}

/// Assemble the Normal-Game-Start (NGS) mission schema.
pub fn assembly_ngs_schema(
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Vec<MovingTransition> {
    let output = compile::make_on_stage_handler(
        app_config, run_config, None, None,
    );
    // The handler builds a complete stage-check loop:
    //   start_state → stage check → ON_STAGE battle / UNCLEAR recovery / OFF_STAGE exit
    //   with loop-back transitions from battle and unclear back to the stage check.
    // All transitions, plus start_state/normal_exit/abnormal_exit anchors, are
    // embedded in the returned transition graph.
    output.transitions
}

/// Assemble the Full-Game-Start (FGS) mission schema.
pub fn assembly_fgs_schema(
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> (Vec<MovingTransition>, Vec<MovingTransition>) {
    let boot_output = compile::make_reboot_handler(app_config, run_config, None);
    let stage_output = compile::make_on_stage_handler(
        app_config, run_config, None, None,
    );

    let mut boot_transitions = boot_output.transitions;
    let stage_transitions = stage_output.transitions;

    // Chain: boot normal_exit → stage start_state
    let chain_dur = run_config.perf.checking_duration;
    let chain_trans = MovingTransition::new(chain_dur)
        .unwrap()
        .with_from_state(boot_output.normal_exit.id())
        .with_single_to_state(stage_output.start_state.id());
    boot_transitions.push(chain_trans);

    (boot_transitions, stage_transitions)
}

/// Assemble the Full-Game-Dash-Loop (FGDL) mission schema.
pub fn assembly_fgdl_schema(
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Vec<MovingTransition> {
    let reboot_output = compile::make_reboot_handler(app_config, run_config, None);
    let mut transitions = reboot_output.transitions;

    // Self-loop: reboot normal_exit → reboot start_state
    let loop_dur = run_config.perf.checking_duration;
    let loop_trans = MovingTransition::new(loop_dur)
        .unwrap()
        .with_from_state(reboot_output.normal_exit.id())
        .with_single_to_state(reboot_output.start_state.id());
    transitions.push(loop_trans);

    transitions
}
