//! Mission assembly — port of `kazu/assembly.py`.
//!
//! Wires compile handlers into complete missions for each run mode
//! (FGS, NGS, AFG, ANG, FGDL).
//!
// TODO: wire into cmd_run for mode-based mission dispatch.
#![allow(dead_code)]

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
    let stage_output = compile::make_on_stage_handler(
        app_config, run_config, None, None, None,
    );
    stage_output.transitions
}

/// Assemble the Full-Game-Start (FGS) mission schema.
pub fn assembly_fgs_schema(
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> (Vec<MovingTransition>, Vec<MovingTransition>) {
    let boot_output = compile::make_reboot_handler(app_config, run_config, None);
    let stage_output = compile::make_on_stage_handler(
        app_config, run_config, None, None, None,
    );
    (boot_output.transitions, stage_output.transitions)
}

/// Assemble the Full-Game-Dash-Loop (FGDL) mission schema.
pub fn assembly_fgdl_schema(
    app_config: &AppConfig,
    run_config: &RunConfig,
) -> Vec<MovingTransition> {
    let reboot_output = compile::make_reboot_handler(app_config, run_config, None);
    reboot_output.transitions
}
