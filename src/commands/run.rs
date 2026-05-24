use crate::cli::RunModeArg;
use crate::compile;
use crate::config::{load_run_config, AppConfig};
use log::{error, info, warn};
use mentabotix_rs::botix::Botix;
use mentabotix_rs::state::MovingState;
use std::path::PathBuf;

#[allow(clippy::too_many_arguments)]
pub fn cmd_run(
    app_config: AppConfig,
    port: Option<String>,
    run_config_path: Option<PathBuf>,
    mode: RunModeArg,
    _disable_camera: bool,
    _camera: Option<i32>,
    _camera_res_mul: Option<f64>,
    _team_color: Option<String>,
) {
    use bdmc_rs::controller::CloseLoopController;

    let run_config = run_config_path
        .as_deref()
        .map(load_run_config)
        .unwrap_or_default();

    let effective_port = port.or_else(|| {
        if app_config.motion.port != crate::config::default_port() {
            Some(app_config.motion.port.clone())
        } else {
            None
        }
    });

    info!(
        "Run mode: {:?}, team: {}, port: {:?}",
        mode, run_config.team_color, effective_port
    );
    info!(
        "Missions: boot={:?}, stage={:?}, off_stage={:?}",
        run_config.missions.boot.len(),
        run_config.missions.stage.len(),
        run_config.missions.off_stage.len()
    );

    let controller = match CloseLoopController::new(None, None, None, effective_port.as_deref()) {
        Ok(c) => c,
        Err(e) => {
            error!("Failed to initialize controller: {}", e);
            return;
        }
    };

    let mut all_states: Vec<MovingState> = Vec::new();
    let mut all_transitions = Vec::new();

    let pack_names = match mode {
        RunModeArg::Fgs | RunModeArg::Fgdl => &run_config.missions.boot,
        RunModeArg::Ngs | RunModeArg::Ang => &run_config.missions.stage,
        RunModeArg::Afg => &run_config.missions.off_stage,
    };

    for pack_name in pack_names {
        match compile::get_handler(pack_name) {
            Some(handler) => {
                let transitions = handler(&app_config, &run_config);
                all_transitions.extend(transitions);
            }
            None => {
                warn!("Unknown pack: {}", pack_name);
            }
        }
    }

    if all_transitions.is_empty() {
        info!("No transitions from handlers; using demo mission.");
        let s_start = MovingState::straight(300);
        let s_halt = MovingState::halt();
        all_states.push(s_start);
        all_states.push(s_halt);

        let t_drive = mentabotix_rs::transition::MovingTransition::new(2.0)
            .unwrap()
            .with_from_state(all_states[0].id())
            .with_single_to_state(all_states[1].id());
        all_transitions.push(t_drive);
    }

    match Botix::build_full(controller, all_states, all_transitions) {
        Ok(mut botix) => {
            info!("Mission built. Executing...");
            match botix.execute() {
                Ok(()) => info!("Mission complete."),
                Err(e) => error!("Mission error: {}", e),
            }
        }
        Err(e) => {
            error!("Failed to build mission: {}", e);
        }
    }

    info!("KAZU stopped.");
}
