use crate::cli::RunModeArg;
use crate::config::{load_run_config, AppConfig};
use log::{error, info};
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
    use mentabotix_rs::{Botix, MovingState, MovingTransition};

    // Load run config if provided.
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

    // Initialize controller.
    let controller = match CloseLoopController::new(None, None, None, effective_port.as_deref()) {
        Ok(c) => c,
        Err(e) => {
            error!("Failed to initialize controller: {}", e);
            return;
        }
    };

    // Build a simple demo mission: drive straight → halt.
    let s_start = MovingState::straight(300);
    let s_halt = MovingState::halt();

    let t_drive = MovingTransition::new(2.0)
        .unwrap()
        .with_from_state(s_start.id())
        .with_single_to_state(s_halt.id());

    match Botix::build_full(controller, vec![s_start, s_halt], vec![t_drive]) {
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
