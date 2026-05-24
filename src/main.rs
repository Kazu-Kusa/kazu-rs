//! kazu-rs — A Dedicated Robots Control System
//!
//! Rust port of the Python `kazu` CLI. Provides subcommands for sensor reading,
//! motor control, mission execution, config management, and more.

mod cli;
mod commands;
mod compile;
mod config;
mod constant;
mod judgers;
mod assembly;
mod signal_light;
mod static_utils;
use clap::Parser;
use cli::{Cli, Commands};
use config::load_app_config;
use log::{error, LevelFilter};
use std::process;

fn main() {
    let cli = Cli::parse();

    // Init logger
    let level = cli
        .log_level
        .map(LevelFilter::from)
        .unwrap_or(LevelFilter::Info);
    env_logger::Builder::new()
        .filter_level(level)
        .format_timestamp_millis()
        .init();

    let app_config = load_app_config(&cli.app_config_path);

    match cli.command {
        Commands::Config { action } => commands::cmd_config(action),
        Commands::Run {
            port,
            run_config_path,
            mode,
            disable_camera,
            camera,
            camera_res_mul,
            team_color,
        } => commands::cmd_run(
            app_config,
            port,
            run_config_path,
            mode,
            disable_camera,
            camera,
            camera_res_mul,
            team_color,
        ),
        Commands::Check {
            device,
            port,
            camera,
        } => commands::cmd_check(app_config, device, port, camera),
        Commands::Read {
            devices,
            use_screen,
            interval,
            port,
        } => commands::cmd_read(app_config, devices, use_screen, interval, port),
        Commands::Viz {
            packname,
            destination,
            run_config_path,
        } => commands::cmd_viz(app_config, packname, destination, run_config_path),
        Commands::Cmd {
            shell,
            port,
            duration,
            speeds,
        } => commands::cmd_cmd(app_config, shell, port, duration, speeds),
        Commands::Ports { check, timeout } => commands::cmd_ports(app_config, check, timeout),
        Commands::Msg { port } => commands::cmd_msg(app_config, port),
        Commands::Light { shell, color } => commands::cmd_light(shell, color),
        Commands::Breaker { port } => commands::cmd_breaker(app_config, port),
        Commands::Bench { target } => commands::cmd_bench(app_config, target),
        #[cfg(feature = "vision")]
        Commands::Tag {
            camera,
            camera_res_mul,
            interval,
        } => commands::cmd_tag(app_config, camera, camera_res_mul, interval),
        #[cfg(not(feature = "vision"))]
        Commands::Tag { .. } => {
            error!("The 'tag' command requires the 'vision' feature. Rebuild with --features vision");
            process::exit(1);
        }
        Commands::Record {
            output_dir,
            interval,
            run_config_path,
        } => commands::cmd_record(app_config, output_dir, interval, run_config_path),
    }
}
