use crate::cli::ConfigAction;
use crate::config::AppConfig;
use crate::config::RunConfig;
use log::error;
use std::fs;
use std::path::PathBuf;
use std::process;

pub fn cmd_config(action: ConfigAction) {
    match action {
        ConfigAction::ExportApp { path } => {
            let dest = path.unwrap_or_else(|| PathBuf::from("kazu_app.toml"));
            let config = AppConfig::default();
            let toml_str = toml::to_string_pretty(&config).unwrap();
            fs::write(&dest, &toml_str).unwrap_or_else(|e| {
                error!("Failed to write config: {}", e);
                process::exit(1);
            });
            println!("App config written to {}", dest.display());
        }
        ConfigAction::ExportRun { path } => {
            let dest = path.unwrap_or_else(|| PathBuf::from("kazu_run.toml"));
            let config = RunConfig::default();
            let toml_str = toml::to_string_pretty(&config).unwrap();
            fs::write(&dest, &toml_str).unwrap_or_else(|e| {
                error!("Failed to write config: {}", e);
                process::exit(1);
            });
            println!("Run config written to {}", dest.display());
        }
    }
}
