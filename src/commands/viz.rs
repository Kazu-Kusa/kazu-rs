use crate::compile;
use crate::config::{load_run_config, AppConfig};
use log::{error, info, warn};
use std::fs;
use std::path::PathBuf;
use std::process;

pub fn cmd_viz(
    app_config: AppConfig,
    packnames: Vec<String>,
    destination: PathBuf,
    run_config_path: Option<PathBuf>,
) {
    use mentabotix_rs::{export_structure, ArrowStyle};

    let run_config = run_config_path
        .as_deref()
        .map(load_run_config)
        .unwrap_or_default();

    let export_all = packnames.iter().any(|p| p == "all");
    let packs: Vec<&str> = if export_all {
        compile::all_handler_names().to_vec()
    } else {
        packnames.iter().map(|s| s.as_str()).collect()
    };

    if let Err(e) = fs::create_dir_all(&destination) {
        error!("Failed to create {}: {}", destination.display(), e);
        process::exit(1);
    }

    info!(
        "Exporting {} pack(s) to {}",
        packs.len(),
        destination.display()
    );

    for &name in &packs {
        match compile::get_handler(name) {
            Some(handler) => {
                let transitions = handler(&app_config, &run_config);
                let filename = destination.join(format!("{}.puml", name));
                match export_structure(&filename, &transitions, ArrowStyle::Down) {
                    Ok(()) => println!("  ✓ {}", filename.display()),
                    Err(e) => error!("  ✗ {}: {}", name, e),
                }
            }
            None => {
                warn!("Unknown pack: {}", name);
                info!("Available: {:?}", compile::all_handler_names());
            }
        }
    }

    println!(
        "Done. {} file(s) written to {}",
        packs.len(),
        destination.display()
    );
}
