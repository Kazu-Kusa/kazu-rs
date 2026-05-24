use crate::config::AppConfig;
use log::{error, info};
use std::process;
use std::thread;
use std::time::Duration;

pub fn cmd_breaker(app_config: AppConfig, port: Option<String>) {
    use bdmc_rs::controller::CloseLoopController;

    let port_name = port.unwrap_or(app_config.motion.port.clone());

    let mut controller = match CloseLoopController::new(None, None, None, Some(&port_name)) {
        Ok(c) => c,
        Err(e) => {
            error!("Failed to connect to {}: {}", port_name, e);
            process::exit(1);
        }
    };

    info!("EMERGENCY STOP — sending full stop and reset");
    controller.set_motors_speed(&[0.0; 4]).ok();
    thread::sleep(Duration::from_millis(100));
    controller.close();
    println!("Breaker engaged. Motors stopped, controller closed.");
}
