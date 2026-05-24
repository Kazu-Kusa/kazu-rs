use crate::config::AppConfig;
use log::error;
use std::io::{self, BufRead, Write};
use std::process;
use std::thread;
use std::time::Duration;

pub fn cmd_cmd(
    app_config: AppConfig,
    shell: bool,
    port: Option<String>,
    duration: Option<f64>,
    speeds: Vec<i32>,
) {
    use bdmc_rs::controller::CloseLoopController;

    let port_name = port.unwrap_or(app_config.motion.port.clone());

    let mut controller = match CloseLoopController::new(None, None, None, Some(&port_name)) {
        Ok(c) => c,
        Err(e) => {
            error!("Failed to connect to {}: {}", port_name, e);
            process::exit(1);
        }
    };

    if shell {
        println!("Open shell mode. Enter 'quit' to exit.");
        println!("Format: <duration> <speed1> [speed2] [speed3] [speed4]");
        let stdin = io::stdin();
        loop {
            print!(">> ");
            io::stdout().flush().unwrap();
            let mut line = String::new();
            stdin.lock().read_line(&mut line).unwrap();
            let line = line.trim().to_string();

            if line == "quit" {
                break;
            }

            let tokens: Vec<&str> = line.split_whitespace().collect();
            if tokens.len() < 2 || tokens.len() > 5 || tokens.len() == 4 {
                eprintln!("Invalid command. Use 2, 3, or 5 tokens.");
                continue;
            }

            let dur: f64 = match tokens[0].parse() {
                Ok(v) => v,
                Err(_) => {
                    eprintln!("Invalid duration: {}", tokens[0]);
                    continue;
                }
            };

            let spds: Vec<i32> = tokens[1..].iter().filter_map(|t| t.parse().ok()).collect();

            if spds.len() != tokens.len() - 1 {
                eprintln!("Invalid speed values");
                continue;
            }

            // Expand speeds to 4 motors
            let speeds_4: [f64; 4] = match spds.len() {
                1 => [spds[0] as f64; 4],
                2 => [
                    spds[0] as f64,
                    spds[0] as f64,
                    spds[1] as f64,
                    spds[1] as f64,
                ],
                4 => [
                    spds[0] as f64,
                    spds[1] as f64,
                    spds[2] as f64,
                    spds[3] as f64,
                ],
                _ => continue,
            };

            println!("Moving for {dur}s at speeds {:?}", speeds_4);
            controller.set_motors_speed(&speeds_4).ok();
            thread::sleep(Duration::from_secs_f64(dur));
            controller.set_motors_speed(&[0.0; 4]).ok();
        }
    } else if let (Some(dur), true) = (duration, !speeds.is_empty()) {
        let speeds_4: Vec<f64> = match speeds.len() {
            1 => vec![speeds[0] as f64; 4],
            2 => vec![
                speeds[0] as f64,
                speeds[0] as f64,
                speeds[1] as f64,
                speeds[1] as f64,
            ],
            4 => vec![
                speeds[0] as f64,
                speeds[1] as f64,
                speeds[2] as f64,
                speeds[3] as f64,
            ],
            _ => {
                error!("Provide 1, 2, or 4 speed values, got {}", speeds.len());
                process::exit(1);
            }
        };
        println!("Moving for {dur}s at speeds {:?}", speeds_4);
        controller.set_motors_speed(&speeds_4).ok();
        thread::sleep(Duration::from_secs_f64(dur));
        controller.set_motors_speed(&[0.0; 4]).ok();
    } else {
        error!("Specify duration and speeds, or use -s for shell mode");
        process::exit(1);
    }

    controller.close();
}
