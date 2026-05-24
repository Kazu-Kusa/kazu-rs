use crate::config::AppConfig;
use log::error;
use std::io::{self, BufRead, Write};
use std::process;

pub fn cmd_msg(app_config: AppConfig, port: Option<String>) {
    use bdmc_rs::controller::CloseLoopController;
    use std::io::Read;

    let port_name = port.unwrap_or(app_config.motion.port.clone());

    let mut controller = match CloseLoopController::new(None, None, None, Some(&port_name)) {
        Ok(c) => c,
        Err(e) => {
            error!("Failed to connect to {}: {}", port_name, e);
            process::exit(1);
        }
    };

    println!("Start streaming input. Enter 'quit' to quit.");
    let stdin = io::stdin();
    loop {
        print!("> ");
        io::stdout().flush().unwrap();
        let mut line = String::new();
        stdin.lock().read_line(&mut line).unwrap();
        let line = line.trim().to_string();

        if line == "quit" {
            break;
        }

        let cmd = format!("{}\r", line);
        controller.send_cmd(cmd.as_bytes()).ok();

        // Read response — note: serial read may block
        if let Some(serial) = controller.serial_mut() {
            let mut buf = [0u8; 256];
            match serial.read(&mut buf) {
                Ok(n) if n > 0 => {
                    let resp = String::from_utf8_lossy(&buf[..n]);
                    println!("Receive: {}", resp.trim());
                }
                Ok(_) => {}
                Err(_) => {
                    // timeout or read error — expected for some controllers
                }
            }
        }
    }

    controller.close();
    println!("Quit streaming.");
}
