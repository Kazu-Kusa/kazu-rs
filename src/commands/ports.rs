use crate::config::AppConfig;
use std::time::Duration;

pub fn cmd_ports(_app_config: AppConfig, check: bool, timeout: f64) {
    use bdmc_rs::ports::find_serial_ports;

    let ports = find_serial_ports();
    println!("{:=^40}", " Serial Ports ");
    if ports.is_empty() {
        println!("  No serial ports found.");
    }
    for port in &ports {
        if check {
            // Try to open to check availability
            match serialport::new(port, 115200)
                .timeout(Duration::from_secs_f64(timeout))
                .open()
            {
                Ok(_) => println!("  {port:30} [AVAILABLE]"),
                Err(_) => println!("  {port:30} [IN USE / UNAVAILABLE]"),
            }
        } else {
            println!("  {port:30} [---]");
        }
    }
    println!("{:=^40}", "");
}
