use log::{error, warn};
use std::io::{self, BufRead, Write};
use std::process;

pub fn cmd_light(shell: bool, color: Option<String>) {
    if shell {
        println!("Light shell mode. Enter color name or 'quit' to exit.");
        println!("Available: red, green, blue, yellow, cyan, magenta, white, black, off");
        let stdin = io::stdin();
        loop {
            print!("light> ");
            io::stdout().flush().unwrap();
            let mut line = String::new();
            stdin.lock().read_line(&mut line).unwrap();
            let line = line.trim().to_string();

            if line == "quit" {
                break;
            }

            let c = parse_color(&line);
            println!("Set light to: {:?}", c);
            // In a full port: set signal light via hardware
        }
    } else if let Some(c) = color {
        let parsed = parse_color(&c);
        println!("Set light to: {:?}", parsed);
    } else {
        error!("Specify a color or use -s for shell mode");
        process::exit(1);
    }
}

pub fn parse_color(name: &str) -> (u8, u8, u8) {
    match name.to_lowercase().as_str() {
        "red" => (255, 0, 0),
        "green" => (0, 255, 0),
        "blue" => (0, 0, 255),
        "yellow" => (255, 255, 0),
        "cyan" => (0, 255, 255),
        "magenta" => (255, 0, 255),
        "white" => (255, 255, 255),
        "black" | "off" => (0, 0, 0),
        "orange" => (255, 128, 0),
        "purple" => (128, 0, 128),
        _ => {
            warn!("Unknown color '{}', using white", name);
            (255, 255, 255)
        }
    }
}
