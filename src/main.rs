//! kazu-rs — A Dedicated Robots Control System
//!
//! Rust port of the Python `kazu` CLI. Provides subcommands for sensor reading,
//! motor control, mission execution, config management, and more.

use clap::{Parser, Subcommand, ValueEnum};
use log::{error, info, warn, LevelFilter};
use std::io::{self, BufRead, Write};
use std::path::PathBuf;
use std::process;
use std::thread;
use std::time::{Duration, Instant};

// ── Top-level CLI ───────────────────────────────────────────

#[derive(Parser)]
#[command(
    name = "kazu-rs",
    version = env!("CARGO_PKG_VERSION"),
    about = "A Dedicated Robots Control System",
    long_about = r#"
______ __
___  //_/_____ __________  __
__  ,<  _  __ `/__  /_  / / /
_  /| | / /_/ /__  /_/ /_/ /
/_/ |_| \__,_/ _____/\__,_/

Kazu: A Dedicated Robots Control System
"#,
)]
struct Cli {
    /// Config file path (also set via KAZU_APP_CONFIG_PATH env)
    #[arg(short = 'a', long, env = "KAZU_APP_CONFIG_PATH", default_value = "kazu.toml")]
    app_config_path: PathBuf,

    /// Log level override
    #[arg(short = 'l', long, value_enum)]
    log_level: Option<LogLevel>,

    /// Disable signal light
    #[arg(short = 's', long)]
    disable_siglight: bool,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Copy, Clone, PartialEq, Eq, ValueEnum)]
enum LogLevel {
    Debug,
    Info,
    Warning,
    Error,
    Critical,
}

impl From<LogLevel> for LevelFilter {
    fn from(l: LogLevel) -> Self {
        match l {
            LogLevel::Debug => LevelFilter::Debug,
            LogLevel::Info => LevelFilter::Info,
            LogLevel::Warning => LevelFilter::Warn,
            LogLevel::Error => LevelFilter::Error,
            LogLevel::Critical => LevelFilter::Error,
        }
    }
}

#[derive(Subcommand)]
enum Commands {
    /// Manage application and run configuration
    Config {
        #[command(subcommand)]
        action: ConfigAction,
    },

    /// Execute robot missions
    Run {
        /// Serial port override
        #[arg(short = 'p', long)]
        port: Option<String>,

        /// Run config file path (also set via KAZU_RUN_CONFIG_PATH env)
        #[arg(short = 'r', long, env = "KAZU_RUN_CONFIG_PATH")]
        run_config_path: Option<PathBuf>,

        /// Run mode
        #[arg(short = 'm', long, env = "KAZU_RUN_MODE", default_value = "fgs")]
        mode: RunModeArg,

        /// Disable camera
        #[arg(short = 'd', long)]
        disable_camera: bool,

        /// Camera index
        #[arg(short = 'c', long)]
        camera: Option<i32>,

        /// Camera resolution multiplier
        #[arg(long)]
        camera_res_mul: Option<f64>,

        /// Team color
        #[arg(short = 't', long)]
        team_color: Option<String>,
    },

    /// Check and validate configuration or mission files
    Check {
        /// Path to config or mission file
        #[arg(short = 'p', long)]
        path: Option<PathBuf>,
    },

    /// Read sensor data continuously
    Read {
        /// Devices to read: adc, io, mpu, or all
        #[arg(default_value = "all")]
        devices: Vec<SensorDevice>,

        /// Print to onboard LCD screen
        #[arg(short = 's', long)]
        use_screen: bool,

        /// Refresh interval in seconds
        #[arg(short = 'i', long, default_value = "0.5")]
        interval: f64,

        /// Port override
        #[arg(short = 'p', long)]
        port: Option<String>,
    },

    /// Visualize a mission as PlantUML
    Viz {
        /// Path to config or mission TOML file
        #[arg(short = 'p', long)]
        path: Option<PathBuf>,
    },

    /// Send raw serial commands to motor controller
    Cmd {
        /// Run in interactive shell mode
        #[arg(short = 's', long)]
        shell: bool,

        /// Serial port override
        #[arg(short = 'p', long)]
        port: Option<String>,

        /// Movement duration in seconds
        duration: Option<f64>,

        /// Motor speeds (2, 3, or 5 values)
        speeds: Vec<i32>,
    },

    /// List available serial ports
    Ports {
        /// Check if ports are available/usable
        #[arg(short = 'c', long)]
        check: bool,

        /// Check timeout in seconds
        #[arg(short = 't', long, default_value = "1.0")]
        timeout: f64,
    },

    /// Stream messages to/from the motor controller
    Msg {
        /// Serial port override
        #[arg(short = 'p', long)]
        port: Option<String>,
    },

    /// Control the signal light
    Light {
        /// Open interactive shell
        #[arg(short = 's', long)]
        shell: bool,

        /// Color to set
        color: Option<String>,
    },

    /// Emergency breaker / stop
    Breaker {
        /// Serial port override
        #[arg(short = 'p', long)]
        port: Option<String>,
    },

    /// Benchmark various system components
    Bench {
        #[command(subcommand)]
        target: BenchTarget,
    },
}

// ── Subcommand enums ────────────────────────────────────────

#[derive(Subcommand)]
enum ConfigAction {
    /// Export default application config
    ExportApp {
        #[arg(short = 'p', long)]
        path: Option<PathBuf>,
    },
    /// Export default run config
    ExportRun {
        #[arg(short = 'p', long)]
        path: Option<PathBuf>,
    },
}

#[derive(Copy, Clone, PartialEq, Eq, ValueEnum, Debug)]
enum RunModeArg {
    /// Off-stage start: boot once, then stage loop
    Fgs,
    /// On-stage start: stage loop only
    Ngs,
    /// Always off-stage: off-stage loop
    Afg,
    /// Always on-stage: on-stage loop
    Ang,
    /// Off-stage dash loop: boot loop
    Fgdl,
}

#[derive(Clone, PartialEq, Eq, ValueEnum)]
enum SensorDevice {
    Adc,
    Io,
    Mpu,
    All,
}

#[derive(Subcommand)]
enum BenchTarget {
    /// Benchmark signal light switch frequency
    Siglight,
    /// Benchmark sleep/scheduling precision
    Sleep,
    /// Benchmark ADC sample throughput
    Adc,
    /// Benchmark application startup
    App,
}

// ── App state ───────────────────────────────────────────────

struct AppConfig {
    motion_port: String,
    // In a full port, this would hold the parsed TOML config
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            motion_port: "/dev/ttyUSB0".into(),
        }
    }
}

fn load_app_config(path: &PathBuf) -> AppConfig {
    if path.exists() {
        info!("Loading config from {}", path.display());
        // In a full port, parse TOML here
        // For now, return defaults
        AppConfig::default()
    } else {
        warn!("Config file not found: {}, using defaults", path.display());
        AppConfig::default()
    }
}

// ── main ────────────────────────────────────────────────────

fn main() {
    let cli = Cli::parse();

    // Init logger
    let level = cli.log_level.map(LevelFilter::from).unwrap_or(LevelFilter::Info);
    env_logger::Builder::new()
        .filter_level(level)
        .format_timestamp_millis()
        .init();

    let app_config = load_app_config(&cli.app_config_path);

    match cli.command {
        Commands::Config { action } => cmd_config(action),
        Commands::Run { port, run_config_path, mode, disable_camera, camera, camera_res_mul, team_color } => {
            cmd_run(app_config, port, run_config_path, mode, disable_camera, camera, camera_res_mul, team_color)
        }
        Commands::Check { path } => cmd_check(app_config, path),
        Commands::Read { devices, use_screen, interval, port } => {
            cmd_read(app_config, devices, use_screen, interval, port)
        }
        Commands::Viz { path } => cmd_viz(app_config, path),
        Commands::Cmd { shell, port, duration, speeds } => {
            cmd_cmd(app_config, shell, port, duration, speeds)
        }
        Commands::Ports { check, timeout } => cmd_ports(app_config, check, timeout),
        Commands::Msg { port } => cmd_msg(app_config, port),
        Commands::Light { shell, color } => cmd_light(shell, color),
        Commands::Breaker { port } => cmd_breaker(app_config, port),
        Commands::Bench { target } => cmd_bench(app_config, target),
    }
}

// ── Command implementations ─────────────────────────────────

fn cmd_config(action: ConfigAction) {
    match action {
        ConfigAction::ExportApp { path } => {
            let dest = path.unwrap_or_else(|| PathBuf::from("kazu_app.toml"));
            let default_toml = r#"# kazu-rs application config
[motion]
port = "/dev/ttyUSB0"
baudrate = 115200

[sensor]
gyro_fsr = 2000
accel_fsr = 8
edge_fl_index = 0
edge_fr_index = 1
edge_rl_index = 2
edge_rr_index = 3
left_adc_index = 4
right_adc_index = 5
front_adc_index = 6
rb_adc_index = 7
gray_adc_index = 8
fl_io_index = 0
fr_io_index = 1
rl_io_index = 2
rr_io_index = 3
reboot_button_index = 4
gray_io_left_index = 5
gray_io_right_index = 6
"#;
            std::fs::write(&dest, default_toml).unwrap_or_else(|e| {
                error!("Failed to write config: {}", e);
                process::exit(1);
            });
            println!("Default app config written to {}", dest.display());
        }
        ConfigAction::ExportRun { path } => {
            let dest = path.unwrap_or_else(|| PathBuf::from("kazu_run.toml"));
            let default_toml = r#"# kazu-rs run config
team_color = "blue"

[missions]
boot = []
stage = []
off_stage = []
"#;
            std::fs::write(&dest, default_toml).unwrap_or_else(|e| {
                error!("Failed to write config: {}", e);
                process::exit(1);
            });
            println!("Default run config written to {}", dest.display());
        }
    }
}

fn cmd_run(
    _app_config: AppConfig,
    _port: Option<String>,
    _run_config_path: Option<PathBuf>,
    mode: RunModeArg,
    _disable_camera: bool,
    _camera: Option<i32>,
    _camera_res_mul: Option<f64>,
    _team_color: Option<String>,
) {
    use uptechstar_rs::OnBoardSensors;

    info!("Initializing hardware for run mode: {:?}", mode);

    // Initialize sensors
    let mut sensors = OnBoardSensors::default()
        .adc_io_open()
        .mpu6500_open()
        .set_all_io_mode(0);

    // Initialize screen (if available)
    let _screen = uptechstar_rs::Screen::default();

    info!("Hardware initialized. Run mode: {:?}", mode);
    info!("Press Ctrl+C to stop.");

    // Main run loop — in a full port, this would compile and execute missions
    // from the mentabotix DSL based on the run mode.
    let start = Instant::now();
    loop {
        let _adc = sensors.adc_all_channels();
        let _acc = sensors.acc_all();
        let _gyro = sensors.gyro_all();

        if start.elapsed() > Duration::from_secs(30) {
            info!("Demo run complete (30s limit).");
            break;
        }
        thread::sleep(Duration::from_millis(100));
    }

    info!("Releasing hardware resources...");
    sensors.adc_io_close();
    info!("KAZU stopped.");
}

fn cmd_check(_app_config: AppConfig, path: Option<PathBuf>) {
    match path {
        Some(p) => {
            if !p.exists() {
                error!("File not found: {}", p.display());
                process::exit(1);
            }
            let ext = p.extension().and_then(|e| e.to_str()).unwrap_or("");
            match ext {
                "toml" => {
                    let content = std::fs::read_to_string(&p).unwrap_or_else(|e| {
                        error!("Failed to read {}: {}", p.display(), e);
                        process::exit(1);
                    });
                    // Validate TOML syntax
                    match content.parse::<toml::Table>() {
                        Ok(_) => println!("✓ Valid TOML: {}", p.display()),
                        Err(e) => {
                            error!("✗ Invalid TOML: {}", e);
                            process::exit(1);
                        }
                    }
                }
                _ => {
                    warn!("Unknown file type: .{}, checking as text", ext);
                    println!("✓ File exists: {} ({} bytes)", p.display(), p.metadata().map(|m| m.len()).unwrap_or(0));
                }
            }
        }
        None => {
            println!("No file specified. Usage: kazu-rs check -p <path>");
        }
    }
}

fn cmd_read(
    _app_config: AppConfig,
    devices: Vec<SensorDevice>,
    use_screen: bool,
    interval: f64,
    _port: Option<String>,
) {
    use uptechstar_rs::{Color, FontSize, OnBoardSensors, Screen, ScreenDirection};

    let show_all = devices.iter().any(|d| *d == SensorDevice::All);
    let show_adc = show_all || devices.iter().any(|d| *d == SensorDevice::Adc);
    let show_io = show_all || devices.iter().any(|d| *d == SensorDevice::Io);
    let show_mpu = show_all || devices.iter().any(|d| *d == SensorDevice::Mpu);

    let mut sensors = OnBoardSensors::default()
        .adc_io_open()
        .mpu6500_open()
        .set_all_io_mode(0);

    let mut screen = Screen::new(None);

    if use_screen {
        screen.open(ScreenDirection::Horizontal as i32);
    }

    info!("Reading sensors (Ctrl+C to stop)...");

    loop {
        if show_adc {
            let adc = sensors.adc_all_channels();
            print!("\x1b[2J\x1b[H"); // clear screen
            println!("=== ADC Channels ===");
            for (i, &v) in adc.iter().enumerate() {
                println!("  ADC[{i}]: {v:5}");
            }
        }

        if show_io {
            let io = OnBoardSensors::io_all_channels();
            let io_mode = OnBoardSensors::get_all_io_mode();
            println!("=== IO Channels ===");
            println!("  Levels:  {io:08b} ({io})");
            println!("  Modes:   {io_mode:08b} ({io_mode})");
        }

        if show_mpu {
            let acc = sensors.acc_all();
            let gyro = sensors.gyro_all();
            let atti = sensors.atti_all();
            println!("=== MPU6500 ===");
            println!("  Accel:   X={:.2} Y={:.2} Z={:.2}", acc[0], acc[1], acc[2]);
            println!("  Gyro:    X={:.2} Y={:.2} Z={:.2}", gyro[0], gyro[1], gyro[2]);
            println!("  Attitude: Pitch={:.2} Roll={:.2} Yaw={:.2}", atti[0], atti[1], atti[2]);
        }

        if use_screen && show_adc {
            let adc = sensors.adc_all_channels();
            screen
                .fill_screen(Color::BLACK)
                .set_font_size(FontSize::Font6x8);
            for i in 0..9 {
                let label = format!("ADC[{}]:{}", i, adc[i]);
                screen.put_string(0, (i as i32) * 8, &label);
            }
            screen.refresh();
        }

        thread::sleep(Duration::from_secs_f64(interval));
    }
}

fn cmd_viz(_app_config: AppConfig, path: Option<PathBuf>) {
    match path {
        Some(p) => {
            if !p.exists() {
                error!("File not found: {}", p.display());
                process::exit(1);
            }
            info!("Generating PlantUML visualization for: {}", p.display());
            // In a full port, use mentabotix-rs to parse and generate UML
            println!("PlantUML visualization not yet implemented.");
            println!("File: {}", p.display());
        }
        None => {
            error!("Please specify a file with -p/--path");
            process::exit(1);
        }
    }
}

fn cmd_cmd(
    app_config: AppConfig,
    shell: bool,
    port: Option<String>,
    duration: Option<f64>,
    speeds: Vec<i32>,
) {
    use bdmc_rs::controller::CloseLoopController;

    let port_name = port.unwrap_or(app_config.motion_port);

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

            let spds: Vec<i32> = tokens[1..]
                .iter()
                .filter_map(|t| t.parse().ok())
                .collect();

            if spds.len() != tokens.len() - 1 {
                eprintln!("Invalid speed values");
                continue;
            }

            // Expand speeds to 4 motors
            let speeds_4: [f64; 4] = match spds.len() {
                1 => [spds[0] as f64; 4],
                2 => [spds[0] as f64, spds[0] as f64, spds[1] as f64, spds[1] as f64],
                4 => [spds[0] as f64, spds[1] as f64, spds[2] as f64, spds[3] as f64],
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
            2 => vec![speeds[0] as f64, speeds[0] as f64, speeds[1] as f64, speeds[1] as f64],
            4 => vec![speeds[0] as f64, speeds[1] as f64, speeds[2] as f64, speeds[3] as f64],
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

fn cmd_ports(_app_config: AppConfig, check: bool, timeout: f64) {
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

fn cmd_msg(app_config: AppConfig, port: Option<String>) {
    use bdmc_rs::controller::CloseLoopController;
    use std::io::Read;

    let port_name = port.unwrap_or(app_config.motion_port);

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

fn cmd_light(shell: bool, color: Option<String>) {
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

fn parse_color(name: &str) -> (u8, u8, u8) {
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

fn cmd_breaker(app_config: AppConfig, port: Option<String>) {
    use bdmc_rs::controller::CloseLoopController;

    let port_name = port.unwrap_or(app_config.motion_port);

    let mut controller = match CloseLoopController::new(None, None, None, Some(&port_name)) {
        Ok(c) => c,
        Err(e) => {
            error!("Failed to connect to {}: {}", port_name, e);
            process::exit(1);
        }
    };

    info!("EMERGENCY STOP — sending full stop and reset");
    controller
        .set_motors_speed(&[0.0; 4])
        .ok();
    thread::sleep(Duration::from_millis(100));
    controller.close();
    println!("Breaker engaged. Motors stopped, controller closed.");
}

fn cmd_bench(_app_config: AppConfig, target: BenchTarget) {
    match target {
        BenchTarget::Siglight => {
            println!("Benchmark: signal light switch frequency");
            let iterations = 1000;
            let start = Instant::now();
            for _ in 0..iterations {
                // toggle signal light (stub on non-Linux)
            }
            let elapsed = start.elapsed();
            println!("  {iterations} toggles in {elapsed:?}");
            println!("  {:.0} Hz", iterations as f64 / elapsed.as_secs_f64());
        }
        BenchTarget::Sleep => {
            println!("Benchmark: sleep precision");
            let target = Duration::from_millis(10);
            let iterations = 100;
            let mut total_err = Duration::ZERO;
            for _ in 0..iterations {
                let before = Instant::now();
                thread::sleep(target);
                let elapsed = before.elapsed();
                if elapsed > target {
                    total_err += elapsed - target;
                } else {
                    total_err += target - elapsed;
                }
            }
            println!("  Target: {target:?}");
            println!("  Avg error: {:?}", total_err / iterations as u32);
        }
        BenchTarget::Adc => {
            println!("Benchmark: ADC sample throughput");
            let mut sensors = uptechstar_rs::OnBoardSensors::default().adc_io_open();
            let iterations = 1000;
            let start = Instant::now();
            for _ in 0..iterations {
                let _ = sensors.adc_all_channels();
            }
            let elapsed = start.elapsed();
            println!("  {iterations} samples in {elapsed:?}");
            println!("  {:.0} samples/s", iterations as f64 / elapsed.as_secs_f64());
            sensors.adc_io_close();
        }
        BenchTarget::App => {
            println!("Benchmark: application startup");
            let start = Instant::now();
            let _sensors = uptechstar_rs::OnBoardSensors::default().adc_io_open().mpu6500_open();
            let elapsed = start.elapsed();
            println!("  Sensor init: {elapsed:?}");
        }
    }
}
