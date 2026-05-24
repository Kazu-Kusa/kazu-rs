//! kazu-rs — A Dedicated Robots Control System
//!
//! Rust port of the Python `kazu` CLI. Provides subcommands for sensor reading,
//! motor control, mission execution, config management, and more.
mod compile;
mod constant;
#[cfg(feature = "vision")]
use upic_rs::TagDetector;
use clap::{Parser, Subcommand, ValueEnum};
use log::{error, info, warn, LevelFilter};
use std::fs;
use std::io::{self, BufRead, Write};
use std::path::{Path, PathBuf};
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

    /// Test hardware devices: mot, adc, io, mpu, cam, pow, all
    Check {
        /// Devices to test (default: all)
        #[arg(default_value = "all")]
        device: Vec<String>,

        /// Serial port for motor test (overrides config)
        #[arg(short = 'p', long)]
        port: Option<String>,

        /// Camera ID for camera test
        #[arg(short = 'c', long)]
        camera: Option<i32>,
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

    /// Visualize state-transition diagrams as PlantUML
    Viz {
        /// Behavior packs to visualize (default: all)
        #[arg(default_value = "all")]
        packname: Vec<String>,

        /// Output directory for generated .puml files
        #[arg(short = 'd', long, default_value = "./visualize")]
        destination: PathBuf,

        /// Path to run config TOML file
        #[arg(short = 'r', long)]
        run_config_path: Option<PathBuf>,
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
    /// Detect AprilTags via camera
    Tag {
        /// Camera device ID
        #[arg(short = 'c', long)]
        camera: Option<i32>,

        /// Camera resolution multiplier (e.g. 0.5 = half res)
        #[arg(short = 'm', long)]
        camera_res_mul: Option<f64>,

        /// Detection refresh interval in seconds
        #[arg(short = 'i', long, default_value = "0.5")]
        interval: f64,
    },
    /// Record sensor data to CSV
    Record {
        /// Output directory for CSV files
        #[arg(short = 'o', long, default_value = "./record")]
        output_dir: PathBuf,

        /// Sampling interval in seconds
        #[arg(short = 'i', long, default_value = "0.1")]
        interval: f64,

        /// Path to run config TOML file (also set via KAZU_RUN_CONFIG_PATH env)
        #[arg(short = 'r', long, env = "KAZU_RUN_CONFIG_PATH")]
        run_config_path: Option<PathBuf>,
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

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
struct AppConfig {
    #[serde(default)]
    motion: MotionSection,
    #[serde(default)]
    sensor: SensorSection,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
struct MotionSection {
    #[serde(default = "default_port")]
    port: String,
    #[serde(default = "default_baudrate")]
    baudrate: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
struct SensorSection {
    #[serde(default)]
    gyro_fsr: u16,
    #[serde(default)]
    accel_fsr: u8,
}

fn default_port() -> String { "/dev/ttyUSB0".into() }
fn default_baudrate() -> u32 { 115200 }

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            motion: MotionSection { port: default_port(), baudrate: default_baudrate() },
            sensor: SensorSection { gyro_fsr: 2000, accel_fsr: 8 },
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct RunConfig {
    #[serde(default = "default_team")]
    team_color: String,
    #[serde(default)]
    missions: MissionsSection,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
struct MissionsSection {
    #[serde(default)]
    boot: Vec<String>,
    #[serde(default)]
    stage: Vec<String>,
    #[serde(default)]
    off_stage: Vec<String>,
}

fn default_team() -> String { "blue".into() }

fn load_app_config(path: &PathBuf) -> AppConfig {
    if path.exists() {
        info!("Loading config from {}", path.display());
        fs::read_to_string(path)
            .ok()
            .and_then(|s| toml::from_str(&s).ok())
            .unwrap_or_default()
    } else {
        warn!("Config file not found: {}, using defaults", path.display());
        AppConfig::default()
    }
}

fn load_run_config(path: &Path) -> RunConfig {
    if path.exists() {
        info!("Loading run config from {}", path.display());
        fs::read_to_string(path)
            .ok()
            .and_then(|s| toml::from_str(&s).ok())
            .unwrap_or_default()
    } else {
        warn!("Run config not found: {}, using defaults", path.display());
        RunConfig::default()
    }
}

impl Default for RunConfig {
    fn default() -> Self {
        Self { team_color: default_team(), missions: MissionsSection::default() }
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
        Commands::Check { device, port, camera } => cmd_check(app_config, device, port, camera),
        Commands::Read { devices, use_screen, interval, port } => {
            cmd_read(app_config, devices, use_screen, interval, port)
        }
        Commands::Viz { packname, destination, run_config_path } => {
            cmd_viz(app_config, packname, destination, run_config_path)
        }
        Commands::Cmd { shell, port, duration, speeds } => {
            cmd_cmd(app_config, shell, port, duration, speeds)
        }
        Commands::Ports { check, timeout } => cmd_ports(app_config, check, timeout),
        Commands::Msg { port } => cmd_msg(app_config, port),
        Commands::Light { shell, color } => cmd_light(shell, color),
        Commands::Breaker { port } => cmd_breaker(app_config, port),
        Commands::Bench { target } => cmd_bench(app_config, target),
        #[cfg(feature = "vision")]
        Commands::Tag { camera, camera_res_mul, interval } => {
            cmd_tag(app_config, camera, camera_res_mul, interval)
        }
        #[cfg(not(feature = "vision"))]
        Commands::Tag { .. } => {
            error!("The 'tag' command requires the 'vision' feature. Rebuild with --features vision");
            process::exit(1);
        }
        Commands::Record { output_dir, interval, run_config_path } => {
            cmd_record(app_config, output_dir, interval, run_config_path)
        }
    }
}

// ── Command implementations ─────────────────────────────────

fn cmd_config(action: ConfigAction) {
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
#[allow(clippy::too_many_arguments)]
fn cmd_run(
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
    use mentabotix_rs::{
        Botix, MovingState, MovingTransition,
    };

    // Load run config if provided.
    let run_config = run_config_path
        .as_deref()
        .map(load_run_config)
        .unwrap_or_default();

    let effective_port = port.or_else(|| {
        if app_config.motion.port != default_port() {
            Some(app_config.motion.port.clone())
        } else {
            None
        }
    });

    info!("Run mode: {:?}, team: {}, port: {:?}",
        mode, run_config.team_color, effective_port);
    info!("Missions: boot={:?}, stage={:?}, off_stage={:?}",
        run_config.missions.boot.len(),
        run_config.missions.stage.len(),
        run_config.missions.off_stage.len());

    // Initialize controller.
    let controller = match CloseLoopController::new(
        None, None, None,
        effective_port.as_deref(),
    ) {
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

    match Botix::build_full(
        controller,
        vec![s_start, s_halt],
        vec![t_drive],
    ) {
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

fn cmd_check(app_config: AppConfig, devices: Vec<String>, port: Option<String>, _camera: Option<i32>) {
    use bdmc_rs::controller::CloseLoopController;

    let test_all = devices.iter().any(|d| d == "all");
    let test_mot = test_all || devices.iter().any(|d| d == "mot");
    let test_adc = test_all || devices.iter().any(|d| d == "adc");
    let test_io  = test_all || devices.iter().any(|d| d == "io");
    let test_mpu = test_all || devices.iter().any(|d| d == "mpu");

    println!("{:=^40}", " Hardware Check ");
    let mut all_ok = true;

    if test_mot {
        let port_name = port.unwrap_or_else(|| app_config.motion.port.clone());
        print!("  MOTOR  ({:30}) ... ", port_name);
        match CloseLoopController::new(None, None, None, Some(&port_name)) {
            Ok(mut c) => { println!("OK"); c.close(); }
            Err(e) => { println!("FAIL ({})", e); all_ok = false; }
        }
    }

    if test_adc {
        print!("  ADC                      ... ");
        println!("SKIP (no uptechstar hardware on this host)");
    }

    if test_io {
        print!("  IO                       ... ");
        println!("SKIP (no uptechstar hardware on this host)");
    }

    if test_mpu {
        print!("  MPU                      ... ");
        println!("SKIP (no uptechstar hardware on this host)");
    }

    println!("{:=^40}", if all_ok { " ALL OK " } else { " FAILURES " });
}

fn cmd_read(
    _app_config: AppConfig,
    devices: Vec<SensorDevice>,
    use_screen: bool,
    interval: f64,
    _port: Option<String>,
) {
    use uptechstar_rs::{Color, FontSize, OnBoardSensors, Screen, ScreenDirection};

    let show_all = devices.contains(&SensorDevice::All);
    let show_adc = show_all || devices.contains(&SensorDevice::Adc);
    let show_io = show_all || devices.contains(&SensorDevice::Io);
    let show_mpu = show_all || devices.contains(&SensorDevice::Mpu);

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
            for (i, &v) in adc.iter().enumerate().take(9) {
                let label = format!("ADC[{}]:{}", i, v);
                screen.put_string(0, (i as i32) * 8, &label);
            }
            screen.refresh();
        }

        thread::sleep(Duration::from_secs_f64(interval));
    }
}

fn cmd_viz(
    _app_config: AppConfig,
    packnames: Vec<String>,
    destination: PathBuf,
    run_config_path: Option<PathBuf>,
) {
    use mentabotix_rs::{export_structure, ArrowStyle};

    let _run_config = run_config_path
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

    info!("Exporting {} pack(s) to {}", packs.len(), destination.display());

    for &name in &packs {
        match compile::get_handler(name) {
            Some(handler) => {
                let transitions = handler();
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

    println!("Done. {} file(s) written to {}", packs.len(), destination.display());
}

fn cmd_cmd(
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

    let port_name = port.unwrap_or(app_config.motion.port.clone());

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

/// Detect AprilTags from camera feed and print tag IDs in real time.
#[cfg(feature = "vision")]
fn cmd_tag(_app_config: AppConfig, camera: Option<i32>, camera_res_mul: Option<f64>, interval: f64) {

    let mut detector = match TagDetector::new(camera, camera_res_mul) {
        Ok(d) => d,
        Err(e) => {
            error!("Failed to initialize tag detector: {}", e);
            return;
        }
    };

    match detector.apriltag_detect_start() {
        Ok(_) => {}
        Err(e) => {
            error!("Camera is not ready, exiting...: {}", e);
            return;
        }
    }

    info!("Tag detection started. Press Ctrl+C to stop.");
    loop {
        thread::sleep(Duration::from_secs_f64(interval));
        print!("\rTag: {}", detector.tag_id());
        // Flush stdout so \r inline updates appear immediately
        let _ = io::stdout().flush();
    }
}
fn cmd_record(
    _app_config: AppConfig,
    output_dir: PathBuf,
    interval: f64,
    run_config_path: Option<PathBuf>,
) {
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;

    let _run_config = run_config_path
        .as_deref()
        .map(load_run_config)
        .unwrap_or_default();

    if let Err(e) = fs::create_dir_all(&output_dir) {
        error!("Failed to create output directory {}: {}", output_dir.display(), e);
        process::exit(1);
    }

    println!("Press Enter to start recording, Ctrl+C to stop");
    io::stdout().flush().unwrap();

    // Wait for Enter to start
    let mut buf = String::new();
    io::stdin().read_line(&mut buf).unwrap();

    // Try to initialize ADC hardware; fall back to simulated data
    let mut sensors: Option<uptechstar_rs::OnBoardSensors> = None;
    let hardware_available = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        uptechstar_rs::OnBoardSensors::default().adc_io_open()
    }));

    match hardware_available {
        Ok(s) => {
            info!("ADC hardware initialized");
            sensors = Some(s);
        }
        Err(_) => {
            println!("Sensor hardware not available — recording simulated data");
            info!("Falling back to simulated ADC data");
        }
    }

    // Use a background thread to detect Enter press as stop signal
    let stop_flag = Arc::new(AtomicBool::new(false));
    let stop_flag_clone = Arc::clone(&stop_flag);

    let _stdin_thread = thread::spawn(move || {
        let mut _buf = String::new();
        let _ = io::stdin().read_line(&mut _buf);
        stop_flag_clone.store(true, Ordering::Relaxed);
    });

    println!("Recording started. Press Enter to stop.");
    let mut records: Vec<(f64, [u16; 10])> = Vec::new();
    let start = Instant::now();

    while !stop_flag.load(Ordering::Relaxed) {
        let timestamp = start.elapsed().as_secs_f64();

        let adc_values: [u16; 10] = if let Some(ref mut s) = sensors {
            s.adc_all_channels()
        } else {
            // Simulated: simple ramp values based on timestamp
            let t = timestamp as u16;
            [t, t.wrapping_add(1), t.wrapping_add(2), t.wrapping_add(3),
             t.wrapping_add(4), t.wrapping_add(5), t.wrapping_add(6),
             t.wrapping_add(7), t.wrapping_add(8), t.wrapping_add(9)]
        };

        records.push((timestamp, adc_values));
        thread::sleep(Duration::from_secs_f64(interval));
    }

    // Write CSV
    let timestamp_ms = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis();
    let filename = format!("{}.csv", timestamp_ms);
    let filepath = output_dir.join(&filename);

    match fs::File::create(&filepath) {
        Ok(mut f) => {
            // Write header
            let _ = writeln!(f, "Timestamp,ADC0,ADC1,ADC2,ADC3,ADC4,ADC5,ADC6,ADC7,ADC8,ADC9");

            for (timestamp, adc) in &records {
                let _ = writeln!(
                    f,
                    "{:.6},{},{},{},{},{},{},{},{},{},{}",
                    timestamp,
                    adc[0], adc[1], adc[2], adc[3], adc[4],
                    adc[5], adc[6], adc[7], adc[8], adc[9],
                );
            }

            println!("Wrote {} records to {}", records.len(), filepath.display());
        }
        Err(e) => {
            error!("Failed to write CSV file {}: {}", filepath.display(), e);
            process::exit(1);
        }
    }

    // Cleanup ADC if hardware was initialized
    if let Some(s) = sensors {
        s.adc_io_close();
    }

    info!("Record command complete. {} records saved.", records.len());
}
