//! CLI argument types for kazu-rs.

use clap::{Parser, Subcommand, ValueEnum};
use log::LevelFilter;

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
pub struct Cli {
    /// Config file path (also set via KAZU_APP_CONFIG_PATH env)
    #[arg(
        short = 'a',
        long,
        env = "KAZU_APP_CONFIG_PATH",
        default_value = "kazu.toml"
    )]
    pub app_config_path: PathBuf,

    /// Log level override
    #[arg(short = 'l', long, value_enum)]
    pub log_level: Option<LogLevel>,

    /// Disable signal light
    #[arg(short = 's', long)]
    pub disable_siglight: bool,

    #[command(subcommand)]
    pub command: Commands,
}

#[derive(Copy, Clone, PartialEq, Eq, ValueEnum)]
pub enum LogLevel {
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
pub enum Commands {
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
pub enum ConfigAction {
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
pub enum RunModeArg {
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
pub enum SensorDevice {
    Adc,
    Io,
    Mpu,
    All,
}

#[derive(Subcommand)]
pub enum BenchTarget {
    /// Benchmark signal light switch frequency
    Siglight,
    /// Benchmark sleep/scheduling precision
    Sleep,
    /// Benchmark ADC sample throughput
    Adc,
    /// Benchmark application startup
    App,
}

use std::path::PathBuf;
