use std::collections::HashMap;
use std::thread;
use std::time::{Duration, Instant};
use serialport::{SerialPort, DataBits, Parity, StopBits};
use log::{debug, info, warn, error, trace};
use crate::cmds;

pub type Context = HashMap<String, serde_json::Value>;
pub type Direction = i8; // 1 or -1

/// Serial configuration for the motor controller
#[derive(Clone, Debug)]
pub struct SerialConfig {
    pub baudrate: u32,
    pub data_bits: DataBits,
    pub parity: Parity,
    pub stop_bits: StopBits,
    pub timeout: Duration,
}

impl Default for SerialConfig {
    fn default() -> Self {
        Self {
            baudrate: 115200,
            data_bits: DataBits::Eight,
            parity: Parity::None,
            stop_bits: StopBits::One,
            timeout: Duration::from_secs(2),
        }
    }
}

/// A struct representing a motor's ID and direction
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct MotorInfo {
    pub code_sign: i32,
    pub direction: Direction,
}

impl MotorInfo {
    pub fn new(code_sign: i32, direction: Direction) -> Self {
        Self { code_sign, direction }
    }
}

impl Default for MotorInfo {
    fn default() -> Self {
        Self::new(1, 1)
    }
}

/// Classic motor infos for 4-motor setup
pub const CLASSIC_MIS: [MotorInfo; 4] = [
    MotorInfo { code_sign: 1, direction: 1 },
    MotorInfo { code_sign: 2, direction: 1 },
    MotorInfo { code_sign: 3, direction: 1 },
    MotorInfo { code_sign: 4, direction: 1 },
];

/// CloseLoopController is a struct designed to manage and control a system involving multiple motors with closed-loop feedback.
/// It provides methods for setting motor speeds, sending commands, introducing delays with breakers, and updating a shared context.
/// The controller maintains a connection to a serial client for communication with the hardware.
///
/// Key features and functionality include:
///
/// 1. **Initialization**:
///    - Accepts a list of `MotorInfo` objects, specifying motor IDs and directions, and an optional serial port for communication.
///    - Initializes a serial connection for interfacing with the hardware.
///
/// 2. **Context Management**:
///    - Maintains a HashMap (`context`) to store shared data across the application.
///
/// 3. **Motor Control**:
///    - `set_motors_speed`: Sets the speed of each motor based on a provided list of speeds, ensuring consistency with the provided `MotorInfo`.
///    - `send_cmd`: Sends a command to the hardware.
///
/// 4. **Delay Functions**:
///    - `delay`: Introduces a simple delay for a specified duration.
///    - `delay_with_breaker`: Delays execution for a given time, periodically checking a breaker function that can abort the delay if it returns true.
///
/// Overall, the CloseLoopController serves as a central hub for coordinating motor operations, handling communication with the hardware, managing shared data, and introducing controlled delays with breakers in a closed-loop motor control system.
pub struct CloseLoopController {
    serial: Option<Box<dyn SerialPort>>,
    motor_infos: Vec<MotorInfo>,
    context: Context,
    config: SerialConfig,
}

impl CloseLoopController {
    /// Create a new CloseLoopController
    pub fn new(
        motor_infos: Option<Vec<MotorInfo>>,
        context: Option<Context>,
        config: Option<SerialConfig>,
        port: Option<&str>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        debug!("Creating new CloseLoopController");
        trace!("Parameters - motor_infos: {:?}, context provided: {}, config provided: {}, port: {:?}", 
               motor_infos.is_some(), context.is_some(), config.is_some(), port);

        let motor_infos = motor_infos.unwrap_or_else(|| CLASSIC_MIS.to_vec());
        let config = config.unwrap_or_default();
        let context = context.unwrap_or_default();

        info!("Initializing controller with {} motors", motor_infos.len());
        debug!("Serial config - baudrate: {}, timeout: {:?}", config.baudrate, config.timeout);

        // Check for unique motor infos
        let mut unique_check = std::collections::HashSet::new();
        for motor_info in &motor_infos {
            if !unique_check.insert(motor_info) {
                error!("Duplicate motor info detected: {:?}", motor_info);
                return Err("Motor infos must be unique".into());
            }
        }

        let mut controller = Self {
            serial: None,
            motor_infos,
            context,
            config,
        };

        if let Some(port_name) = port {
            info!("Opening serial port during initialization: {}", port_name);
            controller.open(port_name)?;
        }

        info!("CloseLoopController created successfully");
        Ok(controller)
    }

    /// Open the serial port
    pub fn open(&mut self, port: &str) -> Result<&mut Self, Box<dyn std::error::Error>> {
        info!("Attempting to open serial port: {}", port);
        debug!("Serial configuration: baudrate={}, data_bits={:?}, parity={:?}, stop_bits={:?}, timeout={:?}", 
               self.config.baudrate, self.config.data_bits, self.config.parity, self.config.stop_bits, self.config.timeout);
        
        match serialport::new(port, self.config.baudrate)
            .data_bits(self.config.data_bits)
            .parity(self.config.parity)
            .stop_bits(self.config.stop_bits)
            .timeout(self.config.timeout)
            .open()
        {
            Ok(serial) => {
                self.serial = Some(serial);
                info!("Serial port {} opened successfully", port);
                Ok(self)
            }
            Err(e) => {
                error!("Failed to open serial port {}: {}", port, e);
                Err(e.into())
            }
        }
    }

    /// Close the serial port
    pub fn close(&mut self) -> &mut Self {
        if self.serial.is_some() {
            info!("Closing serial port");
            self.serial = None;
            debug!("Serial port closed successfully");
        } else {
            warn!("Attempted to close serial port, but no port was open");
        }
        self
    }

    /// Execute a function and return self
    pub fn wait_exec<F>(&mut self, function: F) -> &mut Self
    where
        F: FnOnce(),
    {
        trace!("Executing wait_exec function");
        function();
        trace!("wait_exec function completed");
        self
    }

    /// Get reference to the serial port
    pub fn serial(&self) -> Option<&Box<dyn SerialPort>> {
        self.serial.as_ref()
    }

    /// Get mutable reference to the serial port
    pub fn serial_mut(&mut self) -> Option<&mut Box<dyn SerialPort>> {
        self.serial.as_mut()
    }

    /// Get reference to the context
    pub fn context(&self) -> &Context {
        &self.context
    }

    /// Get mutable reference to the context
    pub fn context_mut(&mut self) -> &mut Context {
        &mut self.context
    }

    /// Get motor IDs
    pub fn motor_ids(&self) -> Vec<i32> {
        self.motor_infos.iter().map(|info| info.code_sign).collect()
    }

    /// Get motor directions
    pub fn motor_dirs(&self) -> Vec<Direction> {
        self.motor_infos.iter().map(|info| info.direction).collect()
    }

    /// Get motor infos
    pub fn motor_infos(&self) -> &Vec<MotorInfo> {
        &self.motor_infos
    }

    /// Set motor infos
    pub fn set_motor_infos(&mut self, motor_infos: Vec<MotorInfo>) {
        info!("Updating motor infos, new count: {}", motor_infos.len());
        debug!("New motor infos: {:?}", motor_infos);
        self.motor_infos = motor_infos;
    }

    /// Set the speed for each motor based on the provided speeds
    pub fn set_motors_speed(&mut self, speeds: &[f64]) -> Result<&mut Self, Box<dyn std::error::Error>> {
        debug!("Setting motor speeds: {:?}", speeds);
        
        if speeds.len() != self.motor_infos.len() {
            error!("Speed array length ({}) does not match motor count ({})", 
                   speeds.len(), self.motor_infos.len());
            return Err("Length of speeds must equal the number of motors".into());
        }

        if let Some(ref mut serial) = self.serial {
            let mut command = String::new();
            for (motor_info, &speed) in self.motor_infos.iter().zip(speeds.iter()) {
                let adjusted_speed = (speed * motor_info.direction as f64) as i32;
                command.push_str(&format!(
                    "{}v{}\r",
                    motor_info.code_sign,
                    adjusted_speed
                ));
                trace!("Motor {} command: {}v{}", motor_info.code_sign, motor_info.code_sign, adjusted_speed);
            }
            
            debug!("Sending motor speed command: {:?}", command.trim());
            match serial.write_all(command.as_bytes()) {
                Ok(_) => {
                    info!("Motor speeds set successfully");
                    trace!("Command sent: {}", command.trim());
                }
                Err(e) => {
                    error!("Failed to send motor speed command: {}", e);
                    return Err(e.into());
                }
            }
        } else {
            warn!("Attempted to set motor speeds but no serial port is open");
        }

        Ok(self)
    }

    /// Send a command to the serial port
    pub fn send_cmd(&mut self, cmd: &[u8]) -> Result<&mut Self, Box<dyn std::error::Error>> {
        debug!("Sending command: {:?}", String::from_utf8_lossy(cmd));
        
        if let Some(ref mut serial) = self.serial {
            match serial.write_all(cmd) {
                Ok(_) => {
                    trace!("Command sent successfully");
                }
                Err(e) => {
                    error!("Failed to send command: {}", e);
                    return Err(e.into());
                }
            }
        } else {
            warn!("Attempted to send command but no serial port is open");
        }
        Ok(self)
    }

    /// Introduce a delay with a breaker function
    pub fn delay_with_breaker<F>(
        &mut self,
        delay_sec: f64,
        mut breaker: F,
        check_interval: f64,
    ) -> &mut Self
    where
        F: FnMut() -> bool,
    {
        debug!("Starting delay with breaker: {:.2}s delay, {:.2}s check interval", delay_sec, check_interval);
        
        let start_time = Instant::now();
        let delay_duration = Duration::from_secs_f64(delay_sec);
        let check_duration = Duration::from_secs_f64(check_interval);

        // Initial check
        if breaker() {
            debug!("Breaker triggered immediately, skipping delay");
            return self;
        }

        let mut check_count = 0;
        while start_time.elapsed() < delay_duration {
            thread::sleep(check_duration);
            check_count += 1;
            
            if breaker() {
                debug!("Breaker triggered after {:.2}s ({} checks)", 
                       start_time.elapsed().as_secs_f64(), check_count);
                break;
            }
        }

        let elapsed = start_time.elapsed().as_secs_f64();
        if elapsed >= delay_sec {
            debug!("Delay completed normally after {:.2}s", elapsed);
        }

        self
    }

    /// Static method to delay with breaker and return breaker result
    pub fn delay_with_breaker_match<F, T>(
        delay_sec: f64,
        mut breaker: F,
        check_interval: f64,
    ) -> T
    where
        F: FnMut() -> T,
        T: Clone + Default,
    {
        debug!("Starting static delay with breaker match: {:.2}s delay, {:.2}s check interval", delay_sec, check_interval);
        
        let start_time = Instant::now();
        let delay_duration = Duration::from_secs_f64(delay_sec);
        let check_duration = Duration::from_secs_f64(check_interval);

        // Initial check
        let result = breaker();
        if !matches!(result, _) {
            debug!("Initial breaker check returned result immediately");
            return result;
        }

        let mut last_result = result;
        let mut check_count = 0;
        while start_time.elapsed() < delay_duration {
            thread::sleep(check_duration);
            check_count += 1;
            last_result = breaker();
        }

        debug!("Static delay with breaker completed after {:.2}s ({} checks)", 
               start_time.elapsed().as_secs_f64(), check_count);
        last_result
    }

    /// Introduce a simple delay
    pub fn delay(&mut self, delay_sec: f64) -> &mut Self {
        debug!("Starting simple delay: {:.2}s", delay_sec);
        thread::sleep(Duration::from_secs_f64(delay_sec));
        trace!("Simple delay completed");
        self
    }
}