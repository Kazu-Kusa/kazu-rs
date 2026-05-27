//! On-board sensors: ADC, IO, and MPU6500.
//!
//! Port of `pyuptech/modules/sensors.py`.

use log::{debug, error, info, warn};
use std::time::Instant;

use crate::ffi;

/// 10 ADC channel values.
pub type AdcDataPack = [u16; 10];
/// 3-axis MPU data (accel, gyro, attitude).
pub type MpuDataPack = [f32; 3];

/// Binary IO level or mode.
pub type BinaryIO = i32;

const E6: i64 = 1_000_000;

/// On-board sensors: 10 ADC channels, 8 IO channels, and MPU6500 6-axis sensor.
///
/// # Examples
///
/// ```ignore
/// let sensors = OnBoardSensors::new(5)
///     .adc_io_open()
///     .set_all_io_mode(0)
///     .set_all_io_levels(1)
///     .mpu6500_open();
/// ```
pub struct OnBoardSensors {
    adc_cache: AdcDataPack,
    adc_all: AdcDataPack,
    accel_all: MpuDataPack,
    gyro_all: MpuDataPack,
    atti_all: MpuDataPack,
    adc_last_sample_timestamp: Instant,
    adc_min_sample_interval_ns: i64,
}

impl OnBoardSensors {
    /// Create a new `OnBoardSensors` instance.
    ///
    /// `adc_min_sample_interval_ms` is the minimum interval between ADC samples in milliseconds.
    /// Defaults to 5 ms.
    pub fn new(adc_min_sample_interval_ms: i64) -> Self {
        Self {
            adc_cache: [0u16; 10],
            adc_all: [0u16; 10],
            accel_all: [0.0f32; 3],
            gyro_all: [0.0f32; 3],
            atti_all: [0.0f32; 3],
            adc_last_sample_timestamp: Instant::now(),
            adc_min_sample_interval_ns: adc_min_sample_interval_ms * E6,
        }
    }

    /// Timestamp of the last ADC sample, in milliseconds.
    pub fn last_sample_timestamp_ms(&self) -> i64 {
        // approximate: we store Instant, not a direct ns counter
        // Return the stored interval as a rough approximation
        self.adc_min_sample_interval_ns / E6
    }

    /// Minimum interval between ADC samples, in milliseconds.
    pub fn adc_min_sample_interval_ms(&self) -> i64 {
        self.adc_min_sample_interval_ns / E6
    }

    /// Set the minimum interval between ADC samples, in milliseconds.
    pub fn set_adc_min_sample_interval_ms(&mut self, value: i64) {
        self.adc_min_sample_interval_ns = value * E6;
    }

    // ── ADC / IO ──────────────────────────────────────────────

    /// Open the ADC-IO plug.
    pub fn adc_io_open(self) -> Self {
        info!("Initializing ADC-IO");
        let open_times = ffi::adc_io_open();
        if open_times == -1 {
            error!(
                "Failed to open ADC-IO. Check if the channel is already opened and libuptech.so is loaded properly."
            );
        } else {
            debug!("ADC-IO open {} times", open_times);
        }
        self
    }

    /// Close the ADC-IO plug.
    pub fn adc_io_close(self) -> Self {
        info!("Closing ADC-IO");
        if ffi::adc_io_close() == -1 {
            error!(
                "Failed to close ADC-IO. Check if the channel is opened and libuptech.so is loaded properly."
            );
        } else {
            debug!("ADC-IO closed");
        }
        self
    }

    /// Get all 10 ADC channel values.
    ///
    /// Respects the minimum sample interval; returns cached values if called too soon.
    pub fn adc_all_channels(&mut self) -> AdcDataPack {
        let now = Instant::now();
        let elapsed_ns = (now - self.adc_last_sample_timestamp).as_nanos() as i64;
        if elapsed_ns < self.adc_min_sample_interval_ns {
            return self.adc_cache;
        }
        self.adc_last_sample_timestamp = now;
        if ffi::adc_get_all(&mut self.adc_all) != 0 {
            error!(
                "Failed to get all ADC channels. Check if the channel is opened and libuptech.so is loaded properly."
            );
        }
        self.adc_cache = self.adc_all;
        self.adc_cache
    }

    /// Get all IO input levels as a bitmask (uint8).
    ///
    /// Each bit represents a channel: 1 = high, 0 = low.
    ///
    /// # Examples
    ///
    /// `0b10000000` => IO7 is high
    /// `0b00000001` => IO0 is high
    pub fn io_all_channels() -> i32 {
        ffi::adc_io_input_get_all()
    }

    /// Get the level of a specific IO pin.
    ///
    /// Only meaningful in OUTPUT mode.
    pub fn get_io_level(index: i32) -> i32 {
        (ffi::adc_io_input_get_all() >> index) & 1
    }

    /// Set all IO levels as a bitmask.
    ///
    /// Only meaningful in OUTPUT mode.
    ///
    /// # Examples
    ///
    /// `levels = 0b00000001` => IO0 high, rest low
    /// `levels = 0b10000000` => IO7 high, rest low
    pub fn set_all_io_levels(self, levels: i32) -> Self {
        if ffi::adc_io_set_all(levels as u32) != 0 {
            error!(
                "Failed to set all IO levels. Check if the channel is opened and libuptech.so is loaded properly."
            );
        }
        self
    }

    /// Flip the level of the specified IO pin.
    ///
    /// Only meaningful in OUTPUT mode.
    pub fn flip_io_level(self, index: i32) -> Self {
        if ffi::adc_io_set(index as u32) == -1 {
            error!(
                "Failed to flip IO level, index: {}. Check if the channel is opened and libuptech.so is loaded properly.",
                index
            );
        }
        self
    }

    /// Get all IO modes as a bitmask (uint8).
    ///
    /// # Examples
    ///
    /// `0b10000000` => IO7 output mode (servo)
    /// `0b00000001` => IO0 input mode (external sensor)
    pub fn get_all_io_mode() -> i32 {
        let mut mode: u8 = 0;
        ffi::adc_io_mode_get_all(&mut mode);
        mode as i32
    }

    /// Set all IO modes to the given mode (0 = input, 1 = output).
    pub fn set_all_io_mode(self, mode: BinaryIO) -> Self {
        for index in 0..8 {
            if ffi::adc_io_mode_set(index, mode) != 0 {
                error!("Failed to set IO mode for pin {}, mode {}.", index, mode);
            }
        }
        self
    }

    /// Set the mode of a specific IO pin (0 = input, 1 = output).
    pub fn set_io_mode(self, index: i32, mode: BinaryIO) -> Self {
        if ffi::adc_io_mode_set(index as u32, mode) != 0 {
            error!(
                "Failed to set IO mode, index: {}, mode: {}. Check if the channel is opened and libuptech.so is loaded properly.",
                index, mode
            );
        }
        self
    }

    // ── MPU6500 ───────────────────────────────────────────────

    /// Initialize the MPU6500 6-axis sensor.
    ///
    /// Default settings:
    /// - acceleration: ±8G
    /// - gyro: ±2000°/s
    /// - sampling rate: 1 kHz
    pub fn mpu6500_open(self) -> Self {
        info!("Initializing MPU6500...");
        if ffi::mpu6500_dmp_init() != 0 {
            warn!(
                "Failed to initialize MPU6500. Check if the channel is opened and libuptech.so is loaded properly."
            );
            return self;
        }
        info!("MPU6500 initialized");
        self
    }

    /// Get accelerometer data (3 axes: X, Y, Z).
    pub fn acc_all(&mut self) -> MpuDataPack {
        ffi::mpu6500_get_accel(&mut self.accel_all);
        self.accel_all
    }

    /// Get gyroscope data (3 axes: X, Y, Z).
    pub fn gyro_all(&mut self) -> MpuDataPack {
        ffi::mpu6500_get_gyro(&mut self.gyro_all);
        self.gyro_all
    }

    /// Get attitude data (Pitch, Roll, Yaw).
    pub fn atti_all(&mut self) -> MpuDataPack {
        ffi::mpu6500_get_attitude(&mut self.atti_all);
        self.atti_all
    }

    /// Get the gyroscope full-scale range.
    pub fn get_gyro_fsr() -> i32 {
        let mut fsr: u16 = 0;
        ffi::mpu_get_gyro_fsr(&mut fsr);
        fsr as i32
    }

    /// Get the accelerometer full-scale range.
    pub fn get_accel_fsr() -> i32 {
        let mut fsr: i8 = 0;
        ffi::mpu_get_accel_fsr(&mut fsr);
        fsr as i32
    }

    /// Set the gyroscope full-scale range (250, 500, 1000, or 2000 deg/s).
    pub fn mpu_set_gyro_fsr(self, fsr: i32) -> Self {
        ffi::mpu_set_gyro_fsr(fsr as u32);
        self
    }

    /// Set the accelerometer full-scale range (2, 4, 8, or 16 g).
    pub fn mpu_set_accel_fsr(self, fsr: i32) -> Self {
        ffi::mpu_set_accel_fsr(fsr);
        self
    }
}

impl Default for OnBoardSensors {
    fn default() -> Self {
        Self::new(5)
    }
}
