//! Sensor emulation with random data.
//!
//! Port of `pyuptech/modules/emulation.py`.
//!
//! Provides a `SensorEmulator` that extends `OnBoardSensors` with randomized
//! data for testing without real hardware.

use rand::Rng;

use crate::sensors::{AdcDataPack, BinaryIO, MpuDataPack};

/// Emulated sensor data using random values.
///
/// Overrides all hardware-accessing methods with random-data generators.
pub struct SensorEmulator {
    mpu_rand_range: (u32, u32),
    adc_rand_range: (u16, u16),
    io_rand_range: (u8, u8),
}

impl SensorEmulator {
    /// Create a new emulator with the given random ranges.
    pub fn new(
        mpu_rand_range: (u32, u32),
        adc_rand_range: (u16, u16),
        io_rand_range: (u8, u8),
    ) -> Self {
        Self {
            mpu_rand_range,
            adc_rand_range,
            io_rand_range,
        }
    }

    /// Random value in the MPU range as f32.
    fn rand_mpu(&self) -> f32 {
        let mut rng = rand::thread_rng();
        rng.gen_range(self.mpu_rand_range.0..=self.mpu_rand_range.1) as f32
    }

    /// Random value in the ADC range.
    fn rand_adc(&self) -> u16 {
        let mut rng = rand::thread_rng();
        rng.gen_range(self.adc_rand_range.0..=self.adc_rand_range.1)
    }

    /// Random value in the IO range.
    fn rand_io(&self) -> u8 {
        let mut rng = rand::thread_rng();
        rng.gen_range(self.io_rand_range.0..=self.io_rand_range.1)
    }

    /// No-op: does not open real hardware.
    pub fn adc_io_open(self) -> Self {
        self
    }

    /// No-op: does not close real hardware.
    pub fn adc_io_close(self) -> Self {
        self
    }

    /// No-op: does not set real IO mode.
    pub fn set_io_mode(self, _index: i32, _mode: BinaryIO) -> Self {
        self
    }

    /// No-op: does not set real IO mode.
    pub fn set_all_io_mode(self, _mode: BinaryIO) -> Self {
        self
    }

    /// No-op: does not set real IO levels.
    pub fn set_all_io_levels(self, _levels: i32) -> Self {
        self
    }

    /// Returns random IO levels.
    pub fn io_all_channels(&self) -> i32 {
        self.rand_io() as i32
    }

    /// Returns random ADC data for all 10 channels.
    pub fn adc_all_channels(&self) -> AdcDataPack {
        let mut buf = [0u16; 10];
        for v in buf.iter_mut() {
            *v = self.rand_adc();
        }
        buf
    }

    /// No-op: does not initialize real MPU.
    pub fn mpu6500_open(self) -> Self {
        self
    }

    /// Returns random accelerometer data.
    pub fn acc_all(&self) -> MpuDataPack {
        [self.rand_mpu(), self.rand_mpu(), self.rand_mpu()]
    }

    /// Returns random gyroscope data.
    pub fn gyro_all(&self) -> MpuDataPack {
        [self.rand_mpu(), self.rand_mpu(), self.rand_mpu()]
    }

    /// Returns random attitude data.
    pub fn atti_all(&self) -> MpuDataPack {
        [self.rand_mpu(), self.rand_mpu(), self.rand_mpu()]
    }

    /// Returns a random IO level (0 or 1).
    pub fn get_io_level(&self, _index: i32) -> i32 {
        let mut rng = rand::thread_rng();
        rng.gen_range(0..=1)
    }

    /// Returns a random IO mode bitmask.
    pub fn get_all_io_mode(&self) -> i32 {
        self.rand_io() as i32
    }
}

impl Default for SensorEmulator {
    fn default() -> Self {
        Self::new((0, u32::MAX), (0, u16::MAX), (0, u8::MAX))
    }
}
