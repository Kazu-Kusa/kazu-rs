//! `uptechstar-rs` — Rust port of `pyuptech`.
//!
//! Provides FFI bindings to `libuptech.so` for the Raspberry Pi ARM HF platform,
//! with stub implementations on other platforms for development and testing.
//!
//! # Modules
//!
//! - [`sensors`]: On-board sensors (ADC, IO, MPU6500)
//! - [`screen`]: LCD screen, LEDs, fonts, colors
//! - [`emulation`]: Random-data sensor emulation for offline testing
//! - [`pins`]: Pin setter/getter/mode-setter constructor utilities
//! - [`ffi`]: Low-level FFI bindings (internal)

pub mod emulation;
pub mod ffi;
pub mod pins;
pub mod screen;
pub mod sensors;

pub use emulation::SensorEmulator;
pub use screen::{Color, FontSize, Screen, ScreenDirection};
pub use sensors::{AdcDataPack, BinaryIO, MpuDataPack, OnBoardSensors};
