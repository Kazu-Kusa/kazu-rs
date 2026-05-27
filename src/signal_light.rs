//! Signal light (LED) registry — port of `kazu/signal_light.py`.
//!
//! Manages LED color assignments for different robot states, providing
//! visual feedback during competition.
//!
// Blocked on: uptechstar-rs Screen hardware for real LED output.
#![allow(dead_code)]

use log::warn;
use std::collections::HashMap;

/// RGB color value.
pub type ColorRgb = (u8, u8, u8);

/// A registry that maps purpose labels to LED color patterns.
#[derive(Default)]
pub struct SigLightRegistry {
    /// Named color registrations: purpose → (color, priority).
    registrations: HashMap<String, (ColorRgb, u32)>,
}

impl SigLightRegistry {
    /// Create a new empty registry.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register all LEDs to display a single color for a given purpose.
    pub fn register_all(&mut self, purpose: &str, color: ColorRgb) {
        self.registrations.insert(purpose.to_string(), (color, 0));
    }

    /// Register individual LED colors for a purpose.
    pub fn register_singles(&mut self, purpose: &str, _color0: ColorRgb, _color1: ColorRgb) {
        // In the full port, this would address individual LEDs.
        // For now, store as a combined entry.
        self.registrations.insert(purpose.to_string(), (_color0, 0));
    }

    /// Look up the color for a given purpose.
    pub fn get(&self, purpose: &str) -> Option<&(ColorRgb, u32)> {
        self.registrations.get(purpose)
    }

    /// Remove all registrations.
    pub fn clear(&mut self) {
        self.registrations.clear();
    }
}

/// Global signal light registry (mirrors Python module-level singleton).
/// In production, this would be wired to the actual LCD hardware via `uptechstar-rs`.
pub static mut SIG_LIGHT_REGISTRY: Option<SigLightRegistry> = None;

/// Initialize the global registry (call once at startup).
pub fn init_sig_light() {
    unsafe {
        SIG_LIGHT_REGISTRY = Some(SigLightRegistry::new());
    }
}

/// Set all LEDs to black (off) — emergency shutdown.
pub fn set_all_black() {
    warn!("set_all_black: LED hardware not available on this platform");
}
