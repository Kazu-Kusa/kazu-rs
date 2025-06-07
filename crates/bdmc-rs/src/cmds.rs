

/// Restart the device
pub const RESET: &'static [u8] = b"RESET\r";
/// Stop the motor
pub const FULL_STOP: &'static [u8] = b"v0\r";

/// Define counterclockwise direction as positive
pub const ADL: &'static [u8] = b"ADL\r";
/// Define clockwise direction as positive
pub const ADR: &'static [u8] = b"ADR\r";

/// Disable position response
pub const NPOFF: &'static [u8] = b"NPOFF\r";
/// Disable velocity response
pub const NVOFF: &'static [u8] = b"NVOFF\r";
/// Write parameters to the driver's EEPROM
pub const EEPSAVE: &'static [u8] = b"EEPSAVE\r";