/// Restart the device
pub const RESET: &[u8] = b"RESET\r";
/// Stop the motor
pub const FULL_STOP: &[u8] = b"v0\r";

/// Define counterclockwise direction as positive
pub const ADL: &[u8] = b"ADL\r";
/// Define clockwise direction as positive
pub const ADR: &[u8] = b"ADR\r";

/// Disable position response
pub const NPOFF: &[u8] = b"NPOFF\r";
/// Disable velocity response
pub const NVOFF: &[u8] = b"NVOFF\r";
/// Write parameters to the driver's EEPROM
pub const EEPSAVE: &[u8] = b"EEPSAVE\r";
