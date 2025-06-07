use serialport::{available_ports, SerialPortType};

/// Finds and returns a list of USB TTY devices with the specified ID product and ID vendor.
/// 
/// # Arguments
/// * `id_product` - the product ID (0 means any product ID)
/// * `id_vendor` - the vendor ID (0 means any vendor ID)
/// 
/// # Returns
/// A vector of USB TTY device paths that match the specified criteria
pub fn find_usb_tty(id_product: u16, id_vendor: u16) -> Vec<String> {
    available_ports()
        .unwrap_or_default()
        .into_iter()
        .filter(|port| {
            if let SerialPortType::UsbPort(usb_info) = &port.port_type {
                (id_vendor == 0 || usb_info.vid == id_vendor) && 
                (id_product == 0 || usb_info.pid == id_product)
            } else {
                false
            }
        })
        .map(|port| port.port_name)
        .collect()
}

/// Finds and returns a list of all available serial ports.
/// 
/// # Returns
/// A vector of serial port device paths
pub fn find_serial_ports() -> Vec<String> {
    available_ports()
        .unwrap_or_default()
        .into_iter()
        .map(|port| port.port_name)
        .collect()
}