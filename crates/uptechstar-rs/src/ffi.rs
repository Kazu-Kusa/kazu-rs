//! FFI bindings to libuptech.so.
//!
//! On Linux (target platform: Raspberry Pi ARM HF), loads the shared library at runtime.
//! On all other platforms, provides no-op stubs so the crate compiles and can be tested.

#[cfg(target_os = "linux")]
mod real {
    use libloading::{Library, Symbol};
    use log::{debug, error, info};
    use std::sync::OnceLock;

    static LIB: OnceLock<Option<Library>> = OnceLock::new();

    fn lib() -> Option<&'static Library> {
        LIB.get_or_init(|| {
            let path = "/usr/lib/libuptech.so";
            info!("Loading libuptech from {}", path);
            match unsafe { Library::new(path) } {
                Ok(lib) => {
                    info!("libuptech loaded successfully");
                    Some(lib)
                }
                Err(e) => {
                    error!("Failed to load libuptech: {}", e);
                    None
                }
            }
        })
        .as_ref()
    }

    macro_rules! ffi_call {
        ($fn_name:ident($($arg:expr),*) -> $ret:ty) => {{
            if let Some(lib) = lib() {
                unsafe {
                    let func: Symbol<unsafe extern "C" fn($($arg),*) -> $ret> =
                        lib.get(stringify!($fn_name).as_bytes()).unwrap();
                    func($($arg),*)
                }
            } else {
                error!("libuptech not loaded, cannot call {}", stringify!($fn_name));
                Default::default()
            }
        }};
        ($fn_name:ident($($arg:expr),*)) => {{
            if let Some(lib) = lib() {
                unsafe {
                    let func: Symbol<unsafe extern "C" fn($($arg),*)> =
                        lib.get(stringify!($fn_name).as_bytes()).unwrap();
                    func($($arg),*);
                }
            } else {
                error!("libuptech not loaded, cannot call {}", stringify!($fn_name));
            }
        }};
    }

    pub fn adc_io_open() -> i32 {
        ffi_call!(adc_io_open() -> i32)
    }

    pub fn adc_io_close() -> i32 {
        ffi_call!(adc_io_close() -> i32)
    }

    pub fn adc_io_input_get_all() -> i32 {
        ffi_call!(adc_io_InputGetAll() -> i32)
    }

    pub fn adc_io_set_all(levels: u32) -> i32 {
        ffi_call!(adc_io_SetAll(levels) -> i32)
    }

    pub fn adc_io_set(index: u32) -> i32 {
        ffi_call!(adc_io_Set(index) -> i32)
    }

    pub fn adc_io_mode_get_all(mode: &mut u8) -> i32 {
        if let Some(lib) = lib() {
            unsafe {
                let func: Symbol<unsafe extern "C" fn(*mut u8) -> i32> =
                    lib.get(b"adc_io_ModeGetAll").unwrap();
                func(mode as *mut u8)
            }
        } else {
            error!("libuptech not loaded");
            -1
        }
    }

    pub fn adc_io_mode_set(index: u32, mode: i32) -> i32 {
        ffi_call!(adc_io_ModeSet(index, mode) -> i32)
    }

    pub fn adc_get_all(buf: &mut [u16; 10]) -> i32 {
        if let Some(lib) = lib() {
            unsafe {
                let func: Symbol<unsafe extern "C" fn(*mut u16) -> i32> =
                    lib.get(b"ADC_GetAll").unwrap();
                func(buf.as_mut_ptr())
            }
        } else {
            error!("libuptech not loaded");
            -1
        }
    }

    pub fn mpu6500_dmp_init() -> i32 {
        ffi_call!(mpu6500_dmp_init() -> i32)
    }

    pub fn mpu6500_get_accel(buf: &mut [f32; 3]) {
        if let Some(lib) = lib() {
            unsafe {
                let func: Symbol<unsafe extern "C" fn(*mut f32)> =
                    lib.get(b"mpu6500_Get_Accel").unwrap();
                func(buf.as_mut_ptr());
            }
        } else {
            error!("libuptech not loaded");
        }
    }

    pub fn mpu6500_get_gyro(buf: &mut [f32; 3]) {
        if let Some(lib) = lib() {
            unsafe {
                let func: Symbol<unsafe extern "C" fn(*mut f32)> =
                    lib.get(b"mpu6500_Get_Gyro").unwrap();
                func(buf.as_mut_ptr());
            }
        } else {
            error!("libuptech not loaded");
        }
    }

    pub fn mpu6500_get_attitude(buf: &mut [f32; 3]) {
        if let Some(lib) = lib() {
            unsafe {
                let func: Symbol<unsafe extern "C" fn(*mut f32)> =
                    lib.get(b"mpu6500_Get_Attitude").unwrap();
                func(buf.as_mut_ptr());
            }
        } else {
            error!("libuptech not loaded");
        }
    }

    pub fn mpu_get_gyro_fsr(fsr: &mut u16) {
        if let Some(lib) = lib() {
            unsafe {
                let func: Symbol<unsafe extern "C" fn(*mut u16)> =
                    lib.get(b"mpu_get_gyro_fsr").unwrap();
                func(fsr as *mut u16);
            }
        } else {
            error!("libuptech not loaded");
        }
    }

    pub fn mpu_get_accel_fsr(fsr: &mut i8) {
        if let Some(lib) = lib() {
            unsafe {
                let func: Symbol<unsafe extern "C" fn(*mut i8)> =
                    lib.get(b"mpu_get_accel_fsr").unwrap();
                func(fsr as *mut i8);
            }
        } else {
            error!("libuptech not loaded");
        }
    }

    pub fn mpu_set_gyro_fsr(fsr: u32) {
        ffi_call!(mpu_set_gyro_fsr(fsr));
    }

    pub fn mpu_set_accel_fsr(fsr: i32) {
        ffi_call!(mpu_set_accel_fsr(fsr));
    }

    // --- Screen / LCD / LED ---

    pub fn lcd_open(direction: i32) {
        ffi_call!(lcd_open(direction));
    }

    pub fn lcd_close() {
        ffi_call!(lcd_close());
    }

    pub fn lcd_refresh() {
        ffi_call!(LCD_Refresh());
    }

    pub fn lcd_set_font(font: i32) {
        ffi_call!(LCD_SetFont(font));
    }

    pub fn ug_set_forecolor(color: i32) {
        ffi_call!(UG_SetForecolor(color));
    }

    pub fn ug_set_backcolor(color: i32) {
        ffi_call!(UG_SetBackcolor(color));
    }

    pub fn adc_led_set(index: i32, color: i32) {
        ffi_call!(adc_led_set(index, color));
    }

    pub fn ug_fill_screen(color: i32) {
        ffi_call!(UG_FillScreen(color));
    }

    pub fn ug_put_string(x: i32, y: i32, s: &str) {
        if let Some(lib) = lib() {
            let c_str = std::ffi::CString::new(s).unwrap();
            unsafe {
                let func: Symbol<unsafe extern "C" fn(i32, i32, *const i8)> =
                    lib.get(b"UG_PutString").unwrap();
                func(x, y, c_str.as_ptr());
            }
        } else {
            error!("libuptech not loaded");
        }
    }

    pub fn ug_fill_frame(x1: i32, y1: i32, x2: i32, y2: i32, color: i32) {
        ffi_call!(UG_FillFrame(x1, y1, x2, y2, color));
    }

    pub fn ug_fill_round_frame(x1: i32, y1: i32, x2: i32, y2: i32, r: i32, color: i32) {
        ffi_call!(UG_FillRoundFrame(x1, y1, x2, y2, r, color));
    }

    pub fn ug_fill_circle(x0: i32, y0: i32, r: i32, color: i32) {
        ffi_call!(UG_FillCircle(x0, y0, r, color));
    }

    pub fn ug_draw_mesh(x1: i32, y1: i32, x2: i32, y2: i32, color: i32) {
        ffi_call!(UG_DrawMesh(x1, y1, x2, y2, color));
    }

    pub fn ug_draw_frame(x1: i32, y1: i32, x2: i32, y2: i32, color: i32) {
        ffi_call!(UG_DrawFrame(x1, y1, x2, y2, color));
    }

    pub fn ug_draw_round_frame(x1: i32, y1: i32, x2: i32, y2: i32, r: i32, color: i32) {
        ffi_call!(UG_DrawRoundFrame(x1, y1, x2, y2, r, color));
    }

    pub fn ug_draw_pixel(x0: i32, y0: i32, color: i32) {
        ffi_call!(UG_DrawPixel(x0, y0, color));
    }

    pub fn ug_draw_circle(x0: i32, y0: i32, r: i32, color: i32) {
        ffi_call!(UG_DrawCircle(x0, y0, r, color));
    }

    pub fn ug_draw_arc(x0: i32, y0: i32, r: i32, s: i32, color: i32) {
        ffi_call!(UG_DrawArc(x0, y0, r, s, color));
    }

    pub fn ug_draw_line(x1: i32, y1: i32, x2: i32, y2: i32, color: i32) {
        ffi_call!(UG_DrawLine(x1, y1, x2, y2, color));
    }
}

#[cfg(not(target_os = "linux"))]
mod stub {
    use log::warn;

    macro_rules! stub_warn {
        ($name:expr) => {
            warn!("[stub] {} called on non-Linux platform", $name);
        };
    }

    macro_rules! stub_warn_ret {
        ($name:expr, $default:expr) => {{
            warn!("[stub] {} called on non-Linux platform", $name);
            $default
        }};
    }

    pub fn adc_io_open() -> i32 { stub_warn_ret!("adc_io_open", 0) }
    pub fn adc_io_close() -> i32 { stub_warn_ret!("adc_io_close", 0) }
    pub fn adc_io_input_get_all() -> i32 { stub_warn_ret!("adc_io_InputGetAll", 0) }
    pub fn adc_io_set_all(_levels: u32) -> i32 { stub_warn_ret!("adc_io_SetAll", 0) }
    pub fn adc_io_set(_index: u32) -> i32 { stub_warn_ret!("adc_io_Set", 0) }
    pub fn adc_io_mode_get_all(mode: &mut u8) -> i32 { stub_warn!("adc_io_ModeGetAll"); *mode = 0; 0 }
    pub fn adc_io_mode_set(_index: u32, _mode: i32) -> i32 { stub_warn_ret!("adc_io_ModeSet", 0) }
    pub fn adc_get_all(buf: &mut [u16; 10]) -> i32 { stub_warn!("ADC_GetAll"); buf.fill(0); 0 }
    pub fn mpu6500_dmp_init() -> i32 { stub_warn_ret!("mpu6500_dmp_init", 0) }
    pub fn mpu6500_get_accel(buf: &mut [f32; 3]) { stub_warn!("mpu6500_Get_Accel"); buf.fill(0.0); }
    pub fn mpu6500_get_gyro(buf: &mut [f32; 3]) { stub_warn!("mpu6500_Get_Gyro"); buf.fill(0.0); }
    pub fn mpu6500_get_attitude(buf: &mut [f32; 3]) { stub_warn!("mpu6500_Get_Attitude"); buf.fill(0.0); }
    pub fn mpu_get_gyro_fsr(fsr: &mut u16) { stub_warn!("mpu_get_gyro_fsr"); *fsr = 0; }
    pub fn mpu_get_accel_fsr(fsr: &mut i8) { stub_warn!("mpu_get_accel_fsr"); *fsr = 0; }
    pub fn mpu_set_gyro_fsr(_fsr: u32) { stub_warn!("mpu_set_gyro_fsr"); }
    pub fn mpu_set_accel_fsr(_fsr: i32) { stub_warn!("mpu_set_accel_fsr"); }
    pub fn lcd_open(_direction: i32) { stub_warn!("lcd_open"); }
    pub fn lcd_close() { stub_warn!("lcd_close"); }
    pub fn lcd_refresh() { stub_warn!("LCD_Refresh"); }
    pub fn lcd_set_font(_font: i32) { stub_warn!("LCD_SetFont"); }
    pub fn ug_set_forecolor(_color: i32) { stub_warn!("UG_SetForecolor"); }
    pub fn ug_set_backcolor(_color: i32) { stub_warn!("UG_SetBackcolor"); }
    pub fn adc_led_set(_index: i32, _color: i32) { stub_warn!("adc_led_set"); }
    pub fn ug_fill_screen(_color: i32) { stub_warn!("UG_FillScreen"); }
    pub fn ug_put_string(_x: i32, _y: i32, _s: &str) { stub_warn!("UG_PutString"); }
    pub fn ug_fill_frame(_x1: i32, _y1: i32, _x2: i32, _y2: i32, _color: i32) { stub_warn!("UG_FillFrame"); }
    pub fn ug_fill_round_frame(_x1: i32, _y1: i32, _x2: i32, _y2: i32, _r: i32, _color: i32) { stub_warn!("UG_FillRoundFrame"); }
    pub fn ug_fill_circle(_x0: i32, _y0: i32, _r: i32, _color: i32) { stub_warn!("UG_FillCircle"); }
    pub fn ug_draw_mesh(_x1: i32, _y1: i32, _x2: i32, _y2: i32, _color: i32) { stub_warn!("UG_DrawMesh"); }
    pub fn ug_draw_frame(_x1: i32, _y1: i32, _x2: i32, _y2: i32, _color: i32) { stub_warn!("UG_DrawFrame"); }
    pub fn ug_draw_round_frame(_x1: i32, _y1: i32, _x2: i32, _y2: i32, _r: i32, _color: i32) { stub_warn!("UG_DrawRoundFrame"); }
    pub fn ug_draw_pixel(_x0: i32, _y0: i32, _color: i32) { stub_warn!("UG_DrawPixel"); }
    pub fn ug_draw_circle(_x0: i32, _y0: i32, _r: i32, _color: i32) { stub_warn!("UG_DrawCircle"); }
    pub fn ug_draw_arc(_x0: i32, _y0: i32, _r: i32, _s: i32, _color: i32) { stub_warn!("UG_DrawArc"); }
    pub fn ug_draw_line(_x1: i32, _y1: i32, _x2: i32, _y2: i32, _color: i32) { stub_warn!("UG_DrawLine"); }
}

#[cfg(target_os = "linux")]
pub use real::*;

#[cfg(not(target_os = "linux"))]
pub use stub::*;
