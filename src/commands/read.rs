use crate::cli::SensorDevice;
use crate::config::AppConfig;
use log::info;
use std::thread;
use std::time::Duration;

pub fn cmd_read(
    _app_config: AppConfig,
    devices: Vec<SensorDevice>,
    use_screen: bool,
    interval: f64,
    _port: Option<String>,
) {
    use uptechstar_rs::{Color, FontSize, OnBoardSensors, Screen, ScreenDirection};

    let show_all = devices.contains(&SensorDevice::All);
    let show_adc = show_all || devices.contains(&SensorDevice::Adc);
    let show_io = show_all || devices.contains(&SensorDevice::Io);
    let show_mpu = show_all || devices.contains(&SensorDevice::Mpu);

    let mut sensors = OnBoardSensors::default()
        .adc_io_open()
        .mpu6500_open()
        .set_all_io_mode(0);

    let mut screen = Screen::new(None);

    if use_screen {
        screen.open(ScreenDirection::Horizontal as i32);
    }

    info!("Reading sensors (Ctrl+C to stop)...");

    loop {
        if show_adc {
            let adc = sensors.adc_all_channels();
            print!("\x1b[2J\x1b[H"); // clear screen
            println!("=== ADC Channels ===");
            for (i, &v) in adc.iter().enumerate() {
                println!("  ADC[{i}]: {v:5}");
            }
        }

        if show_io {
            let io = OnBoardSensors::io_all_channels();
            let io_mode = OnBoardSensors::get_all_io_mode();
            println!("=== IO Channels ===");
            println!("  Levels:  {io:08b} ({io})");
            println!("  Modes:   {io_mode:08b} ({io_mode})");
        }

        if show_mpu {
            let acc = sensors.acc_all();
            let gyro = sensors.gyro_all();
            let atti = sensors.atti_all();
            println!("=== MPU6500 ===");
            println!(
                "  Accel:   X={:.2} Y={:.2} Z={:.2}",
                acc[0], acc[1], acc[2]
            );
            println!(
                "  Gyro:    X={:.2} Y={:.2} Z={:.2}",
                gyro[0], gyro[1], gyro[2]
            );
            println!(
                "  Attitude: Pitch={:.2} Roll={:.2} Yaw={:.2}",
                atti[0], atti[1], atti[2]
            );
        }

        if use_screen && show_adc {
            let adc = sensors.adc_all_channels();
            screen
                .fill_screen(Color::BLACK)
                .set_font_size(FontSize::Font6x8);
            for (i, &v) in adc.iter().enumerate().take(9) {
                let label = format!("ADC[{}]:{}", i, v);
                screen.put_string(0, (i as i32) * 8, &label);
            }
            screen.refresh();
        }

        thread::sleep(Duration::from_secs_f64(interval));
    }
}
