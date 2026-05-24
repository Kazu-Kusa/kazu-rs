use crate::config::{load_run_config, AppConfig};
use log::{error, info};
use std::fs;
use std::io::{self, Write};
use std::path::PathBuf;
use std::process;
use std::thread;
use std::time::{Duration, Instant};

pub fn cmd_record(
    _app_config: AppConfig,
    output_dir: PathBuf,
    interval: f64,
    run_config_path: Option<PathBuf>,
) {
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;

    let _run_config = run_config_path
        .as_deref()
        .map(load_run_config)
        .unwrap_or_default();

    if let Err(e) = fs::create_dir_all(&output_dir) {
        error!(
            "Failed to create output directory {}: {}",
            output_dir.display(),
            e
        );
        process::exit(1);
    }

    println!("Press Enter to start recording, Ctrl+C to stop");
    io::stdout().flush().unwrap();

    // Wait for Enter to start
    let mut buf = String::new();
    io::stdin().read_line(&mut buf).unwrap();

    // Try to initialize ADC hardware; fall back to simulated data
    let mut sensors: Option<uptechstar_rs::OnBoardSensors> = None;
    let hardware_available = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        uptechstar_rs::OnBoardSensors::default().adc_io_open()
    }));

    match hardware_available {
        Ok(s) => {
            info!("ADC hardware initialized");
            sensors = Some(s);
        }
        Err(_) => {
            println!("Sensor hardware not available — recording simulated data");
            info!("Falling back to simulated ADC data");
        }
    }

    // Use a background thread to detect Enter press as stop signal
    let stop_flag = Arc::new(AtomicBool::new(false));
    let stop_flag_clone = Arc::clone(&stop_flag);

    let _stdin_thread = thread::spawn(move || {
        let mut _buf = String::new();
        let _ = io::stdin().read_line(&mut _buf);
        stop_flag_clone.store(true, Ordering::Relaxed);
    });

    println!("Recording started. Press Enter to stop.");
    let mut records: Vec<(f64, [u16; 10])> = Vec::new();
    let start = Instant::now();

    while !stop_flag.load(Ordering::Relaxed) {
        let timestamp = start.elapsed().as_secs_f64();

        let adc_values: [u16; 10] = if let Some(ref mut s) = sensors {
            s.adc_all_channels()
        } else {
            // Simulated: simple ramp values based on timestamp
            let t = timestamp as u16;
            [
                t,
                t.wrapping_add(1),
                t.wrapping_add(2),
                t.wrapping_add(3),
                t.wrapping_add(4),
                t.wrapping_add(5),
                t.wrapping_add(6),
                t.wrapping_add(7),
                t.wrapping_add(8),
                t.wrapping_add(9),
            ]
        };

        records.push((timestamp, adc_values));
        thread::sleep(Duration::from_secs_f64(interval));
    }

    // Write CSV
    let timestamp_ms = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis();
    let filename = format!("{}.csv", timestamp_ms);
    let filepath = output_dir.join(&filename);

    match fs::File::create(&filepath) {
        Ok(mut f) => {
            // Write header
            let _ = writeln!(
                f,
                "Timestamp,ADC0,ADC1,ADC2,ADC3,ADC4,ADC5,ADC6,ADC7,ADC8,ADC9"
            );

            for (timestamp, adc) in &records {
                let _ = writeln!(
                    f,
                    "{:.6},{},{},{},{},{},{},{},{},{},{}",
                    timestamp,
                    adc[0],
                    adc[1],
                    adc[2],
                    adc[3],
                    adc[4],
                    adc[5],
                    adc[6],
                    adc[7],
                    adc[8],
                    adc[9],
                );
            }

            println!(
                "Wrote {} records to {}",
                records.len(),
                filepath.display()
            );
        }
        Err(e) => {
            error!("Failed to write CSV file {}: {}", filepath.display(), e);
            process::exit(1);
        }
    }

    // Cleanup ADC if hardware was initialized
    if let Some(s) = sensors {
        s.adc_io_close();
    }

    info!(
        "Record command complete. {} records saved.",
        records.len()
    );
}
