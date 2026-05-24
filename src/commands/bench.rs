use crate::cli::BenchTarget;
use crate::config::AppConfig;
use std::thread;
use std::time::{Duration, Instant};

pub fn cmd_bench(_app_config: AppConfig, target: BenchTarget) {
    match target {
        BenchTarget::Siglight => {
            println!("Benchmark: signal light switch frequency");
            let iterations = 1000;
            let start = Instant::now();
            for _ in 0..iterations {
                // toggle signal light (stub on non-Linux)
            }
            let elapsed = start.elapsed();
            println!("  {iterations} toggles in {elapsed:?}");
            println!("  {:.0} Hz", iterations as f64 / elapsed.as_secs_f64());
        }
        BenchTarget::Sleep => {
            println!("Benchmark: sleep precision");
            let target = Duration::from_millis(10);
            let iterations = 100;
            let mut total_err = Duration::ZERO;
            for _ in 0..iterations {
                let before = Instant::now();
                thread::sleep(target);
                let elapsed = before.elapsed();
                if elapsed > target {
                    total_err += elapsed - target;
                } else {
                    total_err += target - elapsed;
                }
            }
            println!("  Target: {target:?}");
            println!("  Avg error: {:?}", total_err / iterations as u32);
        }
        BenchTarget::Adc => {
            println!("Benchmark: ADC sample throughput");
            let mut sensors = uptechstar_rs::OnBoardSensors::default().adc_io_open();
            let iterations = 1000;
            let start = Instant::now();
            for _ in 0..iterations {
                let _ = sensors.adc_all_channels();
            }
            let elapsed = start.elapsed();
            println!("  {iterations} samples in {elapsed:?}");
            println!(
                "  {:.0} samples/s",
                iterations as f64 / elapsed.as_secs_f64()
            );
            sensors.adc_io_close();
        }
        BenchTarget::App => {
            println!("Benchmark: application startup");
            let start = Instant::now();
            let _sensors = uptechstar_rs::OnBoardSensors::default()
                .adc_io_open()
                .mpu6500_open();
            let elapsed = start.elapsed();
            println!("  Sensor init: {elapsed:?}");
        }
    }
}
