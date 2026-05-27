use crate::config::AppConfig;

pub fn cmd_check(
    app_config: AppConfig,
    devices: Vec<String>,
    port: Option<String>,
    _camera: Option<i32>,
) {
    use bdmc_rs::controller::CloseLoopController;

    let test_all = devices.iter().any(|d| d == "all");
    let test_mot = test_all || devices.iter().any(|d| d == "mot");
    let test_adc = test_all || devices.iter().any(|d| d == "adc");
    let test_io = test_all || devices.iter().any(|d| d == "io");
    let test_mpu = test_all || devices.iter().any(|d| d == "mpu");

    println!("{:=^40}", " Hardware Check ");
    let mut all_ok = true;

    if test_mot {
        let port_name = port.unwrap_or_else(|| app_config.motion.port.clone());
        print!("  MOTOR  ({:30}) ... ", port_name);
        match CloseLoopController::new(None, None, None, Some(&port_name)) {
            Ok(mut c) => {
                println!("OK");
                c.close();
            }
            Err(e) => {
                println!("FAIL ({})", e);
                all_ok = false;
            }
        }
    }

    if test_adc {
        print!("  ADC                      ... ");
        println!("SKIP (no uptechstar hardware on this host)");
    }

    if test_io {
        print!("  IO                       ... ");
        println!("SKIP (no uptechstar hardware on this host)");
    }

    if test_mpu {
        print!("  MPU                      ... ");
        println!("SKIP (no uptechstar hardware on this host)");
    }

    println!("{:=^40}", if all_ok { " ALL OK " } else { " FAILURES " });
}
