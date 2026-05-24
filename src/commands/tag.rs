use crate::config::AppConfig;
use log::{error, info};
use std::io::Write;
use std::thread;
use std::time::Duration;

/// Detect AprilTags from camera feed and print tag IDs in real time.
pub fn cmd_tag(
    _app_config: AppConfig,
    camera: Option<i32>,
    camera_res_mul: Option<f64>,
    interval: f64,
) {
    use upic_rs::TagDetector;

    let mut detector = match TagDetector::new(camera, camera_res_mul) {
        Ok(d) => d,
        Err(e) => {
            error!("Failed to initialize tag detector: {}", e);
            return;
        }
    };

    match detector.apriltag_detect_start() {
        Ok(_) => {}
        Err(e) => {
            error!("Camera is not ready, exiting...: {}", e);
            return;
        }
    }

    info!("Tag detection started. Press Ctrl+C to stop.");
    loop {
        thread::sleep(Duration::from_secs_f64(interval));
        print!("\rTag: {}", detector.tag_id());
        // Flush stdout so \r inline updates appear immediately
        let _ = std::io::stdout().flush();
    }
}
