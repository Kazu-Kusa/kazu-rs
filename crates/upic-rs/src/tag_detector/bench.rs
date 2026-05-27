use opencv::Result;
use opencv::prelude::*;

/// Benchmark camera frame acquisition performance over multiple samples.
///
/// This utility function measures the time required to read frames from a camera
/// over a specified number of iterations. It's useful for optimizing camera settings
/// and evaluating system performance for real-time applications.
///
/// # Arguments
///
/// * `camera` - OpenCV VideoCapture instance to test. The camera should already
///   be opened and configured before calling this function.
/// * `test_frames_count` - Number of frame read operations to perform for the
///   benchmark. Higher values provide more accurate statistics but take longer
///   to complete.
///
/// # Returns
///
/// Returns the average frame acquisition time in seconds per frame.
///
/// # Examples
///
/// ```rust
/// let mut camera = opencv::videoio::VideoCapture::new(0, opencv::videoio::CAP_ANY)?;
/// let avg_time = test_frame_time(&mut camera, 100)?;
/// let max_fps = 1.0 / avg_time;
/// println!("Average frame time: {:.4}s, Max FPS: {:.1}", avg_time, max_fps);
/// ```
///
/// # Note
///
/// This function performs blocking frame reads and will take significant time
/// to complete based on the test_frames_count parameter. Results may vary based
/// on camera resolution and system load.
pub fn test_frame_time(
    camera: &mut opencv::videoio::VideoCapture,
    test_frames_count: usize,
) -> Result<f64, Box<dyn std::error::Error>> {
    let mut durations = Vec::with_capacity(test_frames_count);
    let mut frame = opencv::core::Mat::default();

    for _ in 0..test_frames_count {
        let start = std::time::Instant::now();
        camera.read(&mut frame)?;
        let duration = start.elapsed().as_secs_f64();
        durations.push(duration);
    }

    let total_duration: f64 = durations.iter().sum();
    let average_duration = total_duration / test_frames_count as f64;

    // Calculate standard deviation
    let variance: f64 = durations
        .iter()
        .map(|&d| (d - average_duration).powi(2))
        .sum::<f64>()
        / (test_frames_count - 1) as f64;
    let std_error = variance.sqrt();

    log::info!(
        "Frame Time Test Results:\n\
        \tRunning on [{}] frame updates\n\
        \tTotal Time Cost: [{:.4}s]\n\
        \tAverage Frame time: [{:.6}s]\n\
        \tStd Error: [{:.6}s]",
        test_frames_count,
        total_duration,
        average_duration,
        std_error
    );

    Ok(average_duration)
}
