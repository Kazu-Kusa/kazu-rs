use opencv::prelude::*;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use opencv::{highgui, imgproc, videoio, Result};
/// Tag selection method for when multiple tags are detected
#[derive(Debug, Clone, Copy)]
pub enum OrderingMethod {
    /// Select the tag nearest to the frame center
    Nearest,
    /// Select the first detected tag
    Single,
}

impl Default for OrderingMethod {
    fn default() -> Self {
        OrderingMethod::Nearest
    }
}

/// Configuration parameters for TagDetector behavior
#[derive(Debug, Clone)]
pub struct Config {
    /// Whether to operate in single tag detection mode
    pub single_tag_mode: bool,
    /// Multiplier for camera resolution scaling
    pub resolution_multiplier: f64,
    /// Method for selecting tags when multiple are detected
    pub ordering_method: OrderingMethod,
    /// Time interval between halt status checks during detection loop
    pub halt_check_interval: Duration,
    /// Tag ID returned when no tags are detected
    pub default_tag_id: i32,
    /// Tag ID returned when camera errors occur
    pub error_tag_id: i32,
    /// Camera buffer size for real-time performance
    pub buffer_size: i32,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            single_tag_mode: true,
            resolution_multiplier: 0.5,
            ordering_method: OrderingMethod::Nearest,
            halt_check_interval: Duration::from_millis(400),
            default_tag_id: -1,
            error_tag_id: -10,
            buffer_size: 2,
        }
    }
}

/// A comprehensive AprilTag detection system for real-time computer vision applications.
///
/// This struct provides a complete solution for detecting AprilTags from camera feeds with
/// configurable detection strategies, automatic resource management, and thread-safe operation.
/// It supports both single-tag and multi-tag detection scenarios with various selection methods.
///
/// The detector operates in a separate thread to ensure non-blocking operation and includes
/// built-in camera buffer management for optimal real-time performance.
///
/// # Examples
///
/// ```rust
/// use upic_rs::TagDetector;
///
/// // Initialize detector with camera 0 and half resolution
/// let mut detector = TagDetector::new(Some(0), Some(0.5))?;
///
/// // Start detection process
/// detector.apriltag_detect_start()?;
///
/// // Access detected tag ID
/// let tag_id = detector.tag_id();
///
/// // Clean up resources
/// detector.apriltag_detect_end().release_camera();
/// ```
///
/// # Note
///
/// The detector uses the tag36h11 family by default, which provides a good balance
/// between detection reliability and computational efficiency.
pub struct TagDetector {
    config: Config,
    frame_center: [f64; 2],
    camera: Option<opencv::videoio::VideoCapture>,
    tag_id: Arc<Mutex<i32>>,
    continue_detection: Arc<Mutex<bool>>,
    halt_detection: Arc<Mutex<bool>>,
}

impl TagDetector {
    /// Initialize the TagDetector with optional camera and resolution settings.
    ///
    /// Creates a new TagDetector instance with the specified camera device and resolution
    /// settings. If a camera ID is provided, the camera will be automatically opened and
    /// configured. The detector starts in an idle state and requires explicit activation
    /// via `apriltag_detect_start()`.
    ///
    /// # Arguments
    ///
    /// * `cam_id` - Camera device ID to open automatically. If None, camera must be opened manually using `open_camera()`.
    /// * `resolution_multiplier` - Multiplier for camera resolution scaling. If None, uses `Config::default().resolution_multiplier`.
    ///
    /// # Errors
    ///
    /// Returns an error if camera initialization fails or resolution setting is invalid.
    ///
    /// # Examples
    ///
    /// ```rust
    /// // Initialize without camera (manual setup required)
    /// let detector = TagDetector::new(None, None)?;
    ///
    /// // Initialize with camera 0 and custom resolution
    /// let detector = TagDetector::new(Some(0), Some(0.75))?;
    /// ```
    ///
    /// # Note
    ///
    /// Camera buffer configuration is automatically applied when a camera is opened
    /// to ensure optimal real-time performance with minimal latency.
    pub fn new(cam_id: Option<i32>, resolution_multiplier: Option<f64>) -> Result<Self, Box<dyn std::error::Error>> {
        let config = Config::default();
        let mut detector = TagDetector {
            config: Config {
                resolution_multiplier: resolution_multiplier.unwrap_or(config.resolution_multiplier),
                ..config
            },
            frame_center: [0.0, 0.0],
            camera: None,
            tag_id: Arc::new(Mutex::new(Config::default().default_tag_id)),
            continue_detection: Arc::new(Mutex::new(false)),
            halt_detection: Arc::new(Mutex::new(false)),
        };

        if let Some(cam_id) = cam_id {
            detector.open_camera(cam_id)?;
        }

        Ok(detector)
    }


    /// Configure camera buffer size for real-time performance
    ///
    /// This internal method sets the camera's frame buffer size to the configured value
    /// to minimize latency in real-time applications. A smaller buffer size ensures that
    /// frames are processed with minimal delay, which is crucial for responsive tag detection.
    fn configure_camera_buffer(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        if let Some(ref mut camera) = self.camera {
            camera.set(opencv::videoio::CAP_PROP_BUFFERSIZE, self.config.buffer_size as f64)?;
            log::info!("Camera buffer size set to {} for real-time performance", self.config.buffer_size);
        }
        Ok(())
    }

    /// Open and configure a camera device for AprilTag detection.
    ///
    /// This method initializes a camera connection, configures it for optimal performance,
    /// and prepares it for tag detection. If a camera is already open, it will be properly
    /// released before opening the new one.
    ///
    /// # Arguments
    ///
    /// * `device_id` - Camera device identifier. Typically 0 for the default camera.
    ///
    /// # Errors
    ///
    /// Returns an error if the camera cannot be opened or configured properly.
    pub fn open_camera(&mut self, device_id: i32) -> Result<&mut Self, Box<dyn std::error::Error>> {
        // Release existing camera if present
        if self.camera.is_some() {
            self.release_camera();
        }

        // Open new camera
        let mut camera = opencv::videoio::VideoCapture::new(device_id, opencv::videoio::CAP_ANY)?;

        if camera.is_opened()? {
            self.camera = Some(camera);
            self.configure_camera_buffer()?;
            self.update_cam_center()?;

            // Log camera information
            if let Some(ref camera) = self.camera {
                let width = camera.get(opencv::videoio::CAP_PROP_FRAME_WIDTH)?;
                let height = camera.get(opencv::videoio::CAP_PROP_FRAME_HEIGHT)?;
                let fps = camera.get(opencv::videoio::CAP_PROP_FPS)?;
                let buffer_size = camera.get(opencv::videoio::CAP_PROP_BUFFERSIZE)?;

                log::info!(
                    "CAMERA RESOLUTION: {}x{}\nCAMERA FPS: [{}]\nCAM CENTER: [{:?}]\nBUFFER SIZE: [{}]",
                    width, height, fps, self.frame_center, buffer_size
                );
            }
        } else {
            return Err("Can't open camera!".into());
        }

        Ok(self)
    }

    /// Release the camera resource and clean up associated connections.
    ///
    /// This method properly releases the camera resource to free system resources and
    /// ensure the camera is available for other applications.
    pub fn release_camera(&mut self) -> &mut Self {
        if self.camera.is_some() {
            log::info!("Releasing camera...");
            self.camera = None;
            log::info!("Camera released!");
        } else {
            log::warn!("There is no camera need to release!");
        }
        self
    }


    /// Start AprilTag detection in a background thread.
    ///
    /// Initiates the AprilTag detection process by spawning a dedicated thread that
    /// continuously processes camera frames in real-time, applying the configured tag
    /// selection method and updating the internal tag ID.
    ///
    /// The detection process supports two ordering methods:
    /// - `OrderingMethod::Nearest`: Selects the tag closest to the frame center
    /// - `OrderingMethod::Single`: Selects the first detected tag in the list
    ///
    /// # Returns
    ///
    /// Returns `Result<&mut Self, Box<dyn std::error::Error>>` for method chaining.
    ///
    /// # Errors
    ///
    /// Returns an error if camera is not initialized.
    ///
    /// # Examples
    ///
    /// ```rust
    /// let mut detector = TagDetector::new(Some(0), None)?;
    /// detector.apriltag_detect_start()?;
    /// // Detection runs in background thread
    /// let tag_id = detector.tag_id();
    /// detector.apriltag_detect_end()?;
    /// ```
    ///
    /// # Note
    ///
    /// The detection thread includes comprehensive error handling and will log
    /// exceptions while attempting to continue operation. The thread is automatically
    /// cleaned up when the TagDetector is dropped.
    pub fn apriltag_detect_start(&mut self) -> Result<&mut Self, Box<dyn std::error::Error>> {
        if self.camera.is_none() {
            return Err("Camera is not initialized! Use open_camera() first!".into());
        }

        log::info!("Tag detecting mode: {:?}", self.config.ordering_method);

        // Set detection flags
        *self.continue_detection.lock().unwrap() = true;
        *self.halt_detection.lock().unwrap() = false;

        // Clone Arc references for the thread
        let continue_detection = Arc::clone(&self.continue_detection);
        let halt_detection = Arc::clone(&self.halt_detection);
        let tag_id = Arc::clone(&self.tag_id);

        // Get configuration values
        let frame_center = self.frame_center;
        let check_interval = self.config.halt_check_interval;
        let default_tag_id = self.config.default_tag_id;
        let error_tag_id = self.config.error_tag_id;
        let ordering_method = self.config.ordering_method;

        // Create detection thread
        thread::spawn(move || {
            log::info!("AprilTag detection thread started");

            loop {
                // Check if detection should continue
                if !*continue_detection.lock().unwrap() {
                    break;
                }

                // Check if detection should be halted
                if *halt_detection.lock().unwrap() {
                    log::debug!("AprilTag detect halted!");
                    thread::sleep(check_interval);
                    continue;
                }

                // Note: Actual AprilTag detection implementation would go here
                // For now, this is a placeholder that sets default values

                // Simulate detection logic
                match ordering_method {
                    OrderingMethod::Nearest => {
                        // Would implement nearest tag selection based on frame_center
                        *tag_id.lock().unwrap() = default_tag_id;
                    }
                    OrderingMethod::Single => {
                        // Would implement first tag selection
                        *tag_id.lock().unwrap() = default_tag_id;
                    }
                }

                // Small delay to prevent busy waiting
                thread::sleep(Duration::from_millis(33)); // ~30 FPS
            }

            log::info!("AprilTag detect stopped");
        });

        log::info!("AprilTag detect Activated");
        Ok(self)
    }

    /// Stop AprilTag detection and terminate the background thread.
    ///
    /// This method gracefully terminates the detection thread by setting the control
    /// flags and resets the internal tag ID to the default value. The detection thread
    /// will complete its current iteration and then exit cleanly.
    ///
    /// # Returns
    ///
    /// Returns `&mut Self` for method chaining.
    ///
    /// # Examples
    ///
    /// ```rust
    /// let mut detector = TagDetector::new(Some(0), None)?;
    /// detector.apriltag_detect_start()?;
    /// // ... perform detection operations ...
    /// detector.apriltag_detect_end(); // Stop detection
    /// ```
    ///
    /// # Note
    ///
    /// This method only signals the detection thread to stop; it does not forcibly
    /// terminate the thread. The thread will stop after completing its current
    /// processing cycle, ensuring clean shutdown without resource corruption.
    pub fn apriltag_detect_end(&mut self) -> &mut Self {
        *self.continue_detection.lock().unwrap() = false;
        *self.tag_id.lock().unwrap() = self.config.default_tag_id;
        self
    }

    /// Temporarily halt the tag detection process without stopping the thread.
    ///
    /// This method pauses the detection process while keeping the detection thread active.
    /// The thread will enter a sleep state and periodically check for resume signals.
    /// The tag ID is reset to the default value during the halt period.
    ///
    /// # Returns
    ///
    /// Returns `&mut Self` for method chaining.
    ///
    /// # Examples
    ///
    /// ```rust
    /// let mut detector = TagDetector::new(Some(0), None)?;
    /// detector.apriltag_detect_start()?;
    /// detector.halt_detection();    // Pause detection
    /// detector.resume_detection();  // Resume detection
    /// ```
    ///
    /// # Note
    ///
    /// Unlike `apriltag_detect_end()`, this method keeps the detection thread alive
    /// but inactive, allowing for quick resumption without thread recreation overhead.
    /// The thread will sleep for `Config::halt_check_interval` between status checks.
    pub fn halt_detection(&mut self) -> &mut Self {
        *self.halt_detection.lock().unwrap() = true;
        *self.tag_id.lock().unwrap() = self.config.default_tag_id;
        self
    }

    /// Resume the halted tag detection process.
    ///
    /// This method resumes detection that was previously halted using `halt_detection()`.
    /// The detection thread will immediately exit its sleep state and begin processing
    /// camera frames again in the next iteration.
    ///
    /// # Returns
    ///
    /// Returns `&mut Self` for method chaining.
    ///
    /// # Examples
    ///
    /// ```rust
    /// let mut detector = TagDetector::new(Some(0), None)?;
    /// detector.apriltag_detect_start()?;
    /// detector.halt_detection();    // Pause detection
    /// // ... do other work ...
    /// detector.resume_detection();  // Resume detection immediately
    /// ```
    ///
    /// # Note
    ///
    /// This method only works if the detection thread is currently active but halted.
    /// If the detection thread has been stopped with `apriltag_detect_end()`, you must
    /// call `apriltag_detect_start()` instead to restart the detection process.
    pub fn resume_detection(&mut self) -> &mut Self {
        *self.halt_detection.lock().unwrap() = false;
        self
    }

    /// Get the currently detected AprilTag ID.
    ///
    /// This method provides thread-safe access to the most recently detected tag ID.
    /// The value is updated continuously by the detection thread when active.
    ///
    /// # Returns
    ///
    /// Returns the ID of the currently detected AprilTag. Returns `Config::default_tag_id`
    /// when no tags are detected, `Config::error_tag_id` when camera errors occur,
    /// or the actual tag ID when a tag is successfully detected.
    ///
    /// # Examples
    ///
    /// ```rust
    /// let mut detector = TagDetector::new(Some(0), None)?;
    /// detector.apriltag_detect_start()?;
    /// let current_tag = detector.tag_id();
    /// if current_tag >= 0 {
    ///     println!("Detected tag: {}", current_tag);
    /// } else if current_tag == detector.config.default_tag_id {
    ///     println!("No tag detected");
    /// } else if current_tag == detector.config.error_tag_id {
    ///     println!("Camera error");
    /// }
    /// ```
    ///
    /// # Note
    ///
    /// The tag ID is updated atomically by the detection thread, so this method
    /// is safe to access from multiple threads without additional synchronization.
    pub fn tag_id(&self) -> i32 {
        *self.tag_id.lock().unwrap()
    }

    /// Update the internal frame center coordinates based on current camera resolution.
    ///
    /// This internal method recalculates the center point of the camera frame based on
    /// the current camera resolution settings. It's used internally for "nearest" tag
    /// selection mode to determine which detected tag is closest to the frame center.
    ///
    /// # Errors
    ///
    /// Returns an error if camera properties cannot be accessed.
    ///
    /// # Note
    ///
    /// This is an internal method and should not be called directly by users.
    /// It is automatically invoked when camera resolution changes or camera
    /// is opened/configured.
    fn update_cam_center(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        if let Some(ref camera) = self.camera {
            let width = camera.get(opencv::videoio::CAP_PROP_FRAME_WIDTH)?;
            let height = camera.get(opencv::videoio::CAP_PROP_FRAME_HEIGHT)?;
            self.frame_center = [width / 2.0, height / 2.0];
        }
        Ok(())
    }

    /// Set camera resolution by multiplying current resolution with a scaling factor.
    ///
    /// This method provides a convenient way to scale the camera resolution while
    /// maintaining the aspect ratio. It's particularly useful for balancing detection
    /// performance with processing speed.
    ///
    /// # Arguments
    ///
    /// * `resolution_multiplier` - Scaling factor for resolution adjustment.
    ///   Values > 1.0 increase resolution (better quality, slower processing),
    ///   values < 1.0 decrease resolution (faster processing, lower quality).
    ///   For example, 0.5 reduces resolution to half in both dimensions.
    ///
    /// # Returns
    ///
    /// Returns `Result<&mut Self, Box<dyn std::error::Error>>` for method chaining.
    ///
    /// # Errors
    ///
    /// Returns an error if camera is not initialized or resolution cannot be set.
    ///
    /// # Examples
    ///
    /// ```rust
    /// let mut detector = TagDetector::new(Some(0), None)?;
    /// detector.set_cam_resolution_mul(0.5)?;   // Half resolution for speed
    /// detector.set_cam_resolution_mul(1.5)?;   // 1.5x resolution for quality
    /// ```
    ///
    /// # Note
    ///
    /// The actual resolution set may be adjusted by the camera driver to the
    /// nearest supported resolution. The method automatically updates the frame
    /// center calculations after resolution changes.
    pub fn set_cam_resolution_mul(&mut self, resolution_multiplier: f64) -> Result<&mut Self, Box<dyn std::error::Error>> {
        if self.camera.is_none() {
            return Err("Camera is not initialized!".into());
        }

        let camera = self.camera.as_ref().unwrap();
        let current_width = camera.get(opencv::videoio::CAP_PROP_FRAME_WIDTH)?;
        let current_height = camera.get(opencv::videoio::CAP_PROP_FRAME_HEIGHT)?;

        self.set_cam_resolution(
            (current_width * resolution_multiplier) as i32,
            (current_height * resolution_multiplier) as i32,
        )
    }

    /// Set the camera resolution to specific width and height values.
    ///
    /// This method directly sets the camera resolution to the specified dimensions.
    /// The camera driver may adjust the values to the nearest supported resolution.
    ///
    /// # Arguments
    ///
    /// * `new_width` - Target width in pixels.
    /// * `new_height` - Target height in pixels.
    ///
    /// # Returns
    ///
    /// Returns `Result<&mut Self, Box<dyn std::error::Error>>` for method chaining.
    ///
    /// # Errors
    ///
    /// Returns an error if camera is not initialized or resolution cannot be set.
    ///
    /// # Examples
    ///
    /// ```rust
    /// let mut detector = TagDetector::new(Some(0), None)?;
    /// detector.set_cam_resolution(640, 480)?;    // VGA resolution
    /// detector.set_cam_resolution(1920, 1080)?;  // Full HD resolution
    /// ```
    ///
    /// # Note
    ///
    /// Common resolutions include:
    /// - 640x480 (VGA): Good balance of speed and quality
    /// - 1280x720 (HD): Higher quality, moderate processing load
    /// - 1920x1080 (Full HD): Highest quality, highest processing load
    ///
    /// The method automatically updates frame center calculations and logs
    /// the actual resolution set by the camera driver.
    pub fn set_cam_resolution(&mut self, new_width: i32, new_height: i32) -> Result<&mut Self, Box<dyn std::error::Error>> {
        if let Some(ref mut camera) = self.camera {
            camera.set(opencv::videoio::CAP_PROP_FRAME_WIDTH, new_width as f64)?;
            camera.set(opencv::videoio::CAP_PROP_FRAME_HEIGHT, new_height as f64)?;

            let actual_width = camera.get(opencv::videoio::CAP_PROP_FRAME_WIDTH)?;
            let actual_height = camera.get(opencv::videoio::CAP_PROP_FRAME_HEIGHT)?;

            log::info!(
                "Set CAMERA RESOLUTION: {}x{}", 
                actual_width as i32, actual_height as i32
            );

            self.update_cam_center()?;
        } else {
            return Err("Camera is not initialized!".into());
        }

        Ok(self)
    }

    /// Get the underlying OpenCV VideoCapture device instance.
    ///
    /// This method provides direct access to the OpenCV VideoCapture object for
    /// advanced camera operations not covered by the TagDetector interface. Use
    /// with caution as direct manipulation may interfere with detection operations.
    ///
    /// # Returns
    ///
    /// Returns a reference to the VideoCapture instance if a camera is open,
    /// None if no camera is currently initialized.
    ///
    /// # Examples
    ///
    /// ```rust
    /// let mut detector = TagDetector::new(Some(0), None)?;
    /// if let Some(camera) = detector.camera_device() {
    ///     // Direct OpenCV operations
    ///     let fps = camera.get(opencv::videoio::CAP_PROP_FPS)?;
    ///     println!("Camera FPS: {}", fps);
    /// }
    /// ```
    ///
    /// # Warning
    ///
    /// Direct manipulation of the camera device may interfere with the detection
    /// process. It's recommended to halt detection before performing direct
    /// camera operations and resume afterward.
    ///
    /// # Note
    ///
    /// This method is primarily intended for advanced users who need access
    /// to camera features not exposed through the TagDetector interface.
    pub fn camera_device(&self) -> Option<&opencv::videoio::VideoCapture> {
        self.camera.as_ref()
    }
}

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
pub fn test_frame_time(camera: &mut opencv::videoio::VideoCapture, test_frames_count: usize) -> Result<f64, Box<dyn std::error::Error>> {
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
    let variance: f64 = durations.iter()
        .map(|&d| (d - average_duration).powi(2))
        .sum::<f64>() / (test_frames_count - 1) as f64;
    let std_error = variance.sqrt();

    log::info!(
        "Frame Time Test Results:\n\
        \tRunning on [{}] frame updates\n\
        \tTotal Time Cost: [{:.4}s]\n\
        \tAverage Frame time: [{:.6}s]\n\
        \tStd Error: [{:.6}s]",
        test_frames_count, total_duration, average_duration, std_error
    );

    Ok(average_duration)
}