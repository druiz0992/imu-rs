//! # Crate phyphox-rs
//!
//! ## phyphox-rs
//!
//! The `phyphox-rs` crate provides a convenient wrapper to interact with [`Phyphox`](https://phyphox.org/), a mobile phone app
//! that allows to capture sensor data from the smart phone. `phyphox-rs` interacts with a REST API enabled in the phone and
//! records the data from the selected sensors.
//!
//! Features include:
//! - Recording of 3-axis Accelerometer [m/s^2], Gyroscope [rad/s] and Magnetometer (&uT)
//! - Tagging sensors so that readings from different sensor placements can be distinguished.
//! - Selection of read frequency. Note that the sample rate is configured in the mobile app.
//! - Data smoothing with a moving average filter._
//! - Registration of listeners to receive sensor data once received and processed.
//!
//! **NOTE** Currently, `phyphox-rs` only captures data from Accelerometer, Gyroscope and Magnetometer.
//!
//! # Example
//! ```ignore
//!
//! use std::sync::Arc;
//! use std::time::Duration;
//! use tokio::sync::Notify;
//! use tokio::time::sleep;
//! use uuid::Uuid;
//!
//! use phyphox_rs::MeasurementBuffer;
//!
//! #[tokio::main]
//!
//! async fn main() {
//!     let base_url = "http://localhost".to_string();
//!     let sensor_tag = "sensor_1".to_string();
//!     let period_update_millis = Duration::from_millis(100);
//!     let abort_signal = Arc::new(Notify::new());
//!
//!     // Example listener function
//!     let listener = Listener::new(|| {
//!         println!("Received data from {}: {:?}", data.0, data.1);
//!     };
//!     
//!     // Start the sensor processing task
//!     let (task_handle, phyphox) = run(
//!         base_url,
//!         sensor_tag,
//!         period_update_millis,
//!         Arc::clone(&abort_signal),
//!         Some(10)
//!     );
//!
//!     phyphox.register_sensor(listener, SensorType::Accelerometer);
//!
//!     // Let the system run for a few seconds
//!     sleep(Duration::from_secs(5)).await;
//!
//!     // Gracefully stop the background task
//!     println!("Stopping sensor task...");
//!     abort_signal.notify_one();
//!
//!     // Ensure the task finishes before exiting
//!     task_handle.await.unwrap();
//!     println!("Task stopped successfully.");
//! }
//! ```

pub mod adapters;
mod helpers;
pub mod models;
pub mod ports;
pub mod services;

use adapters::production::Phyphox;
use log::error;
use models::errors::PhyphoxError;
use ports::PhyphoxPort;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Notify;

/// Starts the main application logic asynchronously, handling sensor data acquisition and processing.
///
/// This function initializes the required sensors, registers any provided listeners, and begins
/// data collection in a background task. The task will run until the provided `abort_signal` is triggered.
///
/// Returns a tuple containing:
/// - A `tokio::task::JoinHandle<()>` representing the spawned asynchronous task.
/// - An `Arc<Phyphox>` instance, allowing further interaction with the sensor system.
///
/// An error ClientBuild is returned if http client connecting with phyphox app REST API cannot be created.

pub fn run(
    base_url: String,
    sensor_tag: String,
    period_update_millis: Duration,
    abort_signal: Arc<Notify>,
    window_size: Option<usize>,
) -> Result<(tokio::task::JoinHandle<()>, Arc<Phyphox>), PhyphoxError> {
    let phyphox = Arc::new(Phyphox::new(base_url)?);
    let phyphox_clone = Arc::clone(&phyphox);
    let handle = tokio::spawn({
        async move {
            if let Err(e) = phyphox_clone
                .start(period_update_millis, sensor_tag, abort_signal, window_size)
                .await
            {
                error!("Error in Phyphox loop: {:?}", e);
            }
        }
    });
    Ok((handle, phyphox))
}
