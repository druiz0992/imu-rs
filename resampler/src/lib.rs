pub mod resampler;

mod utils;

use log::error;
use std::sync::Arc;

use common::{IMUReadings, IMUSample};
use publisher::Notifiable;
use resampler::{ResamplePolicy, Resampler};

/// Runs the main application logic asynchronously, managing sensors and data processing.
/// Returns a `tokio::task::JoinHandle` representing the asynchronous task running the main logic.
pub fn run<S, T>(
    sensor_tag: String,
    n_buffer: usize,
    resampling_period_millis: u64,
    resampling_policy: Option<ResamplePolicy>,
) -> (tokio::task::JoinHandle<()>, Arc<Resampler<S, T>>)
where
    S: IMUSample,
    T: Send + Sync + IMUReadings<S> + Default + 'static,
{
    let resampler = Arc::new(Resampler::new(n_buffer, sensor_tag));
    let resampler_clone = Arc::clone(&resampler);
    let handle = tokio::spawn({
        async move {
            if let Err(e) = resampler_clone
                .start(resampling_period_millis, resampling_policy)
                .await
            {
                error!("Error in Phyphox loop: {}", e);
            }
        }
    });
    (handle, resampler)
}
