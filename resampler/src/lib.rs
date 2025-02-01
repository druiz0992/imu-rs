pub mod resampler;

mod buffer;
mod utils;

use log::error;
use common::IMUEvent;
use std::sync::Arc;
use std::time::Duration;

use phyphox_rs::buffer::MeasurementBuffer;
use resampler::{ResamplePolicy, Resampler};

/// Runs the main application logic asynchronously, managing sensors and data processing.
/// Returns a `tokio::task::JoinHandle` representing the asynchronous task running the main logic.
pub fn run<F>(
    sensor_tag: String,
    n_buffer: usize,
    period_update_millis: Duration,
    listeners: Option<Vec<F>>,
    resample_policy: Option<ResamplePolicy>,
) -> (tokio::task::JoinHandle<()>, Arc<Resampler>)
where
    F: Fn(Arc<dyn IMUEvent>) + Send + Sync + 'static,
{
    let resampler = Arc::new(Resampler::new(
        n_buffer,
        period_update_millis,
        resample_policy,
        sensor_tag,
    ));
    let resampler_clone = Arc::clone(&resampler);
    let handle = tokio::spawn({
        async move {
            if let Some(listeners) = listeners {
                for listener in listeners.into_iter() {
                    resampler_clone.register(listener);
                }
            }
            if let Err(e) = resampler_clone.start().await {
                error!("Error in Phyphox loop: {}", e);
            }
        }
    });
    (handle, resampler)
}

pub fn listen_to(resampler: Arc<Resampler>) -> impl Fn(Arc<dyn IMUEvent>) {
    move |buffer: Arc<dyn IMUEvent>| {
        let resampler = Arc::clone(&resampler);
        tokio::spawn(async move {
            resampler.handle_notification(buffer).await;
        });
    }
}
