pub mod resampler;

mod utils;

use log::error;
use std::sync::Arc;

use common::traits::imu::{IMUFilter, IMUUntimedSample};
use common::traits::{IMUReadings, IMUSample};
use common::types::filters::{MovingAverage, WeightedMovingAverage};
use common::types::sensors::SensorType;
use resampler::{ResamplePolicy, Resampler};

/// Runs the main application logic asynchronously, managing sensors and data processing.
/// Returns a `tokio::task::JoinHandle` representing the asynchronous task running the main logic.
pub fn run<T, S>(
    sensor_tag: &str,
    sensor_cluster: Vec<SensorType>,
    resampling_period_millis: u64,
    resampling_policy: Option<ResamplePolicy>,
) -> (tokio::task::JoinHandle<()>, Arc<Resampler<T, S>>)
where
    S: IMUSample,
    T: Send + Sync + IMUReadings<S> + 'static,
    S::Untimed: IMUUntimedSample,
    MovingAverage<S::Untimed>: IMUFilter<S>,
    WeightedMovingAverage<S::Untimed>: IMUFilter<S>,
{
    let resampler = Arc::new(Resampler::new(sensor_tag, sensor_cluster));
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
