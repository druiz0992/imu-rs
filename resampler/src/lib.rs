pub mod pipeline;

pub use pipeline::resampler::ResamplePolicy;
pub use pipeline::ResamplerPipeline;

mod utils;

use log::error;
use std::sync::Arc;

use common::traits::imu::{IMUFilter, IMUUntimedSample};
use common::traits::{IMUReadings, IMUSample};
use common::types::filters::{Average, WeightedAverage};
use common::types::sensors::SensorType;

/// Runs the main application logic asynchronously, managing sensors and data processing.
/// Returns a `tokio::task::JoinHandle` representing the asynchronous task running the main logic.
pub fn run<T, S>(
    sensor_tag: &str,
    sensor_cluster: Vec<SensorType>,
    resampling_period_millis: f64,
    resampling_policy: ResamplePolicy,
) -> (tokio::task::JoinHandle<()>, Arc<ResamplerPipeline<T, S>>)
where
    S: IMUSample,
    T: Send + Sync + IMUReadings<S> + 'static,
    S::Untimed: IMUUntimedSample,
    Average<S::Untimed>: IMUFilter<S>,
    WeightedAverage<S::Untimed>: IMUFilter<S>,
{
    let pipeline = Arc::new(ResamplerPipeline::new(sensor_tag, sensor_cluster));

    let pipeline_clone = Arc::clone(&pipeline);
    let handle = tokio::spawn({
        async move {
            if let Err(e) = pipeline_clone
                .start(resampling_policy, resampling_period_millis)
                .await
            {
                error!("Error in Phyphox loop: {}", e);
            }
        }
    });
    (handle, pipeline)
}
