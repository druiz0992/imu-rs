pub mod pipeline;

pub use pipeline::resampler::SmothingPolicy;
pub use pipeline::ResamplerPipeline;

mod utils;

use std::sync::Arc;
use tokio::runtime::Runtime;

use crate::pipeline::cache::{Cache, Interpolable};
use common::traits::imu::{IMUFilter, IMUUntimedSample};
use common::traits::{IMUReadings, IMUSample};
use common::types::filters::{Average, MovingAverage, WeightedAverage};
use common::types::sensors::SensorType;

/// Runs the main application logic asynchronously, managing sensors and data processing.
/// Returns a `tokio::task::JoinHandle` representing the asynchronous task running the main logic.
pub fn run<T, S>(
    sensor_tag: &str,
    sensor_cluster: Vec<SensorType>,
    resampling_period_millis: f64,
    resampling_delay_millis: f64,
    smoothing_policy: SmothingPolicy,
) -> (std::thread::JoinHandle<()>, Arc<ResamplerPipeline<T, S>>)
where
    S: IMUSample + std::fmt::Debug,
    T: Send + Sync + IMUReadings<S> + std::fmt::Debug + 'static,
    S::Untimed: IMUUntimedSample,
    Average<S::Untimed>: IMUFilter<S>,
    MovingAverage<S::Untimed>: IMUFilter<S>,
    WeightedAverage<S::Untimed>: IMUFilter<S>,
    Cache<S, S::Untimed>: Interpolable<S, S::Untimed>,
{
    let pipeline = Arc::new(ResamplerPipeline::new(sensor_tag, sensor_cluster));

    let pipeline_clone = Arc::clone(&pipeline);

    let handle = std::thread::spawn(move || {
        let rt = Runtime::new().unwrap();
        rt.block_on(async {
            pipeline_clone
                .start(
                    smoothing_policy,
                    resampling_period_millis,
                    resampling_delay_millis,
                )
                .await
        })
    });

    (handle, pipeline)
}
