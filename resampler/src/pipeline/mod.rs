pub(crate) mod cache;
pub(crate) mod resampler;
pub mod sink;
pub mod source;

pub(crate) use resampler::Resampler;

use common::types::filters::Average;
use common::types::filters::WeightedAverage;
use dashmap::DashMap;
use std::sync::Arc;
use std::{error::Error, marker::PhantomData};
use tokio::sync::Mutex;
use tokio::time::{sleep, Duration};

use crate::pipeline::cache::{Cache, Interpolable};
use crate::utils;
use crate::SmothingPolicy;
use common::traits::{IMUFilter, IMUReadings, IMUSample, IMUSource, IMUUntimedSample};
use common::types::sensors::SensorType;
use common::types::Clock;
use publisher::PublisherManager;

const MIN_RESAMPLING_PERIOD_MILLIS: f64 = 5.0;

#[derive(Clone)]
pub struct ResamplerPipeline<T, S> {
    // buffer to store samples received from IMU Source
    buffer: Arc<DashMap<SensorType, Mutex<T>>>,
    publishers: PublisherManager<T, SensorType>,
    tag: String,
    sensor_cluster: Vec<SensorType>,
    _phantom_data: PhantomData<S>,
}

impl<T, S> ResamplerPipeline<T, S>
where
    S: IMUSample,
    T: Send + Sync + IMUReadings<S> + 'static,
    S::Untimed: IMUUntimedSample,
    Average<S::Untimed>: IMUFilter<S>,
    WeightedAverage<S::Untimed>: IMUFilter<S>,
    Cache<S, S::Untimed>: Interpolable<S, S::Untimed>,
{
    pub fn new(tag: &str, sensor_cluster: Vec<SensorType>) -> Self {
        let buffer: DashMap<SensorType, Mutex<T>> = sensor_cluster
            .iter()
            .map(|s| (s.clone(), Mutex::new(T::from_vec(tag, s.clone(), vec![]))))
            .collect();
        Self {
            buffer: Arc::new(buffer),
            publishers: PublisherManager::new(&sensor_cluster),
            tag: tag.to_string(),
            sensor_cluster,
            _phantom_data: PhantomData,
        }
    }

    pub async fn collect_samples(&self, buffering_timestamp_secs: f64) -> Vec<T> {
        let mut buffer_clone = utils::clone_and_clear(self.buffer.clone()).await;
        for sensor_buffer in buffer_clone.iter_mut() {
            println!(
                "Resampler processing samples: sensor: {:?}, time{}, n_samples {} ",
                sensor_buffer.get_sensor_type(),
                buffering_timestamp_secs,
                sensor_buffer.get_samples().len()
            );
            utils::collect_samples(sensor_buffer, buffering_timestamp_secs).await;
        }

        buffer_clone
    }

    async fn notify(&self, buffer: Vec<(SensorType, S)>) {
        for (sensor_type, samples) in buffer {
            let readings = T::from_vec(&self.tag, sensor_type.clone(), vec![samples]);
            self.notify_listeners(sensor_type, Arc::new(readings)).await
        }
    }

    pub async fn start(
        &self,
        resample_policy: SmothingPolicy,
        resampling_period_millis: f64,
        resampling_buffering_factor: u32,
    ) -> Result<(), Box<dyn Error>> {
        let resampling_period_millis =
            f64::max(resampling_period_millis, MIN_RESAMPLING_PERIOD_MILLIS);
        let resampling_period_secs = resampling_period_millis / 1000.0;
        let mut resampler = Resampler::<S, S::Untimed>::new(&self.sensor_cluster, resample_policy);
        let resampling_duration_secs = Duration::from_secs_f64(resampling_period_secs);
        let mut resampling_cycles = 1;

        loop {
            sleep(resampling_duration_secs).await;
            let timestamp_now_secs = Clock::now().as_secs();
            let max_sample_age =
                resampling_buffering_factor as f64 * resampling_duration_secs.as_secs_f64();
            let buffering_timestamp = timestamp_now_secs - max_sample_age;
            let resample_timestamp = timestamp_now_secs - max_sample_age / 2.0;

            // collect samples every buffering period = resampling_period * buffering_factor.
            if resampling_buffering_factor == 0
                || resampling_cycles % resampling_buffering_factor == 0
            {
                // raw samples are samples collected by imu source with timestamp after buffering timestamp
                let raw_samples = self.collect_samples(buffering_timestamp).await;

                // smooth collected samples and add timestamp
                resampler.buffer_samples(raw_samples, resample_timestamp);
            }
            let processed_samples = resampler.interpolate(timestamp_now_secs);

            self.notify(processed_samples).await;
            resampling_cycles += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::traits::IMUSink;
    use common::traits::Notifiable;
    use common::types::sensors::SensorReadings;
    use common::types::timed::Sample3D;
    use publisher::{listener, Listener};
    use tokio::time::timeout;
    use uuid::Uuid;

    async fn test_callback() {
        let acc_id = Uuid::new_v4();
        let sensor_cluster = vec![
            SensorType::Accelerometer(acc_id),
            SensorType::Gyroscope(Uuid::new_v4()),
            SensorType::Magnetometer(Uuid::new_v4()),
        ];
        let pipeline = Arc::new(ResamplerPipeline::<SensorReadings<Sample3D>, _>::new(
            "test",
            sensor_cluster,
        ));

        let listener_resampler = pipeline.clone();
        let listener = Listener::new({
            move |_id: Uuid, value: Arc<SensorReadings<Sample3D>>| {
                let resampler = listener_resampler.clone();
                async move { resampler.process_samples(_id, value).await }
            }
        });

        let callback = listener.get_callback();
        let buffer = pipeline
            .buffer
            .get(&SensorType::Accelerometer(acc_id))
            .unwrap();

        let data = buffer.lock().await;
        let snapshot = data.clone();
        drop(data);
        callback(Uuid::new_v4(), Arc::new(snapshot.clone())).await;
    }

    async fn test_callback_with_macro() {
        let acc_id = Uuid::new_v4();
        let sensor_cluster = vec![
            SensorType::Accelerometer(acc_id),
            SensorType::Gyroscope(Uuid::new_v4()),
            SensorType::Magnetometer(Uuid::new_v4()),
        ];
        let pipeline = Arc::new(ResamplerPipeline::<SensorReadings<Sample3D>, _>::new(
            "test",
            sensor_cluster,
        ));
        let listener = listener!(pipeline.process_samples);

        //let listener = listener!(resampler.handle);
        let callback = listener.get_callback();
        let buffer = pipeline
            .buffer
            .get(&SensorType::Accelerometer(acc_id))
            .unwrap();

        let data = buffer.lock().await;
        let snapshot = data.clone();
        drop(data);
        callback(Uuid::new_v4(), Arc::new(snapshot.clone())).await;
    }

    #[tokio::test]
    async fn test_with_timeout() {
        let result = timeout(Duration::from_secs(3), test_callback()).await;
        if result.is_err() {
            panic!("test_callback() aborted after 3 seconds");
        }
        let result = timeout(Duration::from_secs(3), test_callback_with_macro()).await;
        if result.is_err() {
            panic!("test_callback_with_macro() aborted after 3 seconds");
        }
    }
}
