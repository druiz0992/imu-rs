pub(crate) mod interpolator;
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

use crate::utils;
use crate::ResamplePolicy;
use common::traits::{IMUFilter, IMUReadings, IMUSample, IMUSource, IMUUntimedSample};
use common::types::sensors::SensorType;
use common::types::Clock;
use publisher::PublisherManager;

const MIN_RESAMPLING_PERIOD_MILLIS: f64 = 5.0;

#[derive(Clone)]
pub struct ResamplerPipeline<T, S>
where
    S: IMUSample,
    T: Send + Sync + IMUReadings<S> + 'static,
    S::Untimed: IMUUntimedSample,
    Average<S::Untimed>: IMUFilter<S>,
    WeightedAverage<S::Untimed>: IMUFilter<S>,
{
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

    async fn apply_resampling_policies(
        &self,
        resampler: &mut Resampler<S>,
        samples: Vec<T>,
        timestamp_now_secs: f64,
    ) -> Vec<(SensorType, S)> {
        let mut resampled_buffer: Vec<_> = Vec::with_capacity(self.sensor_cluster.len());
        for imu_samples in samples.into_iter() {
            let resampled_samples = resampler.resample(&imu_samples, timestamp_now_secs);
            resampled_buffer.push((imu_samples.get_sensor_type(), resampled_samples));
        }
        resampled_buffer
    }

    pub async fn collect_samples_till_timestamp(
        &self,
        timestamp_now_secs: f64,
        period_secs: f64,
    ) -> Vec<T> {
        let mut buffer_clone = utils::clone_and_clear(self.buffer.clone()).await;
        // Apply the filtering logic using the helper function
        for sensor_buffer in buffer_clone.iter_mut() {
            utils::collect_samples(sensor_buffer, timestamp_now_secs, period_secs).await;
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
        resample_policy: ResamplePolicy,
        resampling_period_millis: f64,
    ) -> Result<(), Box<dyn Error>> {
        let resampling_period_millis =
            f64::min(resampling_period_millis, MIN_RESAMPLING_PERIOD_MILLIS);
        let mut resampler = Resampler::<S>::new(
            &self.sensor_cluster,
            resample_policy,
            resampling_period_millis,
        );
        let resampling_duration_secs = Duration::from_secs_f64(resampling_period_millis * 1000.0);

        sleep(resampling_duration_secs).await;

        loop {
            sleep(resampling_duration_secs).await;
            let timestamp_now_secs = Clock::now().as_secs();

            let raw_samples = self
                .collect_samples_till_timestamp(
                    timestamp_now_secs,
                    resampling_duration_secs.as_secs_f64(),
                )
                .await;

            let processed_samples = self
                .apply_resampling_policies(&mut resampler, raw_samples, timestamp_now_secs)
                .await;

            self.notify(processed_samples).await;
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
