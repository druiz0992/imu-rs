pub mod sink;
pub mod source;

use common::types::filters::Average;
use common::types::filters::WeightedAverage;
use dashmap::DashMap;
use std::collections::HashMap;
use std::sync::Arc;
use std::{error::Error, marker::PhantomData};
use tokio::sync::Mutex;
use tokio::time::{sleep, Duration};

use crate::utils;
use common::traits::{IMUFilter, IMUReadings, IMUSample, IMUSource, IMUUntimedSample};
use common::types::sensors::SensorType;
use publisher::PublisherManager;

const MIN_RESAMPLING_PERIOD_MILLIS: u64 = 5;

#[derive(Default, Clone)]
pub enum ResamplePolicy {
    Averaging,
    FirstSample,
    LastSample,
    #[default]
    WeightedAverage,
}

#[derive(Clone)]
pub struct Resampler<T, S>
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

impl<T, S> Resampler<T, S>
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
        samples: Vec<T>,
        resample_policy: &ResamplePolicy,
        resampling_period_millis: f64,
        timestamp: f64,
        cache: &mut HashMap<SensorType, S>,
    ) -> Vec<(SensorType, S)> {
        let mut resampled_buffer: Vec<_> = Vec::with_capacity(self.sensor_cluster.len());
        for imu_samples in samples.into_iter() {
            let resampled_samples = utils::resample(
                &imu_samples,
                timestamp,
                cache,
                resample_policy,
                resampling_period_millis,
            );
            resampled_buffer.push((imu_samples.get_sensor_type(), resampled_samples));
        }
        resampled_buffer
    }

    pub async fn samples_till_timestamp(&self, timestamp: f64, period: f64) -> Vec<T> {
        let mut buffer_clone = utils::clone_and_clear(self.buffer.clone()).await;
        // Apply the filtering logic using the helper function
        for sensor_buffer in buffer_clone.iter_mut() {
            utils::collect_samples(sensor_buffer, timestamp, period).await;
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
        resampling_period_millis: u64,
        resampling_policy: Option<ResamplePolicy>,
    ) -> Result<(), Box<dyn Error>> {
        if resampling_period_millis < MIN_RESAMPLING_PERIOD_MILLIS {
            return Err(Box::<dyn Error>::from(format!(
                "Resampling period needs to be larger than {MIN_RESAMPLING_PERIOD_MILLIS}"
            )));
        }
        let resampling_policy = resampling_policy.unwrap_or_default();
        let resampling_duration_millis = Duration::from_millis(resampling_period_millis);
        let mut next_resampling_timestamp = resampling_duration_millis.as_secs_f64();
        let mut cache = HashMap::<SensorType, S>::new();
        for sensor_type in self.sensor_cluster.iter() {
            cache.insert(sensor_type.clone(), S::default());
        }

        sleep(resampling_duration_millis).await;

        loop {
            sleep(resampling_duration_millis).await;

            let raw_samples = self
                .samples_till_timestamp(
                    next_resampling_timestamp,
                    resampling_duration_millis.as_secs_f64(),
                )
                .await;

            let processed_samples = self
                .apply_resampling_policies(
                    raw_samples,
                    &resampling_policy,
                    resampling_duration_millis.as_secs_f64(),
                    next_resampling_timestamp,
                    &mut cache,
                )
                .await;

            self.notify(processed_samples).await;

            next_resampling_timestamp += resampling_duration_millis.as_secs_f64();
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
        let resampler = Arc::new(Resampler::<SensorReadings<Sample3D>, _>::new(
            "test",
            sensor_cluster,
        ));

        let listener_resampler = resampler.clone();
        let listener = Listener::new({
            move |_id: Uuid, value: Arc<SensorReadings<Sample3D>>| {
                let resampler = listener_resampler.clone();
                async move { resampler.process_samples(_id, value).await }
            }
        });

        let callback = listener.get_callback();
        let buffer = resampler
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
        let resampler = Arc::new(Resampler::<SensorReadings<Sample3D>, _>::new(
            "test",
            sensor_cluster,
        ));
        let listener = listener!(resampler.process_samples);

        //let listener = listener!(resampler.handle);
        let callback = listener.get_callback();
        let buffer = resampler
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
