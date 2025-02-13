use async_trait::async_trait;
use common::types::filters::moving_average::MovingAverage;
use common::types::filters::weighted_moving_average::WeightedMovingAverage;
use std::collections::HashMap;
use std::sync::Arc;
use std::{error::Error, marker::PhantomData};
use tokio::sync::{Mutex, RwLock};
use tokio::time::{sleep, Duration};
use uuid::Uuid;

use common::traits::{
    IMUFilter, IMUReadings, IMUSample, IMUSink, IMUSource, IMUUntimedSample, Notifiable,
};
use common::types::sensors::SensorType;
use publisher::listener;
use publisher::Listener;
use publisher::PublisherManager;

use crate::utils;

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
    MovingAverage<S::Untimed>: IMUFilter<S>,
    WeightedMovingAverage<S::Untimed>: IMUFilter<S>,
{
    buffer: Arc<RwLock<HashMap<SensorType, Mutex<T>>>>,
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
    MovingAverage<S::Untimed>: IMUFilter<S>,
    WeightedMovingAverage<S::Untimed>: IMUFilter<S>,
{
    pub fn new(tag: &str, sensor_cluster: Vec<SensorType>) -> Self {
        let buffer: HashMap<SensorType, Mutex<T>> = sensor_cluster
            .iter()
            .map(|s| (s.clone(), Mutex::new(T::from_vec(tag, s.clone(), vec![]))))
            .collect();
        Self {
            buffer: Arc::new(RwLock::new(buffer)),
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
    ) -> Result<(), Box<dyn std::error::Error>> {
        for imu_samples in samples.into_iter() {
            let sensor_type = imu_samples.get_sensor_type();
            let n_samples = imu_samples.get_samples().len();

            // Match on the number of samples
            match n_samples {
                0 => {} // No samples, do nothing
                1 => {
                    let sample = imu_samples.get_samples().first().unwrap().clone();
                    cache.insert(sensor_type.clone(), sample);
                }
                _ => {
                    // Apply redundant sample policy
                    let resampled_samples = match resample_policy {
                        ResamplePolicy::Averaging => {
                            utils::compute_average(imu_samples.get_samples())?
                        }
                        ResamplePolicy::FirstSample => {
                            imu_samples.get_samples().first().unwrap().clone()
                        }
                        ResamplePolicy::LastSample => imu_samples
                            .get_samples()
                            .get(n_samples - 1)
                            .cloned()
                            .unwrap(),
                        ResamplePolicy::WeightedAverage => utils::compute_weighted_average(
                            imu_samples.get_samples(),
                            timestamp - resampling_period_millis / 2.0,
                        )?,
                    };
                    cache.insert(sensor_type.clone(), resampled_samples);
                }
            }
            {
                if let Some(mutex) = self.buffer.read().await.get(&sensor_type) {
                    let mut data = mutex.lock().await;
                    let cache_element = cache.get(&sensor_type).unwrap().clone();
                    data.extend(vec![cache_element]);
                }
            }
        }
        Ok(())
    }

    async fn samples_till_timestamp(&self, timestamp: f64, period: f64) -> Vec<T> {
        // Assumption is that we read samples within the timestamp limits, and the rest will be discarded
        // Earlier samples are too old. There shouldnt be any samples past timestamp.
        let buffer_read = self.buffer.read().await;
        let sensor_types: Vec<&SensorType> = buffer_read.keys().collect();
        let mut buffer_clone: Vec<T> = Vec::new();
        for sensor_type in sensor_types {
            if let Some(mutex) = self.buffer.read().await.get(sensor_type) {
                let mut data = mutex.lock().await;
                buffer_clone.push(data.clone());
                data.clear();
            }
        }

        for sensor_buffer in buffer_clone.iter_mut() {
            let filtered_data: Vec<S> = sensor_buffer
                .get_samples()
                .into_iter()
                .filter_map(|sample| {
                    let sample_timestamp = sample.get_timestamp();
                    let sample_data = sample.get_measurement();
                    if sample_timestamp <= timestamp
                        && sample_timestamp >= timestamp - period * 10.0
                    {
                        Some(S::from_measurement(sample_timestamp, sample_data))
                    } else {
                        None
                    }
                })
                .collect();
            sensor_buffer.clear();
            sensor_buffer.extend(filtered_data);
        }

        buffer_clone
    }

    async fn notify(&self) {
        for sensor_type in &self.sensor_cluster {
            if let Some(mutex) = self.buffer.read().await.get(sensor_type) {
                let mut data = mutex.lock().await;
                let buffer_clone = data.clone();
                data.clear();
                self.notify_listeners(sensor_type.clone(), Arc::new(buffer_clone))
                    .await
            }
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

            self.apply_resampling_policies(
                raw_samples,
                &resampling_policy,
                resampling_duration_millis.as_secs_f64(),
                next_resampling_timestamp,
                &mut cache,
            )
            .await?;

            self.notify().await;

            next_resampling_timestamp += resampling_duration_millis.as_secs_f64();
        }
    }
}

#[async_trait]
impl<T, S> IMUSource<T, S> for Resampler<T, S>
where
    S: IMUSample,
    T: Send + Sync + IMUReadings<S> + 'static,
    S::Untimed: IMUUntimedSample,
    MovingAverage<S::Untimed>: IMUFilter<S>,
    WeightedMovingAverage<S::Untimed>: IMUFilter<S>,
{
    fn get_tag(&self) -> &str {
        self.tag.as_str()
    }

    async fn get_available_sensors(&self) -> Result<Vec<SensorType>, String> {
        Ok(self.publishers.get_available_publisher_types().await)
    }

    async fn unregister_listener(&self, id: Uuid) {
        let _ = self.publishers.remove_listener(id).await;
    }

    async fn register_listener(
        &self,
        listener: &mut dyn Notifiable<T>,
        sensor_type: &SensorType,
    ) -> Result<Uuid, String> {
        self.publishers.add_listener(listener, sensor_type).await
    }

    async fn notify_listeners(&self, sensor_type: SensorType, data: Arc<T>) {
        self.publishers.notify_listeners(sensor_type, data).await
    }
}

#[async_trait]
impl<T, S> IMUSink<T, S> for Resampler<T, S>
where
    S: Send + Sync + IMUSample,
    T: Send + Sync + IMUReadings<S> + 'static,
    MovingAverage<S::Untimed>: IMUFilter<S>,
    WeightedMovingAverage<S::Untimed>: IMUFilter<S>,
{
    async fn attach_listener(
        &self,
        source: &dyn IMUSource<T, S>,
        sensor_type: &SensorType,
    ) -> Result<Uuid, String> {
        let mut listener = listener!(self.process_samples);
        source.register_listener(&mut listener, sensor_type).await
    }
    async fn detach_listener(&self, source: &dyn IMUSource<T, S>, id: Uuid) {
        source.unregister_listener(id).await;
    }
    async fn process_samples(&self, _listener_id: Uuid, samples: Arc<T>) {
        let sensor_type = samples.get_sensor_type();
        if let Some(mutex) = self.buffer.read().await.get(&sensor_type) {
            let mut data = mutex.lock().await;
            data.extend(samples.get_samples());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::traits::Notifiable;
    use common::types::sensors::SensorReadings;
    use common::types::timed::Sample3D;
    use publisher::{listener, Listener};
    use tokio::time::timeout;

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
        let buffer_read = resampler.buffer.read().await;
        let buffer = buffer_read.get(&SensorType::Accelerometer(acc_id)).unwrap();

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
        let buffer_read = resampler.buffer.read().await;
        let buffer = buffer_read.get(&SensorType::Accelerometer(acc_id)).unwrap();

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
