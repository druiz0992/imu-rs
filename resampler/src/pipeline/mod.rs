pub(crate) mod cache;
pub(crate) mod resampler;
pub mod sink;
pub mod source;

pub(crate) use resampler::Resampler;

use common::types::filters::Average;
use common::types::filters::WeightedAverage;
use dashmap::DashMap;
use std::marker::PhantomData;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::pipeline::cache::{Cache, Interpolable};
use crate::utils;
use crate::SmothingPolicy;
use common::traits::{IMUFilter, IMUReadings, IMUSample, IMUSource, IMUUntimedSample};
use common::types::filters::MovingAverage;
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
    S: IMUSample + std::fmt::Debug,
    T: Send + Sync + IMUReadings<S> + std::fmt::Debug + 'static,
    S::Untimed: IMUUntimedSample,
    Average<S::Untimed>: IMUFilter<S>,
    WeightedAverage<S::Untimed>: IMUFilter<S>,
    MovingAverage<S::Untimed>: IMUFilter<S>,
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

    pub fn collect_samples(&self, buffering_timestamp_secs: f64) -> Vec<T> {
        let mut buffer_clone = utils::clone_and_clear(self.buffer.clone());
        for sensor_buffer in buffer_clone.iter_mut() {
            utils::collect_samples(sensor_buffer, buffering_timestamp_secs);
        }

        buffer_clone
    }

    fn notify(&self, buffer: Vec<(SensorType, S)>) {
        for (sensor_type, samples) in buffer {
            let readings = T::from_vec(&self.tag, sensor_type.clone(), vec![samples]);
            self.notify_listeners(sensor_type, Arc::new(readings));
        }
    }

    pub fn start(
        &self,
        resample_policy: SmothingPolicy,
        resampling_period_millis: f64,
        resampling_delay_millis: f64,
    ) {
        let resampling_period_millis =
            f64::max(resampling_period_millis, MIN_RESAMPLING_PERIOD_MILLIS);
        let resampling_period_secs = resampling_period_millis / 1000.0;
        let resampling_delay_secs = resampling_delay_millis / 1000.0;
        let mut resampler = Resampler::<S, S::Untimed>::new(&self.sensor_cluster, resample_policy);
        let resampling_duration_secs = Duration::from_secs_f64(resampling_period_secs);

        loop {
            let start_time = Instant::now();

            let timestamp_now_secs = Clock::now().as_secs();
            let buffering_timestamp = timestamp_now_secs - resampling_delay_secs;
            let resample_timestamp = timestamp_now_secs - resampling_delay_secs / 2.0;

            // collect samples every buffering period = resampling_period * buffering_factor.
            if buffering_timestamp > resampler.peek_newest_timestamp() {
                // raw samples are samples collected by imu source with timestamp after buffering timestamp
                let raw_samples = self.collect_samples(buffering_timestamp);

                // smooth collected samples and add timestamp
                resampler.buffer_samples(raw_samples, resample_timestamp);
            }
            let processed_samples = resampler.interpolate(buffering_timestamp);
            self.notify(processed_samples);

            let elapsed = start_time.elapsed();
            if elapsed < resampling_duration_secs {
                std::thread::sleep(resampling_duration_secs - elapsed);
            }
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
    use std::sync::mpsc;
    use uuid::Uuid;

    fn test_callback() {
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
                resampler.process_samples(_id, value);
            }
        });

        let callback = listener.get_callback();
        let buffer = pipeline
            .buffer
            .get(&SensorType::Accelerometer(acc_id))
            .unwrap();

        let data = buffer.lock().unwrap();
        let snapshot = data.clone();
        drop(data);
        callback(Uuid::new_v4(), Arc::new(snapshot.clone()));
    }

    fn test_callback_with_macro() {
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

        //let listener = async_listener!(resampler.handle);
        let callback = listener.get_callback();
        let buffer = pipeline
            .buffer
            .get(&SensorType::Accelerometer(acc_id))
            .unwrap();

        let data = buffer.lock().unwrap();
        let snapshot = data.clone();
        drop(data);
        callback(Uuid::new_v4(), Arc::new(snapshot.clone()));
    }

    fn run_with_timeout<F, T>(func: F, timeout: Duration) -> Option<T>
    where
        F: FnOnce() -> T + Send + 'static,
        T: Send + 'static,
    {
        let (tx, rx) = mpsc::channel();

        // Spawn a thread to execute the function
        std::thread::spawn(move || {
            let result = func();
            let _ = tx.send(result); // Ignore if the receiver is already closed
        });

        // Wait for the result with a timeout
        rx.recv_timeout(timeout).ok()
    }
    #[test]
    fn test_with_timeout() {
        let result = run_with_timeout(|| test_callback(), Duration::from_secs(3));
        match result {
            None => panic!("test_callback() aborted after 3 seconds"),
            _ => (),
        }

        let result = run_with_timeout(|| test_callback_with_macro(), Duration::from_secs(3));
        match result {
            None => panic!("test_callback_with_macro() aborted after 3 seconds"),
            _ => (),
        }
    }
}
