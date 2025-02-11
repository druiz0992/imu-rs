use common::types::filters::moving_average::MovingAverage;
use common::types::filters::weighted_moving_average::WeightedMovingAverage;
use std::sync::Arc;
use std::{error::Error, marker::PhantomData};
use tokio::sync::Mutex;
use tokio::time::{sleep, Duration};
use uuid::Uuid;

use common::types::sensors::SensorType;
use common::{IMUFilter, IMUReadings, IMUSample, IMUUntimedSample};
use publisher::{Listener, Publishable, Publisher};

use crate::utils;

const MIN_RESAMPLING_PERIOD_MILLIS: u64 = 5;

#[derive(Default)]
pub enum ResamplePolicy {
    Averaging,
    FirstSample,
    LastSample,
    #[default]
    WeightedAverage,
}

pub struct Resampler<S, T>
where
    S: IMUSample,
    T: Send + Sync + IMUReadings<S> + 'static,
    S::Untimed: IMUUntimedSample,
    MovingAverage<S::Untimed>: IMUFilter<S>,
    WeightedMovingAverage<S::Untimed>: IMUFilter<S>,
{
    buffer: Vec<Mutex<T>>,
    publisher: Vec<Publisher<T>>,
    _phantom_data: PhantomData<S>,
}

impl<S, T> Resampler<S, T>
where
    S: IMUSample,
    T: Send + Sync + IMUReadings<S> + 'static,
    S::Untimed: IMUUntimedSample,
    MovingAverage<S::Untimed>: IMUFilter<S>,
    WeightedMovingAverage<S::Untimed>: IMUFilter<S>,
{
    pub fn new(n_buffer: usize, tag: &str) -> Self {
        Self {
            buffer: (0..n_buffer)
                .map(|_| Mutex::new(T::from_vec(tag, SensorType::Accelerometer, vec![])))
                .collect(),
            publisher: (0..n_buffer).map(|_| Publisher::new()).collect(),
            _phantom_data: PhantomData,
        }
    }
    /*
    pub fn request_measurements(&self, source: &dyn IMUSource, sensor_type: SensorType) {
        // get tag
        // let tag = source.get_cluster_tag();
        // if source.is_sensor_available(sensor_type) {
        // if sensor is not availabe, error
        //  todo!();
        // }
        // create listener;
        //   listeer = new listener(self.handle);

        // register listener to IMUSource
        //   let id = source.register_listener(listener, sensor_type)

        // add listener id to collector
        //   self.add_new_measurement(id, tag, sensor_type)
        todo!();

        // methds for IMUSource
        // get_tag()
        // get_sensors()
        // register_listener()
        // unrgister_listener()
        // notify_listeners();

        // method for IMUSink
        // self.handle(uuid, arc<T>)
        // self.add_new_measurement(id, tag, sensor_type)
        // self.remove_measurement(id, sensor_type)
        // self.remove_cluster(id, tag)
    }
    */

    // Registers a accelerometer/gyroscope/magentometer listener functions to be called whenever new samples are available.
    // Returns the id of the registered listener.
    async fn register_sensor(&self, mut listener: Listener<T>, sensor_type: SensorType) -> Uuid {
        let sensor_idx = usize::from(sensor_type);
        self.publisher[sensor_idx]
            .register_listener(&mut listener)
            .await
    }

    // Unregisters a accelerometer/gyroscope/magnetometer listeners from the list of registered listeners.
    async fn unregister_sensor(&self, id: Uuid, sensor_type: SensorType) {
        let sensor_idx = usize::from(sensor_type);
        self.publisher[sensor_idx].unregister_listener(id).await;
    }

    pub async fn handle(&self, _id: Uuid, measurement: Arc<T>) {
        let a = measurement.get_samples().get(0).cloned();
        match a {
            Some(_) => println!("SOMTHING"),
            None => print!("NODAD"),
        }
    }
    pub async fn handle_notification(&self, measurement: T, sensor_type: SensorType) {
        let sensor_idx = usize::from(sensor_type);
        let mut buffer = self.buffer[sensor_idx].lock().await;
        buffer.extend(measurement.get_samples())
    }

    async fn apply_resampling_policies(
        &self,
        samples: Vec<T>,
        resample_policy: &ResamplePolicy,
        resampling_period_millis: f64,
        timestamp: f64,
        cache: &mut Vec<S>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        for (idx, imu_samples) in samples.into_iter().enumerate() {
            let n_samples = imu_samples.get_samples().len();
            // if no samples available, take last sample
            if n_samples > 1 {
                // apply redundant sample policy
                cache[idx] = match resample_policy {
                    ResamplePolicy::Averaging => utils::compute_average(imu_samples.get_samples())?,
                    ResamplePolicy::FirstSample => {
                        imu_samples.get_samples().get(0).cloned().unwrap()
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
                }
            } else if n_samples == 1 {
                cache[idx] = imu_samples.get_samples().get(0).cloned().unwrap();
            }
            {
                let mut buffer = self.buffer[idx].lock().await;
                buffer.extend(vec![cache[idx].clone()]);
            }
        }
        Ok(())
    }

    // v(t) = v_prev + (v_next - v_prev) * (t - t_prev) / (t_next - t_prev)
    fn resample(&self, samples: Vec<[f64; 4]>, t_next: f64, cache: &mut Vec<[f64; 4]>) -> Vec<f64> {
        let mut resampled_values = Vec::new();

        resampled_values.push(t_next);
        for i in 0..samples.len() {
            let prev_sample = cache[i];
            let next_sample = samples[i];
            let timestamp = next_sample[0];

            let t_prev = prev_sample[0];

            for j in 1..4 {
                let v_prev = prev_sample[j];
                let v_next = next_sample[j];
                let mut v =
                    v_prev + (v_next - v_prev) * (timestamp - t_prev).abs() / (t_next - t_prev);
                if v_prev == 0.0 {
                    v = v_next
                }
                /*
                println!(
                    "RESAMPLE: v_prev: {}, v_next: {}, timestamp: {}, t_prev: {}, t_next: {}",
                    v_prev, v_next, timestamp, t_prev, t_next
                );
                */
                resampled_values.push(v);
            }
        }

        resampled_values
    }

    async fn samples_till_timestamp(&self, timestamp: f64, period: f64) -> Vec<T> {
        // Assumption is that we read samples within the timestamp limits, and the rest will be discarded
        // Earlier samples are too old. There shouldnt be any samples past timestamp.

        let mut buffer_clone: Vec<_> = {
            let mut clones = Vec::new();
            for b in self.buffer.iter() {
                let mut buffer = b.lock().await; // Lock and get mutable access
                clones.push(buffer.clone()); // Clone before clearing
                buffer.clear(); // Clear the original buffer
            }
            clones
        };

        for sensor_buffer in buffer_clone.iter_mut() {
            let filtered_samples: Vec<S> = sensor_buffer
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
            sensor_buffer.extend(filtered_samples);
        }

        buffer_clone
    }

    async fn notify_listeners(&self) {
        for (idx, publisher) in self.publisher.iter().enumerate() {
            let mut buffer = self.buffer[idx].lock().await;
            let buffer_clone = buffer.clone();
            buffer.clear();
            publisher.notify_listeners(Arc::new(buffer_clone)).await;
        }
    }
    // TODO: Cant be async.
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
        let mut cache = vec![S::default(); self.buffer.len()];

        sleep(resampling_duration_millis).await;

        loop {
            sleep(resampling_duration_millis).await;

            let raw_samples = self
                .samples_till_timestamp(
                    next_resampling_timestamp,
                    resampling_duration_millis.as_secs_f64(),
                )
                .await;
            /*
            println!(
                "RAW SAMPLES ACC: {}, GY: {}, MA: {}, {:?} {:?}",
                raw_samples[0].len(),
                raw_samples[1].len(),
                raw_samples[2].len(),
                raw_samples,
                next_resampling_timestamp.as_secs_f64()
            );
            */

            self.apply_resampling_policies(
                raw_samples,
                &resampling_policy,
                resampling_duration_millis.as_secs_f64(),
                next_resampling_timestamp,
                &mut cache,
            )
            .await?;

            //println!("RAW SAMPLES AFTER POLICY {:?} {:?}", raw_samples, cache);

            //println!("RESAMPLED - {:?} - {:?}", self.tag.clone(), final_samples);
            self.notify_listeners().await;

            next_resampling_timestamp += resampling_duration_millis.as_secs_f64();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::types::sensors::SensorReadings;
    use common::types::timed::Sample3D;
    use publisher::{listener, Listener, Notifiable};

    #[tokio::test]
    async fn test_callback() {
        let resampler = Arc::new(Resampler::<Sample3D, SensorReadings<_>>::new(3, "test"));
        let listener_resampler = resampler.clone();
        let listener = Listener::new({
            move |_id: Uuid, value: Arc<SensorReadings<Sample3D>>| {
                let resampler = listener_resampler.clone();
                async move { resampler.handle(_id, value).await }
            }
        });

        let callback = listener.get_callback();
        let buffer = resampler.buffer[0].lock().await;
        callback(Uuid::new_v4(), Arc::new(buffer.clone())).await;
    }

    #[tokio::test]
    async fn test_callback_with_macro() {
        let resampler = Arc::new(Resampler::<Sample3D, SensorReadings<_>>::new(3, "test"));
        let listener = listener!(resampler.handle);

        //let listener = listener!(resampler.handle);
        let callback = listener.get_callback();
        let buffer = resampler.buffer[0].lock().await;
        callback(Uuid::new_v4(), Arc::new(buffer.clone())).await;
    }
}
