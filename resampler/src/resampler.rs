use std::sync::Arc;
use std::{error::Error, marker::PhantomData};
use tokio::sync::Mutex;
use tokio::time::{sleep, Duration};
use uuid::Uuid;

use common::{IMUReadings, IMUSample, SensorType};
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
{
    buffer: Vec<Mutex<T>>,
    publisher: Vec<Publisher<T>>,
    _phantom_data: PhantomData<S>,
}

impl<S, T> Resampler<S, T>
where
    S: IMUSample,
    T: Send + Sync + IMUReadings<S> + Default + 'static,
{
    pub fn new(n_buffer: usize, tag: String) -> Self {
        Self {
            buffer: (0..n_buffer).map(|_| Mutex::new(T::default())).collect(),
            publisher: (0..n_buffer).map(|_| Publisher::new()).collect(),
            _phantom_data: PhantomData,
        }
    }

    // Registers a accelerometer/gyroscope/magentometer listener functions to be called whenever new samples are available.
    // Returns the id of the registered listener.
    fn register_sensor(&self, listener: Listener<T>, sensor_type: SensorType) -> Uuid {
        self.publisher[sensor_type as usize].register_listener(&listener)
    }

    // Unregisters a accelerometer/gyroscope/magnetometer listeners from the list of registered listeners.
    fn unregister_sensor(&self, id: Uuid, sensor_type: SensorType) {
        self.publisher[sensor_type as usize].unregister_listener(id);
    }

    pub async fn handle(&self, measurement: Arc<T>) {
        let a = measurement.get_samples().get(0).cloned();
        match a {
            Some(_) => println!("SOMTHING"),
            None => print!("NODAD"),
        }
    }
    pub async fn handle_notification(&self, measurement: T, sensor_type: SensorType) {
        let mut buffer = self.buffer[sensor_type as usize].lock().await;
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
                    ResamplePolicy::Averaging => {
                        utils::compute_average(imu_samples.get_samples(), timestamp)?
                    }
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
                .into_iter_samples()
                .filter_map(|sample| {
                    let sample_timestamp = sample.get_timestamp();
                    let sample_data = sample.get_measurement();
                    if sample_timestamp <= timestamp
                        && sample_timestamp >= timestamp - period * 10.0
                    {
                        Some(S::from_untimed(sample_data, sample_timestamp))
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
    use common::{Sample3D, SensorReadings};
    use publisher::{listener, Listener, Notifiable};

    #[tokio::test]
    async fn test_register_sensor() {
        let resampler = Arc::new(Resampler::<Sample3D, SensorReadings<_>>::new(
            3,
            "test".to_string(),
        ));
        let listener_resampler = resampler.clone();
        let listener = Listener::new({
            move |value: Arc<SensorReadings<Sample3D>>| {
                let resampler = listener_resampler.clone();
                async move { resampler.handle(value).await }
            }
        });
        let listener2 = listener!(resampler.handle);

        //let listener = listener!(resampler.handle);
        let callback = listener2.get_callback();
        let buffer = resampler.buffer[0].lock().await;
        callback(Arc::new(buffer.clone())).await;
    }
}
