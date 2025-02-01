use common::IMUEvent;
use publisher::Publisher;

use std::error::Error;
use std::sync::Arc;
use tokio::time::{sleep, Duration};

use crate::buffer::Buffer;
use crate::utils;

pub struct ResampledSamples(Vec<f64>);

impl IMUEvent for ResampledSamples {
    fn get_samples(&self) -> &Vec<Arc<dyn common::IMUSample>> {}
    fn get_sensor_tag(&self) -> &str {
        "Test"
    }
    fn get_sensor_type(&self) -> usize {
        3
    }
}

#[derive(Default)]
pub enum ResamplePolicy {
    Averaging,
    FirstSample,
    LastSample,
    #[default]
    WeightedAverage,
}

pub struct Resampler {
    buffer: Buffer,
    n_buffer: usize,
    period_millis: Duration,
    publisher: Publisher<Arc<dyn IMUEvent>>,
    resample_policy: ResamplePolicy,
    tag: String,
}

impl Resampler {
    pub fn new(
        n_buffer: usize,
        period_millis: Duration,
        resample_policy: Option<ResamplePolicy>,
        tag: String,
    ) -> Self {
        let mut policy = ResamplePolicy::default();
        if let Some(updated_policy) = resample_policy {
            policy = updated_policy;
        }

        Self {
            period_millis,
            resample_policy: policy,
            buffer: Buffer::new(n_buffer),
            publisher: Publisher::new(),
            n_buffer,
            tag,
        }
    }

    pub fn register<F>(&self, listener: F)
    where
        F: Fn(Arc<dyn IMUEvent>) + Send + Sync + 'static,
    {
        self.publisher.register(listener);
    }

    pub async fn handle_notification(&self, measurement: Arc<dyn IMUEvent>) {
        self.buffer.handle_notification(measurement).await;
    }

    fn apply_resampling_policies(
        &self,
        samples: Vec<Vec<[f64; 4]>>,
        timestamp: f64,
        cache: &mut Vec<[f64; 4]>,
    ) -> Vec<[f64; 4]> {
        let mut parsed_samples: Vec<[f64; 4]> = vec![[0.0; 4]; samples.len()];

        for (idx, imu_samples) in samples.iter().enumerate() {
            // if no samples available, take last sample
            if imu_samples.is_empty() {
                parsed_samples[idx] = cache[idx];
            } else if imu_samples.len() > 1 {
                // apply redundant sample policy
                parsed_samples[idx] = match self.resample_policy {
                    ResamplePolicy::Averaging => utils::compute_average(imu_samples, timestamp),
                    ResamplePolicy::FirstSample => {
                        let sample = imu_samples[0];
                        [timestamp, sample[1], sample[2], sample[3]]
                    }
                    ResamplePolicy::LastSample => {
                        let sample = imu_samples[imu_samples.len() - 1];
                        [timestamp, sample[1], sample[2], sample[3]]
                    }
                    ResamplePolicy::WeightedAverage => utils::compute_weighted_average(
                        imu_samples,
                        timestamp - self.period_millis.as_secs_f64() / 2.0,
                    ),
                }
            } else {
                parsed_samples[idx] = samples[idx][0];
            }
        }

        parsed_samples
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

    fn merge(&self, samples: Vec<[f64; 4]>) -> Vec<f64> {
        let mut resampled_values = Vec::new();
        resampled_values.push(samples[0][0]);

        for sample in samples {
            resampled_values.extend(sample.iter().skip(1).cloned());
        }

        resampled_values
    }

    fn cache_samples(&self, samples: &[f64], cache: Vec<[f64; 4]>) -> Vec<[f64; 4]> {
        let mut new_cache = vec![[0.0; 4]; cache.len()];
        let n_buffers = cache.len();
        for i in 0..n_buffers {
            if 1 + i * 3 + 2 < samples.len() {
                let idx = 1 + i * 3;
                new_cache[i] = [
                    samples[0],       // timestamp
                    samples[idx],     // x component
                    samples[idx + 1], // y component
                    samples[idx + 2], // z component
                ];
            }
        }
        new_cache
    }

    // TODO: Cant be async.
    pub async fn start(&self) -> Result<(), Box<dyn Error>> {
        let mut next_resampling_timestamp = self.period_millis;
        let mut cache = vec![[0.0; 4]; self.n_buffer];

        sleep(self.period_millis).await;

        loop {
            sleep(self.period_millis).await;

            let raw_samples = self
                .buffer
                .samples_till_timestamp(
                    next_resampling_timestamp.as_secs_f64(),
                    self.period_millis.as_secs_f64(),
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

            let raw_samples = self.apply_resampling_policies(
                raw_samples,
                next_resampling_timestamp.as_secs_f64(),
                &mut cache,
            );

            //println!("RAW SAMPLES AFTER POLICY {:?} {:?}", raw_samples, cache);

            /*
            let final_samples = self.resample(
                raw_samples,
                next_resampling_timestamp.as_secs_f64(),
                &mut cache,
            );
            */
            let final_samples = self.merge(raw_samples);

            cache = self.cache_samples(&final_samples, cache);

            //println!("RESAMPLED - {:?} - {:?}", self.tag.clone(), final_samples);
            self.publisher.notify((self.tag.clone(), final_samples));

            next_resampling_timestamp += self.period_millis;
        }
    }
}
