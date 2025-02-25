use crate::utils;
use common::types::filters::Average;
use common::types::filters::WeightedAverage;

use common::traits::{IMUFilter, IMUReadings, IMUSample, IMUUntimedSample};
use common::types::sensors::SensorType;

use super::cache::{Cache, Interpolable};

#[derive(Default, Clone)]
pub enum SmothingPolicy {
    Averaging,
    FirstSample,
    LastSample,
    #[default]
    WeightedAverage,
}

#[derive(Default, Clone)]
pub(crate) struct Resampler<T, U>
where
    T: IMUSample,
    T: IMUSample<Untimed = U>,
{
    interpolator: Cache<T, U>,
    policy: SmothingPolicy,
    sensor_cluster: Vec<SensorType>,
}

impl<T, U> Resampler<T, U>
where
    T: IMUSample + std::fmt::Debug,
    T: IMUSample<Untimed = U>,
    U: IMUUntimedSample,
{
    pub(crate) fn new(sensor_cluster: &[SensorType], policy: SmothingPolicy) -> Self {
        Self {
            policy,
            interpolator: Cache::new(sensor_cluster),
            sensor_cluster: sensor_cluster.to_vec().clone(),
        }
    }

    pub(crate) fn buffer_samples<R>(
        &mut self,
        imu_samples_vec: Vec<R>,
        new_sample_timestamp_secs: f64,
    ) where
        R: Send + Sync + IMUReadings<T> + 'static,
        Average<T::Untimed>: IMUFilter<T>,
        WeightedAverage<T::Untimed>: IMUFilter<T>,
        Cache<T, T::Untimed>: Interpolable<T, T::Untimed>,
    {
        for imu_samples in imu_samples_vec.into_iter() {
            let sensor_type = imu_samples.get_sensor_type();
            let resampled_samples = match self.smoothing(&imu_samples, new_sample_timestamp_secs) {
                None => T::from_measurement(
                    new_sample_timestamp_secs,
                    self.interpolator
                        .peek_newest(&sensor_type)
                        .unwrap()
                        .clone()
                        .get_measurement(),
                ),
                Some(sample) => sample,
            };
            // Insert the processed sample into cache
            self.interpolator
                .push(&sensor_type, resampled_samples.clone());
        }
    }

    pub(crate) fn peek_newest_timestamp(&self) -> f64 {
        self.interpolator
            .peek_newest_timestamp(&self.sensor_cluster[0])
            .unwrap()
    }

    fn smoothing<R>(&self, imu_samples: &R, sample_time: f64) -> Option<T>
    where
        R: Send + Sync + IMUReadings<T> + 'static,
        Average<T::Untimed>: IMUFilter<T>,
        WeightedAverage<T::Untimed>: IMUFilter<T>,
        Cache<T, T::Untimed>: Interpolable<T, T::Untimed>,
    {
        let n_samples = imu_samples.get_samples().len();

        match n_samples {
            0 => None,
            1 => {
                let sample = T::from_measurement(
                    sample_time,
                    imu_samples.get_samples()[0].clone().get_measurement(),
                );
                Some(sample)
            }
            _ => {
                // Handle case where there are multiple samples
                match self.policy {
                    SmothingPolicy::Averaging => {
                        utils::compute_average(sample_time, imu_samples.get_samples()).ok()
                    }
                    SmothingPolicy::FirstSample => Some(T::from_measurement(
                        sample_time,
                        imu_samples.get_samples()[0].get_measurement(),
                    )),
                    SmothingPolicy::LastSample => Some(T::from_measurement(
                        sample_time,
                        imu_samples.get_samples()[n_samples - 1].get_measurement(),
                    )),
                    SmothingPolicy::WeightedAverage => {
                        utils::compute_weighted_average(sample_time, imu_samples.get_samples()).ok()
                    }
                }
            }
        }
    }

    pub(crate) fn interpolate(&mut self, timestamp_now_secs: f64) -> Vec<(SensorType, T)>
    where
        Cache<T, T::Untimed>: Interpolable<T, T::Untimed>,
    {
        self.interpolator.interpolate_samples(timestamp_now_secs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::types::sensors::SensorReadings;
    use common::types::timed::Sample3D;
    use uuid::Uuid;

    #[tokio::test]
    async fn test_smoothing_policy_averaging() {
        let acc_id = Uuid::new_v4();
        let sensor = SensorType::Accelerometer(acc_id);
        let sample1 = Sample3D::new(950.0, [1.0, 2.0, 3.0]);
        let sample2 = Sample3D::new(960.0, [4.0, 5.0, 6.0]);
        let resampler = Resampler::new(&[sensor.clone()], SmothingPolicy::Averaging);

        let readings = SensorReadings::from_vec("Test", sensor, vec![sample1, sample2]);
        let resampled_sample = resampler.smoothing(&readings, 1000.0).unwrap();

        assert_eq!(resampled_sample.get_measurement(), [2.5, 3.5, 4.5].into());
        assert_eq!(resampled_sample.get_timestamp_secs(), 1000.0);
    }

    #[tokio::test]
    async fn test_smoothing_policy_one_sample() {
        let acc_id = Uuid::new_v4();
        let sensor = SensorType::Accelerometer(acc_id);
        let sample1 = Sample3D::new(950.0, [1.0, 2.0, 3.0]);
        let resampler = Resampler::new(&[sensor.clone()], SmothingPolicy::Averaging);

        let readings = SensorReadings::from_vec("Test", sensor.clone(), vec![sample1]);

        let resampled_sample = resampler.smoothing(&readings, 1000.0).unwrap();

        assert_eq!(resampled_sample.get_measurement(), [1.0, 2.0, 3.0].into());
        assert_eq!(resampled_sample.get_timestamp_secs(), 1000.0);
    }

    #[tokio::test]
    async fn test_smoothing_policy_averaging_no_samples() {
        let acc_id = Uuid::new_v4();
        let sensor = SensorType::Accelerometer(acc_id);
        let resampler = Resampler::<Sample3D, _>::new(&[sensor.clone()], SmothingPolicy::Averaging);

        let readings = SensorReadings::from_vec("Test", sensor.clone(), vec![]);
        let resampled_sample = resampler.smoothing(&readings, 1000.0);
        assert!(resampled_sample.is_none());
    }

    #[tokio::test]
    async fn test_smoothing_policy_first_sample() {
        let acc_id = Uuid::new_v4();
        let sensor = SensorType::Accelerometer(acc_id);
        let sample1 = Sample3D::new(950.0, [1.0, 2.0, 3.0]);
        let sample2 = Sample3D::new(960.0, [4.0, 5.0, 6.0]);
        let resampler = Resampler::new(&[sensor.clone()], SmothingPolicy::FirstSample);

        let readings = SensorReadings::from_vec("Test", sensor.clone(), vec![sample1, sample2]);
        let resampled_sample = resampler.smoothing(&readings, 1000.0).unwrap();

        assert_eq!(resampled_sample.get_measurement(), [1.0, 2.0, 3.0].into());
        assert_eq!(resampled_sample.get_timestamp_secs(), 1000.0);
    }

    #[tokio::test]
    async fn test_smoothing_policy_last_sample() {
        let acc_id = Uuid::new_v4();
        let sensor = SensorType::Accelerometer(acc_id);
        let sample1 = Sample3D::new(950.0, [1.0, 2.0, 3.0]);
        let sample2 = Sample3D::new(960.0, [4.0, 5.0, 6.0]);
        let resampler = Resampler::new(&[sensor.clone()], SmothingPolicy::LastSample);

        let readings = SensorReadings::from_vec("Test", sensor.clone(), vec![sample1, sample2]);
        let resampled_sample = resampler.smoothing(&readings, 1000.0).unwrap();

        assert_eq!(resampled_sample.get_measurement(), [4.0, 5.0, 6.0].into());
        assert_eq!(resampled_sample.get_timestamp_secs(), 1000.0);
    }

    #[tokio::test]
    async fn test_smoothing_policy_weighted_average() {
        let acc_id = Uuid::new_v4();
        let sensor = SensorType::Accelerometer(acc_id);
        let sample1 = Sample3D::new(950.0, [1.0, 2.0, 3.0]);
        let sample2 = Sample3D::new(960.0, [4.0, 5.0, 6.0]);
        let resampler = Resampler::new(&[sensor.clone()], SmothingPolicy::WeightedAverage);

        let readings = SensorReadings::from_vec(
            "Test",
            sensor.clone(),
            vec![sample1.clone(), sample2.clone()],
        );
        let resampled_sample1 = resampler.smoothing(&readings, 950.0).unwrap();
        let resampled_sample2 = resampler.smoothing(&readings, 960.0).unwrap();

        let eps = 1e-5;
        assert!(
            resampled_sample1.get_measurement() - sample1.get_measurement()
                < [eps, eps, eps].into()
        );
        assert_eq!(resampled_sample1.get_timestamp_secs(), 950.0);
        assert!(
            resampled_sample2.get_measurement() - sample2.get_measurement()
                < [eps, eps, eps].into()
        );
        assert_eq!(resampled_sample2.get_timestamp_secs(), 960.0);
    }
}
