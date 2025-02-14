use crate::utils;
use common::types::filters::Average;
use common::types::filters::WeightedAverage;
use std::collections::HashMap;

use crate::ResamplePolicy;
use common::traits::{IMUFilter, IMUReadings, IMUSample, IMUUntimedSample};
use common::types::sensors::SensorType;

fn apply_resample_policy<T, S>(
    imu_samples: &T,
    resample_policy: &ResamplePolicy,
    timestamp: f64,
    resampling_period_millis: f64,
) -> Option<S>
where
    S: IMUSample,
    T: Send + Sync + IMUReadings<S> + 'static,
    S::Untimed: IMUUntimedSample,
    Average<S::Untimed>: IMUFilter<S>,
    WeightedAverage<S::Untimed>: IMUFilter<S>,
{
    let n_samples = imu_samples.get_samples().len();

    match n_samples {
        0 => None,
        1 => {
            let sample = S::from_measurement(
                timestamp,
                imu_samples.get_samples()[0].clone().get_measurement(),
            );
            Some(sample)
        }
        _ => {
            // Handle case where there are multiple samples
            match resample_policy {
                ResamplePolicy::Averaging => {
                    if let Ok(sample) = utils::compute_average(imu_samples.get_samples()) {
                        Some(S::from_measurement(timestamp, sample.get_measurement()))
                    } else {
                        None
                    }
                }
                ResamplePolicy::FirstSample => Some(S::from_measurement(
                    timestamp,
                    imu_samples.get_samples()[0].get_measurement(),
                )),
                ResamplePolicy::LastSample => Some(S::from_measurement(
                    timestamp,
                    imu_samples.get_samples()[n_samples - 1].get_measurement(),
                )),
                ResamplePolicy::WeightedAverage => utils::compute_weighted_average(
                    imu_samples.get_samples(),
                    timestamp - resampling_period_millis / 2.0,
                )
                .ok(),
            }
        }
    }
}

pub(crate) fn resample<T, S>(
    imu_samples: &T,
    timestamp: f64,
    cache: &mut HashMap<SensorType, S>,
    resample_policy: &ResamplePolicy,
    resampling_period_millis: f64,
) -> S
where
    S: IMUSample,
    T: Send + Sync + IMUReadings<S> + 'static,
    S::Untimed: IMUUntimedSample,
    Average<S::Untimed>: IMUFilter<S>,
    WeightedAverage<S::Untimed>: IMUFilter<S>,
{
    let resampled_samples = match apply_resample_policy(
        imu_samples,
        resample_policy,
        timestamp,
        resampling_period_millis,
    ) {
        None => S::from_measurement(
            timestamp,
            cache
                .get(&imu_samples.get_sensor_type())
                .unwrap()
                .clone()
                .get_measurement(),
        ),
        Some(sample) => sample,
    };

    // Insert the processed sample into cache
    cache.insert(imu_samples.get_sensor_type(), resampled_samples.clone());
    resampled_samples
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::types::sensors::SensorReadings;
    use common::types::timed::Sample3D;
    use uuid::Uuid;

    #[tokio::test]
    async fn test_apply_resampling_policies_averaging() {
        let acc_id = Uuid::new_v4();
        let sensor = SensorType::Accelerometer(acc_id);
        let sample1 = Sample3D::new(950.0, [1.0, 2.0, 3.0]);
        let sample2 = Sample3D::new(960.0, [4.0, 5.0, 6.0]);
        let mut cache = HashMap::new();

        let readings = SensorReadings::from_vec("Test", sensor.clone(), vec![sample1, sample2]);
        cache.insert(SensorType::Accelerometer(acc_id), Sample3D::default());

        let resampled_sample = resample(
            &readings,
            1000.0,
            &mut cache,
            &ResamplePolicy::Averaging,
            100.0,
        );

        assert_eq!(resampled_sample.get_measurement(), [2.5, 3.5, 4.5].into());
        assert_eq!(resampled_sample.get_timestamp(), 1000.0);
        let new_cache = cache.get(&sensor).unwrap();
        assert_eq!(new_cache.get_measurement(), [2.5, 3.5, 4.5].into());
        assert_eq!(new_cache.get_timestamp(), 1000.0);
    }

    #[tokio::test]
    async fn test_apply_resampling_policies_one_sample() {
        let acc_id = Uuid::new_v4();
        let sensor = SensorType::Accelerometer(acc_id);
        let sample1 = Sample3D::new(950.0, [1.0, 2.0, 3.0]);
        let mut cache = HashMap::new();

        let readings = SensorReadings::from_vec("Test", sensor.clone(), vec![sample1]);
        cache.insert(SensorType::Accelerometer(acc_id), Sample3D::default());

        let resampled_sample = resample(
            &readings,
            1000.0,
            &mut cache,
            &ResamplePolicy::Averaging,
            100.0,
        );

        assert_eq!(resampled_sample.get_measurement(), [1.0, 2.0, 3.0].into());
        assert_eq!(resampled_sample.get_timestamp(), 1000.0);
        let new_cache = cache.get(&sensor).unwrap();
        assert_eq!(new_cache.get_measurement(), [1.0, 2.0, 3.0].into());
        assert_eq!(new_cache.get_timestamp(), 1000.0);
    }

    #[tokio::test]
    async fn test_apply_resampling_policies_averaging_no_samples() {
        let acc_id = Uuid::new_v4();
        let sensor = SensorType::Accelerometer(acc_id);
        let mut cache = HashMap::new();

        let readings = SensorReadings::from_vec("Test", sensor.clone(), vec![]);
        cache.insert(SensorType::Accelerometer(acc_id), Sample3D::default());

        let resampled_sample = resample(
            &readings,
            1000.0,
            &mut cache,
            &ResamplePolicy::Averaging,
            100.0,
        );

        assert_eq!(resampled_sample.get_measurement(), [0.0, 0.0, 0.0].into());
        assert_eq!(resampled_sample.get_timestamp(), 1000.0);
        let new_cache = cache.get(&sensor).unwrap();
        assert_eq!(new_cache.get_measurement(), [0.0, 0.0, 0.0].into());
        assert_eq!(new_cache.get_timestamp(), 1000.0);
    }

    #[tokio::test]
    async fn test_apply_resampling_policies_first_sample() {
        let acc_id = Uuid::new_v4();
        let sensor = SensorType::Accelerometer(acc_id);
        let sample1 = Sample3D::new(950.0, [1.0, 2.0, 3.0]);
        let sample2 = Sample3D::new(960.0, [4.0, 5.0, 6.0]);
        let mut cache = HashMap::new();

        let readings = SensorReadings::from_vec("Test", sensor.clone(), vec![sample1, sample2]);
        cache.insert(SensorType::Accelerometer(acc_id), Sample3D::default());

        let resampled_sample = resample(
            &readings,
            1000.0,
            &mut cache,
            &ResamplePolicy::FirstSample,
            100.0,
        );

        assert_eq!(resampled_sample.get_measurement(), [1.0, 2.0, 3.0].into());
        assert_eq!(resampled_sample.get_timestamp(), 1000.0);
        let new_cache = cache.get(&sensor).unwrap();
        assert_eq!(new_cache.get_measurement(), [1.0, 2.0, 3.0].into());
        assert_eq!(new_cache.get_timestamp(), 1000.0);
    }

    #[tokio::test]
    async fn test_apply_resampling_policies_last_sample() {
        let acc_id = Uuid::new_v4();
        let sensor = SensorType::Accelerometer(acc_id);
        let sample1 = Sample3D::new(950.0, [1.0, 2.0, 3.0]);
        let sample2 = Sample3D::new(960.0, [4.0, 5.0, 6.0]);
        let mut cache = HashMap::new();

        let readings = SensorReadings::from_vec("Test", sensor.clone(), vec![sample1, sample2]);
        cache.insert(SensorType::Accelerometer(acc_id), Sample3D::default());

        let resampled_sample = resample(
            &readings,
            1000.0,
            &mut cache,
            &ResamplePolicy::LastSample,
            100.0,
        );

        assert_eq!(resampled_sample.get_measurement(), [4.0, 5.0, 6.0].into());
        assert_eq!(resampled_sample.get_timestamp(), 1000.0);
        let new_cache = cache.get(&sensor).unwrap();
        assert_eq!(new_cache.get_measurement(), [4.0, 5.0, 6.0].into());
        assert_eq!(new_cache.get_timestamp(), 1000.0);
    }

    #[tokio::test]
    async fn test_apply_resampling_policies_weighted_average() {
        let acc_id = Uuid::new_v4();
        let sensor = SensorType::Accelerometer(acc_id);
        let sample1 = Sample3D::new(950.0, [1.0, 2.0, 3.0]);
        let sample2 = Sample3D::new(960.0, [4.0, 5.0, 6.0]);
        let mut cache = HashMap::new();

        let readings =
            SensorReadings::from_vec("Test", sensor.clone(), vec![sample1.clone(), sample2]);
        cache.insert(SensorType::Accelerometer(acc_id), Sample3D::default());

        let resampled_sample = resample(
            &readings,
            1000.0,
            &mut cache,
            &ResamplePolicy::WeightedAverage,
            100.0,
        );

        let eps = 1e-5;
        assert!(
            resampled_sample.get_measurement() - sample1.get_measurement() < [eps, eps, eps].into()
        );
        assert_eq!(resampled_sample.get_timestamp(), 950.0);
        let new_cache = cache.get(&sensor).unwrap();
        assert!(new_cache.get_measurement() - sample1.get_measurement() < [eps, eps, eps].into());
        assert_eq!(new_cache.get_timestamp(), 950.0);
    }
}
