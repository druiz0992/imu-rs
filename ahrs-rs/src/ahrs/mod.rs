pub(crate) mod buffer;
pub(crate) mod sink;
pub(crate) mod source;

use std::sync::{Arc, Mutex};

use ahrs::{Ahrs, Madgwick};

use crate::utils;
use buffer::{AHRSInputSamples, SensorIndex, N_SENSORS};
use common::traits::IMUSample;
use common::types::sensors::{SensorReadings, SensorType};
use common::types::timed::SampleQuaternion;
use common::types::untimed::UnitQuaternion;
use publisher::PublisherManager;

const MADGWICK_BETA: f64 = 0.08;
const DISCARD_N_INITIAL_SAMPLES: usize = 100;

pub struct AHRSFilterManager {
    ahrs_filter: Madgwick<f64>,
    buffer: AHRSInputSamples,
    cache: UnitQuaternion,
    sensor_cluster: [SensorType; N_SENSORS],
    n_samples: usize,
}

impl AHRSFilterManager {
    fn new(
        sensor_cluster: Vec<SensorType>,
        sampling_period_millis: f64,
    ) -> Result<Self, &'static str> {
        let sensor_cluster: [SensorType; N_SENSORS] = sensor_cluster
            .try_into()
            .map_err(|_| "Invalid sensor cluster")?;
        if !utils::check_sensor_cluster(&sensor_cluster) {
            return Err("Invalid sensor cluster");
        }

        Ok(Self {
            ahrs_filter: Madgwick::new(sampling_period_millis / 1000.0, MADGWICK_BETA),
            buffer: AHRSInputSamples::new(),
            sensor_cluster,
            cache: UnitQuaternion::default(),
            n_samples: 0,
        })
    }

    fn update_filter(&mut self, buffer: AHRSInputSamples) -> SampleQuaternion {
        let gyro = buffer
            .get_samples_by_index(usize::from(SensorIndex::Gyroscope))
            .unwrap();
        let accel = buffer
            .get_samples_by_index(usize::from(SensorIndex::Accelerometer))
            .unwrap();
        let mag = buffer
            .get_samples_by_index(usize::from(SensorIndex::Magnetometer))
            .unwrap();
        let q = match self.ahrs_filter.update(&gyro, &accel, &mag) {
            Ok(q) => q,
            Err(_) => &self.cache.inner(),
        };
        self.n_samples += 1;
        let sample_quaternion = SampleQuaternion::from_unit_quaternion(
            buffer.get_timestamp(),
            UnitQuaternion::from_unit_quaternion(*q),
        );
        self.cache = sample_quaternion.get_measurement();
        sample_quaternion
    }

    fn clone_and_clear(&mut self) -> AHRSInputSamples {
        let mut buffer_clone = AHRSInputSamples::new();

        for sensor_type in &self.sensor_cluster {
            let samples = self.buffer.get_samples_by_type(sensor_type).unwrap();
            buffer_clone.set_samples_by_type(sensor_type, samples);
        }
        buffer_clone.set_timestamp(self.buffer.get_timestamp());
        self.buffer.clear();

        buffer_clone
    }
}

#[derive(Clone)]
pub struct AHRSFilter {
    filter: Arc<Mutex<AHRSFilterManager>>,
    tag: String,
    publishers: PublisherManager<SensorReadings<SampleQuaternion>, SensorType>,
    new_measurement: SensorType,
}

impl AHRSFilter {
    pub fn new(
        tag: &str,
        sensor_cluster: Vec<SensorType>,
        new_measurement: SensorType,
        sampling_period_millis: f64,
    ) -> Result<Self, &'static str> {
        AHRSFilterManager::new(sensor_cluster, sampling_period_millis)
            .map(move |filter| Self {
                filter: Arc::new(Mutex::new(filter)),
                tag: tag.to_string(),
                publishers: PublisherManager::new(&[new_measurement.clone()]),
                new_measurement,
            })
            .map_err(|_| "Invalid sensor cluster")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::traits::IMUSample;
    use common::types::timed::Sample3D;
    use nalgebra::Vector3;
    use test_utils::csv_loader;
    use uuid::Uuid;

    #[tokio::test]
    async fn test_ahrs_filter() {
        let test_data = "../test-utils/test_data/sensor_readings.csv";
        let gyro_readings =
            csv_loader::load_csv_columns::<Sample3D>(test_data, &[0, 1, 2, 3]).unwrap();
        let accel_readings =
            csv_loader::load_csv_columns::<Sample3D>(test_data, &[0, 4, 5, 6]).unwrap();
        let mag_readings =
            csv_loader::load_csv_columns::<Sample3D>(test_data, &[0, 7, 8, 9]).unwrap();

        let acc_id = Uuid::new_v4();
        let gyro_id = Uuid::new_v4();
        let mag_id = Uuid::new_v4();
        let sensor_cluster = vec![
            SensorType::Accelerometer(acc_id),
            SensorType::Gyroscope(gyro_id),
            SensorType::Magnetometer(mag_id),
        ];
        let sampling_period_secs = 0.05;
        let mut ahrs_filter =
            AHRSFilterManager::new(sensor_cluster, sampling_period_secs * 1000.0).unwrap();
        let n_samples = accel_readings.len();

        let mut madgwick = Madgwick::new(0.05, MADGWICK_BETA);

        for i in 0..n_samples {
            let gyro = Vector3::from_vec(gyro_readings[i].get_measurement().into());
            let accel = Vector3::from_vec(accel_readings[i].get_measurement().into());
            let mag = Vector3::from_vec(mag_readings[i].get_measurement().into());
            let q_expected = *madgwick.update(&gyro, &accel, &mag).unwrap();

            ahrs_filter
                .buffer
                .set_samples_by_index(SensorIndex::Gyroscope.into(), gyro);
            ahrs_filter
                .buffer
                .set_samples_by_index(SensorIndex::Magnetometer.into(), mag);
            ahrs_filter
                .buffer
                .set_samples_by_index(SensorIndex::Accelerometer.into(), accel);

            let buffer_clone = ahrs_filter.clone_and_clear();
            let q_computed = ahrs_filter.update_filter(buffer_clone);

            assert_eq!(q_expected, q_computed.get_measurement().inner());
        }
    }
}
