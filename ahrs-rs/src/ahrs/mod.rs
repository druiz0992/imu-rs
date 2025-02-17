pub(crate) mod buffer;
pub(crate) mod sink;
pub(crate) mod source;

use std::sync::Arc;
use tokio::sync::Mutex;

use ahrs::{Ahrs, Madgwick};

use crate::utils;
use buffer::{AHRSInputSamples, SensorIndex, N_SENSORS};
use common::types::sensors::{SensorReadings, SensorType};
use common::types::timed::SampleQuaternion;
use common::types::untimed::UnitQuaternion;
use publisher::PublisherManager;

const MADGWICK_BETA: f64 = 0.1;

#[derive(Clone)]
pub struct AHRSFilter {
    ahrs_filter: Arc<Mutex<Madgwick<f64>>>,
    buffer: Arc<Mutex<AHRSInputSamples>>,
    publishers: PublisherManager<SensorReadings<SampleQuaternion>, SensorType>,
    sensor_cluster: [SensorType; N_SENSORS],
    tag: String,
}

impl AHRSFilter {
    pub fn new(
        tag: &str,
        sensor_cluster: [SensorType; N_SENSORS],
        sampling_period_secs: f64,
    ) -> Result<Self, &'static str> {
        if !utils::check_sensor_cluster(&sensor_cluster) {
            return Err("Invalid sensor cluster");
        }

        let buffer = Mutex::new(AHRSInputSamples::new());
        Ok(Self {
            ahrs_filter: Arc::new(Mutex::new(Madgwick::new(
                sampling_period_secs,
                MADGWICK_BETA,
            ))),
            buffer: Arc::new(buffer),
            publishers: PublisherManager::new(&sensor_cluster),
            tag: tag.to_string(),
            sensor_cluster,
        })
    }

    pub async fn update_filter(&self) -> Result<SampleQuaternion, &'static str> {
        let buffer_clone = utils::clone_and_clear(self.buffer.clone(), &self.sensor_cluster).await;

        let gyro = buffer_clone
            .get_samples_by_index(usize::from(SensorIndex::Gyroscope))
            .unwrap();
        let accel = buffer_clone
            .get_samples_by_index(usize::from(SensorIndex::Accelerometer))
            .unwrap();
        let mag = buffer_clone
            .get_samples_by_index(usize::from(SensorIndex::Magnetometer))
            .unwrap();
        let mut filter = self.ahrs_filter.lock().await;
        let q = filter
            .update(&gyro, &accel, &mag)
            .map_err(|_| "Conversion error")?;

        let sample_quaternion = SampleQuaternion::from_unit_quaternion(
            buffer_clone.get_timestamp(),
            UnitQuaternion::from_unit_quaternion(*q),
        );
        Ok(sample_quaternion)
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
        let sensor_cluster = [
            SensorType::Accelerometer(acc_id),
            SensorType::Gyroscope(gyro_id),
            SensorType::Magnetometer(mag_id),
        ];
        let sampling_period_secs = 0.05;
        let ahr_filter = AHRSFilter::new("Test", sensor_cluster, sampling_period_secs).unwrap();
        let n_samples = accel_readings.len();

        let mut madgwick = Madgwick::new(0.05, MADGWICK_BETA);

        for i in 0..n_samples {
            let gyro = Vector3::from_vec(gyro_readings[i].get_measurement().into());
            let accel = Vector3::from_vec(accel_readings[i].get_measurement().into());
            let mag = Vector3::from_vec(mag_readings[i].get_measurement().into());
            let q_expected = *madgwick.update(&gyro, &accel, &mag).unwrap();

            let mut input_buffer = ahr_filter.buffer.lock().await;

            input_buffer.set_samples_by_index(SensorIndex::Gyroscope.into(), gyro);
            input_buffer.set_samples_by_index(SensorIndex::Magnetometer.into(), mag);
            input_buffer.set_samples_by_index(SensorIndex::Accelerometer.into(), accel);
            drop(input_buffer);

            let q_computed = ahr_filter.update_filter().await.unwrap();

            assert_eq!(q_expected, q_computed.get_measurement().inner());
        }
    }
}
