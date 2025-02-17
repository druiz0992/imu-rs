use std::sync::Arc;
use tokio::sync::Mutex;

use common::types::sensors::SensorType;

use crate::{ahrs::buffer::SensorIndex, ahrs::buffer::N_SENSORS, AHRSInputSamples};

pub(crate) fn check_sensor_cluster(sensor_cluster: &[SensorType; 3]) -> bool {
    let mut has_accelerometer = false;
    let mut has_gyro = false;
    let mut has_magnetometer = false;
    for sensor_type in sensor_cluster {
        match sensor_type {
            SensorType::Accelerometer(_) => has_accelerometer = true,
            SensorType::Gyroscope(_) => has_gyro = true,
            SensorType::Magnetometer(_) => has_magnetometer = true,
            _ => {
                return false;
            }
        }
    }
    has_accelerometer && has_gyro && has_magnetometer
}
pub(crate) fn get_sensor_index(sensor_type: &SensorType) -> Option<usize> {
    match sensor_type {
        SensorType::Accelerometer(_) => Some(usize::from(SensorIndex::Accelerometer)),
        SensorType::Gyroscope(_) => Some(usize::from(SensorIndex::Gyroscope)),
        SensorType::Magnetometer(_) => Some(usize::from(SensorIndex::Magnetometer)),
        _ => None,
    }
}

pub(crate) async fn clone_and_clear(
    buffer: Arc<Mutex<AHRSInputSamples>>,
    sensor_cluster: &[SensorType; N_SENSORS],
) -> AHRSInputSamples {
    let mut buffer_lock = buffer.lock().await;
    let mut buffer_clone = AHRSInputSamples::new();

    for sensor_type in sensor_cluster {
        let samples = buffer_lock.get_samples_by_type(sensor_type).unwrap();
        buffer_clone.set_samples_by_type(sensor_type, samples);
    }
    buffer_clone.set_timestamp(buffer_lock.get_timestamp());
    buffer_lock.clear();

    buffer_clone
}
