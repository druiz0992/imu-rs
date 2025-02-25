use common::types::sensors::SensorType;

use crate::{ahrs::buffer::SensorIndex, ahrs::buffer::N_SENSORS};

pub(crate) fn check_sensor_cluster(sensor_cluster: &[SensorType; N_SENSORS]) -> bool {
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
