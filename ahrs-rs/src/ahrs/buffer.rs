use nalgebra::Vector3;

use crate::utils;
use common::types::sensors::SensorType;

pub(crate) const N_SENSORS: usize = 3;
pub(crate) enum SensorIndex {
    Gyroscope = 0,
    Accelerometer = 1,
    Magnetometer = 2,
}

impl TryFrom<usize> for SensorIndex {
    type Error = &'static str;

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(SensorIndex::Gyroscope),
            1 => Ok(SensorIndex::Accelerometer),
            2 => Ok(SensorIndex::Magnetometer),
            _ => Err("Invalid sensor index"),
        }
    }
}

impl TryFrom<SensorType> for SensorIndex {
    type Error = &'static str;

    fn try_from(value: SensorType) -> Result<Self, Self::Error> {
        match value {
            SensorType::Accelerometer(_) => Ok(SensorIndex::Accelerometer),
            SensorType::Gyroscope(_) => Ok(SensorIndex::Gyroscope),
            SensorType::Magnetometer(_) => Ok(SensorIndex::Magnetometer),
            _ => Err("Invalid sensor index"),
        }
    }
}

impl From<SensorIndex> for usize {
    fn from(value: SensorIndex) -> Self {
        value as usize
    }
}

#[derive(Clone, Default, Debug)]
pub struct AHRSInputSamples {
    timestamp: f64,
    samples: [Vector3<f64>; N_SENSORS],
    ready: [bool; N_SENSORS],
}

impl AHRSInputSamples {
    pub(crate) fn new() -> Self {
        Self {
            samples: [Vector3::default(); N_SENSORS],
            timestamp: 0.0,
            ready: [false; N_SENSORS],
        }
    }

    pub(crate) fn set_samples_by_type(&mut self, sensor_type: &SensorType, samples: Vector3<f64>) {
        if let Some(sensor_idx) = utils::get_sensor_index(sensor_type) {
            self.samples[sensor_idx] = samples;
            self.ready[sensor_idx] = true;
        }
    }

    pub(crate) fn samples_ready(&self) -> bool {
        self.ready.iter().all(|&r| r)
    }

    #[allow(dead_code)]
    pub(crate) fn set_samples_by_index(&mut self, sensor_index: usize, samples: Vector3<f64>) {
        if sensor_index < N_SENSORS {
            self.samples[sensor_index] = samples;
        }
    }

    pub(crate) fn get_samples_by_type(&self, sensor_type: &SensorType) -> Option<Vector3<f64>> {
        if let Some(sensor_idx) = utils::get_sensor_index(sensor_type) {
            return Some(self.samples[sensor_idx]);
        }
        None
    }
    pub(crate) fn get_samples_by_index(&self, sensor_index: usize) -> Option<Vector3<f64>> {
        if sensor_index < N_SENSORS {
            return Some(self.samples[sensor_index]);
        }
        None
    }

    pub(crate) fn get_timestamp(&self) -> f64 {
        self.timestamp
    }

    pub(crate) fn clear(&mut self) {
        self.ready.iter_mut().for_each(|flag| *flag = false);
    }

    pub(crate) fn set_timestamp(&mut self, timestamp: f64) {
        self.timestamp = timestamp;
    }
}

#[cfg(test)]
mod tests {
    use uuid::Uuid;

    use super::*;

    #[test]
    fn test_new() {
        let samples = AHRSInputSamples::new();
        assert_eq!(samples.timestamp, 0.0);
        assert_eq!(samples.ready, [false; N_SENSORS]);
        for sample in &samples.samples {
            assert_eq!(*sample, Vector3::default());
        }
    }

    #[test]
    fn test_set_samples_by_type() {
        let mut samples = AHRSInputSamples::new();
        let sensor_type = SensorType::Accelerometer(Uuid::new_v4());
        let sample_data = Vector3::new(1.0, 2.0, 3.0);
        samples.set_samples_by_type(&sensor_type, sample_data);
        assert_eq!(samples.get_samples_by_type(&sensor_type), Some(sample_data));
        assert!(samples.ready[utils::get_sensor_index(&sensor_type).unwrap()]);
    }

    #[test]
    fn test_samples_ready() {
        let mut samples = AHRSInputSamples::new();
        assert!(!samples.samples_ready());
        for i in 0..N_SENSORS {
            samples.ready[i] = true;
        }
        assert!(samples.samples_ready());
    }

    #[test]
    fn test_set_samples_by_index() {
        let mut samples = AHRSInputSamples::new();
        let sample_data = Vector3::new(1.0, 2.0, 3.0);
        samples.set_samples_by_index(1, sample_data);
        assert_eq!(samples.get_samples_by_index(1), Some(sample_data));
    }

    #[test]
    fn test_get_samples_by_type() {
        let mut samples = AHRSInputSamples::new();
        let sensor_type = SensorType::Gyroscope(Uuid::new_v4());
        let sample_data = Vector3::new(4.0, 5.0, 6.0);
        samples.set_samples_by_type(&sensor_type, sample_data);
        assert_eq!(samples.get_samples_by_type(&sensor_type), Some(sample_data));
    }

    #[test]
    fn test_get_samples_by_index() {
        let mut samples = AHRSInputSamples::new();
        let sample_data = Vector3::new(7.0, 8.0, 9.0);
        samples.set_samples_by_index(2, sample_data);
        assert_eq!(samples.get_samples_by_index(2), Some(sample_data));
    }

    #[test]
    fn test_get_timestamp() {
        let samples = AHRSInputSamples::new();
        assert_eq!(samples.get_timestamp(), 0.0);
    }

    #[test]
    fn test_clear() {
        let mut samples = AHRSInputSamples::new();
        for i in 0..N_SENSORS {
            samples.ready[i] = true;
        }
        samples.clear();
        assert_eq!(samples.ready, [false; N_SENSORS]);
    }

    #[test]
    fn test_set_timestamp() {
        let mut samples = AHRSInputSamples::new();
        samples.set_timestamp(123.456);
        assert_eq!(samples.get_timestamp(), 123.456);
    }
}
