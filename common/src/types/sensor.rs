use crate::{IMUReadings, IMUSample};

const DEFAULT_SENSOR_BUFFER_CAPACITY: usize = 64;

#[derive(Clone, Debug, PartialEq, PartialOrd, Hash)]
pub enum SensorType {
    Accelerometer,
    Gyroscope,
    Magnetometer,
}

impl From<SensorType> for usize {
    fn from(value: SensorType) -> Self {
        match value {
            SensorType::Accelerometer => 0,
            SensorType::Gyroscope => 1,
            SensorType::Magnetometer => 2,
        }
    }
}

impl TryFrom<&str> for SensorType {
    type Error = String;
    fn try_from(value: &str) -> Result<Self, Self::Error> {
        match value {
            "Accelerometer" => Ok(Self::Accelerometer),
            "Gyroscope" => Ok(Self::Gyroscope),
            "Magnetometer" => Ok(Self::Magnetometer),
            _ => Err(format!("Unknown sensor type {}", value)),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd, Hash)]
pub struct SensorTag(String);

impl SensorTag {
    pub fn new(tag: &str) -> Self {
        Self(tag.to_string())
    }

    pub fn inner(&self) -> &str {
        self.0.as_str()
    }
}

#[derive(Clone, Debug)]
pub struct SensorReadings<T> {
    buffer: Vec<T>,
    tag: SensorTag,
}

impl<T: IMUSample> SensorReadings<T> {
    pub fn new(tag: &str) -> Self {
        Self {
            tag: SensorTag::new(tag),
            buffer: Vec::with_capacity(DEFAULT_SENSOR_BUFFER_CAPACITY),
        }
    }

    pub fn from_vec(tag: &str, data: Vec<T>) -> Self {
        Self {
            tag: SensorTag::new(tag),
            buffer: data,
        }
    }

    pub fn add_sample(&mut self, elem: T) {
        self.buffer.push(elem);
    }

    pub fn len(&self) -> usize {
        self.buffer.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<T: IMUSample> IMUReadings<T> for SensorReadings<T> {
    fn get_samples_ref(&self) -> &[T] {
        &self.buffer
    }
    fn get_samples(&self) -> Vec<T> {
        self.buffer.clone()
    }
    fn get_sensor_tag(&self) -> &str {
        self.tag.inner()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Sample3D;
    use crate::IMUSample;

    #[test]
    fn test_sensor_tag_new() {
        let tag = SensorTag::new("test_tag");
        assert_eq!(tag.inner(), "test_tag");
    }

    #[test]
    fn test_sensor_new() {
        let sensor = SensorReadings::<Sample3D>::new("test_sensor");
        assert_eq!(sensor.get_sensor_tag(), "test_sensor");
    }

    #[test]
    fn test_sensor_is_empty() {
        let sensor = SensorReadings::<Sample3D>::new("test_sensor");
        assert!(sensor.is_empty());
    }

    #[test]
    fn test_sensor_add_sample() {
        let mut sensor = SensorReadings::<Sample3D>::new("test_sensor");
        let sample = Sample3D::default();
        sensor.add_sample(sample.clone());
        assert_eq!(sensor.len(), 1);
        assert_eq!(sensor.get_samples_ref()[0], sample);
    }

    #[test]
    fn test_iter_samples() {
        let mut sensor = SensorReadings::<Sample3D>::new("IMU1");
        sensor.add_sample(Sample3D::new(1.0, [1.0, 2.0, 3.0]));
        sensor.add_sample(Sample3D::new(2.0, [4.0, 5.0, 6.0]));
        sensor.add_sample(Sample3D::new(3.0, [7.0, 8.0, 8.0]));

        let timestamps: Vec<f64> = sensor.iter_samples().map(|s| s.get_timestamp()).collect();

        assert_eq!(timestamps, vec![1.0, 2.0, 3.0]);
        assert_eq!(sensor.len(), 3);
    }

    #[test]
    fn test_into_iter_samples() {
        let mut sensor = SensorReadings::new("IMU1");
        sensor.add_sample(Sample3D::new(1.0, [1.0, 2.0, 3.0]));
        sensor.add_sample(Sample3D::new(2.0, [4.0, 5.0, 6.0]));
        sensor.add_sample(Sample3D::new(3.0, [7.0, 8.0, 8.0]));

        let timestamps: Vec<f64> = sensor
            .into_iter_samples()
            .map(|s| s.get_timestamp())
            .collect();

        assert_eq!(timestamps, vec![1.0, 2.0, 3.0]);
    }
}
