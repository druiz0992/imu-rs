use super::{SensorTag, SensorType};
use crate::traits::{IMUReadings, IMUSample};

const DEFAULT_SENSOR_BUFFER_CAPACITY: usize = 64;

#[derive(Clone, Debug)]
pub struct SensorReadings<T> {
    buffer: Vec<T>,
    tag: SensorTag,
    sensor_type: SensorType,
}

impl<T: IMUSample> SensorReadings<T> {
    pub fn new(tag: &str, sensor_type: SensorType) -> Self {
        Self {
            tag: SensorTag::new(tag),
            buffer: Vec::with_capacity(DEFAULT_SENSOR_BUFFER_CAPACITY),
            sensor_type,
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

    pub fn get_sensor_type(&self) -> SensorType {
        self.sensor_type.clone()
    }
}

impl<T: IMUSample> IMUReadings<T> for SensorReadings<T> {
    fn get_samples(&self) -> Vec<T> {
        self.buffer.clone()
    }
    fn get_sensor_tag(&self) -> &str {
        self.tag.inner()
    }
    fn get_sensor_type(&self) -> SensorType {
        self.sensor_type.clone()
    }

    fn extend(&mut self, elems: Vec<T>) {
        self.buffer.extend(elems);
    }

    fn clear(&mut self) {
        self.buffer.clear();
    }

    fn from_vec(tag: &str, sensor_type: SensorType, data: Vec<T>) -> Self {
        Self {
            tag: SensorTag::new(tag),
            sensor_type,
            buffer: data,
        }
    }
}

#[cfg(test)]
mod tests {
    use uuid::Uuid;

    use super::*;
    use crate::types::Sample3D;

    #[test]
    fn test_sensor_tag_new() {
        let tag = SensorTag::new("test_tag");
        assert_eq!(tag.inner(), "test_tag");
    }

    #[test]
    fn test_sensor_new() {
        let sensor =
            SensorReadings::<Sample3D>::new("test_sensor", SensorType::Gyroscope(Uuid::new_v4()));
        assert_eq!(sensor.get_sensor_tag(), "test_sensor");
    }

    #[test]
    fn test_sensor_is_empty() {
        let sensor = SensorReadings::<Sample3D>::new(
            "test_sensor",
            SensorType::Other(Uuid::new_v4(), "wer".to_string()),
        );
        assert!(sensor.is_empty());
    }

    #[test]
    fn test_sensor_add_sample() {
        let mut sensor = SensorReadings::<Sample3D>::new(
            "test_sensor",
            SensorType::Accelerometer(Uuid::new_v4()),
        );
        let sample = Sample3D::default();
        sensor.add_sample(sample.clone());
        assert_eq!(sensor.len(), 1);
        assert_eq!(sensor.get_samples()[0], sample);
    }
}
