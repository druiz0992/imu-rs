use super::{SensorTag, SensorType};
use crate::{IMUReadings, IMUSample};

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
    fn get_sensor_type(&self) -> &SensorType {
        &self.sensor_type
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
        let sensor = SensorReadings::<Sample3D>::new("test_sensor", SensorType::Gyroscope);
        assert_eq!(sensor.get_sensor_tag(), "test_sensor");
    }

    #[test]
    fn test_sensor_is_empty() {
        let sensor =
            SensorReadings::<Sample3D>::new("test_sensor", SensorType::Other("wer".to_string()));
        assert!(sensor.is_empty());
    }

    #[test]
    fn test_sensor_add_sample() {
        let mut sensor = SensorReadings::<Sample3D>::new("test_sensor", SensorType::Accelerometer);
        let sample = Sample3D::default();
        sensor.add_sample(sample.clone());
        assert_eq!(sensor.len(), 1);
        assert_eq!(sensor.get_samples_ref()[0], sample);
    }

    #[test]
    fn test_iter_samples() {
        let mut sensor = SensorReadings::<Sample3D>::new("IMU1", SensorType::Magnetometer);
        sensor.add_sample(Sample3D::new(1.0, [1.0, 2.0, 3.0]));
        sensor.add_sample(Sample3D::new(2.0, [4.0, 5.0, 6.0]));
        sensor.add_sample(Sample3D::new(3.0, [7.0, 8.0, 8.0]));

        let timestamps: Vec<f64> = sensor.iter_samples().map(|s| s.get_timestamp()).collect();

        assert_eq!(timestamps, vec![1.0, 2.0, 3.0]);
        assert_eq!(sensor.len(), 3);
    }

    #[test]
    fn test_into_iter_samples() {
        let mut sensor = SensorReadings::new("IMU1", SensorType::Accelerometer);
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
