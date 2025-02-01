use std::sync::Arc;

use common::traits::{IMUSample, IMUReadings};
use common::types::Sensor;

use crate::filter::{self, MovingAverage};

const N_SENSORS_IN_CLUSTER: usize = 3;

#[derive(Clone, Debug, Default, Copy)]
struct SensorCluster<T> {
    cluster: Vec<Box<dyn IMUReadings<T>>>,
}

impl<T: IMUSample> SensorCluster<T> {
    pub fn new(tag: &str) -> Self {
        Self {
            cluster: vec![Box::new(Sensor::new(tag)); N_SENSORS_IN_CLUSTER],
        }
    }

    fn add_measurement(&mut self, measurement: T, sensor_idx: usize) {
        self.cluster[sensor_idx].push(measurement);
    }

    pub(crate) fn create_measurement_buffer(
        sensor: usize,
        sensor_tag: &str,
        mut samples: Vec<T>,
        moving_average: &mut Option<MovingAverage<T>>,
    ) -> Sensor<T> {
        filter::filter_measurement(&mut samples, moving_average);
        let mut buffer = MeasurementBuffer::new(sensor, sensor_tag);
        for sample in samples {
            if let Some(measurement) = Measurement::from_vec(sample) {
                buffer.add_measurement(Arc::new(measurement));
            }
        }
        buffer
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_measurement_buffer_creation() {
        let buffer = MeasurementBuffer::new(GYROSCOPE, "Test");
        assert_eq!(buffer.sensor_type, GYROSCOPE);
        assert!(buffer.samples.is_empty());
    }

    #[test]
    fn test_measurement_buffer_add_measurement() {
        let mut buffer = MeasurementBuffer::new(MAGNETOMETER, "Test");
        let sample = Measurement::from_vec(vec![1.0, 2.0, 3.0, 4.0]).unwrap();
        buffer.add_measurement(Arc::new(sample.clone()));
        assert_eq!(buffer.samples.len(), 1);
        assert_eq!(
            buffer.samples[0].get_sample_data(),
            sample.get_sample_data()
        );
    }

    #[test]
    fn test_measurement_buffer_common_method() {
        let mut buffer = MeasurementBuffer::new(ACCELEROMETER, "Test");
        let sample1 = Measurement::from_vec(vec![1.0, 2.0, 3.0, 4.0]).unwrap();
        let sample2 = Measurement::from_vec(vec![5.0, 6.0, 7.0, 8.0]).unwrap();
        buffer.add_measurement(Arc::new(sample1.clone()));
        buffer.add_measurement(Arc::new(sample2.clone()));

        let common = buffer.get_samples();
        assert_eq!(common[0].get_sample_data(), sample1.get_sample_data());
        assert_eq!(common[1].get_sample_data(), sample2.get_sample_data());
    }

    #[test]
    fn test_measurement_buffer_create_producer() {
        let samples: Vec<Vec<f64>> = vec![vec![1.0, 2.0, 3.0, 4.0], vec![5.0, 6.0, 7.0, 8.0]];

        let expected: Vec<[f64; SAMPLE_N_COORDINATES + 1]> = samples
            .iter()
            .map(|sample| [sample[0], sample[1], sample[2], sample[3]])
            .collect();

        let mut filter = None;
        let buffer =
            MeasurementBuffer::create_measurement_buffer(GYROSCOPE, "Test", samples, &mut filter);

        assert_eq!(buffer.get_sensor_type(), GYROSCOPE);

        for (sample_idx, sample) in buffer.get_samples().iter().enumerate() {
            assert_eq!(sample.get_sample_data(), &expected[sample_idx][1..4]);
            assert_eq!(sample.get_timestamp(), expected[sample_idx][0]);
        }
    }
}
