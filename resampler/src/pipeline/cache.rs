use std::collections::HashMap;
use std::marker::PhantomData;

use common::traits::BasicArithmetic;
use common::traits::{IMUSample, IMUUntimedSample};
use common::types::buffers::CircularBuffer;
use common::types::sensors::SensorType;
use common::types::timed::SampleQuaternion;
use common::types::untimed::{Scalar, UnitQuaternion, XYZ};

trait Lerp: BasicArithmetic {}
impl Lerp for XYZ {}
impl Lerp for Scalar {}

pub trait Interpolable<T, U>
where
    T: IMUSample,
    T: IMUSample<Untimed = U>,
{
    fn interpolate_samples(&self, timestamp: f64) -> Vec<(SensorType, T)>;
}

#[derive(Default, Debug, Clone)]
pub struct Cache<T, U>
where
    T: IMUSample,
    T: IMUSample<Untimed = U>,
{
    cache: HashMap<SensorType, CircularBuffer<T>>,
    _phantom_data: PhantomData<U>,
}

impl<T, U> Cache<T, U>
where
    T: IMUSample,
    T: IMUSample<Untimed = U>,
{
    pub(crate) fn new(sensor_cluster: &[SensorType]) -> Self {
        let mut cache = HashMap::<SensorType, CircularBuffer<T>>::new();
        for sensor_type in sensor_cluster.iter() {
            cache.insert(sensor_type.clone(), CircularBuffer::new(2));
        }
        Self {
            cache,
            _phantom_data: PhantomData,
        }
    }
    pub(crate) fn push(&mut self, sensor_type: &SensorType, elem: T) {
        if let Some(buffer) = self.cache.get_mut(sensor_type) {
            buffer.push(elem);
        }
    }

    pub(crate) fn peek_newest(&self, sensor_type: &SensorType) -> Option<&T> {
        if let Some(buffer) = self.cache.get(sensor_type) {
            // peek at newest
            return Some(buffer.peek_back());
        }
        None
    }

    pub(crate) fn peek_newest_timestamp(&self, sensor_type: &SensorType) -> Option<f64> {
        if let Some(buffer) = self.cache.get(sensor_type) {
            // peek at newest
            return Some(buffer.peek_back().get_timestamp_secs());
        }
        None
    }
    pub(crate) fn peek_oldest(&self, sensor_type: &SensorType) -> Option<&T> {
        if let Some(buffer) = self.cache.get(sensor_type) {
            // peek at newest
            return Some(buffer.peek_front());
        }
        None
    }

    pub(crate) fn get_sensor_types(&self) -> Vec<&SensorType> {
        self.cache.keys().collect()
    }
}

impl<T, U> Interpolable<T, U> for Cache<T, U>
where
    U: IMUUntimedSample + Lerp + Default + Send + Sync + 'static + Clone,
    T: IMUSample<Untimed = U> + std::fmt::Debug,
{
    fn interpolate_samples(&self, timestamp_sec: f64) -> Vec<(SensorType, T)> {
        let sensor_types = self.get_sensor_types();
        let mut interpolated_samples: Vec<(SensorType, T)> = Vec::new();
        for sensor_type in sensor_types {
            let newest = self.peek_newest(sensor_type).unwrap();
            let newest_timestamp = newest.get_timestamp_secs();
            if newest_timestamp > timestamp_sec {
                // we can interpolate
                let oldest = self.peek_oldest(sensor_type).unwrap();
                let oldest_timestamp = oldest.get_timestamp_secs();
                let alpha =
                    (timestamp_sec - oldest_timestamp) / (newest_timestamp - oldest_timestamp);
                let newest_measurement = newest.get_measurement();
                let oldest_measurement = oldest.get_measurement();
                interpolated_samples.push((
                    sensor_type.clone(),
                    T::from_measurement(
                        timestamp_sec,
                        oldest_measurement * (1.0 - alpha) + newest_measurement * alpha,
                    ),
                ));
            } else {
                interpolated_samples.push((sensor_type.clone(), newest.clone()));
            }
        }
        interpolated_samples
    }
}

impl Interpolable<SampleQuaternion, UnitQuaternion> for Cache<SampleQuaternion, UnitQuaternion> {
    fn interpolate_samples(&self, timestamp_sec: f64) -> Vec<(SensorType, SampleQuaternion)> {
        let sensor_types = self.get_sensor_types();
        let mut interpolated_samples: Vec<(SensorType, SampleQuaternion)> = Vec::new();
        for sensor_type in sensor_types {
            let newest = self.peek_newest(sensor_type).unwrap();
            let newest_timestamp = newest.get_timestamp_secs();
            if newest_timestamp > timestamp_sec {
                // we can interpolate
                let oldest = self.peek_oldest(sensor_type).unwrap();
                let oldest_timestamp = oldest.get_timestamp_secs();
                let alpha =
                    (timestamp_sec - oldest_timestamp) / (newest_timestamp - oldest_timestamp);
                let newest_measurement = newest.get_measurement();
                let oldest_measurement = oldest.get_measurement();
                interpolated_samples.push((
                    sensor_type.clone(),
                    SampleQuaternion::from_measurement(
                        timestamp_sec,
                        UnitQuaternion::from_unit_quaternion(
                            oldest_measurement
                                .inner()
                                .slerp(&newest_measurement.inner(), alpha),
                        ),
                    ),
                ));
            } else {
                interpolated_samples.push((sensor_type.clone(), newest.clone()));
            }
        }
        interpolated_samples
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use common::types::timed::Sample3D;
    use uuid::Uuid;

    #[test]
    fn test_interpolator_new() {
        let sensor_types = vec![
            SensorType::Accelerometer(Uuid::new_v4()),
            SensorType::Gyroscope(Uuid::new_v4()),
        ];
        let interpolator: Cache<Sample3D, _> = Cache::new(&sensor_types);
        assert_eq!(interpolator.cache.len(), 2);
    }

    #[test]
    fn test_interpolate_empty_xyz() {
        let acc_id = Uuid::new_v4();
        let gyro_id = Uuid::new_v4();
        let sensor_types = vec![
            SensorType::Accelerometer(acc_id),
            SensorType::Gyroscope(gyro_id),
        ];
        let interpolator: Cache<Sample3D, _> = Cache::new(&sensor_types);
        interpolator.interpolate_samples(0.0);
    }

    #[test]
    fn test_interpolate_quat() {
        let acc_id = Uuid::new_v4();
        let gyro_id = Uuid::new_v4();
        let sensor_types = vec![
            SensorType::Accelerometer(acc_id),
            SensorType::Gyroscope(gyro_id),
        ];
        let interpolator: Cache<SampleQuaternion, _> = Cache::new(&sensor_types);
        interpolator.interpolate_samples(0.0);
    }
    /*
    #[test]
    fn test_interpolator_push() {
        let sensor_types = vec![SensorType::Accelerometer];
        let mut interpolator: Cache<MockIMUSample, MockIMUUntimedSample> =
            Cache::new(&sensor_types);
        let sample = MockIMUSample { timestamp: 1.0 };
        interpolator.push(&SensorType::Accelerometer, sample.clone());
        assert_eq!(
            interpolator
                .peek_newest(&SensorType::Accelerometer)
                .unwrap()
                .get_timestamp_secs(),
            1.0
        );
    }

    #[test]
    fn test_interpolator_peek_newest() {
        let sensor_types = vec![SensorType::Accelerometer];
        let mut interpolator: Cache<MockIMUSample, MockIMUUntimedSample> =
            Cache::new(&sensor_types);
        let sample = MockIMUSample { timestamp: 1.0 };
        interpolator.push(&SensorType::Accelerometer, sample.clone());
        let newest = interpolator
            .peek_newest(&SensorType::Accelerometer)
            .unwrap();
        assert_eq!(newest.get_timestamp_secs(), 1.0);
    }

    #[test]
    fn test_interpolator_peek_oldest() {
        let sensor_types = vec![SensorType::Accelerometer];
        let mut interpolator: Cache<MockIMUSample, MockIMUUntimedSample> =
            Cache::new(&sensor_types);
        let sample1 = MockIMUSample { timestamp: 1.0 };
        let sample2 = MockIMUSample { timestamp: 2.0 };
        interpolator.push(&SensorType::Accelerometer, sample1.clone());
        interpolator.push(&SensorType::Accelerometer, sample2.clone());
        let oldest = interpolator
            .peek_oldest(&SensorType::Accelerometer)
            .unwrap();
        assert_eq!(oldest.get_timestamp_secs(), 1.0);
    }

    #[test]
    fn test_interpolator_peek_newest_timestamp() {
        let sensor_types = vec![SensorType::Accelerometer];
        let mut interpolator: Cache<MockIMUSample, MockIMUUntimedSample> =
            Cache::new(&sensor_types);
        let sample = MockIMUSample { timestamp: 1.0 };
        interpolator.push(&SensorType::Accelerometer, sample.clone());
        let newest_timestamp = interpolator
            .peek_newest_timestamp(&SensorType::Accelerometer)
            .unwrap();
        assert_eq!(newest_timestamp, 1.0);
    }

    #[test]
    fn test_interpolator_peek_oldest_timestamp() {
        let sensor_types = vec![SensorType::Accelerometer];
        let mut interpolator: Cache<MockIMUSample, MockIMUUntimedSample> =
            Cache::new(&sensor_types);
        let sample1 = MockIMUSample { timestamp: 1.0 };
        let sample2 = MockIMUSample { timestamp: 2.0 };
        interpolator.push(&SensorType::Accelerometer, sample1.clone());
        interpolator.push(&SensorType::Accelerometer, sample2.clone());
        let oldest_timestamp = interpolator
            .peek_oldest_timestamp(&SensorType::Accelerometer)
            .unwrap();
        assert_eq!(oldest_timestamp, 1.0);
    }
    */
}
