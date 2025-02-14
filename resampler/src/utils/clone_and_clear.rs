use common::traits::{IMUReadings, IMUSample, IMUUntimedSample};
use common::types::sensors::SensorType;
use dashmap::DashMap;
use std::sync::Arc;
use tokio::sync::Mutex;

pub(crate) async fn clone_and_clear<T, S>(buffer: Arc<DashMap<SensorType, Mutex<T>>>) -> Vec<T>
where
    S: IMUSample,
    T: Send + Sync + IMUReadings<S> + 'static,
    S::Untimed: IMUUntimedSample,
{
    let sensor_types: Vec<SensorType> = buffer.iter().map(|entry| entry.key().clone()).collect();

    let mut buffer_clone: Vec<T> = Vec::new();
    for sensor_type in sensor_types {
        if let Some(mutex) = buffer.get(&sensor_type) {
            let mut data = mutex.lock().await;
            buffer_clone.push(data.clone());
            data.clear();
        }
    }
    buffer_clone
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::types::sensors::SensorReadings;
    use common::types::timed::Sample3D;
    use uuid::Uuid;

    #[tokio::test]
    async fn test_clone_and_clear() {
        let sensor_type = SensorType::Accelerometer(Uuid::new_v4());
        let sample = Sample3D::new(0.0, [1.0, 2.0, 3.0]);
        let mut readings = SensorReadings::new("Test", sensor_type.clone());
        let buffer: Arc<DashMap<SensorType, Mutex<SensorReadings<Sample3D>>>> =
            Arc::new(DashMap::new());

        readings.add_sample(sample.clone());
        buffer.insert(sensor_type.clone(), Mutex::new(readings));

        let result = clone_and_clear(buffer.clone()).await;

        // Check that buffer is cloned
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].get_samples().len(), 1);
        assert_eq!(
            result[0].get_samples()[0].get_measurement(),
            sample.get_measurement()
        );
        // check that buffer is cleared
        let buffer = buffer.get(&sensor_type).unwrap();
        let buffer = buffer.lock().await;
        assert!(buffer.is_empty());
    }

    #[tokio::test]
    async fn test_clone_and_clear_multiple_sensors() {
        let acc_id = Uuid::new_v4();
        let sensor_type1 = SensorType::Accelerometer(acc_id);
        let sensor_type2 = SensorType::Gyroscope(Uuid::new_v4());
        let mut readings_acc = SensorReadings::new("Test", sensor_type1.clone());
        let mut readings_gyro = SensorReadings::new("Test", sensor_type2.clone());
        let buffer: Arc<DashMap<SensorType, Mutex<SensorReadings<Sample3D>>>> =
            Arc::new(DashMap::new());
        let sample_acc = Sample3D::new(0.0, [1.0, 2.0, 3.0]);
        let sample_gyro = Sample3D::new(0.0, [5.0, 6.0, 7.0]);
        readings_acc.add_sample(sample_acc.clone());
        readings_gyro.add_sample(sample_gyro.clone());

        buffer.insert(sensor_type1.clone(), Mutex::new(readings_acc));
        buffer.insert(sensor_type2.clone(), Mutex::new(readings_gyro));

        let result = clone_and_clear(buffer.clone()).await;

        // Check that buffer is cloned
        assert_eq!(result.len(), 2);
        assert_eq!(result[0].get_samples().len(), 1);
        assert_eq!(result[1].get_samples().len(), 1);
        if result[0].get_sensor_type() == SensorType::Accelerometer(acc_id) {
            assert_eq!(
                result[0].get_samples()[0].get_measurement(),
                sample_acc.get_measurement()
            );
            assert_eq!(
                result[1].get_samples()[0].get_measurement(),
                sample_gyro.get_measurement()
            );
        } else {
            assert_eq!(
                result[1].get_samples()[0].get_measurement(),
                sample_acc.get_measurement()
            );
            assert_eq!(
                result[0].get_samples()[0].get_measurement(),
                sample_gyro.get_measurement()
            );
        }

        // check that buffer is cleared
        let readings = buffer.get(&sensor_type1).unwrap();
        let buffer_locked = readings.lock().await;
        assert!(buffer_locked.is_empty());
        drop(buffer_locked);

        let readings = buffer.get(&sensor_type2).unwrap();
        let buffer_locked = readings.lock().await;
        assert!(buffer_locked.is_empty());
    }

    #[tokio::test]
    async fn test_clone_and_clear_empty_buffer() {
        let buffer: Arc<DashMap<SensorType, Mutex<SensorReadings<Sample3D>>>> =
            Arc::new(DashMap::new());

        let result = clone_and_clear(buffer.clone()).await;
        assert_eq!(result.len(), 0);
    }
}
