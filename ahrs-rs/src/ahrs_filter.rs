use std::marker::PhantomData;
use std::sync::Arc;
use tokio::sync::Mutex;

use ahrs::{Ahrs, Madgwick};
use nalgebra::Vector3;
use uuid::Uuid;

use common::{IMUReadings, IMUSample, SampleQuaternion, SensorType};
use publisher::{Listener, Publishable, Publisher};

const MADGWICK_BETA: f64 = 0.1;

pub struct AHRSFilter<T, S>
where
    T: IMUReadings<S> + Send + Sync + 'static,
    S: IMUSample,
{
    ahrs_filter: Madgwick<f64>,
    publisher: Mutex<Publisher<T>>,
    buffer: Vec<Arc<Mutex<T>>>,
    _phantom_data: PhantomData<S>,
}

impl<T, S> AHRSFilter<T, S>
where
    T: IMUReadings<S> + Send + Sync + 'static,
    S: IMUSample,
{
    pub fn new(sampling_period_seconds: f64, n_buffer: usize) -> Self {
        Self {
            ahrs_filter: Madgwick::new(sampling_period_seconds, MADGWICK_BETA),
            buffer: (0..n_buffer)
                .map(|_| {
                    Arc::new(Mutex::new(T::from_vec(
                        "",
                        SensorType::Accelerometer,
                        vec![],
                    )))
                })
                .collect(),
            publisher: Mutex::new(Publisher::new()),
            _phantom_data: PhantomData,
        }
    }

    pub async fn update(&mut self) -> Result<Vec<SampleQuaternion>, Box<dyn std::error::Error>> {
        let mut sensor_readings: Vec<Arc<Vec<S>>> = Vec::with_capacity(3);
        for buffer in self.buffer.iter() {
            let buffer_clone = Arc::clone(buffer);
            let mut buffer_lock = buffer_clone.lock().await;
            let samples = buffer_lock.get_samples();
            sensor_readings.push(Arc::new(samples.clone()));
            buffer_lock.clear();
        }

        let sensors: Vec<(Vec<f64>, Vec<Vector3<f64>>)> = sensor_readings
            .iter()
            .map(|readings| {
                let mut measurements = Vec::new();
                let mut timestamps = Vec::new();
                for r in readings.iter() {
                    measurements.push(Vector3::from_vec(r.get_measurement()));
                    timestamps.push(r.get_timestamp());
                }
                (timestamps, measurements)
            })
            .collect();

        let n_samples = sensors[0].1.len();
        let mut sample_quaternion = Vec::new();
        for i in 0..n_samples {
            let gyro = &sensors[usize::from(SensorType::Gyroscope)].1[i];
            let accel = &sensors[usize::from(SensorType::Accelerometer)].1[i];
            let mag = &sensors[usize::from(SensorType::Magnetometer)].1[i];
            let q = self
                .ahrs_filter
                .update(gyro, accel, mag)
                .map_err(|_| Box::<dyn std::error::Error>::from("Conversion error"))?;
            sample_quaternion.push(SampleQuaternion::from_untimed(
                vec![q.i, q.j, q.k, q.w],
                sensors[0].0[i],
            ));
        }
        Ok(sample_quaternion)
    }

    pub fn register_sensor(&self, listener: Listener<T>) -> Uuid {
        let publisher = self.publisher.blocking_lock();
        publisher.register_listener(&listener)
    }

    pub fn unregister_sensor(&self, id: Uuid) {
        let publisher = self.publisher.blocking_lock();
        publisher.unregister_listener(id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{Sample3D, SensorReadings};
    use nalgebra::UnitQuaternion;
    use test_utils::csv_loader;

    #[tokio::test]
    async fn test1() {
        let test_data = "../test-utils/test_data/sensor_readings.csv";
        let gyro_readings =
            csv_loader::load_csv_columns::<Sample3D>(test_data, &vec![0, 1, 2, 3]).unwrap();
        let accel_readings =
            csv_loader::load_csv_columns::<Sample3D>(test_data, &vec![0, 4, 5, 6]).unwrap();
        let mag_readings =
            csv_loader::load_csv_columns::<Sample3D>(test_data, &vec![0, 7, 8, 9]).unwrap();
        let readings = vec![
            accel_readings.clone(),
            gyro_readings.clone(),
            mag_readings.clone(),
        ];

        let n_samples = accel_readings.len();

        let mut q_expected: Vec<UnitQuaternion<f64>> = Vec::new();
        let mut madgwick = Madgwick::new(0.05, MADGWICK_BETA);

        for i in 0..n_samples {
            let gyro = Vector3::from_vec(gyro_readings[i].get_measurement());
            let accel = Vector3::from_vec(accel_readings[i].get_measurement());
            let mag = Vector3::from_vec(mag_readings[i].get_measurement());
            q_expected.push(madgwick.update(&gyro, &accel, &mag).unwrap().clone());
        }

        let mut filter = AHRSFilter::<SensorReadings<Sample3D>, _>::new(0.05, 3);
        for (idx, buffer) in filter.buffer.iter().enumerate() {
            let mut buffer = buffer.lock().await;
            buffer.extend(readings[idx].clone());
        }

        let q_results = filter.update().await.unwrap();

        assert_eq!(q_results.len(), q_expected.len());
        for (q_r, q_e) in q_results.iter().zip(q_expected.iter()) {
            let q_r_values = q_r.get_measurement();
            assert!((q_r_values[0] - q_e.i).abs() < 1e-4);
            assert!((q_r_values[1] - q_e.j).abs() < 1e-4);
            assert!((q_r_values[2] - q_e.k).abs() < 1e-4);
            assert!((q_r_values[3] - q_e.w).abs() < 1e-4);
        }
    }
}
