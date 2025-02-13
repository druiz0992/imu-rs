use async_trait::async_trait;
use rand::{rngs::StdRng, SeedableRng};
use std::sync::Arc;
use std::time::Duration;
use std::time::{SystemTime, UNIX_EPOCH};
use tokio::sync::{Mutex, Notify};

use super::gaussian::GaussianNoise;
use super::timestamp::Timestamp;
use crate::constants::N_SENSORS;
use crate::models::errors::PhyphoxError;
use crate::ports::PhyphoxPort;
use common::traits::{IMUReadings, IMUSample};
use common::types::buffers::CircularReader;
use common::types::sensors::sensor_type;
use common::types::sensors::{SensorReadings, SensorType};
use common::types::timed::Sample3D;
use common::types::untimed::XYZ;
use publisher::{Publishable, Publisher};
use test_utils::csv_loader::{self, CsvColumnMapper};

const GAUSSIAN_TIME_MEAN: f64 = 0f64;
const GAUSSIAN_TIME_STDEV: f64 = 0.005;
const GAUSSIAN_SENSOR_MEAN: f64 = 0f64;
const GAUSSIAN_SENSOR_STDEV: f64 = 0.5;
const LSB_TIME_MASK: u8 = 0xC0;
const MAX_N_SAMPLES: u8 = 15;

/// Configures mock data acquisition
pub struct PhyphoxMock {
    readings: Mutex<[CircularReader<Sample3D>; N_SENSORS]>,
    timestamps: Mutex<Timestamp>,
    capture_sampling_period_secs: f64,
    time_delta: GaussianNoise,
    sensor_noise: Option<GaussianNoise>,
    sensor_cluster_tag: String,
    sensor_cluster: Vec<SensorType>,
}

impl PhyphoxMock {
    /// Creates a new `Phyphox` instance with the specified configuration.
    /// Returns an ClientBuild error if Http client to connect to Phyphox API cannot be created
    pub(crate) fn new(
        sensor_cluster_tag: &str,
        sensor_cluster: Vec<SensorType>,
        capture_sampling_period_millis: u64,
        add_sensor_noise: bool,
    ) -> Result<Self, PhyphoxError> {
        let test_data = "../test-utils/test_data/sensor_readings.csv";
        let mut gyro_mapper = CsvColumnMapper::new();
        gyro_mapper.add_timestamp().add_gyro();
        let gyro_readings = CircularReader::try_from(
            csv_loader::load_csv_columns::<Sample3D>(test_data, &gyro_mapper.columns()).unwrap(),
        )
        .map_err(PhyphoxError::Other)?;

        let mut accel_mapper = CsvColumnMapper::new();
        accel_mapper.add_timestamp().add_accel();
        let accel_readings = CircularReader::try_from(
            csv_loader::load_csv_columns::<Sample3D>(test_data, &accel_mapper.columns()).unwrap(),
        )
        .map_err(PhyphoxError::Other)?;

        let mut mag_mapper = CsvColumnMapper::new();
        mag_mapper.add_timestamp().add_accel();
        let mag_readings = CircularReader::try_from(
            csv_loader::load_csv_columns::<Sample3D>(test_data, &mag_mapper.columns()).unwrap(),
        )
        .map_err(PhyphoxError::Other)?;

        let readings = Mutex::new([
            accel_readings.clone(),
            gyro_readings.clone(),
            mag_readings.clone(),
        ]);
        Ok(Self {
            sensor_cluster_tag: sensor_cluster_tag.to_string(),
            readings,
            timestamps: Mutex::new(Timestamp::new()),
            capture_sampling_period_secs: capture_sampling_period_millis as f64 / 1000.0,
            time_delta: GaussianNoise::new(GAUSSIAN_TIME_MEAN, GAUSSIAN_TIME_STDEV),
            sensor_noise: add_sensor_noise
                .then(|| GaussianNoise::new(GAUSSIAN_SENSOR_MEAN, GAUSSIAN_SENSOR_STDEV)),
            sensor_cluster,
        })
    }

    async fn get_next_samples(&self, buffer_idx: usize) -> Vec<Sample3D> {
        let mut new_samples = Vec::new();
        let pending_samples = select_random_pending_samples();
        let mut rng = StdRng::from_entropy();
        if pending_samples > 0 {
            let mut readings = self.readings.lock().await;
            let next_sample = readings[buffer_idx].next_element();
            let mut timestamps = self.timestamps.lock().await;
            for _ in 0..pending_samples {
                let sample_timestamp = self
                    .time_delta
                    .add_noise(&mut rng, timestamps.get_reading_timestamp(buffer_idx))
                    .abs();
                if sample_timestamp < timestamps.get_current_timestamp() {
                    timestamps.set_reading_timestamp(buffer_idx, sample_timestamp);
                    let mut next_measurement: Vec<f64> = next_sample.get_measurement().into();

                    if self.sensor_noise.is_some() {
                        let rgen = self.sensor_noise.as_ref().unwrap();
                        next_measurement = rgen.add_noise_vec(&mut rng, next_measurement);
                    }
                    let next_measurement: XYZ = XYZ::try_from(next_measurement).unwrap();
                    new_samples.push(Sample3D::from_measurement(
                        sample_timestamp,
                        next_measurement,
                    ))
                } else {
                    break;
                }
            }
        }
        new_samples
    }
}

fn select_random_pending_samples() -> usize {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_micros();
    let lsb: u8 = now.to_le_bytes()[0];
    if lsb & LSB_TIME_MASK == 0 {
        0
    } else {
        (lsb % MAX_N_SAMPLES) as usize
    }
}

#[async_trait]
impl PhyphoxPort for PhyphoxMock {
    /// Starts the data acquisition process. The process is stopped with a SIGINT signal
    /// Returns FetchData error if it can't connect to REST API.
    async fn start(
        &self,
        period_millis: Duration,
        abort_signal: Option<Arc<Notify>>,
        _window_size: Option<usize>,
        publisher: Option<Vec<Publisher<SensorReadings<Sample3D>>>>,
    ) -> Result<(), PhyphoxError> {
        let abort_signal = abort_signal.unwrap_or(Arc::new(Notify::new()));
        loop {
            tokio::select! {
                    _ = abort_signal.notified() => {
                        break;
                    }
                    _ = tokio::time::sleep(period_millis) => {
                {
                    let mut timestamp = self.timestamps.lock().await;
                    timestamp.incr_current_timestamp(self.capture_sampling_period_secs);
                }

                for sensor in &self.sensor_cluster {
                    let sensor_idx = match usize::from(sensor) {
                        sensor_type::ACCELEROMETER_OFFSET..sensor_type::GYROSCOPE_OFFSET => 0,
                        sensor_type::GYROSCOPE_OFFSET..sensor_type::MAGNETOMETER_OFFSET => 1,
                        sensor_type::MAGNETOMETER_OFFSET..sensor_type::MAX_OFFSET => 2,
                        _ => 2,
                    };
                    let samples = self.get_next_samples(sensor_idx).await;
                    let buffer = SensorReadings::from_vec(&self.sensor_cluster_tag, sensor.clone(), samples);
                    if let Some(publisher) = publisher.as_ref() {
                        publisher[sensor_idx].notify_listeners(Arc::new(buffer)).await;
                    };
                }
            }
            }
        }

        Ok(())
    }

    fn get_tag(&self) -> &str {
        self.sensor_cluster_tag.as_str()
    }

    async fn get_available_sensors(&self) -> Result<Vec<SensorType>, String> {
        Ok(self.sensor_cluster.clone())
    }
    fn get_sensor_cluster(&self) -> Vec<SensorType> {
        self.sensor_cluster.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use uuid::Uuid;

    #[tokio::test]
    async fn test_phyphox_mock_new() {
        let sensor_cluster = vec![
            SensorType::Accelerometer(Uuid::new_v4()),
            SensorType::Gyroscope(Uuid::new_v4()),
            SensorType::Magnetometer(Uuid::new_v4()),
        ];
        let phyphox_mock = PhyphoxMock::new("Test", sensor_cluster, 100, false);
        assert!(phyphox_mock.is_ok());
    }

    #[tokio::test]
    async fn test_phyphox_mock_start_stop() {
        let sensor_cluster = vec![
            SensorType::Accelerometer(Uuid::new_v4()),
            SensorType::Gyroscope(Uuid::new_v4()),
            SensorType::Magnetometer(Uuid::new_v4()),
        ];
        let phyphox_mock = Arc::new(PhyphoxMock::new("Test", sensor_cluster, 100, false).unwrap());
        let period = Duration::from_millis(100);

        let phyphox_mock_clone = Arc::clone(&phyphox_mock);
        let abort_signal = Arc::new(Notify::new());

        let shutdown_signal = abort_signal.clone();

        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_secs(2)).await;
            shutdown_signal.notify_waiters();
        });

        let start_handle = tokio::spawn(async move {
            phyphox_mock_clone
                .start(period, Some(abort_signal), None, None)
                .await
                .unwrap();
        });

        start_handle.await.unwrap();
    }

    #[tokio::test]
    async fn test_pending_samples() {
        let mut greater_than_zero = 0;
        for _ in 0..100 {
            let pending_samples = select_random_pending_samples();
            if pending_samples > 0 {
                greater_than_zero += 1;
            }
            tokio::time::sleep(Duration::from_nanos(350)).await;
            assert!(pending_samples < MAX_N_SAMPLES as usize);
        }
        assert!(greater_than_zero > 0);
    }

    #[tokio::test]
    async fn test_get_next_samples() {
        let sensor_cluster = vec![
            SensorType::Accelerometer(Uuid::new_v4()),
            SensorType::Gyroscope(Uuid::new_v4()),
            SensorType::Magnetometer(Uuid::new_v4()),
        ];
        let phyphox_mock = PhyphoxMock::new("Test", sensor_cluster, 100, true).unwrap();
        {
            let mut timestamp = phyphox_mock.timestamps.lock().await;
            timestamp.incr_current_timestamp(0.1);
        }
        for _ in 0..100 {
            let samples = phyphox_mock.get_next_samples(0).await;
            tokio::time::sleep(Duration::from_nanos(350)).await;
            assert!(samples.len() < MAX_N_SAMPLES as usize);
        }
    }
}
