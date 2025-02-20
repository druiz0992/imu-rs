use async_trait::async_trait;
use rand::{rngs::StdRng, SeedableRng};
use std::path::PathBuf;
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
use common::types::Clock;
use publisher::{Publishable, Publisher};
use test_utils::csv_loader::{self, CsvColumnMapper};

const GAUSSIAN_TIME_MEAN: f64 = 0f64;
const GAUSSIAN_SENSOR_MEAN: f64 = 0f64;
const GAUSSIAN_SENSOR_STDEV: f64 = 0.5;
const LSB_TIME_MASK: u8 = 0xC0;
const MAX_N_SAMPLES: u8 = 15;

/// Configures mock data acquisition
pub struct PhyphoxMock {
    readings: Mutex<[CircularReader<Sample3D>; N_SENSORS]>,
    timestamps: Mutex<Timestamp>,
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
        update_period_millis: f64,
        add_sensor_noise: bool,
    ) -> Result<Self, PhyphoxError> {
        let test_data = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../test-utils/test_data/sensor_readings.csv");
        let test_data = test_data.to_str().unwrap();

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
        mag_mapper.add_timestamp().add_mag();
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
            time_delta: GaussianNoise::new(GAUSSIAN_TIME_MEAN, update_period_millis / 1000.0 * 0.2),
            sensor_noise: add_sensor_noise
                .then(|| GaussianNoise::new(GAUSSIAN_SENSOR_MEAN, GAUSSIAN_SENSOR_STDEV)),
            sensor_cluster,
        })
    }

    async fn get_next_samples(&self, buffer_idx: usize) -> Vec<Sample3D> {
        let mut new_samples = Vec::new();
        let pending_samples = select_random_pending_samples();
        let mut rng = StdRng::from_entropy();
        let mut timestamps = self.timestamps.lock().await;
        let current_timestamp = timestamps.get_current_timestamp();
        if pending_samples > 0 {
            let mut readings = self.readings.lock().await;
            for _ in 0..pending_samples {
                let next_sample = readings[buffer_idx].next_element();
                let mut sample_timestamp = self
                    .time_delta
                    .add_noise(&mut rng, timestamps.get_reading_timestamp(buffer_idx))
                    .abs();
                sample_timestamp = sample_timestamp.min(current_timestamp);
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
                ));
                new_samples.sort_by(|a, b| {
                    a.get_timestamp_secs()
                        .partial_cmp(&b.get_timestamp_secs())
                        .unwrap()
                });
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
        publisher: Option<Vec<Publisher<SensorReadings<Sample3D>>>>,
    ) -> Result<(), PhyphoxError> {
        let abort_signal = abort_signal.unwrap_or(Arc::new(Notify::new()));
        let timestamp_at_boot_secs = Clock::now().as_secs();
        {
            let mut timestamp = self.timestamps.lock().await;
            timestamp.update_all(timestamp_at_boot_secs);
        }
        loop {
            tokio::select! {
                _ = abort_signal.notified() => {
                    break;
                }
                _ = tokio::time::sleep(period_millis) => {
                    let mut timestamp = self.timestamps.lock().await;
                    timestamp.set_current_timestamp(Clock::now().as_secs());
                    drop(timestamp);

                    for sensor in &self.sensor_cluster {
                        let sensor_idx = match usize::from(sensor) {
                            sensor_type::ACCELEROMETER_OFFSET..sensor_type::GYROSCOPE_OFFSET => 0,
                            sensor_type::GYROSCOPE_OFFSET..sensor_type::MAGNETOMETER_OFFSET => 1,
                            sensor_type::MAGNETOMETER_OFFSET..sensor_type::MAX_OFFSET => 2,
                            _ => 2,
                        };
                        let samples = self.get_next_samples(sensor_idx).await;
                        if !samples.is_empty() {
                            let buffer = SensorReadings::from_vec(&self.sensor_cluster_tag, sensor.clone(), samples);
                            if let Some(publisher) = publisher.as_ref() {
                                publisher[sensor_idx].notify_listeners(Arc::new(buffer)).await;
                            };
                        }
                    }
                    let mut timestamp = self.timestamps.lock().await;
                    timestamp.update_all(Clock::now().as_secs());
                    drop(timestamp);
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
        let phyphox_mock = PhyphoxMock::new("Test", sensor_cluster, 100.0, false);
        assert!(phyphox_mock.is_ok());
    }

    #[tokio::test]
    async fn test_phyphox_mock_start_stop() {
        let sensor_cluster = vec![
            SensorType::Accelerometer(Uuid::new_v4()),
            SensorType::Gyroscope(Uuid::new_v4()),
            SensorType::Magnetometer(Uuid::new_v4()),
        ];
        let phyphox_mock =
            Arc::new(PhyphoxMock::new("Test", sensor_cluster, 100.0, false).unwrap());
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
                .start(period, Some(abort_signal), None)
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
        let period_millis = 100.0;
        let factor = 10.0;
        let sensor_cluster = vec![SensorType::Accelerometer(Uuid::new_v4())];
        let phyphox_mock = PhyphoxMock::new("Test", sensor_cluster, period_millis, true).unwrap();
        for _ in 0..10 {
            // update timestamp
            let mut timestamp = phyphox_mock.timestamps.lock().await;
            timestamp.update_all(Clock::now().as_secs());
            drop(timestamp);
            let mut samples = Vec::new();

            for _ in 0..factor as usize {
                let new_samples = phyphox_mock.get_next_samples(0).await;
                assert!(new_samples.len() < MAX_N_SAMPLES as usize);
                samples.extend_from_slice(&new_samples);
                tokio::time::sleep(Duration::from_secs_f64(period_millis / 1000.0 / factor)).await;
            }
            assert!(!samples.is_empty());
        }
    }
}
