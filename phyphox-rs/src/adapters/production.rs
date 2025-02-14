#![allow(dead_code)]

// Functionality for data acquisition from accelerometer, gyroscope and magnetometer phone sensors
// via an HTTP API. It includes methods to fetch sensor data,
// control common, and register listeners to receive for incoming data.

use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use serde_json::Value;
use tokio::sync::Notify;

use common::traits::{IMUFilter, IMUReadings, IMUSample};
use common::types::filters::moving_average::MovingAverage;
use common::types::sensors::{SensorReadings, SensorType};
use common::types::timed::Sample3D;
use common::types::untimed::XYZ;
use common::types::Clock;
use publisher::{Publishable, Publisher};

use crate::constants::N_SENSORS;
use crate::helpers;
use crate::models::errors::PhyphoxError;
use crate::models::http_client::HttpClient;
use crate::ports::PhyphoxPort;

/// Constants for HTTP endpoints and buffer keys.
const GET_CMD: &str = "/get?";
const CONTROL_CMD: &str = "/control?cmd=";
const START_CMD: &str = "start";
const STOP_CMD: &str = "stop";
const CLEAR_CMD: &str = "clear";
const CONFIG_CMD: &str = "/config?";

/// Configures data acquisition
pub struct Phyphox {
    client: HttpClient,
    sensor_cluster_tag: String,
    sensor_cluster: Vec<SensorType>,
}

impl Phyphox {
    /// Creates a new `Phyphox` instance with the specified configuration.
    /// Returns an ClientBuild error if Http client to connect to Phyphox API cannot be created
    pub(crate) fn new(
        base_url: &str,
        sensor_cluster_tag: &str,
        sensor_cluster: Vec<SensorType>,
    ) -> Result<Self, PhyphoxError> {
        let client = HttpClient::new(base_url.to_string())?;

        Ok(Self {
            client,
            sensor_cluster_tag: sensor_cluster_tag.to_string(),
            sensor_cluster,
        })
    }

    /// Returns JSON data from the specified path or FetchData error if it couldnt retrieve data from REST API
    async fn fetch_json(&self, path: &str) -> Result<Value, PhyphoxError> {
        self.client.fetch_json(path).await
    }

    /// Returns a tuple containing the retrieved sensor data and a flag indicating if sensor is still active
    /// # Errors
    /// - FetchData if there is an error connecting to REST API
    /// - IncorrectDataFormat if the data retrieved from the API has an unexpected format
    async fn get_data(
        &self,
        time_var: &str,
        timestamp_at_boot: f64,
        since: Option<f64>,
        variables: &[&str],
    ) -> Result<(Vec<f64>, Vec<XYZ>, bool), PhyphoxError> {
        let query = helpers::build_query(variables, time_var, since);
        let data = self.fetch_json(&format!("{GET_CMD}{}", query)).await?;
        let status = helpers::get_status_from_json(&data)?;
        let results = helpers::parse_results(&data, variables, time_var)?;
        let (timestamp, untimed_data) = helpers::combine_results(results, timestamp_at_boot);

        Ok((timestamp, untimed_data, status))
    }

    /// Sends a control command to the Phyphox server. Returns FetchData error if it can't connect with
    /// the REST API
    async fn control(&self, command: &str) -> Result<(), PhyphoxError> {
        self.fetch_json(&format!("{CONTROL_CMD}{}", command))
            .await?;
        Ok(())
    }
    // Clears common from the phone
    async fn clear_cmd(&self) -> Result<(), PhyphoxError> {
        log::info!("Clearing data...");
        self.control(CLEAR_CMD).await
    }
    // Clears measurement capture on the phone
    async fn start_cmd(&self) -> Result<(), PhyphoxError> {
        log::info!("Starting recording...");
        self.control(START_CMD).await
    }
    // Stops measurement capture on the phone
    async fn stop_cmd(&self) -> Result<(), PhyphoxError> {
        log::info!("Stopping recording...");
        self.control(STOP_CMD).await
    }
}

#[async_trait]
impl PhyphoxPort for Phyphox {
    /// Starts the data acquisition process. The process is stopped with a SIGINT signal
    /// Returns FetchData error if it can't connect to REST API.
    async fn start(
        &self,
        period_millis: Duration,
        abort_signal: Option<Arc<Notify>>,
        window_size: Option<usize>,
        publisher: Option<Vec<Publisher<SensorReadings<Sample3D>>>>,
    ) -> Result<(), PhyphoxError> {
        let timestamp_at_boot = Clock::now().as_f64();
        self.clear_cmd().await?;
        self.start_cmd().await?;

        let mut last_time = [0.0; N_SENSORS];

        log::info!("Fetching data...");

        let mut ma_filters: Vec<Option<MovingAverage<XYZ>>> = window_size
            .map_or(vec![None; N_SENSORS], |w_size| {
                vec![Some(MovingAverage::new(w_size)); N_SENSORS]
            });

        let active_sensor = self
            .get_available_sensors()
            .await
            .map_err(|e| PhyphoxError::Other(e.to_string()))?;

        let abort_signal = abort_signal.unwrap_or(Arc::new(Notify::new()));

        loop {
            tokio::select! {
                _ = abort_signal.notified() => {
                    break;
                }

                _ = tokio::time::sleep(period_millis) => {
                    for sensor in &self.sensor_cluster {
                        if !active_sensor.contains(sensor) {
                            continue;
                        }
                        let sensor_idx = usize::from(sensor);
                        let (time_str, variables, sensor_idx) = helpers::control_str(sensor_idx)?;
                        let (timestamp_info, untimed_data_info,  is_measuring) = match self
                            .get_data(time_str, timestamp_at_boot, Some(last_time[sensor_idx]), &variables)
                            .await {
                                Ok(result) => result,
                                Err(e) => {
                                    log::error!("Error fetching data: {:?}", e);
                                    break;
                                }
                        };

                        if !is_measuring {
                            log::info!("Recording stopped.");
                            break;
                        }

                        helpers::update_measurement_time(&timestamp_info, &mut last_time[sensor_idx]);

                        let timed_samples: Vec<Sample3D> = timestamp_info.into_iter().zip(untimed_data_info.into_iter()).map(|(t, s)| Sample3D::from_measurement(t, s)).collect();
                        let filtered_data = match ma_filters[sensor_idx].as_mut() {
                            Some(ma_filter) => ma_filter.filter_batch(timed_samples),
                            None => Ok(timed_samples)
                        };

                        if let Ok(filtered_data) = filtered_data {
                            let buffer  = SensorReadings::from_vec(&self.sensor_cluster_tag, sensor.clone(), filtered_data);

                            if let Some(publisher) = publisher.as_ref() {
                                publisher[sensor_idx].notify_listeners(Arc::new(buffer)).await
                            };
                        }

                    }
                }
            }
        }

        self.stop_cmd().await?;
        Ok(())
    }
    fn get_tag(&self) -> &str {
        self.sensor_cluster_tag.as_str()
    }

    /// Returns available sensors in phyphox
    async fn get_available_sensors(&self) -> Result<Vec<SensorType>, String> {
        let json = self
            .fetch_json(CONFIG_CMD)
            .await
            .map_err(|_| "Error retrieving available sensors".to_string())?;
        let mut available_sensors: Vec<SensorType> = vec![];

        let sensor_uuids = helpers::extract_uuids(&self.sensor_cluster);

        if let Some(exports) = json.get("export").and_then(|e| e.as_array()) {
            available_sensors = exports
                .iter()
                .filter_map(|entry| {
                    entry
                        .get("set")
                        .and_then(|s| s.as_str())
                        .and_then(|s| helpers::to_type_id(s, &sensor_uuids))
                        .and_then(|s| s.try_into().ok())
                })
                .collect();
        }
        Ok(available_sensors)
    }
    fn get_sensor_cluster(&self) -> Vec<SensorType> {
        self.sensor_cluster.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use uuid::Uuid;
    use wiremock::matchers::method;
    use wiremock::{Mock, MockServer, ResponseTemplate};

    #[tokio::test]
    async fn test_phyphox_new() {
        let sensor_cluster = vec![
            SensorType::Accelerometer(Uuid::new_v4()),
            SensorType::Gyroscope(Uuid::new_v4()),
            SensorType::Magnetometer(Uuid::new_v4()),
        ];
        Phyphox::new("http://localhost", "Test", sensor_cluster)
            .expect("Error creating Phyphox instance");
    }

    #[tokio::test]
    async fn test_phyphox_fetch_json() {
        let mock_server = MockServer::start().await;
        let sensor_cluster = vec![
            SensorType::Accelerometer(Uuid::new_v4()),
            SensorType::Gyroscope(Uuid::new_v4()),
            SensorType::Magnetometer(Uuid::new_v4()),
        ];

        Mock::given(method("GET"))
            .respond_with(ResponseTemplate::new(200).set_body_json(serde_json::json!({
                "key": "value"
            })))
            .mount(&mock_server)
            .await;

        let phyphox = Phyphox::new(mock_server.uri().as_str(), "Test", sensor_cluster).unwrap();

        let result = phyphox.fetch_json("/get?test").await.unwrap();
        assert_eq!(result["key"], "value");
    }

    #[tokio::test]
    async fn test_phyphox_control() {
        let mock_server = MockServer::start().await;
        let sensor_cluster = vec![
            SensorType::Accelerometer(Uuid::new_v4()),
            SensorType::Gyroscope(Uuid::new_v4()),
            SensorType::Magnetometer(Uuid::new_v4()),
        ];

        Mock::given(method("GET"))
            .respond_with(ResponseTemplate::new(200).set_body_json(serde_json::json!({
                "key": "value",
                "nested": {
                    "field": 123
                }
            })))
            .mount(&mock_server)
            .await;

        let phyphox = Phyphox::new(mock_server.uri().as_str(), "Test", sensor_cluster).unwrap();

        let result = phyphox.clear_cmd().await;
        assert!(result.is_ok());
    }

    #[tokio::test]
    async fn test_phyphox_get_data() {
        let mock_server = MockServer::start().await;
        let sensor_cluster = vec![
            SensorType::Accelerometer(Uuid::new_v4()),
            SensorType::Gyroscope(Uuid::new_v4()),
            SensorType::Magnetometer(Uuid::new_v4()),
        ];

        Mock::given(method("GET"))
            .respond_with(ResponseTemplate::new(200).set_body_json(serde_json::json!({
                "buffer": {
                    "accX": { "buffer": [1.0, 2.0], "size": 0, "updateMode": "partial" },
                    "accY": { "buffer": [3.0, 4.0], "size": 0, "updateMode": "partial" },
                    "accZ": { "buffer": [5.0, 6.0], "size": 0, "updateMode": "partial" },
                    "acc_time": { "buffer": [1.0, 2.0], "size": 0, "updateMode": "partial" }
                },
                "status": {
                    "measuring": true
                }
            })))
            .mount(&mock_server)
            .await;

        let phyphox = Phyphox::new(mock_server.uri().as_str(), "Test", sensor_cluster).unwrap();

        let (_timestamps, data, is_measuring) = phyphox
            .get_data("acc_time", 0.0, Some(0.0), &["accX", "accY", "accZ"])
            .await
            .unwrap();

        assert_eq!(
            data,
            vec![
                XYZ::try_from(vec![1.0, 3.0, 5.0]).unwrap(),
                XYZ::try_from(vec![2.0, 4.0, 6.0]).unwrap()
            ]
        );
        assert!(is_measuring);
    }

    #[tokio::test]
    async fn test_phyphox_get_incomplete_data() {
        let mock_server = MockServer::start().await;
        let sensor_cluster = vec![
            SensorType::Accelerometer(Uuid::new_v4()),
            SensorType::Gyroscope(Uuid::new_v4()),
            SensorType::Magnetometer(Uuid::new_v4()),
        ];

        Mock::given(method("GET"))
            .respond_with(ResponseTemplate::new(200).set_body_json(serde_json::json!({
                "buffer": {
                    "accX": { "buffer": [1.0, 2.0], "size": 0, "updateMode": "partial" },
                    "accY": { "buffer": [3.0], "size": 0, "updateMode": "partial" },
                    "accZ": { "buffer": [5.0, 6.0], "size": 0, "updateMode": "partial" },
                    "acc_time": { "buffer": [1.0, 2.0], "size": 0, "updateMode": "partial" }
                },
                "status": {
                    "measuring": true
                }
            })))
            .mount(&mock_server)
            .await;

        let phyphox = Phyphox::new(mock_server.uri().as_str(), "Test", sensor_cluster).unwrap();

        let (_timestamps, data, is_measuring) = phyphox
            .get_data("acc_time", 0.0, Some(0.0), &["accX", "accY", "accZ"])
            .await
            .unwrap();

        assert_eq!(data, vec![XYZ::try_from(vec![1.0, 3.0, 5.0]).unwrap(),]);
        assert!(is_measuring);
    }
}
