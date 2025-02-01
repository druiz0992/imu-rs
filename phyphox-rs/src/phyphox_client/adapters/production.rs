#![allow(dead_code)]

//! Module phyphox
//!
//! Functionality for data acquisition from accelerometer, gyroscope and magnetometer phone sensors
//! via an HTTP API. It includes methods to fetch sensor data,
//! control common, and register listeners to receive for incoming data.

use async_trait::async_trait;
pub use publisher::Publisher;
use serde_json::Value;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Notify;
use uuid::Uuid;

use common::traits::{IMUFilter, IMUReadings, IMUSample};
use common::types::{Sample3D, Sensor, XYZ};

use crate::errors::PhyphoxError;
use crate::filter;
use crate::helpers;
use crate::http_client::HttpClient;
use crate::phyphox_client::ports::PhyphoxPort;

/// Constants for HTTP endpoints and buffer keys.
const GET_CMD: &str = "/get?";
const CONTROL_CMD: &str = "/control?cmd=";
const START_CMD: &str = "start";
const STOP_CMD: &str = "stop";
const CLEAR_CMD: &str = "clear";

const N_SENSORS: usize = 3;
/// Configures data acquisition behavior
pub struct Phyphox {
    client: HttpClient,
    publisher: Publisher<Box<dyn Fn(Sensor<Sample3D>) + Send + Sync>>,
}

impl Phyphox {
    /// Creates a new `Phyphox` instance with the specified configuration.
    /// Returns an ClientBuild error if Http client to connect to Phyphox API cannot be created
    pub fn new(base_url: String) -> Result<Self, PhyphoxError> {
        let client = HttpClient::new(base_url)?;

        Ok(Self {
            client,
            publisher: Publisher::new(),
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
        since: Option<f64>,
        variables: &[&str],
    ) -> Result<(Vec<f64>, Vec<XYZ>, bool), PhyphoxError> {
        let query = helpers::build_query(variables, time_var, since);
        let data = self.fetch_json(&format!("{GET_CMD}{}", query)).await?;
        let status = helpers::get_status_from_json(&data)?;
        let results = helpers::parse_results(&data, variables, time_var)?;
        let (timestamp, untimed_data) = helpers::combine_results(results);

        Ok((timestamp, untimed_data, status))
    }

    /// Sends a control command to the Phyphox server. Returns FetchData error if it can't connect with
    /// the REST API
    async fn control(&self, command: &str) -> Result<(), PhyphoxError> {
        self.fetch_json(&format!("{CONTROL_CMD}{}", command))
            .await?;
        Ok(())
    }
}

#[async_trait]
impl PhyphoxPort for Phyphox {
    // Registers a listener function to be called whenever new common are available.
    // Returns the id of the registered listener.
    fn register<F, S, D>(&self, listener: F) -> Uuid
    where
        F: Fn(S) + Send + Sync + 'static,
        S: IMUReadings<D>,
        D: IMUSample,
    {
        self.publisher.register(Box::new(listener))
    }

    // Unregisters a listener for the list of registered listeners.
    // Returns a ListenerNotFound Error if the listener wasn't registered.
    fn unregister(&self, id: Uuid) -> Result<(), PhyphoxError> {
        self.publisher.unregister(id).map_err(|_| {
            PhyphoxError::ListenerNotFound(format!("Listener with id {} not found", id))
        })
    }

    /// Starts the data acquisition process. The process is stopped with a SIGINT signal
    /// Returns FetchData error if it can't connect to REST API.
    async fn start(
        &self,
        period_millis: Duration,
        sensor_tag: String,
        abort_signal: Arc<Notify>,
        window_size: Option<usize>,
    ) -> Result<(), PhyphoxError> {
        self.clear_cmd().await?;
        self.start_cmd().await?;

        let mut last_time = [0.0; N_SENSORS];

        log::info!("Fetching data...");

        let mut ma_filters: Vec<Option<filter::MovingAverage<XYZ>>> = window_size
            .map_or(vec![None; N_SENSORS], |w_size| {
                vec![Some(filter::MovingAverage::new(w_size)); N_SENSORS]
            });

        loop {
            tokio::select! {
                _ = abort_signal.notified() => {
                    break;
                }

                _ = tokio::time::sleep(period_millis) => {
                    for sensor_idx in 0..=N_SENSORS {
                        let (time_str, variables) = helpers::control_str(sensor_idx)?;
                        let (timestamp_info, untimed_data_info,  is_measuring) = match self
                            .get_data(time_str, Some(last_time[sensor_idx]), &variables)
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

                        let filtered_untimed_data = match ma_filters[sensor_idx].as_mut() {
                            Some(ma_filter) => ma_filter.filter(untimed_data_info),
                            None => untimed_data_info, // No need to clone here
                        };
                        let filtered_data = timestamp_info.into_iter().zip(filtered_untimed_data.into_iter()).map(|(t,s)| Sample3D::from_xyz(t,s)).collect();
                        let buffer = Sensor::from_vec(&sensor_tag, filtered_data);

                        //self.publisher.notify(Arc::new(buffer));
                    }
                }
            }
        }

        self.stop_cmd().await?;
        Ok(())
    }

    // Stops measurement capture on the phone
    async fn stop_cmd(&self) -> Result<(), PhyphoxError> {
        log::info!("Stopping recording...");
        self.control(STOP_CMD).await
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use wiremock::matchers::method;
    use wiremock::{Mock, MockServer, ResponseTemplate};

    #[tokio::test]
    async fn test_phyphox_new() {
        Phyphox::new("http://localhost".to_string()).expect("Error creating Phyphox instance");
    }

    #[tokio::test]
    async fn test_phyphox_fetch_json() {
        let mock_server = MockServer::start().await;

        Mock::given(method("GET"))
            .respond_with(ResponseTemplate::new(200).set_body_json(serde_json::json!({
                "key": "value"
            })))
            .mount(&mock_server)
            .await;

        let phyphox = Phyphox::new(mock_server.uri()).unwrap();

        let result = phyphox.fetch_json("/get?test").await.unwrap();
        assert_eq!(result["key"], "value");
    }

    #[tokio::test]
    async fn test_phyphox_control() {
        let mock_server = MockServer::start().await;

        Mock::given(method("GET"))
            .respond_with(ResponseTemplate::new(200).set_body_json(serde_json::json!({
                "key": "value",
                "nested": {
                    "field": 123
                }
            })))
            .mount(&mock_server)
            .await;

        let phyphox = Phyphox::new(mock_server.uri()).unwrap();

        let result = phyphox.clear_cmd().await;
        assert!(result.is_ok());
    }

    #[tokio::test]
    async fn test_phyphox_get_data() {
        let mock_server = MockServer::start().await;

        Mock::given(method("GET"))
            .respond_with(ResponseTemplate::new(200).set_body_json(serde_json::json!({
                "buffer": {
                    "accX": { "buffer": [1.0, 2.0], "size": 0, "updateMode": "partial" },
                    "accY": { "buffer": [3.0, 4.0], "size": 0, "updateMode": "partial" },
                    "accZ": { "buffer": [5.0, 6.0], "size": 0, "updateMode": "partial" },
                    "acc_time": { "buffer": [7.0, 8.0], "size": 0, "updateMode": "partial" }
                },
                "status": {
                    "measuring": true
                }
            })))
            .mount(&mock_server)
            .await;

        let phyphox = Phyphox::new(mock_server.uri()).unwrap();

        let (data, is_measuring) = phyphox
            .get_data("acc_time", Some(0.0), &["accX", "accY", "accZ"])
            .await
            .unwrap();

        assert_eq!(
            data,
            vec![vec![7.0, 1.0, 3.0, 5.0], vec![8.0, 2.0, 4.0, 6.0]]
        );
        assert!(is_measuring);
    }
}
