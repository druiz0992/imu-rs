#![allow(dead_code)]

// Emulates functionality of real Phyphox Client

use async_trait::async_trait;
use publisher::Listener;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Notify;
use uuid::Uuid;

use crate::models::errors::PhyphoxError;
use crate::ports::PhyphoxPort;
use common::types::SensorType;

/// Configures data acquisition behavior
pub(crate) struct PhyphoxMock;

impl PhyphoxMock {
    /// Creates a new `Phyphox` instance with the specified configuration.
    /// Returns an ClientBuild error if Http client to connect to Phyphox API cannot be created
    pub fn new() -> Self {
        todo!();
    }
}

#[async_trait]
impl<T: Send + Sync + 'static> PhyphoxPort<T> for PhyphoxMock {
    // Registers a listener function to be called whenever new measurements are available.
    // Returns the id of the registered listener.
    fn register_sensor(&self, _listener: Listener<T>, _sensor_type: SensorType) -> Uuid {
        todo!();
    }

    // Unregisters a listener for the list of registered listeners.
    fn unregister_sensor(&self, _id: Uuid, _sensor_type: SensorType) {
        todo!();
    }

    /// Starts the data acquisition process. The process is stopped with a SIGINT signal
    /// Returns FetchData error if it can't connect to REST API.
    async fn start(
        &self,
        _period_millis: Duration,
        _sensor_tag: String,
        _abort_signal: Arc<Notify>,
        _window_size: Option<usize>,
    ) -> Result<(), PhyphoxError> {
        todo!();
    }

    // Stops measurement capture on the phone
    async fn stop_cmd(&self) -> Result<(), PhyphoxError> {
        todo!();
    }
    // Clears common from the phone
    async fn clear_cmd(&self) -> Result<(), PhyphoxError> {
        todo!();
    }
    // Clears measurement capture on the phone
    async fn start_cmd(&self) -> Result<(), PhyphoxError> {
        todo!();
    }
}
