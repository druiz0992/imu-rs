#![allow(dead_code)]

//! Module Mock
//!
//! Emulates functionality of real Phyphox Client

use async_trait::async_trait;
use common::{IMUReadings, IMUSample};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Notify;
use uuid::Uuid;

use crate::errors::PhyphoxError;
use crate::phyphox_client::ports::PhyphoxPort;

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
impl PhyphoxPort for PhyphoxMock {
    // Registers a listener function to be called whenever new common are available.
    // Returns the id of the registered listener.
    fn register<F, S, D>(&self, _listener: F) -> Uuid
    where
        F: Fn(S) + Send + Sync + 'static,
        S: IMUReadings<D>,
        D: IMUSample,
    {
        todo!();
    }

    // Unregisters a listener for the list of registered listeners.
    // Returns a ListenerNotFound Error if the listener wasn't registered.
    fn unregister(&self, _id: Uuid) -> Result<(), PhyphoxError> {
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
