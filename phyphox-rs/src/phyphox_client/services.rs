use common::{IMUReadings, IMUSample};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Notify;
use uuid::Uuid;

use crate::errors::PhyphoxError;
use crate::phyphox_client::ports::PhyphoxPort;

pub struct PhyphoxService<T: PhyphoxPort> {
    client: T,
}
impl<T: PhyphoxPort> PhyphoxService<T> {
    /// Creates a new `Phyphox` instance with the specified configuration.
    /// Returns an ClientBuild error if Http client to connect to Phyphox API cannot be created
    pub fn new(client: T) -> Self {
        PhyphoxService { client }
    }

    // Registers a listener function to be called whenever new common are available.
    // Returns the id of the registered listener.
    pub fn register<F, S, D>(&self, listener: F) -> Uuid
    where
        F: Fn(S) + Send + Sync + 'static,
        S: IMUReadings<D>,
        D: IMUSample,
    {
        self.client.register(listener)
    }

    // Unregisters a listener for the list of registered listeners.
    // Returns a ListenerNotFound Error if the listener wasn't registered.
    pub fn unregister(&self, id: Uuid) -> Result<(), PhyphoxError> {
        self.client.unregister(id)
    }

    /// Starts the data acquisition process. The process is stopped with a SIGINT signal
    /// Returns FetchData error if it can't connect to REST API.
    pub async fn start(
        &self,
        period_millis: Duration,
        sensor_tag: String,
        abort_signal: Arc<Notify>,
        window_size: Option<usize>,
    ) -> Result<(), PhyphoxError> {
        self.client
            .start(period_millis, sensor_tag, abort_signal, window_size)
            .await
    }

    // Stops measurement capture on the phone
    pub async fn stop_cmd(&self) -> Result<(), PhyphoxError> {
        self.client.stop_cmd().await
    }
    // Clears common from the phone
    pub async fn clear_cmd(&self) -> Result<(), PhyphoxError> {
        self.client.clear_cmd().await
    }
    // Clears measurement capture on the phone
    pub async fn start_cmd(&self) -> Result<(), PhyphoxError> {
        self.client.start_cmd().await
    }
}
