use async_trait::async_trait;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Notify;
use uuid::Uuid;

use crate::errors::PhyphoxError;
use common::{IMUReadings, IMUSample};

#[async_trait]
pub trait PhyphoxPort {
    // Registers a listener function to be called whenever new common are available.
    // Returns the id of the registered listener.
    fn register<F, S, D>(&self, listener: F) -> Uuid
    where
        F: Fn(S) + Send + Sync + 'static,
        S: IMUReadings<D>,
        D: IMUSample;

    // Unregisters a listener for the list of registered listeners.
    // Returns a ListenerNotFound Error if the listener wasn't registered.
    fn unregister(&self, id: Uuid) -> Result<(), PhyphoxError>;

    /// Starts the data acquisition process. The process is stopped with a SIGINT signal
    /// Returns FetchData error if it can't connect to REST API.
    async fn start(
        &self,
        period_millis: Duration,
        sensor_tag: String,
        abort_signal: Arc<Notify>,
        window_size: Option<usize>,
    ) -> Result<(), PhyphoxError>;

    // Stops measurement capture on the phone
    async fn stop_cmd(&self) -> Result<(), PhyphoxError>;

    // Clears common from the phone
    async fn clear_cmd(&self) -> Result<(), PhyphoxError>;

    // Clears measurement capture on the phone
    async fn start_cmd(&self) -> Result<(), PhyphoxError>;
}
