use async_trait::async_trait;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Notify;
use uuid::Uuid;

use crate::models::errors::PhyphoxError;
use common::types::SensorType;
use publisher::Listener;

#[async_trait]
pub trait PhyphoxPort<T: Send + Sync + 'static> {
    // Registers a listener function for a given sensor type to be called whenever new measures are available.
    fn register_sensor(&self, listener: Listener<T>, sensor_type: SensorType) -> Uuid;

    // Unregisters a listener for the list of registered listeners.
    fn unregister_sensor(&self, id: Uuid, sensor_type: SensorType);

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
