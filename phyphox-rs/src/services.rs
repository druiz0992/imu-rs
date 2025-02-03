use std::marker::PhantomData;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Notify;
use uuid::Uuid;

use crate::models::errors::PhyphoxError;
use crate::ports::PhyphoxPort;
use common::types::SensorType;
use publisher::Listener;

pub struct PhyphoxService<C, T>
where
    C: PhyphoxPort<T>,
    T: Send + Sync + 'static,
{
    client: C,
    _phantom: PhantomData<T>,
}
impl<C, T> PhyphoxService<C, T>
where
    C: PhyphoxPort<T>,
    T: Send + Sync + 'static,
{
    /// Creates a new `Phyphox` instance with the specified configuration.
    /// Returns an ClientBuild error if Http client to connect to Phyphox API cannot be created
    pub fn new(client: C) -> Self {
        PhyphoxService {
            client,
            _phantom: PhantomData,
        }
    }
    // Registers a accelerometer/gyroscope/magentometer listener functions to be called whenever new samples are available.
    // Returns the id of the registered listener.
    fn register_sensor(&self, listener: Listener<T>, sensor_type: SensorType) -> Uuid {
        self.client.register_sensor(listener, sensor_type)
    }

    // Unregisters a accelerometer/gyroscope/magnetometer listeners from the list of registered listeners.
    fn unregister_sensor(&self, id: Uuid, sensor_type: SensorType) {
        self.client.unregister_sensor(id, sensor_type);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Phyphox;

    #[tokio::test]
    async fn test_phyphox_client_new() {
        let client =
            Phyphox::new("http://localhost".to_string()).expect("Error creating Phyphox instance");
        PhyphoxService::new(client);
    }
}
