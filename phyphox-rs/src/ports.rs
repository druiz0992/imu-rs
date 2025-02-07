use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{Mutex, Notify};

use async_trait::async_trait;
use uuid::Uuid;

use common::types::SensorType;
use common::{IMUReadings, IMUSample};
use publisher::{Listener, Publishable, Publisher};

use crate::constants::N_SENSORS;
use crate::models::errors::PhyphoxError;

#[async_trait]
pub trait PhyphoxPort<T, S>
where
    T: Send + Sync + IMUReadings<S> + 'static,
    S: IMUSample,
{
    /// Starts the data acquisition process. The process is stopped with a SIGINT signal
    /// Returns FetchData error if it can't connect to REST API.
    async fn start(
        &self,
        period_millis: Duration,
        sensor_cluster: &[SensorType],
        abort_signal: Option<Arc<Notify>>,
        window_size: Option<usize>,
        publisher: Option<Arc<[Mutex<Publisher<T>>; N_SENSORS]>>,
    ) -> Result<(), PhyphoxError>;
}
