use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Notify;

use async_trait::async_trait;

use imu_common::types::timed::Sample3D;
use imu_common::types::{SensorReadings, SensorType};
use publisher::Publisher;

use crate::models::errors::PhyphoxError;

#[async_trait]
pub trait PhyphoxPort {
    /// Starts the data acquisition process. The process is stopped with a SIGINT signal
    /// Returns FetchData error if it can't connect to REST API.
    async fn start(
        &self,
        period_millis: Duration,
        abort_signal: Option<Arc<Notify>>,
        publisher: Option<Vec<Publisher<SensorReadings<Sample3D>>>>,
    ) -> Result<(), PhyphoxError>;

    fn get_tag(&self) -> &str;
    fn get_sensor_cluster(&self) -> Vec<SensorType>;
}
