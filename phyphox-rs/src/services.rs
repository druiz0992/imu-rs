use common::{Sample3D, SensorReadings, SensorType};
use log::error;
use publisher::{Listener, Publishable, Publisher};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Notify;
use uuid::Uuid;

use crate::adapters::{mock::PhyphoxMock, production::Phyphox};
/// Generic Phyphox service
use crate::constants::N_SENSORS;
use crate::models::errors::PhyphoxError;
use crate::models::shutdown;
use crate::ports::PhyphoxPort;

const SENSOR_CLUSTER: [SensorType; N_SENSORS] = [
    SensorType::Accelerometer,
    SensorType::Gyroscope,
    SensorType::Magnetometer,
];

/// Configuration of Phyphox service
pub struct PhyphoxService<C>
where
    C: PhyphoxPort,
{
    client: C,
    publisher: [Publisher<SensorReadings<Sample3D>>; N_SENSORS],
    abort_signal: Arc<Notify>,
}

impl<C> PhyphoxService<C>
where
    C: PhyphoxPort,
{
    /// Creates a new `Phyphox` instance with the specified configuration.
    /// Returns an ClientBuild error if Http client to connect to Phyphox API cannot be created
    pub fn new(client: C) -> Self {
        let abort_signal = Arc::new(Notify::new());

        PhyphoxService {
            client,
            abort_signal,
            publisher: std::array::from_fn(|_| Publisher::new()),
        }
    }

    // Registers a accelerometer/gyroscope/magentometer listener functions to be called whenever new samples are available.
    // Returns the id of the registered listener.
    pub async fn register_listener(
        &self,
        mut listener: Listener<SensorReadings<Sample3D>>,
        sensor_type: SensorType,
    ) -> Uuid {
        let sensor_idx = usize::from(sensor_type);
        self.publisher[sensor_idx]
            .register_listener(&mut listener)
            .await
    }

    // Unregisters a accelerometer/gyroscope/magnetometer listeners from the list of registered listeners.
    pub async fn unregister_listener(&self, id: Uuid, sensor_type: SensorType) {
        let sensor_idx = usize::from(sensor_type);
        self.publisher[sensor_idx].unregister_listener(id).await;
    }

    pub async fn notify_listeners(
        &self,
        sensor_type: SensorType,
        data: Arc<SensorReadings<Sample3D>>,
    ) {
        let sensor_idx = usize::from(sensor_type);
        self.publisher[sensor_idx].notify_listeners(data).await;
    }
    /// Starts the data acquisition process. The process is stopped with a SIGINT signal
    /// Returns FetchData error if it can't connect to REST API.
    pub async fn start(
        &self,
        period_millis: Duration,
        window_size: Option<usize>,
        run_for_millis: Option<u64>,
    ) -> Result<(), PhyphoxError> {
        let abort_signal = self.abort_signal.clone();
        shutdown::listen_for_shutdown(Arc::clone(&abort_signal), run_for_millis);
        self.client
            .start(
                period_millis,
                &SENSOR_CLUSTER,
                Some(self.abort_signal.clone()),
                window_size,
                Some(self.publisher.clone()),
            )
            .await
    }
}

/// Starts the phyphox service asynchronously, handling sensor data acquisition and processing.
///
/// This function initializes the required sensors, and begins
/// data collection in a background task.
///
/// An error ClientBuild is returned if http client connecting with phyphox app REST API cannot be created.
///
/// # Returns
///
/// Returns a tuple containing:
/// * A `tokio::task::JoinHandle<()>` representing the spawned asynchronous task.
/// * An `Arc<PhyphoxService<Phyphox>>` instance, allowing further interaction with the sensor system.

pub fn run_service(
    base_url: &str,
    sensor_cluster_tag: &str,
    update_period_millis: u64,
    window_size: Option<usize>,
) -> Result<(tokio::task::JoinHandle<()>, Arc<PhyphoxService<Phyphox>>), PhyphoxError> {
    let phyphox = Phyphox::new(base_url, sensor_cluster_tag)?;
    let phyphox_service: Arc<PhyphoxService<Phyphox>> = Arc::new(PhyphoxService::new(phyphox));
    let handle = tokio::spawn({
        let phyphox_service_clone = phyphox_service.clone();
        async move {
            if let Err(e) = phyphox_service_clone
                .start(
                    Duration::from_millis(update_period_millis),
                    window_size,
                    None, // run until ctrl-c signal
                )
                .await
            {
                error!("Error in Phyphox loop: {:?}", e);
            }
        }
    });
    Ok((handle, phyphox_service))
}

/// Starts the a mock phyphox service that generates pre-stored data.
///
/// Returns a tuple containing:
/// - A `tokio::task::JoinHandle<()>` representing the spawned asynchronous task.
/// - An `Arc<PhyphoxService<PhyphoxMock>>` instance, allowing further interaction with the sensor system.
///
/// An error ClientBuild is returned if http client connecting with phyphox app REST API cannot be created.
pub fn run_mock_service(
    sensor_cluster_tag: &str,
    update_period_millis: u64,
    capture_sampling_period_millis: u64,
    add_sensor_noise: bool,
    run_for_millis: u64,
) -> Result<
    (
        tokio::task::JoinHandle<()>,
        Arc<PhyphoxService<PhyphoxMock>>,
    ),
    PhyphoxError,
> {
    let phyphox = PhyphoxMock::new(
        sensor_cluster_tag,
        capture_sampling_period_millis,
        add_sensor_noise,
    )?;
    let phyphox_service: Arc<PhyphoxService<PhyphoxMock>> = Arc::new(PhyphoxService::new(phyphox));
    let handle = tokio::spawn({
        let phyphox_service_clone = phyphox_service.clone();
        async move {
            if let Err(e) = phyphox_service_clone
                .start(
                    Duration::from_millis(update_period_millis),
                    None,
                    Some(run_for_millis),
                )
                .await
            {
                error!("Error in Phyphox loop: {:?}", e);
            }
        }
    });
    Ok((handle, phyphox_service))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::PhyphoxMock;
    use crate::adapters::production::Phyphox;

    #[tokio::test]
    async fn test_phyphox_client_new() {
        let client =
            Phyphox::new("http://localhost", "Test").expect("Error creating Phyphox instance");
        PhyphoxService::new(client);
    }

    #[tokio::test]
    async fn test_phyphox_mini_client_new() {
        let client = PhyphoxMock::new("Test", 10, false).expect("Error creating Phyphox instance");
        let client_service = Arc::new(PhyphoxService::new(client));

        let client_service_clone = Arc::clone(&client_service);
        let start_task = tokio::task::spawn(async move {
            client_service_clone
                .start(Duration::from_millis(1000), None, Some(1000))
                .await
                .unwrap();
        });

        start_task.await.unwrap();
    }

    #[tokio::test]
    async fn test_run_mock_service() {
        let sensor_tag = "Test";
        let update_period_millis = 100;
        let capture_sampling_period_millis = 5;
        let add_sensor_noise = false;
        let run_for_millis = 1000;
        let (handle, _service) = run_mock_service(
            sensor_tag,
            update_period_millis,
            capture_sampling_period_millis,
            add_sensor_noise,
            run_for_millis,
        )
        .unwrap();

        handle.await.unwrap();
    }
}
