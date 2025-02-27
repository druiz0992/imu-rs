use uuid::Uuid;

use imu_common::traits::IMUSink;
use imu_common::types::sensors::{SensorReadings, SensorType};
use imu_common::types::timed::Sample3D;

use phyphox_rs;
use test_utils::sinks::Plot1D;
use tokio::time::Duration;

#[tokio::main]
async fn main() {
    let sensor_cluster = vec![
        SensorType::Accelerometer(Uuid::new_v4()),
        SensorType::Gyroscope(Uuid::new_v4()),
        SensorType::Magnetometer(Uuid::new_v4()),
    ];

    let plot_refresh_period_millis = 200.0;
    let plot_window_size_samples = 200;
    let plot_1d = Plot1D::new("Raw", sensor_cluster.clone(), plot_window_size_samples);
    plot_1d.start(plot_refresh_period_millis);

    let tag = "Test";

    // Start phyphox  service
    let (handle_phyphox, phyphox) =
        phyphox_rs::run_service("http://192.168.1.34", tag, sensor_cluster.clone(), 400.0).unwrap();

    IMUSink::<SensorReadings<Sample3D>, Sample3D>::attach_listeners(
        &plot_1d,
        &*phyphox,
        &sensor_cluster.clone(),
    )
    .unwrap();

    let timeout_duration = Duration::from_secs(200);
    let _ = tokio::time::timeout(timeout_duration, async {
        handle_phyphox.await.unwrap();
    })
    .await;
}
