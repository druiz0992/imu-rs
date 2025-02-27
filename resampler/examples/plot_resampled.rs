use uuid::Uuid;

use imu_common::traits::IMUSink;
use imu_common::types::sensors::{SensorReadings, SensorType};
use imu_common::types::timed::Sample3D;

use phyphox_rs;
use resampler_rs::{self, SmothingPolicy};
use test_utils::sinks::Plot1D;
use tokio::time::Duration;

#[tokio::main]
async fn main() {
    let tag = "Test";
    let sensor_cluster = vec![
        SensorType::Accelerometer(Uuid::new_v4()),
        SensorType::Gyroscope(Uuid::new_v4()),
        SensorType::Magnetometer(Uuid::new_v4()),
    ];

    let plot_window_size_samples = 200;

    let plot_resampled_refresh_period_millis = 100.0;
    let plot_1d_resampled = Plot1D::new(
        "Resampled",
        sensor_cluster.clone(),
        plot_window_size_samples,
    );
    plot_1d_resampled.start(plot_resampled_refresh_period_millis);

    let plot_raw_refresh_period_millis = 200.0;
    let plot_1d_raw = Plot1D::new("Raw", sensor_cluster.clone(), plot_window_size_samples);
    plot_1d_raw.start(plot_raw_refresh_period_millis);

    // Start phyphox  service
    let sampling_period_millis = 210.0;
    let (handle_phyphox, phyphox) = phyphox_rs::run_service(
        "http://192.168.1.34",
        tag,
        sensor_cluster.clone(),
        sampling_period_millis,
    )
    .unwrap();

    let resampling_period_millis = 100.0;
    let resampling_delay_millis = 500.0;
    let smoothing_policy = SmothingPolicy::WeightedAverage;
    let (_handle_resample, resampler) = resampler_rs::run::<SensorReadings<Sample3D>, _>(
        tag,
        sensor_cluster.clone(),
        resampling_period_millis,
        resampling_delay_millis,
        smoothing_policy,
    );

    resampler
        .attach_listeners(&*phyphox, &sensor_cluster)
        .unwrap();

    IMUSink::<SensorReadings<Sample3D>, Sample3D>::attach_listeners(
        &plot_1d_resampled,
        &*resampler,
        &sensor_cluster.clone(),
    )
    .unwrap();
    IMUSink::<SensorReadings<Sample3D>, Sample3D>::attach_listeners(
        &plot_1d_raw,
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
