# IMU-RS

IMU-RS is a Rust library for interfacing with Inertial Measurement Units (IMUs). This library provides a simple and efficient way to read data from various IMU sensors.

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
imu-rs = "0.1.0"
```

## Usage

Here's a basic example of how to use IMU-RS:

```rust
use uuid::Uuid;

use common::traits::{IMUReadings, IMUSample, IMUSink};
use common::types::sensors::{SensorReadings, SensorType};
use common::types::timed::{Sample3D, SampleQuaternion};

use ahrs_rs::{self, AHRSFilter};
use phyphox_rs;
use resampler_rs::{self, SmothingPolicy};
use std::sync::Arc;
use test_utils::renderable::Box3D;
use test_utils::sinks::Plot3D;
use tokio::time::Duration;

#[tokio::main]
async fn main() {
    let box_3d = Box3D::new();
    let plot_3d = Plot3D::new(box_3d);

    let sensor_cluster = vec![
        SensorType::Accelerometer(Uuid::new_v4()),
        SensorType::Gyroscope(Uuid::new_v4()),
        SensorType::Magnetometer(Uuid::new_v4()),
    ];
    let orientation_measurement = SensorType::Other(Uuid::new_v4(), "Orientation".to_string());
    let tag = "Test";

    // Start phyphox mock service
    let sampling_period_millis = 210.0;
    let (handle_phyphox, phyphox) = phyphox_rs::run_service(
        "http://192.168.1.34",
        tag,
        sensor_cluster.clone(),
        sampling_period_millis,
    )
    .unwrap();
    //phyphox_rs::run_mock_service(tag, sensor_cluster.clone(), 50.0, false, 5000).unwrap();

    // start resampler
    let resampling_period_millis = 100.0;
    let resampling_delay_millis = 500.0;
    let (_handle_resampler, resampler) = resampler_rs::run::<SensorReadings<Sample3D>, _>(
        tag,
        sensor_cluster.clone(),
        resampling_period_millis,
        resampling_delay_millis,
        SmothingPolicy::WeightedAverage,
    );

    // start ahrs
    let ahrs = AHRSFilter::new(
        tag,
        sensor_cluster.clone(),
        orientation_measurement.clone(),
        resampling_period_millis,
    )
    .unwrap();

    // connect resampler to phyphox
    resampler
        .attach_listeners(&*phyphox, &sensor_cluster)
        .await
        .unwrap();

    // connect ahrs to phyphox
    ahrs.attach_listeners(&*resampler, &sensor_cluster)
        .await
        .unwrap();
    // connect samples from ahrs

    IMUSink::<SensorReadings<SampleQuaternion>, SampleQuaternion>::attach_listeners(
        &plot_3d,
        &ahrs,
        &[orientation_measurement.clone()],
    )
    .await
    .unwrap();

    let timeout_duration = Duration::from_secs(500);
    let _ = tokio::time::timeout(timeout_duration, async {
        handle_phyphox.await.unwrap();
    })
    .await;
}

```
