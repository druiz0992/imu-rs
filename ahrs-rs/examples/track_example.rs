use uuid::Uuid;

use common::traits::{IMUReadings, IMUSample, IMUSink};
use common::types::sensors::{SensorReadings, SensorType};
use common::types::timed::{Sample3D, SampleQuaternion};

use ahrs_rs::{self, AHRSFilter};
use common::types::clock::Clock;
use phyphox_rs;
use resampler_rs::{self, SmothingPolicy};
use std::sync::Arc;
use test_utils::renderable::Box3D;
use test_utils::sinks::{MockValue, Plot3D, SinkMock};
use tokio::time::Duration;

fn process_samples(
    _value: MockValue,
    _sensor_type: SensorType,
    samples: Arc<SensorReadings<SampleQuaternion>>,
) {
    if let Some(q) = samples.get_samples().last() {
        println!(
            "{:?}, {:?}, {}",
            q.get_measurement().inner().euler_angles(),
            q.get_timestamp_secs(),
            Clock::now().as_secs()
        );
    };
}

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
        .unwrap();

    // connect ahrs to phyphox
    ahrs.attach_listeners(&*resampler, &sensor_cluster).unwrap();
    // connect samples from ahrs

    IMUSink::<SensorReadings<SampleQuaternion>, SampleQuaternion>::attach_listeners(
        &plot_3d,
        &ahrs,
        &[orientation_measurement.clone()],
    )
    .unwrap();

    // connect samples from ahrs
    let mut sink = SinkMock::<SampleQuaternion>::new();
    sink.register_callback(process_samples);
    sink.attach_listeners(&ahrs, &[orientation_measurement.clone()])
        .unwrap();

    let timeout_duration = Duration::from_secs(500);
    let _ = tokio::time::timeout(timeout_duration, async {
        handle_phyphox.await.unwrap();
    })
    .await;
}
