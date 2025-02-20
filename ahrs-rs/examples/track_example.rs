use uuid::Uuid;

use common::traits::IMUSink;
use common::types::sensors::{SensorReadings, SensorType};
use common::types::timed::{Sample3D, SampleQuaternion};

use ahrs_rs::{self, AHRSFilter};
use phyphox_rs;
use resampler_rs::{self, SmothingPolicy};
use std::sync::Arc;
use test_utils::renderable::Box3D;
use test_utils::sinks::{MockValue, Plot3D, SinkMock};
use tokio::time::Duration;
use tokio::{select, signal};

fn process_samples(
    _value: MockValue,
    sensor_type: SensorType,
    samples: Arc<SensorReadings<SampleQuaternion>>,
) {
    assert_eq!(sensor_type, samples.get_sensor_type());
    println!("{:?}", samples);
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
    let resampling_period_millis = 50.0;
    let tag = "Test";

    // Start phyphox mock service
    let (handle_phyphox, phyphox) =
        phyphox_rs::run_service("http://192.168.1.34", tag, sensor_cluster.clone(), 10.0).unwrap();
    //phyphox_rs::run_mock_service(tag, sensor_cluster.clone(), 50.0, false, 5000).unwrap();

    // start resampler
    let (handle_resampler, resampler) = resampler_rs::run::<SensorReadings<Sample3D>, _>(
        tag,
        sensor_cluster.clone(),
        resampling_period_millis,
        30,
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

    /*
    // connect samples from ahrs
    let mut sink = SinkMock::<SampleQuaternion>::new();
    sink.register_callback(process_samples);
    sink.attach_listeners(&ahrs, &[orientation_measurement.clone()])
        .await
        .unwrap();
    */
    /*
    let _ = select! {
        _ = signal::ctrl_c() => {
            println!("Received Ctrl+C, shutting down...");
        }
        _ = async {
            handle_resampler.await.unwrap();
            handle_phyphox.await.unwrap();
        } => {}
    };
    */
    // Timeout duration: 5 seconds
    let timeout_duration = Duration::from_secs(5);
    // Use timeout to abort test after 5 seconds
    let _ = tokio::time::timeout(timeout_duration, async {
        // Await for both services to finish
        handle_resampler.await.unwrap();
        handle_phyphox.await.unwrap();
    })
    .await;
}
