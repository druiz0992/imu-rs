use resampler_rs::run;
use std::sync::Arc;
use uuid::Uuid;

use common::traits::IMUSink;
use common::types::sensors::{SensorReadings, SensorType};
use common::types::timed::Sample3D;
use phyphox_rs::services;
use resampler_rs::SmothingPolicy;
use test_utils::sinks::{MockValue, SinkMock};
use tokio::time::Duration;

fn process_samples(
    _value: MockValue,
    sensor_type: SensorType,
    samples: Arc<SensorReadings<Sample3D>>,
) {
    assert_eq!(sensor_type, samples.get_sensor_type());
    assert!(
        !matches!(samples.get_sensor_type(), SensorType::Other(_, _)),
        "Unexpected SensorType::Other"
    );
}

#[tokio::test]
async fn test_resampler_pipeline() {
    let sensor_tag = "Test";
    // Sample update frequency
    let update_period_millis = 50.0;
    let add_sensor_noise = false;
    let run_for_millis = 5000;
    let acc_id = Uuid::new_v4();
    let gyro_id = Uuid::new_v4();
    let mag_id = Uuid::new_v4();
    let sensor_cluster = vec![
        SensorType::Accelerometer(acc_id),
        SensorType::Gyroscope(gyro_id),
        SensorType::Magnetometer(mag_id),
    ];

    // Start phyphox mock service
    let (handle_phyphox, phyphox) = services::run_mock_service(
        sensor_tag,
        sensor_cluster.clone(),
        update_period_millis,
        add_sensor_noise,
        run_for_millis,
    )
    .unwrap();

    // start resampler
    let resampling_period_millis = 10.0;
    let resampling_delay_millis = 200.0;
    let resampling_policy = SmothingPolicy::WeightedAverage;
    let (_handle_resampler, resampler) = run::<SensorReadings<Sample3D>, _>(
        sensor_tag,
        sensor_cluster.clone(),
        resampling_period_millis,
        resampling_delay_millis,
        resampling_policy,
    );

    // connect resampler to phyphox
    resampler
        .attach_listeners(&*phyphox, &sensor_cluster)
        .unwrap();

    // connect samples from resampler
    let mut sink = SinkMock::new();
    sink.register_callback(process_samples);
    sink.attach_listeners(&*resampler, &sensor_cluster).unwrap();

    // Timeout duration: 5 seconds
    let timeout_duration = Duration::from_secs(1);

    // Use timeout to abort test after 5 seconds
    let _ = tokio::time::timeout(timeout_duration, async {
        // Await for both services to finish
        handle_phyphox.await.unwrap();
    })
    .await;
}
