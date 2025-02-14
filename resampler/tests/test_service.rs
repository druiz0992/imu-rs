use resampler_rs::run;
use std::sync::Arc;
use uuid::Uuid;

use common::traits::IMUSink;
use common::types::sensors::{SensorReadings, SensorType};
use common::types::timed::Sample3D;
use phyphox_rs::services;
use resampler_rs::ResamplePolicy;
use test_utils::sink_mock::{MockValue, SinkMock};
use tokio::time::Duration;

fn process_samples(
    _value: MockValue,
    sensor_type: SensorType,
    samples: Arc<SensorReadings<Sample3D>>,
) {
    assert_eq!(sensor_type, samples.get_sensor_type());
    assert!(!matches!(sensor_type, SensorType::Gyroscope(_)));
    assert!(
        !matches!(samples.get_sensor_type(), SensorType::Other(_, _)),
        "Unexpected SensorType::Other"
    );
}

#[tokio::test]
async fn test1() {
    let sensor_tag = "Test";
    // Sample update frequency
    let update_period_millis = 20;
    // Sample generation period
    let capture_sampling_period_millis = 10;
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
        capture_sampling_period_millis,
        add_sensor_noise,
        run_for_millis,
    )
    .unwrap();

    // start resampler
    let resampling_period_millis = 100;
    let resampling_policy = Some(ResamplePolicy::WeightedAverage);
    let (handle_resampler, resampler) = run::<SensorReadings<Sample3D>, _>(
        sensor_tag,
        sensor_cluster,
        resampling_period_millis,
        resampling_policy,
    );

    // connect resampler to phyphox
    resampler
        .attach_listener(&*phyphox, &SensorType::Accelerometer(acc_id))
        .await
        .unwrap();
    resampler
        .attach_listener(&*phyphox, &SensorType::Magnetometer(mag_id))
        .await
        .unwrap();
    resampler
        .attach_listener(&*phyphox, &SensorType::Gyroscope(gyro_id))
        .await
        .unwrap();

    // connect samples from resampler
    let mut sink = SinkMock::new();
    sink.register_callback(process_samples);
    sink.attach_listener(&*resampler, &SensorType::Accelerometer(acc_id))
        .await
        .unwrap();
    sink.attach_listener(&*resampler, &SensorType::Magnetometer(mag_id))
        .await
        .unwrap();

    // Timeout duration: 5 seconds
    let timeout_duration = Duration::from_secs(2);

    // Use timeout to abort test after 5 seconds
    let _ = tokio::time::timeout(timeout_duration, async {
        // Await for both services to finish
        handle_resampler.await.unwrap();
        handle_phyphox.await.unwrap();
    })
    .await;
}
