use phyphox_rs::services;
use publisher::AsyncListener;
use std::collections::HashMap;
use std::{sync::Arc, time::Duration};
use tokio::sync::Mutex;
use uuid::Uuid;

use common::traits::{IMUReadings, IMUSample, IMUSink, IMUSource};
use common::types::sensors::sensor_type;
use common::types::sensors::{SensorReadings, SensorType};
use common::types::timed::Sample3D;
use common::types::Clock;
use test_utils::sinks::{MockValue, SinkMock};

fn process_samples(
    value: MockValue,
    sensor_type: SensorType,
    samples: Arc<SensorReadings<Sample3D>>,
) {
    assert!(sensor_type == samples.get_sensor_type());
    assert!(!matches!(sensor_type, SensorType::Gyroscope(_)));
    assert!(
        !matches!(samples.get_sensor_type(), SensorType::Other(_, _)),
        "Unexpected SensorType::Other"
    );
    if let MockValue::Float(timestamp_at_boot_secs) = value {
        if usize::from(sensor_type) < sensor_type::MAGNETOMETER_OFFSET {
            for sample in samples.get_samples() {
                assert!(sample.get_timestamp_secs() < timestamp_at_boot_secs + 0.003);
            }
        }
    }
}

#[tokio::test]
async fn test_receive_accelerometer_samples() {
    let sensor_tag = "Test";
    let update_period_millis = 200.0;
    let add_sensor_noise = false;
    let run_for_millis = 5000;
    let received_samples: Arc<Mutex<Vec<Sample3D>>> = Arc::new(Mutex::new(Vec::new()));
    let received_id = Arc::new(Mutex::new(Uuid::new_v4()));
    let acc_id = Uuid::new_v4();
    let gyro_id = Uuid::new_v4();
    let mag_id = Uuid::new_v4();
    let sensor_cluster = vec![
        SensorType::Accelerometer(acc_id),
        SensorType::Gyroscope(gyro_id),
        SensorType::Magnetometer(mag_id),
    ];

    // Start phyphox mock service
    let (handle, phyphox) = services::run_mock_service(
        sensor_tag,
        sensor_cluster,
        update_period_millis,
        add_sensor_noise,
        run_for_millis,
    )
    .unwrap();

    assert_eq!(phyphox.get_tag(), sensor_tag);

    // create listener handler
    let mut listener = {
        let received_samples = received_samples.clone();
        let received_id = received_id.clone();
        AsyncListener::new(move |id: Uuid, value: Arc<SensorReadings<Sample3D>>| {
            let buffer = received_samples.clone();
            let received_id = received_id.clone();
            async move {
                let mut buffer_lock = buffer.lock().await;
                let mut received_id_lock = received_id.lock().await;
                *received_id_lock = id;
                let tag = value.get_sensor_tag();
                assert_eq!(tag, "Test");
                buffer_lock.extend(value.get_samples().into_iter());
            }
        })
    };

    // install handler
    let expected_id = phyphox
        .register_listener(&mut listener, &SensorType::Accelerometer(acc_id))
        .await
        .unwrap();

    handle.await.unwrap();

    // check that samples were received by handler
    let buffer = received_samples.lock().await;
    let received_id = received_id.lock().await;
    assert!(buffer.len() > 0);
    // the listener receives an id as part of the callback, which can match to which listener function it received
    assert!(expected_id == *received_id)
}

#[tokio::test]
async fn test_receive_multiple_sensors() {
    let sensor_tag = "Test";
    let update_period_millis = 200.0;
    let add_sensor_noise = false;
    let run_for_millis = 5000;
    let received_samples: Arc<Mutex<HashMap<Uuid, Vec<Sample3D>>>> =
        Arc::new(Mutex::new(HashMap::new()));
    let acc_id = Uuid::new_v4();
    let gyro_id = Uuid::new_v4();
    let mag_id = Uuid::new_v4();
    let sensor_cluster = vec![
        SensorType::Accelerometer(acc_id),
        SensorType::Gyroscope(gyro_id),
        SensorType::Magnetometer(mag_id),
    ];

    // Start phyphox mock service
    let (handle, phyphox) = services::run_mock_service(
        sensor_tag,
        sensor_cluster,
        update_period_millis,
        add_sensor_noise,
        run_for_millis,
    )
    .unwrap();

    assert_eq!(
        phyphox.get_available_sensors().await.unwrap(),
        vec![
            SensorType::Accelerometer(acc_id),
            SensorType::Gyroscope(gyro_id),
            SensorType::Magnetometer(mag_id)
        ]
    );

    // create listener handler
    let listener = {
        let received_samples = received_samples.clone();
        AsyncListener::new(move |id: Uuid, value: Arc<SensorReadings<Sample3D>>| {
            let storage = received_samples.clone();
            async move {
                let mut storage_lock = storage.lock().await;
                let tag = value.get_sensor_tag();
                assert_eq!(tag, "Test");
                storage_lock
                    .entry(id)
                    .or_insert_with(Vec::new)
                    .extend(value.get_samples().into_iter());
            }
        })
    };

    // install handlers
    let accel_id = phyphox
        .register_listener(&mut listener.clone(), &SensorType::Accelerometer(acc_id))
        .await
        .unwrap();
    let gyro_id = phyphox
        .register_listener(&mut listener.clone(), &SensorType::Gyroscope(gyro_id))
        .await
        .unwrap();

    handle.await.unwrap();

    // check that samples were received by handler
    let buffer = received_samples.lock().await;
    let accel_samples = buffer.get(&accel_id).unwrap();
    let gyro_samples = buffer.get(&gyro_id).unwrap();
    assert!(!accel_samples.is_empty());
    assert!(!gyro_samples.is_empty());
}

#[tokio::test]
async fn test_stop_receiving_accelerometer_samples() {
    let sensor_tag = "Test";
    let update_period_millis = 200.0;
    let add_sensor_noise = false;
    let run_for_millis = 5000;
    let received_samples: Arc<Mutex<Vec<Sample3D>>> = Arc::new(Mutex::new(Vec::new()));
    let acc_id = Uuid::new_v4();
    let gyro_id = Uuid::new_v4();
    let mag_id = Uuid::new_v4();
    let sensor_cluster = vec![
        SensorType::Accelerometer(acc_id),
        SensorType::Gyroscope(gyro_id),
        SensorType::Magnetometer(mag_id),
    ];
    let timestamp_at_boot_secs = Clock::now().as_secs();

    // Start phyphox mock service
    let (handle, phyphox) = services::run_mock_service(
        sensor_tag,
        sensor_cluster,
        update_period_millis,
        add_sensor_noise,
        run_for_millis,
    )
    .unwrap();

    // create listener handler
    let mut listener = {
        let received_samples = received_samples.clone();
        AsyncListener::new(move |_id: Uuid, value: Arc<SensorReadings<Sample3D>>| {
            let buffer = received_samples.clone();
            async move {
                let mut buffer_lock = buffer.lock().await;
                let tag = value.get_sensor_tag();
                assert_eq!(tag, "Test");
                buffer_lock.extend(value.get_samples().into_iter());
            }
        })
    };

    // install handler
    let id = phyphox
        .register_listener(&mut listener, &SensorType::Accelerometer(acc_id))
        .await
        .unwrap();

    // wait 2 seconds and unregister handler
    tokio::time::sleep(Duration::from_millis(2000)).await;
    phyphox.unregister_listener(id).await;

    handle.await.unwrap();

    // check that there are no samples received after handler was unregistered
    let buffer = received_samples.lock().await;
    let samples = buffer.clone();
    assert!(!samples.is_empty());
    for sample in samples {
        let timestamp = sample.get_timestamp_secs();
        assert!(timestamp < 3.0 + timestamp_at_boot_secs);
    }
}

#[tokio::test]
async fn test_sink() {
    let sensor_tag = "Test";
    let update_period_millis = 200.0;
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
    let (handle, phyphox) = services::run_mock_service(
        sensor_tag,
        sensor_cluster.clone(),
        update_period_millis,
        add_sensor_noise,
        run_for_millis,
    )
    .unwrap();

    let mut sink = SinkMock::new();
    sink.set_value(MockValue::Float(Clock::now().as_secs()));
    sink.register_callback(process_samples);

    let ids = sink
        .attach_listeners(&*phyphox, &sensor_cluster)
        .await
        .unwrap();

    tokio::time::sleep(Duration::from_millis(2000)).await;
    sink.detach_listener(&*phyphox, ids[0]).await;

    handle.await.unwrap();
}
