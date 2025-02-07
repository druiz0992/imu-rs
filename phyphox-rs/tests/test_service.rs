use common::{IMUReadings, IMUSample, Sample3D, SensorReadings, SensorType};
use phyphox_rs::services;
use publisher::Listener;
use std::{sync::Arc, time::Duration};
use tokio::sync::Mutex;

#[tokio::test]
async fn test_receive_accelerometer_samples() {
    let sensor_tag = "Test";
    let period_update_millis = 200;
    let capture_sampling_period_secs = 0.1;
    let add_sensor_noise = false;
    let run_for_millis = 5000;
    let received_samples: Arc<Mutex<Vec<Sample3D>>> = Arc::new(Mutex::new(Vec::new()));

    // Start phyphox mock service
    let (handle, phyphox) = services::run_mock_service::<SensorReadings<Sample3D>, _>(
        sensor_tag,
        period_update_millis,
        capture_sampling_period_secs,
        add_sensor_noise,
        run_for_millis,
    )
    .unwrap();

    // create listener handler
    let listener = {
        let received_samples = received_samples.clone();
        Listener::new(move |value: Arc<SensorReadings<Sample3D>>| {
            let buffer = received_samples.clone();
            async move {
                let mut buffer_lock = buffer.lock().await;
                let tag = value.get_sensor_tag();
                let sensor_type = value.get_sensor_type();
                assert_eq!(tag, "Test");
                assert_eq!(*sensor_type, SensorType::Accelerometer);
                buffer_lock.extend(value.into_iter_samples());
            }
        })
    };

    // install handler
    phyphox
        .register_listener(listener, SensorType::Accelerometer)
        .await;

    handle.await.unwrap();

    // check that samples were received by handler
    let buffer = received_samples.lock().await;
    assert!(buffer.len() > 0);
}

#[tokio::test]
async fn test_receive_multiple_sensors() {
    let sensor_tag = "Test";
    let period_update_millis = 200;
    let capture_sampling_period_secs = 0.1;
    let add_sensor_noise = false;
    let run_for_millis = 5000;
    let received_samples: Arc<Mutex<Vec<Vec<Sample3D>>>> =
        Arc::new(Mutex::new(vec![vec![], vec![]]));

    // Start phyphox mock service
    let (handle, phyphox) = services::run_mock_service::<SensorReadings<Sample3D>, _>(
        sensor_tag,
        period_update_millis,
        capture_sampling_period_secs,
        add_sensor_noise,
        run_for_millis,
    )
    .unwrap();

    // create listener handler
    let listener = {
        let received_samples = received_samples.clone();
        Listener::new(move |value: Arc<SensorReadings<Sample3D>>| {
            let buffer = received_samples.clone();
            async move {
                let mut buffer_lock = buffer.lock().await;
                let tag = value.get_sensor_tag();
                let sensor_type = value.get_sensor_type();
                let sensor_idx = usize::from(sensor_type);
                assert_eq!(tag, "Test");
                if sensor_idx == 0 {
                    assert_eq!(*sensor_type, SensorType::Accelerometer);
                } else {
                    assert_eq!(*sensor_type, SensorType::Gyroscope);
                }
                buffer_lock[usize::from(sensor_type)].extend(value.into_iter_samples());
            }
        })
    };

    // install handlers
    phyphox
        .register_listener(listener.clone(), SensorType::Accelerometer)
        .await;
    phyphox
        .register_listener(listener.clone(), SensorType::Gyroscope)
        .await;

    handle.await.unwrap();

    // check that samples were received by handler
    let buffer = received_samples.lock().await;
    assert!(buffer[0].len() > 0);
    assert!(buffer[1].len() > 0);
    assert!(buffer[0] != buffer[1])
}

#[tokio::test]
async fn test_stop_receiving_accelerometer_samples() {
    let sensor_tag = "Test";
    let period_update_millis = 200;
    let capture_sampling_period_secs = 0.1;
    let add_sensor_noise = false;
    let run_for_millis = 5000;
    let received_samples: Arc<Mutex<Vec<Sample3D>>> = Arc::new(Mutex::new(Vec::new()));

    // Start phyphox mock service
    let (handle, phyphox) = services::run_mock_service::<SensorReadings<Sample3D>, _>(
        sensor_tag,
        period_update_millis,
        capture_sampling_period_secs,
        add_sensor_noise,
        run_for_millis,
    )
    .unwrap();

    // create listener handler
    let listener = {
        let received_samples = received_samples.clone();
        Listener::new(move |value: Arc<SensorReadings<Sample3D>>| {
            let buffer = received_samples.clone();
            async move {
                let mut buffer_lock = buffer.lock().await;
                let tag = value.get_sensor_tag();
                let sensor_type = value.get_sensor_type();
                assert_eq!(tag, "Test");
                assert_eq!(*sensor_type, SensorType::Accelerometer);
                buffer_lock.extend(value.into_iter_samples());
            }
        })
    };

    // install handler
    let id = phyphox
        .register_listener(listener, SensorType::Accelerometer)
        .await;

    // wait 2 seconds and unregister handler
    tokio::time::sleep(Duration::from_millis(2000)).await;
    phyphox
        .unregister_listener(id, SensorType::Accelerometer)
        .await;

    handle.await.unwrap();

    // check that there are no samples received after handler was unregistered
    let buffer = received_samples.lock().await;
    let samples = buffer.clone();
    assert!(samples.len() > 0);
    for sample in samples {
        let timestamp = sample.get_timestamp();
        assert!(timestamp < 3.0);
    }
}
