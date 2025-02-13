use async_trait::async_trait;
use phyphox_rs::services;
use publisher::Listener;
use std::collections::HashMap;
use std::{sync::Arc, time::Duration};
use tokio::sync::{Mutex, RwLock};
use uuid::Uuid;

use common::traits::{IMUReadings, IMUSample, IMUSink, IMUSource};
use common::types::sensors::sensor_type;
use common::types::sensors::{SensorReadings, SensorType};
use common::types::timed::Sample3D;
use publisher::listener;

#[derive(Debug, Clone)]
struct SinkMock {
    control: Arc<RwLock<HashMap<Uuid, SensorType>>>,
}

impl SinkMock {
    fn new() -> Self {
        Self {
            control: Arc::new(RwLock::new(HashMap::new())),
        }
    }
}

#[async_trait]
impl IMUSink<SensorReadings<Sample3D>, Sample3D> for SinkMock {
    async fn attach_listener(
        &self,
        source: &dyn IMUSource<SensorReadings<Sample3D>, Sample3D>,
        sensor_type: &SensorType,
    ) -> Result<Uuid, String> {
        let mut listener = listener!(self.process_samples);
        match source.register_listener(&mut listener, sensor_type).await {
            Ok(id) => {
                let mut control = self.control.write().await;
                control.insert(id, sensor_type.clone());
                Ok(id)
            }
            Err(e) => Err(e),
        }
    }
    async fn detach_listener(
        &self,
        source: &dyn IMUSource<SensorReadings<Sample3D>, Sample3D>,
        id: Uuid,
    ) {
        source.unregister_listener(id).await;
        let mut control = self.control.write().await;
        control.remove_entry(&id);
    }

    async fn process_samples(&self, id: Uuid, samples: Arc<SensorReadings<Sample3D>>) {
        let control = self.control.read().await;
        if let Some(sensor_type) = control.get(&id) {
            assert!(sensor_type == &samples.get_sensor_type());
            assert!(!matches!(sensor_type, SensorType::Gyroscope(_)));
            assert!(
                !matches!(samples.get_sensor_type(), SensorType::Other(_, _)),
                "Unexpected SensorType::Other"
            );
            if usize::from(sensor_type) < sensor_type::MAGNETOMETER_OFFSET {
                for sample in samples.get_samples() {
                    assert!(sample.get_timestamp() < 3.0);
                }
            }
        }
    }
}

#[tokio::test]
async fn test_receive_accelerometer_samples() {
    let sensor_tag = "Test";
    let update_period_millis = 200;
    let capture_sampling_period_millis = 100;
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
        capture_sampling_period_millis,
        add_sensor_noise,
        run_for_millis,
    )
    .unwrap();

    assert_eq!(phyphox.get_tag(), sensor_tag);

    // create listener handler
    let mut listener = {
        let received_samples = received_samples.clone();
        let received_id = received_id.clone();
        Listener::new(move |id: Uuid, value: Arc<SensorReadings<Sample3D>>| {
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
    let update_period_millis = 200;
    let capture_sampling_period_millis = 100;
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
        capture_sampling_period_millis,
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
        Listener::new(move |id: Uuid, value: Arc<SensorReadings<Sample3D>>| {
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
    let update_period_millis = 200;
    let capture_sampling_period_millis = 100;
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

    // Start phyphox mock service
    let (handle, phyphox) = services::run_mock_service(
        sensor_tag,
        sensor_cluster,
        update_period_millis,
        capture_sampling_period_millis,
        add_sensor_noise,
        run_for_millis,
    )
    .unwrap();

    // create listener handler
    let mut listener = {
        let received_samples = received_samples.clone();
        Listener::new(move |_id: Uuid, value: Arc<SensorReadings<Sample3D>>| {
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
        let timestamp = sample.get_timestamp();
        assert!(timestamp < 3.0);
    }
}

#[tokio::test]
async fn test_sink() {
    let sensor_tag = "Test";
    let update_period_millis = 200;
    let capture_sampling_period_millis = 100;
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
        sensor_cluster,
        update_period_millis,
        capture_sampling_period_millis,
        add_sensor_noise,
        run_for_millis,
    )
    .unwrap();

    let sink = SinkMock::new();
    let id_accel = sink
        .attach_listener(&*phyphox, &SensorType::Accelerometer(acc_id))
        .await
        .unwrap();
    sink.attach_listener(&*phyphox, &SensorType::Magnetometer(mag_id))
        .await
        .unwrap();

    tokio::time::sleep(Duration::from_millis(2000)).await;
    sink.detach_listener(&*phyphox, id_accel).await;

    handle.await.unwrap();
}
