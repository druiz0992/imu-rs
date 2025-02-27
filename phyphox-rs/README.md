# phyphox-rs

Wrapper to use Phyphox, an App that allows you to use the sensors in your phone for your experiments. The library collects
accelerometer, gyroscope and magnetometer common from the phone and forwards these common to registered listeners 
for further processing.

In order to use the library, you need to install Phyphox App on your phone, and configure it to collect accelerometer, gyroscope and magnetometer common. Phyphox App needs to be further configured to have remove access enabled so that the library can connect via a REST API to your phone to receive the common.


```rust
    // Create Phyphox Mock service to generate sampled and collect them in a Sink

    use async_trait::async_trait;
    use phyphox_rs::services;
    use publisher::AsyncListener;
    use std::collections::HashMap;
    use std::{sync::Arc, time::Duration};
    use tokio::sync::{Mutex, RwLock};
    use uuid::Uuid;

    use imu_common::traits::{IMUReadings, IMUSample, IMUSink, IMUSource};
    use imu_common::types::sensors::{SensorReadings, SensorType};
    use imu_common::types::timed::Sample3D;
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
            let mut listener = async_listener!(self.process_samples);
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
                // Process sample from sensor_type
            }
        }
    }
    
    #[tokio::main]
    async fn main() {
        let sensor_tag = "Test";
        let update_period_millis = 200;
        let capture_sampling_period_millis = 100;
        let add_sensor_noise = false;
        let run_for_millis = 5000;

        // Start phyphox mock service
        let (handle, phyphox) = services::run_mock_service(
            sensor_tag,
            update_period_millis,
            capture_sampling_period_millis,
            add_sensor_noise,
            run_for_millis,
    )
    .unwrap();

    let sink = SinkMock::new();
    let id_accel = sink
        .attach_listener(&*phyphox, &SensorType::Accelerometer)
        .await
        .unwrap();
    sink.attach_listener(&*phyphox, &SensorType::Magnetometer)
        .await
        .unwrap();

    handle.await.unwrap();
}

```