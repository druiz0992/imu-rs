use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use uuid::Uuid;

use common::traits::{IMUSink, IMUSource};
use common::types::sensors::{SensorReadings, SensorType};
use common::types::timed::Sample3D;
use publisher::listener;
use publisher::Listener;

type MockCallback =
    Arc<Option<Arc<dyn Fn(MockValue, SensorType, Arc<SensorReadings<Sample3D>>) + Send + Sync>>>;

#[derive(Clone)]
pub enum MockValue {
    Int(i64),
    Float(f64),
    Text(String),
}

impl Default for MockValue {
    fn default() -> Self {
        MockValue::Int(0)
    }
}
#[derive(Clone, Default)]
pub struct SinkMock {
    control: Arc<RwLock<HashMap<Uuid, SensorType>>>,
    callback: MockCallback,
    value: MockValue,
}

impl SinkMock {
    pub fn new() -> Self {
        Self {
            control: Arc::new(RwLock::new(HashMap::new())),
            callback: Arc::new(None),
            value: MockValue::default(),
        }
    }

    pub fn register_callback<F>(&mut self, callback: F)
    where
        F: Fn(MockValue, SensorType, Arc<SensorReadings<Sample3D>>) + Send + Sync + 'static,
    {
        self.callback = Arc::new(Some(Arc::new(callback)));
    }

    pub fn set_value(&mut self, value: MockValue) {
        self.value = value;
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
            if let Some(cb) = self.callback.as_ref() {
                cb(self.value.clone(), sensor_type.clone(), samples);
            }
        }
    }
}

impl std::fmt::Debug for SinkMock {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SinkMock")
            .field("control", &self.control)
            .field("callback", &"<callback_fn>")
            .finish()
    }
}
