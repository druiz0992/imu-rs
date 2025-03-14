use std::collections::HashMap;
use std::sync::{Arc, RwLock};
use uuid::Uuid;

use imu_common::traits::{IMUSample, IMUSink, IMUSource};
use imu_common::types::sensors::{SensorReadings, SensorType};
use publisher::{listener, Listener};

type MockAsyncCallback<T> =
    Arc<Option<Arc<dyn Fn(MockValue, SensorType, Arc<SensorReadings<T>>) + Send + Sync>>>;

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
pub struct SinkMock<T> {
    control: Arc<RwLock<HashMap<Uuid, SensorType>>>,
    callback: MockAsyncCallback<T>,
    value: MockValue,
}

impl<T> SinkMock<T>
where
    T: IMUSample,
{
    pub fn new() -> Self {
        Self {
            control: Arc::new(RwLock::new(HashMap::new())),
            callback: Arc::new(None),
            value: MockValue::default(),
        }
    }

    pub fn register_callback<F>(&mut self, callback: F)
    where
        F: Fn(MockValue, SensorType, Arc<SensorReadings<T>>) + Send + Sync + 'static,
    {
        self.callback = Arc::new(Some(Arc::new(callback)));
    }

    pub fn set_value(&mut self, value: MockValue) {
        self.value = value;
    }
}

impl<T> IMUSink<SensorReadings<T>, T> for SinkMock<T>
where
    T: IMUSample,
{
    fn attach_listeners(
        &self,
        source: &dyn IMUSource<SensorReadings<T>, T>,
        sensor_cluster: &[SensorType],
    ) -> Result<Vec<Uuid>, String> {
        let mut listener = listener!(self.process_samples);
        let mut ids = Vec::with_capacity(sensor_cluster.len());
        for sensor_type in sensor_cluster {
            match source.register_listener(&mut listener, sensor_type) {
                Ok(id) => {
                    let mut control = self.control.write().unwrap();
                    control.insert(id, sensor_type.clone());
                    ids.push(id);
                }
                Err(e) => return Err(e),
            }
        }
        Ok(ids)
    }
    fn detach_listener(&self, source: &dyn IMUSource<SensorReadings<T>, T>, id: Uuid) {
        source.unregister_listener(id);
        let mut control = self.control.write().unwrap();
        control.remove_entry(&id);
    }

    fn process_samples(&self, id: Uuid, samples: Arc<SensorReadings<T>>) {
        let control = self.control.read().unwrap();
        if let Some(sensor_type) = control.get(&id) {
            if let Some(cb) = self.callback.as_ref() {
                cb(self.value.clone(), sensor_type.clone(), samples);
            }
        }
    }
}

impl<T> std::fmt::Debug for SinkMock<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SinkMock")
            .field("control", &self.control)
            .field("callback", &"<callback_fn>")
            .finish()
    }
}
