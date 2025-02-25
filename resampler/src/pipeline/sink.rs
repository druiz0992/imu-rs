use async_trait::async_trait;
use publisher::listener;
use publisher::Listener;
use std::sync::Arc;
use uuid::Uuid;

use super::ResamplerPipeline;
use common::traits::{IMUFilter, IMUReadings, IMUSample, IMUSink, IMUSource};
use common::types::filters::Average;
use common::types::filters::WeightedAverage;
use common::types::sensors::SensorType;

#[async_trait]
impl<T, S> IMUSink<T, S> for ResamplerPipeline<T, S>
where
    S: Send + Sync + IMUSample + std::fmt::Debug,
    T: Send + Sync + IMUReadings<S> + std::fmt::Debug + 'static,
    Average<S::Untimed>: IMUFilter<S>,
    WeightedAverage<S::Untimed>: IMUFilter<S>,
{
    async fn attach_listeners(
        &self,
        source: &dyn IMUSource<T, S>,
        sensor_cluster: &[SensorType],
    ) -> Result<Vec<Uuid>, String> {
        let mut listener = listener!(self.process_samples);
        let mut ids = Vec::with_capacity(sensor_cluster.len());
        for sensor_type in sensor_cluster {
            if let Ok(id) = source.register_listener(&mut listener, sensor_type).await {
                ids.push(id);
            } else {
                return Err("Incorrect sensor".to_string());
            }
        }
        Ok(ids)
    }

    async fn process_samples(&self, _listener_id: Uuid, samples: Arc<T>) {
        let sensor_type = samples.get_sensor_type();
        if let Some(mutex) = self.buffer.get(&sensor_type) {
            let mut data = mutex.lock().await;
            data.extend(samples.get_samples());
        }
    }
}
