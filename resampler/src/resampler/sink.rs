use async_trait::async_trait;
use publisher::listener;
use publisher::Listener;
use std::sync::Arc;
use uuid::Uuid;

use super::Resampler;
use common::traits::{IMUFilter, IMUReadings, IMUSample, IMUSink, IMUSource};
use common::types::filters::Average;
use common::types::filters::WeightedAverage;
use common::types::sensors::SensorType;

#[async_trait]
impl<T, S> IMUSink<T, S> for Resampler<T, S>
where
    S: Send + Sync + IMUSample,
    T: Send + Sync + IMUReadings<S> + 'static,
    Average<S::Untimed>: IMUFilter<S>,
    WeightedAverage<S::Untimed>: IMUFilter<S>,
{
    async fn attach_listener(
        &self,
        source: &dyn IMUSource<T, S>,
        sensor_type: &SensorType,
    ) -> Result<Uuid, String> {
        let mut listener = listener!(self.process_samples);
        source.register_listener(&mut listener, sensor_type).await
    }

    async fn process_samples(&self, _listener_id: Uuid, samples: Arc<T>) {
        let sensor_type = samples.get_sensor_type();
        if let Some(mutex) = self.buffer.get(&sensor_type) {
            let mut data = mutex.lock().await;
            data.extend(samples.get_samples());
        }
    }
}
