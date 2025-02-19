use async_trait::async_trait;
use std::sync::Arc;
use uuid::Uuid;

use crate::ResamplerPipeline;
use common::traits::{IMUFilter, IMUReadings, IMUSample, IMUSource, IMUUntimedSample, Notifiable};
use common::types::filters::Average;
use common::types::filters::WeightedAverage;
use common::types::sensors::SensorType;

#[async_trait]
impl<T, S> IMUSource<T, S> for ResamplerPipeline<T, S>
where
    S: IMUSample,
    T: Send + Sync + IMUReadings<S> + 'static,
    S::Untimed: IMUUntimedSample,
    Average<S::Untimed>: IMUFilter<S>,
    WeightedAverage<S::Untimed>: IMUFilter<S>,
{
    fn get_tag(&self) -> &str {
        self.tag.as_str()
    }

    async fn get_available_sensors(&self) -> Result<Vec<SensorType>, String> {
        Ok(self.publishers.get_available_publisher_types().await)
    }

    async fn unregister_listener(&self, id: Uuid) {
        let _ = self.publishers.remove_listener(id).await;
    }

    async fn register_listener(
        &self,
        listener: &mut dyn Notifiable<T>,
        sensor_type: &SensorType,
    ) -> Result<Uuid, String> {
        self.publishers.add_listener(listener, sensor_type).await
    }

    async fn notify_listeners(&self, sensor_type: SensorType, data: Arc<T>) {
        self.publishers.notify_listeners(sensor_type, data).await
    }
}
