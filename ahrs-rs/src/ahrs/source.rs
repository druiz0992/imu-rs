use async_trait::async_trait;
use std::sync::Arc;
use uuid::Uuid;

use super::AHRSFilter;
use common::traits::{IMUSource, Notifiable};
use common::types::sensors::{SensorReadings, SensorType};
use common::types::timed::SampleQuaternion;

#[async_trait]
impl IMUSource<SensorReadings<SampleQuaternion>, SampleQuaternion> for AHRSFilter {
    fn get_tag(&self) -> &str {
        self.tag.as_str()
    }

    async fn get_available_sensors(&self) -> Result<Vec<SensorType>, String> {
        Ok(self.publishers.get_available_publisher_types())
    }

    async fn unregister_listener(&self, id: Uuid) {
        let _ = self.publishers.remove_listener(id);
    }

    async fn register_listener(
        &self,
        listener: &mut dyn Notifiable<SensorReadings<SampleQuaternion>>,
        sensor_type: &SensorType,
    ) -> Result<Uuid, String> {
        self.publishers.add_listener(listener, sensor_type)
    }

    async fn notify_listeners(
        &self,
        sensor_type: SensorType,
        data: Arc<SensorReadings<SampleQuaternion>>,
    ) {
        self.publishers.notify_listeners(sensor_type, data).await
    }
}
