use async_trait::async_trait;
use nalgebra::Vector3;
use publisher::listener;
use publisher::Listener;
use std::sync::Arc;
use uuid::Uuid;

use super::AHRSFilter;
use common::traits::{IMUReadings, IMUSample, IMUSink, IMUSource};
use common::types::sensors::SensorType;
use common::types::timed::Sample3D;

#[async_trait]
impl<T> IMUSink<T, Sample3D> for AHRSFilter
where
    T: Send + Sync + IMUReadings<Sample3D> + 'static,
{
    async fn attach_listener(
        &self,
        source: &dyn IMUSource<T, Sample3D>,
        sensor_type: &SensorType,
    ) -> Result<Uuid, String> {
        let mut listener = listener!(self.process_samples);
        source.register_listener(&mut listener, sensor_type).await
    }

    async fn process_samples(&self, _listener_id: Uuid, samples: Arc<T>) {
        // Copy sample to receiving buffer
        let sensor_type = samples.get_sensor_type();
        if let Some(rx_samples) = samples.get_samples().first() {
            let mut buffer_lock = self.buffer.lock().await;
            buffer_lock.set_samples_by_type(
                &sensor_type,
                Vector3::from_vec(rx_samples.get_measurement().inner().to_vec()),
            );
            buffer_lock.set_timestamp(rx_samples.get_timestamp_secs());

            if buffer_lock.samples_ready() {
                let _ = self.update_filter().await;
            }
        }
    }
}
