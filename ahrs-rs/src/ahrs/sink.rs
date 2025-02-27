use async_trait::async_trait;
use nalgebra::Vector3;
use publisher::listener;
use publisher::AsyncListener;
use std::sync::Arc;
use uuid::Uuid;

use super::AHRSFilter;
use super::DISCARD_N_INITIAL_SAMPLES;
use common::traits::{IMUReadings, IMUSample, IMUSink, IMUSource};
use common::types::sensors::{SensorReadings, SensorType};
use common::types::timed::Sample3D;

#[async_trait]
impl<T> IMUSink<T, Sample3D> for AHRSFilter
where
    T: Send + Sync + IMUReadings<Sample3D> + 'static,
{
    async fn attach_listeners(
        &self,
        source: &dyn IMUSource<T, Sample3D>,
        sensor_cluster: &[SensorType],
    ) -> Result<Vec<Uuid>, String> {
        let mut listener = listener!(self.process_samples);
        let mut ids = Vec::with_capacity(sensor_cluster.len());
        for sensor_type in sensor_cluster {
            if let Ok(id) = source.register_listener(&mut listener, sensor_type).await {
                ids.push(id)
            } else {
                return Err("Incorrect sensor".to_string());
            }
        }
        Ok(ids)
    }

    async fn process_samples(&self, _listener_id: Uuid, samples: Arc<T>) {
        // Copy sample to receiving buffer
        let sensor_type = samples.get_sensor_type();
        if let Some(rx_samples) = samples.get_samples().first() {
            let mut ahrs_lock = self.filter.lock().await;
            ahrs_lock.buffer.set_samples_by_type(
                &sensor_type,
                Vector3::from_vec(rx_samples.get_measurement().inner().to_vec()),
            );
            ahrs_lock
                .buffer
                .set_timestamp(rx_samples.get_timestamp_secs());

            if ahrs_lock.buffer.samples_ready() {
                let buffer_clone = ahrs_lock.clone_and_clear().await;
                let q = ahrs_lock.update_filter(buffer_clone).await;
                let mut readings = SensorReadings::new(&self.tag, self.new_measurement.clone());
                if ahrs_lock.n_samples > DISCARD_N_INITIAL_SAMPLES {
                    readings.add_sample(q.clone());
                    self.publishers
                        .notify_listeners(self.new_measurement.clone(), Arc::new(readings))
                        .await;
                }
            }
            drop(ahrs_lock);
        }
    }
}
