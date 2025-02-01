use std::sync::Arc;
use tokio::sync::Mutex;

use common::{IMUSample, IMUEvent};

pub(crate) struct Sample([f64; 4]);

impl IMUSample for Sample {
    fn get_sample_data(&self) -> Vec<f64> {
        vec![self.0[1], self.0[2], self.0[3]]
    }

    fn get_timestamp(&self) -> f64 {
        self.0[0]
    }
}

/// A buffer to store sensor common, supporting concurrent access and filtering based on timestamps.
pub(crate) struct Buffer {
    buffer: Arc<Mutex<Vec<Vec<Arc<dyn IMUSample>>>>>,
}

impl Buffer {
    /// Creates a new buffer with `n_buffers` separate internal buffers.
    /// Returns a new `Buffer` instance.
    pub(crate) fn new(n_buffers: usize) -> Self {
        let mut buffers: Vec<Vec<Arc<dyn IMUSample>>> = Vec::with_capacity(n_buffers);
        for _ in 0..n_buffers {
            buffers.push(Vec::new());
        }

        Buffer {
            buffer: Arc::new(Mutex::new(buffers)),
        }
    }

    /// Handles a notification by adding the common from the given `MeasurementProducer`
    /// to the appropriate internal buffer based on sensor type.
    /// Returns nothing.
    ///
    /// # Errors
    /// This function requires the `buffer` mutex to be acquired. If locking fails, this function may panic.
    pub(crate) async fn handle_notification(&self, measurement: Arc<dyn IMUEvent>) {
        let mut buffer = self.buffer.lock().await;
        let sensor_type = measurement.get_sensor_type();
        let samples = measurement.get_samples();
        buffer[sensor_type].extend(samples.iter().cloned());
    }

    /// Retrieves and removes all samples from each buffer that have timestamps less than or equal to `timestamp`.
    /// Returns a vector of vectors, where each sub-vector contains samples from the corresponding buffer.
    ///
    /// # Errors
    /// This function requires the `buffer` mutex to be acquired. If locking fails, this function may panic.
    pub(crate) async fn samples_till_timestamp(
        &self,
        timestamp: f64,
        period: f64,
    ) -> Vec<Vec<[f64; 4]>> {
        // Assumption is that we read samples within the timestamp limits, and the rest will be discarded
        // Earlier samples are too old. There shouldnt be any samples past timestamp.

        let buffer_clone = {
            let mut buffer = self.buffer.lock().await;
            let buffer_clone = buffer.clone();
            for b in buffer.iter_mut() {
                b.clear();
            }
            buffer_clone
        };

        let mut result = vec![Vec::new(); buffer_clone.len()];

        for (i, sensor_buffer) in buffer_clone.iter().enumerate() {
            result[i] = sensor_buffer
                .iter()
                .filter_map(|sample| {
                    let sample_timestamp = sample.get_timestamp();
                    let sample_data = sample.get_sample_data();
                    if sample_timestamp <= timestamp
                        && sample_timestamp >= timestamp - period * 10.0
                    {
                        Some([
                            sample_timestamp,
                            sample_data[0],
                            sample_data[1],
                            sample_data[2],
                        ])
                    } else {
                        None
                    }
                })
                .collect();
        }

        result
    }
}
