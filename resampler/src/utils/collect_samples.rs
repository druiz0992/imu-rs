use common::traits::{IMUReadings, IMUSample, IMUUntimedSample};
const SAMPLES_TILL_TIMESTAMP_SAFEGUARD_FACTOR: f64 = 2.0;

pub(crate) async fn collect_samples<T, S>(
    sensor_buffer: &mut T,
    timestamp_now_secs: f64,
    period_secs: f64,
) where
    S: IMUSample,
    T: Send + Sync + IMUReadings<S> + 'static,
    S::Untimed: IMUUntimedSample,
{
    let filtered_data: Vec<S> = sensor_buffer
        .get_samples()
        .into_iter()
        .filter_map(|sample| {
            let sample_timestamp = sample.get_timestamp();
            if sample_timestamp <= timestamp_now_secs
                && sample_timestamp
                    >= timestamp_now_secs - period_secs * SAMPLES_TILL_TIMESTAMP_SAFEGUARD_FACTOR
            {
                Some(sample)
            } else {
                None
            }
        })
        .collect();
    sensor_buffer.clear();
    sensor_buffer.extend(filtered_data);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ResamplerPipeline;
    use common::traits::IMUReadings;
    use common::types::sensors::{SensorReadings, SensorType};
    use common::types::timed::Sample3D;
    use uuid::Uuid;

    #[tokio::test]
    async fn test_samples_till_timestamp_no_samples() {
        let acc_id = Uuid::new_v4();
        let sensor_cluster = vec![SensorType::Accelerometer(acc_id)];
        let pipeline =
            ResamplerPipeline::<SensorReadings<Sample3D>, _>::new("test", sensor_cluster);

        let samples = pipeline.collect_samples_till_timestamp(1000.0, 100.0).await;
        assert!(samples[0].get_samples().is_empty());
    }

    #[tokio::test]
    async fn test_collect_samples_within_period() {
        let acc_id = Uuid::new_v4();
        let sensor_cluster = SensorType::Accelerometer(acc_id);
        let mut readings = SensorReadings::new("Test", sensor_cluster);
        let sample = Sample3D::new(950.0, [1.0, 2.0, 3.0]);
        readings.add_sample(sample.clone());

        collect_samples(&mut readings, 1000.0, 100.0).await;

        assert_eq!(readings.get_samples().len(), 1);
        assert_eq!(readings.get_samples()[0], sample);
    }

    #[tokio::test]
    async fn test_samples_till_timestamp_outside_period() {
        let acc_id = Uuid::new_v4();
        let sensor_cluster = SensorType::Accelerometer(acc_id);
        let mut readings = SensorReadings::new("Test", sensor_cluster);
        let sample = Sample3D::new(450.0, [1.0, 2.0, 3.0]);
        readings.add_sample(sample.clone());

        collect_samples(&mut readings, 1000.0, 100.0).await;

        assert!(readings.get_samples().is_empty());
    }

    #[tokio::test]
    async fn test_samples_till_timestamp_mixed_samples() {
        let acc_id = Uuid::new_v4();
        let sensor_cluster = SensorType::Accelerometer(acc_id);
        let mut readings = SensorReadings::new("Test", sensor_cluster);
        let sample_beyond = Sample3D::new(1050.0, [1.0, 2.0, 3.0]);
        let sample_within = Sample3D::new(950.0, [1.0, 2.0, 3.0]);
        let sample_outside = Sample3D::new(700.0, [4.0, 5.0, 6.0]);
        readings.extend(vec![
            sample_beyond.clone(),
            sample_within.clone(),
            sample_outside.clone(),
        ]);

        collect_samples(&mut readings, 1000.0, 100.0).await;

        assert_eq!(readings.get_samples().len(), 1);
        assert_eq!(readings.get_samples()[0], sample_within);
    }
}
