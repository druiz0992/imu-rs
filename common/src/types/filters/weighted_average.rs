use crate::traits::imu::BasicArithmetic;
use crate::traits::{IMUFilter, IMUSample, IMUUntimedSample};
use crate::types::timed::SampleQuaternion;
use crate::types::untimed::UnitQuaternion;
use std::marker::PhantomData;

/// A weighted moving average filter for IMU data.
///
/// This filter smooths IMU data by applying a weighted moving average algorithm. The weights are
/// determined based on the distance of each sample's timestamp from a specified midpoint, with
/// closer samples having higher weights. The weights are further adjusted using an alpha parameter
/// to control the influence of the distance on the weights.
///
/// # Example
///
/// ```rust
/// use common::types::filters::WeightedAverage;
/// use common::traits::IMUFilter;
/// use common::types::timed::Sample3D;
///
/// // define filter with midpoint at timestamp 5.0
/// let mut filter = WeightedAverage::new(5.0);
///
/// // Create two samples, one with timesamp 1.0, and the other with timestamp 6.0
/// let samples = vec![Sample3D::new(1.0, [1.0, 2.0, 3.0]), Sample3D::new(6.0, [4.3, 3.2, 4.3])];
///
/// // Get smoothe samples
/// let smoothed_samples = filter.filter_batch(samples);
///
/// ```

const WEIGHTED_AVERAGE_EPS: f64 = 1e-10;
const WEIGHTED_AVERAGE_ALPHA: f64 = 0.8;

/// Definition of moving average filter, containing `window_size` elements to do the smoothing.
#[derive(Clone, Debug)]
pub struct WeightedAverage<T> {
    mid_point: f64,
    _phantom_data: PhantomData<T>,
}

impl<T> WeightedAverage<T> {
    /// Initializes new `MovingAverage` filter with `window_size` elements.
    pub fn new(mid_point: f64) -> Self {
        Self {
            mid_point,
            _phantom_data: PhantomData,
        }
    }
}

/// General implementation of IMUFIlter for samples that implement `BasicArithmetic` trait
impl<T, U> IMUFilter<U> for WeightedAverage<T>
where
    T: IMUUntimedSample + BasicArithmetic + Default + Send + Sync + 'static + Clone + Sized,
    U: IMUSample<Untimed = T>,
{
    fn filter_batch(&mut self, samples: Vec<U>) -> Result<Vec<U>, &str> {
        if samples.is_empty() {
            return Err("No samples to filter");
        }
        let mut buffer: Vec<U> = Vec::with_capacity(1);
        let mut total_w = 0.0;
        let mut aggregate = T::default().get_measurement();

        for s in samples {
            let raw_samples = s.get_measurement();
            let sample_timestamp = s.get_timestamp();
            let w = (1.0 / ((sample_timestamp - self.mid_point).abs() + WEIGHTED_AVERAGE_EPS))
                .powf(WEIGHTED_AVERAGE_ALPHA);
            aggregate += raw_samples * w;
            total_w += w;
        }
        aggregate = aggregate / total_w;
        buffer.push(U::from_measurement(self.mid_point, aggregate));
        Ok(buffer)
    }
}

/// Specific implementation of IMUFIlter for quaternion samples
impl IMUFilter<SampleQuaternion> for WeightedAverage<UnitQuaternion>
where
    SampleQuaternion: Sized,
{
    fn filter_batch(
        &mut self,
        samples: Vec<SampleQuaternion>,
    ) -> Result<Vec<SampleQuaternion>, &str> {
        if samples.is_empty() {
            return Err("No samples to filter");
        }
        let mut buffer: Vec<SampleQuaternion> = Vec::with_capacity(1);
        let mut total_w = 0.0;
        let mut aggregate = UnitQuaternion::default().get_measurement().inner();

        for s in samples {
            let raw_sample = s.get_measurement().inner();
            let timestamp = s.get_timestamp();
            let w = (1.0 / ((timestamp - self.mid_point).abs() + WEIGHTED_AVERAGE_EPS))
                .powf(WEIGHTED_AVERAGE_ALPHA);
            aggregate = aggregate.slerp(&raw_sample, w / (total_w + w));
            total_w += w
        }
        buffer.push(SampleQuaternion::from_measurement(
            self.mid_point,
            UnitQuaternion::from_unit_quaternion(aggregate),
        ));
        Ok(buffer)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::timed::sample_3d::Sample3D;
    use crate::types::untimed::XYZ;

    #[test]
    fn test_weighted_moving_average_single_sample() {
        let mut filter = WeightedAverage::new(5.0);
        let samples = vec![Sample3D::new(5.0, [1.0, 2.0, 3.0])];
        let smoothed_samples = filter.filter_batch(samples).unwrap();
        assert_eq!(smoothed_samples.len(), 1);
        assert_eq!(
            smoothed_samples[0].get_measurement(),
            XYZ::new([1.0, 2.0, 3.0])
        );
    }

    #[test]
    fn test_weighted_moving_average_multiple_samples() {
        let eps = 1e-2;
        let mut filter = WeightedAverage::new(5.0);
        let samples = vec![
            Sample3D::new(1.0, [1.0, 2.0, 3.0]),
            Sample3D::new(6.0, [4.0, 5.0, 6.0]),
        ];
        let smoothed_samples = filter.filter_batch(samples).unwrap();
        assert_eq!(smoothed_samples.len(), 1);
        let expected = XYZ::new([3.25, 4.25, 5.25]); // Adjust based on actual weighted average calculation
        assert!(smoothed_samples[0].get_measurement() - expected < XYZ::new([eps, eps, eps]));
    }

    #[test]
    #[should_panic(expected = "No samples to filter")]
    fn test_weighted_moving_average_no_samples() {
        let mut filter = WeightedAverage::new(5.0);
        let samples: Vec<Sample3D> = Vec::new();
        filter.filter_batch(samples).unwrap();
    }

    #[test]
    fn test_weighted_moving_average_quaternion_samples() {
        let mut filter = WeightedAverage::new(5.0);
        let samples = vec![
            SampleQuaternion::from_unit_quaternion(1.0, UnitQuaternion::default()),
            SampleQuaternion::from_unit_quaternion(
                6.0,
                UnitQuaternion::from_unit_quaternion(nalgebra::UnitQuaternion::from_euler_angles(
                    0.1, 0.2, 0.3,
                )),
            ),
        ];
        let smoothed_samples = filter.filter_batch(samples).unwrap();
        assert_eq!(smoothed_samples.len(), 1);
    }
}
