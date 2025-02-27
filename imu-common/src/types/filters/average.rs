use crate::traits::imu::BasicArithmetic;
use crate::traits::{IMUFilter, IMUSample, IMUUntimedSample};
use crate::types::timed::SampleQuaternion;
use crate::types::untimed::UnitQuaternion;
use std::marker::PhantomData;

/// An  averaging filter for IMU (Inertial Measurement Unit) data.
///
/// There are two implementations of the `MovingAverage` filter:
/// 1. For general IMU samples (`IMUUntimedSample`), which can be any type that implements the required traits.
/// 2. Specifically for quaternion samples (`SampleQuaternion`), which are used to represent rotations.
///
/// The `MovingAverage` struct implements the `IMUFilter` trait, which defines a method for filtering a batch of IMU samples.
/// The `filter_batch` method processes a batch of samples, applying the moving average filter to smooth the data.
///
/// ## Example
///
/// ```rust
/// use imu_common::types::filters::Average;
/// use imu_common::types::timed::Sample3D;
/// use imu_common::types::untimed::XYZ;
/// use imu_common::traits::imu::{IMUFilter, IMUSample};
///
/// let mut avg =Average::<XYZ>::new();
/// let samples = vec![
///     Sample3D::from_measurement(0.0, XYZ::new([1.0, 1.0, 1.0])),
///     Sample3D::from_measurement(0.1, XYZ::new([2.0, 2.0, 2.0])),
///     Sample3D::from_measurement(0.2, XYZ::new([3.0, 3.0, 3.0])),
/// ];
/// let filtered_samples = avg.filter_batch(samples).unwrap();
/// ```
///

/// Definition of moving average filter, containing `window_size` elements to do the smoothing. Samples are stored in a circular buffer.
/// When one samples is pushed, oldest sample is popped. Contents of the buffer are used then to do the smoothind
#[derive(Clone, Debug, Default)]
pub struct Average<T> {
    _phantom_data: PhantomData<T>,
}

impl<T> Average<T> {
    /// Initializes new `MovingAverage` filter with `window_size` elements.
    pub fn new() -> Self {
        Self {
            _phantom_data: PhantomData,
        }
    }
}

/// General implementation of IMUFIlter for samples that implement `BasicArithmetic` trait
impl<T, U> IMUFilter<U> for Average<T>
where
    T: IMUUntimedSample + BasicArithmetic + Default + Send + Sync + 'static + Clone,
    U: IMUSample<Untimed = T>,
{
    /// Filters a batch of IMU samples using the moving average filter.
    fn filter_batch(&mut self, samples: Vec<U>) -> Result<Vec<U>, &str> {
        if samples.is_empty() {
            return Err("No samples to filter");
        }
        let aggregate = samples
            .iter()
            .map(|s| s.get_measurement())
            .fold(T::default(), |acc, x| acc + x);

        let average = U::from_measurement(0.0, aggregate / samples.len() as f64);

        Ok(vec![average])
    }
}

/// Specific implementation of IMUFIlter for quaternion samples
impl IMUFilter<SampleQuaternion> for Average<SampleQuaternion>
where
    SampleQuaternion: Sized,
{
    /// Filters a batch of quaternion samples using the moving average filter.
    fn filter_batch(
        &mut self,
        samples: Vec<SampleQuaternion>,
    ) -> Result<Vec<SampleQuaternion>, &str> {
        if samples.is_empty() {
            return Err("No samples to filter");
        }
        let mut smoothed_quaternion = samples[0].get_measurement().inner();

        for (idx, sample) in samples.iter().skip(1).enumerate() {
            let measurement = sample.get_measurement().inner();
            smoothed_quaternion = smoothed_quaternion.slerp(&measurement, 1.0 / (idx + 2) as f64);
        }
        let filtered_data = SampleQuaternion::from_measurement(
            0.0,
            UnitQuaternion::from_unit_quaternion(smoothed_quaternion),
        );
        Ok(vec![filtered_data])
    }
}
