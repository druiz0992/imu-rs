use crate::traits::imu::BasicArithmetic;
use crate::types::buffers::CircularBuffer;
use crate::types::timed::SampleQuaternion;
use crate::types::untimed::UnitQuaternion;
use crate::{IMUFilter, IMUSample, IMUUntimedSample};

/// A moving average filter for IMU (Inertial Measurement Unit) data.
/// The moving average filter is used to smooth out short-term fluctuations and highlight longer-term trends in the data.
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
/// use common::types::filters::MovingAverage;
/// use common::types::timed::Sample3D;
/// use common::types::untimed::XYZ;
/// use common::traits::imu::{IMUFilter, IMUSample};
///
/// let mut ma = MovingAverage::<XYZ>::new(3);
/// let samples = vec![
///     Sample3D::from_measurement(0.0, XYZ::new([1.0, 1.0, 1.0])),
///     Sample3D::from_measurement(0.1, XYZ::new([2.0, 2.0, 2.0])),
///     Sample3D::from_measurement(0.2, XYZ::new([3.0, 3.0, 3.0])),
/// ];
/// let filtered_samples = ma.filter_batch(samples).unwrap();
/// ```
///

const DEFAULT_CAPACITY: usize = 64;

/// Definition of moving average filter, containing `window_size` elements to do the smoothing. Samples are stored in a circular buffer.
/// When one samples is pushed, oldest sample is popped. Contents of the buffer are used then to do the smoothind
#[derive(Clone, Debug)]
pub struct MovingAverage<T> {
    window_size: f64,
    buffer: CircularBuffer<T>,
    aggregate: T,
}

impl<T: IMUUntimedSample> MovingAverage<T> {
    /// Initializes new `MovingAverage` filter with `window_size` elements.
    pub fn new(window_size: usize) -> Self {
        Self {
            window_size: window_size as f64,
            buffer: CircularBuffer::new(window_size),
            aggregate: T::default(),
        }
    }
}

/// General implementation of IMUFIlter for samples that implement `BasicArithmetic` trait
impl<T, U> IMUFilter<U> for MovingAverage<T>
where
    T: IMUUntimedSample + BasicArithmetic + Default + Send + Sync + 'static + Clone,
    U: IMUSample<Untimed = T>,
{
    /// Filters a batch of IMU samples using the moving average filter.
    fn filter_batch(&mut self, samples: Vec<U>) -> Result<Vec<U>, &str> {
        if samples.is_empty() {
            return Err("No samples to filter");
        }
        let mut filtered_data: Vec<U> = Vec::with_capacity(samples.len());
        self.aggregate = T::default();
        for sample in samples {
            let measurement = sample.get_measurement();
            let timestamp = sample.get_timestamp();
            let out = self.buffer.push(measurement.clone());
            self.aggregate -= out;
            self.aggregate += measurement;
            filtered_data.push(U::from_measurement(
                timestamp,
                self.aggregate.clone() / 10.0,
            ));
        }
        Ok(filtered_data)
    }
}

/// Specific implementation of IMUFIlter for quaternion samples
impl IMUFilter<SampleQuaternion> for MovingAverage<UnitQuaternion>
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
        let mut filtered_data: Vec<SampleQuaternion> = Vec::with_capacity(DEFAULT_CAPACITY);

        for sample in samples {
            let measurement = sample.get_measurement();
            let timestamp = sample.get_timestamp();
            let out = self.buffer.push(measurement.clone());
            let mut smoothed_quaternion = self.aggregate.inner();
            smoothed_quaternion =
                smoothed_quaternion.slerp(&out.inner().inverse(), 1.0 / self.window_size);
            smoothed_quaternion =
                smoothed_quaternion.slerp(&measurement.inner(), 1.0 / self.window_size);
            filtered_data.push(SampleQuaternion::from_measurement(
                timestamp,
                UnitQuaternion::from_unit_quaternion(smoothed_quaternion),
            ));
        }
        Ok(filtered_data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use once_cell::sync::Lazy;

    use crate::types::timed::sample_3d::Sample3D;
    use crate::types::untimed::XYZ;

    static SAMPLE_0: Lazy<XYZ> = Lazy::new(|| XYZ::new([0.0, 0.0, 0.0]));
    static SAMPLE_1: Lazy<XYZ> = Lazy::new(|| XYZ::new([1.0, 1.0, 1.0]));
    static SAMPLE_2: Lazy<XYZ> = Lazy::new(|| XYZ::new([2.0, 2.0, 2.0]));
    static SAMPLE_3: Lazy<XYZ> = Lazy::new(|| XYZ::new([3.0, 3.0, 3.0]));
    static SAMPLE_4: Lazy<XYZ> = Lazy::new(|| XYZ::new([4.0, 4.0, 4.0]));
    static SAMPLE_5: Lazy<XYZ> = Lazy::new(|| XYZ::new([5.0, 5.0, 5.0]));

    /// Tests the functionality of the `CircularBuffer` by pushing elements into it and checking the output.
    #[test]
    fn test_circular_buffer() {
        let mut buffer = CircularBuffer::<XYZ>::new(3);

        assert_eq!(buffer.push(SAMPLE_1.clone()), SAMPLE_0.clone());
        assert_eq!(buffer.push(SAMPLE_2.clone()), SAMPLE_0.clone());
        assert_eq!(buffer.push(SAMPLE_3.clone()), SAMPLE_0.clone());
        assert_eq!(buffer.push(SAMPLE_4.clone()), SAMPLE_1.clone());
        assert_eq!(buffer.push(SAMPLE_5.clone()), SAMPLE_2.clone());
    }

    /// Tests the functionality of the `MovingAverage` filter by filtering a batch of samples and checking the output.
    #[test]
    fn test_moving_average() {
        let mut ma = MovingAverage::<XYZ>::new(3);
        let eps = 10e-3;
        let tolerance = XYZ::new([eps, eps, eps]);

        assert!(
            ma.filter_batch(vec![Sample3D::from_measurement(0.0, SAMPLE_1.clone())])
                .unwrap()[0]
                .clone()
                .get_measurement()
                - XYZ::new([0.333, 0.333, 0.333])
                < tolerance
        );

        assert!(
            ma.filter_batch(vec![Sample3D::from_measurement(0.0, SAMPLE_2.clone())])
                .unwrap()[0]
                .clone()
                .get_measurement()
                - SAMPLE_1.clone()
                < tolerance
        );
        assert!(
            ma.filter_batch(vec![Sample3D::from_measurement(0.0, SAMPLE_3.clone())])
                .unwrap()[0]
                .clone()
                .get_measurement()
                - SAMPLE_2.clone()
                < tolerance
        );
        assert!(
            ma.filter_batch(vec![Sample3D::from_measurement(0.0, SAMPLE_4.clone())])
                .unwrap()[0]
                .clone()
                .get_measurement()
                - SAMPLE_3.clone()
                < tolerance
        );
        assert!(
            ma.filter_batch(vec![Sample3D::from_measurement(0.0, SAMPLE_5.clone())])
                .unwrap()[0]
                .clone()
                .get_measurement()
                - SAMPLE_4.clone()
                < tolerance
        );
    }
}
