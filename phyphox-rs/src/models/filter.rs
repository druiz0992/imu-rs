use common::buffers::CircularBuffer;
use common::{IMUFilter, IMUUntimedSample};

const DEFAULT_CAPACITY: usize = 64;

/// Definition of moving average filter, containing `window_size` elements to do the smoothing.
#[derive(Clone, Debug)]
pub(crate) struct MovingAverage<T> {
    window_size: f64,
    buffer: CircularBuffer<T>,
    aggregate: T,
}

impl<T: IMUUntimedSample> MovingAverage<T> {
    /// Initializes new `MovingAverage` filter with `window_size` elements.
    pub(crate) fn new(window_size: usize) -> Self {
        Self {
            window_size: window_size as f64,
            buffer: CircularBuffer::new(window_size),
            aggregate: T::default(),
        }
    }
}

impl<T: IMUUntimedSample> IMUFilter<T> for MovingAverage<T> {
    /// Returns smoothed samples
    fn filter(&mut self, samples: Vec<T>) -> Vec<T> {
        let mut filtered_data = Vec::with_capacity(DEFAULT_CAPACITY);
        for sample in samples {
            let out = self.buffer.push(sample.clone());
            self.aggregate -= out;
            self.aggregate += sample;
            filtered_data.push(self.aggregate.clone() / self.window_size);
        }
        filtered_data
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use once_cell::sync::Lazy;

    use common::types::XYZ;

    static SAMPLE_0: Lazy<XYZ> = Lazy::new(|| XYZ::new([0.0, 0.0, 0.0]));
    static SAMPLE_1: Lazy<XYZ> = Lazy::new(|| XYZ::new([1.0, 1.0, 1.0]));
    static SAMPLE_2: Lazy<XYZ> = Lazy::new(|| XYZ::new([2.0, 2.0, 2.0]));
    static SAMPLE_3: Lazy<XYZ> = Lazy::new(|| XYZ::new([3.0, 3.0, 3.0]));
    static SAMPLE_4: Lazy<XYZ> = Lazy::new(|| XYZ::new([4.0, 4.0, 4.0]));
    static SAMPLE_5: Lazy<XYZ> = Lazy::new(|| XYZ::new([5.0, 5.0, 5.0]));

    #[test]
    fn test_circular_buffer() {
        let mut buffer = CircularBuffer::<XYZ>::new(3);

        assert_eq!(buffer.push(SAMPLE_1.clone()), SAMPLE_0.clone());
        assert_eq!(buffer.push(SAMPLE_2.clone()), SAMPLE_0.clone());
        assert_eq!(buffer.push(SAMPLE_3.clone()), SAMPLE_0.clone());
        assert_eq!(buffer.push(SAMPLE_4.clone()), SAMPLE_1.clone());
        assert_eq!(buffer.push(SAMPLE_5.clone()), SAMPLE_2.clone());
    }

    #[test]
    fn test_moving_average() {
        let mut ma = MovingAverage::<XYZ>::new(3);
        let eps = 10e-3;
        let tolerance = XYZ::new([eps, eps, eps]);

        assert!(
            ma.filter(vec![SAMPLE_1.clone()])[0].clone() - XYZ::new([0.333, 0.333, 0.333])
                < tolerance
        );

        assert!(ma.filter(vec![SAMPLE_2.clone()])[0].clone() - SAMPLE_1.clone() < tolerance);
        assert!(ma.filter(vec![SAMPLE_3.clone()])[0].clone() - SAMPLE_2.clone() < tolerance);
        assert!(ma.filter(vec![SAMPLE_4.clone()])[0].clone() - SAMPLE_3.clone() < tolerance);
        assert!(ma.filter(vec![SAMPLE_5.clone()])[0].clone() - SAMPLE_4.clone() < tolerance);
    }
}
