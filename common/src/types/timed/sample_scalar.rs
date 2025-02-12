use crate::traits::IMUSample;
use crate::types::untimed::Scalar;
/// A structure representing a 1D sample with a timestamp and measurement.
///
/// # Examples
///
/// ```
/// use common::{IMUSample};
/// use common::types::timed::SampleScalar;
/// use common::types::untimed::Scalar;
///
/// let timestamp = 1627846267.0;
/// let measurement =1.0;
/// let sample = SampleScalar::new(timestamp, measurement);
///
/// assert_eq!(sample.get_timestamp(), timestamp);
/// assert_eq!(sample.get_measurement(), Scalar::from(measurement));
/// ```

#[derive(Debug, Clone, Default, PartialEq, PartialOrd)]
pub struct SampleScalar {
    timestamp: f64,
    measurement: Scalar,
}

/// Represents a 3D sample with a timestamp and a measurement.
impl SampleScalar {
    ///  Creates a new `Sample3D` instance from a timestamp and a measurement array.
    pub fn new(timestamp: f64, measurement: f64) -> Self {
        Self {
            timestamp,
            measurement: Scalar::new(measurement),
        }
    }

    /// Creates a new `Sample3D` instance from a timestamp and an `XYZ` measurement.
    pub fn from_scalar(timestamp: f64, measurement: Scalar) -> Self {
        Self {
            timestamp,
            measurement,
        }
    }
}

impl IMUSample for SampleScalar {
    type Untimed = Scalar;

    fn get_measurement(&self) -> Self::Untimed {
        self.measurement.clone()
    }

    fn get_timestamp(&self) -> f64 {
        self.timestamp
    }

    fn from_measurement(timestamp: f64, measurement: Self::Untimed) -> Self {
        Self {
            timestamp,
            measurement,
        }
    }
}
