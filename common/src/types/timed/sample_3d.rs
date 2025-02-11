use crate::types::untimed::{xyz::N_XYZ_COORDINATES, XYZ};
use crate::IMUSample;
/// A structure representing a 3D sample with a timestamp and measurement.
///
/// # Examples
///
/// ```
/// use common::types::timed::Sample3D;
/// use common::types::untimed::XYZ;
/// use common::IMUSample;
///
/// let timestamp = 1627846267.0;
/// let measurement = [1.0, 2.0, 3.0];
/// let sample = Sample3D::new(timestamp, measurement);
///
/// assert_eq!(sample.get_timestamp(), timestamp);
/// assert_eq!(sample.get_measurement(), XYZ::from(measurement));
/// ```

const TIMESTAMP_IDX: usize = 0;
const X_COORD_IDX: usize = 1;
#[allow(dead_code)]
const Y_COORD_IDX: usize = 2;
#[allow(dead_code)]
const Z_COORD_IDX: usize = 3;

#[derive(Debug, Clone, Default, PartialEq, PartialOrd)]
pub struct Sample3D {
    timestamp: f64,
    measurement: XYZ,
}

/// Represents a 3D sample with a timestamp and a measurement.
impl Sample3D {
    ///  Creates a new `Sample3D` instance from a timestamp and a measurement array.
    pub fn new(timestamp: f64, measurement: [f64; N_XYZ_COORDINATES]) -> Self {
        Self {
            timestamp,
            measurement: XYZ::new(measurement),
        }
    }

    /// Creates a new `Sample3D` instance from a timestamp and an `XYZ` measurement.
    pub fn from_xyz(timestamp: f64, measurement: XYZ) -> Self {
        Self {
            timestamp,
            measurement,
        }
    }
}
impl IMUSample for Sample3D {
    type Untimed = XYZ;

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

impl TryFrom<Vec<f64>> for Sample3D {
    type Error = &'static str;

    fn try_from(value: Vec<f64>) -> Result<Self, Self::Error> {
        if value.len() != N_XYZ_COORDINATES + 1 {
            return Err("Invalid length of input vector");
        }
        let measurement = XYZ::try_from(value[X_COORD_IDX..=N_XYZ_COORDINATES].to_vec())?;
        Ok(Sample3D::from_measurement(
            value[TIMESTAMP_IDX],
            measurement,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sample_new() {
        let timestamp = 1627846267.0;
        let measurement = [1.0, 2.0, 3.0];
        let sample = Sample3D::new(timestamp, measurement);

        assert_eq!(sample.timestamp, timestamp);
        assert_eq!(sample.get_measurement(), XYZ::from([1.0, 2.0, 3.0]));
        assert_eq!(
            sample.get_measurement(),
            XYZ::try_from(vec![1.0, 2.0, 3.0]).unwrap()
        );
    }

    #[test]
    fn test_get_measurement() {
        let timestamp = 1627846267.0;
        let measurement = [1.0, 2.0, 3.0];
        let sample = Sample3D::new(timestamp, measurement);

        assert_eq!(sample.get_measurement(), XYZ::from(measurement));
    }

    #[test]
    fn test_get_timestamp() {
        let timestamp = 1627846267.0;
        let measurement = [1.0, 2.0, 3.0];
        let sample = Sample3D::new(timestamp, measurement);

        assert_eq!(sample.get_timestamp(), timestamp);
    }
}
