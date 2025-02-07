use crate::{constants::*, types::xyz::XYZ, IMUSample};
/// A structure representing a 3D sample with a timestamp and measurement.
///
/// # Fields
///
/// * `timestamp` - A floating-point number representing the timestamp of the sample.
/// * `measurement` - An `XYZ` structure representing the 3D measurement.
///
/// # Examples
///
/// ```
/// use common::{Sample3D, IMUSample};
///
/// let timestamp = 1627846267.0;
/// let measurement = [1.0, 2.0, 3.0];
/// let sample = Sample3D::new(timestamp, measurement);
///
/// assert_eq!(sample.get_timestamp(), timestamp);
/// assert_eq!(sample.get_measurement(), measurement);
/// ```

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

    /// Creates a new `Sample3D` instance from a vector containing a timestamp and a measurement.
    pub fn from_vec(sample_vec: [f64; 4]) -> Self {
        Self {
            timestamp: sample_vec[0],
            measurement: XYZ::new([sample_vec[1], sample_vec[2], sample_vec[3]]),
        }
    }
}

impl IMUSample for Sample3D {
    fn get_measurement(&self) -> Vec<f64> {
        self.measurement.inner().into()
    }

    fn get_timestamp(&self) -> f64 {
        self.timestamp
    }

    fn from_untimed(sample: Vec<f64>, timestamp: f64) -> Self {
        let xyz = XYZ::from_vec(sample).unwrap();
        Sample3D::from_xyz(timestamp, xyz)
    }
}

impl TryFrom<Vec<f64>> for Sample3D {
    type Error = String;

    fn try_from(value: Vec<f64>) -> Result<Self, Self::Error> {
        if value.len() != N_XYZ_COORDINATES + 1 {
            return Err("Invalid length of input vector".to_string());
        }
        Ok(Sample3D::from_vec([value[0], value[1], value[2], value[3]]))
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
        assert_eq!(sample.get_measurement(), [1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_get_measurement() {
        let timestamp = 1627846267.0;
        let measurement = [1.0, 2.0, 3.0];
        let sample = Sample3D::new(timestamp, measurement);

        assert_eq!(sample.get_measurement(), measurement);
    }

    #[test]
    fn test_get_timestamp() {
        let timestamp = 1627846267.0;
        let measurement = [1.0, 2.0, 3.0];
        let sample = Sample3D::new(timestamp, measurement);

        assert_eq!(sample.get_timestamp(), timestamp);
    }
}
