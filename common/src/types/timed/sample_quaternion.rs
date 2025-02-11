use crate::types::untimed::unit_quaternion::{N_QUATERNION_COORDINATES, W_QUATERNION_COORD_IDX};
use crate::types::untimed::UnitQuaternion;
use crate::IMUSample;

/// A structure representing a quaternion sample with a timestamp.
///
/// # Examples
///
/// ```
/// use common::types::timed::SampleQuaternion;
/// use common::types::untimed::UnitQuaternion;
/// use common::IMUSample;
///
/// let timestamp = 1627846267.0;
/// let measurement = [1.0, 0.0, 0.0, 0.0];
/// let sample = SampleQuaternion::new(timestamp, measurement);
///
/// assert_eq!(sample.get_timestamp(), timestamp);
/// assert_eq!(sample.get_measurement(), UnitQuaternion::from(measurement));
/// ```

const TIMESTAMP_IDX: usize = 0;

#[derive(Debug, Clone, Default)]
pub struct SampleQuaternion {
    timestamp: f64,
    quaternion: UnitQuaternion,
}

impl SampleQuaternion {
    ///   Creates a new `SampleQuaternion` instance with the given timestamp and measurement.
    pub fn new(timestamp: f64, measurement: [f64; N_QUATERNION_COORDINATES]) -> Self {
        Self {
            timestamp,
            quaternion: UnitQuaternion::new(measurement),
        }
    }

    pub fn from_unit_quaternion(timestamp: f64, quaternion: UnitQuaternion) -> Self {
        Self {
            timestamp,
            quaternion,
        }
    }
}

impl IMUSample for SampleQuaternion {
    type Untimed = UnitQuaternion;

    ///  Returns the quaternion measurement as a vector.
    fn get_measurement(&self) -> Self::Untimed {
        self.quaternion.clone()
    }

    ///  Returns the timestamp of the sample.
    fn get_timestamp(&self) -> f64 {
        self.timestamp
    }

    ///  Creates a `SampleQuaternion` from an untimed sample and a timestamp.
    fn from_measurement(timestamp: f64, measurement: Self::Untimed) -> Self {
        SampleQuaternion::from_unit_quaternion(timestamp, measurement)
    }
}

impl TryFrom<Vec<f64>> for SampleQuaternion {
    type Error = &'static str;

    fn try_from(value: Vec<f64>) -> Result<Self, Self::Error> {
        if value.len() != N_QUATERNION_COORDINATES + 1 {
            return Err("Invalid length input vector");
        }
        let measurement = UnitQuaternion::try_from(
            value[W_QUATERNION_COORD_IDX..=N_QUATERNION_COORDINATES].to_vec(),
        )?;
        Ok(SampleQuaternion::from_measurement(
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
        let measurement = [1.0, 2.0, 3.0, 3.0];
        let sample = SampleQuaternion::new(timestamp, measurement);

        assert_eq!(sample.timestamp, timestamp);
        assert_eq!(sample.get_measurement(), UnitQuaternion::from(measurement));
        assert_eq!(
            sample.get_measurement(),
            UnitQuaternion::try_from(measurement.to_vec()).unwrap()
        );
    }

    #[test]
    fn test_get_measurement() {
        let timestamp = 1627846267.0;
        let measurement = [1.0, 2.0, 3.0, 4.0];
        let sample = SampleQuaternion::new(timestamp, measurement);

        assert_eq!(sample.get_measurement(), UnitQuaternion::from(measurement));
    }

    #[test]
    fn test_get_timestamp() {
        let timestamp = 1627846267.0;
        let measurement = [1.0, 2.0, 3.0, 4.0];
        let sample = SampleQuaternion::new(timestamp, measurement);

        assert_eq!(sample.get_timestamp(), timestamp);
    }
}
