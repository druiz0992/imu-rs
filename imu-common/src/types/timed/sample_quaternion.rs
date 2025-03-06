use crate::traits::IMUSample;
use crate::types::untimed::unit_quaternion::{N_QUATERNION_COORDINATES, W_QUATERNION_COORD_IDX};
use crate::types::untimed::UnitQuaternion;

#[cfg(any(feature = "serde-serialize", test))]
use crate::types::untimed::unit_quaternion::{
    X_QUATERNION_COORD_IDX, Y_QUATERNION_COORD_IDX, Z_QUATERNION_COORD_IDX,
};
#[cfg(any(feature = "serde-serialize", test))]
use serde::{Deserialize, Deserializer, Serialize};
#[cfg(any(feature = "serde-serialize", test))]
use serde_json::Value;

/// A structure representing a quaternion sample with a timestamp.
///
/// # Examples
///
/// ```
/// use imu_common::types::timed::SampleQuaternion;
/// use imu_common::types::untimed::UnitQuaternion;
/// use imu_common::traits::IMUSample;
///
/// let timestamp = 1627846267.0;
/// let measurement = [1.0, 0.0, 0.0, 0.0];
/// let sample = SampleQuaternion::new(timestamp, measurement);
///
/// assert_eq!(sample.get_timestamp_secs(), timestamp);
/// assert_eq!(sample.get_measurement(), UnitQuaternion::from(measurement));
/// ```

const TIMESTAMP_IDX: usize = 0;

#[cfg_attr(any(feature = "serde-serialize", test), derive(Serialize))]
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
    fn get_timestamp_secs(&self) -> f64 {
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
            value[W_QUATERNION_COORD_IDX + 1..=N_QUATERNION_COORDINATES].to_vec(),
        )?;
        Ok(SampleQuaternion::from_measurement(
            value[TIMESTAMP_IDX],
            measurement,
        ))
    }
}

#[cfg(any(feature = "serde-serialize", test))]
impl<'de> Deserialize<'de> for SampleQuaternion {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        // Deserialize the whole input as a string (assumed format: "timestamp, x, y, z")
        let value: Value = Value::deserialize(deserializer)?;

        // Handle the case where the input is an object with a "timestamp" field and a "quaternion" field
        if let Some(obj) = value.as_object() {
            // Extract the timestamp
            let timestamp = obj
                .get("timestamp")
                .and_then(Value::as_f64)
                .unwrap_or_default();

            // Try to extract the "measurement" field and deserialize it using UnitQuaternion deserializer
            if let Some(measurement_value) = obj.get("quaternion") {
                // Deserialize the measurement using UnitQuaternion deserializer
                let measurement: UnitQuaternion = serde_json::from_value(measurement_value.clone())
                    .map_err(serde::de::Error::custom)?;

                // Return the deserialized Sample3D
                return Ok(SampleQuaternion::from_unit_quaternion(
                    timestamp,
                    measurement,
                ));
            }
        }

        // Handle the comma-separated string format like "1.2, 1.2, 2.3, 3.4, 3.4"
        if let Some(scalar_str) = value.as_str() {
            let parts: Vec<f64> = scalar_str
                .split(',')
                .filter_map(|s| s.trim().parse().ok())
                .collect();
            let parts = if parts.len() < 5 {
                let mut parts = parts;
                parts.resize(5, 0.0); // Resize to 3, filling missing values with 0.0
                parts
            } else {
                parts
            };

            // We expect exactly 5 values (timestamp + 4 values for UnitQuaternion)
            if parts.len() == 5 {
                let timestamp = parts[0];
                let measurement = UnitQuaternion::new([
                    parts[W_QUATERNION_COORD_IDX + 1],
                    parts[X_QUATERNION_COORD_IDX + 1],
                    parts[Y_QUATERNION_COORD_IDX + 1],
                    parts[Z_QUATERNION_COORD_IDX + 1],
                ]);

                return Ok(SampleQuaternion::from_unit_quaternion(
                    timestamp,
                    measurement,
                ));
            }
        }

        Err(serde::de::Error::custom("Invalid format for Sample3D"))
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
    fn test_get_timestamp_secs() {
        let timestamp = 1627846267.0;
        let measurement = [1.0, 2.0, 3.0, 4.0];
        let sample = SampleQuaternion::new(timestamp, measurement);

        assert_eq!(sample.get_timestamp_secs(), timestamp);
    }

    #[test]
    fn test_from_unit_quaternion() {
        let timestamp = 1627846267.0;
        let measurement = [1.0, 2.0, 3.0, 4.0];
        let quaternion = UnitQuaternion::new(measurement);
        let sample = SampleQuaternion::from_unit_quaternion(timestamp, quaternion.clone());

        assert_eq!(sample.timestamp, timestamp);
        assert_eq!(sample.get_measurement(), quaternion);
    }

    #[test]
    fn test_try_from_valid_vector() {
        let timestamp = 1627846267.0;
        let measurement = [1.0, 0.0, 0.0, 0.0];
        let mut input = vec![timestamp];
        input.extend_from_slice(&measurement);

        let sample = SampleQuaternion::try_from(input).unwrap();

        assert_eq!(sample.timestamp, timestamp);
        assert_eq!(sample.get_measurement(), UnitQuaternion::from(measurement));
    }

    #[test]
    fn test_try_from_invalid_vector_length() {
        let input = vec![1627846267.0, 1.0, 2.0, 3.0]; // Missing one element

        let result = SampleQuaternion::try_from(input);

        assert!(result.is_err());
        assert_eq!(result.err(), Some("Invalid length input vector"));
    }

    #[test]
    fn test_try_from_invalid_vector_content() {
        let input = vec![1627846267.0, 1.0, 2.0, 3.0, 4.0, 5.0]; // Extra element

        let result = SampleQuaternion::try_from(input);

        assert!(result.is_err());
        assert_eq!(result.err(), Some("Invalid length input vector"));
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_sample_serialize() {
        let timestamp = 1627846267.0;
        let measurement = [1.0, 0.0, 0.0, 0.0];
        let sample = SampleQuaternion::new(timestamp, measurement);

        let serialized = serde_json::to_string(&sample).unwrap();
        let expected =
            r#"{"timestamp":1627846267.0,"quaternion":{"i":0.0,"j":0.0,"k":0.0,"w":1.0}}"#;
        assert_eq!(serialized, expected);
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_sample_deserialize() {
        let data = r#"{"timestamp":1627846267.0,"quaternion":{"w":1.0,"i":0.0,"j":0.0,"k":0.0}}"#;
        let sample: SampleQuaternion = serde_json::from_str(data).unwrap();

        assert_eq!(sample.timestamp, 1627846267.0);
        assert_eq!(
            sample.get_measurement(),
            UnitQuaternion::from([1.0, 0.0, 0.0, 0.0])
        );
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_sample_deserialize_missing_labels() {
        let data = r#"{"timestamp":1627846267.0,"quaternion":{"w":1.0,"k":0.0}}"#;
        let sample: SampleQuaternion = serde_json::from_str(data).unwrap();

        assert_eq!(sample.timestamp, 1627846267.0);
        assert_eq!(
            sample.get_measurement(),
            UnitQuaternion::from([1.0, 0.0, 0.0, 0.0])
        );
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_sample_deserialize_no_labels() {
        let data = r#""1627846267.0,1.0, 0.0, 0.0,0.0""#;
        let sample: SampleQuaternion = serde_json::from_str(data).unwrap();

        assert_eq!(sample.timestamp, 1627846267.0);
        assert_eq!(
            sample.get_measurement(),
            UnitQuaternion::from([1.0, 0.0, 0.0, 0.0])
        );
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_sample_deserialize_no_labels_missing_samples() {
        let data = r#""1627846267.0,1.0, 0.0, 0.0""#;
        let sample: SampleQuaternion = serde_json::from_str(data).unwrap();

        assert_eq!(sample.timestamp, 1627846267.0);
        assert_eq!(
            sample.get_measurement(),
            UnitQuaternion::from([1.0, 0.0, 0.0, 0.0])
        );
    }
}
