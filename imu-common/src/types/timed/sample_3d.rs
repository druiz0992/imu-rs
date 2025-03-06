use crate::traits::IMUSample;
use crate::types::untimed::{xyz::N_XYZ_COORDINATES, XYZ};

#[cfg(any(feature = "serde-serialize", test))]
use nalgebra::Vector3;
#[cfg(any(feature = "serde-serialize", test))]
use serde::{Deserialize, Deserializer, Serialize};
#[cfg(any(feature = "serde-serialize", test))]
use serde_json::Value;

/// A structure representing a 3D sample with a timestamp and measurement.
///
/// # Examples
///
/// ```
/// use imu_common::types::timed::Sample3D;
/// use imu_common::types::untimed::XYZ;
/// use imu_common::traits::IMUSample;
///
/// let timestamp = 1627846267.0;
/// let measurement = [1.0, 2.0, 3.0];
/// let sample = Sample3D::new(timestamp, measurement);
///
/// assert_eq!(sample.get_timestamp_secs(), timestamp);
/// assert_eq!(sample.get_measurement(), XYZ::from(measurement));
/// ```

const TIMESTAMP_IDX: usize = 0;
const X_COORD_IDX: usize = 1;
#[allow(dead_code)]
const Y_COORD_IDX: usize = 2;
#[allow(dead_code)]
const Z_COORD_IDX: usize = 3;

#[cfg_attr(any(feature = "serde-serialize", test), derive(Serialize))]
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

    fn get_timestamp_secs(&self) -> f64 {
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

#[cfg(any(feature = "serde-serialize", test))]
impl<'de> Deserialize<'de> for Sample3D {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        // Deserialize the whole input as a string (assumed format: "timestamp, x, y, z")
        let value: Value = Value::deserialize(deserializer)?;

        // Handle the case where the input is an object with a "timestamp" field and a "measurement" field
        if let Some(obj) = value.as_object() {
            // Extract the timestamp
            let timestamp = obj
                .get("timestamp")
                .and_then(Value::as_f64)
                .unwrap_or_default();

            // Try to extract the "measurement" field and deserialize it using XYZ's deserializer
            if let Some(measurement_value) = obj.get("measurement") {
                // Deserialize the measurement using XYZ's deserializer
                let measurement: XYZ = serde_json::from_value(measurement_value.clone())
                    .map_err(serde::de::Error::custom)?;

                // Return the deserialized Sample3D
                return Ok(Sample3D {
                    timestamp,
                    measurement,
                });
            }
        }

        // Handle the comma-separated string format like "1.2, 2.3, 3.4, 3.4"
        if let Some(scalar_str) = value.as_str() {
            let parts: Vec<f64> = scalar_str
                .split(',')
                .filter_map(|s| s.trim().parse().ok())
                .collect();
            let parts = if parts.len() < 4 {
                let mut parts = parts;
                parts.resize(4, 0.0); // Resize to 3, filling missing values with 0.0
                parts
            } else {
                parts
            };

            // We expect exactly 4 values (timestamp + 3 values for XYZ)
            if parts.len() == 4 {
                let timestamp = parts[0];
                let measurement = XYZ(Vector3::new(parts[1], parts[2], parts[3]));

                return Ok(Sample3D {
                    timestamp,
                    measurement,
                });
            }
        }

        Err(serde::de::Error::custom("Invalid format for Sample3D"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(any(feature = "serde-serialize", test))]
    use serde_json;

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
    fn test_get_timestamp_secs() {
        let timestamp = 1627846267.0;
        let measurement = [1.0, 2.0, 3.0];
        let sample = Sample3D::new(timestamp, measurement);

        assert_eq!(sample.get_timestamp_secs(), timestamp);
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_sample_serialize() {
        let timestamp = 1627846267.0;
        let measurement = [1.0, 2.0, 3.0];
        let sample = Sample3D::new(timestamp, measurement);

        let serialized = serde_json::to_string(&sample).unwrap();
        let expected = r#"{"timestamp":1627846267.0,"measurement":{"x":1.0,"y":2.0,"z":3.0}}"#;
        assert_eq!(serialized, expected);
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_sample_deserialize() {
        let data = r#"{"timestamp":1627846267.0,"measurement":{"x":1.0,"y":2.0,"z":3.0}}"#;
        let sample: Sample3D = serde_json::from_str(data).unwrap();

        assert_eq!(sample.timestamp, 1627846267.0);
        assert_eq!(sample.get_measurement(), XYZ::from([1.0, 2.0, 3.0]));
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_sample_deserialize_missing_labels() {
        let data = r#"{"timestamp":1627846267.0,"measurement":{"x":1.0,"z":3.0}}"#;
        let sample: Sample3D = serde_json::from_str(data).unwrap();

        assert_eq!(sample.timestamp, 1627846267.0);
        assert_eq!(sample.get_measurement(), XYZ::from([1.0, 0.0, 3.0]));
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_sample_deserialize_no_labels() {
        let data = r#""1627846267.0,1.0, 2.0, 3.0""#;
        let sample: Sample3D = serde_json::from_str(data).unwrap();

        assert_eq!(sample.timestamp, 1627846267.0);
        assert_eq!(sample.get_measurement(), XYZ::from([1.0, 2.0, 3.0]));
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_sample_deserialize_no_labels_missing_samples() {
        let data = r#""1627846267.0,1.0, 3.0""#;
        let sample: Sample3D = serde_json::from_str(data).unwrap();

        assert_eq!(sample.timestamp, 1627846267.0);
        assert_eq!(sample.get_measurement(), XYZ::from([1.0, 3.0, 0.0]));
    }

    #[test]
    fn test_try_from_vec() {
        let data = vec![1627846267.0, 1.0, 2.0, 3.0];
        let sample = Sample3D::try_from(data).unwrap();

        assert_eq!(sample.timestamp, 1627846267.0);
        assert_eq!(sample.get_measurement(), XYZ::from([1.0, 2.0, 3.0]));
    }

    #[test]
    fn test_try_from_vec_invalid_length() {
        let data = vec![1627846267.0, 1.0, 2.0];
        let result = Sample3D::try_from(data);

        assert!(result.is_err());
        assert_eq!(result.err(), Some("Invalid length of input vector"));
    }
}
