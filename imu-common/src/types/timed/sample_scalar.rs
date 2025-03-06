use crate::traits::IMUSample;
use crate::types::untimed::Scalar;

#[cfg(any(feature = "serde-serialize", test))]
use serde::{Deserialize, Deserializer, Serialize};
#[cfg(any(feature = "serde-serialize", test))]
use serde_json::Value;

/// A structure representing a 1D sample with a timestamp and measurement.
///
/// # Examples
///
/// ```
/// use imu_common::traits::IMUSample;
/// use imu_common::types::timed::SampleScalar;
/// use imu_common::types::untimed::Scalar;
///
/// let timestamp = 1627846267.0;
/// let measurement =1.0;
/// let sample = SampleScalar::new(timestamp, measurement);
///
/// assert_eq!(sample.get_timestamp_secs(), timestamp);
/// assert_eq!(sample.get_measurement(), Scalar::from(measurement));
/// ```

#[cfg_attr(any(feature = "serde-serialize", test), derive(Serialize))]
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

#[cfg(any(feature = "serde-serialize", test))]
impl<'de> Deserialize<'de> for SampleScalar {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        // Deserialize the whole input as a string (assumed format: "timestamp, x")
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
                let measurement: Scalar = serde_json::from_value(measurement_value.clone())
                    .map_err(serde::de::Error::custom)?;

                // Return the deserialized Sample3D
                return Ok(SampleScalar {
                    timestamp,
                    measurement,
                });
            }
        }

        // Handle the comma-separated string format like "1.2, 2.3"
        if let Some(scalar_str) = value.as_str() {
            let parts: Vec<f64> = scalar_str
                .split(',')
                .filter_map(|s| s.trim().parse().ok())
                .collect();
            let parts = if parts.len() < 2 {
                let mut parts = parts;
                parts.resize(2, 0.0); // Resize to 3, filling missing values with 0.0
                parts
            } else {
                parts
            };

            // We expect exactly 2 values (timestamp + 1 values for Scalar)
            if parts.len() == 2 {
                let timestamp = parts[0];
                let measurement = Scalar::new(parts[1]);

                return Ok(SampleScalar {
                    timestamp,
                    measurement,
                });
            }
        }

        Err(serde::de::Error::custom("Invalid format for SampleScalar"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sample_scalar_new() {
        let timestamp = 1627846267.0;
        let measurement = 1.0;
        let sample = SampleScalar::new(timestamp, measurement);

        assert_eq!(sample.get_timestamp_secs(), timestamp);
        assert_eq!(sample.get_measurement(), Scalar::from(measurement));
    }

    #[test]
    fn test_sample_scalar_from_scalar() {
        let timestamp = 1627846267.0;
        let measurement = Scalar::new(1.0);
        let sample = SampleScalar::from_scalar(timestamp, measurement.clone());

        assert_eq!(sample.get_timestamp_secs(), timestamp);
        assert_eq!(sample.get_measurement(), measurement);
    }

    #[test]
    fn test_sample_scalar_get_measurement() {
        let timestamp = 1627846267.0;
        let measurement = Scalar::new(1.0);
        let sample = SampleScalar::from_scalar(timestamp, measurement.clone());

        assert_eq!(sample.get_measurement(), measurement);
    }

    #[test]
    fn test_sample_scalar_get_timestamp_secs() {
        let timestamp = 1627846267.0;
        let measurement = Scalar::new(1.0);
        let sample = SampleScalar::from_scalar(timestamp, measurement);

        assert_eq!(sample.get_timestamp_secs(), timestamp);
    }

    #[test]
    fn test_sample_scalar_from_measurement() {
        let timestamp = 1627846267.0;
        let measurement = Scalar::new(1.0);
        let sample = SampleScalar::from_measurement(timestamp, measurement.clone());

        assert_eq!(sample.get_timestamp_secs(), timestamp);
        assert_eq!(sample.get_measurement(), measurement);
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_sample_scalar_serialize() {
        let timestamp = 1627846267.0;
        let measurement = Scalar::new(1.0);
        let sample = SampleScalar::from_scalar(timestamp, measurement.clone());

        let serialized = serde_json::to_string(&sample).unwrap();
        let expected = format!(
            r#"{{"timestamp":{:.1},"measurement":{:.1}}}"#,
            timestamp,
            measurement.inner()
        );
        assert_eq!(serialized, expected);
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_sample_scalar_deserialize() {
        let timestamp = 1627846267.0;
        let measurement = Scalar::new(1.0);
        let json = format!(
            r#"{{"timestamp":{},"measurement":{}}}"#,
            timestamp,
            measurement.inner()
        );

        let deserialized: SampleScalar = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.get_timestamp_secs(), timestamp);
        assert_eq!(deserialized.get_measurement(), measurement);
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_sample_scalar_deserialize_no_labels() {
        let data = r#""1627846267.0,1.0""#;
        let sample: SampleScalar = serde_json::from_str(data).unwrap();

        assert_eq!(sample.timestamp, 1627846267.0);
        assert_eq!(sample.get_measurement(), Scalar::new(1.0));
    }
}
