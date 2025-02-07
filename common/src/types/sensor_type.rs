/// Represents different types of sensors.
///
/// # Variants
///
/// - `Accelerometer`: Represents an accelerometer sensor.
/// - `Gyroscope`: Represents a gyroscope sensor.
/// - `Magnetometer`: Represents a magnetometer sensor.
/// - `Other(String)`: Represents any other type of sensor, with a custom string description.
///
/// # Examples
///
/// ```
/// use common::SensorType;
///
/// let sensor = SensorType::Accelerometer;
/// assert_eq!(usize::from(sensor), 0);
///
/// let sensor = SensorType::from("gyroscope");
/// assert_eq!(sensor, SensorType::Gyroscope);
///
/// let sensor = SensorType::from("unknown");
/// assert_eq!(sensor, SensorType::Other(String::from("unknown")));
/// ```
#[derive(Clone, Debug, PartialEq, PartialOrd, Hash)]
#[repr(usize)]
pub enum SensorType {
    Accelerometer = 0,
    Gyroscope,
    Magnetometer,
    Other(String),
}

impl From<&SensorType> for usize {
    fn from(value: &SensorType) -> Self {
        match value {
            &SensorType::Accelerometer => 0,
            &SensorType::Gyroscope => 1,
            &SensorType::Magnetometer => 2,
            &SensorType::Other(_) => 3,
        }
    }
}

impl From<SensorType> for usize {
    fn from(value: SensorType) -> Self {
        usize::from(&value)
    }
}

impl From<&str> for SensorType {
    fn from(value: &str) -> Self {
        let lower_case_value = value.to_lowercase();

        if lower_case_value.contains("acc") {
            Self::Accelerometer
        } else if lower_case_value.contains("gyr") {
            Self::Gyroscope
        } else if lower_case_value.contains("mag") {
            Self::Magnetometer
        } else {
            Self::Other(lower_case_value)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_from_usize() {
        assert_eq!(usize::from(SensorType::Accelerometer), 0);
        assert_eq!(usize::from(SensorType::Gyroscope), 1);
        assert_eq!(usize::from(SensorType::Magnetometer), 2);
        assert_eq!(usize::from(SensorType::Other(String::from("other"))), 3);
    }

    #[test]
    fn test_from_str() {
        assert_eq!(SensorType::from("accelerometer"), SensorType::Accelerometer);
        assert_eq!(SensorType::from("gyroscope"), SensorType::Gyroscope);
        assert_eq!(SensorType::from("magnetometer"), SensorType::Magnetometer);
        assert_eq!(
            SensorType::from("other"),
            SensorType::Other(String::from("other"))
        );
    }

    #[test]
    fn test_from_str_case_insensitive() {
        assert_eq!(SensorType::from("ACCELEROMETER"), SensorType::Accelerometer);
        assert_eq!(SensorType::from("GyRoScope"), SensorType::Gyroscope);
        assert_eq!(SensorType::from("MAGNETOMETER"), SensorType::Magnetometer);
        assert_eq!(
            SensorType::from("OtHeR"),
            SensorType::Other(String::from("other"))
        );
    }

    #[test]
    fn test_from_str_partial_match() {
        assert_eq!(SensorType::from("acc"), SensorType::Accelerometer);
        assert_eq!(SensorType::from("gyr"), SensorType::Gyroscope);
        assert_eq!(SensorType::from("mag"), SensorType::Magnetometer);
        assert_eq!(
            SensorType::from("unknown"),
            SensorType::Other(String::from("unknown"))
        );
    }
}
