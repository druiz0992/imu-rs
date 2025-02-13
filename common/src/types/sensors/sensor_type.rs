use uuid::Uuid;

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
/// use common::types::sensors::SensorType;
/// use uuid::Uuid;
///
/// let sensor = SensorType::Accelerometer(Uuid::new_v4());
/// assert!(usize::from(sensor) <= 1000);
///
/// let gyro_id = Uuid::new_v4();
/// let sensor = SensorType::try_from(format!("gyroscope::{}",gyro_id).as_str()).unwrap();
/// assert_eq!(sensor, SensorType::Gyroscope(gyro_id));
///
/// let other_id = Uuid::new_v4();
/// let sensor = SensorType::try_from(format!("unknown::{}",other_id).as_str()).unwrap();
/// assert_eq!(sensor, SensorType::Other(other_id, String::from("unknown")));
/// ```

pub const SENSOR_BINSIZE: usize = 1000;
pub const ACCELEROMETER_OFFSET: usize = 0;
pub const GYROSCOPE_OFFSET: usize = SENSOR_BINSIZE;
pub const MAGNETOMETER_OFFSET: usize = SENSOR_BINSIZE * 2;
pub const OTHER_OFFSET: usize = SENSOR_BINSIZE * 3;
pub const MAX_OFFSET: usize = SENSOR_BINSIZE * 4;

#[derive(Clone, Debug, PartialEq, PartialOrd, Hash, Eq)]
pub enum SensorType {
    Accelerometer(Uuid),
    Gyroscope(Uuid),
    Magnetometer(Uuid),
    Other(Uuid, String),
}

impl From<&SensorType> for usize {
    fn from(value: &SensorType) -> Self {
        let uuid_to_usize = |uuid: &Uuid| {
            uuid.to_bytes_le()
                .iter()
                .map(|&b| b as usize)
                .sum::<usize>()
        };
        match value {
            SensorType::Accelerometer(uuid) => uuid_to_usize(uuid) % SENSOR_BINSIZE,
            SensorType::Gyroscope(uuid) => GYROSCOPE_OFFSET + uuid_to_usize(uuid) % SENSOR_BINSIZE,
            SensorType::Magnetometer(uuid) => {
                MAGNETOMETER_OFFSET + uuid_to_usize(uuid) % SENSOR_BINSIZE
            }
            SensorType::Other(uuid, _) => OTHER_OFFSET + uuid_to_usize(uuid) % SENSOR_BINSIZE,
        }
    }
}

impl From<SensorType> for usize {
    fn from(value: SensorType) -> Self {
        usize::from(&value)
    }
}

fn extract_sensor_and_id(value: &str) -> Result<(&str, &str), &'static str> {
    if let Some(index) = value.find("::") {
        let (part1, part2) = value.split_at(index);
        if !part2.is_empty() {
            let part2 = &part2[2..]; // Skip the "::" part
            Ok((part1, part2))
        } else {
            Err("Invalid format: No second part after '::'")
        }
    } else {
        Err("Invalid format: Missing '::'")
    }
}

fn get_sensor_id(id: &str) -> Result<Uuid, String> {
    id.parse::<Uuid>().map_err(|e| e.to_string())
}

impl TryFrom<&str> for SensorType {
    type Error = String;

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        let lower_case_value = value.to_lowercase();
        match extract_sensor_and_id(&lower_case_value) {
            Ok((sensor_type, id)) => {
                let id = get_sensor_id(id)?;

                if sensor_type.contains("acc") {
                    Ok(Self::Accelerometer(id))
                } else if sensor_type.contains("gyr") {
                    Ok(Self::Gyroscope(id))
                } else if sensor_type.contains("mag") {
                    Ok(Self::Magnetometer(id))
                } else {
                    Ok(Self::Other(id, sensor_type.to_string()))
                }
            }
            Err(e) => Err(e.to_string()),
        }
    }
}

impl TryFrom<String> for SensorType {
    type Error = String;

    fn try_from(value: String) -> Result<Self, Self::Error> {
        SensorType::try_from(value.as_str())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_from_usize() {
        let acc_id = Uuid::new_v4();
        let gyro_id = Uuid::new_v4();
        let mag_id = Uuid::new_v4();
        let other_id = Uuid::new_v4();
        assert!(usize::from(SensorType::Accelerometer(acc_id)) < GYROSCOPE_OFFSET);
        assert!(
            usize::from(SensorType::Gyroscope(gyro_id)) >= GYROSCOPE_OFFSET
                && usize::from(SensorType::Gyroscope(gyro_id)) < MAGNETOMETER_OFFSET
        );
        assert!(
            usize::from(SensorType::Magnetometer(mag_id)) >= MAGNETOMETER_OFFSET
                && usize::from(SensorType::Magnetometer(mag_id)) < OTHER_OFFSET
        );
        assert!(
            usize::from(SensorType::Other(other_id, String::from("other"))) >= OTHER_OFFSET
                && usize::from(SensorType::Other(other_id, String::from("other"))) < MAX_OFFSET
        );
    }
    #[test]
    fn test_from_str() {
        let acc_id = Uuid::new_v4();
        let gyro_id = Uuid::new_v4();
        let mag_id = Uuid::new_v4();
        let other_id = Uuid::new_v4();
        assert_eq!(
            SensorType::try_from(format!("accelerometer::{}", acc_id).as_str()).unwrap(),
            SensorType::Accelerometer(acc_id)
        );
        assert_eq!(
            SensorType::try_from(format!("gyroscope::{}", gyro_id).as_str()).unwrap(),
            SensorType::Gyroscope(gyro_id)
        );
        assert_eq!(
            SensorType::try_from(format!("magnetometer::{}", mag_id).as_str()).unwrap(),
            SensorType::Magnetometer(mag_id)
        );
        assert_eq!(
            SensorType::try_from(format!("other::{}", other_id).as_str()).unwrap(),
            SensorType::Other(other_id, String::from("other"))
        );
    }

    #[test]
    fn test_from_str_case_insensitive() {
        let acc_id = Uuid::new_v4();
        let gyro_id = Uuid::new_v4();
        let mag_id = Uuid::new_v4();
        let other_id = Uuid::new_v4();
        assert_eq!(
            SensorType::try_from(format!("ACCEleroMeter::{}", acc_id).as_str()).unwrap(),
            SensorType::Accelerometer(acc_id)
        );
        assert_eq!(
            SensorType::try_from(format!("gyrosCOPE::{}", gyro_id).as_str()).unwrap(),
            SensorType::Gyroscope(gyro_id)
        );
        assert_eq!(
            SensorType::try_from(format!("magneTometer::{}", mag_id).as_str()).unwrap(),
            SensorType::Magnetometer(mag_id)
        );
        assert_eq!(
            SensorType::try_from(format!("oThEr::{}", other_id).as_str()).unwrap(),
            SensorType::Other(other_id, String::from("other"))
        );
    }

    #[test]
    fn test_from_str_partial_match() {
        let acc_id = Uuid::new_v4();
        let gyro_id = Uuid::new_v4();
        let mag_id = Uuid::new_v4();
        let other_id = Uuid::new_v4();
        assert_eq!(
            SensorType::try_from(format!("acC::{}", acc_id).as_str()).unwrap(),
            SensorType::Accelerometer(acc_id)
        );
        assert_eq!(
            SensorType::try_from(format!("GyR::{}", gyro_id).as_str()).unwrap(),
            SensorType::Gyroscope(gyro_id)
        );
        assert_eq!(
            SensorType::try_from(format!("Mag::{}", mag_id).as_str()).unwrap(),
            SensorType::Magnetometer(mag_id)
        );
        assert_eq!(
            SensorType::try_from(format!("oThEr::{}", other_id).as_str()).unwrap(),
            SensorType::Other(other_id, String::from("other"))
        );
    }
}
