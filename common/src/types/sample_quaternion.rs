use crate::{constants::*, IMUSample};

#[derive(Debug, Clone, Default, PartialEq, PartialOrd)]
pub struct SampleQuaternion {
    timestamp: f64,
    quaternion: [f64; N_QUATERNION_COORDINATES],
}

impl SampleQuaternion {
    pub fn new(timestamp: f64, measurement: [f64; N_QUATERNION_COORDINATES]) -> Self {
        Self {
            timestamp,
            quaternion: [
                measurement[0],
                measurement[1],
                measurement[2],
                measurement[3],
            ],
        }
    }

    pub fn from_measurement(timestamp: f64, measurement: [f64; N_QUATERNION_COORDINATES]) -> Self {
        Self {
            timestamp,
            quaternion: measurement,
        }
    }

    pub fn from_vec(sample_vec: [f64; N_QUATERNION_COORDINATES + 1]) -> Self {
        Self {
            timestamp: sample_vec[0],
            quaternion: [sample_vec[1], sample_vec[2], sample_vec[3], sample_vec[4]],
        }
    }
}

impl IMUSample for SampleQuaternion {
    fn get_measurement(&self) -> Vec<f64> {
        self.quaternion.to_vec()
    }

    fn get_timestamp(&self) -> f64 {
        self.timestamp
    }

    fn from_untimed(sample: Vec<f64>, timestamp: f64) -> Self {
        let measurement = [sample[0], sample[1], sample[2], sample[3]];
        SampleQuaternion::from_measurement(timestamp, measurement)
    }
}

impl TryFrom<Vec<f64>> for SampleQuaternion {
    type Error = String;

    fn try_from(value: Vec<f64>) -> Result<Self, Self::Error> {
        if value.len() != N_QUATERNION_COORDINATES + 1 {
            return Err("Invalid length of input vector".to_string());
        }
        Ok(SampleQuaternion::from_vec([
            value[0], value[1], value[2], value[3], value[4],
        ]))
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
        assert_eq!(sample.get_measurement(), [1.0, 2.0, 3.0, 3.0]);
    }

    #[test]
    fn test_get_measurement() {
        let timestamp = 1627846267.0;
        let measurement = [1.0, 2.0, 3.0, 4.0];
        let sample = SampleQuaternion::new(timestamp, measurement);

        assert_eq!(sample.get_measurement(), measurement);
    }

    #[test]
    fn test_get_timestamp() {
        let timestamp = 1627846267.0;
        let measurement = [1.0, 2.0, 3.0, 4.0];
        let sample = SampleQuaternion::new(timestamp, measurement);

        assert_eq!(sample.get_timestamp(), timestamp);
    }
}
