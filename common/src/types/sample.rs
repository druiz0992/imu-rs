use crate::{constants::*, types::xyz::XYZ, IMUSample};

#[derive(Debug, Clone, Default, PartialEq, PartialOrd)]
pub struct Sample3D {
    timestamp: f64,
    measurement: XYZ,
}

impl Sample3D {
    pub fn new(timestamp: f64, measurement: [f64; N_XYZ_COORDINATES]) -> Self {
        Self {
            timestamp,
            measurement: XYZ::new(measurement),
        }
    }

    pub fn from_xyz(timestamp: f64, measurement: XYZ) -> Self {
        Self {
            timestamp,
            measurement,
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
