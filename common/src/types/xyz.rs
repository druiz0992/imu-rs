use nalgebra::Vector3;

use std::ops::{Add, AddAssign, Div, Mul, Sub, SubAssign};

use crate::constants::N_XYZ_COORDINATES;
use crate::{IMUSample, IMUUntimedSample};

#[derive(Clone, Debug, PartialEq, PartialOrd, Default)]
pub struct XYZ(Vector3<f64>);

impl XYZ {
    pub fn new(data: [f64; N_XYZ_COORDINATES]) -> Self {
        Self(Vector3::from_vec(data.to_vec()))
    }

    pub fn from_vec(data: Vec<f64>) -> Option<Self> {
        if data.len() != N_XYZ_COORDINATES {
            return None;
        }
        Some(Self(Vector3::from_vec(data.to_vec())))
    }

    pub fn inner(&self) -> [f64; N_XYZ_COORDINATES] {
        [self.0.x, self.0.y, self.0.z]
    }
}

impl IMUUntimedSample for XYZ {
    fn get_measurement(&self) -> Vec<f64> {
        vec![self.0.x, self.0.y, self.0.z]
    }

    fn from_timed(timed_samples: Vec<f64>) -> Option<Self> {
        XYZ::from_vec(timed_samples)
    }
}
impl Add for XYZ {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Sub for XYZ {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl Div<f64> for XYZ {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        Self(self.0 / rhs)
    }
}

impl AddAssign for XYZ {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0
    }
}

impl SubAssign for XYZ {
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0
    }
}

impl Mul<f64> for XYZ {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self(self.0 * rhs)
    }
}

impl<T: IMUSample> From<T> for XYZ {
    fn from(value: T) -> Self {
        Self::from_timed(value.get_measurement()).unwrap_or_default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let data = [1.0, 2.0, 3.0];
        let xyz = XYZ::new(data);
        assert_eq!(xyz.inner(), data);
    }

    #[test]
    fn test_add() {
        let xyz1 = XYZ::new([1.0, 2.0, 3.0]);
        let xyz2 = XYZ::new([4.0, 5.0, 6.0]);
        let result = xyz1 + xyz2;
        assert_eq!(result.inner(), [5.0, 7.0, 9.0]);
    }

    #[test]
    fn test_sub() {
        let xyz1 = XYZ::new([4.0, 5.0, 6.0]);
        let xyz2 = XYZ::new([1.0, 2.0, 3.0]);
        let result = xyz1 - xyz2;
        assert_eq!(result.inner(), [3.0, 3.0, 3.0]);
    }

    #[test]
    fn test_div() {
        let xyz = XYZ::new([4.0, 6.0, 8.0]);
        let result = xyz / 2.0;
        assert_eq!(result.inner(), [2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_add_assign() {
        let mut xyz1 = XYZ::new([1.0, 2.0, 3.0]);
        let xyz2 = XYZ::new([4.0, 5.0, 6.0]);
        xyz1 += xyz2;
        assert_eq!(xyz1.inner(), [5.0, 7.0, 9.0]);
    }

    #[test]
    fn test_sub_assign() {
        let mut xyz1 = XYZ::new([4.0, 5.0, 6.0]);
        let xyz2 = XYZ::new([1.0, 2.0, 3.0]);
        xyz1 -= xyz2;
        assert_eq!(xyz1.inner(), [3.0, 3.0, 3.0]);
    }
}
