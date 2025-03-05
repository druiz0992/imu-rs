use crate::traits::imu::BasicArithmetic;
use crate::traits::IMUUntimedSample;

#[cfg(any(feature = "serde-serialize", test))]
use serde::{Deserialize, Serialize};

use std::ops::{Add, AddAssign, Div, Mul, Sub, SubAssign};

#[cfg_attr(any(feature = "serde-serialize", test), derive(Serialize, Deserialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd, Default)]
pub struct Scalar(f64);

impl Scalar {
    pub fn new(data: f64) -> Self {
        Self(data)
    }

    pub fn inner(&self) -> f64 {
        self.0
    }
}

impl IMUUntimedSample for Scalar {
    fn get_measurement(&self) -> Self {
        self.clone()
    }
}

impl Add for Scalar {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Sub for Scalar {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl Div<f64> for Scalar {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        Self(self.0 / rhs)
    }
}

impl AddAssign for Scalar {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0
    }
}

impl SubAssign for Scalar {
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0
    }
}

impl Mul<f64> for Scalar {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self(self.0 * rhs)
    }
}

impl From<Scalar> for f64 {
    fn from(value: Scalar) -> Self {
        value.inner()
    }
}

impl From<f64> for Scalar {
    fn from(value: f64) -> Self {
        Self(value)
    }
}
impl BasicArithmetic for Scalar {}

#[cfg(test)]
mod tests {

    use super::*;
    #[cfg(any(feature = "serde-serialize", test))]
    use serde_json;
    #[test]
    fn test_scalar_new() {
        let scalar = Scalar::new(5.0);
        assert_eq!(scalar.inner(), 5.0);
    }

    #[test]
    fn test_scalar_add() {
        let scalar1 = Scalar::new(2.0);
        let scalar2 = Scalar::new(3.0);
        let result = scalar1 + scalar2;
        assert_eq!(result.inner(), 5.0);
    }

    #[test]
    fn test_scalar_sub() {
        let scalar1 = Scalar::new(5.0);
        let scalar2 = Scalar::new(3.0);
        let result = scalar1 - scalar2;
        assert_eq!(result.inner(), 2.0);
    }

    #[test]
    fn test_scalar_mul() {
        let scalar = Scalar::new(2.0);
        let result = scalar * 3.0;
        assert_eq!(result.inner(), 6.0);
    }

    #[test]
    fn test_scalar_div() {
        let scalar = Scalar::new(6.0);
        let result = scalar / 2.0;
        assert_eq!(result.inner(), 3.0);
    }

    #[test]
    fn test_scalar_add_assign() {
        let mut scalar1 = Scalar::new(2.0);
        let scalar2 = Scalar::new(3.0);
        scalar1 += scalar2;
        assert_eq!(scalar1.inner(), 5.0);
    }

    #[test]
    fn test_scalar_sub_assign() {
        let mut scalar1 = Scalar::new(5.0);
        let scalar2 = Scalar::new(3.0);
        scalar1 -= scalar2;
        assert_eq!(scalar1.inner(), 2.0);
    }

    #[test]
    fn test_scalar_from_f64() {
        let scalar: Scalar = 5.0.into();
        assert_eq!(scalar.inner(), 5.0);
    }

    #[test]
    fn test_scalar_into_f64() {
        let scalar = Scalar::new(5.0);
        let value: f64 = scalar.into();
        assert_eq!(value, 5.0);
    }

    #[test]
    fn test_get_measurement() {
        let scalar = Scalar::new(5.0);
        let measurement = scalar.get_measurement();
        assert_eq!(measurement.inner(), 5.0);
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_scalar_serialize() {
        let scalar = Scalar::new(5.0);
        let serialized = serde_json::to_string(&scalar).unwrap();
        assert_eq!(serialized, "5.0");
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_scalar_deserialize() {
        let serialized = "5.0";
        let deserialized: Scalar = serde_json::from_str(serialized).unwrap();
        assert_eq!(deserialized.inner(), 5.0);
    }
}
