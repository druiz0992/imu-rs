use std::ops::{Add, AddAssign, Div, Mul, Sub, SubAssign};

use crate::traits::imu::BasicArithmetic;
use crate::traits::IMUUntimedSample;

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
