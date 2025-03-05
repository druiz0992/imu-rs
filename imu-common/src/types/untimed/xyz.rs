use nalgebra::Vector3;
#[cfg(any(feature = "serde-serialize", test))]
use serde::{Deserialize, Deserializer, Serialize, Serializer};
#[cfg(any(feature = "serde-serialize", test))]
use serde_json::Value;

use std::ops::{Add, AddAssign, Div, Mul, Sub, SubAssign};

use crate::traits::imu::BasicArithmetic;
use crate::traits::IMUUntimedSample;

pub const N_XYZ_COORDINATES: usize = 3;

#[derive(Clone, Debug, PartialEq, PartialOrd, Default)]
pub struct XYZ(pub Vector3<f64>);

impl XYZ {
    pub fn new(data: [f64; N_XYZ_COORDINATES]) -> Self {
        Self(Vector3::from_vec(data.to_vec()))
    }

    pub fn from_vector(data: Vector3<f64>) -> Self {
        Self(data)
    }

    pub fn inner(&self) -> [f64; N_XYZ_COORDINATES] {
        [self.0.x, self.0.y, self.0.z]
    }
}

impl IMUUntimedSample for XYZ {
    fn get_measurement(&self) -> Self {
        self.clone()
    }
}

impl From<XYZ> for [f64; N_XYZ_COORDINATES] {
    fn from(value: XYZ) -> Self {
        value.inner()
    }
}

impl From<[f64; N_XYZ_COORDINATES]> for XYZ {
    fn from(value: [f64; N_XYZ_COORDINATES]) -> Self {
        Self(Vector3::from(value))
    }
}

impl From<XYZ> for Vec<f64> {
    fn from(value: XYZ) -> Self {
        value.inner().to_vec()
    }
}

impl TryFrom<Vec<f64>> for XYZ {
    type Error = &'static str;

    fn try_from(value: Vec<f64>) -> Result<Self, Self::Error> {
        if value.len() != N_XYZ_COORDINATES {
            return Err("Can't convert to XYZ");
        }
        Ok(Self(Vector3::from_vec(value)))
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
impl BasicArithmetic for XYZ {}

#[cfg(any(feature = "serde-serialize", test))]
impl Serialize for XYZ {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let vec = self.0;
        let json = serde_json::json!({
            "x": vec.x,
            "y": vec.y,
            "z": vec.z
        });
        json.serialize(serializer)
    }
}

#[cfg(any(feature = "serde-serialize", test))]
impl<'de> Deserialize<'de> for XYZ {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        // Deserialize into a Value (serde_json::Value)
        let value: Value = Value::deserialize(deserializer)?;

        // Handle array format [f64, f64, f64]
        if let Some(arr) = value.as_array() {
            if arr.len() == 3 {
                let x = arr[0].as_f64().unwrap_or_default();
                let y = arr[1].as_f64().unwrap_or_default();
                let z = arr[2].as_f64().unwrap_or_default();
                return Ok(XYZ(Vector3::new(x, y, z)));
            }
        }

        // Handle object format {"x": f64, "y": f64, "z": f64}
        if let Some(obj) = value.as_object() {
            let x = obj.get("x").and_then(Value::as_f64).unwrap_or_default();
            let y = obj.get("y").and_then(Value::as_f64).unwrap_or_default();
            let z = obj.get("z").and_then(Value::as_f64).unwrap_or_default();
            return Ok(XYZ(Vector3::new(x, y, z)));
        }

        // Handle string format "0.0, 1.0, 2.0" (comma-separated values)
        if let Some(scalar_str) = value.as_str() {
            let parts: Vec<f64> = scalar_str
                .split(',')
                .filter_map(|s| s.trim().parse().ok())
                .collect();
            if parts.len() == 3 {
                return Ok(XYZ(Vector3::new(parts[0], parts[1], parts[2])));
            }
        }

        // Fallback to a default value if nothing else matches
        Ok(XYZ(Vector3::default()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(any(feature = "serde-serialize", test))]
    use serde_json;

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

    #[test]
    fn test_mul() {
        let xyz = XYZ::new([1.0, 2.0, 3.0]);
        let result = xyz * 2.0;
        assert_eq!(result.inner(), [2.0, 4.0, 6.0]);
    }

    #[test]
    fn test_try_from_vec() {
        let vec = vec![1.0, 2.0, 3.0];
        let xyz = XYZ::try_from(vec).unwrap();
        assert_eq!(xyz.inner(), [1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_try_from_vec_invalid_length() {
        let vec = vec![1.0, 2.0];
        let result = XYZ::try_from(vec);
        assert!(result.is_err());
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_serialize() {
        let xyz = XYZ::new([1.0, 2.0, 3.0]);
        let serialized = serde_json::to_string(&xyz).unwrap();
        assert_eq!(serialized, r#"{"x":1.0,"y":2.0,"z":3.0}"#);
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_deserialize() {
        let data = r#"{"x": 1.0, "y":2.0,"z":3.0}"#;
        let xyz: XYZ = serde_json::from_str(data).unwrap();
        assert_eq!(xyz.inner(), [1.0, 2.0, 3.0]);
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_deserialize_missing_fields() {
        let data = r#"{"x":1.0,  "z": 3.0}"#;
        let xyz: XYZ = serde_json::from_str(data).unwrap();
        assert_eq!(xyz.inner(), [1.0, 0.0, 3.0]);
    }

    #[cfg(any(feature = "serde-serialize", test))]
    #[test]
    fn test_deserialize_missing_labels() {
        let data = r#""1.0,   2.0,3.0""#;
        let xyz: XYZ = serde_json::from_str(data).unwrap();
        assert_eq!(xyz.inner(), [1.0, 2.0, 3.0]);
    }
}
