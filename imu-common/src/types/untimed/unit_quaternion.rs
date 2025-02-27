use nalgebra::UnitQuaternion as NUnitQuaternion;

use crate::traits::IMUUntimedSample;

pub(crate) const W_QUATERNION_COORD_IDX: usize = 0;
pub(crate) const X_QUATERNION_COORD_IDX: usize = 1;
pub(crate) const Y_QUATERNION_COORD_IDX: usize = 2;
pub(crate) const Z_QUATERNION_COORD_IDX: usize = 3;

pub const N_QUATERNION_COORDINATES: usize = 4;
#[derive(Clone, Debug, PartialEq)]
pub struct UnitQuaternion(NUnitQuaternion<f64>);

impl UnitQuaternion {
    pub fn new(data: [f64; N_QUATERNION_COORDINATES]) -> Self {
        let quaternion = nalgebra::Quaternion::new(
            data[W_QUATERNION_COORD_IDX],
            data[X_QUATERNION_COORD_IDX],
            data[Y_QUATERNION_COORD_IDX],
            data[Z_QUATERNION_COORD_IDX],
        );
        UnitQuaternion::from_quaternion(quaternion)
    }

    pub fn from_quaternion(quaternion: nalgebra::Quaternion<f64>) -> Self {
        let unit_quaternion = NUnitQuaternion::from_quaternion(quaternion);
        UnitQuaternion::from_unit_quaternion(unit_quaternion)
    }

    pub fn from_unit_quaternion(unit_quaternion: nalgebra::UnitQuaternion<f64>) -> Self {
        Self(unit_quaternion)
    }

    pub fn inner(&self) -> NUnitQuaternion<f64> {
        self.0
    }
}

impl Default for UnitQuaternion {
    fn default() -> Self {
        UnitQuaternion::from_unit_quaternion(nalgebra::UnitQuaternion::default())
    }
}

impl IMUUntimedSample for UnitQuaternion {
    fn get_measurement(&self) -> Self {
        self.clone()
    }
}

impl From<UnitQuaternion> for [f64; N_QUATERNION_COORDINATES] {
    fn from(value: UnitQuaternion) -> Self {
        let unit_quaternion = value.inner();
        [
            unit_quaternion.w,
            unit_quaternion.i,
            unit_quaternion.j,
            unit_quaternion.k,
        ]
    }
}

impl From<[f64; N_QUATERNION_COORDINATES]> for UnitQuaternion {
    fn from(value: [f64; N_QUATERNION_COORDINATES]) -> Self {
        Self::new(value)
    }
}

impl From<UnitQuaternion> for Vec<f64> {
    fn from(value: UnitQuaternion) -> Self {
        <[f64; N_QUATERNION_COORDINATES]>::from(value).to_vec()
    }
}

impl TryFrom<Vec<f64>> for UnitQuaternion {
    type Error = &'static str;

    fn try_from(value: Vec<f64>) -> Result<Self, Self::Error> {
        let array: [f64; N_QUATERNION_COORDINATES] =
            value.try_into().map_err(|_| "Conversion failed")?;
        Ok(UnitQuaternion::new(array))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Quaternion, UnitQuaternion as NUnitQuaternion};

    #[test]
    fn test_unit_quaternion_new() {
        let data = [1.0, 0.0, 0.0, 0.0];
        let unit_quaternion = UnitQuaternion::new(data);
        assert_eq!(
            unit_quaternion.inner().quaternion(),
            &Quaternion::new(1.0, 0.0, 0.0, 0.0)
        );
    }

    #[test]
    fn test_default_quaternion_new() {
        let data = [1.0, 0.0, 0.0, 0.0];
        let default_quaternion = UnitQuaternion::default();
        let unit_quaternion = UnitQuaternion::new(data);
        assert_eq!(
            <[f64; 4]>::from(UnitQuaternion::from_unit_quaternion(
                NUnitQuaternion::<f64>::identity()
            )),
            data
        );
        assert_eq!(unit_quaternion, default_quaternion);
        assert_eq!(unit_quaternion.inner(), NUnitQuaternion::<f64>::identity());
    }

    #[test]
    fn test_unit_quaternion_from_quaternion() {
        let quaternion = Quaternion::new(1.0, 0.0, 0.0, 0.0);
        let unit_quaternion = UnitQuaternion::from_quaternion(quaternion);
        assert_eq!(
            unit_quaternion.inner().quaternion(),
            &Quaternion::new(1.0, 0.0, 0.0, 0.0)
        );
    }

    #[test]
    fn test_unit_quaternion_from_unit_quaternion() {
        let unit_quaternion =
            nalgebra::UnitQuaternion::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0));
        let unit_quaternion_wrapper = UnitQuaternion::from_unit_quaternion(unit_quaternion);
        assert_eq!(unit_quaternion_wrapper.inner(), unit_quaternion);
    }

    #[test]
    fn test_unit_quaternion_default() {
        let default_quaternion = UnitQuaternion::default();
        assert_eq!(
            default_quaternion.inner(),
            nalgebra::UnitQuaternion::default()
        );
    }

    #[test]
    fn test_unit_quaternion_get_measurement() {
        let data = [1.0, 0.0, 0.0, 0.0];
        let unit_quaternion = UnitQuaternion::new(data);
        let measurement = unit_quaternion.get_measurement();
        assert_eq!(measurement, unit_quaternion);
    }

    #[test]
    fn test_unit_quaternion_into_array() {
        let data = [1.0, 0.0, 0.0, 0.0];
        let unit_quaternion = UnitQuaternion::new(data);
        let array: [f64; N_QUATERNION_COORDINATES] = unit_quaternion.into();
        assert_eq!(array, data);
    }

    #[test]
    fn test_unit_quaternion_from_array() {
        let data = [1.0, 0.0, 0.0, 0.0];
        let unit_quaternion: UnitQuaternion = data.into();
        assert_eq!(
            unit_quaternion.inner().quaternion(),
            &Quaternion::new(1.0, 0.0, 0.0, 0.0)
        );
    }

    #[test]
    fn test_unit_quaternion_into_vec() {
        let data = [1.0, 0.0, 0.0, 0.0];
        let unit_quaternion = UnitQuaternion::new(data);
        let vec: Vec<f64> = unit_quaternion.into();
        assert_eq!(vec, data.to_vec());
    }

    #[test]
    fn test_unit_quaternion_try_from_vec_success() {
        let data = vec![1.0, 0.0, 0.0, 0.0];
        let unit_quaternion = UnitQuaternion::try_from(data.clone()).unwrap();
        assert_eq!(
            unit_quaternion.inner().quaternion(),
            &Quaternion::new(1.0, 0.0, 0.0, 0.0)
        );
    }

    #[test]
    fn test_unit_quaternion_try_from_vec_failure() {
        let data = vec![1.0, 0.0, 0.0];
        let result = UnitQuaternion::try_from(data);
        assert!(result.is_err());
    }
}
