use nalgebra::UnitQuaternion as NUnitQuaternion;

use crate::IMUUntimedSample;

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
