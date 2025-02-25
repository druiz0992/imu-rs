pub mod box3d;

pub use box3d::Box3D;

pub trait Renderable3D {
    fn vertices(&self) -> Vec<(f64, f64, f64)>;
    fn edges(&self) -> Vec<(usize, usize)>;
}

pub trait RigidBody {
    fn rotate(&self, q: &nalgebra::Quaternion<f64>) -> Vec<(f64, f64, f64)>;
    fn translate(&self, v: &nalgebra::Vector3<f64>) -> Vec<(f64, f64, f64)>;
}
