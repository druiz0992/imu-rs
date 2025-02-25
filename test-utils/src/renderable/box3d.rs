#![allow(dead_code)]

use super::{Renderable3D, RigidBody};
use nalgebra::{Quaternion, Vector3};

const N_BOX_VERTICES: usize = 8;
const N_BOX_EDGES: usize = 12;

#[derive(Debug, Clone, Copy, Default)]
pub struct Edge(usize, usize);

impl Edge {
    fn new(from: usize, to: usize) -> Self {
        Self(from, to)
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Vertex(Vector3<f64>);

impl Vertex {
    fn new(vector3d: Vector3<f64>) -> Self {
        Self(vector3d)
    }
    fn x(&self) -> f64 {
        self.0.x
    }
    fn y(&self) -> f64 {
        self.0.y
    }
    fn z(&self) -> f64 {
        self.0.z
    }
    fn to_vector3(self) -> Vector3<f64> {
        self.0
    }
}

fn box_edges() -> [Edge; N_BOX_EDGES] {
    [
        Edge::new(0, 1),
        Edge::new(1, 2),
        Edge::new(2, 3),
        Edge::new(3, 0),
        Edge::new(4, 5),
        Edge::new(5, 6),
        Edge::new(6, 7),
        Edge::new(7, 4),
        Edge::new(0, 4),
        Edge::new(1, 5),
        Edge::new(2, 6),
        Edge::new(3, 7),
    ]
}
#[derive(Debug, Clone, Default)]
pub struct Box3D {
    vertices: [Vertex; N_BOX_VERTICES],
    edges: [Edge; N_BOX_EDGES],
}

impl Box3D {
    pub fn new() -> Self {
        let vertices = [
            Vertex::new(Vector3::new(0.0, 0.0, 0.0)),
            Vertex::new(Vector3::new(1.0, 0.0, 0.0)),
            Vertex::new(Vector3::new(1.0, 1.0, 0.0)),
            Vertex::new(Vector3::new(0.0, 1.0, 0.0)),
            Vertex::new(Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Vector3::new(1.0, 0.0, 1.0)),
            Vertex::new(Vector3::new(1.0, 1.0, 1.0)),
            Vertex::new(Vector3::new(0.0, 1.0, 1.0)),
        ];

        Self {
            vertices,
            edges: box_edges(),
        }
    }

    pub fn from_vertices(vertices: [Vertex; N_BOX_VERTICES]) -> Self {
        Self {
            vertices,
            edges: box_edges(),
        }
    }

    pub(crate) fn rotate(&self, quaternion: &Quaternion<f64>) -> [Vertex; N_BOX_VERTICES] {
        let mut rotated_box = Vec::with_capacity(N_BOX_VERTICES);
        for &v in self.vertices.iter() {
            let qv = Quaternion::new(0.0, v.x(), v.y(), v.z());
            let rotated_v = quaternion * qv * quaternion.conjugate();
            rotated_box.push(Vertex::new(Vector3::new(
                rotated_v.i,
                rotated_v.j,
                rotated_v.k,
            )));
        }
        rotated_box.try_into().unwrap()
    }

    pub(crate) fn translate(&self, translation_vector: &Vector3<f64>) -> [Vertex; N_BOX_VERTICES] {
        let mut rotated_box = Vec::with_capacity(N_BOX_VERTICES);
        for &v in self.vertices.iter() {
            let translated_v = v.to_vector3() + translation_vector;
            rotated_box.push(Vertex::new(translated_v));
        }
        rotated_box.try_into().unwrap()
    }

    pub(crate) fn rotate_and_translate(
        &self,
        quaternion: &Quaternion<f64>,
        translation_vector: &Vector3<f64>,
    ) -> [Vertex; N_BOX_VERTICES] {
        let rotated_box = Box3D::from_vertices(self.rotate(quaternion));
        rotated_box.translate(translation_vector)
    }
}

impl Renderable3D for Box3D {
    fn vertices(&self) -> Vec<(f64, f64, f64)> {
        let mut v_vector = Vec::with_capacity(N_BOX_VERTICES);
        for v in self.vertices {
            v_vector.push((v.0.x, v.0.y, v.0.z));
        }
        v_vector
    }

    fn edges(&self) -> Vec<(usize, usize)> {
        let mut v_edges = Vec::with_capacity(N_BOX_EDGES);
        for e in self.edges {
            v_edges.push((e.0, e.1))
        }
        v_edges
    }
}

impl RigidBody for Box3D {
    fn rotate(&self, q: &Quaternion<f64>) -> Vec<(f64, f64, f64)> {
        let rotated_box = self.rotate(q);
        let mut v_vector = Vec::with_capacity(N_BOX_VERTICES);
        for v in rotated_box {
            v_vector.push((v.x(), v.y(), v.z()))
        }
        v_vector
    }

    fn translate(&self, t: &Vector3<f64>) -> Vec<(f64, f64, f64)> {
        let tranlated_box = self.translate(t);
        let mut v_vector = Vec::with_capacity(N_BOX_VERTICES);
        for v in tranlated_box {
            v_vector.push((v.x(), v.y(), v.z()))
        }
        v_vector
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{UnitQuaternion, Vector3};

    #[test]
    fn test_box3d_creation() {
        let box3d = Box3D::new();
        assert_eq!(box3d.vertices.len(), N_BOX_VERTICES);
        assert_eq!(box3d.edges.len(), N_BOX_EDGES);
    }

    #[test]
    fn test_box3d_from_vertices() {
        let vertices = [
            Vertex::new(Vector3::new(0.0, 0.0, 0.0)),
            Vertex::new(Vector3::new(1.0, 0.0, 0.0)),
            Vertex::new(Vector3::new(1.0, 1.0, 0.0)),
            Vertex::new(Vector3::new(0.0, 1.0, 0.0)),
            Vertex::new(Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Vector3::new(1.0, 0.0, 1.0)),
            Vertex::new(Vector3::new(1.0, 1.0, 1.0)),
            Vertex::new(Vector3::new(0.0, 1.0, 1.0)),
        ];
        let box3d = Box3D::from_vertices(vertices);
        assert_eq!(box3d.vertices, vertices);
    }

    #[test]
    fn test_box3d_rotate() {
        let box3d = Box3D::new();
        let q = UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        let q = q.quaternion();
        let rotated_vertices = box3d.rotate(q);
        assert_ne!(box3d.vertices, rotated_vertices);
    }

    #[test]
    fn test_box3d_translate() {
        let box3d = Box3D::new();
        let translation_vector = Vector3::new(1.0, 1.0, 1.0);
        let translated_vertices = box3d.translate(&translation_vector);
        for (original, translated) in box3d.vertices.iter().zip(translated_vertices.iter()) {
            assert_eq!(
                translated.to_vector3(),
                original.to_vector3() + translation_vector
            );
        }
    }

    #[test]
    fn test_box3d_rotate_and_translate() {
        let box3d = Box3D::new();
        let q = UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        let q = q.quaternion();
        let translation_vector = Vector3::new(1.0, 1.0, 1.0);
        let transformed_vertices = box3d.rotate_and_translate(q, &translation_vector);
        assert_ne!(box3d.vertices, transformed_vertices);
    }
}
