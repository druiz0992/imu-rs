use async_trait::async_trait;
use gnuplot::{AxesCommon, Color, Figure};
use std::sync::Arc;
use tokio::sync::Mutex;
use uuid::Uuid;

use common::traits::{IMUReadings, IMUSample, IMUSink, IMUSource};
use common::types::sensors::{SensorReadings, SensorType};
use common::types::timed::{Sample3D, SampleQuaternion};
use publisher::listener;
use publisher::AsyncListener;

use crate::renderable::{Renderable3D, RigidBody};

#[derive(Clone)]
pub struct Plot3D<T>
where
    T: Renderable3D + RigidBody,
{
    fg: Arc<Mutex<Figure>>,
    object_3d: T,
}

impl<T> Plot3D<T>
where
    T: Renderable3D + RigidBody,
{
    pub fn new(object_3d: T) -> Self {
        let fg = Figure::new();

        Self {
            fg: Arc::new(Mutex::new(fg)),
            object_3d,
        }
    }

    async fn clear_axes(&self) {
        let mut fg = self.fg.lock().await;
        fg.clear_axes();
    }

    async fn set_grid(&self, flag: bool) {
        let mut fg = self.fg.lock().await;
        fg.clear_axes()
            .axes3d()
            .set_x_grid(flag)
            .set_y_grid(flag)
            .set_z_grid(flag)
            .set_x_ticks(None, &[], &[]);
    }

    pub async fn clear(&self) {
        self.clear_axes().await;
        self.set_grid(false).await;
    }

    pub async fn update(&self, vertices: &[(f64, f64, f64)]) {
        let mut fg = self.fg.lock().await;
        fg.clear_axes();
        let ax = fg
            .axes3d()
            .set_x_grid(false) // Disable grid for x-axis
            .set_y_grid(false) // Disable grid for y-axis
            .set_z_grid(false);
        let edges = self.object_3d.edges();

        for &(i, j) in edges.iter() {
            ax.lines(
                [vertices[i].0, vertices[j].0],
                [vertices[i].1, vertices[j].1],
                [vertices[i].2, vertices[j].2],
                &[Color("blue")],
            );
        }

        fg.show_and_keep_running().unwrap();
    }
}

#[async_trait]
impl<R> IMUSink<SensorReadings<SampleQuaternion>, SampleQuaternion> for Plot3D<R>
where
    R: Renderable3D + RigidBody + Send + Sync + Clone + 'static,
{
    async fn attach_listeners(
        &self,
        source: &dyn IMUSource<SensorReadings<SampleQuaternion>, SampleQuaternion>,
        sensor_cluster: &[SensorType],
    ) -> Result<Vec<Uuid>, String> {
        let mut listener = listener!(self.process_samples);
        let mut ids = Vec::with_capacity(sensor_cluster.len());
        for sensor_type in sensor_cluster {
            match source.register_listener(&mut listener, sensor_type).await {
                Ok(id) => {
                    ids.push(id);
                }
                Err(e) => return Err(e),
            }
        }
        return Ok(ids);
    }
    async fn detach_listener(
        &self,
        _source: &dyn IMUSource<SensorReadings<SampleQuaternion>, SampleQuaternion>,
        _id: Uuid,
    ) {
        todo!();
    }

    async fn process_samples(&self, _id: Uuid, samples: Arc<SensorReadings<SampleQuaternion>>) {
        if let Some(q) = samples.get_samples().first() {
            let q = q.get_measurement().inner();
            let rotated_vertices = self.object_3d.rotate(&q);
            self.update(&rotated_vertices).await;
        }
    }
}

#[async_trait]
impl<R> IMUSink<SensorReadings<Sample3D>, Sample3D> for Plot3D<R>
where
    R: Renderable3D + RigidBody + Send + Sync + Clone + 'static,
{
    async fn attach_listeners(
        &self,
        source: &dyn IMUSource<SensorReadings<Sample3D>, Sample3D>,
        sensor_cluster: &[SensorType],
    ) -> Result<Vec<Uuid>, String> {
        let mut listener = listener!(self.process_samples);
        let mut ids = Vec::with_capacity(sensor_cluster.len());
        for sensor_type in sensor_cluster {
            match source.register_listener(&mut listener, sensor_type).await {
                Ok(id) => {
                    ids.push(id);
                }
                Err(e) => return Err(e),
            }
        }
        return Ok(ids);
    }
    async fn detach_listener(
        &self,
        _source: &dyn IMUSource<SensorReadings<Sample3D>, Sample3D>,
        _id: Uuid,
    ) {
        todo!();
    }

    async fn process_samples(&self, _id: Uuid, samples: Arc<SensorReadings<Sample3D>>) {
        if let Some(acc) = samples.get_samples().first() {
            let acc = nalgebra::Vector3::from_vec(acc.get_measurement().inner().to_vec());
            let traslated_vertices = self.object_3d.translate(&acc);
            self.update(&traslated_vertices).await;
        }
    }
}
