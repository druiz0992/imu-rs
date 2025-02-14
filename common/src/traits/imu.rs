use async_trait::async_trait;
use std::ops::{Add, AddAssign, Div, Mul, Sub, SubAssign};
use std::sync::Arc;
use uuid::Uuid;

use crate::traits::Notifiable;
use crate::types::sensors::SensorType;

pub trait VecF64Convertible: Into<Vec<f64>> + TryFrom<Vec<f64>> + Sized {}
impl<T: Into<Vec<f64>> + TryFrom<Vec<f64>> + Sized> VecF64Convertible for T {}

pub trait BasicArithmetic:
    Sized
    + AddAssign
    + Add<Output = Self>
    + SubAssign
    + Sub<Output = Self>
    + Div<f64, Output = Self>
    + Mul<f64, Output = Self>
{
}

/// Untimed sample from an IMU (Inertial Measurement Unit).
pub trait IMUUntimedSample: Send + Sync + Clone + Default + 'static {
    ///  Returns the measurement data as a vector of `f64` values.
    fn get_measurement(&self) -> Self;
}

/// Timed sample from an IMU (Inertial Measurement Unit).
pub trait IMUSample: Send + Sync + Clone + Default + 'static {
    type Untimed: IMUUntimedSample;

    ///  Returns the timestamp of the sample.
    fn get_timestamp_secs(&self) -> f64;
    ///  Returns the measurement data
    fn get_measurement(&self) -> Self::Untimed;
    /// Returns a IMUSample
    fn from_measurement(timestamp: f64, measurement: Self::Untimed) -> Self;
}

/// Collection of sensor readings from an IMU (Inertial Measurement Unit).
pub trait IMUReadings<T: IMUSample>: Send + Sync + Clone {
    ///   Returns the sensor tag
    fn get_sensor_tag(&self) -> &str;
    fn get_sensor_type(&self) -> SensorType;
    ///   Returns samples
    fn get_samples(&self) -> Vec<T>;
    ///   Adds new samples
    fn extend(&mut self, elems: Vec<T>);
    ///   Creates new IMUReadings
    fn from_vec(tag: &str, readings_type: SensorType, data: Vec<T>) -> Self;
    ///   Clears stored samples
    fn clear(&mut self);
}

pub trait IMUFilter<T>: Send + Sync
where
    T: IMUSample,
{
    ///  Returns the resampled samples
    fn filter_batch(&mut self, samples: Vec<T>) -> Result<Vec<T>, &str>;
}

#[async_trait]
pub trait IMUSource<T, S>: Send + Sync
where
    T: Send + Sync + IMUReadings<S>,
    S: Send + Sync + IMUSample,
{
    fn get_tag(&self) -> &str;
    async fn get_available_sensors(&self) -> Result<Vec<SensorType>, String>;
    async fn unregister_listener(&self, id: Uuid);
    async fn register_listener(
        &self,
        listener: &mut dyn Notifiable<T>,
        sensor_type: &SensorType,
    ) -> Result<Uuid, String>;
    async fn notify_listeners(&self, sensor_type: SensorType, data: Arc<T>);
}

#[async_trait]
pub trait IMUSink<T, S>: Send + Sync
where
    T: Send + Sync + IMUReadings<S>,
    S: Send + Sync + IMUSample,
{
    async fn attach_listener(
        &self,
        source: &dyn IMUSource<T, S>,
        sensor_type: &SensorType,
    ) -> Result<Uuid, String>;
    async fn detach_listener(&self, source: &dyn IMUSource<T, S>, id: Uuid) {
        source.unregister_listener(id).await;
    }

    async fn process_samples(&self, listener_id: Uuid, samples: Arc<T>);
}
