use crate::SensorType;
use std::iter::Iterator;
use std::ops::{Add, AddAssign, Div, Mul, Sub, SubAssign};

/// Untimed sample from an IMU (Inertial Measurement Unit).
pub trait IMUUntimedSample:
    Send
    + Sync
    + Clone
    + Default
    + Add<Output = Self>
    + AddAssign
    + Div<f64, Output = Self>
    + Mul<f64, Output = Self>
    + Sub<Output = Self>
    + SubAssign
    + 'static
{
    ///  Returns the measurement data as a vector of `f64` values.
    fn get_measurement(&self) -> Vec<f64>;
    fn from_timed(timed_samples: Vec<f64>) -> Option<Self>;
}

/// Timed sample from an IMU (Inertial Measurement Unit).
pub trait IMUSample: Send + Sync + Clone + Default + 'static {
    ///  Returns the timestamp of the sample.
    fn get_timestamp(&self) -> f64;
    ///  Returns the measurement data
    fn get_measurement(&self) -> Vec<f64>;
    /// Returns a IMUSample
    fn from_untimed(sample: Vec<f64>, timestamp: f64) -> Self;
}

/// Collection of sensor readings from an IMU (Inertial Measurement Unit).
pub trait IMUReadings<T: IMUSample>: Send + Sync + Clone {
    ///   Returns the sensor tag
    fn get_sensor_tag(&self) -> &str;
    ///   Returns a slive to samples.
    fn get_samples_ref(&self) -> &[T];
    ///   Returns samples
    fn get_samples(&self) -> Vec<T>;
    ///   Adds new samples
    fn extend(&mut self, elems: Vec<T>);
    ///   Creates new IMUReadings
    fn from_vec(tag: &str, readings_type: SensorType, data: Vec<T>) -> Self;
    ///   Clears stored samples
    fn clear(&mut self);
    ///   Returns an iterator over references to the samples.
    fn iter_samples(&self) -> impl Iterator<Item = &T> {
        self.get_samples_ref().iter()
    }
    ///   Returns an iterator over samples.
    #[allow(clippy::wrong_self_convention)]
    fn into_iter_samples(&self) -> impl Iterator<Item = T> {
        self.get_samples().into_iter()
    }
}

/// Resampling for IMU (Inertial Measurement Unit) samples.
pub trait IMUResampler<T: IMUSample>: Send + Sync {
    ///  Returns the resampled samples
    fn resample(&self, samples: Vec<T>, sampling_time: f64, cache: &mut Vec<T>) -> Vec<T>;
}

/// Filtering for IMU (Inertial Measurement Unit) samples.
pub trait IMUFilter<T: IMUUntimedSample>: Send + Sync {
    /// Returns the filtered samples
    fn filter(&mut self, samples: Vec<T>) -> Vec<T>;
}

/*
pub trait IMUSource {
    fn get_tag(&self) -> &str;
    fn get_available_sensors(&self, sensor_type: SensorType) -> bool;
    async fn register_listener(
        &self,
        mut listener: Listener<T>,
        sensor_type: SensorType,
    ) -> Uuid;
    fn unregister_listener(&self)
}
    */
