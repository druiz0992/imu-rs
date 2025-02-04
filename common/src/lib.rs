//! General functionality for `imu-rs` library

pub mod constants;

#[doc(hidden)]
pub mod traits;
#[doc(hidden)]
pub mod types;

// Re-export traits
#[doc(inline)]
pub use traits::imu::{IMUFilter, IMUReadings, IMUResampler, IMUSample, IMUUntimedSample};

// Re-export types
#[doc(inline)]
pub use types::{buffers, Sample3D, SensorReadings, SensorTag, SensorType, XYZ};
