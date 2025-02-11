//! General functionality for `imu-rs` library

#[doc(hidden)]
pub mod traits;
#[doc(hidden)]
pub mod types;

// Re-export traits
#[doc(inline)]
pub use traits::imu::{IMUFilter, IMUReadings, IMUSample, IMUUntimedSample};
