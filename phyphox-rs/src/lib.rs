//! # Crate phyphox-rs
//!
//! ## phyphox-rs
//!
//! The `phyphox-rs` crate provides a convenient wrapper to interact with [`Phyphox`](https://phyphox.org/), a mobile phone app
//! that allows to capture sensor data from the smart phone. `phyphox-rs` interacts with a REST API enabled in the phone and
//! records the data from the selected sensors.
//!
//! Features include:
//! - Recording of 3-axis Accelerometer [m/s^2], Gyroscope [rad/s] and Magnetometer (&uT)
//! - Tagging sensors so that readings from different sensor placements can be distinguished.
//! - Selection of read frequency. Note that the sample rate is configured in the mobile app.
//! - Data smoothing with a moving average filter._
//! - Registration of listeners to receive sensor data once received and processed.
//!
//! **NOTE** Currently, `phyphox-rs` only captures data from Accelerometer, Gyroscope and Magnetometer.

pub(crate) mod adapters;
pub(crate) mod constants;
mod helpers;
pub(crate) mod models;
pub(crate) mod ports;
pub mod services;
