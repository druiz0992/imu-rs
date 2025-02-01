//! # Crate publisher
//!
//! ## publisher
//!
//! The `publisher` crate provides a mechanism for registering and notifying listeners
//! of new events of type `T`.
//!
//! This crate is designed to handle dynamic registration of callback functions (`Fn(T)`) as listeners,
//! ensuring that all registered listeners receive updates when an event occurs.
//!
//! ### Example
//!
//! ```ignore
//! fn main() {
//!     let publisher = Arc::new(Publisher::new());
//!
//!     // Register a listener
//!     let listener_id = publisher.register(|data: String| {
//!         println!("Listener received: {}", data);
//!     });
//!
//!     // Notify all listeners
//!     publisher.notify("Hello, World!".to_string());
//!
//!     // Unregister the listener
//!     publisher.unregister(listener_id).expect("Failed to unregister listener");
//!
//!     // Verify that no listeners are left
//!     assert!(publisher.is_empty());
//! }
//! ```

use std::sync::{Arc, Mutex};
use uuid::Uuid;

#[derive(PartialEq, Clone, Debug)]
pub enum PublisherError {
    ListenerNotFound(String),
}
/// Record of registered listeners that will be notofied of updates
pub struct Publisher<T> {
    listeners: Arc<Mutex<Vec<(Uuid, Box<dyn Fn(T) + Send + Sync>)>>>,
}

impl<T: Clone> Publisher<T> {
    /// Creates a new `Publisher` instance.
    ///
    /// The instance starts with no registered listeners.
    pub fn new() -> Self {
        Self {
            listeners: Arc::new(Mutex::new(Vec::new())),
        }
    }

    /// Registers a listener callback to be notified of updates.
    ///
    /// The listener is a function or closure that accepts a `T`
    pub fn register<F>(&self, listener: F) -> Uuid
    where
        F: Fn(T) + Send + Sync + 'static,
    {
        let id = Uuid::new_v4();
        let mut listeners = self.listeners.lock().unwrap();
        listeners.push((id, Box::new(listener)));
        id
    }

    /// Unregisters a listener callback with a given id
    /// Returns PublisherError::ListenerNotFound() if no id matches with any of the registered listeners
    pub fn unregister(&self, id: Uuid) -> Result<(), PublisherError> {
        let mut listeners = self.listeners.lock().unwrap();
        if let Some(pos) = listeners
            .iter()
            .position(|(listener_id, _)| *listener_id == id)
        {
            let _ = listeners.remove(pos);
            return Ok(());
        }
        return Err(PublisherError::ListenerNotFound(format!(
            "Listener with id {} not found",
            id
        )));
    }

    /// Calls each registered listener with the provided measurement.
    pub fn notify(&self, measurement: T) {
        let listeners = self.listeners.lock().unwrap();
        for (_, listener) in listeners.iter() {
            listener(measurement.clone());
        }
    }

    // Returns true if no listeners registered
    pub fn is_empty(&self) -> bool {
        let listeners = self.listeners.lock().unwrap();
        listeners.len() == 0
    }

    // Returns numberof registered listeners
    pub fn len(&self) -> usize {
        let listeners = self.listeners.lock().unwrap();
        listeners.len()
    }
}

#[cfg(test)]
mod tests {
    #![allow(dead_code)]
    use super::*;
    use std::sync::{Arc, Mutex};

    #[derive(Clone, Debug)]
    struct Measurement {
        timestamp: f64,
        data: Vec<f64>,
    }

    #[test]
    fn test_publisher_initialization() {
        let publisher = Publisher::<Measurement>::new();
        assert!(publisher.is_empty());
    }

    #[test]
    fn test_register_listener() {
        let publisher = Publisher::<Measurement>::new();

        let listener = |measurement: Measurement| {
            println!("Received measurement: {:?}", measurement);
        };

        publisher.register(listener);
        assert_eq!(publisher.len(), 1);
    }

    #[test]
    fn test_unregister_listener() {
        let publisher = Publisher::<Measurement>::new();

        let listener1 = |measurement: Measurement| {
            println!("Received measurement1: {:?}", measurement);
        };
        let listener2 = |measurement: Measurement| {
            println!("Received measurement2: {:?}", measurement);
        };

        let id1 = publisher.register(listener1);
        let id2 = publisher.register(listener2);
        assert_eq!(publisher.len(), 2);

        assert_eq!(publisher.unregister(id2), Ok(()));
        assert_eq!(publisher.len(), 1);
        assert_eq!(publisher.unregister(id1), Ok(()));
        assert_eq!(publisher.len(), 0);
        assert!(publisher.unregister(id1).is_err());
    }

    #[test]
    fn test_notify_listeners() {
        let sensor_measurement = Measurement {
            data: vec![1.0, 2.0, 3.0, 4.0],
            timestamp: 0.0,
        };

        let publisher = Publisher::<Measurement>::new();

        let shared_state = Arc::new(Mutex::new(Vec::new()));

        {
            let shared_state = Arc::clone(&shared_state);
            publisher.register(move |measurement: Measurement| {
                shared_state.lock().unwrap().push(measurement);
            });
        }

        publisher.notify(sensor_measurement);

        // Verify shared state contains the expected common.
        let shared_data = shared_state.lock().unwrap();
        assert_eq!(shared_data.len(), 1);
        assert_eq!(shared_data[0].data, vec![1.0, 2.0, 3.0, 4.0]);
    }
}
