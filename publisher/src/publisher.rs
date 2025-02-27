use dashmap::DashMap;
use rayon::prelude::*;
use std::sync::Arc;
use uuid::Uuid;

use common::traits::Notifiable;
use common::types::Callback;

pub trait Publishable<T> {
    fn register_listener(&self, listener: &mut dyn Notifiable<T>) -> Uuid;
    fn unregister_listener(&self, listener_id: Uuid);
    fn unregister_all(&self);
    fn notify_listeners(&self, data: Arc<T>);
}

#[derive(Clone, Default)]
pub struct Publisher<T> {
    listeners: Arc<DashMap<Uuid, Callback<T>>>,
}

impl<T> Publisher<T> {
    pub fn new() -> Self {
        Self {
            listeners: Arc::new(DashMap::new()),
        }
    }
}

impl<T> Publishable<T> for Publisher<T>
where
    T: Send + Sync + 'static,
{
    fn register_listener(&self, listener: &mut dyn Notifiable<T>) -> Uuid {
        let callback = listener.get_callback();
        let listener_id = Uuid::new_v4();
        listener.set_id(listener_id);
        self.listeners.insert(listener_id, callback);
        listener_id
    }
    fn unregister_all(&self) {
        self.listeners.clear();
    }

    fn unregister_listener(&self, listener_id: Uuid) {
        self.listeners.remove(&listener_id);
    }

    fn notify_listeners(&self, data: Arc<T>) {
        let listeners: Vec<(Uuid, Callback<T>)> = self
            .listeners
            .iter()
            .map(|entry| (*entry.key(), entry.value().clone()))
            .collect();

        listeners.into_par_iter().for_each(|(id, callback)| {
            let data = data.clone();
            callback(id, data);
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{listener, listener::Listener};
    use std::sync::Mutex;

    struct TestHandler {
        data: Arc<Mutex<i32>>,
    }

    impl TestHandler {
        fn new() -> Self {
            Self {
                data: Arc::new(Mutex::new(0)),
            }
        }

        fn handle(&self, _id: Uuid, value: Arc<i32>) {
            let mut data = self.data.lock().unwrap();
            *data = *value;
        }
    }

    #[test]
    fn test_register_and_notify_listener() {
        let publisher = Publisher::new();
        let handler = Arc::new(TestHandler::new());

        let mut listener = Listener::new({
            let handler = handler.clone();
            move |_id: Uuid, value| {
                handler.handle(_id, value);
            }
        });

        let _listener_id = publisher.register_listener(&mut listener);
        publisher.notify_listeners(Arc::new(42));

        std::thread::sleep(std::time::Duration::from_millis(100));
        assert_eq!(*handler.data.lock().unwrap(), 42);
    }

    #[test]
    fn test_register_and_notify_listener_macro() {
        let publisher = Publisher::new();
        let handler = Arc::new(TestHandler::new());

        let mut listener = listener!(handler.handle);

        let _listener_id = publisher.register_listener(&mut listener);
        publisher.notify_listeners(Arc::new(42));

        std::thread::sleep(std::time::Duration::from_millis(100));
        assert_eq!(*handler.data.lock().unwrap(), 42);
    }

    #[test]
    fn test_unregister_listener() {
        let publisher = Publisher::new();
        let handler = Arc::new(TestHandler::new());

        let mut listener = Listener::new({
            let handler = handler.clone();
            move |_id: Uuid, value| {
                handler.handle(_id, value);
            }
        });

        let listener_id = publisher.register_listener(&mut listener);
        publisher.unregister_listener(listener_id);
        publisher.notify_listeners(Arc::new(100));

        std::thread::sleep(std::time::Duration::from_millis(100));
        // Should remain unchanged since listener was removed
        assert_eq!(*handler.data.lock().unwrap(), 0);
    }
}
