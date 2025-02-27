use std::sync::Arc;
use uuid::Uuid;

use imu_common::traits::Notifiable;
use imu_common::types::Callback;

#[derive(Clone)]
pub struct Listener<T> {
    callback: Callback<T>,
    id: Option<Uuid>,
}

impl<T> Listener<T>
where
    T: Send + Sync + 'static,
{
    pub fn new<F>(callback: F) -> Self
    where
        F: Fn(Uuid, Arc<T>) + Send + Sync + 'static,
    {
        let callback = Arc::new(move |id: Uuid, data: Arc<T>| {
            callback(id, data);
        });

        Listener { callback, id: None }
    }
}

impl<T> Notifiable<T> for Listener<T> {
    fn get_callback(&self) -> Callback<T> {
        self.callback.clone()
    }

    fn set_id(&mut self, id: Uuid) {
        self.id = Some(id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::listener;
    use std::sync::Mutex;

    struct TestHandler {
        data: Arc<Mutex<Vec<i32>>>,
    }

    impl TestHandler {
        fn new() -> Self {
            Self {
                data: Arc::new(Mutex::new(Vec::new())),
            }
        }

        fn handle(&self, _id: Uuid, value: Arc<Vec<i32>>) {
            let mut data = self.data.lock().unwrap();
            *data = (*value).clone();
            assert_eq!((*data)[0], 400);
        }
    }

    #[test]
    fn test_new_listener() {
        let listener = Listener::new({
            move |_id: Uuid, value: Arc<i32>| {
                let data = *value;
                assert_eq!(data, 42);
            }
        });

        let callback = listener.get_callback();
        callback(Uuid::new_v4(), Arc::new(42));
    }

    #[test]
    fn test_listener_with_method() {
        let handler = Arc::new(TestHandler::new());

        let listener = Listener::new({
            move |id: Uuid, value: Arc<Vec<i32>>| {
                let handler = handler.clone();
                handler.handle(id, value);
            }
        });

        let callback = listener.get_callback();
        callback(Uuid::new_v4(), Arc::new(vec![400]));
    }

    #[test]
    fn test_listener_with_macro() {
        let handler = Arc::new(TestHandler::new());

        let listener = listener!(handler.handle);

        let callback = listener.get_callback();
        callback(Uuid::new_v4(), Arc::new(vec![400]));
    }
}
