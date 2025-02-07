use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::Mutex;
use uuid::Uuid;

use crate::listener::{Callback, Notifiable};

#[async_trait]
pub trait Publishable<T> {
    async fn register_listener(&self, listener: &mut dyn Notifiable<T>) -> Uuid;
    async fn unregister_listener(&self, listener_id: Uuid);
    async fn notify_listeners(&self, data: Arc<T>);
}

#[derive(Clone, Default)]
pub struct Publisher<T>
where
    T: Send + Sync + 'static,
{
    listeners: Arc<Mutex<HashMap<Uuid, Callback<T>>>>,
}

impl<T> Publisher<T>
where
    T: Send + Sync + 'static,
{
    pub fn new() -> Self {
        Self {
            listeners: Arc::new(Mutex::new(HashMap::new())),
        }
    }
}

#[async_trait]
impl<T> Publishable<T> for Publisher<T>
where
    T: Send + Sync + 'static,
{
    async fn register_listener(&self, listener: &mut dyn Notifiable<T>) -> Uuid {
        let callback = listener.get_callback();
        let listener_id = Uuid::new_v4();
        listener.set_id(listener_id);
        let mut listeners = self.listeners.lock().await;
        listeners.insert(listener_id, callback);
        listener_id
    }

    async fn unregister_listener(&self, listener_id: Uuid) {
        let mut listeners = self.listeners.lock().await;
        listeners.remove(&listener_id);
    }

    async fn notify_listeners(&self, data: Arc<T>) {
        let listeners: Vec<(Uuid, Callback<T>)> = {
            let listeners_guard = self.listeners.lock().await;
            listeners_guard
                .iter()
                .map(|(uuid, callback)| (*uuid, callback.clone()))
                .collect()
        };

        let mut tasks = vec![];

        for listener in listeners {
            let data = data.clone();
            let (uuid, callback) = listener.clone();

            let task = tokio::spawn(async move {
                callback(uuid, data).await;
            });

            tasks.push(task);
        }

        for task in tasks {
            let _ = task.await;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{listener, listener::Listener};
    use tokio::sync::Mutex as AsyncMutex;

    struct TestHandler {
        data: Arc<AsyncMutex<i32>>,
    }

    impl TestHandler {
        fn new() -> Self {
            Self {
                data: Arc::new(AsyncMutex::new(0)),
            }
        }

        async fn handle(&self, _id: Uuid, value: Arc<i32>) {
            let mut data = self.data.lock().await;
            *data = *value;
        }
    }

    #[tokio::test]
    async fn test_register_and_notify_listener() {
        let publisher = Publisher::new();
        let handler = Arc::new(TestHandler::new());

        let mut listener = Listener::new({
            let handler = handler.clone();
            move |_id: Uuid, value| {
                let handler = handler.clone();
                async move {
                    handler.handle(_id, value).await;
                }
            }
        });

        let _listener_id = publisher.register_listener(&mut listener).await;
        publisher.notify_listeners(Arc::new(42)).await;

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        assert_eq!(*handler.data.lock().await, 42);
    }

    #[tokio::test]
    async fn test_register_and_notify_listener_macro() {
        let publisher = Publisher::new();
        let handler = Arc::new(TestHandler::new());

        let mut listener = listener!(handler.handle);

        let _listener_id = publisher.register_listener(&mut listener).await;
        publisher.notify_listeners(Arc::new(42)).await;

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        assert_eq!(*handler.data.lock().await, 42);
    }

    #[tokio::test]
    async fn test_unregister_listener() {
        let publisher = Publisher::new();
        let handler = Arc::new(TestHandler::new());

        let mut listener = Listener::new({
            let handler = handler.clone();
            move |_id: Uuid, value: Arc<i32>| {
                let handler = handler.clone();
                async move {
                    handler.handle(_id, value).await;
                }
            }
        });

        let listener_id = publisher.register_listener(&mut listener).await;
        publisher.unregister_listener(listener_id).await;
        publisher.notify_listeners(Arc::new(100)).await;

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        // Should remain unchanged since listener was removed
        assert_eq!(*handler.data.lock().await, 0);
    }
}
