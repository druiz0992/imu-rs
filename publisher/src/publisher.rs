use async_trait::async_trait;
use dashmap::DashMap;
use std::sync::Arc;
use tokio::task::JoinSet;
use uuid::Uuid;

use common::traits::Notifiable;
use common::types::AsyncCallback;

#[async_trait]
pub trait Publishable<T> {
    fn register_listener(&self, listener: &mut dyn Notifiable<T>) -> Uuid;
    fn unregister_listener(&self, listener_id: Uuid);
    fn unregister_all(&self);
    async fn notify_listeners(&self, data: Arc<T>);
}

#[derive(Clone, Default)]
pub struct Publisher<T> {
    listeners: Arc<DashMap<Uuid, AsyncCallback<T>>>,
}

impl<T> Publisher<T> {
    pub fn new() -> Self {
        Self {
            listeners: Arc::new(DashMap::new()),
        }
    }
}

#[async_trait]
impl<T> Publishable<T> for Publisher<T>
where
    T: Send + Sync + 'static,
{
    fn register_listener(&self, listener: &mut dyn Notifiable<T>) -> Uuid {
        let callback = listener.get_async_callback();
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

    async fn notify_listeners(&self, data: Arc<T>) {
        let listeners: Vec<(Uuid, AsyncCallback<T>)> = self
            .listeners
            .iter()
            .map(|entry| (*entry.key(), entry.value().clone()))
            .collect();

        let mut join_set = JoinSet::new();
        for (uuid, callback) in listeners {
            let data = data.clone();
            join_set.spawn(async move { callback(uuid, data).await });
        }

        while let Some(result) = join_set.join_next().await {
            if let Err(e) = result {
                eprintln!("Task failed: {:?}", e);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{listener, listener::AsyncListener};
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

        let mut listener = AsyncListener::new({
            let handler = handler.clone();
            move |_id: Uuid, value| {
                let handler = handler.clone();
                async move {
                    handler.handle(_id, value).await;
                }
            }
        });

        let _listener_id = publisher.register_listener(&mut listener);
        publisher.notify_listeners(Arc::new(42)).await;

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        assert_eq!(*handler.data.lock().await, 42);
    }

    #[tokio::test]
    async fn test_register_and_notify_listener_macro() {
        let publisher = Publisher::new();
        let handler = Arc::new(TestHandler::new());

        let mut listener = listener!(handler.handle);

        let _listener_id = publisher.register_listener(&mut listener);
        publisher.notify_listeners(Arc::new(42)).await;

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        assert_eq!(*handler.data.lock().await, 42);
    }

    #[tokio::test]
    async fn test_unregister_listener() {
        let publisher = Publisher::new();
        let handler = Arc::new(TestHandler::new());

        let mut listener = AsyncListener::new({
            let handler = handler.clone();
            move |_id: Uuid, value: Arc<i32>| {
                let handler = handler.clone();
                async move {
                    handler.handle(_id, value).await;
                }
            }
        });

        let listener_id = publisher.register_listener(&mut listener);
        publisher.unregister_listener(listener_id);
        publisher.notify_listeners(Arc::new(100)).await;

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        // Should remain unchanged since listener was removed
        assert_eq!(*handler.data.lock().await, 0);
    }
}
