use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use uuid::Uuid;

pub type Callback<T> =
    Arc<dyn Fn(Uuid, Arc<T>) -> Pin<Box<dyn Future<Output = ()> + Send>> + Send + Sync>;

pub trait Notifiable<T>: Sync + Send {
    fn get_callback(&self) -> Callback<T>;
    fn set_id(&mut self, id: Uuid);
}

#[derive(Clone)]
pub struct Listener<T> {
    callback: Callback<T>,
    id: Option<Uuid>,
}

impl<T> Listener<T>
where
    T: Send + Sync + 'static,
{
    pub fn new<F, Fut>(callback: F) -> Self
    where
        F: Fn(Uuid, Arc<T>) -> Fut + Send + Sync + 'static,
        Fut: Future<Output = ()> + Send + 'static,
    {
        let callback = Arc::new(move |id: Uuid, data: Arc<T>| {
            let fut = callback(id, data);
            Box::pin(fut) as Pin<Box<dyn Future<Output = ()> + Send>>
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
    use tokio::sync::Mutex;

    struct TestHandler {
        data: Arc<Mutex<Vec<i32>>>,
    }

    impl TestHandler {
        fn new() -> Self {
            Self {
                data: Arc::new(Mutex::new(Vec::new())),
            }
        }

        async fn handle(&self, _id: Uuid, value: Arc<Vec<i32>>) {
            let mut data = self.data.lock().await;
            *data = (*value).clone();
            assert_eq!((*data)[0], 400);
        }
    }

    #[tokio::test]
    async fn test_new_listener() {
        let listener = Listener::new({
            move |_id: Uuid, value: Arc<i32>| async move {
                let data = *value;
                assert_eq!(data, 42);
            }
        });

        let callback = listener.get_callback();
        callback(Uuid::new_v4(), Arc::new(42)).await;
    }

    #[tokio::test]
    async fn test_listener_with_method() {
        let handler = Arc::new(TestHandler::new());

        let listener = Listener::new({
            move |id: Uuid, value: Arc<Vec<i32>>| {
                let handler = handler.clone();
                async move { handler.handle(id, value).await }
            }
        });

        let callback = listener.get_callback();
        callback(Uuid::new_v4(), Arc::new(vec![400])).await;
    }

    #[tokio::test]
    async fn test_listener_with_macro() {
        let handler = Arc::new(TestHandler::new());

        let listener = listener!(handler.handle);

        let callback = listener.get_callback();
        callback(Uuid::new_v4(), Arc::new(vec![400])).await;
    }
}
