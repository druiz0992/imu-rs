use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use uuid::Uuid;

pub type Callback<T> =
    Arc<dyn Fn(Arc<T>) -> Pin<Box<dyn Future<Output = ()> + Send>> + Send + Sync>;

pub trait Notifiable<T> {
    fn get_callback(&self) -> Callback<T>;
}

#[derive(Clone)]
pub struct Listener<T> {
    callback: Callback<T>,
    listener_id: Option<Uuid>,
}

impl<T> Listener<T>
where
    T: Send + Sync + 'static,
{
    pub fn new<F, Fut>(callback: F) -> Self
    where
        F: Fn(Arc<T>) -> Fut + Send + Sync + 'static,
        Fut: Future<Output = ()> + Send + 'static,
    {
        let callback = Arc::new(move |data: Arc<T>| {
            let fut = callback(data);
            Box::pin(fut) as Pin<Box<dyn Future<Output = ()> + Send>>
        });

        Listener {
            callback,
            listener_id: None,
        }
    }

    pub fn set_id(&mut self, listener_id: Uuid) {
        self.listener_id = Some(listener_id);
    }
}

impl<T> Notifiable<T> for Listener<T> {
    fn get_callback(&self) -> Callback<T> {
        self.callback.clone()
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

        async fn handle(&self, value: Arc<Vec<i32>>) {
            let mut data = self.data.lock().await;
            *data = (*value).clone();
            assert_eq!((*data)[0], 400);
        }
    }

    #[tokio::test]
    async fn test_new_listener() {
        let listener = Listener::new({
            move |value: Arc<i32>| async move {
                let data = *value;
                assert_eq!(data, 42);
            }
        });

        let callback = listener.get_callback();
        callback(Arc::new(42)).await;
    }

    #[tokio::test]
    async fn test_listener_with_method() {
        let handler = Arc::new(TestHandler::new());

        let listener = Listener::new({
            move |value: Arc<Vec<i32>>| {
                let handler = handler.clone();
                async move { handler.handle(value).await }
            }
        });

        let callback = listener.get_callback();
        callback(Arc::new(vec![400])).await;
    }

    #[tokio::test]
    async fn test_listener_with_macro() {
        let handler = Arc::new(TestHandler::new());

        let listener = listener!(handler.handle);

        let callback = listener.get_callback();
        callback(Arc::new(vec![400])).await;
    }

    #[test]
    fn test_set_id() {
        let listener = Listener::new(|_: Arc<i32>| async {});
        let mut listener = listener;
        let id = Uuid::new_v4();
        listener.set_id(id);

        assert_eq!(listener.listener_id, Some(id));
    }
}
