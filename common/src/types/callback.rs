use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use uuid::Uuid;

pub type AsyncCallback<T> =
    Arc<dyn Fn(Uuid, Arc<T>) -> Pin<Box<dyn Future<Output = ()> + Send>> + Send + Sync>;

pub type Callback<T> = Arc<dyn Fn(Uuid, Arc<T>) -> () + Send + Sync>;
