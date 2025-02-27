use std::sync::Arc;
use uuid::Uuid;

pub type Callback<T> = Arc<dyn Fn(Uuid, Arc<T>) -> () + Send + Sync>;
