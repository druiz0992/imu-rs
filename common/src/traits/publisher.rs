use crate::types::{AsyncCallback, Callback};
use uuid::Uuid;

pub trait Notifiable<T>: Sync + Send {
    fn get_async_callback(&self) -> AsyncCallback<T>;
    fn get_callback(&self) -> Callback<T>;
    fn set_id(&mut self, id: Uuid);
}
