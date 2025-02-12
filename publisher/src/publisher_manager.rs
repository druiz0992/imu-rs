use std::cmp::Eq;
use std::collections::HashMap;
use std::hash::Hash;
use std::sync::Arc;
use tokio::sync::RwLock;
use uuid::Uuid;

use crate::Publishable;

use super::publisher::Publisher;
use common::traits::publisher::Notifiable;

/// This module defines the `PublisherManager` struct, which manages publishers and their listeners.
/// It provides functionality to add and remove publishers, as well as to add and remove listeners
/// to/from specific publishers. The `PublisherManager` ensures thread-safe access to its internal
/// data structures using `tokio::sync::RwLock`.
///
/// # Example
///
/// ```rust
/// use uuid::Uuid;
/// use publisher::PublisherManager;
/// use publisher::listener;
/// use publisher::listener::Listener;
/// use common::types::sensors::SensorType;
/// use common::types::timed::Sample3D;
/// use std::sync::Arc;
/// use tokio::runtime::Runtime;
///
/// let rt = Runtime::new().unwrap();
/// rt.block_on(async {
///
///     // Test struct
///     #[derive(Debug, Clone)]
///     struct TestBuffer;
///
///     impl TestBuffer {
///         fn new() -> Self {
///             Self
///         }
///
///         async fn handle(&self, _id: Uuid, samples: Arc<Vec<Sample3D>>) {
///             println!("Samples: {:?}", samples);
///         }
///     }
///     
///     // Create PublisherManager
///     let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);
///     // Add new publisher for Accelerometers
///     manager.add_publisher(SensorType::Accelerometer).await;
///
///     // Prepare to create listener
///     let test_buffer = Arc::new(TestBuffer::new());
///     let mut listener = listener!(test_buffer.handle);
///
///     // add listener to existing Accelerometer publisher
///     let id = manager.add_listener(&mut listener, SensorType::Accelerometer).await.unwrap();
///
///     // remove listener from Accelerometer publisher
///     manager.remove_listener(id).await.unwrap();
/// });
///  
/// ```

#[derive(Clone)]
pub struct PublisherManager<T, S> {
    publishers: Arc<RwLock<HashMap<S, Publisher<T>>>>,
    control: Arc<RwLock<HashMap<Uuid, S>>>,
}

impl<T, S> PublisherManager<T, S>
where
    T: Send + Sync + Clone + 'static,
    S: Send + Sync + Hash + Eq + Clone,
{
    pub fn new(publisher_types: &[S]) -> Self {
        let mut collection = HashMap::<S, Publisher<T>>::new();
        for publisher_type in publisher_types {
            collection.insert(publisher_type.clone(), Publisher::new());
        }

        Self {
            publishers: Arc::new(RwLock::new(collection)),
            control: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    pub async fn add_publisher(&mut self, publisher_type: S) {
        let publisher = Publisher::new();
        let mut publishers = self.publishers.write().await;
        publishers.insert(publisher_type, publisher);
    }

    pub async fn remove_publisher(&mut self, publisher_type: &S) {
        let mut publishers = self.publishers.write().await;
        if let Some(publisher) = publishers.remove(publisher_type) {
            publisher.unregister_all().await;
        }
    }

    pub async fn get_available_publisher_types(&self) -> Vec<S> {
        let publishers = self.publishers.read().await;
        let available_publishers: Vec<S> = publishers.keys().cloned().collect();
        available_publishers
    }

    pub async fn add_listener(
        &self,
        listener: &mut dyn Notifiable<T>,
        publisher_type: S,
    ) -> Result<Uuid, String> {
        let publishers = self.publishers.read().await;
        if let Some(publisher) = publishers.get(&publisher_type) {
            let id = publisher.register_listener(listener).await;
            let mut control = self.control.write().await;
            control.insert(id, publisher_type);
            return Ok(id);
        }
        Err("Publisher doesnt exist".to_string())
    }

    pub async fn remove_listener(&self, id: Uuid) -> Result<(), String> {
        let mut control = self.control.write().await;
        if let Some(publisher_type) = control.remove(&id) {
            let publishers = self.publishers.read().await;
            if let Some(publisher) = publishers.get(&publisher_type) {
                publisher.unregister_listener(id).await;
            } else {
                return Err("Publisher doesnt exist".to_string());
            }
            return Ok(());
        }
        Err("Listener Id not found".to_string())
    }

    pub async fn notify_listeners(&self, publisher_type: S, data: Arc<T>) {
        let publishers = self.publishers.read().await;
        if let Some(publisher) = publishers.get(&publisher_type) {
            publisher.notify_listeners(data).await;
        }
    }

    pub async fn get_publishers(&self) -> Vec<Publisher<T>> {
        let publishers = self.publishers.read().await;
        publishers.values().cloned().collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::listener;
    use crate::Listener;
    use common::types::sensors::SensorType;
    use common::types::timed::Sample3D;

    #[derive(Debug, Clone)]
    struct TestBuffer;

    impl TestBuffer {
        fn new() -> Self {
            Self
        }

        async fn handle(&self, _id: Uuid, samples: Arc<Vec<Sample3D>>) {
            println!("Samples: {:?}", samples);
        }
    }

    #[test]
    fn test_new_manager() {
        let _manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);
    }

    #[tokio::test]
    async fn test_add_publisher() {
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);
        manager.add_publisher(SensorType::Accelerometer).await;
        manager.add_publisher(SensorType::Gyroscope).await;
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.contains(&SensorType::Accelerometer));
        assert!(available_publishers.contains(&SensorType::Gyroscope));
        assert!(available_publishers.len() == 2);
    }

    #[tokio::test]
    async fn test_add_duplicated_publisher() {
        let mut manager =
            PublisherManager::<Vec<Sample3D>, SensorType>::new(&[SensorType::Accelerometer]);
        manager.add_publisher(SensorType::Accelerometer).await;
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.contains(&SensorType::Accelerometer));
        assert!(available_publishers.len() == 1);
    }

    #[tokio::test]
    async fn test_remove_publisher_without_listeners() {
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[
            SensorType::Accelerometer,
            SensorType::Gyroscope,
        ]);
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.len() == 2);

        manager.remove_publisher(&SensorType::Accelerometer).await;
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.len() == 1);
        assert!(available_publishers.contains(&SensorType::Gyroscope));
    }

    #[tokio::test]
    async fn test_remove_unknown_publisher() {
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);
        manager.add_publisher(SensorType::Accelerometer).await;
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.len() == 1);

        manager.remove_publisher(&SensorType::Gyroscope).await;
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.len() == 1);
        assert!(available_publishers.contains(&SensorType::Accelerometer));
    }

    #[tokio::test]
    async fn test_remove_publisher_from_empty_manager() {
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.len() == 0);

        manager.remove_publisher(&SensorType::Gyroscope).await;
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.len() == 0);
    }

    #[tokio::test]
    async fn test_add_listener() {
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);
        manager.add_publisher(SensorType::Accelerometer).await;

        let test_buffer = Arc::new(TestBuffer::new());
        let mut listener = listener!(test_buffer.handle);
        manager
            .add_listener(&mut listener, SensorType::Accelerometer)
            .await
            .unwrap();
    }

    #[tokio::test]
    async fn test_add_2_publishers_other_type_and_check_they_are_treated_differently() {
        let sensors = [
            SensorType::Other("Sensor1".to_string()),
            SensorType::Other("Sensor2".to_string()),
        ];
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&sensors);
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.len() == 2);

        manager.remove_publisher(&sensors[0]).await;
        let available_publishers = manager.get_available_publisher_types().await;
        assert!(available_publishers.len() == 1);
        assert!(available_publishers.contains(&sensors[1].clone()));
    }

    #[tokio::test]
    #[should_panic(expected = "Publisher doesnt exist")]
    async fn test_add_listener_to_nonexistent_publisher() {
        let manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);

        let test_buffer = Arc::new(TestBuffer::new());
        let mut listener = listener!(test_buffer.handle);
        manager
            .add_listener(&mut listener, SensorType::Accelerometer)
            .await
            .unwrap();
    }

    #[tokio::test]
    async fn test_remove_listener() {
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);
        manager.add_publisher(SensorType::Accelerometer).await;

        let test_buffer = Arc::new(TestBuffer::new());
        let mut listener = listener!(test_buffer.handle);
        let id = manager
            .add_listener(&mut listener, SensorType::Accelerometer)
            .await
            .unwrap();

        manager.remove_listener(id).await.unwrap();
    }

    #[tokio::test]
    #[should_panic(expected = "Listener Id not found")]
    async fn test_remove_unknown_listener() {
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);
        manager.add_publisher(SensorType::Accelerometer).await;
        let id = Uuid::new_v4();

        manager.remove_listener(id).await.unwrap();
    }

    #[tokio::test]
    #[should_panic(expected = "Publisher doesnt exist")]
    async fn test_remove_publisher_with_listeners() {
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);
        manager.add_publisher(SensorType::Accelerometer).await;
        manager.add_publisher(SensorType::Gyroscope).await;

        let test_buffer = Arc::new(TestBuffer::new());
        let mut listener = listener!(test_buffer.handle);
        let id = manager
            .add_listener(&mut listener, SensorType::Accelerometer)
            .await
            .unwrap();

        manager.remove_publisher(&SensorType::Accelerometer).await;

        manager.remove_listener(id).await.unwrap();
    }
}
