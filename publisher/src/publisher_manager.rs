use dashmap::DashMap;
use std::cmp::Eq;
use std::hash::Hash;
use std::sync::Arc;
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
///     let acc_id = Uuid::new_v4();
///     manager.add_publisher(SensorType::Accelerometer(acc_id)).await;
///
///     // Prepare to create listener
///     let test_buffer = Arc::new(TestBuffer::new());
///     let mut listener = listener!(test_buffer.handle);
///
///     // add listener to existing Accelerometer publisher
///     let id = manager.add_listener(&mut listener, &SensorType::Accelerometer(acc_id)).await.unwrap();
///
///     // remove listener from Accelerometer publisher
///     manager.remove_listener(id).await.unwrap();
/// });
///  
/// ```

#[derive(Clone)]
pub struct PublisherManager<T, S> {
    publishers: Arc<DashMap<S, Publisher<T>>>,
    control: Arc<DashMap<Uuid, S>>,
}

impl<T, S> PublisherManager<T, S>
where
    T: Send + Sync + Clone + 'static,
    S: Send + Sync + Hash + Eq + Clone + Into<usize>,
{
    pub fn new(publisher_types: &[S]) -> Self {
        let collection = DashMap::<S, Publisher<T>>::new();
        for publisher_type in publisher_types {
            collection.insert(publisher_type.clone(), Publisher::new());
        }

        Self {
            publishers: Arc::new(collection),
            control: Arc::new(DashMap::new()),
        }
    }

    pub async fn add_publisher(&mut self, publisher_type: S) {
        let publisher = Publisher::new();
        self.publishers.insert(publisher_type, publisher);
    }

    pub async fn remove_publisher(&mut self, publisher_type: &S) {
        if let Some((_, publisher)) = self.publishers.remove(publisher_type) {
            publisher.unregister_all().await;
        }
    }

    pub async fn get_available_publisher_types(&self) -> Vec<S> {
        let mut sensor_types: Vec<S> = self
            .publishers
            .iter()
            .map(|entry| entry.key().clone())
            .collect();
        sensor_types.sort_by_key(|sensor_type| (sensor_type.clone()).into());
        sensor_types
    }

    pub async fn add_listener(
        &self,
        listener: &mut dyn Notifiable<T>,
        publisher_type: &S,
    ) -> Result<Uuid, String> {
        if let Some(publisher) = self.publishers.get(publisher_type) {
            let id = publisher.register_listener(listener).await;
            self.control.insert(id, publisher_type.clone());
            return Ok(id);
        }
        Err("Publisher doesnt exist".to_string())
    }

    pub async fn remove_listener(&self, id: Uuid) -> Result<(), String> {
        if let Some((_, publisher_type)) = self.control.remove(&id) {
            if let Some(publisher) = self.publishers.get(&publisher_type) {
                publisher.unregister_listener(id).await;
            } else {
                return Err("Publisher doesnt exist".to_string());
            }
            return Ok(());
        }
        Err("Listener Id not found".to_string())
    }

    pub async fn notify_listeners(&self, publisher_type: S, data: Arc<T>) {
        if let Some(publisher) = self.publishers.get(&publisher_type) {
            publisher.notify_listeners(data).await;
        }
    }

    pub async fn get_publishers_sorted_by_index(&self) -> Vec<Publisher<T>> {
        let sensor_types = self.get_available_publisher_types().await;
        sensor_types
            .iter()
            .map(|sensor_type| self.publishers.get(sensor_type).unwrap().clone())
            .collect()
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
        let acc_id = Uuid::new_v4();
        let gyro_id = Uuid::new_v4();
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);
        manager
            .add_publisher(SensorType::Accelerometer(acc_id))
            .await;
        manager.add_publisher(SensorType::Gyroscope(gyro_id)).await;
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.contains(&SensorType::Accelerometer(acc_id)));
        assert!(available_publishers.contains(&SensorType::Gyroscope(gyro_id)));
        assert!(available_publishers.len() == 2);
    }

    #[tokio::test]
    async fn test_add_duplicated_publisher() {
        let acc_id = Uuid::new_v4();
        let mut manager =
            PublisherManager::<Vec<Sample3D>, SensorType>::new(&[SensorType::Accelerometer(
                acc_id,
            )]);
        manager
            .add_publisher(SensorType::Accelerometer(acc_id))
            .await;
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.contains(&SensorType::Accelerometer(acc_id)));
        assert!(available_publishers.len() == 1);
    }

    #[tokio::test]
    async fn test_add_2_accelerometer_publisher() {
        let acc_id1 = Uuid::new_v4();
        let acc_id2 = Uuid::new_v4();
        let mut manager =
            PublisherManager::<Vec<Sample3D>, SensorType>::new(&[SensorType::Accelerometer(
                acc_id1,
            )]);
        manager
            .add_publisher(SensorType::Accelerometer(acc_id2))
            .await;
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.contains(&SensorType::Accelerometer(acc_id1)));
        assert!(available_publishers.contains(&SensorType::Accelerometer(acc_id2)));
        assert!(available_publishers.len() == 2);
    }

    #[tokio::test]
    async fn test_remove_publisher_without_listeners() {
        let acc_id = Uuid::new_v4();
        let gyro_id = Uuid::new_v4();
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[
            SensorType::Accelerometer(acc_id),
            SensorType::Gyroscope(gyro_id),
        ]);
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.len() == 2);

        manager
            .remove_publisher(&SensorType::Accelerometer(acc_id))
            .await;
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.len() == 1);
        assert!(available_publishers.contains(&SensorType::Gyroscope(gyro_id)));
    }

    #[tokio::test]
    async fn test_remove_unknown_publisher() {
        let acc_id = Uuid::new_v4();
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);
        manager
            .add_publisher(SensorType::Accelerometer(acc_id))
            .await;
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.len() == 1);

        manager
            .remove_publisher(&SensorType::Gyroscope(acc_id))
            .await;
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.len() == 1);
        assert!(available_publishers.contains(&SensorType::Accelerometer(acc_id)));
    }

    #[tokio::test]
    async fn test_remove_publisher_from_empty_manager() {
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.is_empty());

        manager
            .remove_publisher(&SensorType::Gyroscope(Uuid::new_v4()))
            .await;
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.is_empty());
    }

    #[tokio::test]
    async fn test_add_listener() {
        let acc_id = Uuid::new_v4();
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);
        manager
            .add_publisher(SensorType::Accelerometer(acc_id))
            .await;

        let test_buffer = Arc::new(TestBuffer::new());
        let mut listener = listener!(test_buffer.handle);
        manager
            .add_listener(&mut listener, &SensorType::Accelerometer(acc_id))
            .await
            .unwrap();
    }

    #[tokio::test]
    async fn test_add_2_publishers_other_type_and_check_they_are_treated_differently() {
        let other_id1 = Uuid::new_v4();
        let other_id2 = Uuid::new_v4();
        let sensors = [
            SensorType::Other(other_id1, "Sensor1".to_string()),
            SensorType::Other(other_id2, "Sensor1".to_string()),
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
    async fn test_add_2_publishers_same_type_and_check_only_one_added() {
        let other_id1 = Uuid::new_v4();
        let sensors = [
            SensorType::Other(other_id1, "Sensor1".to_string()),
            SensorType::Other(other_id1, "Sensor1".to_string()),
        ];
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&sensors);
        let available_publishers = manager.get_available_publisher_types().await;

        assert!(available_publishers.len() == 1);

        manager.remove_publisher(&sensors[0]).await;
        let available_publishers = manager.get_available_publisher_types().await;
        assert!(available_publishers.is_empty());
    }

    #[tokio::test]
    #[should_panic(expected = "Publisher doesnt exist")]
    async fn test_add_listener_to_nonexistent_publisher() {
        let manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);

        let test_buffer = Arc::new(TestBuffer::new());
        let mut listener = listener!(test_buffer.handle);
        manager
            .add_listener(&mut listener, &SensorType::Accelerometer(Uuid::new_v4()))
            .await
            .unwrap();
    }

    #[tokio::test]
    async fn test_remove_listener() {
        let acc_id = Uuid::new_v4();
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);
        manager
            .add_publisher(SensorType::Accelerometer(acc_id))
            .await;

        let test_buffer = Arc::new(TestBuffer::new());
        let mut listener = listener!(test_buffer.handle);
        let id = manager
            .add_listener(&mut listener, &SensorType::Accelerometer(acc_id))
            .await
            .unwrap();

        manager.remove_listener(id).await.unwrap();
    }

    #[tokio::test]
    #[should_panic(expected = "Listener Id not found")]
    async fn test_remove_unknown_listener() {
        let acc_id = Uuid::new_v4();
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);
        manager
            .add_publisher(SensorType::Accelerometer(acc_id))
            .await;
        let id = Uuid::new_v4();

        manager.remove_listener(id).await.unwrap();
    }

    #[tokio::test]
    #[should_panic(expected = "Publisher doesnt exist")]
    async fn test_remove_publisher_with_listeners() {
        let mut manager = PublisherManager::<Vec<Sample3D>, SensorType>::new(&[]);
        let acc_id = Uuid::new_v4();
        let gyro_id = Uuid::new_v4();
        manager
            .add_publisher(SensorType::Accelerometer(acc_id))
            .await;
        manager.add_publisher(SensorType::Gyroscope(gyro_id)).await;

        let test_buffer = Arc::new(TestBuffer::new());
        let mut listener = listener!(test_buffer.handle);
        let id = manager
            .add_listener(&mut listener, &SensorType::Accelerometer(acc_id))
            .await
            .unwrap();

        manager
            .remove_publisher(&SensorType::Accelerometer(acc_id))
            .await;

        manager.remove_listener(id).await.unwrap();
    }
}
