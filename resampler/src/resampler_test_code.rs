use std::collections::HashMap;
use std::hash::Hash;
use std::sync::{Arc, RwLock};

/// Represents a collector that stores sensor data in a concurrent structure.
pub struct Collector<T1, T2> {
    data: Arc<RwLock<HashMap<String, Arc<RwLock<HashMap<T1, Vec<T2>>>>>>>,
}

impl<T1, T2> Collector<T1, T2>
where
    T1: Eq + Hash + Clone,
    T2: Clone,
{
    /// Creates a new Collector instance.
    pub fn new() -> Self {
        Self {
            data: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Adds a new tag if it doesn't exist.
    pub fn add_tag(&self, tag: &str) {
        let mut data_write = self.data.write().unwrap();
        data_write
            .entry(tag.to_string())
            .or_insert_with(|| Arc::new(RwLock::new(HashMap::new())));
    }

    /// Removes a tag and all its associated data.
    pub fn remove_tag(&self, tag: &str) {
        let mut data_write = self.data.write().unwrap();
        data_write.remove(tag);
    }

    /// Adds a new ID to a tag.
    pub fn add_id(&self, tag: &str, id: T1) {
        let data_read = self.data.read().unwrap();
        if let Some(tag_data) = data_read.get(tag) {
            let mut tag_write = tag_data.write().unwrap();
            tag_write.entry(id).or_insert_with(Vec::new);
        }
    }

    /// Removes an ID from a tag.
    pub fn remove_id(&self, tag: &str, id: &T1) {
        let data_read = self.data.read().unwrap();
        if let Some(tag_data) = data_read.get(tag) {
            let mut tag_write = tag_data.write().unwrap();
            tag_write.remove(id);
        }
    }

    /// Adds samples for a given tag and ID.
    pub fn add_samples(&self, tag: &str, id: T1, samples: Vec<T2>) {
        let data_read = self.data.read().unwrap();
        if let Some(tag_data) = data_read.get(tag) {
            let mut tag_write = tag_data.write().unwrap();
            tag_write.entry(id).or_insert_with(Vec::new).extend(samples);
        }
    }

    /// Consumes selected samples from a tag using a filter function.
    pub fn consume_samples<F>(&self, tag: &str, filter: F) -> Option<HashMap<T1, Vec<T2>>>
    where
        F: Fn(&T1, &Vec<T2>) -> bool,
    {
        let data_read = self.data.read().unwrap();
        if let Some(tag_data) = data_read.get(tag) {
            let mut tag_write = tag_data.write().unwrap();
            let mut consumed = HashMap::new();

            tag_write.retain(|id, samples| {
                if filter(id, samples) {
                    consumed.insert(id.clone(), samples.clone());
                    false // Remove the samples
                } else {
                    true // Keep the samples
                }
            });

            Some(consumed)
        } else {
            None
        }
    }
}

// Example usage
fn main() {
    let collector = Collector::<u32, f64>::new();

    // Add a tag and an ID
    collector.add_tag("accelerometer");
    collector.add_id("accelerometer", 1);

    // Add some sample data
    collector.add_samples("accelerometer", 1, vec![1.1, 2.2, 3.3]);
    collector.add_samples("accelerometer", 2, vec![4.4, 5.5]);

    // Consume samples where the ID is 1
    let consumed = collector.consume_samples("accelerometer", |id, _| *id == 1);

    println!("Consumed samples: {:?}", consumed);
}
