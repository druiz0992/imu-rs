#[derive(Clone, Debug, PartialEq, PartialOrd, Hash)]
pub struct SensorTag(String);

impl SensorTag {
    pub fn new(tag: &str) -> Self {
        Self(tag.to_string())
    }

    pub fn inner(&self) -> &str {
        self.0.as_str()
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    #[test]
    fn test_new() {
        let tag = SensorTag::new("test_tag");
        assert_eq!(tag.inner(), "test_tag");
    }

    #[test]
    fn test_clone() {
        let tag = SensorTag::new("test_tag");
        let cloned_tag = tag.clone();
        assert_eq!(tag, cloned_tag);
    }

    #[test]
    fn test_partial_eq() {
        let tag1 = SensorTag::new("test_tag");
        let tag2 = SensorTag::new("test_tag");
        assert_eq!(tag1, tag2);
    }

    #[test]
    fn test_partial_ord() {
        let tag1 = SensorTag::new("a_tag");
        let tag2 = SensorTag::new("b_tag");
        assert!(tag1 < tag2);
    }

    #[test]
    fn test_hash() {
        let tag = SensorTag::new("test_tag");
        let mut hasher = DefaultHasher::new();
        tag.hash(&mut hasher);
        let hash1 = hasher.finish();

        let mut hasher = DefaultHasher::new();
        tag.hash(&mut hasher);
        let hash2 = hasher.finish();

        assert_eq!(hash1, hash2);
    }
}
