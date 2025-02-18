use crate::constants::N_SENSORS;

pub(super) struct Timestamp {
    current_timestamp: f64,
    readings_timestamp: [f64; N_SENSORS],
}

impl Timestamp {
    pub(super) fn new() -> Self {
        Self {
            current_timestamp: 0f64,
            readings_timestamp: [0f64; N_SENSORS],
        }
    }

    pub(super) fn get_current_timestamp(&self) -> f64 {
        self.current_timestamp
    }

    pub(super) fn set_current_timestamp(&mut self, new_timestamp: f64) {
        self.current_timestamp = new_timestamp;
    }

    pub(super) fn update_all(&mut self, new_timestamp: f64) {
        self.set_current_timestamp(new_timestamp);
        for i in 0..self.readings_timestamp.len() {
            self.set_reading_timestamp(i, new_timestamp);
        }
    }

    pub(super) fn get_reading_timestamp(&self, idx: usize) -> f64 {
        self.readings_timestamp[idx]
    }

    pub(super) fn set_reading_timestamp(&mut self, idx: usize, new_timestamp: f64) {
        self.readings_timestamp[idx] = new_timestamp;
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let timestamp = Timestamp::new();
        assert_eq!(timestamp.get_current_timestamp(), 0f64);
        for i in 0..N_SENSORS {
            assert_eq!(timestamp.get_reading_timestamp(i), 0f64);
        }
    }

    #[test]
    fn test_set_reading_timestamp() {
        let mut timestamp = Timestamp::new();
        timestamp.set_reading_timestamp(0, 1.23);
        assert_eq!(timestamp.get_reading_timestamp(0), 1.23);
        timestamp.set_reading_timestamp(1, 4.56);
        assert_eq!(timestamp.get_reading_timestamp(1), 4.56);
    }
}
