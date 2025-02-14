use std::time::{SystemTime, UNIX_EPOCH};

pub struct Clock(f64);

impl Clock {
    pub fn now() -> Self {
        let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
        let timestamp = now.as_secs() as f64 + now.subsec_micros() as f64 * 1e-6;
        Self(timestamp)
    }

    pub fn as_f64(&self) -> f64 {
        self.0
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clock_now() {
        let clock = Clock::now();
        assert!(clock.as_f64() > 0.0);
    }

    #[test]
    fn test_clock_as_f64() {
        let clock = Clock(12345.678);
        assert_eq!(clock.as_f64(), 12345.678);
    }

    #[test]
    fn test_clock_now_is_recent() {
        let clock = Clock::now();
        let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
        let timestamp = now.as_secs() as f64 + now.subsec_micros() as f64 * 1e-6;
        assert!((clock.as_f64() - timestamp).abs() < 1.0);
    }
}
