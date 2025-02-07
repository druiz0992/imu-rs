#[derive(Clone, Debug)]
pub struct CircularReader<T: Clone> {
    buffer: Vec<T>,
    index: usize,
}

impl<T: Clone> CircularReader<T> {
    /// Creates a new CircularReader with preloaded data.
    pub fn new(data: Vec<T>) -> Result<Self, Box<dyn std::error::Error>> {
        if data.is_empty() {
            return Err(Box::<dyn std::error::Error>::from("Buffer cannot be empty"));
        }
        Ok(Self {
            buffer: data,
            index: 0,
        })
    }

    /// Reads the next element, moving the index forward cyclically.
    pub fn next(&mut self) -> T {
        let elem = &self.buffer[self.index];
        self.index = (self.index + 1) % self.buffer.len();
        elem.clone()
    }

    /// Peeks at the current element without advancing the index.
    pub fn peek(&self) -> &T {
        &self.buffer[self.index]
    }

    /// Resets the reader back to the start.
    pub fn reset(&mut self) {
        self.index = 0;
    }
}

impl<T: Clone> TryFrom<Vec<T>> for CircularReader<T> {
    type Error = String;
    fn try_from(value: Vec<T>) -> Result<Self, Self::Error> {
        Self::new(value).map_err(|e| e.to_string())
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_init() {
        let data = vec![10, 20, 30, 40];
        let mut reader = CircularReader::new(data.clone()).unwrap();

        for i in 0..6 {
            assert_eq!(reader.next(), data[i % data.len()]);
        }
    }
}
