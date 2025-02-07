use std::collections::VecDeque;

/// Circular buffer definition. Buffer has a constant length of `size` elements. Buffer is filled by default samples.
/// When one new sample is pushed to the buffer, the oldest sample is popped out.
#[derive(Clone, Debug)]
pub struct CircularBuffer<T> {
    #[allow(dead_code)]
    buffer: VecDeque<T>,
}

impl<T: Clone + Default> CircularBuffer<T> {
    /// Creates new CircularBuffer with `size` number of default elements of type `T`.
    pub fn new(size: usize) -> Self {
        Self {
            buffer: VecDeque::from(vec![T::default(); size]),
        }
    }

    /// New element `elem` is pushed, and oldest is popped out
    pub fn push(&mut self, elem: T) -> T {
        let out = self.buffer.pop_front().unwrap_or(T::default());
        self.buffer.push_back(elem);
        out
    }

    /// Returns size of CircularBuffer
    pub fn len(&self) -> usize {
        self.buffer.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let buffer: CircularBuffer<i32> = CircularBuffer::new(3);
        assert_eq!(buffer.len(), 3);
        assert_eq!(buffer.buffer.len(), 3);
        assert!(buffer.buffer.iter().all(|&x| x == 0));
    }

    #[test]
    fn test_push() {
        let mut buffer: CircularBuffer<i32> = CircularBuffer::new(3);
        assert_eq!(buffer.push(1), 0);
        assert_eq!(buffer.push(2), 0);
        assert_eq!(buffer.push(3), 0);
        assert_eq!(buffer.push(4), 1);
        assert_eq!(
            buffer.buffer,
            vec![2, 3, 4].into_iter().collect::<VecDeque<_>>()
        );
    }

    #[test]
    fn test_len() {
        let buffer: CircularBuffer<i32> = CircularBuffer::new(5);
        assert_eq!(buffer.len(), 5);
    }
}
