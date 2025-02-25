use std::collections::VecDeque;
/// A circular buffer type, where the buffer has a constant length of `size` elements. The buffer is filled with default samples
/// or with some initial samples given to the constructor. When a new sample is pushed to the buffer, the oldest sample is popped out.
///
/// # Examples
///
/// ```rust
/// use common::types::CircularBuffer;
///
/// // Create a Circular Buffer of 10 `i32` elements
/// let mut buffer: CircularBuffer<i32> = CircularBuffer::new(10);
///
/// let buffer_len = buffer.len();
///
/// assert_eq!(buffer_len, 10);
///
/// for i in 0..20 {
///     let out = buffer.push(i);
///     if i < 10 {
///         assert_eq!(out, 0);
///     } else {
///         assert_eq!(out, i - 10);
///     }
/// }
/// ```

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

    pub fn from_vec(data: Vec<T>) -> Self {
        Self {
            buffer: VecDeque::from(data),
        }
    }

    /// New element `elem` is pushed, and oldest is popped out
    pub fn push(&mut self, elem: T) -> T {
        let out = self.buffer.pop_front().unwrap_or_default();
        self.buffer.push_back(elem);
        out
    }

    /// Returns a reference to the front element of the CircularBuffer
    pub fn peek_front(&self) -> &T {
        self.buffer.front().unwrap()
    }

    /// Returns a reference to the back element of the CircularBuffer
    pub fn peek_back(&self) -> &T {
        self.buffer.back().unwrap()
    }

    pub fn as_slice(&self) -> &[T] {
        self.buffer.as_slices().0
    }
    pub fn as_vec(&self) -> Vec<T> {
        let (first, second) = self.buffer.as_slices();
        [first, second].concat()
    }

    /// Returns size of CircularBuffer
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.buffer.len()
    }
}

impl<T> IntoIterator for CircularBuffer<T> {
    type Item = T;
    type IntoIter = std::vec::IntoIter<T>;

    fn into_iter(self) -> Self::IntoIter {
        // Convert the VecDeque into a Vec and use IntoIter to iterate over it
        self.buffer.into_iter().collect::<Vec<T>>().into_iter()
    }
}

impl<'a, T> IntoIterator for &'a CircularBuffer<T> {
    type Item = &'a T;
    type IntoIter = std::iter::Chain<std::slice::Iter<'a, T>, std::slice::Iter<'a, T>>;

    fn into_iter(self) -> Self::IntoIter {
        let (first, second) = self.buffer.as_slices();
        first.iter().chain(second.iter()) // Chains the two slices into a single iterator
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

    #[test]
    fn test_from_vec() {
        let data = vec![1, 2, 3];
        let buffer: CircularBuffer<i32> = CircularBuffer::from_vec(data.clone());
        assert_eq!(buffer.len(), data.len());
        assert_eq!(buffer.buffer, data.into_iter().collect::<VecDeque<_>>());
    }

    #[test]
    fn test_push_and_pop() {
        let mut buffer: CircularBuffer<i32> = CircularBuffer::new(3);
        buffer.push(1);
        buffer.push(2);
        buffer.push(3);
        assert_eq!(buffer.push(4), 1);
        assert_eq!(buffer.push(5), 2);
        assert_eq!(buffer.push(6), 3);
        assert_eq!(
            buffer.buffer,
            vec![4, 5, 6].into_iter().collect::<VecDeque<_>>()
        );
    }
}
