const DEFAULT_CAPACITY: usize = 64;

#[derive(Clone, Debug)]
pub struct RollingBuffer<T> {
    buffers: Vec<Vec<T>>,
    current_buffer_index: usize,
    next_buffer_index: usize,
}

impl<T: Clone> RollingBuffer<T> {
    pub fn new(number_buffers: usize, default_capacity: Option<usize>) -> Self {
        let capacity = default_capacity.unwrap_or(DEFAULT_CAPACITY);
        Self {
            buffers: (0..number_buffers)
                .map(|_| Vec::with_capacity(capacity))
                .collect(),
            current_buffer_index: 0,
            next_buffer_index: 1 % number_buffers,
        }
    }

    pub fn push(&mut self, elem: T) {
        let buffer_index = self.current_buffer_index;
        self.buffers[buffer_index].push(elem);
    }

    pub fn pop(&mut self) -> Option<T> {
        let buffer_index = self.current_buffer_index;
        self.buffers[buffer_index].pop()
    }

    pub fn flush(&mut self) {
        let buffer_index = self.current_buffer_index;
        self.buffers[buffer_index].clear();
    }

    pub fn next(&mut self) {
        self.current_buffer_index = self.next_buffer_index;
        self.next_buffer_index = (self.next_buffer_index + 1) % self.buffers.len();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let rb: RollingBuffer<i32> = RollingBuffer::new(3, None);
        assert_eq!(rb.buffers.len(), 3);
        assert_eq!(rb.buffers[0].capacity(), DEFAULT_CAPACITY);
        assert_eq!(rb.current_buffer_index, 0);
        assert_eq!(rb.next_buffer_index, 1);
    }

    #[test]
    fn test_push_and_pop() {
        let mut rb: RollingBuffer<i32> = RollingBuffer::new(3, None);
        rb.push(1);
        rb.push(2);
        assert_eq!(rb.pop(), Some(2));
        assert_eq!(rb.pop(), Some(1));
        assert_eq!(rb.pop(), None);
    }

    #[test]
    fn test_flush() {
        let mut rb: RollingBuffer<i32> = RollingBuffer::new(3, None);
        rb.push(1);
        rb.push(2);
        rb.flush();
        assert_eq!(rb.pop(), None);
    }

    #[test]
    fn test_next() {
        let mut rb: RollingBuffer<i32> = RollingBuffer::new(3, None);
        rb.push(1);
        rb.next();
        assert_eq!(rb.pop(), None);
        rb.push(2);
        rb.next();
        assert_eq!(rb.pop(), None);
        rb.next();
        assert_eq!(rb.pop(), Some(1));
    }

    #[test]
    fn test_custom_capacity() {
        let custom_capacity = 128;
        let rb: RollingBuffer<i32> = RollingBuffer::new(3, Some(custom_capacity));
        assert_eq!(rb.buffers[0].capacity(), custom_capacity);
    }
}
