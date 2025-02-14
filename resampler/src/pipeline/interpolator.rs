use std::collections::HashMap;

use common::traits::{IMUSample, IMUUntimedSample};
use common::types::buffers::CircularBuffer;
use common::types::sensors::SensorType;

pub(crate) struct Interpolator<S>
where
    S: IMUSample,
    S::Untimed: IMUUntimedSample,
{
    cache: HashMap<SensorType, CircularBuffer<S>>,
}

impl<S> Interpolator<S>
where
    S: IMUSample,
    S::Untimed: IMUUntimedSample,
{
    pub(crate) fn new(sensor_cluster: &[SensorType]) -> Self {
        let mut cache = HashMap::<SensorType, CircularBuffer<S>>::new();
        for sensor_type in sensor_cluster.iter() {
            cache.insert(sensor_type.clone(), CircularBuffer::new(2));
        }
        Self { cache }
    }

    pub(crate) fn push(&mut self, sensor_type: &SensorType, elem: S) {
        let buffer = self.cache.get_mut(sensor_type).unwrap();
        let newest = buffer.peek_back();
        if newest.get_timestamp() < elem.get_timestamp() {
            buffer.push(elem);
        }
    }

    pub(crate) fn pop(&mut self, sensor_type: &SensorType) -> S {
        todo!();
    }
}
