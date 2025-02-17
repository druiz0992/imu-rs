use common::traits::imu::IMUFilter;
use common::traits::{IMUSample, IMUUntimedSample};
use common::types::filters::{Average, WeightedAverage};

/// Computes the simple average of the time, x, y, and z components of the given samples.
///
/// The result is an array `[t_avg, x_avg, y_avg, z_avg]`, where `x_avg`, `y_avg`, and `z_avg`
/// are the averages of the respective components across all samples.
///
/// # Panics
/// Panics if the input vector `samples` is empty.
pub(crate) fn compute_average<T>(
    timestamp: f64,
    samples: Vec<T>,
) -> Result<T, Box<dyn std::error::Error>>
where
    T: IMUSample,
    T::Untimed: IMUUntimedSample,
    Average<T::Untimed>: IMUFilter<T>,
{
    let mut filter: Average<T::Untimed> = Average::new();
    let filtered_samples = filter.filter_batch(samples)?;
    let averaged_sample = filtered_samples.last().cloned().unwrap_or_default();
    let averaged_sample = T::from_measurement(timestamp, averaged_sample.get_measurement());
    Ok(averaged_sample)
}

/// Computes the weighted average of the t, x, y, and z components of the given samples,
/// where weights are inversely proportional to the distance from `mid_point`.
///
/// A small epsilon (`1e-10`) is added to avoid division by zero for very close timestamps.
///
/// The result is an array `[t_weighted_avg, x_weighted_avg, y_weighted_avg, z_weighted_avg]`.
pub(crate) fn compute_weighted_average<T>(
    timestamp: f64,
    samples: Vec<T>,
) -> Result<T, Box<dyn std::error::Error>>
where
    T: IMUSample,
    T::Untimed: IMUUntimedSample,
    WeightedAverage<T::Untimed>: IMUFilter<T>,
{
    let mut filter: WeightedAverage<T::Untimed> = WeightedAverage::new(timestamp);
    let filtered_samples = filter.filter_batch(samples)?;
    let averaged_sample = filtered_samples.last().cloned().unwrap_or_default();
    let averaged_sample = T::from_measurement(timestamp, averaged_sample.get_measurement());
    Ok(averaged_sample)
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::types::timed::Sample3D;

    #[test]
    fn test_compute_average() {
        let samples = vec![
            Sample3D::new(1.0, [2.0, 3.0, 4.0]),
            Sample3D::new(2.0, [3.0, 4.0, 5.0]),
            Sample3D::new(3.0, [4.0, 5.0, 6.0]),
        ];
        let result = compute_average(1.0, samples).unwrap();
        let averaged_sample = result.get_measurement().inner();

        assert_eq!(averaged_sample[0], 3.0);
        assert_eq!(averaged_sample[1], 4.0);
        assert_eq!(averaged_sample[2], 5.0);
    }

    #[test]
    #[should_panic]
    fn test_compute_average_empty() {
        let samples: Vec<Sample3D> = vec![];
        compute_average(1.0, samples).unwrap();
    }

    #[test]
    fn test_compute_weighted_average() {
        let samples = vec![
            Sample3D::new(1.0, [2.0, 3.0, 4.0]),
            Sample3D::new(2.0, [3.0, 4.0, 5.0]),
            Sample3D::new(3.0, [4.0, 5.0, 6.0]),
        ];
        let mid_point = 2.0;
        let result = compute_weighted_average(mid_point, samples).unwrap();
        let averaged_sample = result.get_measurement().inner();

        assert_eq!(result.get_timestamp_secs(), 2.0);
        assert_eq!(averaged_sample[0], 3.0);
        assert_eq!(averaged_sample[1], 4.0);
        assert_eq!(averaged_sample[2], 5.0);
    }

    #[test]
    #[should_panic]
    fn test_compute_weighted_average_empty() {
        let samples: Vec<Sample3D> = vec![];
        let mid_point = 2.0;
        compute_weighted_average(mid_point, samples).unwrap();
    }
}
