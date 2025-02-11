use common::traits::imu::IMUFilter;
use common::types::filters::{MovingAverage, WeightedMovingAverage};
use common::{IMUSample, IMUUntimedSample};

/// Computes the simple average of the time, x, y, and z components of the given samples.
///
/// The result is an array `[t_avg, x_avg, y_avg, z_avg]`, where `x_avg`, `y_avg`, and `z_avg`
/// are the averages of the respective components across all samples.
///
/// # Panics
/// Panics if the input vector `samples` is empty.
pub(crate) fn compute_average<T>(samples: Vec<T>) -> Result<T, Box<dyn std::error::Error>>
where
    T: IMUSample,
    T::Untimed: IMUUntimedSample,
    MovingAverage<T::Untimed>: IMUFilter<T>,
{
    let mut filter: MovingAverage<T::Untimed> = MovingAverage::new(samples.len());
    let filtered_samples = filter.filter_batch(samples)?;
    let averaged_sample = filtered_samples.last().cloned().unwrap_or_default();
    Ok(averaged_sample)
}

/// Computes the weighted average of the t, x, y, and z components of the given samples,
/// where weights are inversely proportional to the distance from `mid_point`.
///
/// A small epsilon (`1e-10`) is added to avoid division by zero for very close timestamps.
///
/// The result is an array `[t_weighted_avg, x_weighted_avg, y_weighted_avg, z_weighted_avg]`.
pub(crate) fn compute_weighted_average<T>(
    samples: Vec<T>,
    mid_point: f64,
) -> Result<T, Box<dyn std::error::Error>>
where
    T: IMUSample,
    T::Untimed: IMUUntimedSample,
    WeightedMovingAverage<T::Untimed>: IMUFilter<T>,
{
    let mut filter: WeightedMovingAverage<T::Untimed> = WeightedMovingAverage::new(mid_point);
    let filtered_samples = filter.filter_batch(samples)?;
    let averaged_sample = filtered_samples.last().cloned().unwrap_or_default();
    Ok(averaged_sample)
}
