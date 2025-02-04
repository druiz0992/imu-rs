use std::fmt::Debug;

use common::{IMUSample, IMUUntimedSample, XYZ};

const WEIGHTED_AVERAGE_EPS: f64 = 1e-10;
const WEIGHTED_AVERAGE_ALPHA: f64 = 0.8;

/// Computes the simple average of the time, x, y, and z components of the given samples.
///
/// The result is an array `[t_avg, x_avg, y_avg, z_avg]`, where `x_avg`, `y_avg`, and `z_avg`
/// are the averages of the respective components across all samples.
///
/// # Panics
/// Panics if the input vector `samples` is empty.
pub(crate) fn compute_average<S: IMUSample>(
    samples: Vec<S>,
    timestamp: f64,
) -> Result<S, Box<dyn std::error::Error>> {
    let mut aggregate = XYZ::from_timed(samples[0].get_measurement())
        .ok_or("Error converting to untimed sample")?;
    for s in samples.iter().skip(1) {
        aggregate +=
            XYZ::from_timed(s.get_measurement()).ok_or("Error converting to untimed sample")?;
    }
    aggregate = aggregate / samples.len() as f64;
    Ok(S::from_untimed(aggregate.get_measurement(), timestamp))
}

/// Computes the weighted average of the t, x, y, and z components of the given samples,
/// where weights are inversely proportional to the distance from `mid_point`.
///
/// A small epsilon (`1e-10`) is added to avoid division by zero for very close timestamps.
///
/// The result is an array `[t_weighted_avg, x_weighted_avg, y_weighted_avg, z_weighted_avg]`.
pub(crate) fn compute_weighted_average<S: IMUSample>(
    samples: Vec<S>,
    mid_point: f64,
) -> Result<S, Box<dyn std::error::Error>> {
    let mut total_w = 0.0;
    let mut aggregate = XYZ::from_timed(S::default().get_measurement())
        .ok_or("Error converting to untimed sample")?;

    for s in samples {
        let raw_samples =
            XYZ::from_timed(s.get_measurement()).ok_or("Error converting to untimed sample")?;
        let sample_timestamp = s.get_timestamp();
        let w = (1.0 / ((sample_timestamp - mid_point).abs() + WEIGHTED_AVERAGE_EPS))
            .powf(WEIGHTED_AVERAGE_ALPHA);
        aggregate += raw_samples * w;
        total_w = total_w + w;
    }
    aggregate = aggregate / total_w;
    Ok(S::from_untimed(aggregate.get_measurement(), mid_point))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Sample3D;

    #[test]
    fn test_compute_average() {
        let samples = vec![
            Sample3D::from_vec([1.0, 2.0, 3.0, 4.0]),
            Sample3D::from_vec([2.0, 4.0, 6.0, 8.0]),
            Sample3D::from_vec([3.0, 6.0, 9.0, 12.0]),
        ];
        let result = compute_average::<Sample3D>(samples, 2.0).unwrap();
        let data = result.get_measurement();

        assert!((data[0] - 4.0).abs() < 1e-6);
        assert!((data[1] - 6.0).abs() < 1e-6);
        assert!((data[2] - 8.0).abs() < 1e-6);
    }

    #[test]
    fn test_compute_weighted_average() {
        let samples = vec![
            Sample3D::from_vec([1.0, 2.0, 3.0, 4.0]),
            Sample3D::from_vec([2.0, 4.0, 6.0, 8.0]),
            Sample3D::from_vec([3.0, 6.0, 9.0, 12.0]),
        ];
        let result = compute_weighted_average::<Sample3D>(samples, 2.0).unwrap();
        let data = result.get_measurement();

        assert!((data[0] - 4.0).abs() < 1e-6);
        assert!((data[1] - 6.0).abs() < 1e-6);
        assert!((data[2] - 8.0).abs() < 1e-6);
    }

    #[test]
    fn test_compute_weighted_average_close_to_mid_point() {
        let samples = vec![
            Sample3D::from_vec([1.9999999999, 2.0, 3.0, 4.0]),
            Sample3D::from_vec([2.0, 4.0, 6.0, 8.0]),
            Sample3D::from_vec([2.0000000001, 6.0, 9.0, 12.0]),
        ];
        let mid_point = 2.0;
        let result = compute_weighted_average(samples, mid_point).unwrap();
        let data = result.get_measurement();

        // All samples are very close to `mid_point`, so weights should still work due to `eps`.
        assert!((data[0] - 4.0).abs() < 1e-6);
        assert!((data[1] - 6.0).abs() < 1e-6);
        assert!((data[2] - 8.0).abs() < 1e-6);
    }

    #[test]
    fn test_compute_weighted_average_large_range() {
        let samples = vec![
            Sample3D::from_vec([1.0, 2.0, 3.0, 4.0]),
            Sample3D::from_vec([10.0, 20.0, 30.0, 40.0]),
        ];
        let mid_point = 5.0;
        let result = compute_weighted_average(samples, mid_point).unwrap();
        let data = result.get_measurement();

        // Weights: 1/((|1-5|+eps)^0.8 ~ 0.329, 1/(|10-5|+eps)^0.8 ~ 0.276
        // Weighted sum: x = (2.0 * 0.329) + (20.0 * 0.276) = 6.178
        assert!((data[0] - 6.178 / 0.605).abs() < 1e-1);
    }

    #[test]
    #[should_panic]
    fn test_compute_average_empty_samples() {
        let samples: Vec<Sample3D> = vec![];
        compute_average(samples, 1.0).unwrap(); // Should panic due to empty input
    }
}
