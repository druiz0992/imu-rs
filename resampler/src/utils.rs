/// Computes the simple average of the time, x, y, and z components of the given samples.
///
/// The result is an array `[t_avg, x_avg, y_avg, z_avg]`, where `x_avg`, `y_avg`, and `z_avg`
/// are the averages of the respective components across all samples.
///
/// # Panics
/// Panics if the input vector `samples` is empty.
pub(crate) fn compute_average(samples: &Vec<[f64; 4]>, timestamp: f64) -> [f64; 4] {
    let mut x_data = samples[0][1];
    let mut y_data = samples[0][2];
    let mut z_data = samples[0][3];
    for s in samples.iter().skip(1) {
        x_data += s[1];
        y_data += s[2];
        z_data += s[3];
    }
    [
        timestamp,
        x_data / samples.len() as f64,
        y_data / samples.len() as f64,
        z_data / samples.len() as f64,
    ]
}

/// Computes the weighted average of the t, x, y, and z components of the given samples,
/// where weights are inversely proportional to the distance from `mid_point`.
///
/// A small epsilon (`1e-10`) is added to avoid division by zero for very close timestamps.
///
/// The result is an array `[t_weighted_avg, x_weighted_avg, y_weighted_avg, z_weighted_avg]`.
pub(crate) fn compute_weighted_average(samples: &Vec<[f64; 4]>, mid_point: f64) -> [f64; 4] {
    let mut total_w = 0.0;
    let mut x_data = 0.0;
    let mut y_data = 0.0;
    let mut z_data = 0.0;
    let eps = 1e-10;

    for s in samples {
        let w = (1.0 / ((s[0] - mid_point).abs() + eps)).powf(0.8);
        x_data += w * s[1];
        y_data += w * s[2];
        z_data += w * s[3];
        total_w += w;
    }
    [
        mid_point,
        x_data / total_w,
        y_data / total_w,
        z_data / total_w,
    ]
    //[mid_point, x_data / total_w, y_data / total_w, z_data / total_w]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_average() {
        let samples = vec![
            [1.0, 2.0, 3.0, 4.0],
            [2.0, 4.0, 6.0, 8.0],
            [3.0, 6.0, 9.0, 12.0],
        ];
        let result = compute_average(&samples, 2.0);

        assert!((result[0] - 2.0).abs() < 1e-6);
        assert!((result[1] - 4.0).abs() < 1e-6);
        assert!((result[2] - 6.0).abs() < 1e-6);
        assert!((result[3] - 8.0).abs() < 1e-6);
    }

    #[test]
    fn test_compute_weighted_average() {
        let samples = vec![
            [1.0, 2.0, 3.0, 4.0],
            [2.0, 4.0, 6.0, 8.0],
            [3.0, 6.0, 9.0, 12.0],
        ];
        let mid_point = 2.0;
        let result = compute_weighted_average(&samples, mid_point);

        // Weights: 1.0 for 2.0, ~0.5 for 1.0 and 3.0 (due to eps adjustment)
        // Weighted sums: x: 4.0 + (2.0 * ~0.5) + (6.0 * ~0.5) = 8.0
        // Divide by total weight (~2.0): x_avg = 8.0 / 2.0 = 4.0
        assert!((result[0] - 2.0).abs() < 1e-6);
        assert!((result[1] - 4.0).abs() < 1e-6);
        assert!((result[2] - 6.0).abs() < 1e-6);
        assert!((result[3] - 8.0).abs() < 1e-6);
    }

    #[test]
    fn test_compute_weighted_average_close_to_mid_point() {
        let samples = vec![
            [1.9999999999, 2.0, 3.0, 4.0],
            [2.0, 4.0, 6.0, 8.0],
            [2.0000000001, 6.0, 9.0, 12.0],
        ];
        let mid_point = 2.0;
        let result = compute_weighted_average(&samples, mid_point);

        // All samples are very close to `mid_point`, so weights should still work due to `eps`.
        assert!((result[1] - 4.0).abs() < 1e-6);
        assert!((result[1] - 4.0).abs() < 1e-6);
        assert!((result[2] - 6.0).abs() < 1e-6);
        assert!((result[3] - 8.0).abs() < 1e-6);
    }

    #[test]
    fn test_compute_weighted_average_large_range() {
        let samples = vec![[1.0, 2.0, 3.0, 4.0], [10.0, 20.0, 30.0, 40.0]];
        let mid_point = 5.0;
        let result = compute_weighted_average(&samples, mid_point);

        // Weights: 1/(|1-5|+eps) ~ 0.25, 1/(|10-5|+eps) ~ 0.2
        // Weighted sum: x = (2.0 * 0.25) + (20.0 * 0.2) = 4.5
        assert!((result[1] - 4.5 / 0.45).abs() < 1e-6);
    }

    #[test]
    #[should_panic]
    fn test_compute_average_empty_samples() {
        let samples: Vec<[f64; 4]> = vec![];
        compute_average(&samples, 1.0); // Should panic due to empty input
    }
}
