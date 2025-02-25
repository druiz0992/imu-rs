use rand::rngs::StdRng;
use rand_distr::{Distribution, Normal};

/// Functionality to add some Gaussian noise.
#[derive(Clone)]
pub(super) struct GaussianNoise {
    normal: Normal<f64>,
}

impl GaussianNoise {
    /// Creates new distribution from mean and stdev
    pub(super) fn new(mean: f64, stdev: f64) -> Self {
        Self {
            normal: Normal::new(mean, stdev).unwrap(),
        }
    }

    /// Sample from distribution
    pub(super) fn draw_sample(&self, rng: &mut StdRng) -> f64 {
        self.normal.sample(rng)
    }

    /// Adds noise to sample
    pub(super) fn add_noise(&self, rng: &mut StdRng, data: f64) -> f64 {
        data + self.draw_sample(rng).abs()
    }

    // Adds noise to vector of samples
    pub(super) fn add_noise_vec(&self, rng: &mut StdRng, data: Vec<f64>) -> Vec<f64> {
        let mut result = Vec::with_capacity(data.len());
        for d in data {
            result.push(d + self.draw_sample(rng));
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::{rngs::StdRng, SeedableRng};

    #[test]
    fn test_gaussian_new() {
        let mean = 0.0;
        let stdev = 1.0;
        let noise = GaussianNoise::new(mean, stdev);
        assert_eq!(noise.normal.mean(), mean);
        assert_eq!(noise.normal.std_dev(), stdev);
    }

    #[test]
    fn test_gaussian_draw() {
        let mean = 0.0;
        let stdev = 1.0;
        let mut rng = StdRng::from_entropy();
        let noise = GaussianNoise::new(mean, stdev);
        let sample = noise.draw_sample(&mut rng);
        assert!(sample >= mean - 3.0 * stdev && sample <= mean + 3.0 * stdev);
    }

    #[test]
    fn test_add_noise() {
        let mean = 0.0;
        let stdev = 1.0;
        let mut rng = StdRng::from_entropy();
        let noise = GaussianNoise::new(mean, stdev);
        let data = 5.0;
        let result = noise.add_noise(&mut rng, data);
        assert!(result >= data - 3.0 * stdev && result <= data + 3.0 * stdev);
    }

    #[test]
    fn test_add_noise_vec() {
        let mean = 0.0;
        let stdev = 1.0;
        let mut rng = StdRng::from_entropy();
        let noise = GaussianNoise::new(mean, stdev);
        let data = vec![1.0, 2.0, 3.0];
        let result = noise.add_noise_vec(&mut rng, data.clone());
        assert_eq!(result.len(), data.len());
        for i in 0..data.len() {
            assert!(result[i] >= data[i] - 3.0 * stdev && result[i] <= data[i] + 3.0 * stdev);
        }
    }
}
