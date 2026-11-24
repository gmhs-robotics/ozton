//! Normalized finite-return beam mixture (Probabilistic Robotics, chapter 6).
//! No-return packets are handled by the sensor adapter rather than as distances.

#[derive(Debug, Clone, Copy)]
pub struct BeamModel {
    /// Fraction of the outlier prior assigned to unexpected short returns.
    pub short_fraction: f64,
    /// Exponential decay length, as a fraction of the usable sensor range.
    pub short_scale: f64,
    /// Minimum posterior hit probability for accepting an observation.
    pub minimum_hit_probability: f64,
}

impl Default for BeamModel {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl BeamModel {
    pub const DEFAULT: Self = Self {
        short_fraction: 0.5,
        short_scale: 1.0,
        minimum_hit_probability: 0.5,
    };

    pub const fn new() -> Self {
        Self::DEFAULT
    }

    #[must_use]
    pub const fn with_short_returns(mut self, fraction: f64, scale: f64) -> Self {
        self.short_fraction = fraction;
        self.short_scale = scale;
        self
    }

    #[must_use]
    pub const fn with_minimum_hit_probability(mut self, probability: f64) -> Self {
        self.minimum_hit_probability = probability;
        self
    }

    pub(super) fn is_valid(self) -> bool {
        (0.0..=1.0).contains(&self.short_fraction)
            && self.short_scale.is_finite()
            && self.short_scale > 0.0
            && (0.0..=1.0).contains(&self.minimum_hit_probability)
    }

    /// Returns the weighted hit density and total mixture density.
    pub(super) fn likelihood(
        self,
        measured: f64,
        predicted: f64,
        minimum: f64,
        maximum: f64,
        sigma: f64,
        outlier: f64,
    ) -> (f64, f64) {
        let span = maximum - minimum;
        let root_two_sigma = core::f64::consts::SQRT_2 * sigma;
        let normalizer = 0.5
            * (libm::erf((maximum - predicted) / root_two_sigma)
                - libm::erf((minimum - predicted) / root_two_sigma));
        let residual = (measured - predicted) / sigma;
        let hit = (1.0 - outlier) * libm::exp(-0.5 * residual * residual)
            / (libm::sqrt(core::f64::consts::TAU) * sigma * normalizer);
        let random = outlier * (1.0 - self.short_fraction) / span;
        let scale = self.short_scale * span;
        let short = if measured < predicted && predicted > minimum {
            outlier * self.short_fraction * libm::exp(-(measured - minimum) / scale)
                / (scale * -libm::expm1(-(predicted - minimum) / scale))
        } else {
            0.0
        };
        (hit, hit + random + short)
    }
}
