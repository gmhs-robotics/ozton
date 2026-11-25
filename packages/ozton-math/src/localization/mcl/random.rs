use glam::{DVec2, DVec3};
use rand::{RngCore, SeedableRng, rngs::SmallRng};

#[derive(Clone)]
pub(super) struct Random(SmallRng, Option<f64>);

impl Random {
    pub(super) fn seeded(seed: u64) -> Self {
        Self(SmallRng::seed_from_u64(seed), None)
    }

    pub(super) fn uniform(&mut self) -> f64 {
        open_unit(self.0.next_u64())
    }

    fn normal_pair(&mut self) -> DVec2 {
        let radius = crate::numeric::sqrt(-2.0 * libm::log(self.uniform()));
        let angle = core::f64::consts::TAU * self.uniform();
        let (sin, cos) = libm::sincos(angle);
        DVec2::new(cos, sin) * radius
    }

    pub(super) fn normal3(&mut self) -> DVec3 {
        let xy = self.normal_pair();
        let z = match self.1.take() {
            Some(value) => value,
            None => {
                let pair = self.normal_pair();
                self.1 = Some(pair.y);
                pair.x
            }
        };
        DVec3::new(xy.x, xy.y, z)
    }
}

/// Construct the same 52-bit midpoint uniform variate without converting a
/// 64-bit integer to a double. Both operands are within a factor of two,
/// so the subtraction is exact (Sterbenz's lemma).
#[inline]
fn open_unit(bits: u64) -> f64 {
    const BELOW_ONE: f64 = f64::from_bits(1.0_f64.to_bits() - 1);
    f64::from_bits(1.0_f64.to_bits() | (bits >> 12)) - BELOW_ONE
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bit_constructed_uniform_matches_original_midpoints() {
        let mut rng = SmallRng::seed_from_u64(1);
        for bits in [0, u64::MAX, 1 << 63]
            .into_iter()
            .chain((0..10_000).map(|_| rng.next_u64()))
        {
            let expected = ((bits >> 12) as f64 + 0.5) / ((1_u64 << 52) as f64);
            let actual = open_unit(bits);
            assert_eq!(actual.to_bits(), expected.to_bits());
            assert!(actual > 0.0 && actual < 1.0);
        }
    }
}
