use glam::{DVec2, DVec3};
use rand::{RngCore, SeedableRng, rngs::SmallRng};

#[derive(Clone)]
pub(super) struct Random(SmallRng);

impl Random {
    pub(super) fn seeded(seed: u64) -> Self {
        Self(SmallRng::seed_from_u64(seed))
    }

    pub(super) fn uniform(&mut self) -> f64 {
        ((self.0.next_u64() >> 12) as f64 + 0.5) / ((1_u64 << 52) as f64)
    }

    fn normal_pair(&mut self) -> DVec2 {
        let radius = libm::sqrt(-2.0 * libm::log(self.uniform()));
        let angle = core::f64::consts::TAU * self.uniform();
        let (sin, cos) = libm::sincos(angle);
        DVec2::new(cos, sin) * radius
    }

    pub(super) fn normal3(&mut self) -> DVec3 {
        let xy = self.normal_pair();
        DVec3::new(xy.x, xy.y, self.normal_pair().x)
    }
}
