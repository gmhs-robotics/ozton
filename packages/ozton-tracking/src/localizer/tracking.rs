//! Typed composition of a localizer with devices the rest of the robot also uses.

use super::{Ekf, Filter, Localizer, Mcl, Odometry, PoseEstimator};

/// An owned localizer and its optional shared distance devices. If one
/// component alone owns the distance source, leave `distances` as `()` and
/// feed its observations directly to the localizer.
#[must_use]
pub struct Tracking<'a, E = Filter, D = ()> {
    pub localizer: Localizer<'a, E>,
    pub distances: D,
}

impl<'a, E> Tracking<'a, E> {
    pub const fn new(localizer: Localizer<'a, E>) -> Self {
        Self {
            localizer,
            distances: (),
        }
    }
}

impl<'a, E, D> Tracking<'a, E, D> {
    /// Expose distance devices used by both tracking and other robot functions.
    pub fn with_distances<N>(self, distances: N) -> Tracking<'a, E, N> {
        Tracking {
            localizer: self.localizer,
            distances,
        }
    }
}

impl<'a, E: PoseEstimator, D> Tracking<'a, E, D> {
    pub fn with_odometry(self, odometry: Odometry) -> Tracking<'a, Odometry, D> {
        self.with_filter(odometry)
    }

    pub fn with_ekf(self, ekf: Ekf) -> Tracking<'a, Ekf, D> {
        self.with_filter(ekf)
    }

    pub fn with_mcl(self, mcl: Mcl) -> Tracking<'a, Mcl, D> {
        self.with_filter(mcl)
    }

    fn with_filter<N: PoseEstimator>(self, filter: N) -> Tracking<'a, N, D> {
        Tracking {
            localizer: self.localizer.with_filter(filter),
            distances: self.distances,
        }
    }
}

impl<E: PoseEstimator, D> crate::Tracking for Tracking<'_, E, D> {}

impl<E: PoseEstimator, D> crate::TracksPosition for Tracking<'_, E, D> {
    fn position(&self) -> glam::DVec2 {
        self.localizer.position()
    }
}

impl<E: PoseEstimator, D> crate::TracksHeading for Tracking<'_, E, D> {
    fn heading(&self) -> vexide::math::Angle {
        self.localizer.heading()
    }
}

impl<E: PoseEstimator, D> crate::TracksForwardTravel for Tracking<'_, E, D> {
    fn forward_travel(&self) -> f64 {
        self.localizer.forward_travel()
    }
}

impl<E: PoseEstimator, D> crate::TracksVelocity for Tracking<'_, E, D> {
    fn linear_velocity(&self) -> f64 {
        self.localizer.linear_velocity()
    }

    fn angular_velocity(&self) -> f64 {
        self.localizer.angular_velocity()
    }
}
