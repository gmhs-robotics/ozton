//! Reachable signed stopping displacements, with bounded reaction and braking.
use glam::{DMat2, DVec2};

/// Bounds for one independent body-frame rate. All values use SI units.
#[derive(Debug, Clone, Copy)]
pub struct RateBounds {
    pub measured: f64,
    pub requested: f64,
    /// The command already in flight during the actuator reaction interval.
    pub previous: f64,
    pub uncertainty: f64,
    /// Optional physical upper acceleration bound; not a trajectory tuning gain.
    pub acceleration: Option<f64>,
    /// Positive minimum braking deceleration.
    pub braking: f64,
}

impl RateBounds {
    /// Includes the initial position, both possible signs, the reaction
    /// interval and all travel before stopping. No deceleration is assumed
    /// before braking begins, even when the target rate is smaller.
    pub fn stopping_interval(self, reaction: f64) -> Option<[f64; 2]> {
        if ![
            self.measured,
            self.requested,
            self.previous,
            self.uncertainty,
            self.braking,
            reaction,
        ]
        .into_iter()
        .all(f64::is_finite)
            || self.uncertainty < 0.0
            || self.braking <= 0.0
            || reaction < 0.0
            || self
                .acceleration
                .is_some_and(|a| !a.is_finite() || a <= 0.0)
        {
            return None;
        }
        let extent = |sign: f64| {
            let initial = (sign * self.measured + self.uncertainty).max(0.0);
            let ceiling = (sign * self.requested + self.uncertainty)
                .max(sign * self.previous + self.uncertainty)
                .max(initial);
            let (travel, speed) = if let Some(a) = self.acceleration {
                let t = ((ceiling - initial) / a).min(reaction);
                let speed = initial + a * t;
                (
                    initial * t + 0.5 * a * t * t + speed * (reaction - t),
                    speed,
                )
            } else {
                (ceiling * reaction, ceiling)
            };
            travel + speed * speed / (2.0 * self.braking)
        };
        let interval = [-extent(-1.0), extent(1.0)];
        interval.into_iter().all(f64::is_finite).then_some(interval)
    }
}

/// Invert d = v*T + v²/(2*a) using the cancellation-free positive root.
pub fn stoppable_speed(distance: f64, reaction: f64, deceleration: f64) -> Option<f64> {
    if !distance.is_finite()
        || distance < 0.0
        || !reaction.is_finite()
        || reaction < 0.0
        || !deceleration.is_finite()
        || deceleration <= 0.0
    {
        return None;
    }
    if distance == 0.0 {
        return Some(0.0);
    }
    Some(
        2.0 * distance
            / (reaction
                + crate::numeric::sqrt(reaction * reaction + 2.0 * distance / deceleration)),
    )
}

/// Axis projections of an XY covariance ellipse, scaled by the configured
/// confidence radius. Missing covariance is explicitly an unmodeled estimate.
pub fn position_padding(covariance: Option<DMat2>, sigma: f64) -> Option<DVec2> {
    if !sigma.is_finite() || sigma < 0.0 {
        return None;
    }
    let Some(p) = covariance else {
        return Some(DVec2::ZERO);
    };
    let diagonal = DVec2::new(p.x_axis.x, p.y_axis.y);
    if !p.is_finite()
        || diagonal.min_element() < 0.0
        || (p.x_axis.y - p.y_axis.x).abs() > 1e-9
        || p.determinant() < -1e-12
    {
        return None;
    }
    Some(
        DVec2::new(
            crate::numeric::sqrt(diagonal.x),
            crate::numeric::sqrt(diagonal.y),
        ) * sigma,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stopping_from_rest_with_acceleration_bound() {
        let rates = RateBounds {
            measured: 0.0,
            requested: 1.0,
            previous: 0.0,
            uncertainty: 0.0,
            acceleration: Some(2.0),
            braking: 2.0,
        };
        assert_eq!(rates.stopping_interval(0.25), Some([0.0, 0.125]));
    }

    #[test]
    fn opposite_request_cannot_erase_measured_momentum() {
        let rates = RateBounds {
            measured: 0.5,
            requested: -0.3,
            previous: 0.0,
            uncertainty: 0.0,
            acceleration: None,
            braking: 1.0,
        };
        let [negative, positive] = rates.stopping_interval(0.1).unwrap();
        assert!((negative + 0.075).abs() < 1e-12);
        assert!((positive - 0.175).abs() < 1e-12);
    }

    #[test]
    fn in_flight_command_is_not_erased_by_a_new_stop_request() {
        let rates = RateBounds {
            measured: 0.2,
            requested: 0.0,
            previous: 0.8,
            uncertainty: 0.0,
            acceleration: Some(2.0),
            braking: 1.0,
        };
        let [negative, positive] = rates.stopping_interval(0.2).unwrap();
        assert_eq!(negative, 0.0);
        assert!((positive - 0.26).abs() < 1e-12);
    }

    #[test]
    fn covariance_ellipse_expands_only_its_projected_axes() {
        let p = DMat2::from_diagonal(DVec2::new(0.01, 0.04));
        assert_eq!(position_padding(Some(p), 2.0), Some(DVec2::new(0.2, 0.4)));
        assert!(position_padding(Some(-p), 2.0).is_none());
    }
}
