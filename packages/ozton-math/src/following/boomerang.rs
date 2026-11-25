//! Allocation-free carrot guidance for a desired final pose.
use crate::geometry::{DVec2, Pose, direction};

/// A target on the destination's travel axis that approaches the goal as
/// the robot closes the distance. The lead fraction is dimensionless.
#[derive(Debug, Clone, Copy)]
pub struct Boomerang {
    lead: f64,
}

impl Boomerang {
    pub fn new(lead: f64) -> Option<Self> {
        (lead.is_finite() && (0.0..1.0).contains(&lead)).then_some(Self { lead })
    }

    pub fn carrot(self, current: DVec2, target: Pose, reverse: bool) -> DVec2 {
        let sign = if reverse { -1.0 } else { 1.0 };
        target.position
            - direction(target.heading) * (current.distance(target.position) * self.lead * sign)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{position_xyh, qty};

    #[test]
    fn carrot_shrinks_and_respects_reverse_travel() {
        let guidance = Boomerang::new(0.5).unwrap();
        let target = position_xyh!(0 m, 1 m, 0 deg);
        assert!((guidance.carrot(DVec2::ZERO, target, false).y - 0.5).abs() < 1e-12);
        assert!((guidance.carrot(DVec2::ZERO, target, true).y - 1.5).abs() < 1e-12);
        assert_eq!(
            guidance.carrot(target.position, target, false),
            target.position
        );
        let east = Pose::new(DVec2::ZERO, qty!(-90 deg));
        assert!(guidance.carrot(DVec2::Y, east, false).x < 0.0);
        assert!(Boomerang::new(1.0).is_none());
    }
}
