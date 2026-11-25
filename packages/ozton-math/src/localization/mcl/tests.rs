use super::*;

#[test]
fn orthogonal_beams_reduce_seeded_initial_error() {
    let map = [
        Surface::new(
            DVec2::new(-1.0, -1.0),
            DVec2::new(-1.0, 1.0),
            crate::qty!(1 m),
        ),
        Surface::new(DVec2::ONE, DVec2::new(-1.0, 1.0), crate::qty!(1 m)),
    ];
    let beams = [0.0, core::f64::consts::FRAC_PI_2].map(|heading| Beam {
        mount: Pose {
            position: DVec2::ZERO,
            heading: crate::qty!((heading) rad),
        },
        distance: crate::qty!(1.0 m),
        max_range: crate::qty!(2.0 m),
        surface_mask: u128::MAX,
    });
    let start = Pose::new(
        crate::geometry::point(crate::qty!(0.09 m), crate::qty!(0.08 m)),
        crate::qty!(0.0 rad),
    );
    let mut filter = ParticleFilter::new(start, ParticleConfig::default()).unwrap();
    let estimate = filter.observe(&beams, &map).unwrap();
    assert!(estimate.pose.position.length() < start.position.length() * 0.6);
}

#[test]
fn no_surface_is_neutral_and_all_missing_returns_no_estimate() {
    let map = [Surface::new(
        DVec2::new(1.0, 0.2),
        DVec2::new(1.0, 0.8),
        crate::qty!(1 m),
    )];
    let beam = Beam {
        mount: Pose::new(DVec2::ZERO, crate::qty!(-90 deg)),
        distance: crate::qty!(1 m),
        max_range: crate::qty!(2 m),
        surface_mask: 1,
    };
    let mut filter = ParticleFilter::new(Pose::default(), ParticleConfig::default()).unwrap();
    // Two modeled groups disagree about distance; the unmodeled group
    // must retain its initial probability mass after the update.
    for (index, particle) in filter.particles.iter_mut().enumerate() {
        particle.pose = Pose::new(
            match index % 4 {
                0 => DVec2::new(0.0, 0.5),
                1 => DVec2::new(0.2, 0.5),
                _ => DVec2::ZERO,
            },
            Angle::default(),
        );
    }
    let estimate = filter
        .observe_position(&[beam], &map, Angle::default())
        .unwrap();
    assert!(estimate.pose.position.x < 0.05);
    let missing_mass: f64 = filter
        .particles
        .iter()
        .filter(|p| p.pose.position.y == 0.0)
        .map(|p| p.weight)
        .sum();
    assert!((missing_mass - 0.5).abs() < 1e-10);
    let prior: Vec<_> = filter.particles.iter().map(|p| p.weight).collect();
    let missing = Beam {
        surface_mask: 0,
        ..beam
    };
    assert!(
        filter
            .observe_position(&[missing], &map, Angle::default())
            .is_none()
    );
    assert!(
        filter
            .particles
            .iter()
            .zip(prior)
            .all(|(p, weight)| p.weight == weight)
    );
}

#[test]
fn quality_rejects_out_of_range_predictions_and_downweights_distant_returns() {
    let map = [Surface::new(
        DVec2::new(-1.0, 1.0),
        DVec2::new(1.0, 1.0),
        crate::qty!(1 m),
    )];
    let beam = Beam {
        mount: Pose::default(),
        distance: crate::qty!(0.9 m),
        max_range: crate::qty!(1.5 m),
        surface_mask: 1,
    };
    let mut prior = ParticleFilter::new(Pose::default(), ParticleConfig::default()).unwrap();
    for (index, particle) in prior.particles.iter_mut().enumerate() {
        particle.pose = Pose::new(
            DVec2::new(0.0, if index % 2 == 0 { 0.1 } else { 0.0 }),
            Angle::default(),
        );
    }
    let quality = |minimum, sigma| BeamQuality {
        min_range: crate::qty!((minimum) m),
        sigma: crate::qty!((sigma) m),
    };
    let mut near = prior.clone();
    let a = near
        .observe_position_with_quality(&[beam], &[quality(0.02, 0.025)], &map, Angle::default())
        .unwrap();
    let mut far = prior.clone();
    let b = far
        .observe_position_with_quality(&[beam], &[quality(0.02, 0.1)], &map, Angle::default())
        .unwrap();
    assert!(a.pose.position.y > b.pose.position.y);
    let too_close = Beam {
        distance: crate::qty!(1.2 m),
        ..beam
    };
    assert!(
        prior
            .observe_position_with_quality(
                &[too_close],
                &[quality(1.1, 0.1)],
                &map,
                Angle::default()
            )
            .is_none()
    );
    let outside = Beam {
        max_range: crate::qty!(0.8 m),
        distance: crate::qty!(0.7 m),
        ..beam
    };
    assert!(
        prior
            .observe_position_with_quality(
                &[outside],
                &[quality(0.02, 0.1)],
                &map,
                Angle::default()
            )
            .is_none()
    );
}

#[test]
fn unexpected_short_return_cannot_relocate_local_tracking() {
    // Run 4: measured 439 mm, predicted 1414 mm. A plausible nearby
    // return is not evidence that the robot teleported toward the wall.
    let map = [Surface::new(
        DVec2::new(-1.0, 1.414),
        DVec2::new(1.0, 1.414),
        crate::qty!(1 m),
    )];
    let beam = Beam {
        mount: Pose::default(),
        distance: crate::qty!(439 mm),
        max_range: crate::qty!(2 m),
        surface_mask: 1,
    };
    let quality = BeamQuality {
        min_range: crate::qty!(20 mm),
        sigma: crate::qty!(70.7 mm),
    };
    let mut filter = ParticleFilter::new(Pose::default(), ParticleConfig::default()).unwrap();
    let before = filter.position_moments();
    for _ in 0..20 {
        assert!(
            filter
                .observe_position_with_quality(&[beam], &[quality], &map, Angle::default())
                .is_none()
        );
    }
    assert_eq!(filter.position_moments(), before);
    let good = Beam {
        distance: crate::qty!(1.35 m),
        ..beam
    };
    let estimate = filter
        .observe_position_with_quality(&[good], &[quality], &map, Angle::default())
        .unwrap();
    assert!(estimate.pose.position.y > before.0.y);
    assert!(estimate.pose.position.y.abs() < 0.2);
}

#[test]
fn surveyed_flush_contact_conditions_particles_and_releases_without_reset() {
    use crate::path::clearance::{Footprint, WallContact, WallContactConstraint};
    let footprint = Footprint::new(crate::qty!(0.2 m), crate::qty!(0.4 m)).unwrap();
    let pose = Pose::new(DVec2::new(0.0, -0.8), Angle::default());
    let contact = WallContactConstraint::new(
        WallContact::Rear,
        footprint,
        DVec2::ONE,
        pose,
        crate::qty!(1 in),
    )
    .unwrap();
    let mut filter = ParticleFilter::new(pose, ParticleConfig::default()).unwrap();
    filter.reset_at_contact(pose, contact);
    let covariance = filter.position_covariance();
    assert!(covariance.x_axis.x > 0.0);
    assert!(covariance.y_axis.y < 1e-24);
    assert!(
        filter
            .particles
            .iter()
            .all(|p| p.pose.heading == pose.heading)
    );
    filter.predict(BodyDelta {
        translation: DVec2::new(0.0, -0.0001),
        heading: crate::qty!(0.001 rad),
    });
    assert!(filter.particles.iter().all(|p| {
        let (min, _) = footprint.bounds(p.pose);
        min.y >= -1.0 - 1e-12
    }));
    let before = filter.position_covariance();
    filter.set_contact_constraint(None);
    assert_eq!(filter.position_covariance(), before);
    filter.predict(BodyDelta {
        translation: DVec2::new(0.0, 0.2),
        heading: Angle::default(),
    });
    assert!(filter.position_covariance().y_axis.y > 0.0);
}
