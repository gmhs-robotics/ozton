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
    let estimate = filter.observe_position(&[beam], &map, Angle::default()).unwrap();
    assert!(estimate.pose.position.x < 0.05);
    let missing_mass: f64 = filter.particles.iter()
        .filter(|p| p.pose.position.y == 0.0).map(|p| p.weight).sum();
    assert!((missing_mass - 0.5).abs() < 1e-10);
    let prior: Vec<_> = filter.particles.iter().map(|p| p.weight).collect();
    let missing = Beam { surface_mask: 0, ..beam };
    assert!(filter.observe_position(&[missing], &map, Angle::default()).is_none());
    assert!(filter.particles.iter().zip(prior).all(|(p, weight)| p.weight == weight));
}
