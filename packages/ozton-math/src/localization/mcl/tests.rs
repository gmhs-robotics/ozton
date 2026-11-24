use super::*;

#[test]
fn orthogonal_beams_reduce_seeded_initial_error() {
    let map = [
        Surface {
            start: DVec2::new(-1.0, -1.0),
            end: DVec2::new(-1.0, 1.0),
        },
        Surface {
            start: DVec2::ONE,
            end: DVec2::new(-1.0, 1.0),
        },
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
