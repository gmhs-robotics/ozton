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
            heading: Angle::new::<radian>(heading),
        },
        distance: Length::new::<meter>(1.0),
        max_range: Length::new::<meter>(2.0),
        surface_mask: u128::MAX,
    });
    let start = Pose::new(
        crate::geometry::point(Length::new::<meter>(0.09), Length::new::<meter>(0.08)),
        Angle::new::<radian>(0.0),
    );
    let mut filter = ParticleFilter::new(start, ParticleConfig::default()).unwrap();
    let estimate = filter.observe(&beams, &map).unwrap();
    assert!(estimate.pose.position.length() < start.position.length() * 0.6);
}
