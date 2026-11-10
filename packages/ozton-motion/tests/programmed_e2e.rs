use std::{
    convert::Infallible,
    f64::consts::PI,
    time::{Duration, Instant},
};

use glam::DVec2;
use ozton_math::{
    geometry::{BodyDelta, Pose},
    path::{Limits, Path, Waypoint, follower::Follower},
};
use ozton_motion::auton::{AutonError, AutonRobot, Command, DriveOptions, Feedback, Runner};
use vex_sdk_mock as _;
use vexide::runtime::block_on;

const OPTIONS: DriveOptions = DriveOptions {
    track_width: 0.3,
    maximum_wheel_speed: 1.0,
    position_tolerance: 0.018,
    heading_tolerance: 0.09,
    speed_tolerance: 0.03,
    angular_speed_tolerance: 0.08,
    settle: Duration::from_millis(40),
};

struct Sim {
    pose: Pose,
    travel: f64,
    last: Instant,
    wheels: [f64; 2],
    intake: bool,
    intake_moving_ticks: usize,
    stopped: bool,
}

impl Sim {
    fn new(reverse: bool) -> Self {
        Self {
            pose: Pose::new(0.0, 0.0, if reverse { PI } else { 0.0 }),
            travel: 0.0,
            last: Instant::now(),
            wheels: [0.0; 2],
            intake: false,
            intake_moving_ticks: 0,
            stopped: false,
        }
    }
}

impl AutonRobot for Sim {
    type Error = Infallible;

    fn sample(&mut self) -> Result<Option<Feedback>, Self::Error> {
        let now = Instant::now();
        let dt = now.duration_since(self.last).as_secs_f64();
        self.last = now;
        let [left, right] = self.wheels;
        let speed = (left + right) / 2.0;
        let angular_speed = (right - left) / OPTIONS.track_width;
        self.travel += speed * dt;
        self.pose = self.pose.integrate(BodyDelta {
            translation: DVec2::new(speed * dt, 0.0),
            heading: angular_speed * dt,
        });
        Ok(Some(Feedback {
            pose: self.pose,
            forward_travel: self.travel,
            forward_velocity: speed,
            angular_velocity: angular_speed,
            ranges: [None; 8],
            reset_accepted: false,
        }))
    }

    fn update_mechanisms(&mut self, _: Duration) -> Result<(), Self::Error> {
        if self.intake && self.wheels != [0.0; 2] {
            self.intake_moving_ticks += 1;
        }
        Ok(())
    }

    fn drive(&mut self, wheels: [f64; 2], _: Duration) -> Result<(), Self::Error> {
        assert!(
            wheels
                .iter()
                .all(|wheel| wheel.abs() <= OPTIONS.maximum_wheel_speed + 1e-9)
        );
        self.wheels = wheels;
        Ok(())
    }

    fn stop_drive(&mut self) -> Result<(), Self::Error> {
        self.wheels = [0.0; 2];
        Ok(())
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.stopped = true;
        self.intake = false;
        self.stop_drive()
    }

    fn set_pose(&mut self, pose: Pose) -> Result<(), Self::Error> {
        self.pose = pose;
        Ok(())
    }

    fn request_distance_reset(&mut self) {}
}

fn route() -> Path {
    let points: Vec<_> = (0..=12)
        .map(|index| {
            let x = f64::from(index) * 0.05;
            Waypoint::new(x, 0.22 * x.powi(2), 0.8)
        })
        .collect();
    Path::new(
        &points,
        Limits {
            track_width: OPTIONS.track_width,
            linear_speed: 0.8,
            wheel_speed: 1.0,
            acceleration: 2.0,
            deceleration: 2.0,
            lateral_acceleration: 1.0,
        },
    )
    .unwrap()
}

#[test]
fn followers_converge_and_cancel_with_mechanisms_running() {
    block_on(async {
        let path = route();
        for (follower, reverse) in [
            (Follower::PurePursuit { lookahead: 0.14 }, false),
            (Follower::Ramsete { b: 2.0, zeta: 0.7 }, true),
        ] {
            let mut robot = Sim::new(reverse);
            {
                let mut runner = Runner::new(
                    &mut robot,
                    OPTIONS,
                    Duration::from_millis(5),
                    Duration::from_millis(100),
                    Duration::from_secs(6),
                )
                .unwrap();
                runner
                    .run_while(
                        Command::Follow {
                            path: &path,
                            follower,
                            reverse,
                            trigger: Some((0.12, |robot: &mut Sim| robot.intake = true)),
                        },
                        Duration::from_secs(3),
                        || true,
                    )
                    .await
                    .unwrap();
                let robot = runner.robot();
                assert!(
                    robot.pose.position.distance(path.end().position) <= OPTIONS.position_tolerance
                );
                assert_eq!(robot.wheels, [0.0; 2]);
                assert!(robot.intake_moving_ticks > 0);
                assert!(robot.intake);

                let mut enabled_ticks = 0;
                let outcome = runner
                    .run_while(
                        Command::DriveDistance {
                            meters: 0.4,
                            heading: path.end().heading + if reverse { PI } else { 0.0 },
                            maximum_speed: 0.4,
                        },
                        Duration::from_secs(1),
                        || {
                            enabled_ticks += 1;
                            enabled_ticks < 5
                        },
                    )
                    .await;
                assert!(matches!(outcome, Err(AutonError::Cancelled)));
                assert!(runner.robot().stopped);
                assert!(!runner.robot().intake);
            }
            assert_eq!(robot.wheels, [0.0; 2]);
        }
    });
}

#[test]
fn ordered_markers_and_optional_alignment_keep_the_route_running() {
    use ozton_motion::auton::Alignment;

    block_on(async {
        let path = route();
        let mut robot = Sim::new(false);
        let mut runner = Runner::new(
            &mut robot,
            OPTIONS,
            Duration::from_millis(5),
            Duration::from_millis(100),
            Duration::from_secs(4),
        )
        .unwrap();
        let markers = [
            (
                0.10,
                (|robot: &mut Sim| robot.intake = true) as fn(&mut Sim),
            ),
            (0.40, |robot: &mut Sim| {
                assert!(robot.intake_moving_ticks > 0);
                robot.intake = false;
            }),
        ];
        runner
            .run_while(
                Command::FollowMarked {
                    path: &path,
                    follower: Follower::PurePursuit { lookahead: 0.14 },
                    reverse: false,
                    markers: &markers,
                },
                Duration::from_secs(3),
                || true,
            )
            .await
            .unwrap();
        assert!(!runner.robot().intake);
        assert!(
            !runner
                .try_align_while(
                    Alignment {
                        sensor: 0,
                        distance: 0.12,
                        heading: 0.0,
                        maximum_speed: 0.2,
                        direction: 1.0,
                    },
                    Duration::from_millis(30),
                    || true
                )
                .await
                .unwrap()
        );
        assert_eq!(runner.robot().wheels, [0.0; 2]);
        runner
            .run_while(
                Command::WaitUntil(|robot: &Sim| !robot.intake),
                Duration::from_millis(50),
                || true,
            )
            .await
            .unwrap();
        assert!(!runner.robot().stopped);
    });
}
