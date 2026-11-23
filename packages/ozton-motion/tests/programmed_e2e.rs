use std::{
    f64::consts::PI,
    time::{Duration, Instant},
};

use glam::DVec2;
use ozton_math::{
    geometry::{BodyDelta, Pose},
    path::{Limits, Path, Waypoint, follower::Follower},
    uom::{
        ConstZero,
        si::{
            acceleration::meter_per_second_squared,
            angle::radian,
            angular_velocity::radian_per_second,
            f64::{Acceleration, Angle, AngularVelocity, Length, Velocity},
            length::meter,
            velocity::meter_per_second,
        },
    },
};
use ozton_motion::commands::{
    Command, CommandError, DriveOptions, Feedback, MotionPlatform, Runner,
};
use vex_sdk_mock as _;
use vexide::runtime::block_on;

const OPTIONS: DriveOptions = DriveOptions {
    track_width: Length {
        value: 0.3,
        ..Length::ZERO
    },
    maximum_wheel_speed: Velocity {
        value: 1.0,
        ..Velocity::ZERO
    },
    position_tolerance: Length {
        value: 0.018,
        ..Length::ZERO
    },
    heading_tolerance: Angle {
        value: 0.09,
        ..Angle::ZERO
    },
    speed_tolerance: Velocity {
        value: 0.03,
        ..Velocity::ZERO
    },
    angular_speed_tolerance: AngularVelocity {
        value: 0.08,
        ..AngularVelocity::ZERO
    },
    settle: Duration::from_millis(40),
};

struct Sim {
    pose: Pose,
    travel: f64,
    last: Instant,
    started: Instant,
    wheels: [f64; 2],
    intake: bool,
    intake_moving_ticks: usize,
    stopped: bool,
    stop_failures: usize,
    cancelled_resets: usize,
    trusted: bool,
    jammed: bool,
    transient_samples: usize,
    scoring_ticks: usize,
    score_goal: bool,
    ejected_before_ready: bool,
}

impl Sim {
    fn new(reverse: bool) -> Self {
        Self {
            pose: Pose::new(
                ozton_math::geometry::point(Length::new::<meter>(0.0), Length::new::<meter>(0.0)),
                Angle::new::<radian>(if reverse { PI } else { 0.0 }),
            ),
            travel: 0.0,
            last: Instant::now(),
            started: Instant::now(),
            wheels: [0.0; 2],
            intake: false,
            intake_moving_ticks: 0,
            stopped: false,
            stop_failures: 0,
            cancelled_resets: 0,
            trusted: true,
            jammed: false,
            transient_samples: 0,
            scoring_ticks: 0,
            score_goal: false,
            ejected_before_ready: false,
        }
    }
}

#[derive(Debug)]
enum SimError {
    Transient,
}

impl MotionPlatform for Sim {
    fn pose_trusted(&self) -> bool {
        self.trusted
    }
    fn recoverable(&self, _: &Self::Error) -> bool {
        true
    }
    fn can_skip_failure(&self, _: &Self::Error) -> bool {
        true
    }
    type Error = SimError;

    fn sample(&mut self) -> Result<Option<Feedback>, Self::Error> {
        if self.transient_samples > 0 {
            self.transient_samples -= 1;
            return Err(SimError::Transient);
        }
        let now = Instant::now();
        let dt = now.duration_since(self.last).as_secs_f64();
        self.last = now;
        let [left, right] = self.wheels;
        let speed = if self.jammed {
            0.0
        } else {
            (left + right) / 2.0
        };
        let angular_speed = if self.jammed {
            0.0
        } else {
            (right - left) / OPTIONS.track_width.value
        };
        self.travel += speed * dt;
        self.pose = self.pose.integrate(BodyDelta {
            translation: DVec2::new(0.0, speed * dt),
            heading: Angle::new::<radian>(angular_speed * dt),
        });
        Ok(Some(Feedback {
            time: now.saturating_duration_since(self.started),
            pose: self.pose,
            forward_travel: Length::new::<meter>(self.travel),
            forward_velocity: Velocity::new::<meter_per_second>(speed),
            lateral_velocity: Velocity::new::<meter_per_second>(0.0),
            angular_velocity: AngularVelocity::new::<radian_per_second>(angular_speed),
            ranges: [None; 8],
            reset_accepted: false,
        }))
    }

    fn update_subsystems(&mut self, _: Duration) -> Result<(), Self::Error> {
        if self.score_goal {
            self.scoring_ticks += 1;
        }
        if self.intake && self.wheels != [0.0; 2] {
            self.intake_moving_ticks += 1;
        }
        Ok(())
    }

    fn drive(&mut self, wheels: [Velocity; 2], _: Duration) -> Result<(), Self::Error> {
        let wheels = wheels.map(|speed| speed.value);
        assert!(
            wheels
                .iter()
                .all(|wheel| wheel.abs() <= OPTIONS.maximum_wheel_speed.value + 1e-9)
        );
        self.wheels = wheels;
        Ok(())
    }

    fn stop_drive(&mut self) -> Result<(), Self::Error> {
        self.wheels = [0.0; 2];
        Ok(())
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        if self.stop_failures > 0 {
            self.stop_failures -= 1;
            return Err(SimError::Transient);
        }
        self.stopped = true;
        self.intake = false;
        self.stop_drive()
    }

    fn set_pose(&mut self, pose: Pose) -> Result<(), Self::Error> {
        self.pose = pose;
        self.trusted = true;
        Ok(())
    }

    fn request_distance_reset(&mut self) {}
    fn cancel_distance_reset(&mut self) {
        self.cancelled_resets += 1;
    }
}

#[expect(
    clippy::expect_used,
    reason = "invalid path fixture must fail the test"
)]
fn route() -> Path {
    let points: Vec<_> = (0..=12)
        .map(|index| {
            let x = f64::from(index) * 0.05;
            Waypoint::new(
                ozton_math::geometry::point(
                    Length::new::<meter>(-0.22 * x.powi(2)),
                    Length::new::<meter>(x),
                ),
                0.8,
            )
        })
        .collect();
    Path::new(
        &points,
        Limits {
            track_width: Length::new::<meter>(OPTIONS.track_width.value),
            linear_speed: Velocity::new::<meter_per_second>(0.8),
            wheel_speed: Velocity::new::<meter_per_second>(1.0),
            acceleration: Acceleration::new::<meter_per_second_squared>(2.0),
            deceleration: Acceleration::new::<meter_per_second_squared>(2.0),
            lateral_acceleration: Acceleration::new::<meter_per_second_squared>(1.0),
        },
    )
    .expect("valid motion path fixture")
}

#[test]
fn forward_arc_reaches_the_expected_pose() {
    block_on(async {
        let mut robot = Sim::new(false);
        let radius = Length::new::<meter>(0.45);
        let sweep = Angle::new::<radian>(-0.45);
        let end = ozton_math::path::clearance::forward_arc_end(robot.pose, radius, sweep);
        let mut runner = Runner::new(
            &mut robot,
            OPTIONS,
            Duration::from_millis(5),
            Duration::from_millis(100),
            Duration::from_secs(3),
        )
        .unwrap();
        runner
            .run_while(
                Command::DriveArc {
                    radius,
                    sweep,
                    maximum_speed: Velocity::new::<meter_per_second>(0.25),
                },
                Duration::from_secs(2),
                || true,
            )
            .await
            .unwrap();
        let robot = runner.robot_ref();
        assert!(robot.pose.position.distance(end.position) <= OPTIONS.position_tolerance.value);
        assert!((robot.pose.heading - end.heading).value.abs() <= OPTIONS.heading_tolerance.value);
        assert_eq!(robot.wheels, [0.0; 2]);
    });
}

#[test]
fn followers_converge_and_cancel_with_mechanisms_running() {
    block_on(async {
        let path = route();
        for (follower, reverse) in [
            (
                Follower::PurePursuit {
                    lookahead: Length::new::<meter>(0.14),
                },
                false,
            ),
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
                            trigger: Some(ozton_motion::commands::PathMarker::new(
                                Length::new::<meter>(0.12),
                                |robot: &mut Sim| robot.intake = true,
                            )),
                        },
                        Duration::from_secs(3),
                        || true,
                    )
                    .await
                    .unwrap();
                let robot = runner.robot();
                assert!(
                    robot.pose.position.distance(path.end().pose.position)
                        <= OPTIONS.position_tolerance.value
                );
                assert_eq!(robot.wheels, [0.0; 2]);
                assert!(robot.intake_moving_ticks > 0);
                assert!(robot.intake);

                let mut enabled_ticks = 0;
                let outcome = runner
                    .run_while(
                        Command::DriveDistance {
                            meters: Length::new::<meter>(0.4),
                            heading: Some(
                                path.end().pose.heading
                                    + Angle::new::<radian>(if reverse { PI } else { 0.0 }),
                            ),
                            maximum_speed: Velocity::new::<meter_per_second>(0.4),
                        },
                        Duration::from_secs(1),
                        || {
                            enabled_ticks += 1;
                            enabled_ticks < 5
                        },
                    )
                    .await;
                assert!(matches!(outcome, Err(CommandError::Cancelled)));
                assert!(runner.robot().stopped);
                assert!(!runner.robot().intake);
            }
            assert_eq!(robot.wheels, [0.0; 2]);
        }
    });
}

#[test]
fn ordered_markers_and_optional_alignment_keep_the_route_running() {
    use ozton_motion::commands::{Alignment, PathMarker, RangeFacing};

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
            PathMarker::new(Length::new::<meter>(0.10), |robot: &mut Sim| {
                robot.intake = true;
            }),
            PathMarker::new(Length::new::<meter>(0.40), |robot: &mut Sim| {
                assert!(robot.intake_moving_ticks > 0);
                robot.intake = false;
            }),
        ];
        runner
            .run_while(
                Command::FollowMarked {
                    path: &path,
                    follower: Follower::PurePursuit {
                        lookahead: Length::new::<meter>(0.14),
                    },
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
                        distance: Length::new::<meter>(0.12),
                        heading: Angle::new::<radian>(0.0),
                        maximum_speed: Velocity::new::<meter_per_second>(0.2),
                        facing: RangeFacing::Front,
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

#[test]
fn full_pose_point_motion_corrects_lateral_error_and_optional_dsr_can_fall_back() {
    block_on(async {
        for reverse in [false, true] {
            let mut robot = Sim::new(reverse);
            robot.pose.position.x = 0.04;
            let mut runner = Runner::new(
                &mut robot,
                OPTIONS,
                Duration::from_millis(5),
                Duration::from_millis(100),
                Duration::from_secs(5),
            )
            .unwrap();
            runner.robot().intake = true;
            assert!(
                !runner
                    .try_distance_reset_while(Duration::from_millis(30), || true)
                    .await
                    .unwrap()
            );
            assert!(runner.robot().intake && !runner.robot().stopped);
            assert!(runner.robot().cancelled_resets > 0);
            runner
                .run_while(
                    Command::DriveToPoint {
                        target: glam::DVec2::new(-0.10, 0.20),
                        reverse,
                        maximum_speed: Velocity::new::<meter_per_second>(0.5),
                    },
                    Duration::from_secs(4),
                    || true,
                )
                .await
                .unwrap();
            assert!(
                runner
                    .robot()
                    .pose
                    .position
                    .distance(DVec2::new(-0.10, 0.20))
                    <= OPTIONS.position_tolerance.value
            );
            assert_eq!(runner.robot().wheels, [0.0; 2]);
            assert!(runner.robot().intake_moving_ticks > 0);
        }
    });
}

impl ozton_motion::commands::MechanismRobot for Sim {
    type Target = (f64, f64);
    fn set_target(&mut self, _: Self::Target) -> Result<(), Self::Error> {
        self.score_goal = true;
        self.scoring_ticks = 0;
        Ok(())
    }
    fn at_target(&self) -> bool {
        self.score_goal && self.scoring_ticks >= 3
    }
    fn set_action(&mut self, enabled: bool) -> Result<(), Self::Error> {
        if enabled && self.scoring_ticks < 3 {
            self.ejected_before_ready = true;
        }
        self.intake = enabled;
        Ok(())
    }
}

#[test]
fn untrusted_absolute_motion_never_drives_and_explicit_start_allows_it() {
    block_on(async {
        let mut robot = Sim::new(false);
        robot.trusted = false;
        let mut runner = Runner::new(
            &mut robot,
            OPTIONS,
            Duration::from_millis(5),
            Duration::from_millis(100),
            Duration::from_secs(4),
        )
        .unwrap();
        let moved = runner
            .try_run_while(
                Command::DriveToPoint {
                    target: glam::DVec2::new(0.0, 0.1),
                    reverse: false,
                    maximum_speed: Velocity::new::<meter_per_second>(0.3),
                },
                Duration::from_secs(1),
                || true,
            )
            .await
            .unwrap();
        assert!(!moved);
        assert_eq!(runner.robot().travel, 0.0);
        runner.confirm_start(Pose::default()).unwrap();
        runner
            .run_while(
                Command::DriveToPoint {
                    target: glam::DVec2::new(0.0, 0.1),
                    reverse: false,
                    maximum_speed: Velocity::new::<meter_per_second>(0.3),
                },
                Duration::from_secs(3),
                || true,
            )
            .await
            .unwrap();
        assert!(runner.robot().travel > 0.07);
    });
}

#[test]
fn jam_returns_optional_failure_then_fallback_runs_under_same_deadline() {
    block_on(async {
        let mut robot = Sim::new(false);
        robot.jammed = true;
        let mut runner = Runner::new(
            &mut robot,
            OPTIONS,
            Duration::from_millis(5),
            Duration::from_millis(100),
            Duration::from_secs(4),
        )
        .unwrap();
        let completed = runner
            .try_run_while(
                Command::DriveDistance {
                    meters: Length::new::<meter>(0.2),
                    heading: None,
                    maximum_speed: Velocity::new::<meter_per_second>(0.3),
                },
                Duration::from_secs(2),
                || true,
            )
            .await
            .unwrap();
        assert!(!completed);
        assert_eq!(runner.robot().wheels, [0.0; 2]);
        runner.robot().intake = true;
        let completed = runner
            .try_run_while(
                Command::Wait(Duration::from_secs(1)),
                Duration::from_millis(30),
                || true,
            )
            .await
            .unwrap();
        assert!(!completed);
        assert!(!runner.robot().intake);
        runner
            .run_while(
                Command::Wait(Duration::from_millis(20)),
                Duration::from_millis(100),
                || true,
            )
            .await
            .unwrap();
    });
}

#[test]
fn transient_tracking_retries_and_scoring_waits_for_measured_arrival() {
    block_on(async {
        let mut robot = Sim::new(false);
        robot.transient_samples = 3;
        let mut runner = Runner::new(
            &mut robot,
            OPTIONS,
            Duration::from_millis(5),
            Duration::from_millis(100),
            Duration::from_secs(2),
        )
        .unwrap();
        runner
            .perform_while(
                (0.2, 1.0),
                Duration::from_millis(30),
                Duration::from_millis(400),
                || true,
            )
            .await
            .unwrap();
        assert!(!runner.robot().intake);
        assert!(!runner.robot().ejected_before_ready);
        assert!(runner.robot().scoring_ticks >= 3);
    });
}

#[test]
fn captured_loading_wait_services_mechanisms_and_obeys_original_route_deadline() {
    block_on(async {
        let mut robot = Sim::new(false);
        robot.intake = true;
        robot.score_goal = true;
        let mut runner = Runner::new(
            &mut robot,
            OPTIONS,
            Duration::from_millis(5),
            Duration::from_millis(100),
            Duration::from_millis(120),
        )
        .unwrap();
        let before = runner.remaining();
        let required_ticks = 4;
        runner
            .wait_until_while(
                |robot| robot.scoring_ticks >= required_ticks,
                Duration::from_millis(15),
                Duration::from_millis(100),
                || true,
            )
            .await
            .unwrap();
        assert!(runner.robot().scoring_ticks >= required_ticks);
        assert!(
            runner.robot().intake,
            "successful wait retains the intake goal"
        );
        assert_eq!(runner.robot().wheels, [0.0; 2]);
        assert!(runner.remaining() < before);
        let error = runner
            .wait_until_while(
                |_| false,
                Duration::from_millis(15),
                Duration::from_secs(1),
                || true,
            )
            .await
            .unwrap_err();
        assert!(matches!(error, CommandError::RouteTimeout));
        assert_eq!(runner.remaining(), Duration::ZERO);
        assert!(!runner.robot().intake);
        assert!(runner.robot().stopped);
    });
}

#[test]
fn contact_approach_brakes_without_changing_the_field_reference() {
    block_on(async {
        let mut robot = Sim::new(false);
        let creep_speed = 0.1;
        let contact_at = 0.02;
        let mut runner = Runner::new(
            &mut robot,
            OPTIONS,
            Duration::from_millis(5),
            Duration::from_millis(100),
            Duration::from_secs(2),
        )
        .unwrap();
        let mut touched = |robot: &Sim, sample: Feedback, _elapsed: Duration| {
            assert!(
                robot
                    .wheels
                    .iter()
                    .all(|speed| speed.abs() <= creep_speed + 1e-9)
            );
            sample.pose.position.y >= contact_at
        };
        runner
            .run_while(
                Command::ApproachContact {
                    heading: Angle::new::<radian>(0.0),
                    speed: Velocity::new::<meter_per_second>(creep_speed),
                    settle: Duration::from_millis(60),
                    touched: &mut touched,
                },
                Duration::from_secs(1),
                || true,
            )
            .await
            .unwrap();
        let robot = runner.robot();
        assert_eq!(robot.wheels, [0.0; 2]);
        assert!(robot.pose.position.y >= contact_at);
        assert_eq!(robot.travel, robot.pose.position.y);
    });
}
