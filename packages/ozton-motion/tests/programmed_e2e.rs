use std::{
    f64::consts::PI,
    time::{Duration, Instant},
};

use glam::DVec2;
use ozton_math::{
    geometry::{BodyDelta, Pose},
    path::{Limits, Path, Waypoint, follower::Follower},
    si,
    uom::si::f64::Velocity,
};
use ozton_motion::auton::{AutonError, AutonRobot, Command, DriveOptions, Feedback, Runner};
use vex_sdk_mock as _;
use vexide::runtime::block_on;

const OPTIONS: DriveOptions = DriveOptions {
    track_width: si(0.3),
    maximum_wheel_speed: si(1.0),
    position_tolerance: si(0.018),
    heading_tolerance: si(0.09),
    speed_tolerance: si(0.03),
    angular_speed_tolerance: si(0.08),
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
                ozton_math::si(0.0),
                ozton_math::si(0.0),
                ozton_math::si(if reverse { PI } else { 0.0 }),
            ),
            travel: 0.0,
            last: Instant::now(),
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

impl AutonRobot for Sim {
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
            heading: angular_speed * dt,
        });
        Ok(Some(Feedback {
            pose: self.pose,
            forward_travel: si(self.travel),
            forward_velocity: si(speed),
            lateral_velocity: si(0.0),
            angular_velocity: si(angular_speed),
            ranges: [None; 8],
            reset_accepted: false,
        }))
    }

    fn update_custom(&mut self, _: Duration) -> Result<(), Self::Error> {
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

fn route() -> Path {
    let points: Vec<_> = (0..=12)
        .map(|index| {
            let x = f64::from(index) * 0.05;
            Waypoint::new(ozton_math::si(-0.22 * x.powi(2)), ozton_math::si(x), 0.8)
        })
        .collect();
    Path::new(
        &points,
        Limits {
            track_width: ozton_math::si(OPTIONS.track_width.value),
            linear_speed: ozton_math::si(0.8),
            wheel_speed: ozton_math::si(1.0),
            acceleration: ozton_math::si(2.0),
            deceleration: ozton_math::si(2.0),
            lateral_acceleration: ozton_math::si(1.0),
        },
    )
    .unwrap()
}

#[test]
fn followers_converge_and_cancel_with_mechanisms_running() {
    block_on(async {
        let path = route();
        for (follower, reverse) in [
            (
                Follower::PurePursuit {
                    lookahead: si(0.14),
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
                            trigger: Some((si(0.12), |robot: &mut Sim| robot.intake = true)),
                        },
                        Duration::from_secs(3),
                        || true,
                    )
                    .await
                    .unwrap();
                let robot = runner.robot();
                assert!(
                    robot.pose.position.distance(path.end().position)
                        <= OPTIONS.position_tolerance.value
                );
                assert_eq!(robot.wheels, [0.0; 2]);
                assert!(robot.intake_moving_ticks > 0);
                assert!(robot.intake);

                let mut enabled_ticks = 0;
                let outcome = runner
                    .run_while(
                        Command::DriveDistance {
                            meters: si(0.4),
                            heading: si(path.end().heading + if reverse { PI } else { 0.0 }),
                            maximum_speed: si(0.4),
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
                si(0.10),
                (|robot: &mut Sim| robot.intake = true) as fn(&mut Sim),
            ),
            (si(0.40), |robot: &mut Sim| {
                assert!(robot.intake_moving_ticks > 0);
                robot.intake = false;
            }),
        ];
        runner
            .run_while(
                Command::FollowMarked {
                    path: &path,
                    follower: Follower::PurePursuit {
                        lookahead: si(0.14),
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
                        distance: si(0.12),
                        heading: si(0.0),
                        maximum_speed: si(0.2),
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
                        target: [si(-0.10), si(0.20)],
                        reverse,
                        maximum_speed: si(0.5),
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

impl ozton_motion::auton::MechanismRobot for Sim {
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
                    target: [si(0.0), si(0.1)],
                    reverse: false,
                    maximum_speed: si(0.3),
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
                    target: [si(0.0), si(0.1)],
                    reverse: false,
                    maximum_speed: si(0.3),
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
                    meters: si(0.2),
                    heading: si(0.0),
                    maximum_speed: si(0.3),
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
fn a_failed_retry_stop_cancels_only_the_current_runner() {
    block_on(async {
        let mut robot = Sim::new(false);
        robot.transient_samples = 1;
        robot.stop_failures = 1;
        let wait = || Command::Wait(Duration::from_millis(10));
        let mut runner = Runner::new(
            &mut robot,
            OPTIONS,
            Duration::from_millis(5),
            Duration::from_millis(100),
            Duration::from_secs(1),
        )
        .unwrap();
        assert!(matches!(
            runner
                .try_run_while(wait(), Duration::from_millis(100), || true)
                .await,
            Err(AutonError::StopFailed(SimError::Transient)),
        ));
        assert!(runner.robot().stopped);
        assert!(matches!(
            runner
                .run_while(wait(), Duration::from_millis(100), || true)
                .await,
            Err(AutonError::Cancelled),
        ));
        drop(runner);

        // A later competition period starts with a fresh runner.
        Runner::new(
            &mut robot,
            OPTIONS,
            Duration::from_millis(5),
            Duration::from_millis(100),
            Duration::from_secs(1),
        )
        .unwrap()
        .run_while(wait(), Duration::from_millis(100), || true)
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
        assert!(matches!(error, AutonError::RouteTimeout));
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
                    heading: si(0.0),
                    speed: si(creep_speed),
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
