use std::{
    cell::RefCell,
    convert::Infallible,
    rc::Rc,
    time::{Duration, Instant},
};

use glam::DVec2 as Vec2;
use ozton_control::{
    Tolerances,
    loops::{AngularPid, Pid},
};
use ozton_drivetrain::{
    Drivetrain,
    model::{Arcade, DrivetrainModel},
};
use ozton_motion::{Basic, Seeking};
use ozton_tracking::{
    Tracking, TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
};
use vex_sdk_mock as _;
use vexide::{math::Angle, runtime::block_on};

#[derive(Debug)]
struct SimState {
    position: Vec2,
    heading_radians: f64,
    forward_travel: f64,
    linear_velocity: f64,
    angular_velocity: f64,
    throttle_command: f64,
    turn_command: f64,
    max_linear_velocity: f64,
    max_angular_velocity: f64,
    last_update: Instant,
}

impl SimState {
    fn new(max_linear_velocity: f64, max_angular_velocity: f64) -> Self {
        Self {
            position: Vec2::ZERO,
            heading_radians: 0.0,
            forward_travel: 0.0,
            linear_velocity: 0.0,
            angular_velocity: 0.0,
            throttle_command: 0.0,
            turn_command: 0.0,
            max_linear_velocity,
            max_angular_velocity,
            last_update: Instant::now(),
        }
    }

    fn sync(&mut self) {
        let now = Instant::now();
        let dt = now
            .saturating_duration_since(self.last_update)
            .as_secs_f64();
        self.last_update = now;

        self.linear_velocity = self.throttle_command * self.max_linear_velocity;
        self.angular_velocity = -self.turn_command * self.max_angular_velocity;

        if dt <= f64::EPSILON {
            return;
        }

        self.heading_radians = wrap_radians(self.heading_radians + self.angular_velocity * dt);
        let displacement = Vec2::from_angle(self.heading_radians) * (self.linear_velocity * dt);
        self.position += displacement;
        self.forward_travel += self.linear_velocity * dt;
    }
}

#[derive(Clone, Debug)]
struct MockArcadeModel {
    state: Rc<RefCell<SimState>>,
}

impl MockArcadeModel {
    fn new(state: Rc<RefCell<SimState>>) -> Self {
        Self { state }
    }
}

impl DrivetrainModel for MockArcadeModel {
    type Error = Infallible;
}

impl Arcade for MockArcadeModel {
    fn drive_arcade(&mut self, throttle: f64, steer: f64) -> Result<(), Self::Error> {
        let mut state = self.state.borrow_mut();
        state.sync();
        state.throttle_command = throttle.clamp(-1.0, 1.0);
        state.turn_command = steer.clamp(-1.0, 1.0);
        Ok(())
    }
}

#[derive(Clone, Debug)]
struct MockTracking {
    state: Rc<RefCell<SimState>>,
}

impl MockTracking {
    fn new(state: Rc<RefCell<SimState>>) -> Self {
        Self { state }
    }

    fn snapshot(&self) -> SimSnapshot {
        let mut state = self.state.borrow_mut();
        state.sync();
        SimSnapshot {
            position: state.position,
            heading_radians: state.heading_radians,
            forward_travel: state.forward_travel,
            linear_velocity: state.linear_velocity,
            angular_velocity: state.angular_velocity,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct SimSnapshot {
    position: Vec2,
    heading_radians: f64,
    forward_travel: f64,
    linear_velocity: f64,
    angular_velocity: f64,
}

impl Tracking for MockTracking {}

impl TracksPosition for MockTracking {
    fn position(&self) -> Vec2 {
        self.snapshot().position
    }
}

impl TracksHeading for MockTracking {
    fn heading(&self) -> Angle {
        Angle::from_radians(self.snapshot().heading_radians).wrapped_full()
    }
}

impl TracksVelocity for MockTracking {
    fn linear_velocity(&self) -> f64 {
        self.snapshot().linear_velocity
    }

    fn angular_velocity(&self) -> f64 {
        self.snapshot().angular_velocity
    }
}

impl TracksForwardTravel for MockTracking {
    fn forward_travel(&self) -> f64 {
        self.snapshot().forward_travel
    }
}

fn new_drivetrain() -> Drivetrain<MockArcadeModel, MockTracking> {
    let state = Rc::new(RefCell::new(SimState::new(3.5, 5.0)));
    Drivetrain::new(
        MockArcadeModel::new(state.clone()),
        MockTracking::new(state),
    )
}

fn linear_tolerances() -> Tolerances {
    let mut tolerances = Tolerances::new();
    tolerances.error_tolerance = Some(0.04);
    tolerances.velocity_tolerance = Some(0.05);
    tolerances.duration = Some(Duration::from_millis(30));
    tolerances
}

fn angular_tolerances() -> Tolerances {
    let mut tolerances = Tolerances::new();
    tolerances.error_tolerance = Some(0.05);
    tolerances.velocity_tolerance = Some(0.08);
    tolerances.duration = Some(Duration::from_millis(30));
    tolerances
}

fn point_tolerances() -> Tolerances {
    let mut tolerances = Tolerances::new();
    tolerances.error_tolerance = Some(0.05);
    tolerances.velocity_tolerance = Some(0.06);
    tolerances.duration = Some(Duration::from_millis(30));
    tolerances
}

fn basic_motion() -> Basic<Pid, AngularPid> {
    let mut linear = Pid::new(1.1, 0.0, 0.08, None);
    linear.set_output_limit(Some(1.0));

    let mut angular = AngularPid::new(2.6, 0.0, 0.12, None);
    angular.set_output_limit(Some(1.0));

    Basic {
        linear_controller: linear,
        angular_controller: angular,
        linear_tolerances: linear_tolerances(),
        angular_tolerances: angular_tolerances(),
        timeout: Some(Duration::from_secs(2)),
    }
}

fn seeking_motion() -> Seeking<Pid, Pid> {
    let mut linear = Pid::new(1.2, 0.0, 0.06, None);
    linear.set_output_limit(Some(1.0));

    let mut lateral = Pid::new(3.2, 0.0, 0.08, None);
    lateral.set_output_limit(Some(1.0));

    Seeking {
        linear_controller: linear,
        lateral_controller: lateral,
        tolerances: point_tolerances(),
        timeout: Some(Duration::from_secs(3)),
    }
}

#[test]
fn basic_motion_reaches_distance_and_heading_goals() {
    let mut drivetrain = new_drivetrain();
    let mut motion = basic_motion();

    block_on(async {
        motion.drive_distance(&mut drivetrain, 1.2).await;
        motion
            .turn_to_point(&mut drivetrain, Vec2::new(1.2, 1.2))
            .await;
    });

    let final_position = drivetrain.tracking.position();
    let final_heading = drivetrain.tracking.heading().as_radians();
    let final_velocity = drivetrain.tracking.linear_velocity();

    assert!(
        (final_position.x - 1.2).abs() < 0.08,
        "expected x close to 1.2, got {:?}",
        final_position
    );
    assert!(
        final_position.y.abs() < 0.08,
        "expected y near 0, got {:?}",
        final_position
    );
    assert!(
        wrap_radians(final_heading - std::f64::consts::FRAC_PI_2).abs() < 0.08,
        "expected heading near 90deg, got {final_heading}"
    );
    assert!(
        final_velocity.abs() < 0.08,
        "expected robot settled, got {final_velocity}"
    );
}

#[test]
fn seeking_motion_reaches_multiple_waypoints() {
    let mut drivetrain = new_drivetrain();
    let mut motion = seeking_motion();

    block_on(async {
        motion
            .move_to_point(&mut drivetrain, Vec2::new(1.0, 0.0))
            .await;
        motion
            .move_to_point(&mut drivetrain, Vec2::new(1.0, 1.0))
            .await;
        motion
            .move_to_point(&mut drivetrain, Vec2::new(0.4, 1.4))
            .await;
    });

    let final_position = drivetrain.tracking.position();
    let final_heading = drivetrain.tracking.heading().as_radians();
    let final_velocity = drivetrain.tracking.linear_velocity();

    assert!(
        final_position.distance(Vec2::new(0.4, 1.4)) < 0.09,
        "expected final position near target, got {:?}",
        final_position
    );
    assert!(
        final_velocity.abs() < 0.08,
        "expected robot settled at final point, got {final_velocity}"
    );
    assert!(final_heading.is_finite(), "heading should remain finite");
}

fn wrap_radians(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}
