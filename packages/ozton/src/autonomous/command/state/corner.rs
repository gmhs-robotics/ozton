//! Deterministic Match geometry and fixed-step velocity response.

use ozton_math::{
    geometry::{BodyDelta, bearing},
    path::clearance::{Footprint, RectangularClearance, approach_heading},
    uom::ConstZero,
};

use super::*;

const TICK: Duration = Duration::from_millis(10);
const TICKS_ALLOWED: u32 = 800;

#[derive(Default)]
struct DelayedYaw {
    requests: [f64; 12],
    next: usize,
}

impl DelayedYaw {
    fn update(&mut self, yaw: f64, request: f64) -> f64 {
        let delayed = std::mem::replace(&mut self.requests[self.next], request);
        self.next = (self.next + 1) % self.requests.len();
        yaw + (delayed * 1.6 - yaw) * 0.095
    }
}

#[test]
fn match_turn_settles_with_delayed_motor_response() {
    // The logged turn began at -0.347 rad and targeted 0.836 rad. The
    // measured yaw lagged the wheel request and reached about 1.5x its rate.
    let options = DriveOptions::differential(
        ozton_math::length!(0.3292677692 m),
        Velocity::new::<meter_per_second>(0.50),
    )
    .with_settling(
        ozton_math::length!(0.25 in),
        ozton_math::angle!(1.0 deg),
        Velocity::new::<meter_per_second>(0.04),
        AngularVelocity::new::<radian_per_second>(0.07),
        Duration::from_millis(90),
    );
    let heading = ozton_math::angle!(0.836 rad);
    let mut command = Command::Turn { heading };
    let mut state = State::default();
    let mut feedback = Feedback {
        time: Duration::ZERO,
        pose: Pose::new(DVec2::ZERO, ozton_math::angle!(-0.347 rad)),
        angular_velocity: AngularVelocity::ZERO,
        forward_travel: Length::ZERO,
        forward_velocity: Velocity::ZERO,
        lateral_velocity: Velocity::ZERO,
        ranges: [None; MAX_RANGES],
        reset_accepted: false,
    };
    let mut delayed = DelayedYaw::default();
    // Leave time for the Match route to translate and settle its final heading.
    for _ in 0..240 {
        feedback.time += TICK;
        let step = state.step(
            &mut command,
            &mut (),
            feedback,
            feedback.time,
            TICK,
            options,
        );
        if step.complete {
            assert!(
                (feedback.pose.heading - heading).value.abs() <= options.heading_tolerance.value
            );
            return;
        }
        let request = (step.wheels[1] - step.wheels[0]) / options.track_width.value;
        feedback.angular_velocity.value = delayed.update(feedback.angular_velocity.value, request);
        feedback.pose.heading.value += feedback.angular_velocity.value * TICK.as_secs_f64();
    }
    panic!("Match turn did not settle under delayed wheel response");
}

struct Plant {
    sample: Feedback,
    wheels: [f64; 2],
    ticks: u32,
    footprint: Footprint,
    walls: DVec2,
}

impl Plant {
    fn run(&mut self, mut command: Command<'_, ()>, options: DriveOptions) {
        let mut state = State::default();
        let mut delayed_yaw = DelayedYaw::default();
        let started = self.ticks;
        loop {
            self.ticks += 1;
            assert!(
                self.ticks < TICKS_ALLOWED,
                "stalled in {}",
                command.status()
            );
            let elapsed = TICK * (self.ticks - started);
            let step = state.step(&mut command, &mut (), self.sample, elapsed, TICK, options);
            if step.complete {
                return;
            }
            // Bounded lag approximates the wheel response without replaying
            // commands from the old controller into the new one.
            for (wheel, target) in self.wheels.iter_mut().zip(step.wheels) {
                *wheel += (target - *wheel) * 0.25;
            }
            let [left, right] = self.wheels;
            let speed = (left + right) * 0.5;
            let yaw = if matches!(command, Command::Turn { .. }) {
                let request = (step.wheels[1] - step.wheels[0]) / options.track_width.value;
                delayed_yaw.update(self.sample.angular_velocity.value, request)
            } else {
                (right - left) / options.track_width.value
            };
            let travel = speed * TICK.as_secs_f64();
            self.sample.pose = self.sample.pose.integrate(BodyDelta {
                translation: DVec2::new(0.0, travel),
                heading: ozton_math::angle!(yaw * TICK.as_secs_f64(), rad),
            });
            self.sample.forward_travel += ozton_math::length!(travel, m);
            self.sample.forward_velocity = Velocity::new::<meter_per_second>(speed);
            self.sample.angular_velocity = AngularVelocity::new::<radian_per_second>(yaw);
            self.sample.time += TICK;
            let (min, max) = self.footprint.bounds(self.sample.pose);
            assert!(min.cmpge(-self.walls).all() && max.cmple(self.walls).all());
        }
    }
}

#[test]
fn match_corner_reaches_center_with_position_and_heading_settled() {
    let inches = |value| ozton_math::length!(value, in);
    let footprint = Footprint::from_extents(
        inches(7.55),
        inches(7.55),
        inches(7.761804),
        inches(7.238196),
    )
    .unwrap();
    let walls = DVec2::new(inches(19.0).value, inches(21.25).value);
    let field = RectangularClearance::new(footprint, walls, inches(0.5)).unwrap();
    let (rear_left, _) = footprint.bounds(Pose::default());
    let start = Pose::new(
        -walls - rear_left + DVec2::splat(inches(1.0).value),
        Angle::ZERO,
    );
    let target = Pose::default();
    let max_speed = Velocity::new::<meter_per_second>(0.50);
    let departure_speed = Velocity::new::<meter_per_second>(0.30);
    let mut options = DriveOptions::differential(ozton_math::length!(0.3292677692 m), max_speed);
    options.position_tolerance = inches(0.75);
    options.heading_tolerance = ozton_math::angle!(2.0 deg);
    let mut plant = Plant {
        sample: Feedback {
            time: Duration::ZERO,
            pose: start,
            forward_travel: Length::ZERO,
            forward_velocity: Velocity::ZERO,
            lateral_velocity: Velocity::ZERO,
            angular_velocity: AngularVelocity::ZERO,
            ranges: [None; MAX_RANGES],
            reset_accepted: false,
        },
        wheels: [0.0; 2],
        ticks: 0,
        footprint,
        walls,
    };

    let reserve = options.position_tolerance + inches(0.25);
    let plan = field
        .point_departure(start, target, options.track_width / 2.0, reserve)
        .unwrap();
    let arc = plan.arc.unwrap();
    plant.run(
        Command::DriveArc {
            radius: arc.radius,
            sweep: arc.sweep,
            maximum_speed: departure_speed,
        },
        options,
    );
    let pose = plant.sample.pose;
    if !field.can_turn(pose) {
        let meters = field.forward_to_turn(pose, reserve).unwrap();
        plant.run(
            Command::DriveDistance {
                meters,
                heading: Some(pose.heading),
                maximum_speed: departure_speed,
            },
            options,
        );
    }
    assert!(field.can_turn(plant.sample.pose));
    let displacement = target.position - plant.sample.pose.position;
    let (reverse, _) = approach_heading(plant.sample.pose.heading, target.heading, displacement);
    let heading =
        bearing(displacement) + ozton_math::angle!(if reverse { 180.0 } else { 0.0 }, deg);
    plant.run(Command::Turn { heading }, options);
    plant.run(
        Command::DriveToPoint { target: target.position, maximum_speed: max_speed, reverse },
        options,
    );
    plant.run(
        Command::Turn {
            heading: target.heading,
        },
        options,
    );
    assert!(
        plant.sample.pose.position.distance(target.position) <= options.position_tolerance.value
    );
    assert!(
        wrap_heading(plant.sample.pose.heading - target.heading)
            .value
            .abs()
            <= options.heading_tolerance.value
    );
}
