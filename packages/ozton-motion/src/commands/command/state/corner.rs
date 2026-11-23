//! Deterministic Match geometry and fixed-step velocity response.

use ozton_math::{
    geometry::{BodyDelta, bearing},
    path::clearance::{Footprint, RectangularClearance, approach_heading},
    uom::si::{angle::degree, length::inch},
};

use super::*;

const TICK: Duration = Duration::from_millis(10);
const TICKS_ALLOWED: u32 = 800;

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
            let yaw = (right - left) / options.track_width.value;
            let travel = speed * TICK.as_secs_f64();
            self.sample.pose = self.sample.pose.integrate(BodyDelta {
                translation: DVec2::new(0.0, travel),
                heading: Angle::new::<radian>(yaw * TICK.as_secs_f64()),
            });
            self.sample.forward_travel += Length::new::<meter>(travel);
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
    let inches = Length::new::<inch>;
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
    let options = DriveOptions::differential(Length::new::<meter>(0.3292677692), max_speed)
        .with_settling(
            inches(0.25),
            Angle::new::<degree>(1.0),
            Velocity::new::<meter_per_second>(0.04),
            AngularVelocity::new::<radian_per_second>(0.07),
            Duration::from_millis(90),
        );
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

    let plan = field
        .point_departure(
            start,
            target,
            options.track_width / 2.0,
            options.position_tolerance,
        )
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
        let meters = field
            .forward_to_turn(pose, options.position_tolerance)
            .unwrap();
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
    let heading = bearing(displacement) + Angle::new::<degree>(if reverse { 180.0 } else { 0.0 });
    plant.run(Command::Turn { heading }, options);
    plant.run(
        Command::drive_to_point(target.position, max_speed, reverse),
        options,
    );
    plant.run(
        Command::Turn {
            heading: target.heading,
        },
        options,
    );
    assert!(plant.sample.pose.position.distance(target.position) <= inches(0.5).value);
    assert!(
        wrap_heading(plant.sample.pose.heading - target.heading)
            .value
            .abs()
            <= options.heading_tolerance.value
    );
}
