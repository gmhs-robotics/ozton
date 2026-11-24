//! Closed-loop wheel-speed targets for differential motions.

use std::time::Duration;

use ozton_localization::Feedback;
use ozton_math::{
    desaturate,
    geometry::{DVec2, wheel_speeds, wrap_heading},
    uom::{
        ConstZero,
        si::{
            angular_velocity::radian_per_second,
            f64::{Angle, AngularVelocity, Length, Velocity},
            velocity::meter_per_second,
        },
    },
};

mod pid;
#[cfg(test)]
mod tests;
mod tuning;
pub use pid::MotionPid;
pub use tuning::DriveTuning;

#[derive(Debug, Clone, Copy)]
pub struct DriveOptions {
    pub track_width: Length,
    pub maximum_wheel_speed: Velocity,
    pub position_tolerance: Length,
    pub heading_tolerance: Angle,
    pub speed_tolerance: Velocity,
    pub angular_speed_tolerance: AngularVelocity,
    pub settle: Duration,
}

impl DriveOptions {
    /// Pair the drivetrain's calibrated track width with a route-selected wheel speed.
    pub fn for_differential(
        drive: &impl ozton_drivetrain::model::DriveEncoders,
        maximum_wheel_speed: Velocity,
    ) -> Option<Self> {
        Some(Self::differential(
            drive.track_width()?,
            maximum_wheel_speed,
        ))
    }

    /// Supply arrival tolerances, velocity tolerances, and required settle time.
    pub const fn with_settling(
        mut self,
        position: Length,
        heading: Angle,
        speed: Velocity,
        angular_speed: AngularVelocity,
        settle: Duration,
    ) -> Self {
        self.position_tolerance = position;
        self.heading_tolerance = heading;
        self.speed_tolerance = speed;
        self.angular_speed_tolerance = angular_speed;
        self.settle = settle;
        self
    }

    /// Settling defaults for a differential robot.
    pub const fn differential(track_width: Length, maximum_wheel_speed: Velocity) -> Self {
        Self {
            track_width,
            maximum_wheel_speed,
            position_tolerance: ozton_math::length!(0.025, m),
            heading_tolerance: ozton_math::angle!(0.06, rad),
            speed_tolerance: Velocity {
                value: 0.04,
                ..Velocity::ZERO
            },
            angular_speed_tolerance: AngularVelocity {
                value: 0.12,
                ..AngularVelocity::ZERO
            },
            settle: Duration::from_millis(120),
        }
    }

    pub fn valid(self) -> bool {
        [
            self.track_width.value,
            self.maximum_wheel_speed.value,
            self.position_tolerance.value,
            self.heading_tolerance.value,
            self.speed_tolerance.value,
            self.angular_speed_tolerance.value,
        ]
        .iter()
        .all(|value| value.is_finite() && *value > 0.0)
    }
}

/// Targets supported by the differential feedback controller.
#[derive(Clone, Copy, Debug)]
pub enum DifferentialTarget {
    Turn {
        heading: Angle,
    },
    TurnToPoint {
        target: DVec2,
    },
    DriveToPoint {
        target: DVec2,
        reverse: bool,
        maximum_speed: Velocity,
    },
    DriveDistance {
        meters: Length,
        heading: Option<Angle>,
        maximum_speed: Velocity,
    },
    DriveArc {
        radius: Length,
        sweep: Angle,
        maximum_speed: Velocity,
    },
}

/// The differential controller and initial measurements for a single target.
#[derive(Default)]
pub struct DifferentialFeedback {
    pub pid: MotionPid,
    start_travel: Option<f64>,
    start_heading: Option<Angle>,
    point_backing: Option<bool>,
    point_aligning: bool,
}

impl DifferentialFeedback {
    pub fn new(tuning: DriveTuning) -> Self {
        Self {
            pid: MotionPid::new(tuning),
            ..Self::default()
        }
    }

    /// Returns wheel speeds in metres per second and whether the latest sample
    /// meets all arrival tolerances. The caller applies the required settle time.
    pub fn step(
        &mut self,
        mut target: DifferentialTarget,
        feedback: Feedback,
        dt: Duration,
        options: DriveOptions,
    ) -> ([f64; 2], bool) {
        let slow = feedback
            .forward_velocity
            .value
            .hypot(feedback.lateral_velocity.value)
            <= options.speed_tolerance.value
            && feedback.angular_velocity.value.abs() <= options.angular_speed_tolerance.value;
        let error_at = |heading| wrap_heading(heading - feedback.pose.heading).value;
        let drive = |linear, angular| {
            desaturate(
                wheel_speeds(
                    Velocity::new::<meter_per_second>(linear),
                    AngularVelocity::new::<radian_per_second>(angular),
                    options.track_width,
                )
                .map(|speed| speed.value),
                options.maximum_wheel_speed.value,
            )
        };
        let forward_rate = feedback.forward_velocity.value;
        let yaw_rate = feedback.angular_velocity.value;
        let yaw_limit = 2.0 * options.maximum_wheel_speed.value / options.track_width.value;
        // Leave braking authority even inside the arrival band.
        // Conservative default deceleration in SI units; characterize per robot.
        let stopping_limit = |distance: f64, maximum: f64| {
            (2.0 * 2.0 * distance.abs()).sqrt().max(0.12).min(maximum)
        };
        let angular = |pid: &mut MotionPid, error: f64| {
            let limit = (2.0 * 8.0 * error.abs()).sqrt().max(0.5).min(yaw_limit);
            pid.yaw(error, yaw_rate, 0.0, limit, dt)
        };
        match &mut target {
            DifferentialTarget::Turn { heading } => {
                let error = error_at(*heading);
                let ready = error.abs() <= options.heading_tolerance.value;
                (
                    drive(
                        0.0,
                        if ready && slow {
                            0.0
                        } else {
                            angular(&mut self.pid, error)
                        },
                    ),
                    ready && slow,
                )
            }
            DifferentialTarget::TurnToPoint { target } => {
                let delta = *target - feedback.pose.position;
                let error = if delta.length() <= options.position_tolerance.value {
                    0.0
                } else {
                    error_at(ozton_math::geometry::bearing(delta))
                };
                let ready = error.abs() <= options.heading_tolerance.value;
                (
                    drive(
                        0.0,
                        if ready && slow {
                            0.0
                        } else {
                            angular(&mut self.pid, error)
                        },
                    ),
                    ready && slow,
                )
            }
            DifferentialTarget::DriveToPoint {
                target,
                reverse,
                maximum_speed,
            } => {
                let delta = *target - feedback.pose.position;
                let distance = delta.length();
                let reached = distance <= options.position_tolerance.value;
                if reached {
                    if slow {
                        return ([0.0; 2], true);
                    }
                    return (
                        drive(
                            self.pid.forward(0.0, forward_rate, maximum_speed.value, dt),
                            angular(&mut self.pid, 0.0),
                        ),
                        false,
                    );
                }
                let bearing = ozton_math::geometry::bearing(delta);
                let opposite = ozton_math::angle!(180 deg);
                let backing = self.point_backing.get_or_insert(*reverse);
                let mut heading = error_at(if *backing { bearing + opposite } else { bearing });
                // Latch travel direction. Near the goal, allow a deliberate reversal
                // only after nearly stopping, with a wide hysteresis band.
                if distance <= 4.0 * options.position_tolerance.value
                    && heading.abs() > ozton_math::angle!(125 deg).value
                    && forward_rate.abs() <= 2.0 * options.speed_tolerance.value
                    && yaw_rate.abs() <= 0.5
                {
                    *backing = !*backing;
                    heading = error_at(if *backing { bearing + opposite } else { bearing });
                    self.pid.reset();
                    self.point_aligning = false;
                }
                if self.point_aligning {
                    self.point_aligning = heading.abs() > ozton_math::angle!(12 deg).value;
                } else {
                    self.point_aligning = heading.abs() > ozton_math::angle!(35 deg).value;
                }
                let sign = if *backing { -1.0 } else { 1.0 };
                let linear = if self.point_aligning {
                    // Damp residual translation instead of releasing the motors.
                    self.pid.forward(0.0, forward_rate, maximum_speed.value, dt)
                } else {
                    let alignment = heading.cos().max(0.0);
                    self.pid.forward(
                        sign * distance * alignment,
                        forward_rate,
                        stopping_limit(distance, maximum_speed.value),
                        dt,
                    )
                };
                let turn = angular(&mut self.pid, heading);
                (drive(linear, turn), reached && slow)
            }
            DifferentialTarget::DriveDistance {
                meters,
                heading,
                maximum_speed,
            } => {
                let start = *self
                    .start_travel
                    .get_or_insert(feedback.forward_travel.value);
                let error = meters.value - (feedback.forward_travel.value - start);
                let heading =
                    error_at(heading.unwrap_or_else(|| {
                        *self.start_heading.get_or_insert(feedback.pose.heading)
                    }));
                let reached = error.abs() <= options.position_tolerance.value;
                let facing = heading.abs() <= options.heading_tolerance.value;
                let limit = stopping_limit(error, maximum_speed.value);
                let translation_error = if reached || heading.abs() > 0.35 { 0.0 } else { error };
                let speed = if reached && facing && slow {
                    0.0
                } else {
                    self.pid.forward(translation_error, forward_rate, limit, dt)
                };
                let turning = if facing && reached && slow {
                    0.0
                } else {
                    angular(&mut self.pid, heading)
                };
                (drive(speed, turning), reached && facing && slow)
            }
            DifferentialTarget::DriveArc {
                radius,
                sweep,
                maximum_speed,
            } => {
                let start = *self
                    .start_travel
                    .get_or_insert(feedback.forward_travel.value);
                let arc_length = radius.value * sweep.value.abs();
                let travel = feedback.forward_travel.value - start;
                let remaining = arc_length - travel;
                let reached = remaining.abs() <= options.position_tolerance.value;
                let start_heading = *self.start_heading.get_or_insert(feedback.pose.heading);
                let curvature = sweep.value.signum() / radius.value;
                let heading = error_at(
                    start_heading
                        + if reached {
                            *sweep
                        } else {
                            ozton_math::angle!(travel.clamp(0.0, arc_length) * curvature, rad)
                        },
                );
                let facing = heading.abs() <= options.heading_tolerance.value;
                let wheel_limit = options.maximum_wheel_speed.value
                    / (1.0 + options.track_width.value * curvature.abs() * 0.5);
                let limit = stopping_limit(remaining, maximum_speed.value.min(wheel_limit));
                let speed = if reached && facing && slow {
                    0.0
                } else {
                    self.pid.forward(if reached { 0.0 } else { remaining }, forward_rate, limit, dt)
                };
                let feedforward = speed * curvature;
                let turning = if reached && facing && slow {
                    0.0
                } else {
                    let correction_limit = if reached { 0.5 } else { 0.35 };
                    self.pid
                        .yaw(heading, yaw_rate, feedforward, correction_limit, dt)
                };
                (drive(speed, turning), reached && facing && slow)
            }
        }
    }
}
