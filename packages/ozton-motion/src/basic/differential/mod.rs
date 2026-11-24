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
mod terminal;
#[cfg(test)]
mod tests;
mod tuning;
pub use pid::MotionPid;
pub use terminal::TerminalPose;
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
            position_tolerance: ozton_math::qty!(0.025 m),
            heading_tolerance: ozton_math::qty!(0.06 rad),
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
    config: crate::MotionConfig,
    last_wheels: [f64; 2],
}

impl DifferentialFeedback {
    pub const fn new(tuning: DriveTuning) -> Self {
        Self {
            pid: MotionPid::new(tuning),
            start_travel: None,
            start_heading: None,
            point_backing: None,
            point_aligning: false,
            config: crate::MotionConfig::DEFAULT,
            last_wheels: [0.0; 2],
        }
    }

    pub const fn with_config(mut self, config: crate::MotionConfig) -> Self {
        self.pid = MotionPid::new(config.tuning)
            .with_rate_filters(config.linear_rate_filter, config.angular_rate_filter);
        self.config = config;
        self
    }

    /// Reset command state when feedback is interrupted.
    pub fn reset(&mut self) {
        self.pid.reset();
        self.last_wheels = [0.0; 2];
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
        let linear_rate_squared = feedback.forward_velocity.value * feedback.forward_velocity.value
            + feedback.lateral_velocity.value * feedback.lateral_velocity.value;
        let slow = linear_rate_squared <= options.speed_tolerance.value.powi(2)
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
        let config = self.config;
        let yaw_limit = (2.0 * options.maximum_wheel_speed.value / options.track_width.value)
            .min(config.angular_speed.value);
        // Leave braking authority even inside the arrival band.
        // Conservative default deceleration in SI units; characterize per robot.
        let stopping_limit = |distance: f64, maximum: f64| {
            (2.0 * config.deceleration.value * distance.abs())
                .sqrt()
                .max(options.speed_tolerance.value * 3.0)
                .min(maximum)
        };
        let reaction = config.maximum_gap.as_secs_f64() + config.brake_latency.as_secs_f64();
        let forward = |pid: &mut MotionPid, error: f64, limit: f64| {
            let output = pid.forward(error, forward_rate, limit, dt);
            compensate_breakaway(
                output,
                error,
                forward_rate,
                options.position_tolerance.value,
                options.speed_tolerance.value,
                config.minimum_linear_speed.value,
                limit.min(config.tuning.linear.output_limit().unwrap_or(f64::INFINITY)),
                config
                    .forward_braking
                    .unwrap_or(config.deceleration)
                    .value
                    .min(config.deceleration.value),
                reaction,
            )
        };
        let angular = |pid: &mut MotionPid, error: f64| {
            let limit = (2.0 * config.angular_deceleration.value * error.abs())
                .sqrt()
                .max(options.angular_speed_tolerance.value * 4.0)
                .min(yaw_limit);
            let output = pid.yaw(error, yaw_rate, 0.0, limit, dt);
            compensate_breakaway(
                output,
                error,
                yaw_rate,
                options.heading_tolerance.value,
                options.angular_speed_tolerance.value,
                config.minimum_angular_speed.value,
                limit.min(
                    config
                        .tuning
                        .angular
                        .output_limit()
                        .unwrap_or(f64::INFINITY),
                ),
                config
                    .yaw_braking
                    .unwrap_or(config.angular_deceleration)
                    .value
                    .min(config.angular_deceleration.value),
                reaction,
            )
        };
        let (wheels, ready) = match &mut target {
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
                let error = if delta.length_squared() <= options.position_tolerance.value.powi(2) {
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
                    let wheels = if slow {
                        [0.0; 2]
                    } else {
                        drive(
                            self.pid.forward(0.0, forward_rate, maximum_speed.value, dt),
                            angular(&mut self.pid, 0.0),
                        )
                    };
                    return self.limit_step(wheels, slow, dt, options);
                }
                let bearing = ozton_math::geometry::bearing(delta);
                let opposite = ozton_math::qty!(180 deg);
                let backing = self.point_backing.get_or_insert(*reverse);
                let mut heading = error_at(if *backing {
                    bearing + opposite
                } else {
                    bearing
                });
                // Latch travel direction. Near the goal, allow a deliberate reversal
                // only after nearly stopping, with a wide hysteresis band.
                if distance <= config.reversal_distance_factor * options.position_tolerance.value
                    && heading.abs() > config.reversal_heading.value
                    && forward_rate.abs() <= 2.0 * options.speed_tolerance.value
                    && yaw_rate.abs() <= config.reversal_yaw_rate.value
                {
                    *backing = !*backing;
                    heading = error_at(if *backing {
                        bearing + opposite
                    } else {
                        bearing
                    });
                    self.pid.reset();
                    self.point_aligning = false;
                }
                if self.point_aligning {
                    self.point_aligning = heading.abs() > config.realign_exit.value;
                } else {
                    self.point_aligning = heading.abs() > config.realign_enter.value;
                }
                let sign = if *backing { -1.0 } else { 1.0 };
                let linear = if self.point_aligning {
                    // Damp residual translation instead of releasing the motors.
                    self.pid.forward(0.0, forward_rate, maximum_speed.value, dt)
                } else {
                    let alignment = heading.cos().max(0.0);
                    forward(
                        &mut self.pid,
                        sign * distance * alignment,
                        stopping_limit(distance, maximum_speed.value),
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
                let translation_error = if reached || heading.abs() > config.realign_enter.value {
                    0.0
                } else {
                    error
                };
                let speed = if reached && facing && slow {
                    0.0
                } else {
                    forward(&mut self.pid, translation_error, limit)
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
                            ozton_math::qty!((travel.clamp(0.0, arc_length) * curvature) rad)
                        },
                );
                let facing = heading.abs() <= options.heading_tolerance.value;
                let wheel_limit = (options.maximum_wheel_speed.value
                    / (1.0 + options.track_width.value * curvature.abs() * 0.5))
                    .min((config.lateral_acceleration.value / curvature.abs()).sqrt());
                let limit = stopping_limit(remaining, maximum_speed.value.min(wheel_limit));
                let speed = if reached && facing && slow {
                    0.0
                } else {
                    forward(&mut self.pid, if reached { 0.0 } else { remaining }, limit)
                };
                let feedforward = speed * curvature;
                let turning = if reached && facing && slow {
                    0.0
                } else {
                    let correction_limit = if reached {
                        config.terminal_arc_correction_limit.value
                    } else {
                        config.arc_correction_limit.value
                    };
                    if reached && speed == 0.0 {
                        let output = self.pid.yaw(heading, yaw_rate, 0.0, correction_limit, dt);
                        compensate_breakaway(
                            output,
                            heading,
                            yaw_rate,
                            options.heading_tolerance.value,
                            options.angular_speed_tolerance.value,
                            config.minimum_angular_speed.value,
                            correction_limit.min(
                                config
                                    .tuning
                                    .angular
                                    .output_limit()
                                    .unwrap_or(f64::INFINITY),
                            ),
                            config
                                .yaw_braking
                                .unwrap_or(config.angular_deceleration)
                                .value
                                .min(config.angular_deceleration.value),
                            reaction,
                        )
                    } else {
                        self.pid
                            .yaw(heading, yaw_rate, feedforward, correction_limit, dt)
                    }
                };
                (drive(speed, turning), reached && facing && slow)
            }
        };
        self.limit_step(wheels, ready, dt, options)
    }

    /// Feed the actual applied wheel targets back to the slew limiter.
    /// A downstream safety governor may have scaled the requested targets.
    pub fn record_applied_wheels(&mut self, wheels: [f64; 2]) {
        self.last_wheels = wheels;
    }

    /// Apply coupled linear/angular command slew constraints without changing the public target API.
    pub fn limit_step(
        &mut self,
        desired: [f64; 2],
        ready: bool,
        dt: Duration,
        options: DriveOptions,
    ) -> ([f64; 2], bool) {
        if ready {
            self.last_wheels = [0.0; 2];
            return ([0.0; 2], true);
        }
        let width = options.track_width.value;
        let current_v = (self.last_wheels[0] + self.last_wheels[1]) * 0.5;
        let current_w = (self.last_wheels[1] - self.last_wheels[0]) / width;
        let desired_v = (desired[0] + desired[1]) * 0.5;
        let desired_w = (desired[1] - desired[0]) / width;
        let seconds = dt.as_secs_f64();
        let slew = |current: f64, target: f64, acceleration: f64, deceleration: f64| {
            let rate = if (current == 0.0 || current.signum() == target.signum())
                && target.abs() > current.abs()
            {
                acceleration
            } else {
                deceleration
            };
            let step = rate * seconds;
            current + (target - current).clamp(-step, step)
        };
        let v = slew(
            current_v,
            desired_v,
            self.config.acceleration.value,
            self.config.deceleration.value,
        );
        let w = slew(
            current_w,
            desired_w,
            self.config.angular_acceleration.value,
            self.config.angular_deceleration.value,
        );
        self.last_wheels = desaturate(
            [v - w * width * 0.5, v + w * width * 0.5],
            options.maximum_wheel_speed.value,
        );
        (self.last_wheels, false)
    }
}

/// Raise only an error-reducing command while nearly stationary. The floor is
/// subordinate to the controller's output limit, braking distance, slew limits
/// and the downstream clearance governor. It never amplifies a brake command.
#[allow(clippy::too_many_arguments)]
fn compensate_breakaway(
    output: f64,
    error: f64,
    measured: f64,
    tolerance: f64,
    rest_rate: f64,
    minimum: f64,
    limit: f64,
    deceleration: f64,
    reaction: f64,
) -> f64 {
    if minimum <= 0.0
        || output == 0.0
        || output.abs() >= minimum
        || error.abs() <= tolerance
        || measured.abs() > rest_rate
        || output.signum() != error.signum()
    {
        return output;
    }
    // Invert d = v*T + v^2/(2*a) without subtracting nearly equal values.
    let distance = error.abs() - tolerance;
    let stoppable =
        2.0 * distance / (reaction + (reaction * reaction + 2.0 * distance / deceleration).sqrt());
    output.signum() * output.abs().max(minimum.min(limit).min(stoppable))
}
