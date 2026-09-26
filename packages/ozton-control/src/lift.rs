//! Coordinated motion for mirrored lift motors with integrated encoders.

use std::time::Duration;

/// Lift positions use a positive common coordinate for both mirrored motors.
#[derive(Debug, Clone, Copy)]
pub struct LiftProfile {
    pub bottom_degrees: f64,
    pub top_degrees: f64,
    pub maximum_command: f64,
    pub up_rise_per_second: f64,
    pub down_rise_per_second: f64,
    pub near_top_degrees: f64,
    pub near_bottom_degrees: f64,
    pub sync_gain: f64,
    pub position_gain: f64,
}

/// Converts operator or recorded targets into synchronized motor commands.
pub struct LiftController {
    profile: LiftProfile,
    magnitude: f64,
}

impl LiftController {
    pub const fn new(profile: LiftProfile) -> Self {
        Self {
            profile,
            magnitude: 0.0,
        }
    }

    pub fn update(
        &mut self,
        requested_direction: i8,
        target: Option<f64>,
        first_degrees: f64,
        second_degrees: f64,
        dt: Duration,
    ) -> (f64, f64) {
        let position = (first_degrees + second_degrees) / 2.0;
        let direction = match target {
            Some(target) if (target - position).abs() <= 3.0 => 0.0,
            Some(target) => (target - position).signum(),
            None => f64::from(requested_direction.signum()),
        };
        if direction == 0.0
            || direction > 0.0 && position >= self.profile.top_degrees
            || direction < 0.0 && position <= self.profile.bottom_degrees
        {
            self.magnitude = 0.0;
            return (0.0, 0.0);
        }
        let remaining = if direction > 0.0 {
            self.profile.top_degrees - position
        } else {
            position - self.profile.bottom_degrees
        };
        let slowdown = if direction > 0.0 {
            (remaining / self.profile.near_top_degrees).clamp(0.18, 1.0)
        } else {
            (remaining / self.profile.near_bottom_degrees).clamp(0.18, 1.0)
        };
        let target_magnitude = if let Some(target) = target {
            ((target - position).abs() * self.profile.position_gain)
                .min(self.profile.maximum_command)
        } else {
            self.profile.maximum_command
        };
        let rise = if direction > 0.0 {
            self.profile.up_rise_per_second
        } else {
            self.profile.down_rise_per_second
        };
        self.magnitude = (self.magnitude + rise * dt.as_secs_f64()).min(target_magnitude);
        let base = direction * self.magnitude * slowdown;
        let sync = ((first_degrees - second_degrees) * self.profile.sync_gain).clamp(-0.15, 0.15);
        (
            (base - sync).clamp(-self.profile.maximum_command, self.profile.maximum_command),
            (base + sync).clamp(-self.profile.maximum_command, self.profile.maximum_command),
        )
    }
}
