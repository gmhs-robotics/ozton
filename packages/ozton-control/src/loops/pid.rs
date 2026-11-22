use core::time::Duration;
use std::ops::{Deref, DerefMut};

use vexide::math::Angle;

use super::{Feedback, Pid};

/// Vexide angle adapter for the hardware-independent PID.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngularPid(Pid);

macro_rules! forward_setters {
    ($($method:ident($($arg:ident: $ty:ty),+)),+ $(,)?) => {
        $(pub const fn $method(&mut self, $($arg: $ty),+) {
            self.0.$method($($arg),+);
        })+
    };
}

impl AngularPid {
    pub const fn new(kp: f64, ki: f64, kd: f64, integration_range: Option<Angle>) -> Self {
        let range = match integration_range {
            Some(angle) => Some(angle.as_radians()),
            None => None,
        };
        Self(Pid::new(kp, ki, kd, range))
    }

    pub const fn integration_range(&self) -> Option<Angle> {
        match self.0.integration_range() {
            Some(range) => Some(Angle::from_radians(range)),
            None => None,
        }
    }

    pub const fn set_integration_range(&mut self, range: Option<Angle>) {
        self.0.set_integration_range(match range {
            Some(angle) => Some(angle.as_radians()),
            None => None,
        });
    }

    forward_setters!(
        set_gains(kp: f64, ki: f64, kd: f64),
        set_kp(kp: f64),
        set_ki(ki: f64),
        set_kd(kd: f64),
        set_output_limit(limit: Option<f64>),
    );
}

impl Deref for AngularPid {
    type Target = Pid;
    fn deref(&self) -> &Pid {
        &self.0
    }
}

impl DerefMut for AngularPid {
    fn deref_mut(&mut self) -> &mut Pid {
        &mut self.0
    }
}

impl Feedback for AngularPid {
    type State = Angle;
    type Signal = f64;

    fn update(&mut self, measurement: Angle, setpoint: Angle, dt: Duration) -> f64 {
        self.0
            .update_angular(measurement.as_radians(), setpoint.as_radians(), dt)
    }
}
