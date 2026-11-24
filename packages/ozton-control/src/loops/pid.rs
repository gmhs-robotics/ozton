use core::time::Duration;
use std::ops::{Deref, DerefMut};

use ozton_math::uom::si::f64::Angle;

use super::{Feedback, Pid};

/// Angle adapter for the hardware-independent PID.
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
            Some(angle) => Some(angle.value),
            None => None,
        };
        Self(Pid::new(kp, ki, kd, range))
    }

    pub const fn integration_range(&self) -> Option<Angle> {
        match self.0.integration_range() {
            Some(range) => Some(ozton_math::angle!(range, rad)),
            None => None,
        }
    }

    pub const fn set_integration_range(&mut self, range: Option<Angle>) {
        self.0.set_integration_range(match range {
            Some(angle) => Some(angle.value),
            None => None,
        });
    }

    #[must_use]
    pub const fn with_gains(mut self, kp: f64, ki: f64, kd: f64) -> Self {
        self.set_gains(kp, ki, kd);
        self
    }

    #[must_use]
    pub const fn with_kp(mut self, kp: f64) -> Self {
        self.set_kp(kp);
        self
    }

    #[must_use]
    pub const fn with_ki(mut self, ki: f64) -> Self {
        self.set_ki(ki);
        self
    }

    #[must_use]
    pub const fn with_kd(mut self, kd: f64) -> Self {
        self.set_kd(kd);
        self
    }

    #[must_use]
    pub const fn with_integration_range(mut self, range: Angle) -> Self {
        self.set_integration_range(Some(range));
        self
    }

    #[must_use]
    pub const fn without_output_limit(mut self) -> Self {
        self.set_output_limit(None);
        self
    }

    #[must_use]
    pub const fn without_integration_range(mut self) -> Self {
        self.set_integration_range(None);
        self
    }

    /// Set the normalized output limit when declaring a controller.
    #[must_use]
    pub const fn with_output_limit(mut self, limit: f64) -> Self {
        self.set_output_limit(Some(limit));
        self
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
        self.0.update_angular(measurement.value, setpoint.value, dt)
    }
}
