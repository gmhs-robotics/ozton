#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct ScalarPid {
    pub(super) kp: f64,
    pub(super) ki: f64,
    pub(super) kd: f64,
    integral: f64,
    pub(super) output_limit: Option<f64>,
    pub(super) prev_error: Option<f64>,
    pub(super) prev_measurement: Option<f64>,
    pub(super) derivative: f64,
}

impl ScalarPid {
    pub(super) const fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            output_limit: None,
            prev_error: None,
            prev_measurement: None,
            derivative: 0.0,
        }
    }

    pub(super) const fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = None;
        self.prev_measurement = None;
        self.derivative = 0.0;
    }

    pub(super) fn update(
        &mut self,
        error: f64,
        derivative: f64,
        seconds: f64,
        integrate: bool,
        bias: f64,
    ) -> f64 {
        self.prev_error = Some(error);
        if !integrate {
            self.integral = 0.0;
        }
        let base = error * self.kp + derivative * self.kd + bias;
        let limit = self
            .output_limit
            .filter(|limit| limit.is_finite() && *limit >= 0.0)
            .unwrap_or(f64::MAX);
        if integrate && self.ki != 0.0 {
            let candidate = self.integral + error * seconds;
            let next = base + candidate * self.ki;
            if next.abs() <= limit
                || libm::copysign(1.0, next) != libm::copysign(1.0, error * self.ki)
            {
                self.integral = candidate;
            }
        }
        (base + self.integral * self.ki).clamp(-limit, limit)
    }

    pub(super) fn integrate_at(&self, error: f64) -> bool {
        self.prev_error
            .is_some_and(|previous| libm::copysign(1.0, previous) == libm::copysign(1.0, error))
    }
}
