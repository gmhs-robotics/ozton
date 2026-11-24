use glam::DVec2 as Vec2;
use ozton_drivetrain::Drive;

/// Normalized forward and turn inputs for an arcade drivetrain.
#[derive(Clone, Copy, Debug)]
pub struct ArcadeInput {
    pub throttle: f64,
    pub turn: f64,
    output_scale: f64,
}

impl ArcadeInput {
    pub const fn new(throttle: f64, turn: f64) -> Self {
        Self {
            throttle,
            turn,
            output_scale: 1.0,
        }
    }

    /// Scale both input axes by a controller-selected fraction before mixing.
    pub const fn with_output_scale(mut self, scale: f64) -> Self {
        self.output_scale = if scale.is_finite() {
            scale.clamp(0.0, 1.0)
        } else {
            0.0
        };
        self
    }

    pub const fn scaled_axes(self) -> (f64, f64) {
        (
            self.throttle * self.output_scale,
            self.turn * self.output_scale,
        )
    }
}

/// Robot-relative forward and turn control.
pub struct Arcade;

impl Arcade {
    pub fn drive<D: Drive + ?Sized>(
        drive: &mut D,
        forward: f64,
        turn: f64,
    ) -> Result<(), D::Error> {
        drive.drive_vector(Vec2::new(0.0, forward), turn)
    }
}
