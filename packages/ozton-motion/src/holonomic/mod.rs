use glam::DVec2;
use ozton_drivetrain::LateralDrive;

/// Robot-relative translational and rotational control for a holonomic model.
pub struct Holonomic;

impl Holonomic {
    pub fn drive<D: LateralDrive + ?Sized>(
        drive: &mut D,
        translation: impl Into<DVec2>,
        turn: f64,
    ) -> Result<(), D::Error> {
        drive.drive_vector(translation.into(), turn)
    }
}
