use ozton_drivetrain::{Drive, model::tank_axes};

use crate::arcade::drive_command;

/// Left and right side control.
pub struct Tank;

impl Tank {
    pub fn drive<D: Drive + ?Sized>(drive: &mut D, left: f64, right: f64) -> Result<(), D::Error> {
        let (forward, turn) = tank_axes([left, right]);
        drive_command(drive, glam::DVec2::new(0.0, forward), turn, "tank")
    }
}
