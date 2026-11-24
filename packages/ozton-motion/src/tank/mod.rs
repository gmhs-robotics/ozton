use ozton_drivetrain::{Drive, model::tank_axes};

use crate::arcade::Arcade;

/// Left and right side control.
pub struct Tank;

impl Tank {
    pub fn drive<D: Drive + ?Sized>(drive: &mut D, left: f64, right: f64) -> Result<(), D::Error> {
        let (forward, turn) = tank_axes([left, right]);
        Arcade::drive(drive, forward, turn)
    }
}
