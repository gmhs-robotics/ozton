//! Assemble a motion model with its tracking and localization state.
#[cfg(feature = "logging")]
use std::{cell::Cell, time::Duration};
use std::{
    cell::{Ref, RefCell, RefMut},
    ops::{Deref, DerefMut},
    rc::Rc,
};

use ozton_math::{
    localization::observation::{ReadHeading, ReadTilt},
    uom::si::f64::{Angle, Length},
};
use ozton_tracking::odometric::DriveEncoders;

use crate::{
    BrakeMotors, Drive, Footprint, HasBraking, HasLocalization, LateralDrive, MotorAddress,
    model::{Balancing, MotionGeometry},
};

#[cfg(feature = "vex")]
mod device;
#[cfg(feature = "vex")]
pub(crate) mod vex;

/// A drivetrain owns geometry and feedback calibration independently of how it mixes motor outputs.
pub struct Drivetrain<M: Drive, T = (), L = ()> {
    pub model: M,
    state: Rc<RefCell<(T, L)>>,
    driver_output: bool,
    #[cfg(feature = "logging")]
    diagnostic_stamp: Rc<Cell<Option<Duration>>>,
}

impl<M: Drive, T, L> Drivetrain<M, T, L> {
    pub fn new(model: M, tracking: T, localization: L) -> Self {
        Self {
            model,
            state: Rc::new(RefCell::new((tracking, localization))),
            driver_output: false,
            #[cfg(feature = "logging")]
            diagnostic_stamp: Rc::new(Cell::new(None)),
        }
    }

    /// Shared rate gate for independently scheduled diagnostic reporters.
    #[cfg(feature = "logging")]
    #[doc(hidden)]
    pub fn diagnostic_stamp(&self) -> Rc<Cell<Option<Duration>>> {
        Rc::clone(&self.diagnostic_stamp)
    }

    /// Consume the write marker used by competition output watchdogs.
    pub fn take_output(&mut self) -> bool {
        std::mem::take(&mut self.driver_output)
    }

    pub fn tracking(&self) -> Ref<'_, T> {
        Ref::map(self.state.borrow(), |(tracking, _)| tracking)
    }

    pub fn tracking_mut(&mut self) -> RefMut<'_, T> {
        RefMut::map(self.state.borrow_mut(), |(tracking, _)| tracking)
    }

    pub fn localization(&self) -> Ref<'_, L> {
        Ref::map(self.state.borrow(), |(_, localization)| localization)
    }

    pub fn localization_mut(&mut self) -> RefMut<'_, L> {
        RefMut::map(self.state.borrow_mut(), |(_, localization)| localization)
    }

    /// Share the observation sources and estimator with a runtime sampling task.
    #[doc(hidden)]
    pub fn localization_state(&self) -> std::rc::Weak<RefCell<(T, L)>> {
        Rc::downgrade(&self.state)
    }
}

impl<M: Drive + MotionGeometry, T, L> Drivetrain<M, T, L> {
    pub fn footprint(&self) -> Option<Footprint> {
        self.model.footprint()
    }
}

impl<M: Drive, T: DriveEncoders, L> DriveEncoders for Drivetrain<M, T, L> {
    fn track_width(&self) -> Length {
        self.tracking().track_width()
    }
    fn tracking_positions(&mut self) -> [Option<Length>; 2] {
        self.tracking_mut().tracking_positions()
    }
}

impl<M: Drive, T, L> HasLocalization for Drivetrain<M, T, L> {
    type Localization = L;

    fn localization(&self) -> impl Deref<Target = L> {
        Drivetrain::localization(self)
    }

    fn localization_mut(&mut self) -> impl DerefMut<Target = L> {
        Drivetrain::localization_mut(self)
    }
}

impl<M: Drive, T, L> Drive for Drivetrain<M, T, L> {
    type Error = M::Error;

    fn drive_vector(&mut self, vector: glam::DVec2, turn: f64) -> Result<(), Self::Error> {
        self.driver_output = true;
        self.model.drive_vector(vector, turn)
    }

    fn stop_now(&mut self) -> Result<(), Self::Error> {
        self.model.stop_now()
    }
}

impl<M: LateralDrive, T, L> LateralDrive for Drivetrain<M, T, L> {}

impl<M: Drive, T: ReadHeading, L> ReadHeading for Drivetrain<M, T, L> {
    fn read_heading(&self) -> Option<Angle> {
        self.tracking().read_heading()
    }
}

impl<M: Drive, T: ReadTilt, L> ReadTilt for Drivetrain<M, T, L> {
    fn read_tilt(&self) -> Option<[Angle; 2]> {
        self.tracking().read_tilt()
    }
}

impl<M: Balancing, T, L> Balancing for Drivetrain<M, T, L> {
    fn balance(&self, command: &mut [f64; 2]) -> Result<(), Self::Error> {
        self.model.balance(command)
    }
}

impl<M: Drive + HasBraking, T, L> HasBraking for Drivetrain<M, T, L> {
    type BrakeError = M::BrakeError;

    fn apply_brake(&mut self, mode: crate::BrakeMode) -> Result<(), Self::BrakeError> {
        self.model.apply_brake(mode)
    }
}

impl<M: Drive + BrakeMotors, T, L> BrakeMotors for Drivetrain<M, T, L> {
    fn brake_motors(
        &mut self,
        select: impl FnMut(MotorAddress) -> Option<crate::BrakeMode>,
    ) -> Result<(), Self::BrakeError> {
        self.model.brake_motors(select)
    }
}
