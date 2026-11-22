//! Robot drivetrains and models.
//!
//! This crate provides types for representing and modeling different mobile robot drivetrain
//! configurations. A *drivetrain* in ozton is the combination of a *model* which describes how the
//! is able to move and a *tracking* system, which describes how the robot can track its motion.
//!
//! At the heart of this crate is the [`Drivetrain`] struct, which bundles together a model and a
//! tracking system. The [`Drivetrain`] type can enacapsulate many different types of robot
//! drivetrains depending on how its model and tracking logic is implemented.

pub mod model;

use model::{
    Arcade, ArcadeInput, Differential, DriveConfigError, DrivetrainModel, MaxSpeed, ProgramDrive,
};
use ozton_math::uom::si::f64::Angle;
use ozton_tracking::Tracking;

/// A mobile robot drivetrain capable of measuring data about itself.
#[derive(Default, Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub struct Drivetrain<M: DrivetrainModel, T: Tracking> {
    /// Motor collection.
    pub model: M,

    /// Tracking system.
    pub tracking: T,
}

impl<M: DrivetrainModel, T: Tracking> Drivetrain<M, T> {
    /// Creates a new drivetrain from a collection of motors and a tracking system.
    pub const fn new(model: M, tracking: T) -> Self {
        Self { model, tracking }
    }
}

/// Supply heading and tilt to the model's driver assists.
pub trait DriverTracking: Tracking {
    fn driver_orientation(&mut self) -> (Option<Angle>, Option<Angle>);
}

impl DriverTracking for ozton_tracking::NoTracking {
    fn driver_orientation(&mut self) -> (Option<Angle>, Option<Angle>) {
        (None, None)
    }
}

impl<T: DriverTracking> Drivetrain<Differential, T> {
    /// Set the assisted driver speed ceiling for this drivetrain.
    pub fn max_speed(&mut self, limit: MaxSpeed) -> Result<(), DriveConfigError> {
        self.model.max_speed(limit)
    }

    /// Drive with an optional output multiplier and tracking-fed assistance.
    pub fn drive_arcade_input(
        &mut self,
        input: ArcadeInput,
    ) -> Result<(), vexide::smart::PortError> {
        let (heading, tilt) = self.tracking.driver_orientation();
        self.model.drive(input.with_orientation(heading, tilt))
    }
}

impl<T: Tracking> DrivetrainModel for Drivetrain<Differential, T> {
    type Error = vexide::smart::PortError;
}

impl<T: DriverTracking> Arcade for Drivetrain<Differential, T> {
    fn drive_arcade(&mut self, throttle: f64, steer: f64) -> Result<(), Self::Error> {
        if vexide::competition::mode() == vexide::competition::CompetitionMode::Autonomous {
            self.model.drive_arcade(throttle, steer)
        } else {
            self.drive_arcade_input(ArcadeInput::new(throttle, steer))
        }
    }
}

impl<M: DrivetrainModel + ProgramDrive, T: Tracking> ProgramDrive for Drivetrain<M, T> {
    type Error = <M as ProgramDrive>::Error;
    type DriverCommand = M::DriverCommand;
    type AutonCommand = M::AutonCommand;

    fn apply_driver(
        &mut self,
        command: Self::DriverCommand,
        dt: std::time::Duration,
    ) -> Result<(), Self::Error> {
        self.model.apply_driver(command, dt)
    }

    fn apply_auton(
        &mut self,
        command: Self::AutonCommand,
        dt: std::time::Duration,
    ) -> Result<(), Self::Error> {
        self.model.apply_auton(command, dt)
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.model.stop()
    }

    fn take_driver_output(&mut self) -> bool {
        self.model.take_driver_output()
    }
}

pub mod feedback;
