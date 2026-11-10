//! Stock VEX competition robot. Application code owns mechanism behavior.

use std::{fs::File, time::Duration};

use ozton_drivetrain::model::DriverFaultPolicy;
use ozton_math::geometry::Pose;
use vexide::{controller::ControllerState, smart::PortError};

use super::{AutonRobot, ConfiguredChassis, Feedback, RangeId, chassis::ChassisError};

/// Application hooks called by the common driver and autonomous executors.
pub trait Mechanisms {
    fn update(&mut self, _dt: Duration) -> Result<(), PortError> {
        Ok(())
    }
    /// Stop all application-controlled outputs and clear retained motion goals.
    fn stop(&mut self) -> Result<(), PortError>;
    fn occluded_ranges(&self) -> u8 {
        0
    }
}

impl Mechanisms for () {
    fn stop(&mut self) -> Result<(), PortError> {
        Ok(())
    }
}

/// The standard differential chassis and user-owned mechanism state.
pub struct Robot<M> {
    pub drive: ConfiguredChassis,
    pub mechanisms: M,
    driver: fn(&mut M, &ControllerState),
    fault_policy: DriverFaultPolicy,
    log_file: Option<&'static str>,
}

impl<M: Mechanisms> Robot<M> {
    pub fn new(drive: ConfiguredChassis, mechanisms: M) -> Self {
        Self {
            drive,
            mechanisms,
            driver: |_, _| {},
            fault_policy: DriverFaultPolicy::Stop,
            log_file: None,
        }
    }

    /// Set mechanism goals from controller input. Ozton supplies arcade drive.
    pub fn with_driver(mut self, driver: fn(&mut M, &ControllerState)) -> Self {
        self.driver = driver;
        self
    }

    pub fn with_driver_fault_policy(mut self, policy: DriverFaultPolicy) -> Self {
        self.fault_policy = policy;
        self
    }

    /// Flush the journal to an SD file on disable. The drive setup enables capture.
    pub fn with_log_file(mut self, path: &'static str) -> Self {
        self.log_file = Some(path);
        self
    }

    pub fn range(&self, name: &str) -> Option<RangeId> {
        self.drive.range(name)
    }

    pub(super) fn log_file(&self) -> Option<&'static str> {
        self.log_file
    }

    pub(super) fn driver_tick(&mut self, state: &ControllerState, dt: Duration) {
        (self.driver)(&mut self.mechanisms, state);
        let result = self.drive.chassis.drivetrain.model.drive_driver_arcade(
            state.left_stick.y(),
            state.right_stick.x(),
            self.fault_policy,
        );
        if (result.is_err() && matches!(self.fault_policy, DriverFaultPolicy::Stop))
            || self.update_mechanisms(dt).is_err()
        {
            let _ = self.stop();
        }
    }

    pub(super) fn save_log(&mut self) -> Option<&'static str> {
        let path = self.log_file?;
        let journal = &mut self.drive.chassis.journal;
        if journal.is_empty() {
            return None;
        }
        if File::create(path)
            .and_then(|file| journal.write_csv(file))
            .is_err()
        {
            return Some("Telemetry save failed");
        }
        journal.clear();
        None
    }
}

impl<M: Mechanisms> AutonRobot for Robot<M> {
    type Error = ChassisError;

    fn sample(&mut self) -> Result<Option<Feedback>, Self::Error> {
        self.drive.chassis.sample(self.mechanisms.occluded_ranges())
    }

    fn update_mechanisms(&mut self, dt: Duration) -> Result<(), Self::Error> {
        self.mechanisms.update(dt).map_err(ChassisError::Output)
    }

    fn drive(&mut self, speeds: [f64; 2], dt: Duration) -> Result<(), Self::Error> {
        self.drive.chassis.drive(speeds, dt)
    }

    fn stop_drive(&mut self) -> Result<(), Self::Error> {
        self.drive.chassis.stop()
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.stop_drive()
            .and(self.mechanisms.stop().map_err(ChassisError::Output))
    }

    fn set_pose(&mut self, pose: Pose) -> Result<(), Self::Error> {
        self.drive.chassis.set_pose(pose)
    }

    fn request_distance_reset(&mut self) {
        self.drive.chassis.request_distance_reset();
    }

    fn log_step(&mut self, elapsed: Duration, dt: Duration, progress: f64, wheels: [f64; 2]) {
        self.drive.chassis.log_step(elapsed, dt, progress, wheels);
    }
}
