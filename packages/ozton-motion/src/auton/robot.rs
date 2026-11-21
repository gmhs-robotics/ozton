//! Stock VEX competition robot. Application code owns mechanism behavior.
use std::{fs::File, time::Duration};

use ozton_drivetrain::model::DriverFaultPolicy;
use ozton_math::{
    geometry::Pose,
    uom::si::f64::{Length, Velocity},
};
use ozton_runtime::{Subsystem, SubsystemList};
use vexide::{controller::ControllerState, smart::PortError};

use super::{AutonRobot, ConfiguredChassis, Feedback, chassis::ChassisError};

/// Shared input for stock chassis subsystems. A controller sample is available
/// during driver control. Autonomous routines set goals directly on their subsystems.
pub struct StockTick<'a> {
    pub controller: Option<&'a ControllerState>,
    pub elapsed: Duration,
}

/// The standard differential chassis and its registered application subsystems.
pub struct Robot<X = ()> {
    pub drive: ConfiguredChassis,
    pub subsystems: X,
    range_occlusion: fn(&X) -> u8,
    fault_policy: DriverFaultPolicy,
    log_file: Option<&'static str>,
}

/// Assemble a stock controller before handing it to the route runner.
#[must_use]
pub struct RobotBuilder<X = ()> {
    drive: ConfiguredChassis,
    subsystems: X,
    range_occlusion: fn(&X) -> u8,
    fault_policy: DriverFaultPolicy,
    log_file: Option<&'static str>,
}

impl RobotBuilder {
    pub fn new(drive: ConfiguredChassis) -> Self {
        Self {
            drive,
            subsystems: (),
            range_occlusion: |_| 0,
            fault_policy: DriverFaultPolicy::Stop,
            log_file: None,
        }
    }
}

impl<X> RobotBuilder<X> {
    /// Append a subsystem. Configure range occlusion after registering subsystems.
    pub fn add_subsystem<N>(self, subsystem: N) -> RobotBuilder<(X, N)>
    where
        for<'a> N: Subsystem<StockTick<'a>, PortError>,
    {
        RobotBuilder {
            drive: self.drive,
            subsystems: (self.subsystems, subsystem),
            range_occlusion: |_| 0,
            fault_policy: self.fault_policy,
            log_file: self.log_file,
        }
    }

    /// Report which range sensors the current subsystem geometry blocks.
    pub fn with_range_occlusion(mut self, read: fn(&X) -> u8) -> Self {
        self.range_occlusion = read;
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

    pub fn build(self) -> Robot<X> {
        Robot {
            drive: self.drive,
            subsystems: self.subsystems,
            range_occlusion: self.range_occlusion,
            fault_policy: self.fault_policy,
            log_file: self.log_file,
        }
    }
}

impl<X> Robot<X> {
    pub(super) fn log_file(&self) -> Option<&'static str> {
        self.log_file
    }

    pub(super) fn driver_tick(&mut self, state: &ControllerState, dt: Duration)
    where
        for<'a> X: SubsystemList<StockTick<'a>, PortError>,
    {
        let result = self.drive.chassis.drivetrain.model.drive_driver_arcade(
            state.left_stick.y(),
            state.right_stick.x(),
            self.fault_policy,
        );
        if (result.is_err() && matches!(self.fault_policy, DriverFaultPolicy::Stop))
            || self
                .subsystems
                .update_all(&mut StockTick {
                    controller: Some(state),
                    elapsed: dt,
                })
                .is_err()
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

impl<X> AutonRobot for Robot<X>
where
    for<'a> X: SubsystemList<StockTick<'a>, PortError>,
{
    type Error = ChassisError;

    fn pose_trusted(&self) -> bool {
        self.drive
            .chassis
            .drivetrain
            .tracking
            .localizer
            .pose_trust()
            == ozton_tracking::localizer::PoseTrust::Trusted
    }
    fn sample(&mut self) -> Result<Option<Feedback>, Self::Error> {
        self.drive
            .chassis
            .sample((self.range_occlusion)(&self.subsystems))
    }

    fn update_subsystems(&mut self, dt: Duration) -> Result<(), Self::Error> {
        self.subsystems
            .update_all(&mut StockTick {
                controller: None,
                elapsed: dt,
            })
            .map_err(ChassisError::Output)
    }

    fn drive(&mut self, speeds: [Velocity; 2], dt: Duration) -> Result<(), Self::Error> {
        self.drive.chassis.drive(speeds, dt)
    }

    fn stop_drive(&mut self) -> Result<(), Self::Error> {
        self.drive.chassis.stop()
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.stop_drive()
            .and(self.subsystems.stop_all().map_err(ChassisError::Output))
    }

    fn set_pose(&mut self, pose: Pose) -> Result<(), Self::Error> {
        self.drive.chassis.set_pose(pose)
    }

    fn cancel_distance_reset(&mut self) {
        self.drive
            .chassis
            .drivetrain
            .tracking
            .localizer
            .cancel_distance_reset();
    }

    fn request_distance_reset(&mut self) {
        self.drive.chassis.request_distance_reset();
    }

    fn log_step(
        &mut self,
        elapsed: Duration,
        dt: Duration,
        progress: Length,
        wheels: [Velocity; 2],
    ) {
        self.drive.chassis.log_step(elapsed, dt, progress, wheels);
    }
}
