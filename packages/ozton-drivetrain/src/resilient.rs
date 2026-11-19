//! Competition policy: keep every available drive motor running and retry missing motors.

use std::{cell::Cell, rc::Rc};

use ozton_diagnostics::serial::FaultLog;
use vexide::smart::PortError;

use crate::model::{Differential, DrivetrainModel, Tank};

/// Chassis geometry for converting wheel commands and feedback; SI units.
#[derive(Debug, Clone, Copy)]
pub struct DriveGeometry {
    pub maximum_wheel_speed: f64,
    pub wheel_diameter: f64,
}

impl DriveGeometry {
    /// Geometry and maximum wheel speed in physical units.
    pub const fn from_units(
        maximum_wheel_speed: ozton_math::units::Speed,
        wheel_diameter: ozton_math::units::Length,
    ) -> Self {
        Self {
            maximum_wheel_speed: maximum_wheel_speed.as_meters_per_second(),
            wheel_diameter: wheel_diameter.as_meters(),
        }
    }
}

pub trait AdjustableDrive: Tank<Error = PortError> {
    fn set_output_scale(&mut self, scale: f64);
    fn brake_now(&mut self) -> Result<(), PortError> {
        self.stop_now()
    }
}

impl AdjustableDrive for Differential {
    fn set_output_scale(&mut self, scale: f64) {
        Differential::set_output_scale(self, scale);
    }
    fn brake_now(&mut self) -> Result<(), PortError> {
        Differential::brake_now(self, vexide::smart::motor::BrakeMode::Brake)
    }
}

/// Configurable differential output, boost and fault reporting for any geared chassis.
/// Mechanism-dependent limits are supplied by the application as a speed scale.
pub struct ResilientDrive<M: AdjustableDrive = Differential> {
    model: M,
    boost: Option<Rc<Cell<bool>>>,
    normal_scale: f64,
    driver_scale: f64,
    speed_scale: f64,
    geometry: DriveGeometry,
    #[cfg(debug_assertions)]
    debug_command: (f64, f64, f64),
    #[cfg(debug_assertions)]
    last_positions: Cell<[Option<f64>; 2]>,
    status: FaultLog,
}

impl<M: AdjustableDrive> ResilientDrive<M> {
    pub fn new(
        model: M,
        boost: Rc<Cell<bool>>,
        normal_scale: f64,
        geometry: DriveGeometry,
    ) -> Self {
        Self {
            boost: Some(boost),
            ..Self::new_local(model, normal_scale, geometry)
        }
    }

    /// Build a drive whose application writes driver scale directly.
    pub fn new_local(model: M, normal_scale: f64, geometry: DriveGeometry) -> Self {
        Self {
            model,
            boost: None,
            normal_scale,
            driver_scale: normal_scale,
            speed_scale: 1.0,
            geometry,
            #[cfg(debug_assertions)]
            debug_command: (0.0, 0.0, normal_scale),
            #[cfg(debug_assertions)]
            last_positions: Cell::new([None; 2]),
            status: FaultLog::new("drive", vexide::time::user_uptime),
        }
    }

    /// Set the caller-controlled driver output scale before the stability envelope.
    /// Brake and autonomous commands restore the configured normal scale.
    pub fn set_driver_scale(&mut self, scale: f64) {
        self.driver_scale = if scale.is_finite() {
            scale.clamp(0.0, 1.0)
        } else {
            0.0
        };
    }

    fn reset_driver_scale(&mut self) {
        if let Some(boost) = &self.boost {
            boost.set(false);
        }
        self.driver_scale = self.normal_scale;
    }

    /// Apply an application-owned envelope to both driver and autonomous output.
    /// Invalid scales are neutral. This library does not read mechanism height,
    /// impose mechanism geometry, or decide when a mechanism is raised.
    pub fn set_speed_scale(&mut self, scale: f64) {
        self.speed_scale = if scale.is_finite() {
            scale.clamp(0.0, 1.0)
        } else {
            0.0
        };
    }

    fn record_result<E: core::fmt::Debug>(&mut self, result: &Result<(), E>) {
        match result {
            Ok(()) => self.status.recovered(),
            Err(error) => self
                .status
                .fail("write_failed", format_args!(",error={error:?}")),
        }
    }
    fn send_tank(&mut self, left: f64, right: f64, scale: f64) -> Result<(), PortError> {
        self.model.set_output_scale(scale);
        #[cfg(debug_assertions)]
        {
            self.debug_command = (left, right, scale);
        }
        let result = self.model.drive_tank(left, right);
        self.record_result(&result);
        result
    }
    fn send_brake(&mut self) -> Result<(), PortError> {
        #[cfg(debug_assertions)]
        {
            self.debug_command = (0.0, 0.0, self.normal_scale);
        }
        let result = AdjustableDrive::brake_now(&mut self.model);
        self.record_result(&result);
        result
    }

    /// Best-effort stop that retries every motor even after a port failure.
    pub fn brake_now(&mut self) -> Result<(), PortError> {
        self.reset_driver_scale();
        self.model.set_output_scale(self.normal_scale);
        let _ = self.send_brake();
        Ok(())
    }

    pub fn coast_now(&mut self) -> Result<(), PortError> {
        self.reset_driver_scale();
        self.model.set_output_scale(self.normal_scale);
        let _ = self.model.stop_now();
        Ok(())
    }
}

impl<M: AdjustableDrive> DrivetrainModel for ResilientDrive<M> {
    type Error = PortError;
}

impl<M: AdjustableDrive> ResilientDrive<M> {
    /// Same driver response, with observable output errors.
    pub fn live_drive(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        let scale = if self.boost.as_ref().is_some_and(|boost| boost.get()) {
            1.0
        } else {
            self.driver_scale
        } * self.speed_scale;
        let left = if left.is_finite() { left } else { 0.0 };
        let right = if right.is_finite() { right } else { 0.0 };
        self.send_tank(left, right, scale)
    }

    /// Propagate output failures so the caller chooses retry/fallback policy.
    pub fn autonomous_drive(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        self.reset_driver_scale();
        self.model.set_output_scale(self.normal_scale);
        if left == 0.0 && right == 0.0 {
            self.send_brake()
        } else {
            self.send_tank(left, right, self.normal_scale * self.speed_scale)
        }
    }

    pub fn autonomous_brake(&mut self) -> Result<(), PortError> {
        self.reset_driver_scale();
        self.model.set_output_scale(self.normal_scale);
        self.send_brake()
    }
}

impl ResilientDrive {
    /// Read geared-side load telemetry without advancing odometry.
    pub fn wheel_load(&self) -> [Option<crate::model::WheelLoad>; 2] {
        self.model.wheel_load()
    }

    pub fn tracking_positions(&self) -> [Option<f64>; 2] {
        let positions = self
            .model
            .feedback()
            .map(|sample| sample.map(|sample| sample.position));
        #[cfg(debug_assertions)]
        self.last_positions.set(positions);
        positions
    }
    pub fn autonomous_wheel_speeds(
        &mut self,
        speeds: [f64; 2],
    ) -> Result<(), crate::model::WheelSpeedError> {
        self.reset_driver_scale();
        self.model.set_output_scale(self.speed_scale);
        let result =
            self.model
                .drive_wheel_speeds(speeds[0], speeds[1], self.geometry.maximum_wheel_speed);
        #[cfg(debug_assertions)]
        {
            self.debug_command = (
                speeds[0] / self.geometry.maximum_wheel_speed,
                speeds[1] / self.geometry.maximum_wheel_speed,
                self.speed_scale,
            );
        }
        self.record_result(&result);
        result
    }
    pub fn visit_motors(&self, visit: impl FnMut(usize, usize, &vexide::prelude::Motor)) {
        self.model.visit_motors(visit);
    }

    /// Snapshot from the preceding control tick. Reading it does not touch encoders.
    #[cfg(debug_assertions)]
    pub fn snapshot(&self) -> DriveSnapshot {
        let wheel_meters = self
            .last_positions
            .get()
            .map(|position| position.unwrap_or(f64::NAN));
        let circumference = core::f64::consts::PI * self.geometry.wheel_diameter;
        DriveSnapshot {
            command: [self.debug_command.0, self.debug_command.1],
            scale: self.debug_command.2,
            wheel_meters,
            wheel_revolutions: wheel_meters.map(|meters| meters / circumference),
            write_failed: self.status.code().is_some(),
            fault_count: self.status.count(),
        }
    }

    /// Consensus side feedback, converted to wheel revolutions.
    /// Distance autonomous requires both encoders; driver control remains tolerant.
    pub fn wheel_positions(&self) -> Option<(f64, f64)> {
        let [Some(left), Some(right)] = self.model.feedback() else {
            return None;
        };
        let circumference = core::f64::consts::PI * self.geometry.wheel_diameter;
        Some((
            left.position / circumference,
            right.position / circumference,
        ))
    }
}

#[cfg(debug_assertions)]
#[derive(Clone, Copy)]
pub struct DriveSnapshot {
    pub command: [f64; 2],
    pub scale: f64,
    pub wheel_meters: [f64; 2],
    pub wheel_revolutions: [f64; 2],
    pub write_failed: bool,
    pub fault_count: u64,
}

impl<M: AdjustableDrive> Tank for ResilientDrive<M> {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        let _ = self.live_drive(left, right);
        Ok(())
    }

    fn stop_now(&mut self) -> Result<(), PortError> {
        self.coast_now()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn geometry() -> DriveGeometry {
        DriveGeometry {
            maximum_wheel_speed: 1.0,
            wheel_diameter: 0.1,
        }
    }
    #[derive(Default)]
    struct FakeDrive {
        commands: Vec<(f64, f64)>,
        stops: usize,
        scale: f64,
    }
    impl AdjustableDrive for FakeDrive {
        fn set_output_scale(&mut self, scale: f64) {
            self.scale = scale;
        }
    }
    impl DrivetrainModel for FakeDrive {
        type Error = PortError;
    }
    impl Tank for FakeDrive {
        fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), PortError> {
            self.commands.push((left, right));
            Err(PortError::Disconnected { port: 1 })
        }
        fn stop_now(&mut self) -> Result<(), PortError> {
            self.stops += 1;
            Err(PortError::Disconnected { port: 1 })
        }
    }

    #[test]
    fn missing_drive_motor_does_not_abort_driver_or_prevent_retry() {
        let mut drive = ResilientDrive::new(
            FakeDrive::default(),
            Rc::new(Cell::new(false)),
            0.70,
            geometry(),
        );
        assert!(drive.drive_tank(0.4, 0.5).is_ok());
        assert!(drive.drive_tank(0.2, 0.3).is_ok());
        assert_eq!(drive.model.commands, [(0.4, 0.5), (0.2, 0.3)]);
        assert!(drive.stop_now().is_ok());
        assert!(drive.stop_now().is_ok());
        assert_eq!(drive.model.stops, 2);
        assert!(drive.brake_now().is_ok());
        assert_eq!(drive.model.stops, 3);
    }

    #[test]
    fn autonomous_propagates_output_faults_and_stops_clear_boost() {
        let boost = Rc::new(Cell::new(true));
        let mut drive = ResilientDrive::new(FakeDrive::default(), boost.clone(), 0.70, geometry());
        assert!(drive.autonomous_drive(0.35, 0.35).is_err());
        assert!(!boost.get());
        assert_eq!(drive.model.commands, [(0.35, 0.35)]);
        assert!(drive.autonomous_drive(0.0, 0.0).is_err());
        assert!(drive.autonomous_brake().is_err());
        assert_eq!(drive.model.stops, 2);
    }

    #[cfg(debug_assertions)]
    #[test]
    fn debug_drive_keeps_request_scale_and_failed_write_count() {
        let boost = Rc::new(Cell::new(true));
        let mut drive = ResilientDrive::new(FakeDrive::default(), boost, 0.7, geometry());
        assert!(drive.live_drive(0.2, -0.3).is_err());
        assert_eq!(drive.debug_command, (0.2, -0.3, 1.0));
        assert_eq!(drive.status.count(), 1);
        assert!(drive.autonomous_drive(0.35, 0.35).is_err());
        assert_eq!(drive.debug_command, (0.35, 0.35, 0.7));
        assert_eq!(drive.status.count(), 2);
        drive.brake_now().unwrap();
        assert_eq!(drive.debug_command, (0.0, 0.0, 0.7));
        assert_eq!(drive.status.count(), 3);
    }

    #[test]
    fn invalid_commands_become_neutral() {
        let mut drive = ResilientDrive::new(
            FakeDrive::default(),
            Rc::new(Cell::new(false)),
            0.70,
            geometry(),
        );
        assert!(drive.drive_tank(f64::NAN, f64::INFINITY).is_ok());
        assert_eq!(drive.model.commands, [(0.0, 0.0)]);
    }

    #[test]
    fn boost_only_lasts_while_held_and_stop_clears_it() {
        let boost = Rc::new(Cell::new(false));
        let mut drive = ResilientDrive::new(FakeDrive::default(), boost.clone(), 0.70, geometry());
        for held in [false, true, true, false, true] {
            boost.set(held);
            assert!(drive.drive_tank(1.0, -1.0).is_ok());
            assert_eq!(drive.model.scale, if held { 1.0 } else { 0.70 });
        }
        assert!(drive.stop_now().is_ok());
        assert!(!boost.get());
        assert_eq!(drive.model.scale, 0.70);
        assert!(drive.drive_tank(1.0, 1.0).is_ok());
        assert_eq!(drive.model.scale, 0.70);
    }
    #[test]
    fn application_speed_scale_affects_driver_and_autonomous() {
        let mut drive = ResilientDrive::new(
            FakeDrive::default(),
            Rc::new(Cell::new(false)),
            0.75,
            geometry(),
        );
        drive.set_speed_scale(0.4);
        let _ = drive.live_drive(1.0, -1.0);
        assert!((drive.model.scale - 0.3).abs() < 1e-12);
        let _ = drive.autonomous_drive(1.0, -1.0);
        assert!((drive.model.scale - 0.3).abs() < 1e-12);
        drive.set_speed_scale(f64::NAN);
        let _ = drive.live_drive(1.0, 1.0);
        assert_eq!(drive.model.scale, 0.0);
    }
}
