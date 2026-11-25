//! Component and hardware diagnostic rows.
use std::time::Duration;

use ozton_diagnostics::serial;
pub use ozton_diagnostics::serial::FaultLog;
#[cfg(feature = "logging")]
use ozton_math::uom::si::f64::Length;
use vexide::smart::SmartDevice;

#[cfg(feature = "logging")]
mod background;
#[cfg(feature = "logging")]
pub(crate) use background::{register, start_diagnostics};

#[cfg(feature = "logging")]
mod sd;
#[cfg(feature = "logging")]
pub(crate) use sd::{flush_sd_logging, start_sd_logging};

pub const SD_SAMPLE_INTERVAL: Duration = Duration::from_millis(10);

#[cfg(feature = "logging")]
pub(crate) fn claim_drive_sample(stamp: &std::cell::Cell<Option<Duration>>, now: Duration) -> bool {
    if stamp
        .get()
        .is_some_and(|last| now.saturating_sub(last) < SD_SAMPLE_INTERVAL)
    {
        return false;
    }
    stamp.set(Some(now));
    true
}

/// A sampled timestamp for SD telemetry.
#[derive(Clone, Copy)]
pub struct Frame {
    stamp: serial::Frame,
}

impl core::ops::Deref for Frame {
    type Target = serial::Frame;
    fn deref(&self) -> &Self::Target {
        &self.stamp
    }
}

impl Frame {
    /// Write one diagnostic row to SD.
    pub fn row(&self, kind: &str, fields: std::fmt::Arguments<'_>) {
        self.stamp.row(kind, fields);
    }
}

use vexide::{prelude::Motor, smart::motor::MotorControl};

/// Emit the standard diagnostic rows supported by a component.
pub trait ComponentReport {
    fn report(&self, frame: Frame);
}

impl ComponentReport for ozton_localization::differential::DifferentialLocalization {
    fn report(&self, frame: Frame) {
        report_pose(frame, self.pose(), self.pose_trusted());
    }
}

impl ComponentReport for () {
    fn report(&self, _: Frame) {}
}

impl<M, T, L> ComponentReport for ozton_drivetrain::Drivetrain<M, T, L>
where
    M: ozton_drivetrain::model::Drive + ozton_drivetrain::MotorGroups,
    T: ozton_tracking::TrackingSystem + ComponentReport,
    L: ComponentReport,
{
    fn report(&self, frame: Frame) {
        #[cfg(feature = "logging")]
        {
            if !claim_drive_sample(&self.diagnostic_stamp(), frame.time) {
                return;
            }
            let _span = crate::profile::Span::new(crate::profile::Stage::HardwareReport);
            // The central hardware sampler records all connected motors.
        }
        self.localization().report(frame);
        self.tracking().report(frame);
    }
}

impl ComponentReport for ozton_tracking::NoTracking {
    fn report(&self, _: Frame) {}
}

impl<S: ozton_tracking::TrackingSources, D> ComponentReport for ozton_tracking::Tracking<S, D>
where
    Self: ozton_tracking::TrackingSystem,
{
    fn report(&self, frame: Frame) {
        #[cfg(feature = "logging")]
        if let Some(source) = ozton_tracking::TrackingSystem::selected_drive(self) {
            report_drive_feedback(frame, source.feedback(), source.wheels().diameter());
        }
        if let Some(sample) = self.diagnostic_orientation() {
            let (rate, acceleration) = self.imu_telemetry();
            report_imu(
                frame,
                sample.port,
                [sample.pitch, sample.roll, sample.yaw]
                    .map(|angle| angle.map_or(f64::NAN, |value| value.value)),
                rate,
                acceleration,
            );
        }
    }
}

impl ComponentReport for Motor {
    fn report(&self, frame: Frame) {
        MotorSample::read(self).write(frame, format_args!("port_{}", self.port_number()));
    }
}

/// Report the estimated pose in the standard tracking row.
pub fn report_pose(frame: Frame, pose: ozton_math::geometry::Pose, trusted: bool) {
    frame.row(
        "TRACK",
        format_args!(
            "x_right_m={:.6},y_forward_m={:.6},heading_ccw_deg={:.3},trusted={}",
            pose.position.x,
            pose.position.y,
            pose.heading.get::<ozton_math::uom::si::angle::degree>(),
            u8::from(trusted)
        ),
    );
}

/// Report the calibrated drive and its actual motor samples.
#[cfg(feature = "logging")]
pub fn report_differential<T: ozton_tracking::HasDriveWheels, L>(
    frame: Frame,
    drive: &ozton_drivetrain::Drivetrain<ozton_drivetrain::model::Differential, T, L>,
) {
    report_drive_feedback(frame, drive.feedback(), drive.wheel_diameter());
}

/// Sample motors while a drivetrain-only autonomous command owns the robot.
#[cfg(feature = "logging")]
pub fn report_motion_drive<S: ozton_tracking::TrackingSources, D, L>(
    drive: &ozton_drivetrain::Drivetrain<
        ozton_drivetrain::model::Differential,
        ozton_tracking::Tracking<S, D>,
        L,
    >,
)
where
    ozton_tracking::Tracking<S, D>: ozton_tracking::TrackingSystem,
{
    if serial::sd_enabled() {
        let time = serial::now();
        if !claim_drive_sample(&drive.diagnostic_stamp(), time) {
            return;
        }
        let _span = crate::profile::Span::new(crate::profile::Stage::HardwareReport);
        let frame = Frame {
            stamp: serial::Frame {
                sequence: time.as_micros().try_into().unwrap_or(u64::MAX),
                time,
            },
        };
        drive.tracking().report(frame);
    }
}

#[cfg(feature = "logging")]
pub(crate) fn report_drive_feedback(
    frame: Frame,
    feedback: [Option<ozton_drivetrain::WheelFeedback>; 2],
    diameter: Length,
) {
    let [left, right] = feedback.map(|wheel| wheel.map_or(f64::NAN, |wheel| wheel.position.value));
    let [left_mps, right_mps] =
        feedback.map(|wheel| wheel.map_or(f64::NAN, |wheel| wheel.velocity.value));
    let diameter = diameter.value;
    frame.row(
        "DRIVE",
        format_args!("left_m={left:.5},right_m={right:.5},left_mps={left_mps:.5},right_mps={right_mps:.5},wheel_dia_m={diameter:.6}"),
    );
}

/// Report the tracking IMU's sampled orientation and diagnostic telemetry.
pub fn report_imu(
    frame: Frame,
    port: u8,
    orientation: [f64; 3],
    rate: [f64; 3],
    acceleration: [f64; 3],
) {
    frame.row("IMU", format_args!(
        "port={},pitch_rad={:.5},roll_rad={:.5},yaw_rad={:.5},rate_x_dps={:.4},rate_y_dps={:.4},rate_z_dps={:.4},accel_x_g={:.5},accel_y_g={:.5},accel_z_g={:.5}",
        port, orientation[0], orientation[1], orientation[2],
        rate[0], rate[1], rate[2], acceleration[0], acceleration[1], acceleration[2],
    ));
}

/// Smart motor snapshot read only on a scheduled tick. Targets come from the
/// device's own command state, so every control method can use this reporter.
pub struct MotorSample {
    pub port: u8,
    pub volts: f64,
    pub current: f64,
    pub temperature: f64,
    pub rpm: f64,
    pub torque: f64,
    pub position_degrees: f64,
    pub target: MotorControl,
}

impl MotorSample {
    pub fn read(motor: &Motor) -> Self {
        Self {
            port: motor.port_number(),
            volts: motor.voltage().unwrap_or(f64::NAN),
            current: motor.current().unwrap_or(f64::NAN),
            temperature: motor.temperature().unwrap_or(f64::NAN),
            rpm: motor.velocity().unwrap_or(f64::NAN),
            torque: motor.torque().unwrap_or(f64::NAN),
            position_degrees: motor
                .position()
                .map_or(f64::NAN, |position| position.as_degrees()),
            target: motor.target(),
        }
    }

    pub fn requested_voltage(&self) -> f64 {
        match self.target {
            MotorControl::Brake(_) => 0.0,
            MotorControl::Voltage(volts) => volts,
            _ => f64::NAN,
        }
    }

    pub fn write(&self, frame: Frame, name: impl std::fmt::Display) {
        let (mode, rpm, position, brake) = match self.target {
            MotorControl::Brake(brake) => ("brake", 0.0, f64::NAN, Some(brake)),
            MotorControl::Voltage(_) => ("voltage", f64::NAN, f64::NAN, None),
            MotorControl::Velocity(rpm) => ("velocity", f64::from(rpm), f64::NAN, None),
            MotorControl::Position(angle, rpm) => {
                ("position", f64::from(rpm), angle.as_degrees(), None)
            }
        };
        #[cfg(feature = "logging")]
        let name = serial::field(name);
        frame.row("MOTOR", format_args!(
            "name={name},port={},measured_v={:.3},current_a={:.3},temp_c={:.2},rpm={:.2},torque_nm={:.4},request_v={:.3},position_deg={:.3},command_mode={mode},request_rpm={rpm},request_deg={position},brake_mode={brake:?}",
            self.port, self.volts, self.current, self.temperature, self.rpm, self.torque,
            self.requested_voltage(), self.position_degrees,
        ));
    }
}

#[cfg(feature = "logging")]
#[doc(hidden)]
pub fn event(fields: std::fmt::Arguments<'_>) {
    ozton_diagnostics::serial::event(vexide::time::user_uptime(), fields);
}

#[macro_export]
macro_rules! diagnostic_event {
    ($($arg:tt)*) => {
        $crate::log_event!(Info, "diagnostics", $($arg)*)
    };
}
