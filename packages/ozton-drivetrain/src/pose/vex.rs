//! VEX adapter for the generic pose-drive interface.
#[cfg(feature = "logging")]
use std::time::Duration;
use std::time::Instant;

use ozton_localization::{Fault, Feedback, differential::DifferentialLocalization};
use ozton_math::{
    geometry::{DVec2, Pose},
    path::clearance::Obstacles,
    uom::si::f64::Velocity,
};
use ozton_tracking::{HasDriveWheels, Tracking, TrackingSources};

use super::PoseDrive;
use crate::{
    Drive, Drivetrain, Footprint, MotorGroups,
    model::{Differential, WheelSpeedError},
};

type DriveOutputError = <Differential as Drive>::Error;
type TrackedDrive<S, D> = Drivetrain<Differential, Tracking<S, D>, DifferentialLocalization>;
#[derive(Clone, Copy, Debug)]
pub enum DriveMotionFault {
    Tracking(Fault),
    Output(DriveOutputError),
    Brake(DriveOutputError),
    InvalidCommand,
}

impl std::fmt::Display for DriveMotionFault {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Tracking(fault) => write!(f, "Tracking feedback unavailable ({fault:?})"),
            Self::Output(error) => write!(f, "Could not command drive motors ({error})"),
            Self::Brake(error) => write!(f, "Could not stop drive motors ({error})"),
            Self::InvalidCommand => f.write_str("Invalid drivetrain command or wheel geometry"),
        }
    }
}

impl std::error::Error for DriveMotionFault {}

impl From<WheelSpeedError> for DriveMotionFault {
    fn from(error: WheelSpeedError) -> Self {
        match error {
            WheelSpeedError::Port(port) => Self::Output(port),
            _ => Self::InvalidCommand,
        }
    }
}

impl<S: TrackingSources, D> PoseDrive for TrackedDrive<S, D>
where
    Tracking<S, D>: HasDriveWheels,
{
    type Error = DriveMotionFault;

    fn pose(&self) -> Pose {
        self.localization().pose()
    }

    fn footprint(&self) -> Option<Footprint> {
        self.footprint()
    }

    fn rectangular_half_extents(&self) -> Option<DVec2> {
        self.localization().rectangular_half_extents()
    }

    fn motion_obstacles(&self) -> Obstacles {
        self.localization().motion_obstacles()
    }

    fn movable_obstacles(&self) -> Obstacles {
        self.localization().movable_obstacles()
    }

    fn pose_feedback(&self) -> Result<Option<Feedback>, Self::Error> {
        self.localization()
            .feedback()
            .map_err(DriveMotionFault::Tracking)
    }

    fn position_covariance(&self) -> Option<glam::DMat2> {
        self.localization().position_covariance()
    }

    fn velocity_uncertainty(&self) -> [f64; 3] {
        self.localization().velocity_uncertainty()
    }

    fn drive_wheels(&mut self, speeds: [Velocity; 2]) -> Result<(), Self::Error> {
        self.drive_wheel_speeds(speeds).map_err(Into::into)
    }

    fn brake(&mut self) -> Result<(), Self::Error> {
        self.stop_now().map_err(DriveMotionFault::Brake)
    }

    fn emergency_brake(&mut self) -> Result<(), Self::Error> {
        // The characterized native zero-speed response is active, unlike
        // the model's normal zero-voltage coast. On an output fault, try
        // electrical braking on every motor before reporting the failure.
        let result = self.drive_wheel_speeds([ozton_math::qty!(0 m/s); 2]);
        if result.is_err() {
            for group in self.model.motor_groups() {
                let _ = crate::vex::each_motor(&mut group.borrow_mut(), |motor| {
                    motor.brake(vexide::smart::motor::BrakeMode::Brake)
                });
            }
        }
        result.map_err(Into::into)
    }

    fn pose_trusted(&self) -> bool {
        self.localization().pose_trusted()
    }

    fn wait_until(&mut self, deadline: Instant) -> impl std::future::Future<Output = ()> {
        vexide::time::sleep_until(deadline)
    }

    fn motion_active(&self) -> bool {
        vexide::competition::mode() == vexide::competition::CompetitionMode::Autonomous
    }

    #[cfg(feature = "logging")]
    fn log_step(&mut self, elapsed: Duration, dt: Duration) {
        if elapsed.as_millis() / 10 == elapsed.saturating_sub(dt).as_millis() / 10
            || !ozton_diagnostics::serial::sd_enabled()
        {
            return;
        }
        let time = ozton_diagnostics::serial::now();
        let stamp = self.diagnostic_stamp();
        if stamp
            .get()
            .is_some_and(|last| time.saturating_sub(last) < Duration::from_millis(10))
        {
            return;
        }
        stamp.set(Some(time));
        let _span = ozton_diagnostics::profile::Span::new(
            ozton_diagnostics::profile::Stage::HardwareReport,
        );
        let frame = ozton_diagnostics::serial::Frame {
            sequence: time.as_micros().try_into().unwrap_or(u64::MAX),
            time,
        };
        let feedback = self.feedback();
        let [left, right] =
            feedback.map(|wheel| wheel.map_or(f64::NAN, |wheel| wheel.position.value));
        let [left_mps, right_mps] =
            feedback.map(|wheel| wheel.map_or(f64::NAN, |wheel| wheel.velocity.value));
        let diameter = self
            .wheel_diameter()
            .map_or(f64::NAN, |diameter| diameter.value);
        frame.row("DRIVE", format_args!("left_m={left:.5},right_m={right:.5},left_mps={left_mps:.5},right_mps={right_mps:.5},wheel_dia_m={diameter:.6}"));
        let tracking = self.tracking();
        if let Some(sample) = tracking.diagnostic_orientation() {
            let (rate, acceleration) = tracking.imu_telemetry();
            let [pitch, roll, yaw] = [sample.pitch, sample.roll, sample.yaw]
                .map(|angle| angle.map_or(f64::NAN, |value| value.value));
            frame.row("IMU", format_args!("port={},pitch_rad={pitch:.5},roll_rad={roll:.5},yaw_rad={yaw:.5},rate_x_dps={:.4},rate_y_dps={:.4},rate_z_dps={:.4},accel_x_g={:.5},accel_y_g={:.5},accel_z_g={:.5}", sample.port, rate[0], rate[1], rate[2], acceleration[0], acceleration[1], acceleration[2]));
        }
    }

    fn request_distance_reset(&mut self) {
        self.localization_mut().request_distance_reset();
    }

    fn cancel_distance_reset(&mut self) {
        self.localization_mut().cancel_distance_reset();
    }
}
