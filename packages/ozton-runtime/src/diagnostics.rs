//! Opt-in USB and SD sampling for competition and autonomous runner ticks.

#[cfg(feature = "logging")]
use std::time::Instant;
use std::{marker::PhantomData, time::Duration};

use ozton_diagnostics::serial;
pub use ozton_diagnostics::serial::FaultLog;
use ozton_math::uom::si::f64::{Length, Velocity};
use vexide::smart::SmartDevice;

pub const USB_SAMPLE_INTERVAL: Duration = Duration::from_millis(250);
#[cfg(feature = "logging")]
const SD_SAMPLE_INTERVAL: Duration = Duration::from_millis(20);

/// A sampled timestamp with its USB/SD routing.
#[derive(Clone, Copy)]
pub struct Frame {
    stamp: serial::Frame,
    usb: bool,
}

impl core::ops::Deref for Frame {
    type Target = serial::Frame;
    fn deref(&self) -> &Self::Target {
        &self.stamp
    }
}

impl Frame {
    /// Write one diagnostic row to the destinations scheduled for this sample.
    pub fn row(&self, kind: &str, fields: std::fmt::Arguments<'_>) {
        self.stamp.emit(kind, fields, self.usb);
    }
}

/// Start the SD writer for a competition loop. Each wake writes one batch.
/// VEXos file I/O is synchronous. Keep the task alive for the competition.
pub fn start_sd_logging() -> Option<vexide::task::Task<()>> {
    #[cfg(all(feature = "logging", target_os = "vexos"))]
    return Some(vexide::task::spawn(async {
        let mut failed = false;
        loop {
            vexide::time::sleep(Duration::from_secs(1)).await;
            match ozton_diagnostics::sd::flush() {
                Err(error) if !failed => {
                    event(format_args!("sd_log_unavailable,error={error}"));
                    failed = true;
                }
                Ok(()) if failed => {
                    event(format_args!("sd_log_resumed"));
                    failed = false;
                }
                _ => {}
            }
        }
    }));
    #[cfg(not(all(feature = "logging", target_os = "vexos")))]
    None
}
use vexide::{competition::CompetitionMode, prelude::Motor, smart::motor::MotorControl};

/// Motion state from the autonomous runner, in metres and metres per second.
#[derive(Clone, Copy)]
pub struct MotionSample {
    pub elapsed: Duration,
    pub progress: Length,
    pub wheel_targets: [Velocity; 2],
}

/// Shared context for all rows of a robot sample.
#[derive(Clone, Copy)]
pub struct Sample {
    pub mode: CompetitionMode,
    pub dt: Duration,
    pub motion: Option<MotionSample>,
}

pub trait Report<R> {
    fn collect(robot: &R, frame: Frame, sample: Sample);
}

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

impl<R> Report<R> for () {
    fn collect(_: &R, _: Frame, _: Sample) {}
}

impl<M, T, L, A> ComponentReport for ozton_drivetrain::Drivetrain<M, T, L, A>
where
    M: ozton_drivetrain::model::DrivetrainModel + ozton_drivetrain::MotorGroups,
    T: ozton_tracking::Tracking + ComponentReport,
    L: ComponentReport,
{
    fn report(&self, frame: Frame) {
        #[cfg(feature = "logging")]
        {
            let groups = self.model.motor_groups();
            if groups.len() == 2 {
                report_drive_feedback(frame, self.feedback(), self.wheel_diameter());
            }
            report_drive_motors(frame, groups);
        }
        self.localization.report(frame);
        self.tracking.report(frame);
    }
}

impl ComponentReport for ozton_tracking::NoTracking {
    fn report(&self, _: Frame) {}
}

impl ComponentReport for ozton_tracking::sampling::SensorTracking {
    fn report(&self, frame: Frame) {
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
            pose.heading.to_degrees(),
            u8::from(trusted)
        ),
    );
}

/// UI hooks for a builder's runtime slot. Omitted diagnostics add no tick work.
pub trait Telemetry<R> {
    fn record(robot: &mut R, sample: Sample);
    fn driver_output(_robot: &mut R, _elapsed: Duration) {}
    fn status(_robot: &mut R, _status: &str) {}
}

impl<R> Telemetry<R> for () {
    fn record(_: &mut R, _: Sample) {}
}

pub struct Sampler<S> {
    #[cfg(feature = "logging")]
    clock: SampleClock,
    #[cfg(feature = "logging")]
    last_usb: Instant,
    #[cfg(feature = "logging")]
    usb_interval: Duration,
    source: PhantomData<S>,
}

impl<S> Sampler<S> {
    pub(crate) fn new(interval: Duration) -> Self {
        #[cfg(not(feature = "logging"))]
        let _ = interval;
        Self {
            #[cfg(feature = "logging")]
            clock: SampleClock::new(Instant::now(), interval.min(SD_SAMPLE_INTERVAL)),
            #[cfg(feature = "logging")]
            last_usb: Instant::now(),
            #[cfg(feature = "logging")]
            usb_interval: interval,
            source: PhantomData,
        }
    }

    pub(crate) fn driver_output(&mut self, elapsed: Duration) {
        #[cfg(feature = "logging")]
        {
            self.clock.max_driver_output = self.clock.max_driver_output.max(elapsed);
        }
        #[cfg(not(feature = "logging"))]
        let _ = elapsed;
    }

    pub(crate) fn frame(&mut self, sample: Sample) -> Option<Frame> {
        #[cfg(feature = "logging")]
        {
            let now = Instant::now();
            let snapshot = self.clock.tick(now, sample.dt)?;
            let usb = if now.saturating_duration_since(self.last_usb) >= self.usb_interval {
                self.last_usb = now;
                true
            } else {
                false
            };
            let frame = Frame {
                stamp: serial::Frame {
                    sequence: snapshot.sequence,
                    time: vexide::time::user_uptime(),
                },
                usb,
            };
            frame.row(
                "LOOP",
                format_args!(
                    "ticks={},dt_ms={:.3},max_dt_ms={:.3},max_driver_output_us={}",
                    snapshot.ticks,
                    sample.dt.as_secs_f64() * 1000.0,
                    snapshot.max_tick.as_secs_f64() * 1000.0,
                    snapshot.max_driver_output.as_micros(),
                ),
            );
            frame.row("MODE", format_args!("mode={:?}", sample.mode));
            frame.row(
                "BATTERY",
                format_args!(
                    "volts={:.3},current_a={:.3},capacity={:.3},temp_c={}",
                    vexide::battery::voltage(),
                    vexide::battery::current(),
                    vexide::battery::capacity(),
                    vexide::battery::temperature(),
                ),
            );
            if let Some(motion) = sample.motion {
                frame.row(
                    "AUTON",
                    format_args!(
                        "route_ms={},progress_m={:.5},left_target_mps={:.5},right_target_mps={:.5}",
                        motion.elapsed.as_millis(),
                        motion.progress.value,
                        motion.wheel_targets[0].value,
                        motion.wheel_targets[1].value,
                    ),
                );
            }
            Some(frame)
        }
        #[cfg(not(feature = "logging"))]
        {
            let _ = sample;
            None
        }
    }
}

#[cfg(feature = "logging")]
struct SampleClock {
    interval: Duration,
    last: Instant,
    sequence: u64,
    ticks: u64,
    max_tick: Duration,
    max_driver_output: Duration,
}

#[cfg(feature = "logging")]
struct LoopSample {
    sequence: u64,
    ticks: u64,
    max_tick: Duration,
    max_driver_output: Duration,
}

#[cfg(feature = "logging")]
impl SampleClock {
    fn new(now: Instant, interval: Duration) -> Self {
        Self {
            interval,
            last: now,
            sequence: 0,
            ticks: 0,
            max_tick: Duration::ZERO,
            max_driver_output: Duration::ZERO,
        }
    }

    fn tick(&mut self, now: Instant, dt: Duration) -> Option<LoopSample> {
        self.ticks += 1;
        self.max_tick = self.max_tick.max(dt);
        if now.saturating_duration_since(self.last) < self.interval {
            return None;
        }
        self.last = now;
        self.sequence += 1;
        Some(LoopSample {
            sequence: self.sequence,
            ticks: self.ticks,
            max_tick: std::mem::take(&mut self.max_tick),
            max_driver_output: std::mem::take(&mut self.max_driver_output),
        })
    }
}

/// Report the calibrated drive and its actual motor samples.
#[cfg(feature = "logging")]
pub fn report_differential<T: ozton_tracking::Tracking, L, A>(
    frame: Frame,
    drive: &ozton_drivetrain::Drivetrain<ozton_drivetrain::model::Differential, T, L, A>,
) {
    use ozton_drivetrain::MotorGroups;

    report_drive_feedback(frame, drive.feedback(), drive.wheel_diameter());
    report_drive_motors(frame, drive.model.motor_groups());
}

#[cfg(feature = "logging")]
fn report_drive_feedback(
    frame: Frame,
    feedback: [Option<ozton_drivetrain::WheelFeedback>; 2],
    diameter: Option<Length>,
) {
    let [left, right] = feedback.map(|wheel| wheel.map_or(f64::NAN, |wheel| wheel.position.value));
    let diameter = diameter.map_or(f64::NAN, |diameter| diameter.value);
    frame.row(
        "DRIVE",
        format_args!("left_m={left:.5},right_m={right:.5},wheel_dia_m={diameter:.6}"),
    );
}

#[cfg(feature = "logging")]
fn report_drive_motors(frame: Frame, groups: &[Box<[Motor]>]) {
    let names: &[&str] = match groups.len() {
        2 => &["l", "r"],
        4 => &["fl", "fr", "bl", "br"],
        _ => &[],
    };
    for (group, motors) in groups.iter().enumerate() {
        for (index, motor) in motors.iter().enumerate() {
            let sample = MotorSample::read(motor);
            if let Some(name) = names.get(group) {
                sample.write(frame, format_args!("drive_{name}{}", index + 1));
            } else {
                sample.write(frame, format_args!("drive_{}_{}", group + 1, index + 1));
            }
        }
    }
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

#[cfg(feature = "logging")]
#[macro_export]
macro_rules! diagnostic_event {
    ($($arg:tt)*) => {
        $crate::diagnostics::event(format_args!($($arg)*))
    };
}

#[cfg(not(feature = "logging"))]
#[macro_export]
macro_rules! diagnostic_event {
    ($($arg:tt)*) => {{
        if false {
            let _ = format_args!($($arg)*);
        }
    }};
}

#[cfg(all(test, feature = "logging"))]
mod tests {
    use super::*;

    #[test]
    fn sampler_reports_total_ticks_and_resets_window_peaks() {
        let start = Instant::now();
        let interval = Duration::from_millis(250);
        let mut clock = SampleClock::new(start, interval);
        clock.max_driver_output = Duration::from_micros(750);
        assert!(
            clock
                .tick(
                    start + Duration::from_millis(100),
                    Duration::from_millis(30)
                )
                .is_none()
        );
        let first = clock
            .tick(start + interval, Duration::from_millis(10))
            .unwrap();
        assert_eq!(first.sequence, 1);
        assert_eq!(first.ticks, 2);
        assert_eq!(first.max_tick, Duration::from_millis(30));
        assert_eq!(first.max_driver_output, Duration::from_micros(750));

        let second = clock
            .tick(start + interval * 2, Duration::from_millis(12))
            .unwrap();
        assert_eq!(second.sequence, 2);
        assert_eq!(second.ticks, 3);
        assert_eq!(second.max_tick, Duration::from_millis(12));
        assert_eq!(second.max_driver_output, Duration::ZERO);
    }
}
