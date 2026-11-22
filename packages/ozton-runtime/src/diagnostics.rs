//! Opt-in USB and SD sampling for competition and autonomous runner ticks.

#[cfg(any(debug_assertions, feature = "sd-logging"))]
use std::time::Instant;
use std::{marker::PhantomData, time::Duration};

use ozton_diagnostics::serial;
pub use ozton_diagnostics::serial::FaultLog;
use ozton_math::uom::si::f64::{Length, Velocity};
use vexide::smart::SmartDevice;

pub const USB_SAMPLE_INTERVAL: Duration = Duration::from_millis(250);
#[cfg(feature = "sd-logging")]
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
    #[cfg(all(feature = "sd-logging", target_os = "vexos"))]
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
    #[cfg(not(all(feature = "sd-logging", target_os = "vexos")))]
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
    #[cfg(any(debug_assertions, feature = "sd-logging"))]
    clock: SampleClock,
    #[cfg(debug_assertions)]
    last_usb: Instant,
    #[cfg(debug_assertions)]
    usb_interval: Duration,
    source: PhantomData<S>,
}

impl<S> Sampler<S> {
    pub(crate) fn new(interval: Duration) -> Self {
        #[cfg(not(any(debug_assertions, feature = "sd-logging")))]
        let _ = interval;
        Self {
            #[cfg(any(debug_assertions, feature = "sd-logging"))]
            clock: SampleClock::new(Instant::now(), {
                #[cfg(feature = "sd-logging")]
                let interval = interval.min(SD_SAMPLE_INTERVAL);
                interval
            }),
            #[cfg(debug_assertions)]
            last_usb: Instant::now(),
            #[cfg(debug_assertions)]
            usb_interval: interval,
            source: PhantomData,
        }
    }

    pub(crate) fn driver_output(&mut self, elapsed: Duration) {
        #[cfg(any(debug_assertions, feature = "sd-logging"))]
        {
            self.clock.max_driver_output = self.clock.max_driver_output.max(elapsed);
        }
        #[cfg(not(any(debug_assertions, feature = "sd-logging")))]
        let _ = elapsed;
    }

    pub(crate) fn frame(&mut self, sample: Sample) -> Option<Frame> {
        #[cfg(any(debug_assertions, feature = "sd-logging"))]
        {
            let now = Instant::now();
            let snapshot = self.clock.tick(now, sample.dt)?;
            #[cfg(debug_assertions)]
            let usb = if now.saturating_duration_since(self.last_usb) >= self.usb_interval {
                self.last_usb = now;
                true
            } else {
                false
            };
            #[cfg(not(debug_assertions))]
            let usb = false;
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
        #[cfg(not(any(debug_assertions, feature = "sd-logging")))]
        {
            let _ = sample;
            None
        }
    }
}

#[cfg(any(debug_assertions, feature = "sd-logging"))]
struct SampleClock {
    interval: Duration,
    last: Instant,
    sequence: u64,
    ticks: u64,
    max_tick: Duration,
    max_driver_output: Duration,
}

#[cfg(any(debug_assertions, feature = "sd-logging"))]
struct LoopSample {
    sequence: u64,
    ticks: u64,
    max_tick: Duration,
    max_driver_output: Duration,
}

#[cfg(any(debug_assertions, feature = "sd-logging"))]
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

/// Report a stock differential without advancing its encoder feedback.
#[cfg(any(debug_assertions, feature = "sd-logging"))]
pub fn report_differential(frame: Frame, drive: &ozton_drivetrain::model::Differential) {
    report_drive_snapshot(
        frame,
        drive.snapshot(),
        drive.wheel_diameter().map_or(f64::NAN, |v| v.value),
    );
    drive.visit_motors(|side, index, motor| report_drive_motor(frame, side, index, motor));
}

#[cfg(any(debug_assertions, feature = "sd-logging"))]
fn report_drive_motor(frame: Frame, side: usize, index: usize, motor: &Motor) {
    let side = if side == 0 { 'l' } else { 'r' };
    MotorSample::read(motor).write(frame, format_args!("drive_{side}{}", index + 1));
}

#[cfg(any(debug_assertions, feature = "sd-logging"))]
fn report_drive_snapshot(
    frame: Frame,
    snapshot: ozton_drivetrain::model::DriveSnapshot,
    diameter: f64,
) {
    let [left, right] = snapshot.wheel_meters;
    let [left_rev, right_rev] = snapshot.wheel_revolutions;
    frame.row("DRIVE", format_args!(
        "left_cmd={:.4},right_cmd={:.4},scale={:.3},left_rev={left_rev:.6},right_rev={right_rev:.6},left_m={left:.5},right_m={right:.5},wheel_dia_m={:.6},write_failed={},fault_count={}",
        snapshot.command[0], snapshot.command[1], snapshot.scale,
        diameter,
        u8::from(snapshot.write_failed), snapshot.fault_count,
    ));
}

/// Report connected controller axes and buttons using Ozton's stock field names.
pub fn report_controller(frame: Frame, input: Option<&vexide::controller::ControllerState>) {
    let Some(input) = input else {
        frame.row("INPUT", format_args!("ready=0"));
        return;
    };
    frame.row("INPUT", format_args!(
            "throttle={:.4},turn={:.4},r1={},r2={},l1={},l2={},x={},a={},b={},up={},down={},boost={},ready=1",
            input.left_stick.y(), input.right_stick.x(), u8::from(input.button_r1.is_pressed()),
            u8::from(input.button_r2.is_pressed()), u8::from(input.button_l1.is_pressed()),
            u8::from(input.button_l2.is_pressed()), u8::from(input.button_x.is_pressed()),
            u8::from(input.button_a.is_pressed()), u8::from(input.button_b.is_pressed()),
            u8::from(input.button_up.is_pressed()), u8::from(input.button_down.is_pressed()),
            u8::from(input.button_y.is_pressed()),
        ));
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

#[doc(hidden)]
pub fn event(fields: std::fmt::Arguments<'_>) {
    ozton_diagnostics::serial::event(vexide::time::user_uptime(), fields);
}

#[macro_export]
macro_rules! diagnostic_event {
    ($($arg:tt)*) => {
        $crate::diagnostics::event(format_args!($($arg)*))
    };
}

#[cfg(all(test, debug_assertions))]
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
