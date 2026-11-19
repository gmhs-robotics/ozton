//! Opt-in serial sampling for competition and autonomous runner ticks.

#[cfg(debug_assertions)]
use std::time::Instant;
use std::{marker::PhantomData, time::Duration};

pub use ozton_diagnostics::serial::{FaultLog, Frame};
use vexide::{competition::CompetitionMode, prelude::Motor, smart::motor::MotorControl};

/// Motion state from the autonomous runner, in metres and metres per second.
#[derive(Clone, Copy)]
pub struct MotionSample {
    pub elapsed: Duration,
    pub progress: f64,
    pub wheel_targets: [f64; 2],
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

/// Runtime slot used by the builder. Omitted diagnostics add no tick work.
pub trait Telemetry<R> {
    fn record(robot: &mut R, sample: Sample);
    fn driver_output(_robot: &mut R, _elapsed: Duration) {}
}

impl<R> Telemetry<R> for () {
    fn record(_: &mut R, _: Sample) {}
}

pub struct Sampler<S> {
    #[cfg(debug_assertions)]
    clock: SampleClock,
    source: PhantomData<S>,
}

impl<S> Sampler<S> {
    pub(crate) fn new(interval: Duration) -> Self {
        #[cfg(not(debug_assertions))]
        let _ = interval;
        Self {
            #[cfg(debug_assertions)]
            clock: SampleClock::new(Instant::now(), interval),
            source: PhantomData,
        }
    }

    pub(crate) fn driver_output(&mut self, elapsed: Duration) {
        #[cfg(debug_assertions)]
        {
            self.clock.max_driver_output = self.clock.max_driver_output.max(elapsed);
        }
        #[cfg(not(debug_assertions))]
        let _ = elapsed;
    }

    pub(crate) fn frame(&mut self, sample: Sample) -> Option<Frame> {
        #[cfg(debug_assertions)]
        {
            let snapshot = self.clock.tick(Instant::now(), sample.dt)?;
            let frame = Frame {
                sequence: snapshot.sequence,
                time: vexide::time::user_uptime(),
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
            Some(frame)
        }
        #[cfg(not(debug_assertions))]
        {
            let _ = sample;
            None
        }
    }
}

#[cfg(debug_assertions)]
struct SampleClock {
    interval: Duration,
    last: Instant,
    sequence: u64,
    ticks: u64,
    max_tick: Duration,
    max_driver_output: Duration,
}

#[cfg(debug_assertions)]
struct LoopSample {
    sequence: u64,
    ticks: u64,
    max_tick: Duration,
    max_driver_output: Duration,
}

#[cfg(debug_assertions)]
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

/// Smart motor snapshot read only on a scheduled tick. Targets come from the
/// device's own command state, so every control method can use this reporter.
pub struct MotorSample {
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
            "name={name},measured_v={:.3},current_a={:.3},temp_c={:.2},rpm={:.2},torque_nm={:.4},request_v={:.3},position_deg={:.3},command_mode={mode},request_rpm={rpm},request_deg={position},brake_mode={brake:?}",
            self.volts, self.current, self.temperature, self.rpm, self.torque,
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
