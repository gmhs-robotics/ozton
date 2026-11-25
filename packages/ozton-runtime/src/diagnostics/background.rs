//! One periodic hardware sampling task, independent of competition mode.

use std::{
    cell::RefCell,
    rc::{Rc, Weak},
    time::Duration,
};

use ozton_diagnostics::serial;
use vexide::{
    competition,
    task::{Task, spawn},
    time::sleep,
};

use super::Frame;

// Full motor, battery, and smart-device snapshots are for health diagnostics.
// Control and localization traces are recorded independently at their own cadence.
const HARDWARE_SAMPLE_INTERVAL: Duration = Duration::from_millis(100);
const STATUS_POLL_INTERVAL: Duration = Duration::from_millis(25);

type Source = dyn Fn(Frame);

std::thread_local! {
    static SOURCES: RefCell<Vec<Weak<Source>>> = RefCell::new(Vec::new());
}

/// The owner retains the returned handle. Registration never holds a robot alive.
pub(crate) fn register(source: impl Fn(Frame) + 'static) -> Rc<Source> {
    let source: Rc<Source> = Rc::new(source);
    SOURCES.with(|sources| sources.borrow_mut().push(Rc::downgrade(&source)));
    source
}

fn sample_sources(frame: Frame) {
    SOURCES.with(|sources| {
        sources.borrow_mut().retain(|source| {
            let Some(source) = source.upgrade() else {
                return false;
            };
            source(frame);
            true
        });
    });
}

/// Sample connected hardware once per frame. No application reporters required.
pub(crate) fn start_diagnostics() -> Task<()> {
    spawn(async move {
        let mut sequence = 0u64;
        let mut previous_status = None;
        let mut last_sample = None;
        loop {
            sleep(STATUS_POLL_INTERVAL).await;
            let status = competition::status();
            if previous_status != Some(status) {
                crate::log_event!(
                    Info,
                    "competition_status",
                    "VEXos flags=0x{:08x} mode={:?} connected={} system={:?}",
                    status.bits(),
                    status.mode(),
                    status.is_connected(),
                    status.system()
                );
                previous_status = Some(status);
            }
            #[cfg(target_os = "vexos")]
            if ozton_diagnostics::sd::under_pressure() {
                continue;
            }
            if !serial::sd_enabled() {
                continue;
            }
            let time = vexide::time::user_uptime();
            if last_sample.is_some_and(|last| time.saturating_sub(last) < HARDWARE_SAMPLE_INTERVAL)
            {
                continue;
            }
            last_sample = Some(time);
            sequence += 1;
            let frame = Frame {
                stamp: serial::Frame { sequence, time },
            };
            frame.row("MODE", format_args!("mode={:?}", status.mode()));
            frame.row(
                "COMPETITION_STATUS",
                format_args!(
                    "flags=0x{:08x},connected={},system={:?}",
                    status.bits(),
                    status.is_connected(),
                    status.system()
                ),
            );
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
            let _span = crate::profile::Span::new(crate::profile::Stage::HardwareReport);
            ozton_tracking::hardware::sample(frame.stamp);
            sample_sources(frame);
        }
    })
}
