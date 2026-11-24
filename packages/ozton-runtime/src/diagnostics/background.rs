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

// Hardware packets update less often than the 10 ms motion loop. Sampling
// them separately reduces SD write volume without decimating control traces.
const HARDWARE_SAMPLE_INTERVAL: Duration = Duration::from_millis(25);

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
        loop {
            sleep(HARDWARE_SAMPLE_INTERVAL).await;
            #[cfg(target_os = "vexos")]
            if ozton_diagnostics::sd::under_pressure() {
                continue;
            }
            if !serial::sd_enabled() {
                continue;
            }
            sequence += 1;
            let frame = Frame {
                stamp: serial::Frame {
                    sequence,
                    time: vexide::time::user_uptime(),
                },
            };
            frame.row("MODE", format_args!("mode={:?}", competition::mode()));
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
