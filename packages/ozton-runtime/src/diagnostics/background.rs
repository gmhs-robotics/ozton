//! One periodic hardware sampling task, independent of competition mode.

use std::time::Instant;

use vexide::{
    competition,
    task::{Task, spawn},
    time::sleep,
};

use super::{SD_SAMPLE_INTERVAL, Sample, Sampler};

/// Sample connected hardware once per frame. No application reporters required.
pub(crate) fn start_diagnostics() -> Task<()> {
    spawn(async move {
        let mut sampler = Sampler::<()>::new(SD_SAMPLE_INTERVAL);
        sampler.report_control_ticks = false;
        let mut previous = Instant::now();
        loop {
            sleep(SD_SAMPLE_INTERVAL).await;
            let now = Instant::now();
            let dt = now.saturating_duration_since(previous);
            previous = now;
            if let Some(frame) = sampler.frame(Sample {
                mode: competition::mode(),
                dt,
                motion: None,
            }) {
                let _span = crate::profile::Span::new(crate::profile::Stage::HardwareReport);
                ozton_tracking::hardware::sample(frame.stamp);
            }
        }
    })
}
