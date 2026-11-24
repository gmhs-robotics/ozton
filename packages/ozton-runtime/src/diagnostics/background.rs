use std::time::Instant;

use ozton_drivetrain::{Drivetrain, MotorGroups, model::Drive};
use ozton_tracking::{TrackingSystem, odometric::DifferentialTracking};
use vexide::{
    competition,
    task::{Task, spawn},
    time::sleep,
};

use super::{
    ComponentReport, Frame, SD_SAMPLE_INTERVAL, Sample, Sampler, report_drive_feedback,
    report_drive_motors,
};

/// Sample application diagnostics independently of driver and autonomous control.
pub fn start_diagnostics(mut report: impl FnMut(Frame) + 'static) -> Task<()> {
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
                report(frame);
            }
        }
    })
}

/// Capture shared sensor and motor handles without borrowing the drive from its owner.
pub fn drivetrain_reporter<M, T, L>(drive: &Drivetrain<M, T, L>) -> impl Fn(Frame) + 'static
where
    M: Drive + MotorGroups,
    T: TrackingSystem + ComponentReport + 'static,
    L: ComponentReport + 'static,
{
    let groups = drive.model.motor_groups().to_vec();
    let diameter = drive.wheel_diameter();
    let state = drive.localization_state();
    move |frame| {
        let Some(state) = state.upgrade() else { return };
        let Ok(state) = state.try_borrow() else {
            return;
        };
        let (tracking, localization) = &*state;
        if groups.len() == 2 {
            let feedback = tracking
                .drive_wheels()
                .map_or([None; 2], DifferentialTracking::feedback);
            report_drive_feedback(frame, feedback, diameter);
        }
        report_drive_motors(frame, &groups);
        localization.report(frame);
        tracking.report(frame);
    }
}
