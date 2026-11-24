//! Clearance-aware point and heading commands.

use std::time::Duration;

use ozton_math::{
    geometry::{DVec2, Pose, bearing},
    path::clearance::{RectangularClearance, approach_heading},
    uom::si::{
        angle::degree,
        f64::{Angle, Length, Velocity},
        length::meter,
    },
};
use ozton_runtime::clock::CONTROL_PERIOD;

use super::{DriveMotionFault, MAX_CONTROL_GAP, PoseDrive, PoseSeeking};
use crate::commands::{
    Command, CommandError, Runner, TravelDirection, routine_settings::AUTON_SPEED,
};

pub(super) type MotionResult<T = ()> = Result<T, CommandError<DriveMotionFault>>;

const DEPARTURE_SPEED: Velocity = Velocity {
    value: 0.30,
    ..AUTON_SPEED
};
pub(super) async fn drive_to_target<D: PoseDrive>(
    drive: &mut D,
    position: DVec2,
    final_heading: Option<Angle>,
    settings: PoseSeeking,
) -> MotionResult {
    let target = Pose::new(
        position,
        final_heading.unwrap_or_else(|| drive.pose().heading),
    );
    ozton_runtime::log_row!(
        Debug,
        "POINT_REQUEST",
        "x_m={:.5},y_m={:.5},heading_rad={:.5}",
        target.position.x,
        target.position.y,
        target.heading.value
    );
    if !target.is_finite() || settings.stage_timeout < CONTROL_PERIOD {
        return Err(CommandError::Configuration);
    }

    let clearance = clearance(drive)?;
    let options = settings.options(drive).ok_or(CommandError::Configuration)?;
    if clearance.is_some_and(|field| !field.can_turn(target)) {
        ozton_runtime::log_event!(
            Error,
            "path",
            "Target ({:.3}, {:.3}) m lacks clearance to turn",
            target.position.x,
            target.position.y
        );
        return Err(CommandError::InsufficientClearance);
    }
    let mut runner = Runner::new(
        drive,
        options,
        CONTROL_PERIOD,
        MAX_CONTROL_GAP,
        settings.budget,
    )?
    .with_tuning(settings.tuning)?;
    if let Some(field) = clearance {
        depart_wall(&mut runner, field, target, settings.stage_timeout).await?;
    }
    let tolerance = options.position_tolerance.value;
    for _ in 0..2 {
        let start = runner.robot_ref().pose();
        let displacement = target.position - start.position;
        let reverse = match settings.travel {
            Some(travel) => matches!(travel, TravelDirection::Backward),
            None => {
                approach_heading(
                    start.heading,
                    final_heading.unwrap_or(start.heading),
                    displacement,
                )
                .0
            }
        };
        if displacement.length() > tolerance {
            let heading =
                bearing(displacement) + Angle::new::<degree>(if reverse { 180.0 } else { 0.0 });
            turn_checked(&mut runner, clearance, heading, settings.stage_timeout).await?;
        }
        runner
            .drive_to_point(
                target.position,
                options.maximum_wheel_speed,
                reverse,
                settings.stage_timeout,
            )
            .await?;
        if let Some(heading) = final_heading {
            turn_checked(&mut runner, clearance, heading, settings.stage_timeout).await?;
        }
        let error = runner.robot_ref().pose().position.distance(target.position);
        if error <= tolerance {
            return runner.finish_command();
        }
        ozton_runtime::log_event!(
            Warn,
            "path",
            "Position error after approach: {:.3} m",
            error
        );
    }
    Err(runner.finish_error(CommandError::PositionUnsettled))
}

async fn depart_wall<D: PoseDrive>(
    runner: &mut Runner<'_, D>,
    field: RectangularClearance,
    target: Pose,
    timeout: Duration,
) -> MotionResult {
    let start = runner.robot_ref().pose();
    if field.can_turn(start) {
        return Ok(());
    }

    let options = runner.options();
    let reserve = options.position_tolerance + Length::new::<meter>(0.00635);
    let speed = DEPARTURE_SPEED.min(options.maximum_wheel_speed);
    let plan = field
        .point_departure(start, target, options.track_width / 2.0, reserve)
        .ok_or(CommandError::InsufficientClearance)?;
    ozton_runtime::log_row!(
        Debug,
        "CORNER_DEPARTURE",
        "radius_m={:.5},sweep_rad={:.5},advance_m={:.5}",
        plan.arc.map_or(0.0, |arc| arc.radius.value),
        plan.arc.map_or(0.0, |arc| arc.sweep.value),
        plan.advance.value
    );
    if let Some(arc) = plan.arc {
        runner
            .run(
                Command::DriveArc {
                    radius: arc.radius,
                    sweep: arc.sweep,
                    maximum_speed: speed,
                },
                timeout,
            )
            .await?;
    }

    let measured = runner.robot_ref().pose();
    if !field.can_turn(measured) {
        let advance = field
            .forward_to_turn(measured, reserve)
            .ok_or(CommandError::InsufficientClearance)?;
        runner
            .drive_distance(advance, measured.heading, speed, timeout)
            .await?;
    }
    field
        .can_turn(runner.robot_ref().pose())
        .then_some(())
        .ok_or(CommandError::InsufficientClearance)
}

async fn turn_checked<D: PoseDrive>(
    runner: &mut Runner<'_, D>,
    clearance: Option<RectangularClearance>,
    heading: Angle,
    timeout: Duration,
) -> MotionResult {
    if clearance.is_some_and(|field| !field.can_turn(runner.robot_ref().pose())) {
        return Err(CommandError::InsufficientClearance);
    }
    runner.turn_to(heading, timeout).await
}

fn clearance<D: PoseDrive>(drive: &D) -> MotionResult<Option<RectangularClearance>> {
    let Some(footprint) = drive.footprint() else {
        return Ok(None);
    };
    let bounds = drive
        .rectangular_half_extents()
        .ok_or(CommandError::Configuration)?;
    RectangularClearance::new(footprint, bounds, Length::new::<meter>(0.0127))
        .map(Some)
        .ok_or(CommandError::Configuration)
}
