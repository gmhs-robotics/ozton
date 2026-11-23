//! Clearance-aware point and heading commands.

use ozton_drivetrain::{assist::DriveAssist, model::Differential};
use ozton_math::{
    geometry::Pose,
    path::clearance::RectangularClearance,
    uom::si::{
        angle::degree,
        f64::{Angle, Length, Velocity},
        length::meter,
        velocity::meter_per_second,
    },
};
use ozton_runtime::clock::CONTROL_PERIOD;

use super::{DriveMotionFault, MAX_CONTROL_GAP, TrackedDrive};
use crate::commands::{
    Command, CommandError, DriveOptions, Runner,
    routine_settings::{AUTON_SPEED, AUTON_STAGE_TIMEOUT, AUTON_TOTAL_TIMEOUT},
};

type MotionResult<T = ()> = Result<T, CommandError<DriveMotionFault>>;

pub(super) async fn drive_to_point<A: DriveAssist<Differential>>(
    drive: &mut TrackedDrive<A>,
    target: Pose,
) -> MotionResult {
    ozton_runtime::log_row!(
        Debug,
        "POINT_REQUEST",
        "x_m={:.5},y_m={:.5},heading_rad={:.5}",
        target.position.x,
        target.position.y,
        target.heading.value
    );
    if !target.is_finite() {
        return Err(CommandError::Configuration);
    }

    let clearance = clearance(drive)?;
    let options = options(drive)?;
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
        AUTON_TOTAL_TIMEOUT,
    )?;
    if let Some(field) = clearance {
        depart_wall(&mut runner, field, options).await?;
    }
    runner
        .run(
            Command::TurnToPoint {
                target: target.position,
            },
            AUTON_STAGE_TIMEOUT,
        )
        .await?;
    runner
        .drive_to_point(target.position, AUTON_SPEED, false, AUTON_STAGE_TIMEOUT)
        .await?;
    turn_checked(&mut runner, clearance, target.heading).await?;
    runner.finish_command()
}

pub(super) async fn turn_to<A: DriveAssist<Differential>>(
    drive: &mut TrackedDrive<A>,
    heading: Angle,
) -> MotionResult {
    let clearance = clearance(drive)?;
    let options = options(drive)?;
    let mut runner = Runner::new(
        drive,
        options,
        CONTROL_PERIOD,
        MAX_CONTROL_GAP,
        AUTON_STAGE_TIMEOUT,
    )?;
    turn_checked(&mut runner, clearance, heading).await?;
    runner.finish_command()
}

async fn depart_wall<A: DriveAssist<Differential>>(
    runner: &mut Runner<'_, TrackedDrive<A>>,
    field: RectangularClearance,
    options: DriveOptions,
) -> MotionResult {
    let start = runner.robot_ref().localization.pose();
    if field.can_turn(start) {
        return Ok(());
    }

    let reserve = options.position_tolerance;
    let speed = Velocity::new::<meter_per_second>(0.15);
    let advance = if let Some(advance) = field.forward_to_turn(start, reserve) {
        advance
    } else {
        let arc = field
            .departure_arc(start, options.track_width / 2.0, reserve)
            .ok_or(CommandError::InsufficientClearance)?;
        runner
            .run(
                Command::DriveArc {
                    radius: arc.radius,
                    sweep: arc.sweep,
                    maximum_speed: speed,
                },
                AUTON_STAGE_TIMEOUT,
            )
            .await?;
        field
            .forward_to_turn(runner.robot_ref().localization.pose(), reserve)
            .ok_or(CommandError::InsufficientClearance)?
    };

    if advance > Length::new::<meter>(0.0) {
        let heading = runner.robot_ref().localization.pose().heading;
        runner
            .drive_distance(advance, heading, speed, AUTON_STAGE_TIMEOUT)
            .await?;
    }

    if !field.can_turn(runner.robot_ref().localization.pose()) {
        return Err(CommandError::InsufficientClearance);
    }
    Ok(())
}

async fn turn_checked<A: DriveAssist<Differential>>(
    runner: &mut Runner<'_, TrackedDrive<A>>,
    clearance: Option<RectangularClearance>,
    heading: Angle,
) -> MotionResult {
    if clearance.is_some_and(|field| !field.can_turn(runner.robot_ref().localization.pose())) {
        return Err(CommandError::InsufficientClearance);
    }
    runner.turn_to(heading, AUTON_STAGE_TIMEOUT).await
}

fn clearance<A: DriveAssist<Differential>>(
    drive: &TrackedDrive<A>,
) -> MotionResult<Option<RectangularClearance>> {
    let Some(footprint) = drive.footprint() else {
        return Ok(None);
    };
    let bounds = drive
        .localization
        .rectangular_half_extents()
        .ok_or(CommandError::Configuration)?;
    RectangularClearance::new(footprint, bounds, Length::new::<meter>(0.0127))
        .map(Some)
        .ok_or(CommandError::Configuration)
}

fn options<A: DriveAssist<Differential>>(drive: &TrackedDrive<A>) -> MotionResult<DriveOptions> {
    let mut options =
        DriveOptions::for_differential(drive, AUTON_SPEED).ok_or(CommandError::Configuration)?;
    if drive.footprint().is_some() {
        options.position_tolerance = Length::new::<meter>(0.00635);
        options.heading_tolerance = Angle::new::<degree>(2.0);
    }
    Ok(options)
}
