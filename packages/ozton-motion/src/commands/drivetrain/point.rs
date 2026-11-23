//! Clearance-aware point and heading commands.

use std::time::Duration;

use ozton_drivetrain::{assist::DriveAssist, model::Differential};
use ozton_math::{
    geometry::{Pose, bearing},
    path::clearance::{RectangularClearance, approach_heading},
    uom::si::{
        angle::degree,
        angular_velocity::radian_per_second,
        f64::{Angle, AngularVelocity, Length, Velocity},
        length::meter,
    },
};
use ozton_runtime::clock::CONTROL_PERIOD;

use super::{DriveMotionFault, MAX_CONTROL_GAP, TrackedDrive};
use crate::commands::{
    Command, CommandError, DriveOptions, Runner,
    routine_settings::{AUTON_SPEED, AUTON_STAGE_TIMEOUT, AUTON_TOTAL_TIMEOUT},
};

type MotionResult<T = ()> = Result<T, CommandError<DriveMotionFault>>;

const DEPARTURE_SPEED: Velocity = Velocity {
    value: 0.30,
    ..AUTON_SPEED
};
const POINT_WHEEL_SPEED: Velocity = Velocity {
    value: 0.50,
    ..AUTON_SPEED
};

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
        depart_wall(&mut runner, field, options, target).await?;
    }
    let arrival_tolerance = options.position_tolerance.value * 2.0;
    for _ in 0..2 {
        let start = runner.robot_ref().localization.pose();
        let displacement = target.position - start.position;
        if displacement.length() > options.position_tolerance.value {
            let (reverse, _) = approach_heading(start.heading, target.heading, displacement);
            let heading =
                bearing(displacement) + Angle::new::<degree>(if reverse { 180.0 } else { 0.0 });
            turn_checked(&mut runner, clearance, heading).await?;
            runner
                .drive_to_point(
                    target.position,
                    POINT_WHEEL_SPEED,
                    reverse,
                    AUTON_STAGE_TIMEOUT,
                )
                .await?;
        }
        turn_checked(&mut runner, clearance, target.heading).await?;
        let error = runner
            .robot_ref()
            .localization
            .pose()
            .position
            .distance(target.position);
        if error <= arrival_tolerance {
            return runner.finish_command();
        }
        ozton_runtime::log_event!(
            Warn,
            "path",
            "Position shifted {:.3} m during final turn",
            error
        );
    }
    Err(runner.finish_error(CommandError::PositionUnsettled))
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
    target: Pose,
) -> MotionResult {
    let start = runner.robot_ref().localization.pose();
    if field.can_turn(start) {
        return Ok(());
    }

    let reserve = options.position_tolerance;
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
                    maximum_speed: DEPARTURE_SPEED,
                },
                AUTON_STAGE_TIMEOUT,
            )
            .await?;
    }

    let measured = runner.robot_ref().localization.pose();
    if !field.can_turn(measured) {
        let advance = field
            .forward_to_turn(measured, reserve)
            .ok_or(CommandError::InsufficientClearance)?;
        runner
            .drive_distance(
                advance,
                measured.heading,
                DEPARTURE_SPEED,
                AUTON_STAGE_TIMEOUT,
            )
            .await?;
    }
    field
        .can_turn(runner.robot_ref().localization.pose())
        .then_some(())
        .ok_or(CommandError::InsufficientClearance)
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
    let mut options = DriveOptions::for_differential(drive, POINT_WHEEL_SPEED)
        .ok_or(CommandError::Configuration)?;
    if drive.footprint().is_some() {
        options.position_tolerance = Length::new::<meter>(0.00635);
        options.heading_tolerance = Angle::new::<degree>(1.0);
        options.angular_speed_tolerance = AngularVelocity::new::<radian_per_second>(0.07);
        options.settle = Duration::from_millis(90);
    }
    Ok(options)
}
