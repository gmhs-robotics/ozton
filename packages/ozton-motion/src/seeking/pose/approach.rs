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

use super::{
    PoseDrive, PoseSeeking, TravelDirection, error::PoseMotionError, session::PoseSession,
};
use crate::{basic::differential::DifferentialTarget, defaults::AUTON_SPEED};

type MotionResult<D, T = ()> = Result<T, PoseMotionError<<D as PoseDrive>::Error>>;

const DEPARTURE_SPEED: Velocity = Velocity {
    value: 0.30,
    ..AUTON_SPEED
};
pub(super) async fn drive_to_target<D: PoseDrive>(
    drive: &mut D,
    position: DVec2,
    final_heading: Option<Angle>,
    settings: PoseSeeking,
) -> MotionResult<D> {
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
        return Err(PoseMotionError::Configuration);
    }

    let clearance = clearance(drive)?;
    let options = settings
        .options(drive)
        .ok_or(PoseMotionError::Configuration)?;
    if clearance.is_some_and(|field| !field.can_turn(target)) {
        ozton_runtime::log_event!(
            Error,
            "path",
            "Target ({:.3}, {:.3}) m lacks clearance to turn",
            target.position.x,
            target.position.y
        );
        return Err(PoseMotionError::InsufficientClearance);
    }
    let mut session = PoseSession::new(drive, options, settings.tuning, settings.budget)?;
    let result = approach(&mut session, clearance, target, final_heading, settings).await;
    session.finish(result)
}

async fn approach<D: PoseDrive>(
    session: &mut PoseSession<'_, D>,
    clearance: Option<RectangularClearance>,
    target: Pose,
    final_heading: Option<Angle>,
    settings: PoseSeeking,
) -> MotionResult<D> {
    let options = session.options;
    if let Some(field) = clearance {
        depart_wall(session, field, target, settings.stage_timeout).await?;
    }
    let tolerance = options.position_tolerance.value;
    for _ in 0..2 {
        let start = session.drive.pose();
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
            turn_checked(session, clearance, heading, settings.stage_timeout).await?;
        }
        session
            .run(
                DifferentialTarget::DriveToPoint {
                    target: target.position,
                    maximum_speed: options.maximum_wheel_speed,
                    reverse,
                },
                settings.stage_timeout,
            )
            .await?;
        if let Some(heading) = final_heading {
            turn_checked(session, clearance, heading, settings.stage_timeout).await?;
        }
        let error = session.drive.pose().position.distance(target.position);
        if error <= tolerance {
            return Ok(());
        }
        ozton_runtime::log_event!(
            Warn,
            "path",
            "Position error after approach: {:.3} m",
            error
        );
    }
    Err(PoseMotionError::PositionUnsettled)
}

async fn depart_wall<D: PoseDrive>(
    session: &mut PoseSession<'_, D>,
    field: RectangularClearance,
    target: Pose,
    timeout: Duration,
) -> MotionResult<D> {
    let start = session.drive.pose();
    if field.can_turn(start) {
        return Ok(());
    }

    let options = session.options;
    let reserve = options.position_tolerance + Length::new::<meter>(0.00635);
    let speed = DEPARTURE_SPEED.min(options.maximum_wheel_speed);
    let plan = field
        .point_departure(start, target, options.track_width / 2.0, reserve)
        .ok_or(PoseMotionError::InsufficientClearance)?;
    ozton_runtime::log_row!(
        Debug,
        "CORNER_DEPARTURE",
        "radius_m={:.5},sweep_rad={:.5},advance_m={:.5}",
        plan.arc.map_or(0.0, |arc| arc.radius.value),
        plan.arc.map_or(0.0, |arc| arc.sweep.value),
        plan.advance.value
    );
    if let Some(arc) = plan.arc {
        session
            .run(
                DifferentialTarget::DriveArc {
                    radius: arc.radius,
                    sweep: arc.sweep,
                    maximum_speed: speed,
                },
                timeout,
            )
            .await?;
    }

    let measured = session.drive.pose();
    if !field.can_turn(measured) {
        let advance = field
            .forward_to_turn(measured, reserve)
            .ok_or(PoseMotionError::InsufficientClearance)?;
        session
            .run(
                DifferentialTarget::DriveDistance {
                    meters: advance,
                    heading: Some(measured.heading),
                    maximum_speed: speed,
                },
                timeout,
            )
            .await?;
    }
    field
        .can_turn(session.drive.pose())
        .then_some(())
        .ok_or(PoseMotionError::InsufficientClearance)
}

async fn turn_checked<D: PoseDrive>(
    session: &mut PoseSession<'_, D>,
    clearance: Option<RectangularClearance>,
    heading: Angle,
    timeout: Duration,
) -> MotionResult<D> {
    if clearance.is_some_and(|field| !field.can_turn(session.drive.pose())) {
        return Err(PoseMotionError::InsufficientClearance);
    }
    session
        .run(DifferentialTarget::Turn { heading }, timeout)
        .await
}

fn clearance<D: PoseDrive>(drive: &D) -> MotionResult<D, Option<RectangularClearance>> {
    let Some(footprint) = drive.footprint() else {
        return Ok(None);
    };
    let bounds = drive
        .rectangular_half_extents()
        .ok_or(PoseMotionError::Configuration)?;
    RectangularClearance::new(footprint, bounds, Length::new::<meter>(0.0127))
        .map(Some)
        .ok_or(PoseMotionError::Configuration)
}
