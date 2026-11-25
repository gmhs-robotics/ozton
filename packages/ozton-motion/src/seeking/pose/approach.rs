//! Clearance-aware point and heading commands.

use std::time::Duration;

use ozton_math::{
    geometry::{DVec2, Pose, bearing, wrap_heading},
    path::clearance::RectangularClearance,
    uom::si::f64::Angle,
};

use super::{PoseDrive, Seeking, TravelDirection, error::PoseMotionError, session::PoseSession};
use crate::basic::differential::DifferentialTarget;

type MotionResult<D, T = ()> = Result<T, PoseMotionError<<D as PoseDrive>::Error>>;

/// Direct go-to-goal feedback. A pose adds a final angular PID stage;
/// neither primitive allocates or invokes a geometric planner.
pub(super) async fn drive_to_target<D: PoseDrive>(
    drive: &mut D,
    position: DVec2,
    final_heading: Option<Angle>,
    settings: Seeking,
) -> MotionResult<D> {
    if !position.is_finite() || final_heading.is_some_and(|h| !h.value.is_finite()) {
        return Err(PoseMotionError::Configuration);
    }
    let field = clearance(drive, settings.config.safety.clearance)?;
    let options = settings
        .options(drive)
        .ok_or(PoseMotionError::Configuration)?;
    let mut session = PoseSession::new(drive, options, settings.config())?;
    let result = direct(&mut session, field, position, final_heading, settings).await;
    session.finish(result)
}

async fn direct<D: PoseDrive>(
    session: &mut PoseSession<'_, D>,
    field: Option<RectangularClearance>,
    position: DVec2,
    final_heading: Option<Angle>,
    settings: Seeking,
) -> MotionResult<D> {
    let options = session.options;
    let timeout = settings.config.timing.stage_timeout;
    let start = session.drive.pose();
    let displacement = position - start.position;
    let reverse = settings.travel.map_or_else(
        || {
            wrap_heading(bearing(displacement) - start.heading)
                .value
                .abs()
                > core::f64::consts::FRAC_PI_2
        },
        |direction| direction == TravelDirection::Backward,
    );
    if displacement.length() > options.position_tolerance.value {
        let heading = wrap_heading(
            bearing(displacement) + ozton_math::qty!((if reverse { 180.0 } else { 0.0 }) deg),
        );
        let aligned = Pose::new(start.position, heading);
        // Check the entire nominal rotate/translate/rotate maneuver before
        // commanding movement. Live wall feedback remains authoritative.
        if field.is_some_and(|f| {
            !f.contains_turn(start, heading)
                || !f.contains_translation(aligned, position)
                || final_heading
                    .is_some_and(|end| !f.contains_turn(Pose::new(position, heading), end))
        }) {
            return Err(PoseMotionError::InsufficientClearance);
        }
        if wrap_heading(heading - start.heading).value.abs() > options.heading_tolerance.value {
            turn_checked(session, field, heading, timeout).await?;
        }
        session
            .run(
                DifferentialTarget::DriveToPoint {
                    target: position,
                    maximum_speed: options.maximum_wheel_speed,
                    reverse,
                },
                timeout,
            )
            .await?;
    }
    if let Some(heading) = final_heading {
        if wrap_heading(heading - session.drive.pose().heading)
            .value
            .abs()
            > options.heading_tolerance.value
        {
            turn_checked(session, field, heading, timeout).await?;
        }
    }
    if session.drive.pose().position.distance(position) <= options.position_tolerance.value {
        Ok(())
    } else {
        Err(PoseMotionError::PositionUnsettled)
    }
}

async fn turn_checked<D: PoseDrive>(
    session: &mut PoseSession<'_, D>,
    clearance: Option<RectangularClearance>,
    heading: Angle,
    timeout: Duration,
) -> MotionResult<D> {
    if clearance.is_some_and(|field| !field.contains_turn(session.drive.pose(), heading)) {
        return Err(PoseMotionError::InsufficientClearance);
    }
    session
        .run(DifferentialTarget::Turn { heading }, timeout)
        .await
}

pub(super) fn clearance<D: PoseDrive>(
    drive: &D,
    margin: ozton_math::uom::si::f64::Length,
) -> MotionResult<D, Option<RectangularClearance>> {
    let Some(footprint) = drive.footprint() else {
        return Ok(None);
    };
    let bounds = drive
        .rectangular_half_extents()
        .ok_or(PoseMotionError::Configuration)?;
    let mut field = RectangularClearance::new(footprint, bounds, margin)
        .ok_or(PoseMotionError::Configuration)?;
    for polygon in drive.motion_obstacles().into_iter().flatten() {
        field = field
            .with_polygon(polygon)
            .ok_or(PoseMotionError::Configuration)?;
    }
    Ok(Some(field))
}
