//! Direct point and heading feedback commands.

use std::time::Duration;

use ozton_localization::LocalizationEstimate;
use ozton_math::{
    geometry::{DVec2, bearing, wrap_heading},
    path::clearance::RectangularClearance,
    uom::si::f64::Angle,
};

use super::{
    MotionExecution, Engine, TravelDirection, error::PoseMotionError, session::PoseSession,
};
use crate::basic::differential::DifferentialTarget;

type MotionResult<D, T = ()> = Result<T, PoseMotionError<<D as MotionExecution>::Error>>;

/// Direct go-to-goal feedback. A pose adds a final angular PID stage;
/// neither primitive allocates or invokes a geometric planner.
pub(super) async fn drive_to_target<D: MotionExecution>(
    drive: &mut D,
    position: DVec2,
    final_heading: Option<Angle>,
    settings: Engine,
) -> MotionResult<D> {
    if !position.is_finite() || final_heading.is_some_and(|h| !h.value.is_finite()) {
        return Err(PoseMotionError::Configuration);
    }
    let options = settings
        .options(drive)
        .ok_or(PoseMotionError::Configuration)?;
    let mut session = PoseSession::new(drive, options, settings.config())?;
    let result = if final_heading.is_some() {
        direct(&mut session, position, final_heading, settings).await
    } else {
        let pose = session.drive.localization().pose();
        let reverse = settings.travel.map_or_else(
            || {
                wrap_heading(bearing(position - pose.position) - pose.heading)
                    .value
                    .abs()
                    > core::f64::consts::FRAC_PI_2
            },
            |direction| direction == TravelDirection::Backward,
        );
        session
            .run(
                DifferentialTarget::DriveToPoint {
                    target: position,
                    maximum_speed: options.maximum_wheel_speed,
                    reverse,
                },
                settings.config.timing.stage_timeout,
            )
            .await
    };
    session.finish(result)
}

async fn direct<D: MotionExecution>(
    session: &mut PoseSession<'_, D>,
    position: DVec2,
    final_heading: Option<Angle>,
    settings: Engine,
) -> MotionResult<D> {
    let options = session.options;
    let timeout = settings.config.timing.stage_timeout;
    loop {
        let start = session.drive.localization().pose();
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
            if wrap_heading(heading - start.heading).value.abs() > options.heading_tolerance.value {
                turn(session, heading, timeout).await?;
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
            if wrap_heading(heading - session.drive.localization().pose().heading)
                .value
                .abs()
                > options.heading_tolerance.value
            {
                turn(session, heading, timeout).await?;
            }
        }
        if session
            .drive
            .localization()
            .pose()
            .position
            .distance(position)
            <= options.position_tolerance.value
        {
            return Ok(());
        }
        ozton_diagnostics::log_event!(
            Warn,
            "motion",
            "Final turn shifted position; correcting pose"
        );
    }
}

async fn turn<D: MotionExecution>(
    session: &mut PoseSession<'_, D>,
    heading: Angle,
    timeout: Duration,
) -> MotionResult<D> {
    session
        .run(DifferentialTarget::Turn { heading }, timeout)
        .await
}

pub(super) fn clearance<D: MotionExecution>(
    drive: &D,
    margin: ozton_math::uom::si::f64::Length,
    movable: bool,
) -> MotionResult<D, Option<RectangularClearance>> {
    let Some(footprint) = drive.footprint() else {
        return Ok(None);
    };
    let bounds = drive
        .localization()
        .rectangular_half_extents()
        .ok_or(PoseMotionError::Configuration)?;
    let mut field = RectangularClearance::new(footprint, bounds, margin)
        .ok_or(PoseMotionError::Configuration)?;
    for polygon in drive
        .localization()
        .motion_obstacles()
        .into_iter()
        .flatten()
    {
        field = field
            .with_polygon(polygon)
            .ok_or(PoseMotionError::Configuration)?;
    }
    if movable {
        for polygon in drive
            .localization()
            .movable_obstacles()
            .into_iter()
            .flatten()
        {
            field = field
                .with_polygon(polygon)
                .ok_or(PoseMotionError::Configuration)?;
        }
    }
    Ok(Some(field))
}
