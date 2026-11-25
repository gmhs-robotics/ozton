//! Clearance-aware point and heading commands.

use std::time::{Duration, Instant};

use ozton_math::{
    geometry::{DVec2, Pose, bearing, wrap_heading},
    path::{
        clearance::RectangularClearance,
        planner::{LatticePlanner, LatticeStep, ManeuverSearch},
    },
    uom::si::f64::Angle,
};

use super::{
    PathActions, PoseDrive, Seeking, TravelDirection,
    error::PoseMotionError,
    session::{PoseSession, Trajectory},
};
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
    let options = settings.options(drive).ok_or(PoseMotionError::Configuration)?;
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
        || wrap_heading(bearing(displacement) - start.heading).value.abs()
            > core::f64::consts::FRAC_PI_2,
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
                || final_heading.is_some_and(|end| !f.contains_turn(Pose::new(position, heading), end))
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
        if wrap_heading(heading - session.drive.pose().heading).value.abs()
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

pub(super) async fn navigate_to_target<D: PoseDrive>(
    drive: &mut D,
    position: DVec2,
    final_heading: Option<Angle>,
    settings: Seeking,
) -> MotionResult<D> {
    let target = Pose::new(
        position,
        final_heading.unwrap_or_else(|| drive.pose().heading),
    );
    ozton_diagnostics::log_row!(
        Debug,
        "POINT_REQUEST",
        "x_m={:.5},y_m={:.5},heading_rad={:.5}",
        target.position.x,
        target.position.y,
        target.heading.value
    );
    if !target.is_finite() || settings.config.timing.stage_timeout < settings.config.timing.period {
        return Err(PoseMotionError::Configuration);
    }

    let clearance = clearance(drive, settings.config.safety.clearance)?;
    let options = settings
        .options(drive)
        .ok_or(PoseMotionError::Configuration)?;
    let mut session = PoseSession::new(drive, options, settings.config())?;
    if final_heading.is_some() && clearance.is_some_and(|field| !field.contains_pose(target)) {
        ozton_diagnostics::log_event!(
            Error,
            "path",
            "Target ({:.3}, {:.3}) m lacks clearance to turn",
            target.position.x,
            target.position.y
        );
        let result = session
            .await_stationary()
            .await
            .and(Err(PoseMotionError::NoRoute));
        return session.finish(result);
    }
    let result = match session.await_stationary().await {
        Ok(()) => approach(&mut session, clearance, target, final_heading, settings).await,
        Err(error) => Err(error),
    };
    session.finish(result)
}

async fn approach<D: PoseDrive>(
    session: &mut PoseSession<'_, D>,
    clearance: Option<RectangularClearance>,
    target: Pose,
    final_heading: Option<Angle>,
    settings: Seeking,
) -> MotionResult<D> {
    let options = session.options;
    let config = settings.config();
    let start = session.drive.pose();
    let movable = session.drive.movable_obstacles();
    let preferred = clearance.and_then(|mut field| {
        for polygon in movable.iter().flatten() {
            field = field.with_polygon(*polygon)?;
        }
        Some(field)
    });
    // Select a complete continuous maneuver or lattice route while stationary.
    if let Some(heading) = final_heading {
        let target = Pose::new(target.position, heading);
        let direction = settings
            .travel
            .map(|d| matches!(d, TravelDirection::Backward));
        let plan = if clearance.is_none() || preferred.is_some() {
            if let Some(mut search) = ManeuverSearch::new(
                start,
                target,
                config.path_limits(options.track_width),
                preferred,
                direction,
            ) {
                let started = Instant::now();
                let mut slice_started = started;
                loop {
                    let span = ozton_diagnostics::profile::Span::new(
                        ozton_diagnostics::profile::Stage::Planner,
                    );
                    let more = search.step();
                    drop(span);
                    if !more {
                        break;
                    }
                    // V5 localization runs on the same cooperative executor.
                    // Keep each uninterrupted planning slice short.
                    if slice_started.elapsed() >= Duration::from_millis(2) {
                        session
                            .drive
                            .wait_until(Instant::now() + Duration::from_millis(1))
                            .await;
                        if !session.drive.motion_active() {
                            return Err(PoseMotionError::Cancelled);
                        }
                        slice_started = Instant::now();
                    }
                }
                ozton_diagnostics::log_row!(
                    Debug,
                    "PLAN_SEARCH",
                    "elapsed_us={}",
                    started.elapsed().as_micros()
                );
                // Always let the tracking worker publish a new observation
                // before handing control to the trajectory follower.
                session
                    .drive
                    .wait_until(Instant::now() + config.timing.period)
                    .await;
                if !session.drive.motion_active() {
                    return Err(PoseMotionError::Cancelled);
                }
                let measured = session.drive.pose();
                if measured.position.distance(start.position) > options.position_tolerance.value
                    || wrap_heading(measured.heading - start.heading).value.abs()
                        > options.heading_tolerance.value
                {
                    None
                } else {
                    search.finish()
                }
            } else {
                None
            }
        } else {
            None
        };
        if let Some(plan) = plan {
            return session
                .follow(
                    Trajectory {
                        path: &plan.path,
                        follower: config.follower(),
                        reverse: plan.reverse,
                        target,
                        clearance,
                        timeout: config.timing.stage_timeout,
                    },
                    &mut PathActions::new(),
                )
                .await;
        }
    }
    // Fallback is permitted before any motion, never after execution fails.
    lattice_approach(session, clearance, preferred, target, final_heading, settings).await
}

async fn lattice_approach<D: PoseDrive>(
    session: &mut PoseSession<'_, D>,
    clearance: Option<RectangularClearance>,
    preferred: Option<RectangularClearance>,
    target: Pose,
    final_heading: Option<Angle>,
    settings: Seeking,
) -> MotionResult<D> {
    let config = settings.config();
    let direction = settings
        .travel
        .map(|d| matches!(d, TravelDirection::Backward));
    let planner = LatticePlanner::new();
    let options = session.options;
    let tolerance = options.position_tolerance.value;
    let start = session.drive.pose();
    let route = lattice_plan(
        session, planner, preferred, clearance, target, final_heading, direction,
    )
    .await?;
    session
        .drive
        .wait_until(Instant::now() + config.timing.period)
        .await;
    if !session.drive.motion_active() {
        return Err(PoseMotionError::Cancelled);
    }
    let measured = session.drive.pose();
    if measured.position.distance(start.position) > options.position_tolerance.value
        || wrap_heading(measured.heading - start.heading).value.abs()
            > options.heading_tolerance.value
    {
        return Err(PoseMotionError::PositionUnsettled);
    }
    for step in route.ok_or(PoseMotionError::NoRoute)? {
        match step {
            LatticeStep::Turn(heading) => {
                turn_checked(session, clearance, heading, config.timing.stage_timeout).await?;
            }
            LatticeStep::Translate {
                target: point,
                reverse,
            } => {
                let start = session.drive.pose();
                let displacement = point - start.position;
                if displacement.length() <= tolerance {
                    continue;
                }
                let heading = wrap_heading(
                    bearing(displacement)
                        + ozton_math::qty!((if reverse { 180.0 } else { 0.0 }) deg),
                );
                if wrap_heading(heading - start.heading).value.abs()
                    > options.heading_tolerance.value
                {
                    turn_checked(session, clearance, heading, config.timing.stage_timeout).await?;
                }
                let aligned = session.drive.pose();
                if clearance.is_some_and(|field| !field.contains_translation(aligned, point)) {
                    return Err(PoseMotionError::InsufficientClearance);
                }
                session
                    .run(
                        DifferentialTarget::DriveToPoint {
                            target: point,
                            maximum_speed: options.maximum_wheel_speed,
                            reverse,
                        },
                        config.timing.stage_timeout,
                    )
                    .await?;
            }
        }
    }
    if let Some(heading) = final_heading {
        if wrap_heading(heading - session.drive.pose().heading)
            .value
            .abs()
            > options.heading_tolerance.value
        {
            turn_checked(session, clearance, heading, config.timing.stage_timeout).await?;
        }
    }

    let error = session.drive.pose().position.distance(target.position);
    if error <= tolerance {
        Ok(())
    } else {
        ozton_diagnostics::log_event!(
            Warn,
            "path",
            "Position error after approach: {:.3} m",
            error
        );
        Err(PoseMotionError::PositionUnsettled)
    }
}

#[allow(clippy::too_many_arguments)]
async fn lattice_plan<D: PoseDrive>(
    session: &mut PoseSession<'_, D>,
    planner: LatticePlanner,
    preferred: Option<RectangularClearance>,
    clearance: Option<RectangularClearance>,
    target: Pose,
    heading: Option<Angle>,
    direction: Option<bool>,
) -> MotionResult<D, Option<Vec<LatticeStep>>> {
    let start = session.drive.pose();
    let track_width = session.options.track_width;
    let drive = &mut *session.drive;
    let mut slice_started = Instant::now();
    for field in [preferred.as_ref(), clearance.as_ref()] {
        if field.is_none() && clearance.is_some() {
            continue;
        }
        let route = planner
            .plan_cooperative(
                field,
                start,
                target.position,
                heading,
                direction,
                track_width,
                async || {
                    if slice_started.elapsed() >= Duration::from_millis(2) {
                        drive.wait_until(Instant::now() + Duration::from_millis(1)).await;
                        slice_started = Instant::now();
                    }
                    drive.motion_active()
                },
            )
            .await;
        if !drive.motion_active() {
            return Err(PoseMotionError::Cancelled);
        }
        if route.is_some() {
            return Ok(route);
        }
        if preferred.is_none() {
            break;
        }
    }
    Ok(None)
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
