//! Clearance-aware point and heading commands.

use std::time::Duration;

use ozton_math::{
    geometry::{DVec2, Pose, bearing, wrap_heading},
    path::{
        clearance::RectangularClearance,
        planner::{LatticePlanner, LatticeStep},
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

pub(super) async fn drive_to_target<D: PoseDrive>(
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
            .prepare_replan()
            .await
            .and(Err(PoseMotionError::NoRoute));
        return session.finish(result);
    }
    let result = approach(&mut session, clearance, target, final_heading, settings).await;
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
    // Prefer a continuous maneuver that avoids fixed and movable objects.
    // If a feasible continuous approach is blocked during execution, stop
    // safely and replan using the orientation-aware lattice from the live pose.
    if let Some(heading) = final_heading {
        let target = Pose::new(target.position, heading);
        let direction = settings
            .travel
            .map(|d| matches!(d, TravelDirection::Backward));
        let span =
            ozton_diagnostics::profile::Span::new(ozton_diagnostics::profile::Stage::Planner);
        let plan = if clearance.is_none() || preferred.is_some() {
            ozton_math::path::planner::plan(
                start,
                target,
                config.path_limits(options.track_width),
                preferred,
                direction,
            )
        } else {
            None
        };
        drop(span);
        if let Some(plan) = plan {
            let result = session
                .follow(
                    Trajectory {
                        path: &plan.path,
                        follower: config.follower(),
                        reverse: plan.reverse,
                        target,
                        clearance,
                        timeout: settings.config.timing.stage_timeout,
                    },
                    &mut PathActions::new(),
                )
                .await;
            match result {
                Ok(()) => return Ok(()),
                Err(error) if replannable(&error) => session.prepare_replan().await?,
                Err(error) => return Err(error),
            }
        }
    }
    // A bounded A* search represents orientation explicitly. Prefer a route
    // avoiding movable objects, then relax only their planning cost.
    let tolerance = options.position_tolerance.value;
    // A new plan is bounded by the existing progress deadline and must start
    // from verified rest. Never retry hardware, cancellation or tracking faults.
    let replan_started = std::time::Instant::now();
    loop {
        let before = session.drive.pose();
        let result = lattice_approach(
            session,
            clearance,
            preferred,
            target,
            final_heading,
            settings,
        )
        .await;
        match result {
            Ok(()) => return Ok(()),
            Err(error) if replannable(&error) => {
                session.prepare_replan().await?;
                let after = session.drive.pose();
                // No changed state means the deterministic planner would
                // return the same route. Do not loop indefinitely.
                if (after.position.distance(before.position) <= tolerance
                    && wrap_heading(after.heading - before.heading).value.abs()
                        <= options.heading_tolerance.value)
                    || replan_started.elapsed() >= config.timing.stage_timeout
                {
                    return Err(PoseMotionError::NoRoute);
                }
            }
            Err(error) => return Err(error),
        }
    }
}

fn replannable<E>(error: &PoseMotionError<E>) -> bool {
    matches!(
        error,
        PoseMotionError::InsufficientClearance
            | PoseMotionError::PositionUnsettled
            | PoseMotionError::NoProgress
    )
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
    let find = |start| {
        let plan = |field| {
            planner.plan(
                field,
                start,
                target.position,
                final_heading,
                direction,
                options.track_width,
            )
        };
        preferred
            .as_ref()
            .and_then(|field| plan(Some(field)))
            .or_else(|| plan(clearance.as_ref()))
    };
    let mut route = find(session.drive.pose());
    if route.is_none() {
        if let Some(field) = clearance.filter(|field| !field.can_turn(session.drive.pose())) {
            depart_wall(session, field, target, config.timing.stage_timeout, config).await?;
            route = find(session.drive.pose());
        }
    }
    for step in route.ok_or(PoseMotionError::InsufficientClearance)? {
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
                if clearance.is_some_and(|field| {
                    !field.contains_translation(Pose::new(start.position, heading), point)
                }) {
                    return Err(PoseMotionError::InsufficientClearance);
                }
                if wrap_heading(heading - start.heading).value.abs()
                    > options.heading_tolerance.value
                {
                    turn_checked(session, clearance, heading, config.timing.stage_timeout).await?;
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

async fn depart_wall<D: PoseDrive>(
    session: &mut PoseSession<'_, D>,
    field: RectangularClearance,
    target: Pose,
    timeout: Duration,
    config: crate::MotionConfig,
) -> MotionResult<D> {
    let start = session.drive.pose();
    if field.can_turn(start) {
        return Ok(());
    }

    let options = session.options;
    let reserve = options.position_tolerance + config.safety.departure_reserve;
    let speed = config
        .safety
        .departure_speed
        .min(options.maximum_wheel_speed);
    let plan = field
        .point_departure(start, target, options.track_width / 2.0, reserve)
        .ok_or(PoseMotionError::InsufficientClearance)?;
    ozton_diagnostics::log_row!(
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
