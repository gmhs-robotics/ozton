//! Clearance-aware point and heading commands.

use std::time::Duration;

use ozton_math::{
    geometry::{DVec2, Pose, bearing},
    path::clearance::{RectangularClearance, approach_heading},
    uom::si::f64::Angle,
};
use ozton_runtime::clock::CONTROL_PERIOD;

use super::{PoseDrive, Seeking, TravelDirection, error::PoseMotionError, session::PoseSession};
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
    ozton_runtime::log_row!(
        Debug,
        "POINT_REQUEST",
        "x_m={:.5},y_m={:.5},heading_rad={:.5}",
        target.position.x,
        target.position.y,
        target.heading.value
    );
    if !target.is_finite() || settings.config.timing.stage_timeout < CONTROL_PERIOD {
        return Err(PoseMotionError::Configuration);
    }

    let clearance = clearance(drive, settings.config.safety.clearance)?;
    let options = settings
        .options(drive)
        .ok_or(PoseMotionError::Configuration)?;
    if final_heading.is_some() && clearance.is_some_and(|field| !field.contains_pose(target)) {
        ozton_runtime::log_event!(
            Error,
            "path",
            "Target ({:.3}, {:.3}) m lacks clearance to turn",
            target.position.x,
            target.position.y
        );
        return Err(PoseMotionError::InsufficientClearance);
    }
    let mut session = PoseSession::new(drive, options, settings.effective_config())?;
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
    let config = settings.effective_config();
    let start = session.drive.pose();
    // Prefer a single continuous maneuver, including a clearance-aware wall departure.
    // A planned trajectory failure is not silently retried as a different command.
    if let Some(heading) = final_heading {
        let target = Pose::new(target.position, heading);
        let direction = settings
            .travel
            .map(|d| matches!(d, TravelDirection::Backward));
        let span = ozton_runtime::profile::Span::new(ozton_runtime::profile::Stage::Planner);
        let plan = ozton_math::path::planner::plan(
            start,
            target,
            config.path_limits(options.track_width),
            clearance,
            direction,
        );
        drop(span);
        if let Some(plan) = plan {
            return session
                .follow(
                    &plan.path,
                    config.follower(),
                    plan.reverse,
                    target,
                    clearance,
                    settings.config.timing.stage_timeout,
                )
                .await;
        }
    }
    // Fall back only when no feasible continuous candidate was generated.
    if let Some(field) = clearance {
        if final_heading.is_some() && !field.can_turn(target) {
            return Err(PoseMotionError::InsufficientClearance);
        }
        depart_wall(
            session,
            field,
            target,
            settings.config.timing.stage_timeout,
            settings.effective_config(),
        )
        .await?;
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
                bearing(displacement) + ozton_math::qty!((if reverse { 180.0 } else { 0.0 }) deg);
            // A point-only target does not require enough space to rotate at
            // the destination; only the actual approach orientation must fit.
            if clearance.is_some_and(|field| {
                !field.contains_translation(Pose::new(start.position, heading), target.position)
            }) {
                return Err(PoseMotionError::InsufficientClearance);
            }
            turn_checked(session, clearance, heading, settings.config.timing.stage_timeout).await?;
        }
        session
            .run(
                DifferentialTarget::DriveToPoint {
                    target: target.position,
                    maximum_speed: options.maximum_wheel_speed,
                    reverse,
                },
                settings.config.timing.stage_timeout,
            )
            .await?;
        if let Some(heading) = final_heading {
            turn_checked(session, clearance, heading, settings.config.timing.stage_timeout).await?;
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
    config: crate::MotionConfig,
) -> MotionResult<D> {
    let start = session.drive.pose();
    if field.can_turn(start) {
        return Ok(());
    }

    let options = session.options;
    let reserve = options.position_tolerance + config.safety.departure_reserve;
    let speed = config.safety.departure_speed.min(options.maximum_wheel_speed);
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
    for (min, max) in drive.motion_obstacles().into_iter().flatten() {
        field = field
            .with_obstacle(min, max)
            .ok_or(PoseMotionError::Configuration)?;
    }
    Ok(Some(field))
}
