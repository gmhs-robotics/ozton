//! Least-squares body motion from projected wheel travel.
use glam::{DMat2, DMat3, DVec2, DVec3};
use uom::si::{angle::radian, f64::Angle};

use crate::geometry::BodyDelta;
const RANK_TOLERANCE: f64 = 1e-9;

/// Solve observations in the right/forward frame with an optional known yaw.
pub fn fit_wheel_motion(
    observations: impl Iterator<Item = (DVec3, f64)> + Clone,
    yaw: Option<Angle>,
    partial: bool,
) -> Option<BodyDelta> {
    let yaw_scale = observations
        .clone()
        .map(|(row, _)| row.z.abs())
        .fold(0.0, f64::max);
    let yaw_scale = if yaw_scale > 0.0 { yaw_scale } else { 1.0 };
    let mut normal = DMat3::ZERO;
    let mut rhs = DVec3::ZERO;
    for (mut row, mut travel) in observations {
        if let Some(yaw) = yaw {
            travel -= row.z * yaw.value;
            row.z = 0.0;
        } else {
            row.z /= yaw_scale;
        }
        normal.x_axis += row * row.x;
        normal.y_axis += row * row.y;
        normal.z_axis += row * row.z;
        rhs += row * travel;
    }

    let [xx, yy, zz] = [normal.x_axis.x, normal.y_axis.y, normal.z_axis.z];
    let translation = DMat2::from_cols(normal.x_axis.truncate(), normal.y_axis.truncate());
    if let Some(heading) = yaw {
        if translation.determinant() > RANK_TOLERANCE * xx * yy {
            return Some(BodyDelta {
                translation: translation.inverse() * rhs.truncate(),
                heading,
            });
        }
    } else if normal.determinant() > RANK_TOLERANCE * xx * yy * zz {
        let result = normal.inverse() * rhs;
        return Some(BodyDelta {
            translation: result.truncate(),
            heading: Angle::new::<radian>(result.z / yaw_scale),
        });
    }

    if !partial {
        return None;
    }
    let heading = if let Some(heading) = yaw {
        heading.value
    } else if translation.determinant() <= RANK_TOLERANCE * xx * yy {
        // For parallel wheels, a difference in lever arms can still resolve yaw.
        let (along, cross, travel) = if xx >= yy {
            (xx, normal.z_axis.x, rhs.x)
        } else {
            (yy, normal.z_axis.y, rhs.y)
        };
        let determinant = along * zz - cross * cross;
        if determinant > RANK_TOLERANCE * along * zz {
            (along * rhs.z - cross * travel) / determinant / yaw_scale
        } else {
            0.0
        }
    } else {
        0.0
    };

    let rhs = rhs.truncate() - normal.z_axis.truncate() * (heading * yaw_scale);
    let planar = if translation.determinant() > RANK_TOLERANCE * xx * yy {
        translation.inverse() * rhs
    } else {
        // The pseudoinverse of a rank-one normal matrix preserves the
        // measured component and contributes zero on the unobserved axis.
        let trace = xx + yy;
        if trace > RANK_TOLERANCE {
            rhs / trace
        } else {
            DVec2::ZERO
        }
    };
    (planar.is_finite() && heading.is_finite()).then_some(BodyDelta {
        translation: planar,
        heading: Angle::new::<radian>(heading),
    })
}
