//! Read and update localization state without depending on a drivetrain.

use glam::DVec2;
use ozton_math::{
    geometry::Pose,
    path::clearance::{Obstacles, WallContact},
};

use crate::{Fault, Feedback, Localization, field::Boundary};

/// Localization state required by field-relative motion.
pub trait LocalizationEstimate {
    fn pose(&self) -> Pose;
    fn pose_trusted(&self) -> bool;
    fn feedback(&self) -> Result<Option<Feedback>, Fault>;
    fn rectangular_half_extents(&self) -> Option<DVec2>;
    fn motion_obstacles(&self) -> Obstacles;
    fn movable_obstacles(&self) -> Obstacles {
        Obstacles::default()
    }
    fn wall_contact(&self) -> Option<WallContact>;
    fn release_wall_contact(&mut self);
    fn request_distance_reset(&mut self);
    fn cancel_distance_reset(&mut self);
}

impl<B: Boundary> LocalizationEstimate for Localization<B> {
    fn pose(&self) -> Pose {
        Localization::pose(self)
    }

    fn pose_trusted(&self) -> bool {
        Localization::pose_trusted(self)
    }

    fn feedback(&self) -> Result<Option<Feedback>, Fault> {
        Localization::feedback(self)
    }

    fn rectangular_half_extents(&self) -> Option<DVec2> {
        self.boundary().bounds()
    }

    fn motion_obstacles(&self) -> Obstacles {
        Localization::motion_obstacles(self)
    }

    fn movable_obstacles(&self) -> Obstacles {
        Localization::movable_obstacles(self)
    }

    fn wall_contact(&self) -> Option<WallContact> {
        Localization::wall_contact(self)
    }

    fn release_wall_contact(&mut self) {
        Localization::release_wall_contact(self);
    }

    fn request_distance_reset(&mut self) {
        Localization::request_distance_reset(self);
    }

    fn cancel_distance_reset(&mut self) {
        Localization::cancel_distance_reset(self);
    }
}
