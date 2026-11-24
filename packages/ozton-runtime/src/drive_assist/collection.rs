//! Assist state stored inline in installation order.

use std::time::Instant;

/// The owned assist tuple and timing state shared across borrowed drive views.
pub struct DriveAssists<A> {
    pub(super) items: A,
    pub(super) last_tick: Option<(Instant, bool)>,
}

impl<A> DriveAssists<A> {
    pub const fn new(items: A) -> Self {
        Self {
            items,
            last_tick: None,
        }
    }
}

/// Invalidate any assist state when drive output is stopped externally.
pub trait ResetDriveAssists {
    fn reset_drive_assists(&mut self);
}

impl ResetDriveAssists for () {
    fn reset_drive_assists(&mut self) {}
}

impl<A> ResetDriveAssists for DriveAssists<A> {
    fn reset_drive_assists(&mut self) {
        self.last_tick = None;
    }
}
