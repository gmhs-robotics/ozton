//! Ordered drive assists selected at assembly time.

use std::time::Duration;

use ozton_tracking::sources::vex::Orientation;

use super::DriveAssist;
use crate::model::DrivetrainModel;

/// An ordered collection of assists with a type independent of its contents.
pub struct DriveAssists<M: DrivetrainModel>(Vec<Box<dyn DriveAssist<M>>>);

impl<M: DrivetrainModel> Default for DriveAssists<M> {
    fn default() -> Self {
        Self::new()
    }
}

impl<M: DrivetrainModel> DriveAssists<M> {
    pub fn new() -> Self {
        Self(Vec::new())
    }

    /// Append an assist to the collection.
    pub fn with(mut self, assist: impl DriveAssist<M> + 'static) -> Self {
        self.0.push(Box::new(assist));
        self
    }
}

impl<M: DrivetrainModel> DriveAssist<M> for DriveAssists<M> {
    fn needs_orientation(&self) -> bool {
        self.0.iter().any(|assist| assist.needs_orientation())
    }

    fn apply(&mut self, axes: &mut [f64; 2], orientation: Option<Orientation>, dt: Duration) {
        for assist in &mut self.0 {
            assist.apply(axes, orientation, dt);
        }
    }

    fn wheels(&mut self, model: &M, wheels: &mut [f64; 2]) -> Result<(), M::Error> {
        let mut result = Ok(());
        for assist in &mut self.0 {
            result = result.and(assist.wheels(model, wheels));
        }
        result
    }

    fn reset(&mut self) {
        for assist in &mut self.0 {
            assist.reset();
        }
    }
}
