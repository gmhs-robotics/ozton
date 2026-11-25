//! Select and prepare the reusable mathematical controller once per path.

use ozton_math::following::{LtvLqr, Ramsete};

use crate::PathGuidance;

#[derive(Clone, Copy)]
pub(super) enum Guidance {
    Tangent,
    PurePursuit,
    Ramsete(Ramsete),
    Lqr(LtvLqr),
}

impl From<PathGuidance> for Guidance {
    fn from(value: PathGuidance) -> Self {
        match value {
            PathGuidance::Tangent => Self::Tangent,
            PathGuidance::PurePursuit => Self::PurePursuit,
            PathGuidance::Ramsete { b, zeta } => Self::Ramsete(Ramsete::new(b, zeta).unwrap()),
            PathGuidance::Lqr { state, input } => Self::Lqr(LtvLqr::new(state, input).unwrap()),
        }
    }
}

impl Guidance {
    pub fn calculate(self, error: [f64; 3], reference: [f64; 2]) -> Option<[f64; 2]> {
        match self {
            Self::Ramsete(controller) => Some(controller.calculate(error, reference)),
            Self::Lqr(controller) => Some(controller.calculate(error, reference)),
            _ => None,
        }
    }
}
