//! Select the mathematical controller once per path.

use ozton_math::following::{LtvLqr, Ramsete};

use crate::PathGuidance;

#[derive(Clone, Copy)]
pub(super) enum Guidance {
    PurePursuit,
    Ramsete(Ramsete),
    Lqr(LtvLqr),
}

impl From<PathGuidance> for Guidance {
    fn from(value: PathGuidance) -> Self {
        match value {
            PathGuidance::PurePursuit => Self::PurePursuit,
            PathGuidance::Ramsete { b, zeta } => Self::Ramsete(Ramsete::new(b, zeta).unwrap()),
            PathGuidance::Lqr { state, input } => Self::Lqr(LtvLqr::new(state, input).unwrap()),
        }
    }
}
