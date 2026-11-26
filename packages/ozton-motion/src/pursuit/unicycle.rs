//! Select the mathematical controller once per path.

use ozton_math::following::{LtvLqr, Ramsete};

use crate::PathGuidance;

#[derive(Clone, Copy)]
pub(super) enum Guidance {
    PurePursuit,
    Ramsete(Ramsete),
    Lqr(LtvLqr),
}

impl Guidance {
    pub(super) fn new(value: PathGuidance) -> Option<Self> {
        match value {
            PathGuidance::PurePursuit => Some(Self::PurePursuit),
            PathGuidance::Ramsete { b, zeta } => Ramsete::new(b, zeta).map(Self::Ramsete),
            PathGuidance::Lqr { state, input } => LtvLqr::new(state, input).map(Self::Lqr),
        }
    }
}
