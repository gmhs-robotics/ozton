//! Distinguish separate activations of the same competition mode.
//! Vexide owns the callback future; this token also invalidates Ozton-owned
//! background futures that may otherwise miss a short intermediate mode.

use std::sync::atomic::{AtomicU32, Ordering};
use vexide::competition::{self, CompetitionMode};

static ACTIVATION: AtomicU32 = AtomicU32::new(0);

pub(crate) fn current() -> u32 {
    ACTIVATION.load(Ordering::Relaxed)
}

pub(super) fn advance() {
    ACTIVATION.fetch_add(1, Ordering::Relaxed);
}

pub(crate) fn is_current(activation: u32, mode: CompetitionMode) -> bool {
    current() == activation && competition::mode() == mode
}
