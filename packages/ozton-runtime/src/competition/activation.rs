//! Distinguish separate activations of the same competition mode.
//! Dropping the callback invalidates its work even if a short intermediate
//! mode never gets polled. No motor writes are performed by this guard.

#[cfg(target_os = "vexos")]
use std::sync::atomic::{AtomicU32, Ordering};

use vexide::competition::{self, CompetitionMode};
#[cfg(target_os = "vexos")]
static ACTIVATION: AtomicU32 = AtomicU32::new(0);

#[cfg(not(target_os = "vexos"))]
std::thread_local! {
    static ACTIVATION: std::cell::Cell<u32> = const { std::cell::Cell::new(0) };
}

pub(crate) fn current() -> u32 {
    #[cfg(target_os = "vexos")]
    {
        ACTIVATION.load(Ordering::Relaxed)
    }
    #[cfg(not(target_os = "vexos"))]
    {
        ACTIVATION.get()
    }
}

pub(crate) fn advance() {
    #[cfg(target_os = "vexos")]
    ACTIVATION.fetch_add(1, Ordering::Relaxed);
    #[cfg(not(target_os = "vexos"))]
    ACTIVATION.set(current().wrapping_add(1));
}

// Keep SD I/O out of route execution, not the entire autonomous mode.
#[cfg(all(feature = "logging", target_os = "vexos"))]
static ROUTE_RUNNING: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

#[cfg(all(feature = "logging", target_os = "vexos"))]
pub(crate) fn route_running() -> bool {
    ROUTE_RUNNING.load(Ordering::Relaxed)
}

#[cfg(all(feature = "logging", target_os = "vexos"))]
pub(super) struct RouteScope;

#[cfg(all(feature = "logging", target_os = "vexos"))]
impl RouteScope {
    pub(super) fn enter() -> Self {
        ROUTE_RUNNING.store(true, Ordering::Relaxed);
        Self
    }
}

#[cfg(all(feature = "logging", target_os = "vexos"))]
impl Drop for RouteScope {
    fn drop(&mut self) {
        ROUTE_RUNNING.store(false, Ordering::Relaxed);
    }
}

/// Invalidates work synchronously when vexide drops an active mode future.
pub(super) struct Activation(u32);

impl Activation {
    pub(super) fn enter() -> Self {
        advance();
        Self(current())
    }

    /// Cancel route-owned work, while keeping the idle part of this mode scoped.
    pub(super) fn renew(&mut self) {
        advance();
        self.0 = current();
    }
}

impl Drop for Activation {
    fn drop(&mut self) {
        if current() == self.0 {
            advance();
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) struct Token {
    activation: u32,
    mode: CompetitionMode,
    connected: bool,
}

impl Token {
    pub(crate) fn capture(mode: CompetitionMode) -> Self {
        Self {
            activation: current(),
            mode,
            connected: competition::is_connected(),
        }
    }

    pub(crate) fn active(self) -> bool {
        let status = competition::status();
        current() == self.activation
            && status.mode() == self.mode
            && status.is_connected() == self.connected
    }
}

#[cfg(all(test, not(target_os = "vexos")))]
mod tests {
    use vex_sdk_mock as _;

    use super::*;

    #[test]
    fn renewed_scope_invalidates_both_route_and_idle_work() {
        let mut activation = Activation::enter();
        let route = Token::capture(competition::mode());
        activation.renew();
        assert!(!route.active());
        let idle = Token::capture(competition::mode());
        assert!(idle.active());
        drop(activation);
        assert!(!idle.active());
    }

    #[test]
    fn dropping_activation_invalidates_its_tokens_before_reentry() {
        let first = Activation::enter();
        let token = Token::capture(competition::mode());
        assert!(token.active());
        drop(first);
        assert!(!token.active());
        let second = Activation::enter();
        let replacement = Token::capture(competition::mode());
        assert!(!token.active());
        assert!(replacement.active());
        drop(second);
        assert!(!replacement.active());
    }
}
