//! Competition routes and lifecycle for application-owned robots.

pub use ozton_ui::Ui;
use vexide::competition::CompetitionMode;

pub(crate) mod activation;
mod routes;
mod runtime;
mod selector_task;
mod session;

pub use routes::RobotRoute;
pub use runtime::{OztonCompete, Runtime};
pub(crate) use session::while_in_mode;

pub use crate::mechanism::Mechanism;

/// Whether an autonomous motion may continue issuing commands.
pub fn autonomous_active() -> bool {
    vexide::competition::mode() == CompetitionMode::Autonomous
}
