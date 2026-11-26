use std::{future::Future, pin::Pin};

use super::OztonCompete;

type RouteHandler<R> =
    for<'a> fn(
        &'a mut R,
    ) -> Pin<Box<dyn Future<Output = Result<(), <R as OztonCompete>::Error>> + 'a>>;

/// A named async routine registered with `route!` and run by Ozton.
pub struct RobotRoute<R: OztonCompete> {
    pub label: &'static str,
    handler: RouteHandler<R>,
}

impl<R: OztonCompete> RobotRoute<R> {
    #[doc(hidden)]
    pub const fn new(label: &'static str, handler: RouteHandler<R>) -> Self {
        Self { label, handler }
    }

    pub(super) async fn run(self, robot: &mut R) -> Result<(), R::Error> {
        (self.handler)(robot).await
    }
}

impl<R: OztonCompete> Copy for RobotRoute<R> {}

impl<R: OztonCompete> Clone for RobotRoute<R> {
    fn clone(&self) -> Self {
        *self
    }
}

/// Register an async function or method, for example `route!("Skills", Self::skills)`.
/// The handler takes `&mut R` and returns `Result<(), R::Error>`.
#[macro_export]
macro_rules! route {
    ($label:expr, $handler:path $(,)?) => {
        $crate::competition::RobotRoute::new($label, |robot| {
            ::std::boxed::Box::pin($handler(robot))
        })
    };
}
