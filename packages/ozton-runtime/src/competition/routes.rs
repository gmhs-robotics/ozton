//! Register and dispatch typed robot callbacks.

use std::{future::Future, pin::Pin};

#[doc(hidden)]
pub trait IntoRouteResult<E> {
    fn into_route_result(self) -> Result<(), E>;
}

impl<E> IntoRouteResult<E> for () {
    fn into_route_result(self) -> Result<(), E> {
        Ok(())
    }
}

impl<E, F: Into<E>> IntoRouteResult<E> for Result<(), F> {
    fn into_route_result(self) -> Result<(), E> {
        self.map_err(Into::into)
    }
}

type RouteHandler<R, E> =
    for<'a> fn(&'a mut R) -> Pin<Box<dyn Future<Output = Result<(), E>> + 'a>>;

/// One named async routine on an application robot.
pub struct RobotRoute<R, E> {
    label: &'static str,
    handler: Option<RouteHandler<R, E>>,
}

impl<R, E> RobotRoute<R, E> {
    #[doc(hidden)]
    pub const fn new(label: &'static str, handler: RouteHandler<R, E>) -> Self {
        Self {
            label,
            handler: Some(handler),
        }
    }

    /// A selectable driver tool, such as a supervised calibration trial.
    pub const fn driver_only(label: &'static str) -> Self {
        Self {
            label,
            handler: None,
        }
    }
}

impl<R, E> Copy for RobotRoute<R, E> {}
impl<R, E> Clone for RobotRoute<R, E> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<R, E> RobotRoute<R, E> {
    pub const fn label(self) -> &'static str {
        self.label
    }

    pub(super) async fn run(self, robot: &mut R) -> Result<(), E> {
        if let Some(handler) = self.handler {
            handler(robot).await
        } else {
            Ok(())
        }
    }

    pub const fn allows_autonomous(self) -> bool {
        self.handler.is_some()
    }
}

/// Name an async routine: `route!("Label", Self::method)`.
#[macro_export]
macro_rules! route {
    (for $robot:ty, $label:expr, $handler:path $(,)?) => {
        $crate::competition::RobotRoute::<$robot, _>::new($label, |robot| {
            ::std::boxed::Box::pin(async move {
                $crate::competition::IntoRouteResult::into_route_result($handler(robot).await)
            })
        })
    };
    ($label:expr, $handler:path $(,)?) => {
        $crate::competition::RobotRoute::new($label, |robot| {
            ::std::boxed::Box::pin(async move {
                $crate::competition::IntoRouteResult::into_route_result($handler(robot).await)
            })
        })
    };
}
