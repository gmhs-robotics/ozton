//! Register and dispatch typed robot callbacks.

use std::{future::Future, pin::Pin};

/// The available choices, shared by the Brain selector and route dispatch.
pub trait RouteSet: Copy + 'static {
    fn label(self) -> &'static str;
}

/// Dispatch the selected async routine.
#[allow(async_fn_in_trait)]
pub trait Routes<R, E>: RouteSet {
    async fn run(self, robot: &mut R) -> Result<(), E>;
    fn allows_autonomous(self) -> bool {
        true
    }
}

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

impl<R: 'static, E: 'static> RouteSet for RobotRoute<R, E> {
    fn label(self) -> &'static str {
        self.label
    }
}

impl<R: 'static, E: 'static> Routes<R, E> for RobotRoute<R, E> {
    async fn run(self, robot: &mut R) -> Result<(), E> {
        if let Some(handler) = self.handler {
            handler(robot).await
        } else {
            Ok(())
        }
    }

    fn allows_autonomous(self) -> bool {
        self.handler.is_some()
    }
}

/// A registered route on the robot assembled by the builder.
pub struct RegisteredRoute<L: crate::RobotLayout, E>(RobotRoute<crate::CompetitionBot<L>, E>);

impl<L: crate::RobotLayout, E> RegisteredRoute<L, E> {
    #[doc(hidden)]
    pub const fn new(
        label: &'static str,
        handler: RouteHandler<crate::CompetitionBot<L>, E>,
    ) -> Self {
        Self(RobotRoute::new(label, handler))
    }

    /// Add a driver-only choice to the selector.
    pub const fn driver_only(label: &'static str) -> Self {
        Self(RobotRoute::driver_only(label))
    }
}

impl<L: crate::RobotLayout, E> Copy for RegisteredRoute<L, E> {}
impl<L: crate::RobotLayout, E> Clone for RegisteredRoute<L, E> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<L: crate::RobotLayout + 'static, E: 'static> RouteSet for RegisteredRoute<L, E> {
    fn label(self) -> &'static str {
        self.0.label
    }
}

impl<L: crate::RobotLayout + 'static, E: 'static> Routes<crate::CompetitionBot<L>, E>
    for RegisteredRoute<L, E>
{
    async fn run(self, robot: &mut crate::CompetitionBot<L>) -> Result<(), E> {
        self.0.run(robot).await
    }

    fn allows_autonomous(self) -> bool {
        self.0.handler.is_some()
    }
}

/// Name an async routine. Use `route!(for RobotType, "Label", RobotType::method)`
/// for an application-owned robot and `route!("Label", function)` with the builder.
#[macro_export]
macro_rules! route {
    (for $robot:ty, $label:expr, $handler:path $(,)?) => {
        $crate::route!(@register RobotRoute::<$robot, _>, $label, $handler)
    };
    ($label:expr, $handler:path $(,)?) => {
        $crate::route!(@register RegisteredRoute, $label, $handler)
    };
    (@register $route:ident $(::<$robot:ty, _>)?, $label:expr, $handler:path) => {
        $crate::competition::$route$(::<$robot, _>)?::new($label, |robot| {
            ::std::boxed::Box::pin(async move {
                $crate::competition::IntoRouteResult::into_route_result($handler(robot).await)
            })
        })
    };
}
