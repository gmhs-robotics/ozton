# Competition routes

VEX competition control has three modes: disabled, autonomous and driver. Ozton's runtime uses vexide's scheduler and a route selector on the Brain.

## Application-owned robots

Implement `OztonCompete` for your robot. Give each autonomous choice an ID in your own route enum, register its label, and dispatch the ID in `autonomous`. Autonomous and driver callbacks borrow the same robot and use the same error type.

```rust,ignore
#[derive(Clone, Copy, PartialEq, Eq)]
enum Route {
    Forward,
    Skills,
}

// Inside impl OztonCompete for Robot
type Route = Route;
const ROUTES: &'static [RobotRoute<Route>] = &[
    RobotRoute::new("Forward", Route::Forward),
    RobotRoute::new("Skills", Route::Skills),
];
const DEFAULT_ROUTE: Option<Route> = Some(Route::Forward);

async fn autonomous(&mut self, route: Route) -> Result<(), Self::Error> {
    match route {
        Route::Forward => self.forward().await,
        Route::Skills => self.skills().await,
    }
}
```

A driver-only robot can use `std::convert::Infallible` as its route type, leave the route list empty, and write `match route {}` in its autonomous callback.

Return `robot.with_brain(display)` from the async startup block passed to `ozton::run`. For a custom UI use `Runtime::new(robot).with_ui(Ui::custom(...))`. Select a route while disabled. The selection is read when autonomous begins. See [the complete first robot](getting-started.md) for context.

## Mode changes

When the mode changes, the previous callback stops being polled; Ozton-owned TaskSlot work is scoped to that activation. Code must yield to the executor. Independent detached tasks are your responsibility. A direct autonomous-to-driver transition need not clear latched motor commands; driver code should explicitly command every output it owns. A disabled activation stops outputs. Normal route completion, faults and motion cleanup may also stop motors.

Use mode_changed to reset application state without accidentally issuing an old mode's motor outputs. Do not hold RefCell borrows across await points.


## Unreachable goals

Routes are authored in advance. Motion commands report insufficient clearance,
tracking faults, cancellation or failure to settle rather than searching for
an alternative route. Propagate faults unless the application can explicitly
establish a safe next action. Do not silently retry or skip failed scoring moves.
