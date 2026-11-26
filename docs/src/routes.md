# Competition routes

VEX competition control has three modes: disabled, autonomous and driver. Ozton's runtime uses vexide's scheduler and a route selector on the Brain.

## Application-owned robots

Implement `OztonCompete` for your robot and list its autonomous routines in `ROUTES`. Each `route!` entry pairs a menu label with an async method, function, or closure that takes `&mut Robot` and returns `Result<(), Robot::Error>`, where `Error` is the robot's `OztonCompete` associated type. Ozton runs the selected routine on the same robot used by driver control.

```rust,ignore
use ozton::{route, runtime::competition::RobotRoute};

// Inside impl OztonCompete for Robot
const ROUTES: &'static [RobotRoute<Self>] = &[
    route!("Forward", Self::forward),
    route!("Skills", Self::skills),
    route!("Disable", async |_| Ok(())),
];
```

Use distinct menu labels. The Brain menu displays the registered routes in order and starts on the first one. Add a no-op route explicitly to offer a Disable option. A driver-only robot can omit `ROUTES` to skip the selector. The selected route future is boxed once per autonomous activation so routines with different future types fit in the same static list.

Build and attach the UI in the async startup block passed to `ozton::run`.

```rust,ignore
use ozton::{runtime::Runtime, ui::Ui};

let ui = Ui::builder(display).build();
let runtime = Runtime::new(robot).with_ui(ui);
```

The builder uses the stock dark theme. Use `with_theme` to change its colors or `with_background` to draw behind the menu. For a custom UI, pass a selector factory to `with_ui`. The factory receives the registered choices in order and returns a `RouteSelector`. Initialize it on the first choice.

Select a route while disabled. Ozton reads the selection when autonomous begins. See [the complete first robot](getting-started.md) for context.

## Mode changes

When the mode changes, the previous callback stops being polled; Ozton-owned TaskSlot work is scoped to that activation. Code must yield to the executor. Independent detached tasks are your responsibility. A direct autonomous-to-driver transition need not clear latched motor commands; driver code should explicitly command every output it owns. A disabled activation stops outputs. Normal route completion, faults and motion cleanup may also stop motors.

Use mode_changed to reset application state without accidentally issuing an old mode's motor outputs. Do not hold RefCell borrows across await points.


## Unreachable goals

Routes are authored in advance. Motion commands report insufficient clearance,
tracking faults, cancellation or failure to settle rather than searching for
an alternative route. Propagate faults unless the application can explicitly
establish a safe next action. Do not silently retry or skip failed scoring moves.
