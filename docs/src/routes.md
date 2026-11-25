# Competition routes

VEX competition control has three modes: disabled, autonomous and driver. Ozton's runtime uses vexide's scheduler and a route selector on the Brain.

## Application-owned robots

Implement OztonCompete for your robot struct. Register each routine with route!("Label", Self::function) in the ROUTES constant. Each function takes &mut self and returns a Result with the same error type as your driver callback.

```rust
const ROUTES: &'static [RobotRoute<Self, Self::Error>] = &[
    route!("Forward", Self::forward),
];
```

Return `robot.with_brain(display)` from the async startup block passed to `ozton::run`. For a custom UI use `Runtime::new(robot).with_ui(Ui::custom(...))`. Select a route while disabled. The selection is read when autonomous begins. See [the complete first robot](getting-started.md) for context.

## Mode changes

When the mode changes, the previous callback stops being polled; Ozton-owned TaskSlot work is scoped to that activation. Code must yield to the executor. Independent detached tasks are your responsibility. A direct autonomous-to-driver transition need not clear latched motor commands; driver code should explicitly command every output it owns. A disabled activation stops outputs. Normal route completion, faults and motion cleanup may also stop motors.

Use mode_changed to reset application state without accidentally issuing an old mode's motor outputs. Do not hold RefCell borrows across await points.


## Unreachable goals

Routes are authored in advance. Motion commands report insufficient clearance,
tracking faults, cancellation or failure to settle rather than searching for
an alternative route. Propagate faults unless the application can explicitly
establish a safe next action. Do not silently retry or skip failed scoring moves.
