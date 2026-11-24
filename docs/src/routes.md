# Competition routes

VEX competition control has three modes: disabled, autonomous and driver. Ozton's runtime uses vexide's scheduler and a route selector on the Brain.

## Application-owned robots

Implement OztonCompete for your robot struct. Register each routine with route!(for Self, "Label", Self::function) in the ROUTES constant. Each function takes &mut self and returns a Result with the same error type as your driver callback.

```rust
const ROUTES: &'static [RobotRoute<Self, Self::Error>] = &[
    route!(for Self, "Forward", Self::forward),
];
```

Pass your robot to Runtime::new, add Ui::Brain(display), then await compete. Select a route while disabled. The selection is read when autonomous begins. See [the complete first robot](getting-started.md) for context.

## Mode changes

When the mode changes, the previous callback stops being polled; Ozton-owned TaskSlot work is scoped to that activation. Code must yield to the executor. Independent detached tasks are your responsibility. A direct autonomous-to-driver transition need not clear latched motor commands; driver code should explicitly command every output it owns. A disabled activation stops outputs. Normal route completion, faults and motion cleanup may also stop motors.

Use mode_changed to reset application state without accidentally issuing an old mode's motor outputs. Do not hold RefCell borrows across await points.

## Builder alternative

RobotBuilder supports a configured competition robot, mechanism coordination, a Program and RegisteredRoute. It is useful when you want a reusable robot layout with more integrated behavior. The [programmed example](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton/examples/programmed.rs) demonstrates this style. You do not need to adopt the builder to use Ozton motion algorithms.
