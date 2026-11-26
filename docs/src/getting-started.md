# Your first robot

Start with a V5 Brain, a controller and at least one motor on each side of a differential drivetrain. If you already have a vexide project that uploads successfully, keep that project.

## Add Ozton

Add the following dependency to Cargo.toml. Keep the vexide version and Rust toolchain compatible with Ozton's checked-in Cargo.lock and rust-toolchain.toml.

```toml
ozton = { git = "https://github.com/gmhs-robotics/ozton", branch = "fix/motion-convergence" }
```

For a local checkout, use a path dependency pointing to packages/ozton instead. The development branch currently uses vexide 0.9.0-beta.1.

## A complete example

The code below is included directly from the repository's [first_robot.rs](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton/examples/first_robot.rs). Change the ports, motor cartridges and direction to match your own robot. Keep the wheels off the ground for the first test.

```rust,ignore
{{#include ../../packages/ozton/examples/first_robot.rs}}
```

`Differential` owns the left and right motor groups. `Drive::arcade` mixes normalized forward and turn inputs, typically between −1 and 1. A value of 0.25 commands a quarter of the available voltage.

The async main function uses vexide's normal entry attribute and passes a robot-initialization block to `ozton::run`. Ozton handles startup failures, startup timing and optional diagnostics. Return the configured runtime with `Ok(...)`; Ozton runs competition for you. No separate initialization function or custom entry macro is required.

`Runtime` owns the robot. The `route!` entry registers the single autonomous choice in the Brain selector. Driver control reads the controller each interval and yields so the executor can run other tasks and observe mode changes.

The example's Forward route is intentionally timed. It demonstrates competition dispatch, not accurate distance control. Next, read [coordinates](coordinates.md) and [localization](localization.md).

## Build and try it

In the Ozton repository, use the checked-in toolchain and build the example:

```sh
cargo build -p ozton --example first_robot --release --target thumbv7a-vex-v5
```

Use your vexide upload workflow to upload the resulting firmware to the Brain. In your own project, keep the normal cargo-v5 or Nix upload command you already use. Select Forward while disabled, then enable autonomous with your field controller. Test driver control separately. If the robot turns when you command forward, correct motor polarity before doing anything else.
