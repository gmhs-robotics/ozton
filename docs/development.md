# Build and troubleshoot

Run these commands from the Ozton repository with its pinned Rust nightly and
`rust-src`.

```sh
cargo check --workspace --all-targets
cargo test --workspace --all-targets
cargo test -p ozton --all-targets --no-default-features --features recording
cargo build -p ozton --example programmed --target thumbv7a-vex-v5
cargo build -p ozton --examples --target thumbv7a-vex-v5 --no-default-features --features recording
```

To render this book, install mdBook and run:

```sh
mdbook build
mdbook serve --open
```

The HTML build goes to `target/book`. This manual uses live source includes for
its full examples.

| Symptom | Check |
| --- | --- |
| Build says to choose one mode | Disable default features when selecting `recording`. |
| Route rejected at startup | Check the reported route name, export format, and physical limits. |
| Route starts with `PathStart` | Check the declared start position, field convention and heading for the travel direction. |
| Pose jumps or stops updating | Check encoder direction, IMU calibration, sensor health and localizer rejection logs. |
| Alignment never settles | Verify sensor orientation, clear line of sight, the selected range and the command deadline. |
| Calibration tool reports an underdetermined fit | Collect independent motions and enough samples for the selected experiment. |

Older robot projects should replace `record` and the separate `derive` feature
with `recording`, and migrate `CoupledDifferential` to `Differential::geared`
or `DifferentialConfig`.
