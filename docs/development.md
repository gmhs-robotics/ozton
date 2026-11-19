# Build and troubleshoot

Run these commands from the Ozton repository with its pinned Rust nightly and
`rust-src`.

```sh
cargo check --workspace --all-targets
cargo test --workspace --all-targets
cargo check --workspace --all-targets --no-default-features --features ozton/recording
cargo test --workspace --all-targets --no-default-features --features ozton/recording
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

## Publish the book

The **Publish the Ozton Book** workflow builds the book and deploys its static
output to [docs.eli.best](https://docs.eli.best/) on Vercel. It runs on relevant
pushes to `main` and can also be dispatched from the Actions tab.

In the GitHub `docs-production` environment, set `VERCEL_TOKEN` as a secret
and set `VERCEL_ORG_ID` and `VERCEL_PROJECT_ID` as variables for the `ozton-docs`
Vercel project. Use a token scoped to the project's Vercel team. The workflow
passes the token to Vercel and uploads the rendered book from `target/book`.

| Symptom | Check |
| --- | --- |
| Build says to choose one mode | Disable default features when selecting `recording`. |
| Path import or profiling fails at startup | Check the export format and chassis limits passed to `Path::new`. |
| Route starts with `PathStart` | Check the declared start position, field convention and heading for the travel direction. |
| Pose jumps or stops updating | Check encoder direction, IMU calibration, sensor health and localizer rejection logs. |
| Alignment never settles | Verify sensor orientation, clear line of sight, the selected range and the command deadline. |
| Calibration tool reports an underdetermined fit | Collect independent motions and enough samples for the selected experiment. |

Older robot projects should replace `record` and the separate `derive` feature
with `recording`, and migrate `CoupledDifferential` to `Differential::geared`
or `DifferentialConfig`.

To move a programmed robot from manual menu and competition setup to the
assembled API, install application state with `with_controls`, register
routes with `autonomous_routes!`, and call
`.with_autonomous::<Route>(display).compete().await`. See the
[programmed example](programmed.md#complete-example) for the hook signatures.
