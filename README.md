# Ozton

[![Rust CI](https://github.com/gmhs-robotics/ozton/actions/workflows/rust.yml/badge.svg)](https://github.com/gmhs-robotics/ozton/actions/workflows/rust.yml)

Ozton is a Rust library for VEX V5 competition robots built with vexide.
It provides differential drive, localization, path following, and a Brain
selector for autonomous routines.

## Get started

Read [The Ozton Book](https://ozton.eli.best/) for setup, examples, and calibration.
Use the `programmed` feature (enabled by default) for routines written in Rust.
Use `recording` with default features disabled to capture and replay routes.
Choose one mode per firmware build.

To work from source, clone this repository and use its pinned Rust nightly with
`rust-src`. Build one of the complete V5 examples from the repository root.

```sh
cargo build -p ozton --example programmed --target thumbv7a-vex-v5 --locked
cargo build -p ozton --example full --target thumbv7a-vex-v5 --locked --no-default-features --features recording
```

## Guides

- [Choose a firmware mode](docs/introduction.md)
- [Build a programmed robot](docs/programmed.md)
- [Import PATH.JERRYIO routes and write autonomous](docs/routes.md)
- [Configure odometry, DSR, EKF and MCL](docs/localization.md)
- [Build mechanisms](docs/mechanisms.md)
- [Record and replay routes](docs/recording.md)
- [Run calibration trials and fit telemetry](docs/calibration.md)
- [Build and troubleshoot](docs/development.md)

Render the full manual with `mdbook build`, or open it with `mdbook serve --open`.
See [CONTRIBUTING.md](CONTRIBUTING.md) for checks to run before opening a pull request.

The [programmed example](packages/ozton/examples/programmed.rs) assembles a
`runtime::Robot`, registers a named route with `autonomous_routes!`, and uses
`Runner` for path following and range alignment. The recording examples derive
`RecordedRobot` for [open-loop playback](packages/ozton/examples/full.rs) and
[GPS-corrected playback](packages/ozton/examples/gps_voltage_tracking.rs).

Ozton draws on [Evian](https://github.com/vexide/evian),
[LemLib](https://github.com/LemLib/LemLib),
[RW-Template](https://richardbwang.github.io/RW-Template/),
[TitanReset](https://github.com/tubaplayerdis/TitanReset), and
[HoloLib](https://github.com/calhighrobotics/HoloLib).

## License

Choose [MIT](LICENSE-MIT.md) or [Apache 2.0](LICENSE-APACHE.md).
