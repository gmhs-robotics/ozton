# Ozton

Ozton is a Rust framework for VEX V5 competition robots. It provides
differential drive, localization, a Brain selector, path following, and an
exclusive recording and playback mode.

- [Choose a firmware mode](docs/introduction.md)
- [Build a programmed robot](docs/programmed.md)
- [Import PATH.JERRYIO routes and write autonomous](docs/routes.md)
- [Configure odometry, DSR, EKF and MCL](docs/localization.md)
- [Build mechanisms](docs/mechanisms.md)
- [Record and replay routes](docs/recording.md)
- [Run calibration trials and fit telemetry](docs/calibration.md)
- [Build and troubleshoot](docs/development.md)

Render the full manual with `mdbook build`, or open it with `mdbook serve --open`.

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
