# Introduction

Ozton helps you write competition firmware for VEX V5 robots using [vexide].
It provides drivetrain control, localization, and a Brain menu for selecting
autonomous routines. Your application defines the hardware, the driver controls,
and what the robot does on the field.

## Choose a firmware mode

| To do this | Use this mode | Start here |
| --- | --- | --- |
| Write autonomous routines in Rust, follow paths, and coordinate mechanisms | `programmed` (default) | [Build a programmed robot][programmed] |
| Capture a driven routine to SD and replay it during autonomous | `recording` with default features disabled | [Record and replay][recording] |

Choose the competition loop in your firmware entry point. The two library
features can be enabled together when dependencies need both APIs.

## Read the book by task

- [The Origin][origin] defines the axes and units used for poses, paths, and sensor mounts.
- [Configure localization][localization] shows how to combine encoders, an IMU,
  tracking wheels, and field references.
- [Calibrate and verify][calibration] covers measurements and repeatability on the field.
- [Troubleshooting][troubleshooting] links common symptoms to the relevant checks.

[vexide]: https://vexide.dev/
[programmed]: programmed.md
[recording]: recording.md
[origin]: coordinates.md
[localization]: localization.md
[calibration]: calibration.md
[troubleshooting]: development.md
