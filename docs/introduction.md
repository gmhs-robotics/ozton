# Introduction

Ozton helps you write competition firmware for VEX V5 robots using [vexide].
It provides drivetrain control, localization, and a Brain menu for selecting
autonomous routines. Your application defines the hardware, the driver controls,
and what the robot does on the field.

Start with [Build a programmed robot][programmed] to wire your robot and
write a driver mapping and autonomous route.

## Read the book by task

- [The Origin][origin] defines the axes and units used for poses, paths, and sensor mounts.
- [Configure localization][localization] shows how to combine encoders, an IMU,
  tracking wheels, and field references.
- [Calibrate and verify][calibration] covers measurements and repeatability on the field.
- [Troubleshooting][troubleshooting] links common symptoms to the relevant checks.

[vexide]: https://vexide.dev/
[programmed]: programmed.md
[origin]: coordinates.md
[localization]: localization.md
[calibration]: calibration.md
[troubleshooting]: development.md
