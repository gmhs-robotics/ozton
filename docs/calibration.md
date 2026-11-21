# Configure and calibrate a chassis

Start with a known pose, then validate motor directions, encoder signs, IMU
orientation, wheel diameters, and physical track width. Check a slow straight
drive and an in-place rotation before enabling field corrections. Repeat the
tests in both directions.

## Run a Brain calibration trial

Use `compete_with_calibration` with a trial fixture built from `motion::auton::Robot<M>`.
Pass the regular `Route<M>` entries and the trials your hardware supports.
Each trial has a label and a dedicated CSV path on the SD card.

```rust,ignore
use ozton::motion::auton::{Calibration, CalibrationTrial, compete_with_calibration};
use ozton::uom::si::{angle::degree, f64::{Angle, Length}, length::meter};

let trials = [
    Calibration::new("Straight 1 m", "straight.csv",
        CalibrationTrial::Straight { distance: Length::new::<meter>(1.0) }),
    Calibration::new("Turn 180°", "turn.csv",
        CalibrationTrial::Turn { angle: Angle::new::<degree>(180.0) }),
    Calibration::new("Front range", "range.csv",
        CalibrationTrial::Range { sensor: "front", reference: Length::new::<meter>(0.50) }),
    Calibration::new("Drive feedforward", "voltage.csv", CalibrationTrial::Feedforward),
];
compete_with_calibration(robot, display, controller, routes, trials).await?;
```

1. Choose the trial while disabled, then enter driver mode in a clear area.
2. Hold **A** to sample. Straight and feedforward use the left stick Y.
   Turn uses the right stick X. Range sampling keeps the chassis stopped.
3. For a straight or turn trial, release **A** at the measured mark. Repeat
   the experiment in both directions before disabling. For feedforward, vary
   speed and acceleration in both directions. Each hold is limited to eight
   seconds. Moving trials command up to 3 V before chassis output shaping.
4. Disable to write the CSV. A new successful session replaces that trial’s
   file. Keep the previous file if you need samples from multiple sessions.
5. Fit the data with `cargo run -p ozton-tools -- calibrate straight.csv`.
   Use the experiment name to select one side or one sensor where applicable.

## Collect reference samples

The [programmed example](programmed.md#complete-example) saves a bounded journal
to `auton.csv` in its `CompetitionControls::mode_changed` hook on disable.
`Journal::write_csv` also accepts a serial or file writer. A capacity of zero
disables logging. A full buffer overwrites its oldest records and reports how
many it dropped. Size the buffer for the sample frequency and experiment length.

For live serial diagnostics, call `with_diagnostics::<MyReport>(interval)`
after `with_ui`. Implement `runtime::diagnostics::Report<CompetitionBot>`
to select the data to emit. Ozton schedules the report in debug builds after
competition ticks and during runner steps. Use `Frame::row` to give related
rows a common sequence and timestamp. The programmed example uses
`MotorSample::read` to report the intake, and `mark_driver_output(tick)`
after the drive write to populate `max_driver_output_us` in `DBG,LOOP`.
Use the SD journal or typed trial CSV for `ozton-tools calibrate`.

Add typed calibration rows with `diagnostics::calibration::CalibrationSample`.
The ordinary telemetry shows commands and measurements. Calibration rows also
include the known reference or the applied motor voltage. Give each experiment
a name, such as `drive_left` or `front_range`. To see the CSV columns and units:

```sh
cargo run -p ozton-tools -- --help
cargo run -p ozton-tools -- calibrate auton.csv front_range
```

- **Wheel diameter.** Measure signed straight-line travel with a tape. Log
  signed wheel turns from both sides. Convert motor turns to wheel turns using
  each gear ratio before logging. Fit both directions and keep the surfaces and
  load representative.
- **Track width and pod offsets.** Run in-place rotations and record measured
  heading change, signed side travel, and each pod's travel. Use calibrated
  wheel diameters and independently measured heading. Include clockwise and
  counterclockwise trials. Use [tracking-wheel offsets](localization.md#height-aware-dsr-and-horizontal-odometry)
  measured from The Origin.
- **Distance sensors.** Measure from the sensor face to a flat target at known
  distances. Collect at least three readings. Keep sensors in separate groups.
  Apply the fitted bias and noise as `RangeSensorConfig::bias` and `sigma`.
- **Feedforward.** During a supervised drive test, capture slow ramps and
  acceleration transients in both directions. After each chassis sample call
  `log_feedforward_calibration(["drive_left", "drive_right"])` to record the
  measured speed, acceleration, and voltage of the reference motors. Use the
  results as `MotorFeedforward` values for the corresponding side. Review the
  sample count and RMSE before applying the fit. The tool reports a
  degenerate or unphysical fit as an error.

The feedforward model and experiment design follow
[WPILib system identification](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html).

## Validate field behavior

1. Run repeated routes on wheel odometry. Measure terminal position and heading
   independently, and record median and 95th-percentile error and the number of
   completed runs.
2. Try an explicit single-wall reset, then two independently visible walls.
   Mask or block a sensor and check that its observation is rejected. Run
   with a sensor temporarily disconnected and verify that reconnecting does
   not inject travel.
3. Compare EKF or MCL candidates to the odometry baseline in shadow mode.
   Only enable the candidate after it improves repeated runs on the actual
   field. Lower the MCL particle count or number of mapped surfaces if it
   consumes too much of the tick budget.
4. On the Brain, check the 10 ms control period under maximum sensor and
   mechanism load. The `auton,step` row's `b` column is measured tick
   interval in seconds. For a CSV with at least one autonomous step, get p99:

   ```sh
   awk -F, '$3 == "auton" && $4 == "step" { print $6 * 1000 }' auton.csv |
     sort -g | awk '{ ms[NR] = $1 } END { if (NR) print ms[int((99 * NR + 99) / 100)] " ms" }'
   ```

5. Verify that disable, a control gap, and a drive fault stop both sides and
   any retained mechanism commands. Check the route selector and SD save and
   reload on the Brain before a match.
