# Logging and profiling

Build firmware with the logging feature and insert a writable SD card before starting the program. USB shows operational messages. The SD sink records detailed telemetry in a new numbered log (`1.log`, `2.log`, …), using the lossless schema-4 delta format. The SD card's `index.log` stores the highest allocated number. Keep this file even when deleting old logs; existing numbered files are never reopened. Logging is optional; normal competition firmware can omit it.

```sh
cargo build --release --features logging
python3 tools/decode_log.py 1.log > telemetry.jsonl
python3 tools/decode_log.py 1.log --legacy > telemetry.log
```

The Python decoder is in the Ozton repository, not necessarily your application directory. Copy the SD file to your computer first. The older ozton-tools view-log command expects decoded DBG rows; use --legacy first.

## What is recorded

- An optional `BUILD` record contains the application name and version, application and Ozton Git revisions and tree states, compiler, Rust target, Cargo profile, and application configuration.
- MOTION_STEP and PATH_TICK: target wheel speeds, measured pose and motion timing.
- LOCALIZATION_POSE, LOCALIZATION_RANGE, TRACKING_INPUT and TRACKING_WHEEL: raw inputs, estimator output and correction decisions.
- MCL and DSR: particle or reset decisions.
- MOTOR, DRIVE and IMU: measured device values and requested motor control.
- LEGACY_MOTION_TICK and LEGACY_PURSUIT_TICK: normalized controller outputs.
- PROFILE: per-stage execution-time summaries.

Ozton starts the diagnostics task automatically when competition begins. It reads the Brain's smart-port device table once per sampling frame (nominally 25 ms) and records every connected motor, including raw vexide motors in your own mechanisms. No reporter or registration is required in application code. The task uses read-only SDK access: it does not create another motor handle, change device settings or issue commands.

Motor rows use the port number as the stable identifier. Shared Ozton motor groups also supply the exact software command (including voltage); for unregistered motors the SDK's available mode, brake, RPM and position target registers are recorded instead. Unknown requested values are NaN, not fabricated from measured voltage. Localization workers automatically supply pose, calibrated wheel feedback and IMU samples. During motion, a shared rate gate avoids duplicate drivetrain snapshots.

The scheduler may produce fewer samples; use actual timestamps. Serialization reuses scratch buffers and a bounded schema dictionary. `ozton::run` handles startup error and panic logging, SD writer lifetime and the startup duration. With logging disabled, no hardware-sampling task is started.

For a flat `#[vexide::main]`, return `Result<(), StartupError>` and bind
`let startup = Startup::with_metadata(metadata);` before constructing devices.
`StartupError` logs initialization errors propagated with `?`. Call
`startup.ready()` just before awaiting `runtime.compete()` to record initialization
time. Keep the guard in scope through competition so it owns the SD writer.
`Startup`, `StartupError`, and `BuildMetadata` are available in `ozton::runtime`.
Applications using an initialization future can use `ozton::run_with_metadata`.

The application supplies build-time information for the BUILD record.
Override captures both Git commits and reports each source tree
as `clean`, `dirty`, or `unknown` when repository information is unavailable.
A dirty commit identifies the base revision. Keep the build inputs or commit
the changes when exact reproduction matters. `configuration` is the selected
Override field profile. In older logs with no BUILD record, use the operator's
run notes to identify the firmware.

## Read profiling records

PROFILE rows summarize each active stage over a flush window (normally about one second). They include count, total_us, mean_us, max_us, p95_upper_us and over_budget. The p95 value is an upper histogram-bucket estimate, not an exact percentile. ControlInterval reports time between control ticks against the configured period plus 1 ms; other stages count samples exceeding the nominal 10 ms budget.

Tracking measures device sampling. Localization includes estimator work and its logging. MclPredict and MclObserve are nested inside Localization. MotionTick covers synchronous work in physical-velocity Seeking, Planner measures the synchronous path search, HardwareReport measures diagnostic snapshots, and SdFlush measures synchronous SD I/O. These timings are inclusive; do not add nested totals together as if they were independent CPU consumption.

A long flush or planning spike may delay the next control tick. A high average in localization may indicate expensive particle work. Use counts and timestamps to distinguish infrequent spikes from continuous load. SD_GAP indicates dropped rows or uncertain writes. Disable and allow the writer to flush before removing the card.

## Format

SESSION marks a new run. BUILD is a plain DBG row so provenance stays readable directly on the card. S rows define a schema ID, record kind and field names. R rows use that ID and list values. Schema 4 adds D rows, which store a hexadecimal change mask and only the changed values, with independent histories for named devices and ports. Full S/R keyframes are emitted after sync or recovery. All recorded timestamps and numeric precision are preserved. The decoder also reads schema 3. The writer keeps the file open, submits roughly 4 KiB per write, syncs about once a second and backs off after slow calls. SD_QUEUE reports backlog. Optional hardware sampling pauses under pressure and the bounded queue reserves space for operational records. No synchronous VEXos I/O call has a guaranteed latency. Preserve original logs so a later decoder can recover all available fields.

Startup records enter the bounded memory buffer immediately. SD file opening
runs on the writer task after the hardware initialization future has yielded,
so synchronous card setup can overlap IMU calibration. The startup event reports
elapsed application initialization and the SD event reports card-open duration.
A failed card probe disables the SD sink; operational USB messages remain.

USB is reserved for readable operational events: command start/settle, trajectory
terminal correction, distance-reset request/result, sustained clearance-governor
engagement/release, and motion interruption/failure. Governor changes must persist
for 250 ms before being announced; the safety check itself is not delayed.
Stall events distinguish currently governed output from unrestricted output.
Numeric per-tick motion, localization and profiling rows remain SD-only.
