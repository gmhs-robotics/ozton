# Logging and profiling

Build firmware with the logging feature and insert a writable SD card before starting the program. USB shows operational messages. The SD sink records detailed telemetry in OZTON3.LOG, using the lossless schema-4 delta format. Logging is optional; normal competition firmware can omit it.

```sh
cargo build --release --features logging
python3 tools/decode_log.py OZTON3.LOG > telemetry.jsonl
python3 tools/decode_log.py OZTON3.LOG --legacy > telemetry.log
```

The Python decoder is in the Ozton repository, not necessarily your application directory. Copy the SD file to your computer first. The older ozton-tools view-log command expects decoded DBG rows; use --legacy first.

## What is recorded

- MOTION_STEP and PATH_TICK: target wheel speeds, measured pose and motion timing.
- LOCALIZATION_POSE, LOCALIZATION_RANGE, TRACKING_INPUT and TRACKING_WHEEL: raw inputs, estimator output and correction decisions.
- MCL and DSR: particle or reset decisions.
- MOTOR, DRIVE and IMU: measured device values and requested motor control.
- LEGACY_MOTION_TICK and LEGACY_PURSUIT_TICK: normalized controller outputs.
- PROFILE: per-stage execution-time summaries.

Ozton starts the diagnostics task automatically when competition begins. It reads the Brain's smart-port device table once per sampling frame (nominally 25 ms) and records every connected motor, including raw vexide motors in your own mechanisms. No reporter or registration is required in application code. The task uses read-only SDK access: it does not create another motor handle, change device settings or issue commands.

Motor rows use the port number as the stable identifier. Shared Ozton motor groups also supply the exact software command (including voltage); for unregistered motors the SDK's available mode, brake, RPM and position target registers are recorded instead. Unknown requested values are NaN, not fabricated from measured voltage. Localization workers automatically supply pose, calibrated wheel feedback and IMU samples. During motion, a shared rate gate avoids duplicate drivetrain snapshots.

The scheduler may produce fewer samples; use actual timestamps. Serialization reuses scratch buffers and a bounded schema dictionary. `ozton::run` handles startup error and panic logging, SD writer lifetime and the startup duration. With logging disabled, no hardware-sampling task is started.

## Read profiling records

PROFILE rows summarize each active stage over a flush window (normally about one second). They include count, total_us, mean_us, max_us, p95_upper_us and over_budget. The p95 value is an upper histogram-bucket estimate, not an exact percentile. ControlInterval reports time between control ticks against the configured period plus 1 ms; other stages count samples exceeding the nominal 10 ms budget.

Tracking measures device sampling. Localization includes estimator work and its logging. MclPredict and MclObserve are nested inside Localization. MotionTick covers synchronous work in physical-velocity Seeking, Planner measures the synchronous path search, HardwareReport measures diagnostic snapshots, and SdFlush measures synchronous SD I/O. These timings are inclusive; do not add nested totals together as if they were independent CPU consumption.

A long flush or planning spike may delay the next control tick. A high average in localization may indicate expensive particle work. Use counts and timestamps to distinguish infrequent spikes from continuous load. SD_GAP indicates dropped rows or uncertain writes. Disable and allow the writer to flush before removing the card.

## Format

SESSION marks a new run. S rows define a schema ID, record kind and field names; R rows use that ID and list values. Uncompressed DBG records also occur. Schema 4 adds D rows: a hexadecimal change mask followed by only the changed values, with independent histories for named devices/ports. Full S/R keyframes are emitted after sync or recovery. All recorded timestamps and numeric precision are preserved; the decoder also reads schema 3. The writer keeps the file open, submits roughly 4 KiB per write, syncs about once a second and backs off after slow calls. SD_QUEUE reports backlog. Optional hardware sampling pauses under pressure and the bounded queue reserves space for operational records. No synchronous VEXos I/O call has a guaranteed latency. Preserve original logs so a later decoder can recover all available fields.
