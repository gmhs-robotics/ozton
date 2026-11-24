# Logging and profiling

Build firmware with the logging feature and insert a writable SD card before starting the program. USB shows operational messages. The SD sink records detailed telemetry in OZTON3.LOG, using the compact schema-3 format. Logging is optional; normal competition firmware can omit it.

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

Shared drivetrain reporters are rate-limited to one hardware snapshot per 10 ms, so a background reporter and motion command do not both read the same devices. The scheduler may produce fewer samples; use actual timestamps. Serialization reuses scratch buffers and a bounded schema dictionary.

## Read profiling records

PROFILE rows summarize each active stage over a flush window (normally about one second). They include count, total_us, mean_us, max_us, p95_upper_us and over_budget. The p95 value is an upper histogram-bucket estimate, not an exact percentile. ControlInterval reports time between control ticks against the configured period plus 1 ms; other stages count samples exceeding the nominal 10 ms budget.

Tracking measures device sampling. Localization includes estimator work and its logging. MclPredict and MclObserve are nested inside Localization. MotionTick covers synchronous work in PoseSeeking, Planner measures the synchronous path search, HardwareReport measures diagnostic snapshots, and SdFlush measures synchronous SD I/O. These timings are inclusive; do not add nested totals together as if they were independent CPU consumption.

A long flush or planning spike may delay the next control tick. A high average in localization may indicate expensive particle work. Use counts and timestamps to distinguish infrequent spikes from continuous load. SD_GAP indicates dropped rows or uncertain writes. Disable and allow the writer to flush before removing the card.

## Format

SESSION marks a new run. S rows define a schema ID, record kind and field names; R rows use that ID and list values. Uncompressed DBG records also occur. Schemas are re-emitted for each flush batch. The decoder handles both forms. Preserve original logs so a later decoder can recover all available fields.
