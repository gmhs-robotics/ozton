# AGENTS.md

## Task Completion Requirements

- `cargo check --workspace --all-targets` and
  `cargo test --workspace --all-targets` must be completed before considering
  tasks completed.
- Use conventional commit messages for any git commits created in this repo.

## Overview

`ozton` is a vex robotics metaframework centered around:

- robot recording/playback (`ozton-record`)
- drivetrain abstractions (`ozton-drivetrain`)
- tracking/localization (`ozton-tracking`)
- math/control/motion primitives ported from `evian`

The repo is organized as a Rust workspace with one crate per responsibility
under `packages/`.

## Workspace Layout

- `packages/ozton`
  - meta crate
  - re-exports the workspace crates behind features
  - provides the public prelude
- `packages/ozton-derive`
  - proc-macro crate for auto-generating recording frame/application glue
- `packages/ozton-record`
  - recording/playback runtime and frame abstractions
  - user-facing recording API should center on:
    - `#[derive(RecordedRobot)]` on the robot struct
    - `#[record(skip)]` for non-recorded fields like controllers
    - implementing only `get_new_frame`
  - live-vs-playback behavior belongs in field/device types, not in user robot code
- `packages/ozton-math`
  - math utilities and lightweight geometry types
- `packages/ozton-control`
  - feedback/feedforward control loops and tolerances
- `packages/ozton-motion`
  - motion algorithms that act on drivetrain + tracking traits
- `packages/ozton-tracking`
  - tracking traits and tracking implementations
  - includes wheeled tracking and GPS-backed tracking
- `packages/ozton-drivetrain`
  - drivetrain wrapper and drivetrain models

## Core Design Rules

- Keep crates narrowly scoped.
- Prefer trait-based integration points over concrete coupling.
- Motion code should depend on tracking traits, not sensor implementations.
- Drivetrain code should depend on model traits, not specific autonomous
  routines.
  - The `ozton` crate should stay a thin re-export/meta crate.
- Recording should not force users to hand-write transform/apply boilerplate.
- Tracking-corrected drivetrain replay is a separate recording workflow from plain
  open-loop drivetrain replay.

## Conventions

This codebase should operate in SI units unless an external API or device forces
another unit. If that does happen, please use a reasonable conversion factor to
convert it into an SI unit.

## Maintainability

Long term maintainability is a core priority. If you add new functionality,
first check if there are shared logic that can be extracted to a separate
module. Duplicate logic across multiple files is a code smell and should be
avoided. Don't be afraid to change existing code. Don't take shortcuts by just
adding local logic to solve a problem.

## External Libraries

- Prefer established crates.io libraries over local reimplementation for
  commodity primitives.
- Current examples:
  - angles come from `vexide-devices`
  - 2D vector math is backed by `glam`
- Before writing a custom PID, geometry, math, or motion helper, check whether
  a mature crate already fits the required API, units model, and `no_std`
  constraints.

## Documentation

- Public traits and structs should document usage, what they represent, and
  interesting internals.
- Examples that require hardware setup should use `ignore` or `no_run` doctests.

## References

- Vexide: https://vexide.dev
- Vexide Repo: https://github.com/vexide/vexide
- Evian (strong reference implementation): https://github.com/vexide/evian
- Autons (for selection UI, reference): https://github.com/vexide/autons
