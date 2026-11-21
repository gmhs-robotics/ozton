# AGENTS.md

## Task Completion Requirements

- Hard rule: Whenever you change any file in this repository, you must create a
  git commit containing the changes before finishing the task. Do not leave
  changes uncommitted unless the user explicitly instructs you to do so. If a
  commit is blocked, report the blocker explicitly.
- `cargo check --workspace --all-targets` and
  `cargo test --workspace --all-targets` must be completed before considering
  tasks completed.
- Use conventional commit messages for any git commits created in this repo.

## Overview

Ozton runs programmed VEX V5 competitions. Use `RobotBuilder`, `Program`, and
`route!` for application assembly. See [programmed.md](docs/programmed.md) when
changing competition hooks, device ownership, or route registration.

## Workspace Layout

Workspace crates live under `packages/`.

## Workspace Layout

- `packages/ozton`
  - meta crate
  - re-exports the workspace crates behind features
  - provides the public prelude
- `packages/ozton-math`
  - hardware-independent geometry, localization, and path math
- `packages/ozton-control`
  - feedback/feedforward control loops and tolerances
- `packages/ozton-motion`
  - motion algorithms that act on drivetrain + tracking traits
- `packages/ozton-tracking`
  - tracking traits and tracking implementations
  - timestamped localization and VEX sensor adapters
- `packages/ozton-drivetrain`
  - drivetrain wrapper and drivetrain models
- `packages/ozton-runtime`
  - shared selector, command clock, and VEX actuator adapters
- `packages/ozton-diagnostics` and `packages/ozton-tools`
  - telemetry schema, path conversion, and host calibration

## Core Design Rules

- Keep crates narrowly scoped.
- Prefer trait-based integration points over concrete coupling.
- Motion code should depend on tracking traits, not sensor implementations.
- Drivetrain code should depend on model traits, not specific autonomous
  routines.
  - The `ozton` crate should stay a thin re-export/meta crate.
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
- Before writing a custom PID, geometry, math, or motion helper, check whether a
  mature crate already fits the required API, units model, and `no_std`
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
