# Contributing to Ozton

Use the Rust nightly pinned in `rust-toolchain.toml` with `rust-src` and
`rustfmt`. Run the checks below from the workspace root.

```sh
cargo fmt --all --check
cargo check --workspace --all-targets --locked
cargo test --workspace --all-targets --locked
cargo test --workspace --all-targets --locked --no-default-features --features ozton/recording
cargo test --workspace --doc --locked
mdbook build
mdbook test
```

Use SI units at library boundaries. Follow the field and heading conventions
in [the coordinate guide](docs/introduction.md#coordinates-and-units). When a
public API changes, update its Rustdoc, the relevant book chapter, and any
examples that use it. Keep runnable hardware examples under
[`packages/ozton/examples`](packages/ozton/examples).

For hardware-dependent changes, describe the robot setup and what you verified
on a V5 Brain. For a bug report, include the Ozton revision, firmware mode,
expected behavior, and a small reproduction or relevant log excerpt.
