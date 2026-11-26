# Development and deployment

Ozton is a Rust workspace. Use its pinned nightly and V5 build configuration. If you contribute code, keep examples and guides synchronized with the exact API you change.

```sh
cargo fmt --all --check
cargo check --workspace --all-targets --locked
cargo test --workspace --all-targets --locked
cargo clippy --workspace --all-targets --locked --features ozton/logging -- -D warnings
cargo check -p ozton --locked --all-features
cargo check -p ozton --locked --no-default-features
cargo check -p ozton --examples --locked --target thumbv7a-vex-v5
```

These commands check host behavior and cross-compilation. For hardware validation,
build the exact firmware revisions with logging enabled. Match the BUILD record
to the source, compare localization against measured positions, and inspect
`PROFILE` for V5 control, MCL and SD latency under the installed field geometry.
Keep logging off for competition unless you deliberately need it.

## Documentation

The book uses [mdBook](https://rust-lang.github.io/mdBook/). Chapters live in `docs/src`, the navigation is defined in `docs/src/SUMMARY.md`, and the configuration is in `docs/book.toml`. Install mdBook 0.5.3 locally, then run:

```sh
mdbook serve docs --open
mdbook build docs
```

Vercel uses the root `vercel.json` to install the pinned, checksum-verified mdBook binary and build the book. The output is `docs/book`. The Vercel project is intended to track `gmhs-robotics/ozton`. Once its Git connection is active, pushes to the production branch update [ozton.eli.best](https://ozton.eli.best), while other branches receive preview URLs. Keep `main` as the long-term production branch after the development changes are merged. To verify a deployment, compare the commit SHA in Vercel with the commit you pushed. The independent Documentation GitHub Actions workflow checks the book on changes, without requiring firmware compilation.

## Contributing

Keep robot-specific tuning in your application, not in library defaults. Add regression tests for motion behavior, invalid feedback and competition cancellation. Document behavior and failure modes rather than only listing APIs.
