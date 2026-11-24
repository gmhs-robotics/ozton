# Development and deployment

Ozton is a Rust workspace. Use its pinned nightly and V5 build configuration. If you contribute code, keep examples and guides synchronized with the exact API you change.

```sh
cargo fmt --all --check
cargo check --workspace --all-targets --locked
cargo test --workspace --all-targets --locked
cargo check -p ozton --locked --all-features
cargo check -p ozton --locked --no-default-features
cargo check -p ozton --examples --locked --target thumbv7a-vex-v5
```

These are host and cross-compile checks, not hardware tests. For firmware, build against the exact Ozton commit your application uses. Keep logging off for competition unless you deliberately need it.

## Documentation

The book uses [mdBook](https://rust-lang.github.io/mdBook/). Chapters live in `docs/src`, the navigation is defined in `docs/src/SUMMARY.md`, and the configuration is in `docs/book.toml`. Install mdBook 0.5.3 locally, then run:

```sh
mdbook serve docs --open
mdbook build docs
```

Vercel uses the root `vercel.json` to install the pinned, checksum-verified mdBook binary and build the book. The output is `docs/book`. Connect the existing ozton-docs project to gmhs-robotics/ozton in Vercel Git settings. Set the production branch to main once the changes are merged. Other branches receive previews. The custom domain remains ozton.eli.best. The independent Documentation GitHub Actions workflow checks the book on changes, without requiring firmware compilation.

## Contributing

Keep robot-specific tuning in your application, not in library defaults. Add regression tests for motion behavior, invalid feedback and competition cancellation. Document behavior and failure modes rather than only listing APIs.
