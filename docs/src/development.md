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

The source pages live in docs/src and the dependency-free static site builder is docs/build.mjs. Run node docs/build.mjs to generate docs/site. Vercel reads the root vercel.json and builds the site directly from GitHub when the repository is connected to the ozton-docs project. The build command is node docs/build.mjs and the output directory is docs/site. Keep ozton.eli.best on that project. Use main as the production branch once the guide is merged; development branches receive preview deployments. Do not deploy unreviewed main-branch firmware changes merely to publish documentation.

## Contributing

Keep robot-specific tuning in your application, not in library defaults. Add regression tests for motion behavior, invalid feedback and competition cancellation. Document behavior and failure modes rather than only listing APIs.
