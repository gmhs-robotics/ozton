//! Application startup and diagnostics lifetime.

use std::{fmt::Display, future::Future, process::ExitCode};

/// Git revision and working-tree state captured by the application's build.
/// Use `"unknown"` when building from a source archive without Git information.
#[derive(Debug, Clone, Copy)]
pub struct SourceRevision {
    pub commit: &'static str,
    pub tree: &'static str,
}

/// Build provenance emitted once at startup when diagnostics are enabled.
#[derive(Debug, Clone, Copy)]
pub struct BuildMetadata {
    pub application: &'static str,
    pub version: &'static str,
    pub source: SourceRevision,
    pub ozton: SourceRevision,
    pub compiler: &'static str,
    pub target: &'static str,
    pub profile: &'static str,
    pub configuration: &'static str,
}

impl BuildMetadata {
    #[cfg(feature = "logging")]
    fn record(self) {
        use ozton_diagnostics::serial::{Level, field, row};

        row(
            Level::Info,
            std::time::Duration::ZERO,
            "BUILD",
            format_args!(
                "application={},version={},commit={},tree={},ozton_commit={},ozton_tree={},compiler={},target={},profile={},configuration={}",
                field(self.application),
                field(self.version),
                field(self.source.commit),
                field(self.source.tree),
                field(self.ozton.commit),
                field(self.ozton.tree),
                field(self.compiler),
                field(self.target),
                field(self.profile),
                field(self.configuration),
            ),
        );
    }
}

/// Initialize an application and run its competition runtime. Device construction
/// belongs inside the async block so startup telemetry includes sensor calibration.
/// No application-level panic hook, SD writer, or diagnostic task is needed.
pub async fn run<A: Application, E: Display>(
    initialize: impl Future<Output = Result<A, E>>,
) -> ExitCode {
    run_application(None, initialize).await
}

/// Start the competition application with a provenance row in USB and SD logs.
pub async fn run_with_metadata<A: Application, E: Display>(
    metadata: BuildMetadata,
    initialize: impl Future<Output = Result<A, E>>,
) -> ExitCode {
    run_application(Some(metadata), initialize).await
}

async fn run_application<A: Application, E: Display>(
    metadata: Option<BuildMetadata>,
    initialize: impl Future<Output = Result<A, E>>,
) -> ExitCode {
    let startup = Startup::initialize(metadata);
    match initialize.await {
        Ok(runtime) => {
            startup.ready();
            runtime.compete().await
        }
        Err(error) => {
            crate::log_event!(Error, "startup", "Could not initialize robot ({error})");
            #[cfg(not(feature = "logging"))]
            eprintln!("Could not initialize robot ({error})");
            ExitCode::FAILURE
        }
    }
}

/// A completed competition application returned by the initialization block.
#[allow(async_fn_in_trait)]
pub trait Application {
    async fn compete(self) -> !;
}

impl<R: crate::OztonCompete> Application for crate::Runtime<R> {
    async fn compete(self) -> ! {
        crate::Runtime::compete(self).await
    }
}

/// Keeps startup diagnostics alive while the robot initializes and competes.
#[must_use = "keep the startup guard alive through competition"]
pub struct Startup {
    #[cfg(feature = "logging")]
    _writer: Option<vexide::task::Task<()>>,
    #[cfg(feature = "logging")]
    started: std::time::Instant,
}

impl Startup {
    /// Start diagnostics and record build provenance before initializing any devices.
    pub fn with_metadata(metadata: BuildMetadata) -> Self {
        Self::initialize(Some(metadata))
    }

    fn initialize(
        #[cfg_attr(not(feature = "logging"), allow(unused_variables))] metadata: Option<
            BuildMetadata,
        >,
    ) -> Self {
        let startup = Self::begin();
        #[cfg(feature = "logging")]
        if let Some(metadata) = metadata {
            metadata.record();
        }
        crate::log_event!(Info, "startup", "Initializing robot");
        startup
    }

    /// Record the time spent initializing the robot.
    pub fn ready(&self) {
        #[cfg(feature = "logging")]
        crate::log_event!(
            Info,
            "startup",
            "Robot ready in {:.3}s",
            self.started.elapsed().as_secs_f64()
        );
    }

    pub(crate) fn begin() -> Self {
        #[cfg(feature = "logging")]
        {
            let started = std::time::Instant::now();
            ozton_diagnostics::serial::set_clock(vexide::time::user_uptime);
            // Keep the platform panic handler: diagnostics supplement it rather
            // than suppressing its normal display and backtrace behavior.
            static HOOK: std::sync::Once = std::sync::Once::new();
            HOOK.call_once(|| {
                let previous = std::panic::take_hook();
                std::panic::set_hook(Box::new(move |panic| {
                    crate::log_event!(Error, "panic", "{panic}");
                    crate::diagnostics::flush_sd_logging();
                    previous(panic);
                }));
            });
            Self {
                _writer: crate::diagnostics::start_sd_logging(),
                started,
            }
        }
        #[cfg(not(feature = "logging"))]
        Self {}
    }
}

impl Drop for Startup {
    fn drop(&mut self) {
        #[cfg(feature = "logging")]
        crate::diagnostics::flush_sd_logging();
    }
}

/// Return this error from the entry point to record failures propagated with `?`
/// before the startup guard flushes the log.
pub struct StartupError(String);

impl std::fmt::Debug for StartupError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Could not initialize robot ({})", self.0)
    }
}

impl<E: Display> From<E> for StartupError {
    fn from(error: E) -> Self {
        let message = error.to_string();
        crate::log_event!(Error, "startup", "Could not initialize robot ({message})");
        Self(message)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct NeverStart;

    impl Application for NeverStart {
        async fn compete(self) -> ! {
            match std::future::pending::<std::convert::Infallible>().await {}
        }
    }

    #[test]
    fn startup_error_returns_failure_without_starting_competition() {
        let exit = vexide::runtime::block_on(run(async { Err::<NeverStart, _>("bad geometry") }));
        assert_eq!(exit, ExitCode::FAILURE);
    }

    #[test]
    fn flat_entry_point_preserves_the_initialization_error() {
        fn initialize() -> Result<(), StartupError> {
            Err("bad geometry")?;
            Ok(())
        }

        assert_eq!(
            format!("{:?}", initialize().unwrap_err()),
            "Could not initialize robot (bad geometry)"
        );
    }
}
