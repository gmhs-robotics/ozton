//! Application startup and diagnostics lifetime.

use std::{fmt::Display, future::Future, process::ExitCode};

/// Initialize an application and run its competition runtime. Device construction
/// belongs inside the async block so startup telemetry includes sensor calibration.
/// No application-level panic hook, SD writer, or diagnostic task is needed.
pub async fn run<R, E>(
    initialize: impl Future<Output = Result<crate::Runtime<R>, E>>,
) -> ExitCode
where
    R: crate::OztonCompete,
    E: Display,
{
    let _startup = Startup::begin();
    #[cfg(feature = "logging")]
    let started = std::time::Instant::now();
    crate::log_event!(Info, "startup", "Initializing robot");
    match initialize.await {
        Ok(runtime) => {
            #[cfg(feature = "logging")]
            crate::log_event!(
                Info, "startup", "Robot ready in {:.2}s", started.elapsed().as_secs_f64()
            );
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

/// Owns startup logging until application exit or cancellation.
pub(crate) struct Startup {
    #[cfg(feature = "logging")]
    _writer: Option<vexide::task::Task<()>>,
}

impl Startup {
    pub(crate) fn begin() -> Self {
        #[cfg(feature = "logging")]
        {
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
            Self { _writer: crate::diagnostics::start_sd_logging() }
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
