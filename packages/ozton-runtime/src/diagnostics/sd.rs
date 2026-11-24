//! Startup and periodic flushing for the firmware SD sink.

/// Probe and start the SD writer before initializing devices. Keep the task
/// alive for the program. Competition also calls this for applications that
/// leave SD initialization to the runtime.
pub fn start_sd_logging() -> Option<vexide::task::Task<()>> {
    #[cfg(all(feature = "logging", target_os = "vexos"))]
    {
        use std::time::Duration;

        use ozton_diagnostics::{log_event, sd};
        match sd::enable_at_startup()? {
            Ok(()) => log_event!(Info, "SD", "Recording all levels to {}", sd::PATH),
            Err(error) => {
                log_event!(
                    Warn,
                    "SD",
                    "Could not open {} ({error}). USB logging active.",
                    sd::PATH
                );
                return None;
            }
        }
        Some(vexide::task::spawn(async {
            let mut failed = false;
            loop {
                vexide::time::sleep(Duration::from_secs(1)).await;
                ozton_diagnostics::profile::flush_summary();
                let _span = ozton_diagnostics::profile::Span::new(ozton_diagnostics::profile::Stage::SdFlush);
                match sd::flush() {
                    Err(error) if !failed => {
                        log_event!(
                            Warn,
                            "SD",
                            "Could not flush {} ({error}). Retrying.",
                            sd::PATH
                        );
                        failed = true;
                    }
                    Ok(()) if failed => {
                        log_event!(Info, "SD", "Recording resumed on {}", sd::PATH);
                        failed = false;
                    }
                    _ => {}
                }
            }
        }))
    }
    #[cfg(not(all(feature = "logging", target_os = "vexos")))]
    None
}

/// Give the startup error record a chance to reach the card before exiting.
pub fn flush_sd_logging() {
    #[cfg(all(feature = "logging", target_os = "vexos"))]
    if let Err(error) = ozton_diagnostics::sd::flush() {
        ozton_diagnostics::log_event!(Warn, "SD", "Final flush failed ({error})");
    }
}
