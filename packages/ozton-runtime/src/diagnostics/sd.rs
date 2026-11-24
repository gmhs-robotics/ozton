//! Startup and periodic flushing for the firmware SD sink.

/// Probe and start the SD writer before initializing devices. Keep the task
/// alive for the program. Competition also calls this for applications that
/// leave SD initialization to the runtime.
pub(crate) fn start_sd_logging() -> Option<vexide::task::Task<()>> {
    #[cfg(all(feature = "logging", target_os = "vexos"))]
    {
        use std::time::{Duration, Instant};

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
            let mut last_sync = Instant::now();
            let mut delay = Duration::from_millis(40);
            loop {
                vexide::time::sleep(delay).await;
                let sync = last_sync.elapsed() >= Duration::from_secs(1);
                if sync {
                    ozton_diagnostics::profile::flush_summary();
                    ozton_diagnostics::log_row!(
                        Trace,
                        "SD_QUEUE",
                        "queued_bytes={}",
                        sd::queued_bytes()
                    );
                    last_sync = Instant::now();
                }
                let started = Instant::now();
                let span = ozton_diagnostics::profile::Span::new(
                    ozton_diagnostics::profile::Stage::SdFlush,
                );
                let result = sd::flush_chunk(4096, sync);
                drop(span);
                // A slow SDK call cannot be preempted. Reduce I/O duty cycle
                // instead of scheduling back-to-back writes on the executor.
                delay = (started.elapsed() * 8)
                    .clamp(Duration::from_millis(40), Duration::from_millis(500));
                match result {
                    Err(error) if !failed => {
                        log_event!(
                            Warn,
                            "SD",
                            "Could not flush {} ({error}). Retrying.",
                            sd::PATH
                        );
                        failed = true;
                    }
                    Ok(_) if failed => {
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
pub(crate) fn flush_sd_logging() {
    #[cfg(all(feature = "logging", target_os = "vexos"))]
    if let Err(error) = ozton_diagnostics::sd::flush() {
        ozton_diagnostics::log_event!(Warn, "SD", "Final flush failed ({error})");
    }
}
