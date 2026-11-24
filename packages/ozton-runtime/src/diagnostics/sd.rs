//! Startup and periodic flushing for the firmware SD sink.

/// Queue startup records immediately; open the card after device startup yields. Keep the task
/// alive for the program. Competition also calls this for applications that
/// leave SD initialization to the runtime.
pub(crate) fn start_sd_logging() -> Option<vexide::task::Task<()>> {
    #[cfg(all(feature = "logging", target_os = "vexos"))]
    {
        use std::time::{Duration, Instant};

        use ozton_diagnostics::{log_event, sd};
        if !sd::prepare_startup() {
            return None;
        }
        Some(vexide::task::spawn(async {
            // Give the caller's hardware initialization its first poll. The
            // synchronous SD open can then overlap hardware IMU calibration.
            vexide::time::sleep(Duration::from_millis(1)).await;
            let started = Instant::now();
            match sd::enable_at_startup() {
                Some(Ok(())) => log_event!(
                    Info,
                    "SD",
                    "Recording all levels to {} (open {} ms)",
                    sd::PATH,
                    started.elapsed().as_millis()
                ),
                Some(Err(error)) => {
                    log_event!(
                        Warn,
                        "SD",
                        "Could not open {} ({error}). USB logging active.",
                        sd::PATH
                    );
                    return;
                }
                None => return,
            }
            let mut failed = false;
            let mut last_sync = Instant::now();
            let mut last_summary = Instant::now();
            let mut was_autonomous = false;
            let mut delay = Duration::from_millis(40);
            loop {
                vexide::time::sleep(delay).await;
                let autonomous = crate::competition::autonomous_active();
                let sync_period = Duration::from_secs(if autonomous { 5 } else { 1 });
                let sync = last_sync.elapsed() >= sync_period || (was_autonomous && !autonomous);
                was_autonomous = autonomous;
                if last_summary.elapsed() >= Duration::from_secs(1) {
                    ozton_diagnostics::profile::flush_summary();
                    ozton_diagnostics::log_row!(
                        Trace,
                        "SD_QUEUE",
                        "queued_bytes={}",
                        sd::queued_bytes()
                    );
                    last_summary = Instant::now();
                }
                if sync {
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
