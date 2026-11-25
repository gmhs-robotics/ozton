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
            let path = match sd::enable_at_startup() {
                Some(Ok(path)) => {
                    log_event!(
                        Info,
                        "SD",
                        "Recording all levels to {path} (open {} ms)",
                        started.elapsed().as_millis()
                    );
                    path
                }
                Some(Err(error)) => {
                    log_event!(
                        Warn,
                        "SD",
                        "Could not create numbered log ({error}). USB logging active."
                    );
                    return;
                }
                None => return,
            };
            let mut failed = false;
            let mut last_sync = Instant::now();
            let mut last_summary = Instant::now();
            let mut was_autonomous = false;
            let mut delay = Duration::from_millis(40);
            let mut bytes_written = 0usize;
            let mut writes = 0u64;
            let mut peak_queue = 0usize;
            let mut max_flush_us = 0u128;
            let mut errors = 0u64;
            loop {
                vexide::time::sleep(delay).await;
                let autonomous = crate::competition::autonomous_active();
                let sync_period = Duration::from_secs(if autonomous { 5 } else { 1 });
                let sync = last_sync.elapsed() >= sync_period || (was_autonomous && !autonomous);
                let autonomous_ended = was_autonomous && !autonomous;
                was_autonomous = autonomous;
                peak_queue = peak_queue.max(sd::queued_bytes());
                if autonomous_ended || last_summary.elapsed() >= Duration::from_secs(5) {
                    ozton_diagnostics::profile::flush_summary();
                    ozton_diagnostics::log_row!(
                        Trace,
                        "SD_QUEUE",
                        "queued_bytes={},peak_queue_bytes={peak_queue},written_bytes={bytes_written},writes={writes},max_flush_us={max_flush_us},errors={errors},window_ms={}",
                        sd::queued_bytes(),
                        last_summary.elapsed().as_millis()
                    );
                    last_summary = Instant::now();
                    bytes_written = 0;
                    writes = 0;
                    peak_queue = 0;
                    max_flush_us = 0;
                    errors = 0;
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
                let elapsed = started.elapsed();
                max_flush_us = max_flush_us.max(elapsed.as_micros());
                writes += 1;
                match &result {
                    Ok(bytes) => bytes_written += *bytes,
                    Err(_) => errors += 1,
                }
                // A slow SDK call cannot be preempted. Reduce I/O duty cycle
                // instead of scheduling back-to-back writes on the executor.
                delay = (elapsed * 8).clamp(Duration::from_millis(40), Duration::from_millis(500));
                match result {
                    Err(error) if !failed => {
                        log_event!(Warn, "SD", "Could not flush {path} ({error}). Retrying.");
                        failed = true;
                    }
                    Ok(_) if failed => {
                        log_event!(Info, "SD", "Recording resumed on {path}");
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
