//! Select the macro definition when compiling the diagnostics crate. Consumers
//! neither read the clock nor evaluate log arguments in an unlogged build.

/// Emit a human-readable event, with the same message recorded on SD.
#[cfg(feature = "logging")]
#[macro_export]
macro_rules! log_event {
    ($level:ident, $source:expr, $($arg:tt)*) => {{
        let level = $crate::serial::Level::$level;
        if $crate::serial::enabled(level) {
            $crate::serial::message(level, $crate::serial::now(), $source, format_args!($($arg)*));
        }
    }};
}

#[cfg(not(feature = "logging"))]
#[macro_export]
macro_rules! log_event {
    ($level:ident, $source:expr, $($arg:tt)*) => {{
        if false {
            let _ = ($source, format_args!($($arg)*));
        }
    }};
}

/// Emit typed SD fields. Values must be comma-free single-line strings.
#[cfg(feature = "logging")]
#[macro_export]
macro_rules! log_row {
    ($level:ident, $kind:expr, $($arg:tt)*) => {{
        if $crate::serial::sd_enabled() {
            $crate::serial::row(
                $crate::serial::Level::$level,
                $crate::serial::now(),
                $kind,
                format_args!($($arg)*),
            );
        }
    }};
}

#[cfg(not(feature = "logging"))]
#[macro_export]
macro_rules! log_row {
    ($level:ident, $kind:expr, $($arg:tt)*) => {{
        if false {
            let _ = ($kind, format_args!($($arg)*));
        }
    }};
}

#[cfg(all(test, not(feature = "logging")))]
mod tests {
    #[test]
    fn disabled_calls_do_not_evaluate_arguments() {
        fn unexpected() -> u8 {
            unreachable!("log arguments must be skipped")
        }
        log_event!(Info, "test", "value={}", unexpected());
        log_row!(Trace, "TEST", "value={}", unexpected());
    }
}
