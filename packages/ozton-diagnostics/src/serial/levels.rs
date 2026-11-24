//! Level routing and the on-disk event encoding.

use std::{
    fmt,
    io::{self, Write},
    time::Duration,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Level {
    Error,
    Warn,
    Info,
    Debug,
    Trace,
}

impl Level {
    fn label(self) -> &'static str {
        match self {
            Self::Error => "ERROR",
            Self::Warn => "WARN",
            Self::Info => "INFO",
            Self::Debug => "DEBUG",
            Self::Trace => "TRACE",
        }
    }

    fn usb(self) -> bool {
        matches!(self, Self::Error | Self::Warn | Self::Info)
    }
}

/// Whether the SD sink opened successfully at startup.
pub fn sd_enabled() -> bool {
    #[cfg(target_os = "vexos")]
    return crate::sd::enabled();
    #[cfg(not(target_os = "vexos"))]
    false
}

#[doc(hidden)]
pub fn enabled(level: Level) -> bool {
    level.usb() || sd_enabled()
}

#[doc(hidden)]
pub fn now() -> Duration {
    vexide::time::user_uptime()
}

/// Encode one string field for a schema 2 row.
pub fn field(value: impl fmt::Display) -> impl fmt::Display {
    Escaped(value)
}

/// Escape free-form values so every SD row is one line of comma-separated fields.
pub(super) struct Escaped<T>(pub T);

impl<T: fmt::Display> fmt::Display for Escaped<T> {
    fn fmt(&self, out: &mut fmt::Formatter<'_>) -> fmt::Result {
        use fmt::Write as _;
        struct Encoder<'a, 'b>(&'a mut fmt::Formatter<'b>);
        impl fmt::Write for Encoder<'_, '_> {
            fn write_str(&mut self, text: &str) -> fmt::Result {
                let mut start = 0;
                for (index, character) in text.char_indices() {
                    if character.is_ascii_control() || matches!(character, '%' | ',' | '=') {
                        self.0.write_str(&text[start..index])?;
                        write!(self.0, "%{:02X}", u32::from(character))?;
                        start = index + 1;
                    }
                }
                self.0.write_str(&text[start..])
            }
        }
        write!(Encoder(out), "{}", self.0)
    }
}

#[doc(hidden)]
pub fn message(level: Level, time: Duration, source: &str, message: fmt::Arguments<'_>) {
    let source_field = Escaped(source);
    let message_field = Escaped(message);
    row(
        level,
        time,
        "EVENT",
        format_args!("source={source_field},message={message_field}"),
    );
    if level.usb() {
        let ms = time.as_millis();
        let _ = writeln!(
            io::stdout().lock(),
            "[{:>6}.{:03}s] {:<5} [{source}] {message}",
            ms / 1000,
            ms % 1000,
            level.label(),
        );
    }
}

#[doc(hidden)]
pub fn row(level: Level, time: Duration, kind: &str, fields: fmt::Arguments<'_>) {
    enqueue(
        level,
        format_args!("DBG,{kind},t_ms={},{fields}", time.as_millis()),
    );
}

pub(super) fn enqueue(level: Level, fields: fmt::Arguments<'_>) {
    #[cfg(target_os = "vexos")]
    crate::sd::enqueue(format_args!("{fields},level={}", level.label()), level.usb());
    #[cfg(not(target_os = "vexos"))]
    let _ = (level, fields);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_operational_levels_reach_usb() {
        assert!(Level::Error.usb() && Level::Warn.usb() && Level::Info.usb());
        assert!(!Level::Debug.usb() && !Level::Trace.usb());
    }

    #[test]
    fn message_delimiters_cannot_forge_fields_or_records() {
        let value = format_args!("pose=x,y\nÉtat 50%\r");
        assert_eq!(Escaped(value).to_string(), "pose%3Dx%2Cy%0AÉtat 50%25%0D");
    }
}
