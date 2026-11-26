//! Serial records with a common timestamp and sequence, independent of hardware.

use std::{
    fmt,
    io::{self, Write},
    time::Duration,
};

#[cfg(feature = "logging")]
mod levels;
mod macros;
#[cfg(feature = "logging")]
pub use levels::{Level, enabled, field, message, now, row, sd_enabled, set_clock};

/// Stamp shared by every row in one diagnostic sample.
#[derive(Clone, Copy, Debug)]
pub struct Frame {
    pub sequence: u64,
    pub time: Duration,
}

impl Frame {
    /// Write a DBG row to any sink, including a serial tap or test buffer.
    pub fn write_row(
        &self,
        mut sink: impl Write,
        kind: &str,
        fields: fmt::Arguments<'_>,
    ) -> io::Result<()> {
        writeln!(
            sink,
            "DBG,{kind},seq={},t_ms={},{}",
            self.sequence,
            self.time.as_millis(),
            fields
        )
    }

    /// Record a debug sample on SD.
    pub fn row(&self, kind: &str, fields: fmt::Arguments<'_>) {
        #[cfg(feature = "logging")]
        self.record(Level::Debug, kind, fields);
        #[cfg(not(feature = "logging"))]
        let _ = (kind, fields);
    }

    /// Record a sample at an explicit level on SD.
    #[cfg(feature = "logging")]
    pub fn record(&self, level: Level, kind: &str, fields: fmt::Arguments<'_>) {
        levels::enqueue(
            level,
            format_args!(
                "DBG,{kind},seq={},t_ms={},{fields}",
                self.sequence,
                self.time.as_millis()
            ),
        );
    }
}

/// An informational event independent of periodic sampling.
pub fn event(time: Duration, fields: fmt::Arguments<'_>) {
    #[cfg(feature = "logging")]
    message(Level::Info, time, "diagnostics", fields);
    #[cfg(not(feature = "logging"))]
    let _ = (time, fields);
}

/// Persistent status for outputs or controllers. Emits only fault/recovery
/// edges while counting every observed failure. The clock runs only on edges.
pub struct FaultLog {
    #[cfg(feature = "logging")]
    source: &'static str,
    #[cfg(feature = "logging")]
    clock: fn() -> Duration,
    code: Option<&'static str>,
    count: u64,
}

impl FaultLog {
    pub const fn new(source: &'static str, clock: fn() -> Duration) -> Self {
        #[cfg(not(feature = "logging"))]
        let _ = (source, clock);
        Self {
            #[cfg(feature = "logging")]
            source,
            #[cfg(feature = "logging")]
            clock,
            code: None,
            count: 0,
        }
    }

    pub fn code(&self) -> Option<&'static str> {
        self.code
    }

    pub fn count(&self) -> u64 {
        self.count
    }

    pub fn fault(&mut self, code: &'static str) {
        self.fail(code, format_args!(""));
    }

    pub fn fail(&mut self, code: &'static str, details: fmt::Arguments<'_>) {
        self.count = self.count.saturating_add(1);
        let previous = self.code.replace(code);
        #[cfg(not(feature = "logging"))]
        let _ = (previous, details);
        #[cfg(feature = "logging")]
        if previous != Some(code) {
            message(
                Level::Error,
                (self.clock)(),
                self.source,
                format_args!("Fault {code} (count {}){details}", self.count),
            );
        }
    }

    pub fn recovered(&mut self) {
        let previous = self.code.take();
        #[cfg(not(feature = "logging"))]
        let _ = previous;
        #[cfg(feature = "logging")]
        if let Some(previous) = previous {
            message(
                Level::Info,
                (self.clock)(),
                self.source,
                format_args!("Recovered from {previous} (fault count {})", self.count),
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn two_rows_share_a_frame_stamp() {
        let stamp = Frame {
            sequence: 9,
            time: Duration::from_millis(50),
        };
        let mut bytes = Vec::new();
        stamp
            .write_row(&mut bytes, "ONE", format_args!("value=2"))
            .unwrap();
        stamp
            .write_row(&mut bytes, "TWO", format_args!("value=NaN"))
            .unwrap();
        assert_eq!(
            String::from_utf8(bytes).unwrap(),
            "DBG,ONE,seq=9,t_ms=50,value=2\nDBG,TWO,seq=9,t_ms=50,value=NaN\n"
        );
    }
}
