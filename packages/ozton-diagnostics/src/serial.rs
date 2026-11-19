//! Serial records with a common timestamp and sequence, independent of hardware.

use std::{
    fmt,
    io::{self, Write},
    time::Duration,
};

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

    /// Emit one complete row to stdout.
    pub fn row(&self, kind: &str, fields: fmt::Arguments<'_>) {
        let _ = self.write_row(io::stdout().lock(), kind, fields);
    }
}

/// An event independent of periodic sampling.
pub fn event(time: Duration, fields: fmt::Arguments<'_>) {
    println!("DBG,EVENT,t_ms={},{}", time.as_millis(), fields);
}

/// Persistent status for outputs or controllers. Emits only fault/recovery
/// edges while counting every observed failure. The clock runs only on edges.
pub struct FaultLog {
    source: &'static str,
    clock: fn() -> Duration,
    code: Option<&'static str>,
    count: u64,
}

impl FaultLog {
    pub const fn new(source: &'static str, clock: fn() -> Duration) -> Self {
        Self {
            source,
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
        if self.code.replace(code) != Some(code) {
            println!(
                "DBG,FAULT,t_ms={},source={},code={},count={}{}",
                (self.clock)().as_millis(),
                self.source,
                code,
                self.count,
                details
            );
        }
    }

    pub fn recovered(&mut self) {
        if let Some(previous) = self.code.take() {
            println!(
                "DBG,RECOVERED,t_ms={},source={},previous={},count={}",
                (self.clock)().as_millis(),
                self.source,
                previous,
                self.count
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
