//! Bounded telemetry shared between firmware and host tools.

use std::{
    collections::VecDeque,
    io::{self, Write},
    time::Duration,
};

pub mod calibration;
#[cfg(all(feature = "sd-logging", any(target_os = "vexos", test)))]
pub mod sd;
pub mod serial;

pub const CSV_HEADER: &str = "version,time_us,kind,name,a,b,c,d,e";

/// Numeric event with a stable, comma-free kind and name. Missing values are empty.
#[derive(Debug, Clone, Copy)]
pub struct Event {
    pub time: Duration,
    pub kind: &'static str,
    pub name: &'static str,
    pub values: [Option<f64>; 5],
}

impl Event {
    pub fn new(time: Duration, kind: &'static str, name: &'static str, values: [f64; 5]) -> Self {
        Self {
            time,
            kind,
            name,
            values: values.map(|value| value.is_finite().then_some(value)),
        }
    }
}

/// Stores the newest events in bounded memory. Use a zero capacity to disable logging.
pub struct Journal {
    events: VecDeque<Event>,
    capacity: usize,
    dropped: u64,
}

impl Journal {
    pub fn new(capacity: usize) -> Self {
        Self {
            events: VecDeque::with_capacity(capacity),
            capacity,
            dropped: 0,
        }
    }

    pub fn enabled(&self) -> bool {
        self.capacity != 0
    }

    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }

    /// Store a valid row. Disabled journals and malformed labels return false.
    pub fn push(&mut self, event: Event) -> bool {
        if !self.enabled() {
            return false;
        }
        if [event.kind, event.name]
            .iter()
            .any(|label| label.contains([',', '\r', '\n', '"']))
        {
            self.dropped = self.dropped.saturating_add(1);
            return false;
        }
        if self.events.len() == self.capacity {
            self.events.pop_front();
            self.dropped = self.dropped.saturating_add(1);
        }
        self.events.push_back(event);
        true
    }

    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    pub fn clear(&mut self) {
        self.events.clear();
        self.dropped = 0;
    }

    /// Exports a snapshot to an SD file or serial writer after the robot stops.
    /// The buffer remains available if the sink fails.
    pub fn write_csv(&self, mut writer: impl Write) -> io::Result<()> {
        writeln!(writer, "{CSV_HEADER}")?;
        if self.dropped > 0 {
            writeln!(writer, "1,0,journal,dropped,{},,,,", self.dropped)?;
        }
        for event in &self.events {
            write!(
                writer,
                "1,{},{},{}",
                event.time.as_micros(),
                event.kind,
                event.name
            )?;
            for value in event.values {
                write!(writer, ",")?;
                if let Some(value) = value.filter(|value| value.is_finite()) {
                    write!(writer, "{value}")?;
                }
            }
            writeln!(writer)?;
        }
        writer.flush()
    }
}
