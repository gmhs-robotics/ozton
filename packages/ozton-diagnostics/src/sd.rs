//! Per-run VEX Brain telemetry with a bounded staging buffer.

use std::{
    fmt::{self, Write as _},
    fs::File,
    io::{self, Write},
    sync::Mutex,
};

mod index;

use index::open_log;
const BUFFER_BYTES: usize = 512 * 1024;
const SESSION: &[u8] = b"DBG,SESSION,t_ms=0,schema=4,level=INFO\n";
#[cfg(target_os = "vexos")]
const RESERVED_BYTES: usize = 32 * 1024;

struct Stream {
    schema: String,
    identity: String,
    previous: String,
    emitted: bool,
}

#[derive(Default)]
struct Pending {
    bytes: Vec<u8>,
    head: usize,
    dropped_rows: u64,
    uncertain_bytes: usize,
    session_started: bool,
    schemas: Vec<Stream>,
    row: String,
    schema: String,
    identity: String,
    values: String,
    file: Option<File>,
    #[cfg(target_os = "vexos")]
    probed: bool,
    #[cfg(target_os = "vexos")]
    prepared: bool,
}

static PENDING: Mutex<Pending> = Mutex::new(Pending {
    bytes: Vec::new(),
    head: 0,
    dropped_rows: 0,
    uncertain_bytes: 0,
    session_started: false,
    schemas: Vec::new(),
    row: String::new(),
    schema: String::new(),
    identity: String::new(),
    values: String::new(),
    file: None,
    #[cfg(target_os = "vexos")]
    probed: false,
    #[cfg(target_os = "vexos")]
    prepared: false,
});

#[cfg(target_os = "vexos")]
use std::sync::atomic::{AtomicBool, Ordering};

#[cfg(target_os = "vexos")]
static ENABLED: AtomicBool = AtomicBool::new(false);

#[cfg(target_os = "vexos")]
pub(crate) fn enabled() -> bool {
    ENABLED.load(Ordering::Relaxed)
}

/// Claim the startup writer and begin bounded in-memory recording before I/O.
/// A second runtime startup must not create another writer.
#[cfg(target_os = "vexos")]
pub fn prepare_startup() -> bool {
    let Ok(mut pending) = PENDING.lock() else {
        return false;
    };
    if pending.prepared {
        return false;
    }
    pending.prepared = true;
    ENABLED.store(true, Ordering::Relaxed);
    true
}

/// Open the card once, after device initialization has been polled.
/// Early startup records are already queued.
#[cfg(target_os = "vexos")]
pub fn enable_at_startup() -> Option<io::Result<String>> {
    let mut pending = match PENDING.lock() {
        Ok(pending) => pending,
        Err(_) => return Some(Err(io::Error::other("SD log lock poisoned"))),
    };
    if pending.probed {
        return None;
    }
    pending.probed = true;
    let result = open_log(std::path::Path::new("")).map(|(name, file)| {
        pending.file = Some(file);
        name
    });
    if result.is_ok() {
        let additional = BUFFER_BYTES.saturating_sub(pending.bytes.len());
        pending.bytes.reserve(additional);
        ENABLED.store(true, Ordering::Relaxed);
    } else {
        ENABLED.store(false, Ordering::Relaxed);
        pending.bytes.clear();
        pending.head = 0;
        pending.reset_streams();
    }
    Some(result)
}

#[cfg(target_os = "vexos")]
pub(crate) fn enqueue(row: fmt::Arguments<'_>, priority: bool) {
    if !enabled() {
        return;
    }
    let Ok(mut pending) = PENDING.try_lock() else {
        return;
    };
    if !priority && pending.queued_bytes() >= BUFFER_BYTES - RESERVED_BYTES {
        pending.dropped_rows = pending.dropped_rows.saturating_add(1);
        return;
    }
    // Reclaim the written prefix before capacity checks can drop useful rows.
    if pending.head > 0 && pending.bytes.len() >= BUFFER_BYTES - RESERVED_BYTES {
        let remaining = pending.queued_bytes();
        let head = pending.head;
        pending.bytes.copy_within(head.., 0);
        pending.bytes.truncate(remaining);
        pending.head = 0;
    }
    pending.push(row);
}

/// Pending data not yet submitted to the card.
pub fn queued_bytes() -> usize {
    PENDING
        .try_lock()
        .map_or(BUFFER_BYTES, |pending| pending.queued_bytes())
}

pub fn under_pressure() -> bool {
    queued_bytes() >= BUFFER_BYTES / 2
}

struct BatchWriter<'a>(&'a mut Vec<u8>);

impl fmt::Write for BatchWriter<'_> {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        if text.len() > BUFFER_BYTES - self.0.len() {
            return Err(fmt::Error);
        }
        self.0.extend_from_slice(text.as_bytes());
        Ok(())
    }
}

/// Emergency/final flush. The periodic writer uses bounded chunks instead.
pub fn flush() -> io::Result<()> {
    flush_chunk(usize::MAX, true).map(|_| ())
}

/// Submit at most one chunk of complete lines, optionally syncing the file.
/// VEXos writes and syncs synchronously; callers must yield between chunks.
pub fn flush_chunk(max_bytes: usize, sync: bool) -> io::Result<usize> {
    let mut pending = PENDING
        .try_lock()
        .map_err(|_| io::Error::other("SD log buffer unavailable"))?;
    #[cfg(target_os = "vexos")]
    if !enabled() {
        return Ok(0);
    }
    if pending.queued_bytes() == 0 && !sync {
        return Ok(0);
    }
    let Some(mut file) = pending.file.take() else {
        // Startup is the only place where a run file is allocated.
        return Ok(0);
    };
    let result = pending.write_chunk(&mut file, max_bytes, sync);
    // Keep the same handle on failure: never reopen an existing log.
    pending.file = Some(file);
    result
}

impl Pending {
    fn queued_bytes(&self) -> usize {
        self.bytes.len() - self.head
    }

    fn reset_streams(&mut self) {
        for stream in &mut self.schemas {
            stream.emitted = false;
        }
    }

    fn push(&mut self, row: fmt::Arguments<'_>) {
        // Reuse both scratch strings and schema storage. The steady-state
        // path performs no per-record heap allocation or intermediate field Vec.
        self.row.clear();
        if self.row.write_fmt(row).is_err() {
            self.dropped_rows = self.dropped_rows.saturating_add(1);
            return;
        }
        let mut parts = self.row.split(',');
        let prefix = parts.next();
        let kind = parts.next();
        let start = self.bytes.len();
        let result = if let (Some("DBG"), Some(kind)) = (prefix, kind) {
            if matches!(kind, "EVENT" | "SD_GAP" | "SESSION")
                || !parts.clone().all(|part| part.contains('='))
            {
                writeln!(BatchWriter(&mut self.bytes), "{}", self.row)
            } else {
                self.schema.clear();
                self.schema.push_str(kind);
                self.identity.clear();
                self.values.clear();
                for (i, (key, value)) in parts.filter_map(|part| part.split_once('=')).enumerate() {
                    self.schema.push(',');
                    self.schema.push_str(key);
                    if key == "port"
                        || (key == "name" && kind == "MOTOR")
                        || (key == "source" && kind == "LOCALIZATION_POSE")
                        || (key == "index"
                            && matches!(kind, "LOCALIZATION_RANGE" | "TRACKING_WHEEL"))
                        || (key == "axis" && kind == "MOTION_PID")
                        || (key == "stage" && kind == "PROFILE")
                    {
                        self.identity.push_str(key);
                        self.identity.push('=');
                        self.identity.push_str(value);
                        self.identity.push(',');
                    }
                    if i != 0 {
                        self.values.push(',');
                    }
                    self.values.push_str(value);
                }
                let found = self.schemas.iter().position(|stream| {
                    stream.schema == self.schema && stream.identity == self.identity
                });
                if found.is_none() && self.schemas.len() >= 128 {
                    writeln!(BatchWriter(&mut self.bytes), "{}", self.row)
                } else {
                    let index = found.unwrap_or(self.schemas.len());
                    let previous = found
                        .map(|index| &self.schemas[index])
                        .filter(|stream| stream.emitted);
                    let mut output = BatchWriter(&mut self.bytes);
                    let encoded = (|| -> fmt::Result {
                        if let Some(previous) = previous {
                            if let Some(mask) = delta_mask(&previous.previous, &self.values) {
                                // Exact ASCII header cost relative to the full row.
                                let delta_size = 1
                                    + (64 - mask.leading_zeros()).div_ceil(4).max(1) as usize
                                    + self
                                        .values
                                        .split(',')
                                        .enumerate()
                                        .filter(|(i, _)| mask & (1 << i) != 0)
                                        .map(|(_, value)| value.len() + 1)
                                        .sum::<usize>();
                                if delta_size < self.values.len() {
                                    write!(output, "D,{index},{mask:x}")?;
                                    for (i, value) in self.values.split(',').enumerate() {
                                        if mask & (1 << i) != 0 {
                                            write!(output, ",{value}")?;
                                        }
                                    }
                                    return writeln!(output);
                                }
                            }
                        } else {
                            writeln!(output, "S,{index},{}", self.schema)?;
                        }
                        writeln!(output, "R,{index},{}", self.values)
                    })();
                    if encoded.is_ok() {
                        if let Some(index) = found {
                            let stream = &mut self.schemas[index];
                            stream.previous.clear();
                            stream.previous.push_str(&self.values);
                            stream.emitted = true;
                        } else {
                            self.schemas.push(Stream {
                                schema: self.schema.clone(),
                                identity: self.identity.clone(),
                                previous: self.values.clone(),
                                emitted: true,
                            });
                        }
                    }
                    encoded
                }
            }
        } else {
            writeln!(BatchWriter(&mut self.bytes), "{}", self.row)
        };
        if result.is_err() {
            self.bytes.truncate(start);
            self.dropped_rows = self.dropped_rows.saturating_add(1);
        }
    }

    #[cfg(test)]
    fn write(&mut self, mut file: impl Write) -> io::Result<()> {
        self.write_chunk(&mut file, usize::MAX, true).map(|_| ())
    }

    fn write_chunk(
        &mut self,
        file: &mut impl Write,
        max_bytes: usize,
        sync: bool,
    ) -> io::Result<usize> {
        let remaining = &self.bytes[self.head..];
        let len = if remaining.len() <= max_bytes {
            remaining.len()
        } else {
            remaining[..max_bytes]
                .iter()
                .rposition(|&byte| byte == b'\n')
                .map(|i| i + 1)
                .unwrap_or_else(|| {
                    remaining
                        .iter()
                        .position(|&b| b == b'\n')
                        .map_or(remaining.len(), |i| i + 1)
                })
        };
        let result = (|| -> io::Result<()> {
            if self.uncertain_bytes != 0 {
                file.write_all(b"\n")?;
            }
            if !self.session_started {
                file.write_all(SESSION)?;
                self.session_started = true;
            }
            if self.dropped_rows != 0 || self.uncertain_bytes != 0 {
                writeln!(
                    file,
                    "DBG,SD_GAP,dropped_rows={},uncertain_bytes={},level=WARN",
                    self.dropped_rows, self.uncertain_bytes
                )?;
            }
            file.write_all(&remaining[..len])?;
            if sync {
                file.flush()?;
            }
            Ok(())
        })();
        if let Err(error) = result {
            self.uncertain_bytes = self.uncertain_bytes.saturating_add(self.queued_bytes());
            self.bytes.clear();
            self.head = 0;
            self.reset_streams();
            return Err(error);
        }
        self.dropped_rows = 0;
        self.uncertain_bytes = 0;
        self.head += len;
        if self.head == self.bytes.len() {
            self.bytes.clear();
            self.head = 0;
        } else if self.head >= 64 * 1024 && self.head >= self.bytes.len() / 2 {
            let remaining = self.queued_bytes();
            self.bytes.copy_within(self.head.., 0);
            self.bytes.truncate(remaining);
            self.head = 0;
        }
        if sync {
            self.reset_streams();
        }
        Ok(len)
    }
}

/// A bit identifies a changed field. No rounding or sample omission is used.
fn delta_mask(previous: &str, current: &str) -> Option<u64> {
    let mut old = previous.split(',');
    let mut mask = 0_u64;
    for (i, value) in current.split(',').enumerate() {
        if i >= 64 {
            return None;
        }
        if old.next()? != value {
            mask |= 1 << i;
        }
    }
    old.next().is_none().then_some(mask)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_an_entire_row_when_the_buffer_is_full() {
        let mut pending = Pending {
            bytes: vec![b'a'; BUFFER_BYTES - 3],
            ..Pending::default()
        };
        pending.push(format_args!("a,{}", 42));
        assert_eq!(pending.dropped_rows, 1);
        assert_eq!(pending.bytes.len(), BUFFER_BYTES - 3);
    }

    #[test]
    fn fresh_log_writes_one_session_and_reports_a_gap() {
        let mut pending = Pending {
            bytes: b"DBG,EVENT,t_ms=2,source=startup,message=boot,level=INFO\n".to_vec(),
            dropped_rows: 3,
            ..Pending::default()
        };
        let mut output = Vec::new();
        pending.write(&mut output).unwrap();
        pending.push(format_args!(
            "DBG,EVENT,t_ms=4,source=startup,message=ready,level=INFO"
        ));
        pending.write(&mut output).unwrap();
        assert_eq!(
            String::from_utf8(output).unwrap(),
            "DBG,SESSION,t_ms=0,schema=4,level=INFO\nDBG,SD_GAP,dropped_rows=3,uncertain_bytes=0,level=WARN\nDBG,EVENT,t_ms=2,source=startup,message=boot,level=INFO\nDBG,EVENT,t_ms=4,source=startup,message=ready,level=INFO\n"
        );
    }

    #[test]
    fn delta_rows_keep_device_histories_separate() {
        let mut pending = Pending::default();
        for (port, rpm) in [(1, 10), (2, 20), (1, 11), (2, 21)] {
            pending.push(format_args!(
                "DBG,MOTOR,port={port},rpm={rpm},constant=123456789012345678901234567890,level=TRACE"
            ));
        }
        let output = String::from_utf8(pending.bytes).unwrap();
        assert!(output.contains("D,0,2,11\n"));
        assert!(output.contains("D,1,2,21\n"));
    }

    #[test]
    fn alternating_localization_sources_have_independent_delta_histories() {
        let mut pending = Pending::default();
        for (source, x) in [("estimate", 1), ("odometry", 2), ("estimate", 3)] {
            pending.push(format_args!(
                "DBG,LOCALIZATION_POSE,source={source},x={x},y=0.000000,heading=0.000000,level=TRACE"
            ));
        }
        let output = String::from_utf8(pending.bytes).unwrap();
        assert!(output.contains("D,0,2,3\n"));
        assert!(output.contains("S,1,LOCALIZATION_POSE"));
    }

    #[test]
    fn chunked_writes_preserve_rows_and_reset_at_sync() {
        let mut pending = Pending::default();
        for value in 0..10 {
            pending.push(format_args!("DBG,TEST,value={value},level=TRACE"));
        }
        let mut output = Vec::new();
        while pending.queued_bytes() != 0 {
            pending.write_chunk(&mut output, 32, false).unwrap();
        }
        pending.write_chunk(&mut output, 32, true).unwrap();
        pending.push(format_args!("DBG,TEST,value=11,level=TRACE"));
        pending.write(&mut output).unwrap();
        let output = String::from_utf8(output).unwrap();
        assert_eq!(output.matches("DBG,SESSION").count(), 1);
        assert_eq!(output.matches("S,0,TEST").count(), 2);
        assert!(output.contains("R,0,11,TRACE\n"));
    }

    #[test]
    fn failed_batch_does_not_create_a_second_session() {
        let mut pending = Pending::default();
        pending.push(format_args!("DBG,TEST,t_ms=1,level=TRACE"));
        let mut header = [0; SESSION.len()];
        assert!(pending.write(header.as_mut_slice()).is_err());
        let mut output = header.to_vec();
        assert!(pending.uncertain_bytes > 0);

        pending.push(format_args!("DBG,TEST,t_ms=2,level=TRACE"));
        pending.write(&mut output).unwrap();
        let output = String::from_utf8(output).unwrap();
        assert_eq!(output.matches("DBG,SESSION").count(), 1);
        assert!(output.contains("DBG,SD_GAP"));
        assert!(output.contains("S,0,TEST,t_ms,level"));
        assert!(output.contains("R,0,2,TRACE"));
    }
}
