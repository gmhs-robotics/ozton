//! Append-only VEX Brain telemetry with a bounded staging buffer.

use std::{
    fmt::{self, Write as _},
    fs::{File, OpenOptions},
    io::{self, Write},
    sync::Mutex,
};

pub const PATH: &str = "OZTON3.LOG";
const BUFFER_BYTES: usize = 512 * 1024;
const SESSION: &[u8] = b"DBG,SESSION,t_ms=0,schema=3,level=INFO\n";

#[derive(Default)]
struct Pending {
    bytes: Vec<u8>,
    dropped_rows: u64,
    uncertain_bytes: usize,
    session_started: bool,
    schemas: Vec<String>,
    #[cfg(target_os = "vexos")]
    probed: bool,
}

static PENDING: Mutex<Pending> = Mutex::new(Pending {
    bytes: Vec::new(),
    dropped_rows: 0,
    uncertain_bytes: 0,
    session_started: false,
    schemas: Vec::new(),
    #[cfg(target_os = "vexos")]
    probed: false,
});

#[cfg(target_os = "vexos")]
use std::sync::atomic::{AtomicBool, Ordering};

#[cfg(target_os = "vexos")]
static ENABLED: AtomicBool = AtomicBool::new(false);

#[cfg(target_os = "vexos")]
pub(crate) fn enabled() -> bool {
    ENABLED.load(Ordering::Relaxed)
}

/// Probe the card once per program run. A successful open enables enqueueing.
#[cfg(target_os = "vexos")]
pub fn enable_at_startup() -> Option<io::Result<()>> {
    let mut pending = match PENDING.lock() {
        Ok(pending) => pending,
        Err(_) => return Some(Err(io::Error::other("SD log lock poisoned"))),
    };
    if pending.probed {
        return None;
    }
    pending.probed = true;
    Some(open_log().map(|_| ENABLED.store(true, Ordering::Relaxed)))
}

fn open_log() -> io::Result<File> {
    OpenOptions::new().create(true).append(true).open(PATH)
}

#[cfg(target_os = "vexos")]
pub(crate) fn enqueue(row: fmt::Arguments<'_>) {
    if !enabled() {
        return;
    }
    let Ok(mut pending) = PENDING.try_lock() else {
        return;
    };
    pending.push(row);
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

/// Append buffered rows to the card. An unavailable card leaves rows queued.
/// Competition schedules this separately from report collection.
pub fn flush() -> io::Result<()> {
    let mut pending = PENDING
        .try_lock()
        .map_err(|_| io::Error::other("SD log buffer unavailable"))?;
    if pending.bytes.is_empty() && pending.dropped_rows == 0 && pending.uncertain_bytes == 0 {
        return Ok(());
    }
    pending.append(open_log()?)
}

impl Pending {
    fn push(&mut self, row: fmt::Arguments<'_>) {
        // Schema dictionary is scoped to a flush batch. A decoder can recover
        // after partial writes without relying on earlier dictionary records.
        let text = row.to_string();
        let mut parts = text.split(',');
        let prefix = parts.next();
        let kind = parts.next();
        let fields: Option<Vec<_>> = parts.map(|part| part.split_once('=')).collect();
        let start = self.bytes.len();
        let result = if let (Some("DBG"), Some(kind), Some(fields)) = (prefix, kind, fields) {
            if kind == "EVENT" || kind == "SD_GAP" || kind == "SESSION" {
                writeln!(BatchWriter(&mut self.bytes), "{text}")
            } else {
                let mut schema = kind.to_owned();
                for (key, _) in &fields { schema.push(','); schema.push_str(key); }
                let id = self.schemas.iter().position(|old| *old == schema);
                let index = id.unwrap_or(self.schemas.len());
                let mut output = BatchWriter(&mut self.bytes);
                let encoded = (|| -> fmt::Result {
                    if id.is_none() { writeln!(output, "S,{index},{schema}")?; }
                    write!(output, "R,{index}")?;
                    for (_, value) in &fields { write!(output, ",{value}")?; }
                    writeln!(output)
                })();
                if encoded.is_ok() && id.is_none() { self.schemas.push(schema); }
                encoded
            }
        } else { writeln!(BatchWriter(&mut self.bytes), "{text}") };
        if result.is_err() {
            self.bytes.truncate(start);
            self.dropped_rows = self.dropped_rows.saturating_add(1);
        }
    }

    fn append(&mut self, mut file: impl Write) -> io::Result<()> {
        if !self.session_started || self.uncertain_bytes != 0 {
            // A previous writer or power loss can leave an incomplete last row.
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
        let result = file.write_all(&self.bytes).and_then(|()| file.flush());
        if result.is_ok() {
            self.dropped_rows = 0;
            self.uncertain_bytes = 0;
        } else {
            // Some bytes may already be on disk. Avoid replaying partial batches.
            self.uncertain_bytes = self.uncertain_bytes.saturating_add(self.bytes.len());
        }
        self.bytes.clear();
        self.schemas.clear();
        result
    }
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
    fn log_appends_one_session_and_reports_a_gap() {
        let mut pending = Pending {
            bytes: b"DBG,EVENT,t_ms=2,source=startup,message=boot,level=INFO\n".to_vec(),
            dropped_rows: 3,
            ..Pending::default()
        };
        let mut output = b"partial previous row".to_vec();
        pending.append(&mut output).unwrap();
        pending.push(format_args!(
            "DBG,EVENT,t_ms=4,source=startup,message=ready,level=INFO"
        ));
        pending.append(&mut output).unwrap();
        assert_eq!(
            String::from_utf8(output).unwrap(),
            "partial previous row\nDBG,SESSION,t_ms=0,schema=3,level=INFO\nDBG,SD_GAP,dropped_rows=3,uncertain_bytes=0,level=WARN\nDBG,EVENT,t_ms=2,source=startup,message=boot,level=INFO\nDBG,EVENT,t_ms=4,source=startup,message=ready,level=INFO\n"
        );
    }

    #[test]
    fn failed_batch_does_not_create_a_second_session() {
        let mut pending = Pending::default();
        pending.push(format_args!("DBG,TEST,t_ms=1,level=TRACE"));
        let mut header = [0; SESSION.len() + 1];
        assert!(pending.append(header.as_mut_slice()).is_err());
        let mut output = header.to_vec();
        assert!(pending.uncertain_bytes > 0);

        pending.push(format_args!("DBG,TEST,t_ms=2,level=TRACE"));
        pending.append(&mut output).unwrap();
        let output = String::from_utf8(output).unwrap();
        assert_eq!(output.matches("DBG,SESSION").count(), 1);
        assert!(output.contains("DBG,SD_GAP"));
        assert!(output.contains("S,0,TEST,t_ms,level"));
        assert!(output.contains("R,0,2,TRACE"));
    }
}
