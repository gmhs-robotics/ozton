//! Streaming renderer for Ozton's DBG session and telemetry rows.

use std::io::{self, BufRead, Write};

pub(crate) fn view(
    mut reader: impl BufRead,
    mut out: impl Write,
    kind_filter: Option<&str>,
) -> io::Result<()> {
    let mut line = String::new();
    let mut session = 0;
    let mut schema_2 = false;
    let mut found = false;
    let mut compact = CompactRows::default();
    loop {
        line.clear();
        if reader.read_line(&mut line)? == 0 {
            break;
        }
        if !line.ends_with('\n') {
            break;
        }
        let decoded;
        let line = line.trim_end();
        let record_line = if line.starts_with("DBG,") {
            line
        } else {
            decoded = compact.decode(line);
            let Some(decoded) = decoded.as_deref() else {
                continue;
            };
            decoded
        };
        let Some(record) = record_line.strip_prefix("DBG,") else {
            continue;
        };
        let Some((kind, payload)) = record.split_once(',') else {
            continue;
        };
        found = true;
        if kind == "SESSION" {
            compact = CompactRows::default();
            schema_2 = payload
                .split(',')
                .any(|field| matches!(field, "schema=2" | "schema=4"));
            session += 1;
            if session > 1 {
                writeln!(out)?;
            }
            writeln!(out, "Session {session}")?;
            continue;
        }
        if kind_filter.is_some_and(|filter| !filter.eq_ignore_ascii_case(kind)) {
            continue;
        }
        let (payload, level) = if schema_2 {
            let Some((payload, level)) = payload.rsplit_once(",level=") else {
                continue;
            };
            if !matches!(level, "ERROR" | "WARN" | "INFO" | "DEBUG" | "TRACE") {
                continue;
            }
            (payload, Some(level))
        } else {
            (payload, None)
        };
        let (sequence, payload) = strip_field(payload, "seq=");
        let (time, fields) = strip_field(payload, "t_ms=");
        if let Some(ms) = time.and_then(|time| time.parse::<u64>().ok()) {
            write!(out, "{:>7}.{:03}s  ", ms / 1000, ms % 1000)?;
        } else {
            write!(out, "{:>13}", "")?;
        }
        if let Some(sequence) = sequence {
            write!(out, "#{sequence:<8}  ")?;
        } else {
            write!(out, "{:>11}", "")?;
        }
        if let Some(level) = level {
            write!(out, "{level:<5}  ")?;
        }
        if schema_2
            && kind == "EVENT"
            && let Some((source, message)) = fields
                .strip_prefix("source=")
                .and_then(|fields| fields.split_once(",message="))
        {
            writeln!(
                out,
                "[{}] {}",
                decode(source).escape_debug(),
                decode(message).escape_debug()
            )?;
        } else {
            writeln!(out, "{kind:<13}  {}", fields.replace(',', ", "))?;
        }
    }
    if !found {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "no complete Ozton log records found",
        ));
    }
    Ok(())
}

// Schema 4 keeps separate field lists and previous values for each stream.
#[derive(Default)]
struct CompactRows {
    streams: Vec<Option<CompactStream>>,
}

struct CompactStream {
    kind: String,
    keys: Vec<String>,
    values: Option<Vec<String>>,
}

impl CompactRows {
    fn decode(&mut self, line: &str) -> Option<String> {
        let mut parts = line.split(',');
        let tag = parts.next()?;
        let index: usize = parts.next()?.parse().ok()?;
        if index >= 128 {
            return None;
        }
        if tag == "S" {
            let kind = parts.next()?.to_owned();
            let keys = parts.map(str::to_owned).collect();
            self.streams
                .resize_with(self.streams.len().max(index + 1), || None);
            self.streams[index] = Some(CompactStream {
                kind,
                keys,
                values: None,
            });
            return None;
        }
        let stream = self.streams.get_mut(index)?.as_mut()?;
        match tag {
            "R" => {
                let values: Vec<_> = parts.map(str::to_owned).collect();
                stream.values = (values.len() == stream.keys.len()).then_some(values);
            }
            "D" => {
                let mask = u64::from_str_radix(parts.next()?, 16).ok()?;
                let values = stream.values.as_mut()?;
                if values.len() > 64
                    || (values.len() < 64 && mask >> values.len() != 0)
                    || parts.clone().count() != mask.count_ones() as usize
                {
                    stream.values = None;
                    return None;
                }
                for (i, value) in values.iter_mut().enumerate() {
                    if mask & (1 << i) != 0 {
                        *value = parts.next()?.to_owned();
                    }
                }
            }
            _ => return None,
        }
        let mut row = format!("DBG,{}", stream.kind);
        for (key, value) in stream.keys.iter().zip(stream.values.as_ref()?) {
            use std::fmt::Write as _;
            write!(row, ",{key}={value}").ok()?;
        }
        Some(row)
    }
}

fn strip_field<'a>(payload: &'a str, prefix: &str) -> (Option<&'a str>, &'a str) {
    payload
        .strip_prefix(prefix)
        .and_then(|value| value.split_once(','))
        .map_or((None, payload), |(value, rest)| (Some(value), rest))
}

fn decode(value: &str) -> String {
    let mut decoded = Vec::with_capacity(value.len());
    let mut bytes = value.as_bytes();
    while let Some((&byte, rest)) = bytes.split_first() {
        if byte == b'%'
            && let Some(hex) = rest.get(..2).and_then(|hex| std::str::from_utf8(hex).ok())
            && let Ok(value) = u8::from_str_radix(hex, 16)
        {
            decoded.push(value);
            bytes = &rest[2..];
        } else {
            decoded.push(byte);
            bytes = rest;
        }
    }
    String::from_utf8_lossy(&decoded).into_owned()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn filters_sample_rows_and_keeps_sessions() {
        let input = b"USB banner\nDBG,SESSION,t_ms=0,schema=1\nDBG,MOTOR,seq=7,t_ms=1234,name=lift,rpm=42\nDBG,LOOP,seq=7,t_ms=1234,ticks=64\nDBG,MOTOR,seq=8,t_ms=1254,name=";
        let mut output = Vec::new();
        view(&input[..], &mut output, Some("motor")).unwrap();
        assert_eq!(
            String::from_utf8(output).unwrap(),
            "Session 1\n      1.234s  #7         MOTOR          name=lift, rpm=42\n"
        );
    }

    #[test]
    fn decodes_compact_rows_even_when_other_streams_are_filtered() {
        let input = b"DBG,SESSION,t_ms=0,schema=4,level=INFO\nS,0,MOTOR,t_ms,port,rpm,level\nR,0,100,11,42,TRACE\nD,0,5,120,43\nS,1,MODE,t_ms,mode,level\nR,1,120,Autonomous,DEBUG\n";
        let mut output = Vec::new();
        view(&input[..], &mut output, Some("motor")).unwrap();
        let output = String::from_utf8(output).unwrap();
        assert!(output.contains("port=11, rpm=42"));
        assert!(output.contains("port=11, rpm=43"));
        assert!(!output.contains("MODE"));
    }

    #[test]
    fn renders_schema_two_events_and_skips_incomplete_records() {
        let input = b"DBG,SESSION,t_ms=0,schema=2,level=INFO\nDBG,EVENT,t_ms=1234,source=motion,message=x%3D1%2Cy%3D2,level=WARN\nDBG,MOTOR,t_ms=1260,name=lift\n";
        let mut output = Vec::new();
        view(&input[..], &mut output, None).unwrap();
        let output = String::from_utf8(output).unwrap();
        assert!(output.contains("WARN   [motion] x=1,y=2"));
        assert!(!output.contains("MOTOR"));
    }
}
