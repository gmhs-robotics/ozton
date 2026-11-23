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
    loop {
        line.clear();
        if reader.read_line(&mut line)? == 0 {
            break;
        }
        if !line.ends_with('\n') {
            break;
        }
        let Some(record) = line.trim_end().strip_prefix("DBG,") else {
            continue;
        };
        let Some((kind, payload)) = record.split_once(',') else {
            continue;
        };
        found = true;
        if kind == "SESSION" {
            schema_2 = payload.split(',').any(|field| field == "schema=2");
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
    fn renders_schema_two_events_and_skips_incomplete_records() {
        let input = b"DBG,SESSION,t_ms=0,schema=2,level=INFO\nDBG,EVENT,t_ms=1234,source=motion,message=x%3D1%2Cy%3D2,level=WARN\nDBG,MOTOR,t_ms=1260,name=lift\n";
        let mut output = Vec::new();
        view(&input[..], &mut output, None).unwrap();
        let output = String::from_utf8(output).unwrap();
        assert!(output.contains("WARN   [motion] x=1,y=2"));
        assert!(!output.contains("MOTOR"));
    }
}
