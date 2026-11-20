//! Streaming renderer for Ozton's DBG session and telemetry rows.

use std::io::{self, BufRead, Write};

pub(crate) fn view(
    mut reader: impl BufRead,
    mut out: impl Write,
    kind_filter: Option<&str>,
) -> io::Result<()> {
    let mut line = String::new();
    let mut session = 0;
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
        writeln!(out, "{kind:<13}  {}", fields.replace(',', ", "))?;
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
}
