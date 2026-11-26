#!/usr/bin/env python3
"""Decode schema 3/4 Ozton logs to JSON Lines or schema-2-compatible DBG rows.

Usage: python3 tools/decode_log.py 1.log > telemetry.jsonl
       python3 tools/decode_log.py 1.log --legacy > telemetry.log
"""
import argparse
import json
from urllib.parse import unquote

def records(lines):
    schemas = {}
    previous = {}
    session = 0
    for line in lines:
        if not line.endswith("\n"):
            continue
        parts = line.rstrip("\r\n").split(",")
        if len(parts) < 2:
            continue
        if parts[0] == "S" and len(parts) >= 4:
            schemas[parts[1]] = (parts[2], parts[3:])
            previous.pop(parts[1], None)
            continue
        if parts[0] == "R":
            schema = schemas.get(parts[1])
            if schema is None or len(parts) - 2 != len(schema[1]):
                previous.pop(parts[1], None)
                continue
            kind, keys = schema
            previous[parts[1]] = parts[2:]
            fields = dict(zip(keys, parts[2:]))
        elif parts[0] == "D":
            schema = schemas.get(parts[1])
            base = previous.get(parts[1])
            if schema is None or base is None or len(parts) < 3:
                continue
            try:
                mask = int(parts[2], 16)
            except ValueError:
                previous.pop(parts[1], None)
                continue
            if mask < 0 or mask >> len(base) or mask.bit_count() != len(parts) - 3:
                previous.pop(parts[1], None)
                continue
            changed = iter(parts[3:])
            values = [next(changed) if mask & (1 << i) else v for i, v in enumerate(base)]
            previous[parts[1]] = values
            kind, keys = schema
            fields = dict(zip(keys, values))
        elif parts[0] == "DBG":
            kind = parts[1]
            fields = dict(p.split("=", 1) for p in parts[2:] if "=" in p)
            if kind == "SESSION":
                session += 1
                schemas.clear()
                previous.clear()
            elif kind == "SD_GAP" and fields.get("uncertain_bytes", "0") != "0":
                previous.clear()
        else:
            continue
        yield session, kind, fields

def value(s):
    s = unquote(s)
    if s in ("true", "false"):
        return s == "true"
    if s.lower() in ("nan", "inf", "-inf", "none"):
        return None
    try:
        return int(s)
    except ValueError:
        try:
            return float(s)
        except ValueError:
            return s

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    parser.add_argument("--legacy", action="store_true")
    args = parser.parse_args()
    with open(args.log, encoding="utf-8", errors="replace") as f:
        for session, kind, fields in records(f):
            if args.legacy:
                print(",".join(["DBG", kind] + [f"{k}={v}" for k, v in fields.items()]))
            else:
                decoded = {
                    k: unquote(v) if kind == "BUILD" and k != "t_ms" else value(v)
                    for k, v in fields.items()
                }
                print(json.dumps({"session": session, "kind": kind, **decoded}, allow_nan=False))

if __name__ == "__main__":
    main()
