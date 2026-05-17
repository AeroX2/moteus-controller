#!/usr/bin/env python3
"""Probe an fdcanusb CDC port and print line-based responses."""

from __future__ import annotations

import argparse
import datetime
import re
import sys
import time

import serial

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(errors="replace")


DEFAULT_COMMANDS = (
    "tel list",
    "tel get firmware",
    "tel get git",
    "conf enumerate",
)


def _normalize_port(port: str) -> str:
    """Strip a trailing @baud suffix (e.g. COM5@3000000)."""
    return re.sub(r"@\d+$", "", port.strip())


def _read_until_quiet(port: serial.Serial, quiet_ms: int, total_ms: int) -> list[bytes]:
    lines: list[bytes] = []
    quiet_deadline = time.monotonic() + (quiet_ms / 1000.0)
    total_deadline = time.monotonic() + (total_ms / 1000.0)

    while time.monotonic() < total_deadline:
        raw = port.readline()
        if raw:
            quiet_deadline = time.monotonic() + (quiet_ms / 1000.0)
            lines.append(raw.rstrip(b"\r\n"))
            continue
        if time.monotonic() >= quiet_deadline:
            break
    return lines


def _is_text(raw: bytes) -> bool:
    return all((32 <= b <= 126) or b in (9,) for b in raw)


def _parse_binary_payload(raw: bytes) -> bytes | None:
    if len(raw) < 4:
        return None
    payload_size = int.from_bytes(raw[:4], byteorder="little", signed=False)
    if payload_size + 4 != len(raw):
        return None
    return raw[4:]


def _decode_git_payload(payload: bytes) -> str:
    if len(payload) != 29:
        return f"git payload unexpected length={len(payload)}"
    git_hash = payload[:20].hex()
    dirty = bool(payload[20])
    timestamp = int.from_bytes(payload[21:29], byteorder="little", signed=False)
    ts_text = "n/a"
    if timestamp:
        ts_text = datetime.datetime.fromtimestamp(
            timestamp, tz=datetime.timezone.utc
        ).isoformat()
    return (
        f"git.hash={git_hash} git.dirty={int(dirty)} "
        f"git.timestamp={timestamp} ({ts_text})"
    )


def _decode_firmware_payload(payload: bytes) -> str:
    if len(payload) != 12:
        return f"firmware payload unexpected length={len(payload)}"
    words = [
        int.from_bytes(payload[0:4], byteorder="little", signed=False),
        int.from_bytes(payload[4:8], byteorder="little", signed=False),
        int.from_bytes(payload[8:12], byteorder="little", signed=False),
    ]
    return "firmware.serial_number=" + ",".join(f"0x{word:08x}" for word in words)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial port, e.g. COM5 or COM5@3000000")
    parser.add_argument("--baud", type=int, default=115200, help="Host-side baud setting")
    parser.add_argument("--read-timeout-ms", type=int, default=120, help="Readline timeout")
    parser.add_argument("--quiet-ms", type=int, default=250, help="Stop when quiet for this long")
    parser.add_argument("--total-ms", type=int, default=2500, help="Hard stop per command")
    parser.add_argument(
        "--command",
        action="append",
        dest="commands",
        help="Command to send (can be repeated). Defaults to common probe commands.",
    )
    args = parser.parse_args()

    commands = args.commands if args.commands else list(DEFAULT_COMMANDS)
    serial_port = _normalize_port(args.port)

    print(f"# opening {serial_port} (baud={args.baud})")
    try:
        with serial.Serial(
            port=serial_port,
            baudrate=args.baud,
            timeout=max(0.01, args.read_timeout_ms / 1000.0),
            write_timeout=1.0,
        ) as port:
            port.reset_input_buffer()
            port.reset_output_buffer()

            for cmd in commands:
                print(f"\n> {cmd}")
                port.write((cmd + "\n").encode("ascii", errors="strict"))
                port.flush()

                lines = _read_until_quiet(port, args.quiet_ms, args.total_ms)
                if not lines:
                    print("< (no response)")
                    continue

                pending_emit: str | None = None
                for raw in lines:
                    if _is_text(raw):
                        text = raw.decode("ascii", errors="replace")
                        print(f"< {text}")
                        if text.startswith("emit "):
                            pending_emit = text.split(" ", 1)[1].strip()
                        else:
                            pending_emit = None
                        continue

                    print(f"< [bin] {raw.hex()}")
                    payload = _parse_binary_payload(raw)
                    if payload is None:
                        continue
                    print(f"< [bin] payload_len={len(payload)}")
                    if pending_emit == "git":
                        print(f"< [decode] {_decode_git_payload(payload)}")
                    elif pending_emit == "firmware":
                        print(f"< [decode] {_decode_firmware_payload(payload)}")
                    pending_emit = None
    except serial.SerialException as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
