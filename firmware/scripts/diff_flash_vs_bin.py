"""Dump app flash via openocd ST-Link, compare to firmware.bin.

By default reads .pio/build/<env>/firmware.bin where <env> is taken from --env (defaults to
moteus). Run with --env moteus_openblt_can right after a CAN upload so the reference bin
matches what the CAN path actually programmed.

Also prints the first 16 bytes of both env builds when both exist, so you can sanity-check
that the two envs are producing the same binary.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


APP_BASE = 0x08005000
SCRIPT_DIR = Path(__file__).resolve().parent
FIRMWARE_DIR = SCRIPT_DIR.parent


def find_openocd() -> Path:
    pio_pkg = Path.home() / ".platformio" / "packages" / "tool-openocd" / "bin"
    for name in ("openocd.exe", "openocd"):
        p = pio_pkg / name
        if p.is_file():
            return p
    found = shutil.which("openocd")
    if found:
        return Path(found)
    raise SystemExit("openocd not found")


def dump_flash(openocd: Path, dump_path: Path, length: int) -> None:
    scripts = openocd.parent.parent / "openocd" / "scripts"
    cmd = [
        str(openocd),
        "-s", str(scripts),
        "-f", "interface/stlink.cfg",
        "-f", "target/stm32g4x.cfg",
        "-c", f"init; halt; dump_image {dump_path.as_posix()} 0x{APP_BASE:08X} {length}; resume; exit",
    ]
    print("+", " ".join(cmd))
    r = subprocess.run(cmd, check=False)
    if r.returncode != 0:
        raise SystemExit(r.returncode)


def diff(bin_a: Path, bin_b: Path) -> None:
    a = bin_a.read_bytes()
    b = bin_b.read_bytes()
    n = min(len(a), len(b))
    diffs = [i for i in range(n) if a[i] != b[i]]
    print(f"\nreference:  {bin_a} ({len(a)} bytes)")
    print(f"flash dump: {bin_b} ({len(b)} bytes)")
    print(f"diff count: {len(diffs)} of {n} compared")
    if not diffs:
        print("Flash matches reference bin.")
        return
    first = diffs[0]
    addr = APP_BASE + first
    lo = max(0, first - 8)
    hi = min(n, first + 24)
    print(f"\nFirst diff at file offset 0x{first:04X} (flash addr 0x{addr:08X})")
    print(f"  ref:   {a[lo:hi].hex(' ')}")
    print(f"  flash: {b[lo:hi].hex(' ')}")

    # Coverage map: split flash into 63-byte windows (matches XCP PROGRAM_MAX payload).
    # For each window, count how many bytes match. If the bug is "6 real bytes per frame
    # then zeros", windows will have ~6 matching bytes (plus coincidental zero matches).
    print(f"\nCoverage map by 63-byte windows (the PROGRAM_MAX chunk size):")
    print(f"  win | offset    | match/63 | first8 ref           | first8 flash")
    print(f"  ----+-----------+----------+----------------------+---------------------")
    window = 63
    max_print = 24
    diff_pattern = {}
    n_windows = (n + window - 1) // window
    for w in range(n_windows):
        lo_w = w * window
        hi_w = min(n, lo_w + window)
        matches = sum(1 for i in range(lo_w, hi_w) if a[i] == b[i])
        diff_pattern[matches] = diff_pattern.get(matches, 0) + 1
        if w < max_print:
            ref_hex = a[lo_w:lo_w + 8].hex(' ')
            flash_hex = b[lo_w:lo_w + 8].hex(' ')
            print(f"  {w:3d} | 0x{lo_w:08X}| {matches:>3}/{hi_w-lo_w}    | {ref_hex} | {flash_hex}")
    if n_windows > max_print:
        print(f"  ... ({n_windows - max_print} more windows omitted)")
    print(f"\nHistogram of matching bytes per 63-byte window (across all {n_windows} windows):")
    for matches in sorted(diff_pattern):
        print(f"  {matches:>3} matching bytes  -> {diff_pattern[matches]:>4} windows")


def cross_env_check() -> None:
    moteus = FIRMWARE_DIR / ".pio" / "build" / "moteus" / "firmware.bin"
    can = FIRMWARE_DIR / ".pio" / "build" / "moteus_openblt_can" / "firmware.bin"
    if not (moteus.is_file() and can.is_file()):
        return
    a = moteus.read_bytes()
    b = can.read_bytes()
    print(f"\nCross-env sanity check:")
    print(f"  moteus/firmware.bin             ({len(a):>6} bytes)  first 16: {a[:16].hex(' ')}")
    print(f"  moteus_openblt_can/firmware.bin ({len(b):>6} bytes)  first 16: {b[:16].hex(' ')}")
    if len(a) != len(b):
        print(f"  -> different sizes! at least one env is stale; pio run -e <name> -t upload would")
        print(f"     program the older bin. Force a clean rebuild before flashing.")
    elif a != b:
        first = next(i for i in range(len(a)) if a[i] != b[i])
        print(f"  -> same size but differ at offset 0x{first:04X}; one env has a stale build artifact")
    else:
        print(f"  -> same bytes. envs are in sync.")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--env", default="moteus", help="PIO env whose bin to compare against (default: moteus)")
    p.add_argument("--bin", type=Path, help="Path to bin file (overrides --env)")
    args = p.parse_args()

    if args.bin is not None:
        ref = args.bin
    else:
        ref = FIRMWARE_DIR / ".pio" / "build" / args.env / "firmware.bin"
    if not ref.is_file():
        print(f"reference bin not found: {ref}", file=sys.stderr)
        return 1

    openocd = find_openocd()
    with tempfile.TemporaryDirectory() as td:
        dump = Path(td) / "flash_dump.bin"
        dump_flash(openocd, dump, ref.stat().st_size)
        diff(ref, dump)
    cross_env_check()
    return 0


if __name__ == "__main__":
    sys.exit(main())
