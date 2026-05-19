"""Wipe the STM32G431 to a clean factory state via ST-Link/openocd.

Runs `stm32l4x mass_erase 0` (the openocd command also covers G4). After this the chip
flash is uniformly 0xFF and you must reflash the bootloader before anything else.

Diagnostic use: rules out "CAN upload programs over a stale half-erased image" by giving
the next upload a clean slate to work against.

Run from firmware/:
    python scripts/mass_erase.py
"""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path


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


def main() -> int:
    openocd = find_openocd()
    scripts = openocd.parent.parent / "openocd" / "scripts"
    cmd = [
        str(openocd),
        "-s", str(scripts),
        "-f", "interface/stlink.cfg",
        "-f", "target/stm32g4x.cfg",
        "-c", "init; reset halt; stm32l4x mass_erase 0; exit",
    ]
    print("+", " ".join(cmd))
    r = subprocess.run(cmd, check=False)
    if r.returncode == 0:
        print("\nMass erase complete. Chip flash is now 0xFF everywhere.")
        print("Next: pio run -d ../bootloader -e moteus_g431 -t upload")
    return r.returncode


if __name__ == "__main__":
    sys.exit(main())
