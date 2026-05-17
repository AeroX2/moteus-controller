#!/usr/bin/env python3
"""Upload OpenBLT application firmware using openblt_flash (LibOpenBLT only).

Expects a linked ELF (VMAs correct for the OpenBLT app region). Converts to Motorola S-record
with arm-none-eabi-objcopy, then invokes the host tool.

Configuration (environment variables):

  LibOpenBLT (host_openblt_flash — build manually with CMake; Linux can0 + fdcanusb gs_usb):
  OPENBLT_LIBOPENBLT_FLASH   Path to openblt_flash executable (optional if host_openblt_flash/build/... exists)
  OPENBLT_LIBOPENBLT_TRANSPORT   can (default) or rs232
  OPENBLT_UART_PORT          For rs232: COM7 or /dev/ttyUSB0
  OPENBLT_UART_BAUD          For rs232: default 57600

  OPENBLT_CAN_DEVICE       Mandatory for CAN, e.g. mjbots_fdcanusb:COM7 or can0
  OPENBLT_CAN_TID          Optional hex host->target ID (default 667)
  OPENBLT_CAN_RID          Optional hex target->host ID (default 7E1)

PlatformIO: use env:moteus_openblt_can (extends env:moteus) and set custom_openblt_* in platformio.ini; see pio_openblt_can_env.py.

  Pre-reset (Commander \"R\" over fdcanusb serial) — same COM as mjbots_fdcanusb:COMx unless OPENBLT_RESET_COM_PORT is set:
  OPENBLT_RESET_DISCOVER             If 1/true: sniff the serial stream (like can_monitor.py) for
                                     rcv <id> frames whose first data byte is 0x00/0x01/0x03 (motion/status/init),
                                     then send pre-reset to each unique id. OPENBLT_RESET_TARGET_ID is ignored.
  OPENBLT_RESET_DISCOVER_SECONDS    Listen window in seconds (default 1.5).
  OPENBLT_RESET_DISCOVER_EXCLUDE    Comma-separated hex ids to skip (default 7E1,667 — OpenBLT / host traffic).
  OPENBLT_RESET_TARGET_ID           If not using discover: one or more ids (comma-separated) for pre-reset only.

  openblt_flash (rebuilt host tool): OPENBLT_CONNECT_WAIT_MS — max time to retry BltSessionStart (default 120000).
"""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path

def _default_libopenblt_flash() -> Path | None:
    """Pick newest openblt_flash under host_openblt_flash/ (avoids stale build_mingw shadowing build/)."""
    fw = Path(__file__).resolve().parent.parent
    found: list[Path] = []
    for sub in ("build_mingw", "build"):
        root = fw / "host_openblt_flash" / sub
        for name in ("openblt_flash.exe", "openblt_flash"):
            p = root / name
            if p.is_file():
                found.append(p)
    if not found:
        return None
    found.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return found[0]


def find_toolchain_bin() -> Path | None:
    home = Path.home()
    packages = home / ".platformio" / "packages"
    candidates = [
        packages / "toolchain-gccarmnoneeabi" / "bin",
        packages / "toolchain-gccarmnoneeabi@1.120301.0" / "bin",
    ]
    for base in candidates:
        if (base / "arm-none-eabi-gcc.exe").is_file() or (base / "arm-none-eabi-gcc").is_file():
            return base
    return None


def _objcopy_exe(tool_bin: Path) -> Path:
    name = "arm-none-eabi-objcopy.exe" if sys.platform == "win32" else "arm-none-eabi-objcopy"
    return tool_bin / name


def elf_to_srec(objcopy: Path, elf: Path, srec: Path) -> None:
    cmd = [str(objcopy), "-O", "srec", str(elf), str(srec)]
    print("+", " ".join(cmd))
    r = subprocess.run(cmd, check=False)
    if r.returncode != 0:
        raise SystemExit(r.returncode)


def _env_str(name: str, default: str = "") -> str:
    v = os.environ.get(name, default)
    return v.strip() if isinstance(v, str) else default


def _env_bool(name: str, default: bool = False) -> bool:
    v = _env_str(name, "1" if default else "0").lower()
    return v in ("1", "true", "yes", "on")


def _parse_rcv_device_id(line: str) -> str | None:
    """Match can_monitor.parse_message: moteus telemetry on fdcanusb as `rcv <id> <datahex> ...`."""
    parts = line.strip().split()
    if len(parts) < 3 or parts[0] != "rcv":
        return None
    can_id = parts[1]
    if not can_id or not all(c in "0123456789abcdefABCDEF" for c in can_id):
        return None
    try:
        data_bytes = bytes.fromhex(parts[2])
    except ValueError:
        return None
    if not data_bytes:
        return None
    if data_bytes[0] in (0x00, 0x01, 0x03):
        return can_id
    return None


def _parse_exclude_ids(raw: str) -> set[str]:
    out: set[str] = set()
    for chunk in raw.replace(";", ",").split(","):
        t = chunk.strip()
        if t:
            out.add(t.upper())
    return out


def _parse_reset_target_ids(raw: str) -> list[str]:
    return [x.strip() for x in raw.replace(";", ",").split(",") if x.strip()]


def _normalize_com_port(raw: str) -> str:
    """Normalize fdcanusb COM notation, allowing optional @baud suffix."""
    s = raw.strip()
    if not s:
        return s
    if "@" in s:
        s = s.split("@", 1)[0].strip()
    return s


def _discover_moteus_ids_on_serial(com: str, seconds: float, exclude: set[str]) -> list[str]:
    """Listen on fdcanusb serial for telemetry-like frames; return sorted unique CAN ids (hex strings)."""
    try:
        import serial  # pyserial
    except Exception:
        print(
            "Reset discover requires pyserial (`pip install pyserial`).",
            file=sys.stderr,
        )
        raise SystemExit(2)

    seen: set[str] = set()
    deadline = time.monotonic() + max(0.1, seconds)
    try:
        with serial.Serial(com, 115200, timeout=0.05, write_timeout=1.0) as ser:
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
            while time.monotonic() < deadline:
                line = ser.readline().decode("utf-8", errors="ignore")
                if not line:
                    continue
                cid = _parse_rcv_device_id(line)
                if cid and cid.upper() not in exclude:
                    seen.add(cid)
    except PermissionError as e:
        print(
            f"Reset discover: cannot open {com} — port is in use (often can_monitor.py or "
            f"another tool holding fdcanusb; only one program can open the COM port).\n"
            f"  {e}",
            file=sys.stderr,
        )
        raise SystemExit(2) from e
    except OSError as e:
        if getattr(e, "errno", None) in (13, 16) or "Access is denied" in str(e):
            print(
                f"Reset discover: cannot open {com} — access denied (close can_monitor / "
                f"any app using this fdcanusb serial port).\n"
                f"  {e}",
                file=sys.stderr,
            )
            raise SystemExit(2) from e
        print(f"Reset discover failed on {com}: {e}", file=sys.stderr)
        raise SystemExit(2) from e
    except Exception as e:
        print(f"Reset discover failed on {com}: {e}", file=sys.stderr)
        raise SystemExit(2) from e

    def _sort_key(x: str) -> tuple[int, str]:
        try:
            return (int(x, 16), x)
        except ValueError:
            return (0, x)

    return sorted(seen, key=_sort_key)


def _maybe_send_pre_reset() -> None:
    """Optionally send Commander 'R' over fdcanusb serial before launching flasher."""
    if not _env_bool("OPENBLT_PRE_RESET_R", False):
        return

    com = _env_str("OPENBLT_RESET_COM_PORT")
    if not com:
        # Convenience: derive COM from mjbots_fdcanusb:COMx device setting.
        dev = _env_str("OPENBLT_CAN_DEVICE")
        if dev.lower().startswith("mjbots_fdcanusb:"):
            com = dev.split(":", 1)[1].strip()
    com = _normalize_com_port(com)
    if not com:
        print(
            "Pre-reset enabled but no COM port set. Set OPENBLT_RESET_COM_PORT "
            "(or custom_openblt_reset_com_port).",
            file=sys.stderr,
        )
        raise SystemExit(2)

    discover = _env_bool("OPENBLT_RESET_DISCOVER", False)
    target_ids: list[str] = []
    if discover:
        sec_s = _env_str("OPENBLT_RESET_DISCOVER_SECONDS")
        try:
            listen_s = 1.5 if not sec_s else max(0.1, float(sec_s))
        except ValueError:
            print("OPENBLT_RESET_DISCOVER_SECONDS must be a float.", file=sys.stderr)
            raise SystemExit(2)
        ex_default = "7E1,667"
        ex_raw = _env_str("OPENBLT_RESET_DISCOVER_EXCLUDE", ex_default)
        exclude = _parse_exclude_ids(ex_raw)
        print(
            f"openblt_host_upload: discovering CAN ids on {com} for {listen_s:.2f}s "
            f"(exclude {','.join(sorted(exclude))})..."
        )
        target_ids = _discover_moteus_ids_on_serial(com, listen_s, exclude)
        if not target_ids:
            print(
                "openblt_host_upload: no devices discovered (telemetry quiet?). "
                "Continuing upload without pre-reset.",
                file=sys.stderr,
            )
            return
        print("openblt_host_upload: pre-reset targets:", ", ".join(target_ids))
    else:
        raw = _env_str("OPENBLT_RESET_TARGET_ID")
        if not raw:
            print(
                "Pre-reset enabled but no target CAN ID set. Set OPENBLT_RESET_TARGET_ID "
                "(or custom_openblt_reset_target_id), or set OPENBLT_RESET_DISCOVER=1 "
                "(custom_openblt_reset_discover) to sniff fdcanusb like can_monitor.",
                file=sys.stderr,
            )
            raise SystemExit(2)
        target_ids = _parse_reset_target_ids(raw)

    try:
        import serial  # pyserial
    except Exception:
        print(
            "Pre-reset requires pyserial (`pip install pyserial`) to write fdcanusb serial.",
            file=sys.stderr,
        )
        raise SystemExit(2)

    try:
        with serial.Serial(com, 115200, timeout=0.2, write_timeout=1.0) as ser:
            for target_id in target_ids:
                # 'R' + NUL for Commander parser.
                line = f"can send {target_id} 5200 Fb\n"
                ser.write(line.encode("ascii"))
                ser.flush()
    except PermissionError as e:
        print(
            f"Pre-reset: cannot open {com} — port in use; close can_monitor (or any fdcanusb "
            f"serial user) before upload.\n  {e}",
            file=sys.stderr,
        )
        raise SystemExit(2) from e
    except OSError as e:
        if getattr(e, "errno", None) in (13, 16) or "Access is denied" in str(e):
            print(
                f"Pre-reset: cannot open {com} — access denied; close other apps using this COM port.\n"
                f"  {e}",
                file=sys.stderr,
            )
            raise SystemExit(2) from e
        print(f"Failed to send pre-reset on {com}: {e}", file=sys.stderr)
        raise SystemExit(2) from e
    except Exception as e:
        print(f"Failed to send pre-reset on {com}: {e}", file=sys.stderr)
        raise SystemExit(2) from e

    delay_s = 0.5
    d = _env_str("OPENBLT_RESET_DELAY")
    if d:
        try:
            delay_s = max(0.0, float(d))
        except ValueError:
            print("OPENBLT_RESET_DELAY must be a float seconds value.", file=sys.stderr)
            raise SystemExit(2)
    if delay_s > 0:
        time.sleep(delay_s)

def _libopenblt_flash_cmd(srec: Path) -> list[str]:
    exe = _env_str("OPENBLT_LIBOPENBLT_FLASH")
    if not exe:
        fb = _default_libopenblt_flash()
        exe = str(fb) if fb else ""
    if not exe:
        fw = Path(__file__).resolve().parent.parent
        hf = fw / "host_openblt_flash"
        print(
            "openblt_flash not found. Build with CMake; look for openblt_flash.exe under host_openblt_flash/build/ or build_mingw/ (upload picks newest).\n"
            f"  Linux: cmake -S {hf} -B {hf / 'build'} -G Ninja && cmake --build {hf / 'build'} --parallel\n"
            "  Windows + MSYS2 UCRT64: open MSYS2 UCRT64, cd to firmware/host_openblt_flash, then:\n"
            "    mkdir -p build_mingw && cd build_mingw && cmake .. -G \"MinGW Makefiles\" && cmake --build . --parallel\n"
            "  Or from PowerShell (PATH so gcc finds DLLs; use full path to CMake if not on PATH):\n"
            '    C:\\msys64\\usr\\bin\\bash.exe -lc "export PATH=/ucrt64/bin:/usr/bin:/bin && cd /c/.../firmware/host_openblt_flash && mkdir -p build_mingw && cd build_mingw && cmake .. -G MinGW Makefiles && cmake --build . --parallel"\n'
            "or set OPENBLT_LIBOPENBLT_FLASH / custom_openblt_libopenblt_flash.",
            file=sys.stderr,
        )
        raise SystemExit(2)
    transport = _env_str("OPENBLT_LIBOPENBLT_TRANSPORT", "can").lower()
    cmd = [exe]
    if transport == "rs232":
        port = _env_str("OPENBLT_UART_PORT")
        if not port:
            print(
                "RS232 mode requires OPENBLT_UART_PORT (or custom_openblt_uart_port).",
                file=sys.stderr,
            )
            raise SystemExit(2)
        cmd.extend(["-t", "rs232", "-p", port])
        ub = _env_str("OPENBLT_UART_BAUD")
        if ub:
            cmd.extend(["-ub", ub])
    else:
        device = _env_str("OPENBLT_CAN_DEVICE")
        if not device:
            print(
                "Set OPENBLT_CAN_DEVICE (e.g. can0 or mjbots_fdcanusb:COM7), or custom_openblt_can_device.",
                file=sys.stderr,
            )
            raise SystemExit(2)
        cmd.extend(["-d", device])
        tid = _env_str("OPENBLT_CAN_TID")
        if tid:
            cmd.extend(["-tid", tid])
        rid = _env_str("OPENBLT_CAN_RID")
        if rid:
            cmd.extend(["-rid", rid])
    cmd.append(str(srec))
    return cmd


def main() -> int:
    p = argparse.ArgumentParser(description="Upload firmware via openblt_flash (LibOpenBLT).")
    p.add_argument("elf", type=Path, help="Path to application .elf")
    args = p.parse_args()
    elf = args.elf.resolve()
    if not elf.is_file():
        print("ELF not found:", elf, file=sys.stderr)
        return 1
    print(f"openblt_host_upload: {elf}", flush=True)

    tool_bin = find_toolchain_bin()
    if tool_bin is None:
        print(
            "arm-none-eabi-objcopy not found under ~/.platformio/packages/toolchain-gccarmnoneeabi.",
            file=sys.stderr,
        )
        return 1
    objcopy = _objcopy_exe(tool_bin)
    if not objcopy.is_file():
        print("Missing:", objcopy, file=sys.stderr)
        return 1

    srec = elf.with_name(f"{elf.stem}_openblt_host.srec")
    try:
        elf_to_srec(objcopy, elf, srec)
    except SystemExit as e:
        return int(e.code) if e.code is not None else 1
    print(f"openblt_host_upload: wrote {srec.name}", flush=True)

    try:
        _maybe_send_pre_reset()
    except SystemExit as e:
        return int(e.code) if e.code is not None else 2
    print(
        "openblt_host_upload: starting openblt_flash (long waits = bootloader not reachable on CAN; "
        "see OPENBLT_CONNECT_WAIT_MS)",
        flush=True,
    )

    cmd = _libopenblt_flash_cmd(srec)

    # Feaser tools load interface DLLs from the Host directory; cwd avoids PATH issues.
    cwd = str(Path(cmd[0]).resolve().parent)

    print("+", " ".join(shlex.quote(c) for c in cmd), flush=True)
    try:
        r = subprocess.run(
            cmd,
            cwd=cwd,
            stdin=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        print("Executable not found (check path):", cmd[0], file=sys.stderr)
        return 127
    return r.returncode if r.returncode is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())
