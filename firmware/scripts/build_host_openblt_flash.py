#!/usr/bin/env python3
"""Configure and build host_openblt_flash (LibOpenBLT host tool).

On Windows, Visual Studio / standalone CMake plus MinGW often mis-detects or fails to run
the compiler when Ninja invokes gcc (PATH, pyenv shims for ``ninja``, etc.). This script
builds inside an MSYS2 UCRT64 environment, which matches how the project is tested.

Install (once) in MSYS2 UCRT64, for example::

    pacman -Syu
    pacman -S --needed base-devel mingw-w64-ucrt-x86_64-toolchain \\
        mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-ninja

Optional: set MSYS2_ROOT if MSYS2 is not installed at C:\\msys64.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path


def _firmware_dir() -> Path:
    return Path(__file__).resolve().parent.parent


def _find_msys2_root() -> Path | None:
    env = os.environ.get("MSYS2_ROOT", "").strip()
    if env:
        p = Path(env)
        if (p / "msys2_shell.cmd").is_file():
            return p
    default = Path(r"C:\msys64")
    if (default / "msys2_shell.cmd").is_file():
        return default
    return None


def _run_unix(build_dir: Path, host_dir: Path) -> int:
    build_dir.mkdir(parents=True, exist_ok=True)
    ninja = shutil.which("ninja")
    gen = ["-G", "Ninja"] if ninja else []
    try:
        subprocess.run(["cmake", str(host_dir), *gen], cwd=build_dir, check=True)
        subprocess.run(["cmake", "--build", "."], cwd=build_dir, check=True)
    except (OSError, subprocess.CalledProcessError):
        return 1
    return 0


def _run_windows_msys2(msys2: Path, build_dir: Path, host_dir: Path) -> int:
    build_dir.mkdir(parents=True, exist_ok=True)
    build_u = str(build_dir.resolve()).replace("\\", "/")
    host_u = str(host_dir.resolve()).replace("\\", "/")
    inner = f"cd '{build_u}' && cmake '{host_u}' -G Ninja && cmake --build ."
    cmd = [
        str(msys2 / "msys2_shell.cmd"),
        "-defterm",
        "-no-start",
        "-ucrt64",
        "-c",
        inner,
    ]
    return subprocess.call(cmd)


def main() -> int:
    firmware = _firmware_dir()
    host_dir = firmware / "host_openblt_flash"
    build_dir = host_dir / "build"
    if not (host_dir / "CMakeLists.txt").is_file():
        print(f"Missing {host_dir / 'CMakeLists.txt'}", file=sys.stderr)
        return 2

    if sys.platform == "win32":
        msys2 = _find_msys2_root()
        if not msys2:
            print(
                "MSYS2 not found (expected msys2_shell.cmd under MSYS2_ROOT or C:\\msys64).\n"
                "Install MSYS2, then in an UCRT64 shell: pacman -S --needed base-devel "
                "mingw-w64-ucrt-x86_64-toolchain mingw-w64-ucrt-x86_64-cmake "
                "mingw-w64-ucrt-x86_64-ninja",
                file=sys.stderr,
            )
            return 2
        return _run_windows_msys2(msys2, build_dir, host_dir)

    return _run_unix(build_dir, host_dir)


if __name__ == "__main__":
    raise SystemExit(main())
