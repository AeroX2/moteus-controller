"""
Pre-build script: apply SimpleFOC alignment patch via patch_ng.

Finds .pio/libdeps/<env>/Simple FOC/, applies scripts/simplefoc_alignment.patch,
and skips if already patched (FOCMotor.cpp contains the patched alignment block).

PlatformIO extra_scripts cannot pass arguments (e.g. patch path, -d, -p), so this
wrapper finds the library and applies the patch; you cannot use just
  extra_scripts = pre:scripts/patch_ng.py
with the patch path in platformio.ini.
"""
import os
import glob
import sys

try:
    _script_dir = os.path.dirname(os.path.abspath(__file__))
except NameError:
    _script_dir = os.path.join(os.getcwd(), "scripts")
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)
import patch_ng

# Marker in patched source (from simplefoc-alignment.patch)
ALREADY_PATCHED_MARKER = "alignment_speed_elec_rev_per_s"
PATCH_FILE = "simplefoc_alignment.patch"


def main():
    # Emit a banner every build so it's obvious this ran.
    print("[apply_simplefoc] Running pre-build patch check...", flush=True)
    script_dir = _script_dir
    project_dir = os.path.dirname(script_dir)
    libdeps = os.path.join(project_dir, ".pio", "libdeps")
    if not os.path.isdir(libdeps):
        print("[apply_simplefoc] .pio/libdeps not found (run build once to install deps)", flush=True)
        return

    # Prefer the active PlatformIO environment's libdeps folder, so we patch
    # exactly what the build uses (and not some other cached env).
    pioenv = None
    try:
        if "Import" in globals():
            Import("env")  # type: ignore[name-defined]  # provided by PlatformIO
            pioenv = env.get("PIOENV")  # type: ignore[name-defined]
    except Exception:
        pioenv = None

    candidates = []
    if pioenv:
        candidates.append(os.path.join(libdeps, str(pioenv), "Simple FOC"))
        candidates.extend(glob.glob(os.path.join(libdeps, str(pioenv), "Simple*FOC*")))

    if not candidates:
        candidates = glob.glob(os.path.join(libdeps, "*", "Simple FOC"))
        if not candidates:
            candidates = glob.glob(os.path.join(libdeps, "*", "Simple*FOC*"))

    simple_foc_roots = [d for d in candidates if os.path.isdir(d)]
    if not simple_foc_roots:
        print("[apply_simplefoc] Simple FOC not found under .pio/libdeps", flush=True)
        return

    simple_foc_root = os.path.abspath(simple_foc_roots[0])
    print(f"[apply_simplefoc] Using SimpleFOC root: {simple_foc_root}", flush=True)
    focmotor = os.path.join(simple_foc_root, "src", "common", "base_classes", "FOCMotor.cpp")
    if not os.path.isfile(focmotor):
        print("[apply_simplefoc] FOCMotor.cpp not found under", simple_foc_roots[0], flush=True)
        return

    with open(focmotor, "r", encoding="utf-8", errors="replace") as f:
        if ALREADY_PATCHED_MARKER in f.read():
            print("[apply_simplefoc] Already patched:", focmotor, flush=True)
            return

    patch_path = os.path.join(script_dir, PATCH_FILE)
    if not os.path.isfile(patch_path):
        print("[apply_simplefoc] Patch not found:", patch_path, flush=True)
        sys.exit(1)

    with open(focmotor, "rb") as f:
        before_bytes = f.read()

    with open(patch_path, "rb") as f:
        ps = patch_ng.PatchSet()
        ps.parse(f)
    if not getattr(ps, "items", None):
        print("[apply_simplefoc] Patch parse produced no items", flush=True)
        sys.exit(1)

    ok = ps.apply(strip=1, root=simple_foc_root)
    if not ok:
        print("[apply_simplefoc] patch_ng apply failed", flush=True)
        sys.exit(1)
    with open(focmotor, "rb") as f:
        after_bytes = f.read()
    if before_bytes == after_bytes:
        print("[apply_simplefoc] Patch reported success but file unchanged (likely hunk mismatch)", flush=True)
        sys.exit(1)
    # Sanity check: ensure the marker made it into the file so we don't get
    # a false-positive "patched" message when upstream changes break the patch.
    with open(focmotor, "r", encoding="utf-8", errors="replace") as f:
        if ALREADY_PATCHED_MARKER not in f.read():
            print("[apply_simplefoc] Patch applied but marker not found; patch likely mismatched upstream SimpleFOC version", flush=True)
            sys.exit(1)
    print("[apply_simplefoc] Patched:", focmotor, flush=True)


def _running_under_platformio() -> bool:
    # PlatformIO/SCons provides Import() in the script globals.
    return "Import" in globals()


if _running_under_platformio() or __name__ == "__main__":
    main()
