"""Post-link: patch OpenBLT vector checksum into the application ELF (stdlib only).

OpenBLT (Feaser STM32 port) refuses to start the user program unless FlashVerifyChecksum()
passes: sum of the first seven 32-bit words in the vector table plus the word at offset
0x1D8 equals zero. ST-Link uploads do not run NvmDone/FlashWriteChecksum, so the
placeholder in startup_stm32g431xx_openblt.s must be replaced after link.

See bootloader/src/stm32/flash.c FlashVerifyChecksum / BOOT_FLASH_VECTOR_TABLE_CS_OFFSET.
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

ELFMAG = b"\x7fELF"
EI_CLASS = 4
ELFCLASS32 = 1
PT_LOAD = 1
BOOT_FLASH_VECTOR_TABLE_CS_OFFSET = 0x1D8


def _u32(data: bytes, off: int) -> int:
    return struct.unpack_from("<I", data, off)[0]


def _patch_elf32_phdrs(data: bytearray, app_flash_base: int) -> bool:
    if data[:4] != ELFMAG or len(data) < 52:
        return False
    if data[EI_CLASS] != ELFCLASS32:
        return False
    e_phoff = _u32(data, 28)
    e_phentsize = struct.unpack_from("<H", data, 42)[0]
    e_phnum = struct.unpack_from("<H", data, 44)[0]
    if e_phentsize < 32 or e_phnum == 0:
        return False

    vec_lo = app_flash_base

    vec_file_off: int | None = None
    cs_file_off: int | None = None

    for i in range(e_phnum):
        p = e_phoff + i * e_phentsize
        if p + 32 > len(data):
            return False
        p_type = _u32(data, p)
        if p_type != PT_LOAD:
            continue
        p_offset = _u32(data, p + 4)
        p_vaddr = _u32(data, p + 8)
        p_filesz = _u32(data, p + 16)
        seg_end = p_vaddr + p_filesz
        if p_vaddr <= vec_lo < seg_end:
            vec_file_off = p_offset + (vec_lo - p_vaddr)
        if p_vaddr <= app_flash_base + BOOT_FLASH_VECTOR_TABLE_CS_OFFSET < seg_end:
            cs_file_off = p_offset + (app_flash_base + BOOT_FLASH_VECTOR_TABLE_CS_OFFSET - p_vaddr)

    if vec_file_off is None or cs_file_off is None:
        return False
    if vec_file_off + 0x1C > len(data) or cs_file_off + 4 > len(data):
        return False

    s = 0
    for k in range(7):
        s = (s + _u32(data, vec_file_off + 4 * k)) & 0xFFFFFFFF
    chk = ((~s) + 1) & 0xFFFFFFFF
    struct.pack_into("<I", data, cs_file_off, chk)
    return True


def patch_elf(elf_path: Path, app_flash_base: int) -> None:
    raw = bytearray(elf_path.read_bytes())
    if not _patch_elf32_phdrs(raw, app_flash_base):
        raise SystemExit(
            f"patch_openblt_vector_checksum: could not locate PT_LOAD for "
            f"0x{app_flash_base:08X} in {elf_path}"
        )
    elf_path.write_bytes(raw)


def _app_base_from_env(env) -> int:
    v = env.GetProjectOption("board_upload.offset_address", "").strip()
    if not v:
        return 0
    return int(v, 0)


def _should_patch(env) -> bool:
    ld = env.GetProjectOption("board_build.ldscript", "")
    return "OPENBLT" in ld.upper()


# Manual: python patch_openblt_vector_checksum.py <firmware.elf> 0x08005000
# (PlatformIO also runs this script with argv [script] only.)
if len(sys.argv) == 3 and sys.argv[1].lower().endswith(".elf"):
    if not Path(sys.argv[1]).is_file():
        print("patch_openblt_vector_checksum: file not found:", sys.argv[1], file=sys.stderr)
        raise SystemExit(2)
    patch_elf(Path(sys.argv[1]), int(sys.argv[2], 0))
    raise SystemExit(0)

Import("env")  # type: ignore[name-defined]


if _should_patch(env):
    _app_base = _app_base_from_env(env)
    if _app_base != 0:

        def _post_elf(source, target, env):  # noqa: ARG001
            elf = Path(str(target[0]))
            if elf.suffix.lower() != ".elf" or not elf.is_file():
                return
            patch_elf(elf, _app_base)
            print(
                f"patch_openblt_vector_checksum: patched {elf.name} "
                f"(app @ 0x{_app_base:08X})"
            )

        env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", _post_elf)
