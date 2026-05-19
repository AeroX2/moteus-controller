# moteus-controller

A custom servo BLDC driver loosely based on the [mjbots Moteus r4.11](https://github.com/mjbots/moteus/tree/main/hw/controller/r4.11) hardware.

Main differences from upstream Moteus r4.11:

- **STSPIN32G4** integrated gate driver + STM32G431 instead of separate DRV8353S and MCU.
- Schematic and BOM trimmed/substituted for **JLCPCB** assembly.
- **AS5048A** magnetic encoder.
- **MAX3051** CAN transceiver.
- Firmware built on **SimpleFOC** + Arduino-stm32duino instead of the mjbots firmware.

## Repository layout

```
moteus-controller/
├── pcb/                # KiCad schematic + PCB
├── firmware/           # SimpleFOC-based motor controller firmware (PlatformIO + Arduino)
├── bootloader/         # OpenBLT bootloader for STM32G431VB  (submodule -> AeroX2/OpenBLT-STM32)
└── openblt/            # OpenBLT library + mjbots fdcanusb driver (submodule -> AeroX2/openblt)
```

`firmware/`, `bootloader/`, and `openblt/` each have their own build setup and README.

## Quick build / flash

Prerequisites: [PlatformIO Core](https://platformio.org/install/cli), an ST-Link, optionally a [mjbots fdcanusb](https://github.com/mjbots/fdcanusb) for CAN-FD uploads.

Clone with submodules:

```bash
git clone --recurse-submodules https://github.com/AeroX2/moteus-controller.git
cd moteus-controller
```

Flash the bootloader once (writes at `0x08000000`, ~8 KB):

```bash
pio run -d bootloader -e moteus_g431 -t upload
```

Flash the application (writes at `0x08005000`):

```bash
pio run -d firmware -e moteus -t upload
```

Power-cycle the board; the bootloader runs for ~3 s, then jumps to the app.

After the bootloader is in place, subsequent firmware uploads can go over CAN-FD via fdcanusb (no ST-Link needed):

```bash
pio run -d firmware -e moteus_openblt_can -t upload
```

## Bootloader

`bootloader/` is a bare-metal **stm32cube + PlatformIO** port of the upstream [Feaser OpenBLT](https://github.com/feaser/openblt) Nucleo-G431 demo, adapted for the moteus-controller hardware:

- HSI → PLL → 170 MHz (no HSE crystal on this board).
- FDCAN1 routed to PA11/PA12 (vs Nucleo's PB8/PB9).
- 20 KB reserved at `0x08000000`; app starts at `0x08005000`; the final 2 KB flash page (`0x0801F800`) is reserved for application PID storage and never touched by the bootloader.

OpenBLT library + STM32G4 port sources are compiled directly from the `openblt/` submodule by `bootloader/src/build.py` — there are no local copies of those files in `bootloader/`, so updating to a newer OpenBLT is just a submodule bump.

## CAN-FD upload (`moteus_openblt_can` env)

The CAN-FD upload path uses a custom **mjbots fdcanusb** host driver added to LibOpenBLT (see `openblt/Host/Source/LibOpenBLT/port/windows/canif/fdcanusb/`) plus a small wrapper (`firmware/host_openblt_flash/`) and a Python harness (`firmware/scripts/openblt_host_upload.py`) that handles pre-reset, device discovery, and srec conversion.

Build the host tool once:

```bash
python firmware/scripts/build_host_openblt_flash.py
```

Then `pio run -d firmware -e moteus_openblt_can -t upload` does the rest.

Tuning knobs live in `firmware/platformio.ini` under `[env:moteus_openblt_can]`:

- `custom_openblt_can_device` — fdcanusb COM port and USB baud.
- `custom_openblt_can_tid` / `custom_openblt_can_rid` — XCP CAN IDs (default `667` / `7E1`, must match `bootloader/src/blt_conf.h`).
- `custom_openblt_reset_discover` / `custom_openblt_reset_target_id` — control how the running application is asked to reboot into the bootloader before uploading.

## License

The original mjbots Moteus design is licensed under Apache 2.0; see `LICENSE`. OpenBLT and its derivatives are GPLv3 (see `openblt/Doc/license.html`).
