/* Minimal OpenBLT host flasher using LibOpenBLT (same flow as BootCommander).
 * License: GNU GPL v3+ (same as OpenBLT / Feaser LibOpenBLT).
 *
 * Linux + SocketCAN: bring can0 up for 1 Mbit/s CAN FD without BRS (match bootloader
 *   / application), then e.g. ./openblt_flash -d can0 firmware_openblt_host.srec
 *
 * Windows: supports standard LibOpenBLT CAN backends and mjbots fdcanusb via:
 *   -d mjbots_fdcanusb:COM7
 * Or use -t rs232 -p COM7 for XCP-on-UART.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>
#include <stdint.h>

#include "openblt.h"

#if defined(_WIN32)
#include <windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")
#endif

/* Pick a WriteData chunk that minimises XCP framing overhead. Using a multiple
 * of (maxProgCto - 1) == 63 avoids the degenerate "1 PROGRAM of 1 byte + 1
 * PROGRAM_MAX of 63 bytes" split on a 64-byte chunk. 504 = 8 * 63 amortises
 * one SET_MTA over 8 PROGRAM_MAX frames (9 frames per 504 bytes ≈ 56 bytes/frame). */
enum { ERASE_CHUNK = 32768 };
enum { XCP_WRITE_CHUNK = 504 };

static void usage(const char *argv0) {
  fprintf(stderr,
          "Usage: %s [options] <firmware.s19|srec>\n"
          "  CAN (default):  -d <device>   Linux: can0  Windows: peak_pcanusb or mjbots_fdcanusb:COM7\n"
          "                  CAN is fixed 1 Mbit/s nominal, 5 Mbit/s BRS, 64-byte XCP payload.\n"
          "                  -tid HEX     default 667 (host->target)\n"
          "                  -rid HEX     default 7E1 (target->host)\n",
          argv0);
}

static uint32_t parse_hex_u32(const char *s) {
  return (uint32_t)strtoul(s, NULL, 16);
}

int main(int argc, char **argv) {
  (void)setvbuf(stdout, NULL, _IONBF, 0);
  (void)setvbuf(stderr, NULL, _IONBF, 0);

#if defined(_WIN32)
  /* Raise Windows multimedia timer resolution to 1 ms so Sleep/WaitForSingleObject
   * on our per-write OVERLAPPED events don't round up to 15.625 ms ticks. This
   * kills the occasional ~16 ms outlier WriteFile latencies we otherwise see. */
  (void)timeBeginPeriod(1);
#endif

  const char *can_device = NULL;
  uint32_t tid = 0x667u;
  uint32_t rid = 0x7E1u;
  const char *firmware = NULL;

  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-d") && i + 1 < argc) {
      can_device = argv[++i];
    } else if (!strcmp(argv[i], "-tid") && i + 1 < argc) {
      tid = parse_hex_u32(argv[++i]);
    } else if (!strcmp(argv[i], "-rid") && i + 1 < argc) {
      rid = parse_hex_u32(argv[++i]);
    } else if (argv[i][0] == '-') {
      usage(argv[0]);
      return 1;
    } else {
      firmware = argv[i];
    }
  }

  if (!firmware) {
    usage(argv[0]);
    return 1;
  }
  if (!can_device) {
    fprintf(stderr, "CAN mode requires -d <device> (e.g. can0 or mjbots_fdcanusb:COM7@1000000)\n");
    return 1;
  }

  tBltSessionSettingsXcpV10 xcp = {0};
  xcp.timeoutT1 = 1000u;
  xcp.timeoutT3 = 2000u;
  xcp.timeoutT4 = 10000u;
  xcp.timeoutT5 = 1000u;
  xcp.timeoutT6 = 50u;
  xcp.timeoutT7 = 2000u;
  xcp.seedKeyFile = NULL;
  xcp.connectMode = 0u;

  tBltTransportSettingsXcpV10Can can = {0};
  can.deviceName = can_device;
  can.deviceChannel = 0u;
  can.baudrate = 1000000u;
  can.transmitId = tid;
  can.receiveId = rid;
  can.useExtended = 0u;
  can.brsBaudrate = 5000000u;

  printf("LibOpenBLT %s\n", BltVersionGetString());
  printf("Config: device=%s tid=0x%03lx rid=0x%03lx nominal=%lu brs=%lu chunk=%u\n",
         can.deviceName,
         (unsigned long)can.transmitId,
         (unsigned long)can.receiveId,
         (unsigned long)can.baudrate,
         (unsigned long)can.brsBaudrate,
         (unsigned)XCP_WRITE_CHUNK);
  printf("Loading %s\n", firmware);

  BltFirmwareInit(BLT_FIRMWARE_PARSER_SRECORD);
  if (BltFirmwareLoadFromFile(firmware, 0u) != BLT_RESULT_OK) {
    fprintf(stderr, "Firmware load failed\n");
    BltFirmwareTerminate();
    return 2;
  }
  if (BltFirmwareGetSegmentCount() == 0u) {
    fprintf(stderr, "No firmware segments\n");
    BltFirmwareTerminate();
    return 2;
  }

  printf("Connecting...\n");
  BltSessionInit(BLT_SESSION_XCP_V10, &xcp, BLT_TRANSPORT_XCP_V10_CAN, &can);
  if (BltSessionStart() != BLT_RESULT_OK) {
    unsigned long wait_ms = 120000UL;
    const char *wait_env = getenv("OPENBLT_CONNECT_WAIT_MS");
    if (wait_env && wait_env[0]) {
      unsigned long v = strtoul(wait_env, NULL, 10);
      if (v > 0UL && v <= 3600000UL) {
        wait_ms = v;
      }
    }
    fprintf(stderr,
            "Waiting for target (reset into bootloader if needed), timeout %lu ms "
            "(set OPENBLT_CONNECT_WAIT_MS to change)...\n",
            wait_ms);
    uint32_t elapsed = 0u;
    uint32_t next_report = 5000u;
    while (BltSessionStart() != BLT_RESULT_OK) {
      if (elapsed >= wait_ms) {
        fprintf(stderr,
                "Timeout: bootloader did not respond (wrong tid/rid, not in bootloader, "
                "CAN wiring, or fdcanusb COM port already open in another app).\n");
        BltSessionStop();
        BltSessionTerminate();
        BltFirmwareTerminate();
        return 6;
      }
      BltUtilTimeDelayMs(20);
      elapsed += 20u;
      if (elapsed >= next_report) {
        fprintf(stderr, "... still waiting (%u ms)\n", (unsigned)elapsed);
        next_report += 5000u;
      }
    }
  }

  uint32_t r = BltSessionCheckInfoTable();
  if (r == BLT_RESULT_ERROR_SESSION_INFO_TABLE) {
    fprintf(stderr, "Info table check: update rejected by target\n");
    BltSessionTerminate();
    BltFirmwareTerminate();
    return 3;
  }

  uint32_t const t_erase_start = BltUtilTimeGetSystemTime();
  for (uint32_t seg = 0; seg < BltFirmwareGetSegmentCount(); seg++) {
    uint32_t base = 0, len = 0;
    uint8_t const *data = BltFirmwareGetSegment(seg, &base, &len);
    if (!data || len == 0u) {
      continue;
    }
    printf("Erase 0x%08lx +%lu\n", (unsigned long)base, (unsigned long)len);
    uint32_t left = len;
    uint32_t addr = base;
    while (left > 0u) {
      uint32_t n = left > ERASE_CHUNK ? ERASE_CHUNK : left;
      if (BltSessionClearMemory(addr, n) != BLT_RESULT_OK) {
        fprintf(stderr, "Erase failed at 0x%08lx\n", (unsigned long)addr);
        BltSessionStop();
        BltSessionTerminate();
        BltFirmwareTerminate();
        return 4;
      }
      addr += n;
      left -= n;
    }
  }
  uint32_t const t_erase_ms = BltUtilTimeGetSystemTime() - t_erase_start;

  uint32_t total_bytes = 0u;
  uint32_t const t_prog_start = BltUtilTimeGetSystemTime();
  for (uint32_t seg = 0; seg < BltFirmwareGetSegmentCount(); seg++) {
    uint32_t base = 0, len = 0;
    uint8_t const *data = BltFirmwareGetSegment(seg, &base, &len);
    if (!data || len == 0u) {
      continue;
    }
    printf("Program 0x%08lx +%lu\n", (unsigned long)base, (unsigned long)len);
    uint32_t left = len;
    uint32_t addr = base;
    uint8_t const *p = data;
    uint32_t sent = 0u;
    uint32_t next_report = 4096u;
    uint32_t t0 = BltUtilTimeGetSystemTime();
    while (left > 0u) {
      uint32_t n = left > XCP_WRITE_CHUNK ? XCP_WRITE_CHUNK : left;
      if (BltSessionWriteData(addr, n, p) != BLT_RESULT_OK) {
        fprintf(stderr, "Write failed at 0x%08lx\n", (unsigned long)addr);
        BltSessionStop();
        BltSessionTerminate();
        BltFirmwareTerminate();
        return 5;
      }
      addr += n;
      p += n;
      left -= n;
      sent += n;
      if (sent >= next_report || left == 0u) {
        uint32_t elapsed_ms = BltUtilTimeGetSystemTime() - t0;
        if (elapsed_ms == 0u) {
          elapsed_ms = 1u;
        }
        double elapsed_s = (double)elapsed_ms / 1000.0;
        double kbps = (double)sent / 1024.0 / elapsed_s;
        uint32_t pct = (len == 0u) ? 100u : (uint32_t)(((uint64_t)sent * 100u) / (uint64_t)len);
        printf("  ... %lu/%lu bytes (%lu%%, %.1f KiB/s)\n",
               (unsigned long)sent,
               (unsigned long)len,
               (unsigned long)pct,
               kbps);
        next_report += 4096u;
      }
    }
    total_bytes += sent;
  }
  uint32_t const t_prog_ms = BltUtilTimeGetSystemTime() - t_prog_start;

  BltSessionStop();
  BltSessionTerminate();
  BltFirmwareTerminate();

  double const prog_s = (t_prog_ms > 0u) ? ((double)t_prog_ms / 1000.0) : 0.001;
  double const prog_kbps = (double)total_bytes / 1024.0 / prog_s;
  printf("\n");
  printf("Programmed %lu bytes in %.2f s  =>  %.1f KiB/s "
         "(erase %.2f s, program %.2f s)\n",
         (unsigned long)total_bytes, prog_s, prog_kbps,
         (double)t_erase_ms / 1000.0, prog_s);

#if defined(_WIN32)
  (void)timeEndPeriod(1);
#endif
  return 0;
}
