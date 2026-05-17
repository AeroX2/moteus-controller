"""Post: export custom_openblt_* from platformio.ini into os.environ for openblt_host_upload.py."""

Import("env")

if env.get("PIOENV") != "moteus_openblt_can":
    Return()

import os

# (environment variable, platformio.ini option name)
_EXPORT = (
    ("OPENBLT_CAN_DEVICE", "custom_openblt_can_device"),
    ("OPENBLT_CAN_TID", "custom_openblt_can_tid"),
    ("OPENBLT_CAN_RID", "custom_openblt_can_rid"),
    ("OPENBLT_LIBOPENBLT_FLASH", "custom_openblt_libopenblt_flash"),
    ("OPENBLT_LIBOPENBLT_TRANSPORT", "custom_openblt_libopenblt_transport"),
    ("OPENBLT_UART_PORT", "custom_openblt_uart_port"),
    ("OPENBLT_UART_BAUD", "custom_openblt_uart_baud"),
    ("OPENBLT_PRE_RESET_R", "custom_openblt_pre_reset_r"),
    ("OPENBLT_RESET_COM_PORT", "custom_openblt_reset_com_port"),
    ("OPENBLT_RESET_TARGET_ID", "custom_openblt_reset_target_id"),
    ("OPENBLT_RESET_DELAY", "custom_openblt_reset_delay"),
    ("OPENBLT_RESET_DISCOVER", "custom_openblt_reset_discover"),
    ("OPENBLT_RESET_DISCOVER_SECONDS", "custom_openblt_reset_discover_seconds"),
    ("OPENBLT_RESET_DISCOVER_EXCLUDE", "custom_openblt_reset_discover_exclude"),
    ("OPENBLT_CONNECT_WAIT_MS", "custom_openblt_connect_wait_ms"),
)

for env_key, opt in _EXPORT:
    val = env.GetProjectOption(opt, "").strip()
    if val:
        os.environ[env_key] = val
