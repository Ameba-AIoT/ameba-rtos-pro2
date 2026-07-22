"""
Device info resource for AmebaPro2.

Pro2 does not use .rdev device profiles — it uses a single flash_ntz.bin
flashed via uartfwburn. This resource exposes basic Pro2 hardware info.
"""

import json
import os
from typing import Any, Dict, List

from mcp.server.fastmcp import FastMCP

from ameba_dev_mcp._paths import TOOLS_ROOT

PG_TOOL_DIR = os.path.join(TOOLS_ROOT, "Pro2_PG_tool _v1.4.3")

_PRO2_DEVICE_INFO = {
    "device_name": "RTL8735B",
    "description": "AmebaPro2 SoC",
    "flash_tool": "uartfwburn",
    "flash_image": "flash_ntz.bin",
    "monitor_baudrate": 115200,    # serial monitor (UART log) baudrate
    "flash_baudrate": 3000000,     # uartfwburn flash baudrate (hardcoded, not from board_info)
    "download_mode": "manual (J27 jumper + RESET)",
    "memory_types": ["nor", "nand"],
    "nor_flags": ["-U"],
    "nand_flags": ["-n", "pro2"],
    "partial_flash": {
        "nor": "-s <hex_offset>  (64K aligned)",
        "nand": "-t <type_id>",
    },
}


def get_pro2_device_info() -> Dict[str, Any]:
    """Return hardware info for the AmebaPro2 RTL8735B SoC."""
    pg_tool_present = os.path.isdir(PG_TOOL_DIR)
    uartfwburn = None
    for name in ["uartfwburn.exe", "uartfwburn.linux", "uartfwburn.darwin"]:
        path = os.path.join(PG_TOOL_DIR, name)
        if os.path.isfile(path):
            uartfwburn = path
            break
    return {
        **_PRO2_DEVICE_INFO,
        "pg_tool_dir": PG_TOOL_DIR,
        "pg_tool_present": pg_tool_present,
        "uartfwburn_path": uartfwburn,
    }


def register_device_resources(mcp: FastMCP) -> None:
    """Register AmebaPro2 device info resources."""

    @mcp.resource("device://profiles")
    def get_device_profiles() -> str:
        """
        List AmebaPro2 device hardware information.

        Returns JSON with RTL8735B flash tool details and PG tool availability.
        """
        info = get_pro2_device_info()
        return json.dumps(
            {"devices": [{"device_name": "RTL8735B", **info}], "total_count": 1},
            indent=2,
        )

    @mcp.resource("device://{device_name}/info")
    def get_device_profile(device_name: str) -> str:
        """
        Get hardware info for a Pro2 device (RTL8735B).

        Args:
            device_name: Device name (RTL8735B)
        """
        if device_name.upper() not in ("RTL8735B", "PRO2"):
            return json.dumps(
                {"error": f"Unknown device '{device_name}' — Pro2 only supports RTL8735B"}
            )
        return json.dumps(get_pro2_device_info(), indent=2)

    @mcp.resource("device://{device_name}/{memory_type}/info")
    def get_device_profile_by_memory(device_name: str, memory_type: str) -> str:
        """
        Get hardware info for a Pro2 device with specific memory type.

        Args:
            device_name:  Device name (RTL8735B)
            memory_type:  "nor" or "nand"
        """
        info = get_pro2_device_info()
        info["selected_memory_type"] = memory_type
        info["flash_flags"] = ["-n", "pro2"] if memory_type == "nand" else ["-U"]
        return json.dumps(info, indent=2)

