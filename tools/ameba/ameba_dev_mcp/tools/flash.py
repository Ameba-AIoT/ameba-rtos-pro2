"""
Alias-driven firmware flashing tool for AmebaPro2 SoC.

The single MCP entry point is `flash_firmware_tool(alias)`. Everything
else (port, baudrate, memory_type, images) is read from
board_info.json5 + project_info.json5.

Pro2 differences from ameba-rtos:
  - Flash tool:   uartfwburn (not AmebaFlash.py)
  - Single image: flash_ntz.bin  (no multi-image partition table)
  - No .rdev profile needed
  - Download mode is MANUAL — no auto-download circuit
      AMB82 (standard): Set J27 jumper → press RESET, then remove jumper + RESET to boot
      AMB82-MINI:       Press UART_DOWNLOAD + RESET simultaneously → after flash, just press RESET to boot
  - Baudrate:     3000000 (flash baudrate; monitor baudrate is separate)
  - NOR flag:     -U
  - NAND flag:    -n pro2
  - Success markers: "nor download success" / "nand download success"
"""

from __future__ import annotations

import json
import logging
import os
import subprocess
import sys
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

from mcp.server.fastmcp import FastMCP

from ameba_dev_mcp._paths import PROJECT_ROOT, SDK_ROOT, TOOLS_ROOT
from ameba_dev_mcp.tools.project import _FLASH_NTZ_NN_BIN
from ameba_dev_mcp.config.board_session import manager as session_manager
from ameba_dev_mcp.config.loader import (
    ConfigLoadError,
    attach_selected_via_default,
    ensure_board_info_template,
    load_board_info,
    load_project_info,
    resolve_alias_or_error,
    resolve_board,
    validate_board_config,
    validate_project_config,
)
from ameba_dev_mcp.models.schemas import ConfigError

logger = logging.getLogger(__name__)

# Path to Pro2_PG_tool directory (inside the Pro2 SDK)
PG_TOOL_DIR = os.path.join(TOOLS_ROOT, "Pro2_PG_tool_v1.4.3")

DEBUG_LOG_DIR = os.path.join(PROJECT_ROOT, "mcp_flash_log")
DEBUG_LOG_FILE = os.path.join(DEBUG_LOG_DIR, "flash.log")
_log_initialized = False


def _init_log_file():
    global _log_initialized
    if not _log_initialized:
        try:
            os.makedirs(DEBUG_LOG_DIR, exist_ok=True)
            with open(DEBUG_LOG_FILE, "w") as f:
                f.write(f"=== Log cleared at {datetime.now().isoformat()} ===\n")
            _log_initialized = True
        except Exception:
            pass


def _log_to_file(msg: str) -> None:
    _init_log_file()
    try:
        with open(DEBUG_LOG_FILE, "a") as f:
            f.write(f"{datetime.now().isoformat()} - {msg}\n")
    except Exception:
        pass


def _err_envelope(errors: List[ConfigError], *, alias: str) -> Dict[str, Any]:
    return {
        "success": False,
        "alias": alias,
        "errors": [e.model_dump() for e in errors],
    }


# ---------------------------------------------------------------------------
# uartfwburn lookup
# ---------------------------------------------------------------------------

def _find_uartfwburn(pg_tool_dir: str) -> Optional[str]:
    """Look for uartfwburn under various OS names in the PG tool directory.

    Candidates are ordered by platform so the native binary is always
    preferred over cross-platform ones (e.g. .exe is never chosen on Linux).
    """
    import platform as _platform
    system = _platform.system().lower()  # 'linux', 'darwin', 'windows'
    machine = _platform.machine().lower()  # 'x86_64', 'arm64', ...

    if system == "linux":
        candidates = [
            "uartfwburn.linux",
            "uartfwburn",
        ]
    elif system == "darwin":
        if "arm" in machine:
            candidates = [
                "uartfwburn.arm.darwin",
                "uartfwburn.darwin",
                "uartfwburn",
            ]
        else:
            candidates = [
                "uartfwburn.darwin",
                "uartfwburn.arm.darwin",
                "uartfwburn",
            ]
    else:  # Windows or WSL running Windows tools
        candidates = [
            "uartfwburn.exe",
            "uartfwburn",
        ]

    for name in candidates:
        path = os.path.join(pg_tool_dir, name)
        if os.path.isfile(path):
            return path
    return None


# ---------------------------------------------------------------------------
# Subprocess driver
# ---------------------------------------------------------------------------

def _run_uartfwburn(
    args: List[str],
    timeout: int = 120,
    memory_type: str = "nor",
) -> Dict[str, Any]:
    """Execute uartfwburn with the given args and interpret the result."""
    _log_to_file(f"CMD: {' '.join(args)}")
    success_marker = "download success"
    try:
        result = subprocess.run(
            args,
            capture_output=True,
            text=True,
            stdin=subprocess.DEVNULL,
            timeout=timeout,
        )
        stdout = result.stdout or ""
        stderr = result.stderr or ""
        _log_to_file(f"Return code: {result.returncode}")
        _log_to_file("STDOUT:\n" + stdout)
        if stderr:
            _log_to_file("STDERR:\n" + stderr)

        found_success = success_marker in stdout
        if found_success and result.returncode == 0:
            return {
                "success": True,
                "message": "Firmware flashed successfully",
                "stdout": stdout,
                "stderr": stderr,
                "returncode": 0,
            }
        if result.returncode != 0 or not found_success:
            # Try to surface the most useful line from stdout/stderr
            lines = (stdout + "\n" + stderr).splitlines()
            # Look for an error line first
            err_line = ""
            for line in reversed(lines):
                stripped = line.strip()
                if stripped and "error" in stripped.lower():
                    err_line = stripped
                    break
            if not err_line:
                # Fall back to the last non-empty line
                for line in reversed(lines):
                    if line.strip():
                        err_line = line.strip()
                        break
            msg = err_line or f"uartfwburn exited with code {result.returncode}"
            return {
                "success": False,
                "message": msg,
                "stdout": stdout,
                "stderr": stderr,
                "returncode": result.returncode,
            }
        # returncode == 0 but no success marker
        return {
            "success": False,
            "message": (
                f"uartfwburn finished (rc=0) but '{success_marker}' "
                "not found in output; verify manually."
            ),
            "stdout": stdout,
            "stderr": stderr,
            "returncode": result.returncode,
        }
    except subprocess.TimeoutExpired:
        _log_to_file(f"Subprocess timed out after {timeout}s")
        return {
            "success": False,
            "message": (
                f"Flash timed out after {timeout}s. "
                "Is the board in download mode? (J27 jumper + reset)"
            ),
            "stdout": "",
            "stderr": "",
            "returncode": -1,
        }
    except Exception as ex:
        _log_to_file(f"Subprocess exception: {ex}")
        return {
            "success": False,
            "message": f"Failed to launch uartfwburn: {ex}",
            "stdout": "",
            "stderr": str(ex),
            "returncode": -1,
        }


# ---------------------------------------------------------------------------
# Main flash entry
# ---------------------------------------------------------------------------

def flash_firmware(alias: Optional[str] = None) -> Dict[str, Any]:
    """Flash firmware. `alias=None` is accepted only when board_info has
    exactly one board configured; otherwise ALIAS_REQUIRED is returned.
    """
    # Step 1: load + resolve board
    try:
        binfo = load_board_info(PROJECT_ROOT)
    except ConfigLoadError as ex:
        missing = any(e.code == "BOARD_CONFIG_MISSING" for e in ex.errors)
        env = _err_envelope(ex.errors, alias=alias)
        if missing:
            try:
                env["template_path"] = ensure_board_info_template(PROJECT_ROOT)
                env["docs_url"] = "docs/board_info.md"
                env["resource_url"] = "debug://hardware"
            except Exception:
                pass
        return env

    resolved_alias, alias_errors = resolve_alias_or_error(binfo, alias)
    if alias_errors is not None:
        env = _err_envelope(alias_errors, alias=alias)
        env["configured_aliases"] = _alias_summary(binfo)
        env["available_local_ports"] = available_local_port_names()
        return env
    alias = resolved_alias

    board_errs = validate_board_config(PROJECT_ROOT, alias=alias)
    if board_errs:
        return _err_envelope(board_errs, alias=alias)
    board = resolve_board(binfo, alias)

    # Step 2: load project entry for this SoC
    try:
        pinfo = load_project_info(PROJECT_ROOT)
    except ConfigLoadError as ex:
        return _err_envelope(ex.errors, alias=alias)

    entry = pinfo.projects.get(board.soc)
    if entry is None:
        return _err_envelope(
            [
                ConfigError(
                    code="PROJECT_NOT_CONFIGURED",
                    field_path=f"projects.{board.soc}",
                    message=f"SoC '{board.soc}' has no entry in project_info.json5",
                    hint="Run build_firmware to auto-create an entry, or add manually.",
                )
            ],
            alias=alias,
        )

    proj_errs = validate_project_config(PROJECT_ROOT, soc=board.soc)
    fatal = [e for e in proj_errs if e.code != "AUTO_IMAGES_MISSING"]
    if fatal:
        return _err_envelope(fatal, alias=alias)

    # Step 3: resolve flash_ntz.bin path
    if not entry.images:
        return _err_envelope(
            [
                ConfigError(
                    code="IMAGE_FILE_NOT_FOUND",
                    field_path=f"projects.{board.soc}.images",
                    message=f"No images configured for SoC '{board.soc}' in project_info.json5",
                    hint="Run build_firmware to populate the images entry.",
                )
            ],
            alias=alias,
        )

    image_path = entry.images[0].path
    if not os.path.isfile(image_path):
        return _err_envelope(
            [
                ConfigError(
                    code="IMAGE_FILE_NOT_FOUND",
                    field_path=f"projects.{board.soc}.images[0].path",
                    message=f"flash_ntz.bin not found: {image_path}",
                    hint="Run build_firmware to (re)generate flash_ntz.bin.",
                )
            ],
            alias=alias,
        )

    # Step 4: locate uartfwburn
    uartfwburn = _find_uartfwburn(PG_TOOL_DIR)
    if uartfwburn is None:
        return _err_envelope(
            [
                ConfigError(
                    code="UARTFWBURN_NOT_FOUND",
                    field_path="tools/Pro2_PG_tool_v1.4.3",
                    message=f"uartfwburn not found under {PG_TOOL_DIR}",
                    hint=(
                        "Ensure the Pro2_PG_tool directory exists in the SDK at "
                        "tools/Pro2_PG_tool_v1.4.3/ with uartfwburn.exe (Windows) "
                        "or uartfwburn.linux (Linux)."
                    ),
                )
            ],
            alias=alias,
        )

    # Step 5: release any active monitor session (subprocess will reopen port)
    if session_manager.get(alias) is not None:
        logger.info("Releasing monitor session on '%s' before flashing", alias)
        session_manager.disconnect(alias)
    try:
        from ameba_dev_mcp.tools.serial import _drop_aag_parser
        _drop_aag_parser(alias)
    except Exception:
        pass

    # Step 6: build uartfwburn command
    # Flash baudrate is always 3000000 (independent of monitor baudrate in board_info).
    # memory_type from board_info.json5 determines the flash flag:
    #   NOR  → -U
    #   NAND → -n pro2
    # NN firmware (flash_ntz.nn.bin) requires the extra -t 0x81cf flag.
    memory_type = board.memory_type or "nor"
    flash_baudrate = 3000000
    port = board.port
    is_nn = os.path.basename(image_path) == _FLASH_NTZ_NN_BIN

    cmd = [uartfwburn, "-p", port, "-f", image_path, "-b", str(flash_baudrate)]
    if memory_type == "nand":
        cmd += ["-n", "pro2"]
    else:
        cmd += ["-U"]  # NOR flag
    if is_nn:
        cmd += ["-t", "0x81cf"]

    _log_to_file(f"Flashing {board.soc} on {port} ({memory_type}, {flash_baudrate}bps): {image_path}")

    # Step 7: run with a reminder about manual download mode
    download_mode_hint = (
        "IMPORTANT: Pro2 requires MANUAL download mode. "
        "Board must already be in download mode before calling this tool. "
        "AMB82 (standard): set J27 jumper then press RESET. "
        "AMB82-MINI: press UART_DOWNLOAD + RESET simultaneously, release RESET first. "
        "After flashing: AMB82 remove J27 + press RESET; AMB82-MINI just press RESET."
    )

    flash_timeout = int(os.environ.get("AMEBA_FLASH_TIMEOUT", "180"))
    res = _run_uartfwburn(cmd, timeout=flash_timeout, memory_type=memory_type)

    if not res["success"]:
        return {
            "success": False,
            "alias": alias,
            "soc": board.soc,
            "transport": board.transport,
            "errors": [
                ConfigError(
                    code="FLASH_HW_ERROR",
                    field_path=f"boards.{alias}",
                    message=res["message"],
                    hint=(
                        f"Pro2 needs J27 jumper + RESET (or UART_DOWNLOAD + RESET for Mini) "
                        f"for download mode. "
                        f"Check port '{port}', baudrate {flash_baudrate}, and that "
                        f"the firmware image is valid. Full log at {DEBUG_LOG_FILE}."
                    ),
                ).model_dump()
            ],
            "log_path": DEBUG_LOG_FILE,
            "download_mode_hint": download_mode_hint,
        }

    return {
        "success": True,
        "alias": alias,
        "soc": board.soc,
        "transport": board.transport,
        "memory_type": memory_type,
        "port": port,
        "baudrate": flash_baudrate,
        "image_flashed": os.path.basename(image_path),
        "image_path": image_path,
        "log_path": DEBUG_LOG_FILE,
        "message": res["message"],
        "download_mode_hint": download_mode_hint,
    }


# ---------------------------------------------------------------------------
# list_serial_ports helpers  (same logic as ameba-rtos, local-only for Pro2)
# ---------------------------------------------------------------------------

def _lsof_holder(device: str) -> Tuple[Optional[bool], Optional[str]]:
    """Best-effort: return (busy, holder) for a local serial device using lsof."""
    import shutil

    if shutil.which("lsof") is None:
        return None, None
    try:
        proc = subprocess.run(
            ["lsof", "-t", device],
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except Exception:
        return None, None
    pids = [p.strip() for p in (proc.stdout or "").splitlines() if p.strip()]
    if not pids:
        return False, None

    parts: List[str] = []
    for pid in pids:
        cmd = ""
        try:
            with open(f"/proc/{pid}/comm") as f:
                cmd = f.read().strip()
        except Exception:
            pass
        parts.append(f"pid={pid} cmd={cmd}" if cmd else f"pid={pid}")
    return True, ", ".join(parts)


def _list_local_ports() -> List[Dict[str, Any]]:
    try:
        from serial.tools import list_ports
    except Exception as ex:
        return [{"error": f"pyserial list_ports unavailable: {ex}"}]

    out = []
    for p in list_ports.comports():
        busy, holder = _lsof_holder(p.device)
        held_by_self_alias = session_manager.find_local_holder(p.device)
        out.append(
            {
                "device": p.device,
                "name": p.name,
                "description": p.description,
                "hwid": p.hwid,
                "busy": busy,
                "held_by_self": held_by_self_alias is not None,
                "held_by_self_alias": held_by_self_alias,
                "holder": holder,
            }
        )
    return out


def available_local_port_names() -> List[str]:
    """Best-effort local port device names (used to enrich error hints)."""
    try:
        from serial.tools import list_ports

        return [p.device for p in list_ports.comports()]
    except Exception:
        return []


def _alias_summary(binfo) -> Dict[str, Dict[str, Any]]:
    """Compact alias → {soc, transport, port} map for error hints."""
    out: Dict[str, Dict[str, Any]] = {}
    for name in sorted(binfo.boards.keys()):
        b = binfo.boards[name]
        out[name] = {
            "soc": b.soc,
            "transport": b.transport,
            "port": b.port,
        }
    return out


def list_serial_ports(alias: Optional[str] = None) -> Dict[str, Any]:
    """Enumerate serial ports (local only for Pro2 — no RemoteService)."""
    if alias is None:
        return {"success": True, "scope": "local", "ports": _list_local_ports()}

    try:
        binfo = load_board_info(PROJECT_ROOT)
    except ConfigLoadError as ex:
        env = _err_envelope(ex.errors, alias=alias)
        if any(e.code == "BOARD_CONFIG_MISSING" for e in ex.errors):
            try:
                env["template_path"] = ensure_board_info_template(PROJECT_ROOT)
                env["docs_url"] = "docs/board_info.md"
                env["resource_url"] = "debug://hardware"
            except Exception:
                pass
        return env

    resolved_alias, alias_errors = resolve_alias_or_error(binfo, alias)
    if alias_errors is not None:
        env = _err_envelope(alias_errors, alias=alias)
        env["configured_aliases"] = _alias_summary(binfo)
        env["available_local_ports"] = available_local_port_names()
        return env
    alias = resolved_alias
    return {"success": True, "scope": "local", "alias": alias, "ports": _list_local_ports()}


# ---------------------------------------------------------------------------
# MCP registration
# ---------------------------------------------------------------------------

def register_flash_tools(mcp: FastMCP) -> None:
    @mcp.tool()
    async def flash_firmware_tool(alias: Optional[str] = None) -> Dict[str, Any]:
        """
        Flash firmware to the AmebaPro2 board identified by `alias`.

        All connection / image parameters come from board_info.json5 +
        project_info.json5. The firmware image (flash_ntz.bin or
        flash_ntz.nn.bin) is produced by build_firmware.

        *** IMPORTANT — MANUAL DOWNLOAD MODE REQUIRED ***
        AmebaPro2 has NO auto-download circuit. The user MUST physically
        prepare the board BEFORE this tool is called.

        There are two Pro2 board variants — ask the user which one they have
        if unknown, then instruct accordingly:

          AMB82 (standard board):
            1. Set the J27 jumper on the board
            2. Press the RESET button
            3. THEN call this tool immediately

          AMB82-MINI:
            1. Press and hold UART_DOWNLOAD button
            2. While holding, press RESET button simultaneously
            3. Release both buttons
            4. THEN call this tool immediately

        ALWAYS remind the user to do these steps before calling this tool.
        Do NOT call this tool and then ask — ask FIRST, wait for the user to
        confirm the board is in download mode, then call.

        Args:
            alias: Board alias from board_info.json5 (e.g. "RTL8735B_COM5").
                   May be omitted ONLY when board_info.json5 has exactly
                   one board configured.

        Returns:
            On success: {success: true, alias, soc, image_flashed, ...}
            On failure: {success: false, alias, errors: [{code, message, hint}, ...]}

        Error codes specific to Pro2:
          UARTFWBURN_NOT_FOUND   — check tools/Pro2_PG_tool_v1.4.3/ in SDK
          FLASH_HW_ERROR         — uartfwburn failed; check J27 jumper + RESET
          IMAGE_FILE_NOT_FOUND   — run build_firmware first
          PROJECT_NOT_CONFIGURED — run build_firmware first
        """
        result = flash_firmware(alias)
        attach_selected_via_default(result, alias, PROJECT_ROOT)
        return result

    @mcp.tool()
    async def list_serial_ports_tool(alias: Optional[str] = None) -> Dict[str, Any]:
        """
        List available serial ports on the local machine (Pro2 is local-only).

        Args:
            alias: Optional board alias. When given, confirms the board's
                   configured port is visible. When omitted, lists all
                   local ports regardless of board_info.json5.
        """
        return list_serial_ports(alias)

