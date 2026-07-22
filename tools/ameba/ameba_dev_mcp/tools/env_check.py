"""
Environment pre-check for AmebaPro2: a single read-only diagnostic that
surfaces every condition that could later cause a flash / serial tool to fail.

Returns a structured report covering:
  - JSON config files (board_info.json5 / project_info.json5) presence + validity
  - Per-alias port visibility (local lsof / pyserial enumeration)
  - uartfwburn tool presence check
  - Build toolchain check (platform-aware):
      Windows  → msys64 directory + cmake inside msys2
      Linux    → cmake on system PATH
      macOS    → cmake on system PATH

Pro2-specific differences from ameba-rtos env_check:
  - NO reset smoke test (Pro2 has no auto-download circuit; J27 + manual reset needed)
  - NO remote service reachability check (Pro2 uses local USB only)
"""

from __future__ import annotations

import logging
import os
import shutil
import subprocess
import sys
from typing import Any, Dict, List, Optional

_IS_WINDOWS = sys.platform == "win32"

from mcp.server.fastmcp import FastMCP

from ameba_dev_mcp._paths import PROJECT_ROOT, TOOLS_ROOT
from ameba_dev_mcp.config.board_session import manager as session_manager
from ameba_dev_mcp.config.loader import (
    ConfigLoadError,
    board_info_path,
    ensure_board_info_template,
    ensure_project_info_template,
    load_board_info,
    load_project_info,
    project_info_path,
    resolve_board,
    save_board_info,
)
from ameba_dev_mcp.models.schemas import (
    BoardEntry,
    BoardInfo,
    ConfigError,
)

logger = logging.getLogger(__name__)

# Path to Pro2 PG tool
PG_TOOL_DIR = os.path.join(TOOLS_ROOT, "Pro2_PG_tool _v1.4.3")

_UARTFWBURN_CANDIDATES = [
    "uartfwburn.exe",
    "uartfwburn.linux",
    "uartfwburn.darwin",
    "uartfwburn.arm.darwin",
    "uartfwburn",
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _find_uartfwburn() -> Optional[str]:
    """Return the path to uartfwburn if found, else None."""
    for name in _UARTFWBURN_CANDIDATES:
        path = os.path.join(PG_TOOL_DIR, name)
        if os.path.isfile(path):
            return path
    return None


def _config_section() -> tuple[Dict[str, Any], Any]:
    """Probe both JSON config files; auto-create empty templates on miss."""
    bpath = board_info_path(PROJECT_ROOT)
    ppath = project_info_path(PROJECT_ROOT)

    out: Dict[str, Any] = {
        "board_info": {
            "present": os.path.isfile(bpath),
            "path": bpath,
            "valid": False,
            "errors": [],
            "alias_count": 0,
        },
        "project_info": {
            "present": os.path.isfile(ppath),
            "path": ppath,
            "valid": False,
            "errors": [],
        },
        "docs_url": None,
        "resource_url": None,
    }

    # Auto-create templates so the first run leaves the user a starting point.
    if not out["board_info"]["present"]:
        try:
            ensure_board_info_template(PROJECT_ROOT)
            out["board_info"]["template_created"] = True
        except Exception as ex:
            out["board_info"]["errors"].append({
                "code": "TEMPLATE_CREATE_FAILED",
                "message": f"failed to create empty board_info template: {ex}",
            })
    if not out["project_info"]["present"]:
        try:
            ensure_project_info_template(PROJECT_ROOT)
            out["project_info"]["template_created"] = True
        except Exception as ex:
            out["project_info"]["errors"].append({
                "code": "TEMPLATE_CREATE_FAILED",
                "message": f"failed to create empty project_info template: {ex}",
            })

    # Load and validate.
    binfo = None
    try:
        binfo = load_board_info(PROJECT_ROOT)
        out["board_info"]["valid"] = True
        out["board_info"]["alias_count"] = len(binfo.boards)
    except ConfigLoadError as ex:
        out["board_info"]["errors"].extend(e.model_dump() for e in ex.errors)

    try:
        load_project_info(PROJECT_ROOT)
        out["project_info"]["valid"] = True
    except ConfigLoadError as ex:
        out["project_info"]["errors"].extend(e.model_dump() for e in ex.errors)

    # Surface docs when the user likely needs them.
    if (
        out["board_info"]["errors"]
        or out["project_info"]["errors"]
        or not out["board_info"]["valid"]
        or not out["project_info"]["valid"]
        or out["board_info"]["alias_count"] == 0
    ):
        out["docs_url"] = "docs/board_info.md"
        out["resource_url"] = "debug://hardware"

    return out, binfo


def _local_port_visibility(port: str) -> Dict[str, Any]:
    """Look up a local serial port in pyserial's enumeration."""
    from ameba_dev_mcp.tools.flash import _list_local_ports

    for entry in _list_local_ports():
        if entry.get("device") == port or entry.get("name") == port:
            return {
                "visible": True,
                "busy": entry.get("busy"),
                "holder": entry.get("holder"),
                "held_by_self": entry.get("held_by_self", False),
                "held_by_self_alias": entry.get("held_by_self_alias"),
            }
    return {
        "visible": False,
        "busy": None,
        "holder": None,
        "held_by_self": False,
        "held_by_self_alias": None,
    }


def _check_cmake_windows() -> Dict[str, Any]:
    """Windows-specific cmake check: look for msys64 and cmake inside it."""
    from ameba_dev_mcp.tools.project import _find_msys64

    msys64_dir = _find_msys64()
    if msys64_dir is None:
        return {
            "present": False,
            "method": "msys2",
            "msys64_dir": None,
            "error": (
                "msys64 not found. AmebaPro2 requires msys2/mingw to build on Windows.\n"
                "Download msys64_v10_3.7z from:\n"
                "  https://github.com/Ameba-AIoT/ameba-tool-rtos-pro2/releases/tag/msys64_v10_3\n"
                "Extract to C:\\msys64_v10_3\\msys64 (or set AMEBA_MSYS64_DIR env var)."
            ),
            "setup_url": "https://github.com/Ameba-AIoT/ameba-tool-rtos-pro2/releases/tag/msys64_v10_3",
        }

    # cmake detection strategy (matches the same logic used by build_firmware):
    #   1. Check well-known Windows cmake install locations via _find_win_cmake_dir().
    #      This is identical to what _msys2_extra_path_prefix() does when it injects
    #      PATH into the build bash command — so if build works, this check passes too.
    #   2. Fall back to running `cmake --version` inside msys2 bash (covers the case
    #      where cmake is installed inside msys2 itself via `pacman -S cmake`).
    bash_exe = os.path.join(msys64_dir, "usr", "bin", "bash.exe")
    from ameba_dev_mcp.tools.project import (
        _find_asdk_toolchain_dir,
        _find_win_cmake_dir,
        _msys2_extra_path_prefix,
    )

    cmake_ver = None
    cmake_ok = False
    cmake_path = None

    # Strategy 1: find cmake.exe in known Windows locations (same as build path)
    win_cmake_dir = _find_win_cmake_dir()
    if win_cmake_dir:
        cmake_exe = os.path.join(win_cmake_dir, "cmake.exe")
        cmake_ok = os.path.isfile(cmake_exe)
        if cmake_ok:
            cmake_path = cmake_exe
            try:
                proc = subprocess.run(
                    [cmake_exe, "--version"],
                    capture_output=True, timeout=10,
                )
                stdout = proc.stdout.decode("utf-8", errors="replace")
                cmake_ver = stdout.strip().splitlines()[0] if stdout.strip() else None
            except Exception:
                pass

    # Strategy 2: try msys2 bash with injected PATH (e.g. cmake via pacman)
    if not cmake_ok:
        path_prefix = _msys2_extra_path_prefix(msys64_dir)
        try:
            proc = subprocess.run(
                [bash_exe, "--login", "-c", f"{path_prefix}cmake --version"],
                capture_output=True, timeout=15,
            )
            stdout = proc.stdout.decode("utf-8", errors="replace")
            if proc.returncode == 0:
                cmake_ok = True
                cmake_ver = stdout.strip().splitlines()[0] if stdout.strip() else None
        except Exception:
            pass

    result: Dict[str, Any] = {
        "present": cmake_ok,
        "method": "msys2",
        "msys64_dir": msys64_dir,
        "cmake_version": cmake_ver,
    }
    if cmake_path:
        result["path"] = cmake_path
    if not cmake_ok:
        result["error"] = (
            f"cmake not found inside msys2 bash (msys64={msys64_dir}).\n"
            "Install cmake from https://cmake.org/download/ (Windows x86_64 .msi) "
            "to one of the standard locations (e.g. C:\\Program Files\\CMake\\bin)."
        )

    toolchain_dir = _find_asdk_toolchain_dir(msys64_dir)
    result["toolchain"] = {
        "present": toolchain_dir is not None,
        "path": toolchain_dir,
    }
    if toolchain_dir is None:
        result["toolchain"]["error"] = (
            f"ARM toolchain (asdk-10.3.0/8.4.0/8.3.0) not found under {msys64_dir}.\n"
            "Download from: "
            "https://github.com/Ameba-AIoT/ameba-toolchain/releases/tag/V10.3.0-amebe-rtos-pro2\n"
            f"and extract it directly under {msys64_dir} (e.g. {msys64_dir}\\asdk-10.3.0\\)."
        )
    return result


def _check_cmake_unix() -> Dict[str, Any]:
    """Linux/macOS cmake check: look on system PATH."""
    cmake_path = shutil.which("cmake")
    if cmake_path is None:
        platform = "macOS" if sys.platform == "darwin" else "Linux"
        install_hint = (
            "brew install cmake" if sys.platform == "darwin"
            else "sudo apt-get install cmake"
        )
        return {
            "present": False,
            "method": "system_path",
            "path": None,
            "error": (
                f"cmake not found on PATH ({platform}).\n"
                f"Install with: {install_hint}\n"
                "Also ensure the ASDK toolchain is on PATH — see:\n"
                "  https://github.com/Ameba-AIoT/ameba-toolchain/releases/tag/V10.3.0-amebe-rtos-pro2"
            ),
        }

    cmake_ver = None
    try:
        proc = subprocess.run(
            ["cmake", "--version"], capture_output=True, text=True, timeout=5
        )
        cmake_ver = proc.stdout.strip().splitlines()[0] if proc.stdout.strip() else None
    except Exception:
        pass

    return {
        "present": True,
        "method": "system_path",
        "path": cmake_path,
        "cmake_version": cmake_ver,
    }


def _tools_section() -> Dict[str, Any]:
    """Check that build and flash tools are accessible (platform-aware)."""
    uartfwburn_path = _find_uartfwburn()

    cmake_info = _check_cmake_windows() if _IS_WINDOWS else _check_cmake_unix()

    result: Dict[str, Any] = {
        "uartfwburn": {
            "present": uartfwburn_path is not None,
            "path": uartfwburn_path,
        },
        "cmake": cmake_info,
        "platform": (
            "windows" if _IS_WINDOWS
            else ("darwin" if sys.platform == "darwin" else "linux")
        ),
    }

    if uartfwburn_path is None:
        result["uartfwburn"]["error"] = (
            f"uartfwburn not found under {PG_TOOL_DIR}. "
            "Ensure the Pro2_PG_tool directory is present in the SDK."
        )

    return result


# ---------------------------------------------------------------------------
# Main entry
# ---------------------------------------------------------------------------

def env_pre_check(
    *,
    soc_filter: Optional[str] = None,
    terse: bool = False,
) -> Dict[str, Any]:
    """Build a complete environment readiness report for AmebaPro2.

    NOTE: No reset smoke test is performed because Pro2 has no auto-download
    circuit. Instead, the report checks uartfwburn and cmake availability.
    """
    config, binfo = _config_section()
    tools = _tools_section()

    aliases: List[Dict[str, Any]] = []
    blocking_codes: List[str] = []
    ok_count = 0

    if binfo is not None:
        for alias_name in sorted(binfo.boards.keys()):
            entry = binfo.boards[alias_name]
            if soc_filter and entry.soc != soc_filter:
                continue
            try:
                board = resolve_board(binfo, alias_name)
            except Exception as ex:
                aliases.append({
                    "alias": alias_name,
                    "soc": entry.soc,
                    "transport": entry.transport,
                    "port": entry.port,
                    "errors": [{"code": "BOARD_RESOLVE_FAILED", "message": str(ex)}],
                })
                blocking_codes.append("BOARD_RESOLVE_FAILED")
                continue

            row: Dict[str, Any] = {
                "alias": alias_name,
                "soc": board.soc,
                "transport": board.transport,
                "port": board.port,
                "errors": [],
                # Pro2 has no auto-download; note is informational only
                "reset_note": (
                    "Pro2 requires J27 jumper + RESET button for download mode "
                    "(manual — no auto-download circuit)"
                ),
            }

            # Local port visibility check
            vis = _local_port_visibility(board.port)
            row["port_visible"] = vis["visible"]
            row["port_busy"] = vis["busy"]
            row["port_holder"] = vis["holder"]
            row["held_by_self"] = vis["held_by_self"]
            row["held_by_self_alias"] = vis["held_by_self_alias"]
            if not vis["visible"]:
                row["errors"].append({
                    "code": "PORT_NOT_VISIBLE",
                    "message": (
                        f"port '{board.port}' not present in local pyserial list"
                    ),
                    "hint": (
                        "Re-plug the USB cable, install/reinstall the USB-serial "
                        "driver, or fix the `port` field for this alias."
                    ),
                })
                blocking_codes.append("PORT_NOT_VISIBLE")

            if not row["errors"]:
                ok_count += 1
            aliases.append(row)

    # Tool-level blocking codes
    if not tools["uartfwburn"]["present"]:
        blocking_codes.append("UARTFWBURN_NOT_FOUND")
    if not tools["cmake"]["present"]:
        blocking_codes.append("CMAKE_NOT_FOUND")
    toolchain_ok = tools["cmake"].get("toolchain", {}).get("present", True) if _IS_WINDOWS else True
    if _IS_WINDOWS and not toolchain_ok:
        blocking_codes.append("TOOLCHAIN_NOT_FOUND")

    summary = {
        "ok_aliases": ok_count,
        "blocked_aliases": len(aliases) - ok_count,
        "blocking_codes": sorted(set(blocking_codes)),
    }

    next_steps: List[str] = []
    if not config["board_info"]["valid"] or config["board_info"]["alias_count"] == 0:
        next_steps.append(
            f"Edit {config['board_info']['path']} per docs/board_info.md "
            "(see debug://hardware for board wiring)."
        )
    if not config["project_info"]["valid"]:
        next_steps.append(
            f"Edit {config['project_info']['path']} per docs/project_info.md, "
            "or run build_firmware to auto-populate it."
        )
    if "PORT_NOT_VISIBLE" in summary["blocking_codes"]:
        next_steps.append(
            "Some alias ports are not visible — re-plug the USB cable "
            "and/or check the USB-serial driver."
        )
    if "UARTFWBURN_NOT_FOUND" in summary["blocking_codes"]:
        next_steps.append(
            "uartfwburn not found — ensure tools/Pro2_PG_tool _v1.4.3/ "
            "is present in the SDK root."
        )
    if "CMAKE_NOT_FOUND" in summary["blocking_codes"]:
        cmake_err = tools.get("cmake", {}).get("error", "")
        if _IS_WINDOWS:
            next_steps.append(
                "cmake/msys2 not ready for Windows build — "
                "see tools.cmake.error for setup instructions. "
                "Download msys64_v10_3.7z from: "
                "https://github.com/Ameba-AIoT/ameba-tool-rtos-pro2/releases/tag/msys64_v10_3"
            )
        else:
            next_steps.append(
                "cmake not found — install cmake and the ASDK toolchain. "
                "Toolchain download: "
                "https://github.com/Ameba-AIoT/ameba-toolchain/releases/tag/V10.3.0-amebe-rtos-pro2"
            )
    if "TOOLCHAIN_NOT_FOUND" in summary["blocking_codes"]:
        next_steps.append(
            "ARM toolchain not found inside msys64 — "
            "see tools.cmake.toolchain.error for setup instructions."
        )

    success = (
        config["board_info"]["valid"]
        and config["project_info"]["valid"]
        and config["board_info"]["alias_count"] > 0
        and summary["blocked_aliases"] == 0
        and tools["uartfwburn"]["present"]
        and tools["cmake"]["present"]
        and toolchain_ok
    )

    if terse:
        result: Dict[str, Any] = {
            "success": success,
            "summary": summary,
            "next_steps": next_steps,
        }
    else:
        result = {
            "success": success,
            "config": config,
            "tools": tools,
            "aliases": aliases,
            "summary": summary,
            "next_steps": next_steps,
        }

    # When board_info is empty, provide setup guidance
    if config["board_info"]["valid"] and config["board_info"]["alias_count"] == 0:
        result["agent_should_ask"] = {
            "via": "AskUserQuestion calls to gather board configuration",
            "after": "call apply_board_config_tool with the answers, then re-run env_pre_check_tool",
            "step1_first_ask": {
                "via": "AskUserQuestion (every user)",
                "questions": [
                    {
                        "key": "board_count",
                        "prompt": "How many AmebaPro2 boards on the bench?",
                        "type": "int>=1",
                    },
                    {
                        "key": "transport",
                        "prompt": "Are the boards connected locally (USB)?",
                        "type": "enum",
                        "options": ["local"],
                        "note": "Pro2 uses local USB only.",
                    },
                ],
            },
            "step2_pair_soc_with_port": {
                "required_for_every_board": True,
                "note": (
                    "apply_board_config_tool needs (soc, port) for every entry. "
                    "Call list_serial_ports_tool(), show results, ask user to map "
                    "each port to a SoC (RTL8735B for Pro2)."
                ),
                "goal_shape": [{"soc": "RTL8735B", "port": "COM3"}],
            },
        }

    return result


# ---------------------------------------------------------------------------
# apply_board_config (same logic as ameba-rtos)
# ---------------------------------------------------------------------------

def _sanitize_alias_part(s: str) -> str:
    out = []
    for ch in s:
        out.append(ch if ch.isalnum() or ch in ("_", "-") else "_")
    return "".join(out).strip("_") or "x"


_UNCHANGED_DEFAULT_ALIAS = "<<unchanged>>"


def apply_board_config(
    boards: List[Dict[str, Any]],
    merge: bool = True,
    default_alias: Optional[str] = _UNCHANGED_DEFAULT_ALIAS,
) -> Dict[str, Any]:
    """Atomic write of board entries to board_info.json5."""
    ensure_board_info_template(PROJECT_ROOT)
    try:
        existing = load_board_info(PROJECT_ROOT)
    except ConfigLoadError as ex:
        return {"success": False, "errors": [e.model_dump() for e in ex.errors]}

    new_entries: Dict[str, BoardEntry] = {}
    errors: List[Dict[str, Any]] = []
    for i, raw in enumerate(boards):
        try:
            soc = raw["soc"]
            port = raw["port"]
        except KeyError as k:
            errors.append({"index": i, "error": f"missing required field: {k}"})
            continue
        alias = (
            raw.get("alias")
            or f"{_sanitize_alias_part(soc)}_{_sanitize_alias_part(port)}"
        )
        entry_kwargs: Dict[str, Any] = {
            "soc": soc,
            "transport": "local",
            "port": port,
        }
        for k in (
            "memory_type",
            "baudrate",
            "monitor_baudrate",
            "chip_erase",
            "add_crlf",
        ):
            if raw.get(k) is not None:
                entry_kwargs[k] = raw[k]
        try:
            new_entries[alias] = BoardEntry(**entry_kwargs)
        except Exception as ex:
            errors.append({"index": i, "alias": alias, "error": str(ex)})

    if errors:
        return {"success": False, "errors": errors, "applied": []}

    if merge:
        merged = dict(existing.boards)
        merged.update(new_entries)
    else:
        merged = new_entries

    if default_alias == _UNCHANGED_DEFAULT_ALIAS:
        final_default = (
            existing.default_alias
            if (existing.default_alias is not None and existing.default_alias in merged)
            else None
        )
    elif default_alias is None or default_alias == "":
        final_default = None
    else:
        if default_alias not in merged:
            return {
                "success": False,
                "errors": [
                    {
                        "code": "DEFAULT_ALIAS_NOT_FOUND",
                        "field_path": "default_alias",
                        "message": (
                            f"default_alias '{default_alias}' is not among the "
                            f"merged aliases ({sorted(merged.keys())})."
                        ),
                        "hint": (
                            "Either pass an alias listed in `boards` or omit "
                            "`default_alias` to leave the field as is."
                        ),
                    }
                ],
                "applied": [],
            }
        final_default = default_alias

    final = BoardInfo(
        schema_version=existing.schema_version,
        defaults=existing.defaults,
        default_alias=final_default,
        boards=merged,
    )
    path = save_board_info(PROJECT_ROOT, final)
    return {
        "success": True,
        "path": path,
        "applied": sorted(new_entries.keys()),
        "total_aliases": len(merged),
        "default_alias": final_default,
        "config_paths": {
            "board_info": board_info_path(PROJECT_ROOT),
            "project_info": project_info_path(PROJECT_ROOT),
        },
        "remind_user": (
            "Bench config saved. Two files own this state:\n"
            f"  - {board_info_path(PROJECT_ROOT)}  (boards / ports)\n"
            f"  - {project_info_path(PROJECT_ROOT)}  (per-SoC flash images, auto-filled by build)\n"
            "When boards change: edit board_info.json5 directly or re-run the setup flow.\n"
            "project_info.json5 is regenerated by build_firmware on next build."
        ),
    }


# ---------------------------------------------------------------------------
# MCP registration
# ---------------------------------------------------------------------------

def register_env_check_tools(mcp: FastMCP) -> None:
    @mcp.tool()
    async def apply_board_config_tool(
        boards: List[Dict[str, Any]],
        merge: bool = True,
        default_alias: Optional[str] = None,
        set_default_alias: bool = False,
    ) -> Dict[str, Any]:
        """Atomically write board entries to board_info.json5 for AmebaPro2.

        Args:
            boards:            list of dicts. Required: soc, port. Optional:
                               alias (auto = "<SOC>_<PORT>"),
                               memory_type ("nor"/"nand"; default "nor"),
                               baudrate (default 115200 for Pro2 serial monitor),
                               monitor_baudrate.
            merge:             True (default) keeps existing aliases.
                               False replaces the entire boards list.
            default_alias:     Alias to mark as the bench's default.
                               Ignored unless set_default_alias=True.
            set_default_alias: Set True with default_alias to update the field.

        Returns: {success, path, applied: [alias...], total_aliases,
                  default_alias} or {success: false, errors: [...]}.
        """
        if set_default_alias:
            return apply_board_config(boards, merge=merge, default_alias=default_alias)
        return apply_board_config(boards, merge=merge)

    @mcp.tool()
    async def env_pre_check_tool(
        soc_filter: Optional[str] = None,
        terse: bool = False,
    ) -> Dict[str, Any]:
        """
        Read-only environment readiness check for AmebaPro2. Run this FIRST
        when the user reports "nothing works" or before starting a flash/test
        session.

        Pro2-specific notes:
          - NO reset smoke test (Pro2 has no auto-download circuit).
            J27 jumper + RESET is always manual.
          - Checks uartfwburn presence in tools/Pro2_PG_tool _v1.4.3/
          - Checks cmake presence on PATH (required for build_firmware)

        Args:
            soc_filter:  If set (e.g. "RTL8735B"), only aliases whose SoC
                         matches are included in the report.
            terse:       Default False. When True, drop per-alias `aliases`
                         array and full `config` block; keep only `success`,
                         `summary`, `next_steps`. Use for routine pre-flight.

        Returns:
          {
            "success": bool,
            "config": {
              "board_info":   {present, path, valid, errors, alias_count},
              "project_info": {present, path, valid, errors},
            },
            "tools": {
              "uartfwburn": {present, path, error?},
              "cmake":      {present, path, error?},
            },
            "aliases": [
              {
                "alias", "soc", "transport", "port",
                "port_visible", "port_busy", "port_holder",
                "held_by_self", "held_by_self_alias",
                "reset_note",   # informational: J27 + RESET required
                "errors": [{code, message, hint}, ...],
              }, ...
            ],
            "summary": {ok_aliases, blocked_aliases, blocking_codes: [...]},
            "next_steps": [str, ...],
            "agent_should_ask": {...}  # ONLY when alias_count==0
          }

        Common blocking codes:
          BOARD_CONFIG_MISSING / BOARD_CONFIG_INVALID — fix board_info.json5
          PROJECT_CONFIG_MISSING / PROJECT_CONFIG_INVALID — fix project_info.json5
          PORT_NOT_VISIBLE    — cable or driver issue
          UARTFWBURN_NOT_FOUND — Pro2_PG_tool directory missing from SDK
          CMAKE_NOT_FOUND     — cmake not on PATH (or not found inside msys2 on Windows)
          TOOLCHAIN_NOT_FOUND — ARM toolchain (asdk-10.3.0/...) not found inside msys64 (Windows only)
        """
        return env_pre_check(soc_filter=soc_filter, terse=terse)
