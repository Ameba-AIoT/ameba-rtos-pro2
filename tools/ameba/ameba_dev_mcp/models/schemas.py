"""
Pydantic models for MCP service validation.

These models define the API contracts for tools and resources.
"""

from typing import Optional, List, Dict, Literal
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator





# ===========================================================================
# project_info.json5 / board_info.json5 schema (Step 6.2)
# ===========================================================================

_HEX_ADDR_RE = r"^0x[0-9a-fA-F]{1,8}$"


class ProjectImageEntry(BaseModel):
    """One image inside `projects.<soc>.images[]`.

    Auto mode: type/path/start_addr/end_addr are all populated by build_firmware.
    Manual mode: only path + start_addr required; end_addr optional (used only
    for non-overlap checks); type optional.
    """
    model_config = ConfigDict(extra="forbid")

    type: Optional[str] = Field(default=None, description="Region type, e.g. IMG_BOOT")
    path: str = Field(..., description="Absolute path to the .bin file")
    start_addr: str = Field(..., description="Start flash address (hex)", pattern=_HEX_ADDR_RE)
    end_addr: Optional[str] = Field(default=None, description="End address inclusive (hex), optional in manual", pattern=_HEX_ADDR_RE)
    optional: bool = Field(default=False, description="If True, missing file is a warning, not an error (e.g. VFS)")


class ProjectEntry(BaseModel):
    model_config = ConfigDict(extra="forbid")

    flash_layout_setting_mode: Literal["auto", "manual"] = Field(...)
    build_dir: str = Field(..., description="Absolute path to build_<SOC>/")
    images: List[ProjectImageEntry] = Field(default_factory=list)


class ProjectInfo(BaseModel):
    model_config = ConfigDict(extra="forbid")

    # schema 1.1: memory_type / chip_erase moved to board_info.json5 (they
    # describe the physical flash on the board, not the project's images).
    schema_version: float = 1.1
    projects: Dict[str, ProjectEntry] = Field(default_factory=dict)


class SerialLogRecord(BaseModel):
    """Per-board serial-log capture config.

    When `enable` is true, opening the board's serial session via MCP
    starts a background reader that captures the FULL serial stream
    (timestamped, AAG-decoded) to a log file — independent of any
    `drain_first` the agent-facing read tools perform.

    `file_name` is auto-managed when it matches the generated pattern
    `<alias>_<YYYYMMDD>_<HHMMSS>.log`: a new file is created per day and
    the field is written back. A user-supplied custom name (not matching
    the pattern) is left untouched and never rotated.
    """
    model_config = ConfigDict(extra="forbid")

    enable: bool = Field(default=False, description="Capture serial output to a log file")
    log_dir: Optional[str] = Field(
        default=None,
        description="Log directory. Relative paths resolve under PROJECT_ROOT; "
                    "default is PROJECT_ROOT/mcp_serial_log.",
    )
    file_name: Optional[str] = Field(
        default=None,
        description="Log file name. Auto-generated/rotated when it matches "
                    "<alias>_<YYYYMMDD>_<HHMMSS>.log; a custom name is kept as-is.",
    )


class BoardEntry(BaseModel):
    model_config = ConfigDict(extra="forbid")

    soc: str = Field(..., description="SoC name, e.g. RTL8735B")
    transport: Literal["local"] = "local"
    port: str = Field(..., description="Serial port, e.g. /dev/ttyUSB0 or COM5")
    memory_type: Optional[Literal["nor", "nand", "ram"]] = Field(
        default=None,
        description="Flash type on this board; overrides defaults.memory_type.",
    )
    baudrate: Optional[int] = Field(default=None, description="Override default flash/connect baudrate")
    monitor_baudrate: Optional[int] = Field(default=None)
    chip_erase: Optional[bool] = Field(default=None)
    add_crlf: Optional[bool] = Field(default=None, description="Internal: serial_write trailing CRLF")
    serial_log_record: Optional[SerialLogRecord] = Field(default=None)


class BoardInfoDefaults(BaseModel):
    model_config = ConfigDict(extra="forbid")

    baudrate: int = 115200
    monitor_baudrate: int = 115200
    memory_type: Literal["nor", "nand", "ram"] = "nor"
    chip_erase: bool = False
    add_crlf: bool = True


class BoardInfo(BaseModel):
    model_config = ConfigDict(extra="forbid")

    # schema 1.1: gained defaults.memory_type + per-board memory_type
    # (moved here from project_info.json5).
    schema_version: float = 1.1
    defaults: BoardInfoDefaults = Field(default_factory=BoardInfoDefaults)
    default_alias: Optional[str] = Field(
        default=None,
        description=(
            "Optional fallback alias used when callers omit `alias` and "
            "boards has 2+ entries. When set, must be a key of `boards`. "
            "Tool envelopes report `selected_via=\"default\"` when this fires."
        ),
    )
    boards: Dict[str, BoardEntry] = Field(default_factory=dict)

    @model_validator(mode="after")
    def _check_default_alias(self):
        if self.default_alias is not None and self.default_alias not in self.boards:
            raise ValueError(
                f"default_alias '{self.default_alias}' is not in boards "
                f"({sorted(self.boards.keys())})"
            )
        return self


class ResolvedBoard(BaseModel):
    """A BoardEntry with all defaults filled in. Used by serial / flash tools."""
    model_config = ConfigDict(extra="forbid")

    alias: str
    soc: str
    transport: Literal["local"] = "local"
    port: str
    memory_type: Literal["nor", "nand", "ram"]
    baudrate: int
    monitor_baudrate: int
    chip_erase: bool
    add_crlf: bool
    serial_log_record: Optional[SerialLogRecord] = None


class ConfigError(BaseModel):
    """One validation error in a config file."""
    model_config = ConfigDict(extra="forbid")

    code: str
    field_path: str = ""
    message: str
    hint: Optional[str] = None

