"""
Stub flashcfg_parser for AmebaPro2.

AmebaPro2 uses a single flat flash_ntz.bin; there is no multi-image
flash layout header to parse. This stub exists so that the shared
loader.py can import ParsedProject without error. Pro2 uses the
_sync_project_info_fallback path in project.py instead.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional


class FlashCfgParseError(Exception):
    """Raised when flash config parsing fails."""


@dataclass
class ParsedImage:
    type: str = ""
    path: str = ""
    start_addr: str = "0x0"
    end_addr: str = ""
    optional: bool = False


@dataclass
class ParsedProject:
    soc: str = ""
    build_dir: str = ""
    images: List[ParsedImage] = field(default_factory=list)


def parse_project(
    sdk_root: str,
    soc: str,
    build_base: Optional[str] = None,
) -> ParsedProject:
    """Pro2 stub — always raises FlashCfgParseError.

    AmebaPro2 does not use multi-image flash layout headers. Flash image
    layout is managed directly in project_info.json5 by the CMake build
    via _sync_project_info in project.py.
    """
    raise FlashCfgParseError(
        f"parse_project is not supported for AmebaPro2 (soc={soc!r}). "
        "Use build_firmware to populate project_info.json5 with flash_ntz.bin."
    )
