#!/usr/bin/env python3
"""
FastMCP Server for AmebaPro2 SoC Development.

This is the main entry point for the MCP service that provides
AI agents with tools for AmebaPro2 firmware building, flashing
and serial communication.

Adapted from ameba-rtos's ameba_dev_mcp for AmebaPro2 SDK.
Key differences from ameba-rtos:
  - Build: CMake-based (not ameba.py)
  - Flash: uartfwburn (not AmebaFlash.py)
  - No Kconfig tools (Pro2 is CMake-based)
  - No flashcfg_parser (Pro2 uses single flash_ntz.bin)

Usage:
    # Run with stdio transport (default)
    python -m ameba_dev_mcp.server

    # Or using FastMCP CLI
    fastmcp dev ameba_dev_mcp.server:mcp
"""

import os
import sys
import logging
import argparse
from contextlib import asynccontextmanager
from typing import AsyncIterator

# Ensure parent directory is in path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from mcp.server.fastmcp import FastMCP

# Import tools and resources
from ameba_dev_mcp.tools.flash import register_flash_tools
from ameba_dev_mcp.tools.serial import register_serial_tools, cleanup_all_connections
from ameba_dev_mcp.tools.project import register_project_tools
from ameba_dev_mcp.tools.env_check import register_env_check_tools
from ameba_dev_mcp.tools.video_example import register_video_example_tools
from ameba_dev_mcp.resources.devices import register_device_resources
from ameba_dev_mcp.resources.boards import register_board_resources
from ameba_dev_mcp.resources.config import register_config_resources
from ameba_dev_mcp.resources.debug import register_debug_resources
from ameba_dev_mcp.prompts import register_prompts


# Configure logging
def setup_logging(level: int = logging.INFO) -> None:
    """Configure logging for the server."""
    logging.basicConfig(
        level=level,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        handlers=[
            logging.StreamHandler(sys.stderr)
        ]
    )


# Server state
class ServerState:
    """Server state container."""

    def __init__(self):
        self.serial_connections = {}
        self.monitor_threads = {}


@asynccontextmanager
async def app_lifespan(server: FastMCP) -> AsyncIterator[ServerState]:
    """
    Manage server lifecycle with resource cleanup.
    """
    state = ServerState()
    logging.info("AmebaPro2 MCP Server starting up...")

    yield state

    # Cleanup on shutdown
    logging.info("AmebaPro2 MCP Server shutting down...")
    cleanup_all_connections()
    logging.info("Cleanup complete")


# Create FastMCP server instance
mcp = FastMCP(
    name="AmebaPro2 SoC Development Server",
    lifespan=app_lifespan,
)


def register_all() -> None:
    """Register all tools and resources with the server."""
    # Register flash tools (Pro2: uses uartfwburn)
    register_flash_tools(mcp)
    logging.info("Registered flash tools (Pro2/uartfwburn)")

    # Register serial tools (shared with ameba-rtos)
    register_serial_tools(mcp)
    logging.info("Registered serial tools")

    # Register project tools (Pro2: cmake-based build)
    register_project_tools(mcp)
    logging.info("Registered project tools (Pro2/cmake)")

    # Register video example selector tools
    register_video_example_tools(mcp)
    logging.info("Registered video example tools")

    # Register environment pre-check tool
    register_env_check_tools(mcp)
    logging.info("Registered env-check tools")

    # Register device resources
    register_device_resources(mcp)
    logging.info("Registered device resources")

    # Register board / config resources
    register_board_resources(mcp)
    register_config_resources(mcp)
    logging.info("Registered board/config resources")

    # Register hardware-debug reference
    register_debug_resources(mcp)
    logging.info("Registered debug resources")

    # Register prompts (skills like /ameba-setup-boards)
    register_prompts(mcp)
    logging.info("Registered prompts")


def main() -> None:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="FastMCP Server for AmebaPro2 SoC Development"
    )
    # Pro2 MCP server runs over stdio only (launched by Claude Code via launcher.bat/sh).
    # HTTP/SSE transport is not supported — Pro2 does not use AmebaRemoteService.
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging"
    )

    args = parser.parse_args()

    # Setup logging
    log_level = logging.DEBUG if args.debug else logging.INFO
    setup_logging(log_level)

    # Register all tools and resources
    register_all()

    # Run server over stdio
    mcp.run()


if __name__ == "__main__":
    main()
