#!/usr/bin/env bash
# Launcher for ameba-dev-pro2 MCP server.
# Activates the Pro2 SDK venv and runs ameba-mcp.
#
# Usage (in claude_desktop_config.json or .mcp.json):
#   "command": "<SDK_ROOT>/tools/ameba/ameba_dev_mcp/launcher.sh"
#
# No --project-root needed: Pro2 SDK root IS the project root.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Go up three levels: ameba_dev_mcp -> ameba -> tools -> SDK_ROOT
SDK_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

MCP_BIN="${SDK_ROOT}/.venv/bin/ameba-mcp"

if [ ! -f "${MCP_BIN}" ]; then
    echo "[ameba-mcp-pro2 launcher] ERROR: venv not found." >&2
    echo "[ameba-mcp-pro2 launcher] Run 'source ${SDK_ROOT}/env.sh' once to set up the environment, then restart Claude Code." >&2
    exit 1
fi

source "${SDK_ROOT}/.venv/bin/activate"
exec "${MCP_BIN}" "$@"
