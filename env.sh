#!/usr/bin/env bash
# One-time Python venv setup for the ameba-dev-pro2 MCP server.
# Run this ONCE before registering the MCP server with Claude Code.
# After this, launcher.sh starts the MCP server instantly with no delay.
#
# Usage:
#   source env.sh       # (source so the venv is activated in your shell too)
#   bash env.sh         # (or just run it to set up without activating)
#
# Then register the MCP server (only needed once):
#   claude mcp add ameba-dev-pro2 -- "<SDK_ROOT>/tools/ameba/ameba_dev_mcp/launcher.sh"

set -e

if [ -n "${BASH_SOURCE-}" ]; then
    _SELF="${BASH_SOURCE[0]}"
elif [ -n "${ZSH_VERSION-}" ]; then
    _SELF="${(%):-%x}"
else
    _SELF="$0"
fi
BASE_DIR="$(cd "$(dirname "$_SELF")" && pwd)"

VENV="$BASE_DIR/.venv"
MCP_BIN="$VENV/bin/ameba-mcp"

echo "[ameba-dev-pro2] Setting up Python virtual environment..."

if [ -f "$VENV/bin/python" ] || [ -f "$VENV/Scripts/python.exe" ]; then
    echo "[ameba-dev-pro2] venv already exists, skipping creation."
else
    python3 -m venv "$VENV"
    echo "[ameba-dev-pro2] venv created at $VENV"
fi

echo "[ameba-dev-pro2] Installing / updating ameba-dev-mcp package..."
"$VENV/bin/pip" install -e "$BASE_DIR/tools/ameba"

# If sourced, activate the venv in the caller's shell too.
if [[ "${BASH_SOURCE[0]}" != "${0}" ]] || [[ -n "${ZSH_VERSION-}" && "${ZSH_EVAL_CONTEXT}" == *:file:* ]]; then
    # shellcheck disable=SC1091
    source "$VENV/bin/activate"
    echo "[ameba-dev-pro2] venv activated in current shell."
fi

echo ""
echo "================================================================================"
echo "  ameba-dev-pro2 MCP environment is ready."
echo ""
echo "  If not already added, register the MCP server with Claude Code:"
echo "    claude mcp add ameba-dev-pro2 -- \\"
echo "      \"$BASE_DIR/tools/ameba/ameba_dev_mcp/launcher.sh\""
echo "================================================================================"
