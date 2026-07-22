@echo off
REM Launcher for ameba-dev-pro2 MCP server.
REM Activates the Pro2 SDK venv and runs ameba-mcp.
REM
REM Usage (in claude_desktop_config.json or .mcp.json):
REM   "command": "<SDK_ROOT>\\tools\\ameba\\ameba_dev_mcp\\launcher.bat"
REM
REM No --project-root needed: Pro2 SDK root IS the project root.

setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
REM Go up three levels: ameba_dev_mcp -> ameba -> tools -> SDK_ROOT
for %%I in ("%SCRIPT_DIR%..\..\..\") do set "SDK_ROOT=%%~fI"
if "%SDK_ROOT:~-1%"=="\" set "SDK_ROOT=%SDK_ROOT:~0,-1%"

set "MCP_BIN=%SDK_ROOT%\.venv\Scripts\ameba-mcp.exe"

if not exist "%MCP_BIN%" (
    echo [ameba-mcp-pro2 launcher] ERROR: venv not found. 1>&2
    echo [ameba-mcp-pro2 launcher] Run "%SDK_ROOT%\env.bat" once to set up the environment, then restart Claude Code. 1>&2
    exit /b 1
)

call "%SDK_ROOT%\.venv\Scripts\activate.bat" 1>&2
"%MCP_BIN%" %*
exit /b %ERRORLEVEL%
