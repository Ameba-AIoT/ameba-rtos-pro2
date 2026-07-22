@echo off
REM One-time Python venv setup for the ameba-dev-pro2 MCP server.
REM Run this ONCE before registering the MCP server with Claude Code.
REM After this, launcher.bat starts the MCP server instantly with no delay.
REM
REM Usage:
REM   env.bat
REM
REM Then register the MCP server (only needed once):
REM   claude mcp add ameba-dev-pro2 -- "<SDK_ROOT>\tools\ameba\ameba_dev_mcp\launcher.bat"

setlocal enabledelayedexpansion

set "BASE_DIR=%~dp0"
if "%BASE_DIR:~-1%"=="\" set "BASE_DIR=%BASE_DIR:~0,-1%"

set "VENV=%BASE_DIR%\.venv"
set "VENV_PYTHON=%VENV%\Scripts\python.exe"
set "MCP_BIN=%VENV%\Scripts\ameba-mcp.exe"

echo [ameba-dev-pro2] Setting up Python virtual environment...

if exist "%VENV_PYTHON%" (
    echo [ameba-dev-pro2] venv already exists, skipping creation.
) else (
    python -m venv "%VENV%"
    if errorlevel 1 (
        echo [ameba-dev-pro2] ERROR: failed to create venv. Is Python ^(3.8+^) on PATH?
        exit /b 1
    )
    echo [ameba-dev-pro2] venv created at %VENV%
)

echo [ameba-dev-pro2] Installing / updating ameba-dev-mcp package...
"%VENV%\Scripts\pip" install -e "%BASE_DIR%\tools\ameba"
if errorlevel 1 (
    echo [ameba-dev-pro2] ERROR: pip install failed.
    exit /b 1
)

echo.
echo ================================================================================
echo   ameba-dev-pro2 MCP environment is ready.
echo.
echo   If not already added, register the MCP server with Claude Code:
echo     claude mcp add ameba-dev-pro2 -- 
echo       "%BASE_DIR%\tools\ameba\ameba_dev_mcp\launcher.bat"
echo ================================================================================
