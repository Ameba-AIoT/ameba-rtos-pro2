"""
Project-level tools for AmebaPro2 SoC: build (CMake-based).

Pro2 uses a CMake build system — not ameba.py.

Build sequence:
  First time  : cmake .. -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake
  Subsequent  : cmake --build . --target flash
  Output      : <cmake_build_dir>/flash_ntz.bin

Platform differences:
  Windows  — cmake must run inside msys2 bash (msys64_v10_3).
             msys64 root is auto-detected or set via AMEBA_MSYS64_DIR env var.
             Toolchain (asdk-10.3.0-mingw32-*) is added to msys2 .bashrc by the user.
  Linux    — cmake runs directly; toolchain (asdk-10.3.0-linux-*) added to PATH.
  macOS    — same as Linux but uses asdk-10.3.0-darwin-*.

project_info.json5 stores the cmake_project_dir, build_dir and the path
to the output flash_ntz.bin under images[0].path. After a successful
build, project_info.json5 is updated automatically.
"""

import hashlib
import json
import logging
import os
import re
import shutil
import subprocess
import sys
from typing import List, Optional

from mcp.server.fastmcp import FastMCP

from ameba_dev_mcp._paths import PROJECT_ROOT, SDK_ROOT

logger = logging.getLogger(__name__)

_ANSI_RE = re.compile(r"\x1b\[[0-9;]*[mK]")
_IS_WINDOWS = sys.platform == "win32"

# Default CMake project path relative to SDK_ROOT
_DEFAULT_CMAKE_PROJECT_DIR = os.path.join(
    SDK_ROOT,
    "project",
    "realtek_amebapro2_v0_example",
    "GCC-RELEASE",
)
_DEFAULT_BUILD_DIR = os.path.join(_DEFAULT_CMAKE_PROJECT_DIR, "build")
_FLASH_NTZ_BIN = "flash_ntz.bin"
_FLASH_NTZ_NN_BIN = "flash_ntz.nn.bin"

# Standard msys64 search locations (Windows only).
# User can override with AMEBA_MSYS64_DIR env var.
_MSYS64_CANDIDATES = [
    r"D:\msys64_v10_3\msys64",   # default from Pro2 tool release
    r"C:\msys64_v10_3\msys64",
    r"D:\msys64",
    r"C:\msys64",
    r"C:\tools\msys64",
    r"C:\msys2\msys64",
]


# ---------------------------------------------------------------------------
# Windows / msys2 helpers
# ---------------------------------------------------------------------------

def _find_msys64() -> Optional[str]:
    """Return the msys64 root directory, or None if not found.

    Checks AMEBA_MSYS64_DIR env var first, then common install locations.
    """
    env_dir = os.environ.get("AMEBA_MSYS64_DIR", "").strip()
    if env_dir:
        bash = os.path.join(env_dir, "usr", "bin", "bash.exe")
        if os.path.isfile(bash):
            return env_dir
        logger.warning("AMEBA_MSYS64_DIR=%r set but bash.exe not found there", env_dir)

    for candidate in _MSYS64_CANDIDATES:
        bash = os.path.join(candidate, "usr", "bin", "bash.exe")
        if os.path.isfile(bash):
            return candidate
    return None


def _win_path_to_posix(path: str) -> str:
    """Convert a Windows absolute path to an msys2 POSIX path.

    e.g.  D:\\foo\\bar  →  /d/foo/bar
          C:\\Users\\x  →  /c/Users/x
    """
    path = path.replace("\\", "/")
    if len(path) >= 2 and path[1] == ":":
        drive = path[0].lower()
        path = f"/{drive}{path[2:]}"
    return path


def _win_junction_candidates() -> List[str]:
    """Return the ordered list of junction path candidates for this SDK_ROOT.

    Each candidate is a short drive-root path suffixed with an 8-char hash of
    SDK_ROOT so that multiple repos on the same machine never share a name:
        D:\amebapro2_sdk_a1b2c3d4
        C:\amebapro2_sdk_a1b2c3d4
    The hash is deterministic — the same SDK_ROOT always produces the same
    candidate list, so the junction is reused across MCP restarts.
    """
    suffix = hashlib.sha1(SDK_ROOT.encode()).hexdigest()[:8]
    name = f"amebapro2_sdk_{suffix}"
    return [
        f"D:\\{name}",
        f"C:\\{name}",
    ]


def _ensure_win_junction() -> Optional[str]:
    """On Windows, ensure a short drive-root junction pointing to SDK_ROOT exists.

    Tries each candidate in order (D:\\amebapro2_sdk_<hash>, C:\\amebapro2_sdk_<hash>).
    The junction name is derived from a hash of SDK_ROOT, so different repos
    get different junction names and never collide with each other.
    - If a candidate already exists and resolves to SDK_ROOT → no-op, return it.
    - If a candidate path does not exist → create it with mklink /J and return it.
    - If a candidate exists but points elsewhere → skip (don't clobber).

    Returns the junction path on success, or None if no junction could be
    created (e.g. insufficient privileges).  Logs a warning on failure.
    """
    if not _IS_WINDOWS:
        return None

    norm_sdk = os.path.normcase(os.path.realpath(SDK_ROOT))

    for junction in _win_junction_candidates():
        if os.path.isdir(junction):
            # Already exists — verify it points to the right place
            if os.path.normcase(os.path.realpath(junction)) == norm_sdk:
                logger.debug("Junction already exists and is correct: %s", junction)
                return junction
            else:
                logger.warning(
                    "Junction %s exists but points elsewhere — skipping", junction
                )
                continue

        # Doesn't exist — try to create it
        drive = os.path.splitdrive(junction)[0]
        if not os.path.isdir(drive + "\\"):
            continue  # drive doesn't exist on this machine

        cmd = ['cmd', '/c', 'mklink', '/J', junction, SDK_ROOT]
        logger.info("Creating junction: mklink /J %s %s", junction, SDK_ROOT)
        ret = subprocess.call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if ret == 0 and os.path.isdir(junction):
            logger.info("Junction created: %s -> %s", junction, SDK_ROOT)
            return junction
        else:
            logger.warning("Failed to create junction %s (ret=%d)", junction, ret)

    return None


def _shorten_win_path(path: str) -> str:
    """On Windows, return a shorter equivalent path via a drive-root junction.

    The AmebaPro2 SDK may sit under a very long Windows path
    (e.g. D:\\ray_huang\\Desktop\\training\\mcp_sdk_development\\ameba-rtos-pro2).
    cmake bakes every source/include path into the generated Makefiles; if those
    paths are long, the compile command lines exceed the Windows 32 767-char
    CreateProcess limit ("Argument list too long").

    A Windows junction at D:\\amebapro2_sdk pointing to the SDK root makes all
    paths much shorter.  If D:\\amebapro2_sdk exists and resolves to the same
    real directory as SDK_ROOT, we rewrite ``path`` through the junction.

    Returns the original ``path`` unchanged if no shortening is possible.
    """
    if not _IS_WINDOWS:
        return path

    norm_sdk = os.path.normcase(os.path.realpath(SDK_ROOT))
    norm_path = os.path.normcase(os.path.realpath(path))
    real_sdk  = os.path.realpath(SDK_ROOT)
    real_path = os.path.realpath(path)

    for junction in _win_junction_candidates():
        if not os.path.isdir(junction):
            continue
        # Use realpath to follow the junction to its actual target
        norm_junction_target = os.path.normcase(os.path.realpath(junction))
        if norm_junction_target != norm_sdk:
            continue
        # path must be inside SDK_ROOT
        if not norm_path.startswith(norm_sdk):
            continue
        # Use case-preserved realpath for relpath to avoid lowercasing the result
        rel = os.path.relpath(real_path, real_sdk)
        shortened = os.path.join(junction, rel)
        logger.debug("Shortened path: %s -> %s", path, shortened)
        return shortened
    return path


def _make_msys2_cmd(msys64_dir: str, build_dir_posix: str,
                    cmake_args: List[str]) -> List[str]:
    """Wrap a cmake command in a msys2 bash --login -c script.

    `-c` makes the shell non-interactive, so the user's ~/.bashrc hits its
    own `[[ "$-" != *i* ]] && return` guard and never reaches the toolchain /
    cmake PATH exports the official Pro2 setup guide has users add there.
    We don't rely on ~/.bashrc (or MSYS2_PATH_EXTRA, which this msys2 build's
    /etc scripts don't read either) — instead we locate cmake and the ARM
    toolchain ourselves and export PATH directly inside the script.

    Each argument is individually shell-quoted with shlex.quote so that
    arguments containing spaces (e.g. "-G Unix Makefiles") survive the
    bash -c round-trip correctly.
    """
    import shlex
    bash_exe = os.path.join(msys64_dir, "usr", "bin", "bash.exe")
    cmake_str = " ".join(shlex.quote(a) for a in cmake_args)
    path_prefix = _msys2_extra_path_prefix(msys64_dir)
    shell_script = f'{path_prefix}cd "{build_dir_posix}" && {cmake_str}'
    return [bash_exe, "--login", "-c", shell_script]


_WIN_CMAKE_CANDIDATES = [
    r"C:\Program Files\CMake\bin",
    r"C:\Program Files (x86)\CMake\bin",
    r"D:\Program Files\CMake\bin",
    r"D:\CMake\bin",
]

# ARM toolchain version fallback order, mirrors the official ~/.bashrc setup
# script (asdk-10.3.0 preferred, older SDKs may only have 8.x installed).
_ASDK_VERSION_CANDIDATES = ["asdk-10.3.0", "asdk-8.4.0", "asdk-8.3.0"]


def _find_win_cmake_dir() -> Optional[str]:
    """Return the Windows cmake bin dir if cmake.exe is found.

    Search order:
      1. System PATH (shutil.which) — picks up cmake installed via winget,
         the official cmake installer with 'Add to PATH', or any custom install.
      2. Well-known fallback locations (_WIN_CMAKE_CANDIDATES).
    """
    # 1. Try system PATH first
    cmake_on_path = shutil.which("cmake")
    if cmake_on_path:
        return os.path.dirname(os.path.abspath(cmake_on_path))

    # 2. Fall back to known install locations
    for d in _WIN_CMAKE_CANDIDATES:
        if os.path.isfile(os.path.join(d, "cmake.exe")):
            return d
    return None


def _find_asdk_toolchain_dir(msys64_dir: str) -> Optional[str]:
    """Return the POSIX path to the Pro2 ARM toolchain bin dir inside msys64,
    or None if not found (e.g. `D:\\msys64_v10_3\\msys64\\asdk-10.3.0\\mingw32\\newlib\\bin`).
    """
    for name in _ASDK_VERSION_CANDIDATES:
        bin_dir = os.path.join(msys64_dir, name, "mingw32", "newlib", "bin")
        if os.path.isdir(bin_dir):
            return _win_path_to_posix(bin_dir)
    return None


def _msys2_extra_path_prefix(msys64_dir: str) -> str:
    """Return a shell snippet exporting cmake + ARM toolchain onto PATH.

    Empty string if neither is found (caller falls back to whatever ~/.bashrc
    or the default msys2 PATH already provides).
    """
    extra_dirs = []
    toolchain_dir = _find_asdk_toolchain_dir(msys64_dir)
    if toolchain_dir:
        extra_dirs.append(toolchain_dir)
    cmake_dir = _find_win_cmake_dir()
    if cmake_dir:
        extra_dirs.append(_win_path_to_posix(cmake_dir))
    if not extra_dirs:
        return ""
    return f'export PATH="{":".join(extra_dirs)}:$PATH"; '


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _read_current_soc() -> Optional[str]:
    info_file = os.path.join(PROJECT_ROOT, "soc_info.json")
    if not os.path.exists(info_file):
        return None
    try:
        with open(info_file) as f:
            return json.load(f).get("soc", {}).get("name")
    except Exception:
        return None


def _write_soc_info(soc_name: str) -> None:
    info_file = os.path.join(PROJECT_ROOT, "soc_info.json")
    with open(info_file, "w", encoding="utf-8") as f:
        json.dump({"soc": {"name": soc_name}}, f, indent=4)


def _get_cmake_dirs(soc: str) -> tuple[str, str]:
    """Return (cmake_project_dir, build_dir) for the given SoC.

    Pro2 has a single fixed CMake project directory, so cmake_project_dir
    always falls back to the default. build_dir is read from
    project_info.json5 when available — it may already be a shortened
    junction path (D:\\amebapro2_sdk_<hash>\\...) written by a previous
    build, saving a redundant _shorten_win_path() call.
    Falls back to _DEFAULT_BUILD_DIR if not set.
    """
    try:
        from ameba_dev_mcp.config.loader import load_project_info

        pinfo = load_project_info(PROJECT_ROOT)
        entry = pinfo.projects.get(soc)
        if entry is not None:
            build_dir = getattr(entry, "build_dir", None)
            if build_dir:
                return _DEFAULT_CMAKE_PROJECT_DIR, build_dir
    except Exception:
        pass
    return _DEFAULT_CMAKE_PROJECT_DIR, _DEFAULT_BUILD_DIR


def _postprocess(raw: str) -> str:
    """Strip ANSI codes."""
    return _ANSI_RE.sub("", raw)


def _resolve_cmake_runner(build_dir: str) -> tuple[Optional[str], Optional[dict]]:
    """Return (msys64_dir_or_None, error_dict_or_None).

    On non-Windows, always returns (None, None) — cmake runs directly.
    On Windows, locates msys64 or returns an error dict describing the problem.
    """
    if not _IS_WINDOWS:
        return None, None

    msys64_dir = _find_msys64()
    if msys64_dir is None:
        return None, {
            "success": False,
            "summary": "msys64 not found — cannot build on Windows without it",
            "output": (
                "AmebaPro2 requires msys2/mingw to run cmake on Windows.\n\n"
                "Setup steps:\n"
                "  1. Download msys64_v10_3.7z from:\n"
                "     https://github.com/Ameba-AIoT/ameba-tool-rtos-pro2/releases/tag/msys64_v10_3\n"
                "  2. Extract to C:\\msys64_v10_3\\msys64  (or another path)\n"
                "  3. Download the Windows toolchain from:\n"
                "     https://github.com/Ameba-AIoT/ameba-toolchain/releases/tag/V10.3.0-amebe-rtos-pro2\n"
                "     (asdk-10.3.0-mingw32-newlib-build-3633-x86_64.zip)\n"
                "  4. Extract the toolchain and add its bin/ to msys2 ~/.bashrc:\n"
                "     export PATH=/path/to/asdk-10.3.0/mingw32/newlib/bin:$PATH\n"
                "  5. Install cmake in msys2 (https://cmake.org/download/, add to ~/.bashrc):\n"
                "     export PATH=/c/Program\\ Files/CMake/bin:$PATH\n"
                "  6. Set AMEBA_MSYS64_DIR env var if msys64 is not in a standard location:\n"
                "     AMEBA_MSYS64_DIR=C:\\your\\path\\msys64"
            ),
            "log_path": None,
            "soc": None,
        }
    logger.info("Using msys64 at: %s", msys64_dir)
    return msys64_dir, None


# ---------------------------------------------------------------------------
# Post-configure path patching (Windows only)
# ---------------------------------------------------------------------------

def _patch_cmake_build_files(build_dir: str) -> int:
    """After cmake configure on Windows/msys2, spill long make variables in
    ``flags.make`` files into GCC response files (``.rsp``) to work around the
    Windows 32 767-char ``CreateProcess`` command-line limit.

    cmake 3.20 does not yet support ``CMAKE_C_USE_RESPONSE_FILE_FOR_INCLUDES``
    (added in 3.21).  ``C_INCLUDES`` for the main application target expands to
    ~32 000 chars of ``-I<path>`` flags; combined with the rest of the compile
    command this exceeds Windows's limit and causes
    "Argument list too long" (E2BIG / Error 126) inside MSYS2's ``/bin/sh``.

    Fix: for every ``flags.make`` found under ``build_dir``, extract make
    variable values that exceed ``_RSP_THRESHOLD`` chars into a ``.rsp`` file
    alongside the ``flags.make``, then replace the variable value with
    ``@<path_to_rsp>``.  GCC/G++ read the flags from that file directly,
    keeping the shell command line short.

    **Only** ``flags.make`` files are touched — in particular ``CMakeCache.txt``
    is intentionally left alone so cmake's compiler re-validation keeps working.

    Returns the number of ``flags.make`` files that were modified.
    """
    import re as _re

    # Make variable lines that may be very long in flags.make
    # e.g.  C_INCLUDES = -ID:/long/path1 -ID:/long/path2 ...
    VAR_LINE_RE = _re.compile(
        r'^(C_INCLUDES|C_DEFINES|CXX_INCLUDES|CXX_DEFINES|ASM_INCLUDES)\s*=\s*(.+)$',
        _re.MULTILINE,
    )

    # Threshold: spill to rsp only when the value exceeds this many chars
    _RSP_THRESHOLD = 4096

    patched = 0

    for dirpath, dirnames, filenames in os.walk(build_dir):
        dirnames[:] = [d for d in dirnames if not d.startswith(".")]
        if "flags.make" not in filenames:
            continue
        fpath = os.path.join(dirpath, "flags.make")
        try:
            with open(fpath, "r", encoding="utf-8", errors="replace") as fh:
                content = fh.read()
        except Exception as exc:
            logger.warning("Could not read %s: %s", fpath, exc)
            continue

        def _maybe_rspify(m: "_re.Match") -> str:
            var_name = m.group(1)
            value = m.group(2)
            if len(value) <= _RSP_THRESHOLD:
                return m.group(0)
            rsp_name = f"{var_name.lower()}.rsp"
            rsp_path = os.path.join(dirpath, rsp_name)
            try:
                with open(rsp_path, "w", encoding="utf-8", newline="\n") as rh:
                    rh.write(value + "\n")
                # Use forward-slash path — arm-none-eabi-gcc.exe (MinGW binary)
                # handles both Windows and POSIX paths in @file arguments.
                rsp_fwd = rsp_path.replace("\\", "/")
                logger.debug("Wrote rsp: %s (%d chars)", rsp_path, len(value))
                return f"{var_name} = @{rsp_fwd}"
            except Exception as exc2:
                logger.warning("Could not write rsp %s: %s", rsp_path, exc2)
                return m.group(0)

        new_content = VAR_LINE_RE.sub(_maybe_rspify, content)
        if new_content != content:
            try:
                with open(fpath, "w", encoding="utf-8", newline="") as fh:
                    fh.write(new_content)
                patched += 1
            except Exception as exc:
                logger.warning("Could not write %s: %s", fpath, exc)

    logger.info("_patch_cmake_build_files: patched %d flags.make file(s) in %s",
                patched, build_dir)
    return patched


# Alias used in the configure block below
_patch_compiler_paths = _patch_cmake_build_files


# ---------------------------------------------------------------------------
# Core build logic
# ---------------------------------------------------------------------------

def run_build_quiet(
    soc: Optional[str] = None,
    clean: bool = False,
    pristine: bool = False,
    example: Optional[str] = None,
    video_example: bool = False,
    nn: bool = False,
) -> dict:
    """Run the Pro2 CMake build and return a result dict.

    Keys returned:
      success      bool
      summary      short human-readable status
      output       build log
      log_path     path to full log
      soc          the SoC that was built
      build_env    "windows/msys2" | "linux" | "darwin"
    """
    effective_soc = soc or _read_current_soc()
    if not effective_soc:
        return {
            "success": False,
            "summary": "No SoC selected. Pass soc= or run set_target first.",
            "output": "",
            "log_path": None,
            "soc": None,
        }

    cmake_project_dir, build_dir = _get_cmake_dirs(effective_soc)

    log_dir = os.path.join(PROJECT_ROOT, f"build_{effective_soc}")
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, "build.log")

    # Resolve msys64 (Windows only); fail fast if not found
    msys64_dir, env_err = _resolve_cmake_runner(build_dir)
    if env_err is not None:
        env_err["log_path"] = log_path
        env_err["soc"] = effective_soc
        return env_err

    build_env = (
        "windows/msys2" if _IS_WINDOWS
        else ("darwin" if sys.platform == "darwin" else "linux")
    )

    # On Windows, shorten cmake_project_dir and build_dir through the D:\sdk
    # junction (if available) so that all generated -I paths are short and stay
    # within the Windows 32 767-char CreateProcess limit.
    if _IS_WINDOWS:
        cmake_project_dir = _shorten_win_path(cmake_project_dir)
        build_dir = _shorten_win_path(build_dir)
        logger.info("cmake_project_dir (shortened): %s", cmake_project_dir)
        logger.info("build_dir (shortened): %s", build_dir)

    # Convert build_dir to POSIX once (used by all msys2 commands)
    build_dir_posix = _win_path_to_posix(build_dir) if _IS_WINDOWS else build_dir

    def _make_cmd(cmake_args: List[str]) -> List[str]:
        """Wrap cmake_args for the current platform."""
        if _IS_WINDOWS:
            return _make_msys2_cmd(msys64_dir, build_dir_posix, cmake_args)
        # Linux/macOS: run cmake directly; cwd= handled by subprocess
        return cmake_args

    build_timeout = int(os.environ.get("AMEBA_BUILD_TIMEOUT", "1800"))

    def _run(cmake_args: List[str]) -> subprocess.CompletedProcess:
        cmd = _make_cmd(cmake_args)
        logger.info("CMD: %s", " ".join(cmd))
        return subprocess.run(
            cmd,
            # cwd is only meaningful for non-Windows; Windows uses cd inside bash
            cwd=None if _IS_WINDOWS else build_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            timeout=build_timeout,
            # Always read as bytes; decode with utf-8 errors=replace afterward
            # to avoid cp950 / gbk codec failures on Windows with msys2 output.
        )

    try:
        # Ensure short junction exists on Windows BEFORE configure so cmake
        # bakes the short path into all generated Makefiles from the start.
        if _IS_WINDOWS:
            _ensure_win_junction()

        # Pristine: wipe build dir and start fresh
        if pristine and os.path.isdir(build_dir):
            import shutil
            shutil.rmtree(build_dir)
            logger.info("Pristine build: removed %s", build_dir)

        os.makedirs(build_dir, exist_ok=True)

        # Clean: remove intermediate artifacts (non-fatal)
        if clean and not pristine:
            try:
                _run(["cmake", "--build", ".", "--target", "clean"])
            except Exception:
                pass

        # Decide: configure (first time) or incremental build
        cmake_cache = os.path.join(build_dir, "CMakeCache.txt")
        if not os.path.exists(cmake_cache):
            configure_args = [
                "cmake", "..",
                "-G", "Unix Makefiles",
                "-DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake",
            ]
            # On Windows/msys2, pass compiler paths in POSIX form so cmake
            # caches short /d/... paths instead of long D:\... Windows paths.
            # Long Windows paths cause "Argument list too long" at compile time.
            if _IS_WINDOWS and msys64_dir is not None:
                toolchain_bin = _find_asdk_toolchain_dir(msys64_dir)
                if toolchain_bin:
                    configure_args += [
                        f"-DCMAKE_C_COMPILER={toolchain_bin}/arm-none-eabi-gcc",
                        f"-DCMAKE_CXX_COMPILER={toolchain_bin}/arm-none-eabi-g++",
                        f"-DCMAKE_ASM_COMPILER={toolchain_bin}/arm-none-eabi-gcc",
                    ]
            if video_example:
                # Video/NN examples use -DVIDEO_EXAMPLE=ON instead of -DEXAMPLE
                configure_args.append("-DVIDEO_EXAMPLE=ON")
            elif example:
                configure_args.append(f"-DEXAMPLE={example}")
            logger.info("Running CMake configure")
            proc_cfg = _run(configure_args)
            raw_cfg = proc_cfg.stdout.decode("utf-8", errors="replace")
            if proc_cfg.returncode != 0:
                stripped = _postprocess(raw_cfg)
                with open(log_path, "w", encoding="utf-8") as f:
                    f.write(stripped)
                return {
                    "success": False,
                    "summary": "CMake configure failed",
                    "output": stripped,
                    "log_path": log_path,
                    "soc": effective_soc,
                    "build_env": build_env,
                }

        # On Windows, cmake (a .exe) bakes the full Windows-style toolchain
        # path into every generated Makefile rule and expands C_INCLUDES to
        # ~32 000 chars, both of which cause "Argument list too long" inside
        # MSYS2's /bin/sh (Windows CreateProcess limit is 32 767 chars).
        #
        # cmake --build triggers cmake_check_build_system which may regenerate
        # flags.make AFTER our patch runs.  To prevent that:
        #   1. Run `make cmake_check_build_system` so cmake does its final
        #      regeneration first.
        #   2. Patch flags.make (spill C_INCLUDES into a .rsp response file).
        #   3. Run `make flash -j` directly — cmake_check_build_system fires
        #      again as a make dependency but cmake won't regenerate because no
        #      CMake inputs (CMakeLists.txt, toolchain.cmake, …) changed.
        # Determine build target and expected output binary
        build_target = "flash_nn" if nn else "flash"
        flash_bin_name = _FLASH_NTZ_NN_BIN if nn else _FLASH_NTZ_BIN

        if _IS_WINDOWS:
            logger.info("Running cmake_check_build_system (pre-patch)")
            _run(["make", "cmake_check_build_system"])
            logger.info("Patching flags.make files (C_INCLUDES → .rsp)")
            _patch_compiler_paths(build_dir)
            logger.info("Running make %s -j4", build_target)
            proc = _run(["make", build_target, "-j4"])
        else:
            logger.info("Running cmake --build . --target %s -j4", build_target)
            proc = _run(["cmake", "--build", ".", "--target", build_target, "-j4"])

    except subprocess.TimeoutExpired:
        return {
            "success": False,
            "summary": (
                f"Build timed out after {build_timeout}s "
                "(set AMEBA_BUILD_TIMEOUT env var to override)"
            ),
            "output": "",
            "log_path": log_path,
            "soc": effective_soc,
            "build_env": build_env,
        }
    except Exception as exc:
        return {
            "success": False,
            "summary": str(exc),
            "output": "",
            "log_path": None,
            "soc": effective_soc,
            "build_env": build_env,
        }

    raw = proc.stdout.decode("utf-8", errors="replace")
    stripped = _postprocess(raw)

    with open(log_path, "w", encoding="utf-8") as f:
        f.write(stripped)

    flash_ntz = os.path.join(build_dir, flash_bin_name)
    success = proc.returncode == 0 and os.path.isfile(flash_ntz)

    if proc.returncode == 0 and not os.path.isfile(flash_ntz):
        summary = f"Build succeeded but {flash_bin_name} not found in {build_dir}"
    elif success:
        has_warnings = ": warning:" in raw
        summary = "Build done (with warnings)" if has_warnings else "Build done"
    else:
        summary = "Build failed"

    project_info_status: Optional[dict] = None
    if success:
        project_info_status = _sync_project_info(effective_soc, flash_ntz, build_dir, cmake_project_dir)

    result = {
        "success": success,
        "summary": summary,
        "output": stripped,
        "log_path": log_path,
        "soc": effective_soc,
        "build_env": build_env,
    }
    if msys64_dir is not None:
        result["msys64_dir"] = msys64_dir
    if project_info_status is not None:
        result["project_info"] = project_info_status
    return result


def _sync_project_info(
    soc: str,
    flash_ntz: str,
    build_dir: str,
    cmake_project_dir: str,
) -> dict:
    """After a successful build, update project_info.json5 with the new flash_ntz.bin path."""
    try:
        from ameba_dev_mcp.config.loader import (
            ConfigLoadError,
            update_project_for_soc_pro2,
        )
    except Exception as ex:
        return {"updated": False, "error": f"import failed: {ex}"}

    try:
        info = update_project_for_soc_pro2(
            PROJECT_ROOT, soc, flash_ntz, build_dir, cmake_project_dir
        )
    except ConfigLoadError as ex:
        return {
            "updated": False,
            "soc": soc,
            "error": "; ".join(f"[{e.code}] {e.message}" for e in ex.errors),
        }
    except Exception as ex:
        return {"updated": False, "soc": soc, "error": str(ex)}

    try:
        size = os.path.getsize(flash_ntz)
    except OSError:
        size = None
    return {
        "updated": True,
        "soc": soc,
        "build_dir": build_dir,
        "image_count": 1,
        "images": [{"name": os.path.basename(flash_ntz), "size_bytes": size}],
    }


# ---------------------------------------------------------------------------
# MCP registration
# ---------------------------------------------------------------------------

def register_project_tools(mcp: FastMCP) -> None:
    """Register project-level MCP tools for AmebaPro2."""

    @mcp.tool()
    async def set_target(soc_name: str) -> dict:
        """Set the build target SoC. Must be called before build_firmware when
        switching targets. Subsequent build_firmware calls will use this target
        automatically without needing to pass any parameters.

        For AmebaPro2 the only valid SoC name is "RTL8735B" (as of this SDK
        version).

        Args:
            soc_name: SoC name to target (e.g. RTL8735B).

        Returns:
            success   Whether the target was set successfully.
            soc       The SoC name that was set.
        """
        try:
            _write_soc_info(soc_name)
            return {"success": True, "soc": soc_name}
        except Exception as exc:
            return {"success": False, "error": str(exc)}

    @mcp.tool()
    async def build_firmware(
        clean: bool = False,
        pristine: bool = False,
        summary_only: bool = True,
        alias: Optional[str] = None,
        example: Optional[str] = None,
        video_example: bool = False,
        nn: bool = False,
    ) -> dict:
        """Build Pro2 firmware using the CMake build system.

        Three build modes:
          Normal example  : -DEXAMPLE=<name>  → make flash    → flash_ntz.bin
          Video (no NN)   : -DVIDEO_EXAMPLE=ON → make flash    → flash_ntz.bin
          Video (with NN) : -DVIDEO_EXAMPLE=ON → make flash_nn → flash_ntz.nn.bin

        IMPORTANT — DO NOT call this tool in parallel for different SoCs.

        Args:
            clean:         Remove intermediate object files before rebuilding
                           (cmake --build . --target clean, then rebuild).
            pristine:      Remove the entire build directory and reconfigure.
                           WARNING: This resets ALL CMake cache — any custom
                           settings will be lost. ALWAYS confirm with the user
                           before passing pristine=True.
            summary_only:  Default True. On SUCCESSFUL builds, drop the
                           `output` field — the agent only needs success +
                           log_path. On FAILED builds, `output` is always
                           returned so the agent can read errors. Pass False
                           to always include `output`.
            alias:         Optional. Board alias from board_info.json5; the
                           alias's SoC is used as the build target, equivalent
                           to running set_target before this call. Omit to use
                           the SoC selected by the most recent set_target call.
            example:       Optional. Example name to build (e.g. "mqtt"). Sets
                           -DEXAMPLE=<name> during cmake configure. Only takes
                           effect on a fresh configure (no CMakeCache.txt); use
                           pristine=True to force reconfigure with a new example.
                           Ignored when video_example=True.
            video_example: Default False. When True, configure with
                           -DVIDEO_EXAMPLE=ON instead of -DEXAMPLE. Use for
                           all video/NN examples. Requires pristine=True when
                           switching from a non-video build.
            nn:            Default False. When True (and video_example=True),
                           build target is flash_nn and output is
                           flash_ntz.nn.bin. Set True for examples that include
                           a neural network model. Switching between nn=True
                           and nn=False does NOT require pristine=True — both
                           use -DVIDEO_EXAMPLE=ON; only the build target
                           differs.

        Returns:
            success         Whether the build succeeded.
            summary         One-line status ("Build done" / "Build failed" / …).
            output          CMake build log (omitted on success when summary_only=True).
            log_path        Absolute path to the full unstripped build log.
            soc             The SoC that was built.
            project_info    On success: {updated, soc, build_dir, image_count,
                            images: [{name, size_bytes}, ...]}.
            retargeted_from When `alias` triggered a SoC switch, the
                            previously-active SoC; otherwise omitted.
        """
        retargeted_from = None
        if alias is not None:
            try:
                from ameba_dev_mcp.config.loader import (
                    ConfigLoadError,
                    load_board_info,
                    resolve_alias_or_error,
                    resolve_board,
                )

                binfo = load_board_info(PROJECT_ROOT)
            except ConfigLoadError as ex:
                return {
                    "success": False,
                    "summary": "Failed to load board_info.json5",
                    "errors": [e.model_dump() for e in ex.errors],
                    "soc": None,
                    "log_path": None,
                }
            resolved, alias_errs = resolve_alias_or_error(binfo, alias)
            if alias_errs is not None:
                return {
                    "success": False,
                    "summary": f"alias '{alias}' resolution failed",
                    "errors": [e.model_dump() for e in alias_errs],
                    "soc": None,
                    "log_path": None,
                }
            target_soc = resolve_board(binfo, resolved).soc
            current_soc = _read_current_soc()
            if current_soc != target_soc:
                try:
                    _write_soc_info(target_soc)
                    retargeted_from = current_soc
                except Exception as ex:
                    return {
                        "success": False,
                        "summary": f"set_target({target_soc}) failed: {ex}",
                        "soc": current_soc,
                        "log_path": None,
                    }

        result = run_build_quiet(
            clean=clean,
            pristine=pristine,
            example=example,
            video_example=video_example,
            nn=nn,
        )
        if retargeted_from is not None:
            result["retargeted_from"] = retargeted_from
        if summary_only and result.get("success"):
            result.pop("output", None)
        return result
