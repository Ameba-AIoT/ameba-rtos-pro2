"""
Alias-driven serial session manager.

Maps `alias -> ActiveSession`. Each session wraps a local pyserial.Serial.
The MCP serial / flash / reset_device tools take only an alias and resolve
the actual board from board_info.json5 via this manager.

A single global `manager` is exposed; tests can construct their own
`BoardSessionManager()` for isolation.
"""

from __future__ import annotations

import atexit
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import serial
from serial.serialutil import SerialException

from ameba_dev_mcp.config import serial_log
from ameba_dev_mcp.models.schemas import ConfigError, ResolvedBoard

logger = logging.getLogger(__name__)


class BoardSessionError(Exception):
    """Connection-time failure carrying a structured ConfigError."""
    def __init__(self, error: ConfigError):
        self.error = error
        super().__init__(error.message)


# ---------------------------------------------------------------------------
# Session record
# ---------------------------------------------------------------------------

@dataclass
class ActiveSession:
    alias: str
    board: ResolvedBoard
    connection: object              # serial.Serial
    is_remote: bool = False
    aag_parser: object = None       # populated lazily by serial_read tool
    reader: object = None           # LoggingReader (sole RX consumer) when logging on
    serial_logger: object = None    # SerialLogger (owns the .log file) when logging on
    created_at: float = field(default_factory=time.monotonic)
    last_used_at: float = field(default_factory=time.monotonic)

    def is_open(self) -> bool:
        c = self.connection
        if hasattr(c, "is_open"):
            v = c.is_open
            return bool(v() if callable(v) else v)
        if hasattr(c, "isOpen"):
            return bool(c.isOpen())
        return False

    def close(self) -> None:
        # Stop the background log pump FIRST so it stops reading before the
        # underlying connection is torn down, then flush/close the log file.
        if self.reader is not None:
            try:
                self.reader.stop()
            except Exception as ex:
                logger.warning("Error stopping log reader %s: %s", self.alias, ex)
        if self.serial_logger is not None:
            try:
                self.serial_logger.close()
            except Exception as ex:
                logger.warning("Error closing serial log %s: %s", self.alias, ex)
        try:
            self.connection.close()
        except Exception as ex:
            logger.warning("Error closing session %s: %s", self.alias, ex)

    def touch(self) -> None:
        self.last_used_at = time.monotonic()


# ---------------------------------------------------------------------------
# Manager
# ---------------------------------------------------------------------------

_IDLE_TTL_SECONDS = 30 * 60  # 30 min: same as the legacy connection_id reaper


class BoardSessionManager:
    """Process-wide singleton mapping alias -> ActiveSession."""

    def __init__(self):
        self._lock = threading.RLock()
        self._sessions: Dict[str, ActiveSession] = {}
        self._reaper_started = False

    # -- lookup ------------------------------------------------------------

    def get(self, alias: str) -> Optional[ActiveSession]:
        with self._lock:
            return self._sessions.get(alias)

    def aliases(self) -> List[str]:
        with self._lock:
            return list(self._sessions.keys())

    def find_local_holder(self, device: str) -> Optional[str]:
        """Return the alias of an OPEN local session bound to `device`, or None.

        Used by `list_serial_ports_tool` to flag held_by_self for local ports.
        """
        with self._lock:
            for alias, sess in self._sessions.items():
                if not sess.is_remote and sess.board.port == device and sess.is_open():
                    return alias
        return None

    # -- connect / disconnect ---------------------------------------------

    def connect(self, board: ResolvedBoard, *, monitor_mode: bool = False) -> ActiveSession:
        """Open (or reuse) a serial session for `board`.

        `monitor_mode=True` uses board.monitor_baudrate; otherwise the
        flash baudrate. Idempotent: a second call returns the existing
        session if it's still open.
        """
        alias = board.alias
        with self._lock:
            existing = self._sessions.get(alias)
            if existing is not None and existing.is_open():
                existing.touch()
                return existing
            if existing is not None:
                # stale entry; drop it
                self._sessions.pop(alias, None)

            baud = board.monitor_baudrate if monitor_mode else board.baudrate
            conn = self._open_local(board, baud)

            sess = ActiveSession(
                alias=alias,
                board=board,
                connection=conn,
            )
            # Attach background log capture if this board enabled it. The
            # LoggingReader becomes the sole RX consumer; failure here is
            # non-fatal — the tools fall back to reading `conn` directly.
            try:
                attached = serial_log.attach(conn, board)
                if attached is not None:
                    sess.reader, sess.serial_logger = attached
            except Exception as ex:
                logger.warning("serial-log attach failed for %s: %s", alias, ex)
            self._sessions[alias] = sess
            self._ensure_reaper()
            return sess

    def disconnect(self, alias: str) -> bool:
        with self._lock:
            sess = self._sessions.pop(alias, None)
        if sess is None:
            return False
        sess.close()
        return True

    def disconnect_all(self) -> None:
        with self._lock:
            sessions = list(self._sessions.values())
            self._sessions.clear()
        for s in sessions:
            s.close()

    # -- transport-specific opens -----------------------------------------

    @staticmethod
    def _open_local(board: ResolvedBoard, baudrate: int) -> serial.Serial:
        try:
            return serial.Serial(port=board.port, baudrate=baudrate, timeout=1.0)
        except SerialException as ex:
            msg = str(ex).lower()
            if "no such file" in msg or "could not open port" in msg and "errno 2" in msg:
                code = "PORT_NOT_FOUND"
                hint = (
                    f"Serial port '{board.port}' does not exist. Check the USB cable "
                    f"and update boards.{board.alias}.port in board_info.json5."
                )
            elif "permission denied" in msg or "errno 13" in msg:
                code = "PORT_BUSY"
                hint = (
                    f"Permission denied on '{board.port}'. Either another process holds "
                    "it (close monitors / VS Code serial views) or your user is not in the "
                    "dialout/uucp group."
                )
            elif "device or resource busy" in msg or "errno 16" in msg:
                code = "PORT_BUSY"
                hint = f"'{board.port}' is held by another process. Close it and retry."
            else:
                code = "PORT_OPEN_FAILED"
                hint = f"Failed to open '{board.port}': {ex}"
            raise BoardSessionError(ConfigError(
                code=code,
                field_path=f"boards.{board.alias}.port",
                message=f"Cannot open serial port '{board.port}' for board '{board.alias}': {ex}",
                hint=hint,
            ))
        except Exception as ex:
            raise BoardSessionError(ConfigError(
                code="PORT_OPEN_FAILED",
                field_path=f"boards.{board.alias}.port",
                message=f"Unexpected error opening '{board.port}': {ex}",
            ))

    # -- idle reaper ------------------------------------------------------

    def _ensure_reaper(self) -> None:
        if self._reaper_started:
            return
        self._reaper_started = True
        t = threading.Thread(
            target=self._reaper_loop, daemon=True, name="board-session-reaper",
        )
        t.start()

    def _reaper_loop(self) -> None:
        while True:
            time.sleep(60)
            try:
                self._reap_expired()
            except Exception as ex:
                logger.debug("reaper iteration failed: %s", ex)

    def _reap_expired(self) -> None:
        now = time.monotonic()
        expired: List[str] = []
        with self._lock:
            for a, s in self._sessions.items():
                if now - s.last_used_at > _IDLE_TTL_SECONDS:
                    expired.append(a)
        for a in expired:
            logger.info("Reaping idle board session '%s'", a)
            self.disconnect(a)


# ---------------------------------------------------------------------------
# Process-wide singleton
# ---------------------------------------------------------------------------

manager = BoardSessionManager()


def _atexit_cleanup() -> None:
    try:
        manager.disconnect_all()
    except Exception:
        pass


atexit.register(_atexit_cleanup)
