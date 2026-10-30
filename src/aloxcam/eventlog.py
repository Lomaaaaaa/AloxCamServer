"""LabVIEW-format event logs (``Log Server Error`` / ``Log Server Infos``) and the Python diagnostic log.

Spec: 09 §7.1–§7.4 (inputs, algorithm, file names, folders, line format), §7.5 (on-screen log → ring buffer, D19),
§7.7 (reference sketch and the separate diagnostic log); fixes D10 (append, switchable), D11 (info labels,
switchable), D13 (create ``logs``), D9 (folders resolved once); design §3.3.5.

Event-log lines go only through :class:`EventLogs` (the :class:`aloxcam.dispatch.EventSink` of the server). Everything
else (warnings, connections, tracebacks) goes to the stdlib ``logging`` diagnostic log configured by
:func:`setup_diagnostic_logging`.
"""

from __future__ import annotations

import codecs
import logging
import os
import sys
import threading
from collections import deque
from collections.abc import Callable, Mapping
from datetime import date, datetime
from pathlib import Path
from types import MappingProxyType
from typing import IO

from aloxcam.errors import ErrorCluster
from aloxcam.settings import HIGH_MAG, LOW_MAG, Folders, LoggingOptions

__all__ = ["STEM", "DailyFileHandler", "EventLogs", "setup_diagnostic_logging"]

log = logging.getLogger(__name__)

#: ``Server Type`` → file stem (09 §7.3); any unlisted value uses ``camera``.
STEM: Mapping[str, str] = MappingProxyType({"Camera": "camera", "Monochromator": "wavelength", "XY Stages": "xy",
                                            "Z Stage": "z"})
_DEFAULT_STEM = "camera"
_CAMERA = "Camera"
#: ``Log Server Error`` tags of the non-camera server types (09 §7.2); unlisted → ``ERR -``.
_ERR_TAG: Mapping[str, str] = MappingProxyType({"Z Stage": "ERR (Z STAGE)-", "Monochromator": "ERR (MONO WL)-",
                                                "XY Stages": "ERR (XY STAGE)-"})
_EOL = "\r\n"                                            # LabVIEW End-of-Line on Windows (09 §7.3)
_ROOT_LOGGER = "aloxcam"
_DIAG_FORMAT = "%(asctime)s %(levelname)s %(name)s [%(threadName)s] %(message)s"
_diag_handlers: list[logging.Handler] = []               # installed by setup_diagnostic_logging (replaced on each call)


def _stamp(when: datetime, tag: str, decimal_separator: str) -> str:
    """``Format Date/Time String("%d/%m/%Y - %H:%M:%S%3u - " + tag)`` (09 §7.2, §7.4)."""
    return f"{when:%d/%m/%Y - %H:%M:%S}{decimal_separator}{when.microsecond // 1000:03d} - {tag}"


def _log_file_name(when: datetime, stem: str) -> str:
    """``RemoveChars(FormatDateTime("%y%m%d_<stem>_events"), ".", ",") + ".log"`` (09 §7.2)."""
    return f"{when:%y%m%d}_{stem}_events".replace(".", "").replace(",", "") + ".log"


class EventLogs:
    """``Log Server Error.vi`` / ``Log Server Infos.vi`` (09 §7); implements :class:`aloxcam.dispatch.EventSink`.

    Thread-safe: one lock around formatting and I/O (a leaf lock, design §2). Each line is written by opening and
    closing the daily file (09 §7.3), so several processes can share it. Write failures are swallowed as LabVIEW's
    ``Clear Errors`` does and reported once per file in the diagnostic log. ``recent()`` keeps the last
    ``recent_lines`` lines (the on-screen log, 09 §7.5, D19 fix).
    """

    def __init__(self, folders: Folders, options: LoggingOptions, *, now: Callable[[], datetime] = datetime.now,
                 recent_lines: int = 1000) -> None:
        self.folders = folders
        self.options = options
        self._now = now
        self._lock = threading.Lock()
        self._recent: deque[str] = deque(maxlen=max(0, recent_lines))
        self._reported: set[str] = set()
        try:
            codecs.lookup(options.encoding)
            self._encoding = options.encoding
        except LookupError:
            log.warning("event logs: unknown encoding %r; cp1252 used", options.encoding)
            self._encoding = "cp1252"

    # -- folders and files
    def _logs_dirs(self) -> list[tuple[str, Path | None]]:
        return [("vision", self.folders.vision), ("data", self.folders.data)]

    def ensure_dirs(self) -> list[str]:
        """Create ``<vision>/logs`` and ``<data>/logs`` when ``create_log_dirs`` (09 D13 fix); returns warnings
        (folder unavailable, creation failed, or missing with creation disabled). Never raises."""
        warnings: list[str] = []
        for name, folder in self._logs_dirs():
            use = "error event log" if name == "vision" else "information event log"
            if folder is None:
                warnings.append(f"<{name}> folder unavailable: {use} disabled")
                continue
            logs = folder / "logs"
            if self.options.create_log_dirs:
                try:
                    logs.mkdir(parents=True, exist_ok=True)
                except OSError as exc:
                    warnings.append(f"{logs}: cannot be created ({exc}); {use} lines will be lost")
            elif not logs.is_dir():
                warnings.append(f"{logs}: missing and create_log_dirs = FALSE; {use} lines will be lost")
        return warnings

    @staticmethod
    def _path(folder: Path | None, server_type: str, when: datetime) -> Path | None:
        if folder is None:
            return None
        return folder / "logs" / _log_file_name(when, STEM.get(server_type, _DEFAULT_STEM))

    def error_log_path(self, server_type: str, when: datetime) -> Path | None:
        """``<vision>/logs/<yymmdd>_<stem>_events.log`` (09 §7.3); None when ``<vision>`` is unavailable."""
        return self._path(self.folders.vision, server_type, when)

    def info_log_path(self, server_type: str, when: datetime) -> Path | None:
        """``<data>/logs/<yymmdd>_<stem>_events.log`` (09 §7.3, D12 kept); None when ``<data>`` is unavailable."""
        return self._path(self.folders.data, server_type, when)

    # -- line format (09 §7.4)
    @staticmethod
    def format_error_line(when: datetime, err: ErrorCluster, server_type: str, extra: str, command: str | None,
                          camera_type: str, decimal_separator: str = ".") -> str:
        """``<dd/mm/YYYY> - <HH:MM:SS><sep><mmm> - <tag><command name>(<code>) : <source> (<extra>)``; the command
        name is printed for the ``Camera`` server type only (09 §7.2, §7.4)."""
        if server_type == _CAMERA:
            tag = "ERR (LOW MAG)-" if camera_type == LOW_MAG else "ERR (HI MAG)-"
            name = command or ""
        else:
            tag = _ERR_TAG.get(server_type, "ERR -")
            name = ""
        return f"{_stamp(when, tag, decimal_separator)}{name}({err.code}) : {err.source} ({extra})"

    @staticmethod
    def format_info_line(when: datetime, err: ErrorCluster, server_type: str, extra: str, camera_type: str,
                         decimal_separator: str = ".", fix_info_labels: bool = True) -> str:
        """``<dd/mm/YYYY> - <HH:MM:SS><sep><mmm> - <tag>(<code>) : <source> (<extra>)``. Camera tags are
        ``INFO (HI)-`` / ``INFO (LO)-``, swapped as in LabVIEW when ``fix_info_labels`` is FALSE (09 D11)."""
        if server_type == _CAMERA:
            low = camera_type == LOW_MAG
            if not fix_info_labels:
                low = not low
            tag = "INFO (LO)-" if low else "INFO (HI)-"
        else:
            tag = "INFO -"
        return f"{_stamp(when, tag, decimal_separator)}({err.code}) : {err.source} ({extra})"

    # -- EventSink
    def error(self, err: ErrorCluster, server_type: str, extra: str, *, command: str | None = "Get Heartbeat",
              camera_type: str = HIGH_MAG) -> ErrorCluster:
        """``Log Server Error``: writes only if ``err.status`` (warnings ignored); returns ``err`` (09 §7.1).
        Appends, or writes at offset 0 when ``append_errors`` is FALSE (09 D10)."""
        if not err.status:
            return err
        with self._lock:
            when = self._now()
            line = self.format_error_line(when, err, server_type, extra, command, camera_type,
                                          self.options.decimal_separator)
            self._recent.append(line)
            self._write(self.error_log_path(server_type, when), "vision", line, append=self.options.append_errors)
        return err

    def info(self, err: ErrorCluster, server_type: str, extra: str, *,
             camera_type: str = HIGH_MAG) -> ErrorCluster:
        """``Log Server Infos``: always writes (appends); returns ``err`` (09 §7.1)."""
        with self._lock:
            when = self._now()
            line = self.format_info_line(when, err, server_type, extra, camera_type, self.options.decimal_separator,
                                         self.options.fix_info_labels)
            self._recent.append(line)
            self._write(self.info_log_path(server_type, when), "data", line, append=True)
        return err

    def recent(self) -> list[str]:
        """The last ``recent_lines`` lines, oldest first (09 §7.5, D19)."""
        with self._lock:
            return list(self._recent)

    # -- I/O (called with the lock held)
    def _report_once(self, key: str, message: str, *args: object) -> None:
        if key not in self._reported:
            self._reported.add(key)
            log.warning(message + "; further failures of this file are not reported", *args)

    def _write(self, path: Path | None, folder_name: str, line: str, *, append: bool) -> None:
        if path is None:
            self._report_once(f"<{folder_name}>", "event log: <%s> folder unavailable, line not written: %s",
                              folder_name, line)
            return
        data = (line + _EOL).encode(self._encoding, errors="replace")
        try:
            if self.options.create_log_dirs:
                path.parent.mkdir(parents=True, exist_ok=True)
            if append:
                with open(path, "ab") as f:
                    f.write(data)
            else:                                        # LabVIEW Log Server Error: Set File Position(start, 0)
                fd = os.open(path, os.O_WRONLY | os.O_CREAT | getattr(os, "O_BINARY", 0), 0o666)
                try:
                    view = memoryview(data)
                    while view:
                        view = view[os.write(fd, view):]
                finally:
                    os.close(fd)
        except OSError as exc:                           # LabVIEW: Clear Errors (09 §7.2)
            self._report_once(str(path), "event log %s: write failed (%s), line lost: %s", path, exc, line)


# ---------------------------------------------------------------------------------------------- diagnostic log
class DailyFileHandler(logging.Handler):
    """Writes to ``<directory>/<yymmdd>_<stem>.log`` and switches file when the local date of the record changes.

    The file is opened in append mode on first use; ``create_dir`` creates ``directory`` when missing. When a file
    cannot be opened, the failure is reported once (``handleError``) and records are dropped until the date changes.
    """

    def __init__(self, directory: Path, stem: str, *, encoding: str = "utf-8", create_dir: bool = True,
                 level: int = logging.NOTSET) -> None:
        super().__init__(level)
        self.directory = Path(directory)
        self.stem = stem
        self.encoding = encoding
        self.create_dir = create_dir
        self._stream: IO[str] | None = None
        self._date: date | None = None
        self._failed_date: date | None = None

    def path_for(self, day: date) -> Path:
        """The file of ``day``."""
        return self.directory / f"{day:%y%m%d}_{self.stem}.log"

    @property
    def current_path(self) -> Path | None:
        """The file in use, or None before the first record."""
        return None if self._date is None else self.path_for(self._date)

    def _open(self, day: date) -> IO[str]:
        if self._stream is not None:
            self._stream.close()
            self._stream = None
        self._date = day
        if self.create_dir:
            self.directory.mkdir(parents=True, exist_ok=True)
        self._stream = open(self.path_for(day), "a", encoding=self.encoding, errors="replace")
        return self._stream

    def emit(self, record: logging.LogRecord) -> None:
        day = datetime.fromtimestamp(record.created).date()
        if day == self._failed_date:
            return
        try:
            stream = self._stream if (self._stream is not None and day == self._date) else self._open(day)
            stream.write(self.format(record) + "\n")
            stream.flush()
        except Exception:
            self._failed_date = day
            self.handleError(record)

    def close(self) -> None:
        self.acquire()
        try:
            if self._stream is not None:
                self._stream.close()
                self._stream = None
        finally:
            self.release()
            super().close()


def setup_diagnostic_logging(folders: Folders | None, options: LoggingOptions, *, level: str | None = None,
                             stderr: bool = True) -> list[logging.Handler]:
    """Configure the ``aloxcam`` logger (09 §7.7): a :class:`DailyFileHandler` at
    ``<vision>/logs/<yymmdd>_<diag_file_stem>.log`` (UTF-8) when ``<vision>`` is known, and a stderr handler.

    ``level`` (e.g. ``--log-level``) overrides ``options.diag_level``. Handlers installed by an earlier call are
    removed and closed first, so the call is idempotent. Returns the handlers added.
    """
    logger = logging.getLogger(_ROOT_LOGGER)
    while _diag_handlers:
        old = _diag_handlers.pop()
        logger.removeHandler(old)
        old.close()
    name = (level or options.diag_level or "INFO").strip().upper()
    numeric = logging.getLevelName(name)
    if not isinstance(numeric, int):
        log.warning("unknown diagnostic log level %r; INFO used", name)
        numeric = logging.INFO
    logger.setLevel(numeric)
    formatter = logging.Formatter(_DIAG_FORMAT)
    handlers: list[logging.Handler] = []
    if folders is not None and folders.vision is not None:
        handlers.append(DailyFileHandler(folders.vision / "logs", options.diag_file_stem,
                                         create_dir=options.create_log_dirs))
    if stderr:
        handlers.append(logging.StreamHandler(sys.stderr))
    for handler in handlers:
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        _diag_handlers.append(handler)
    return handlers
