"""aloxcam.eventlog: LabVIEW event-log lines, files and defects D10/D11/D13, and the diagnostic log
(09 §7, design §3.3.5)."""

from __future__ import annotations

import logging
import re
import threading
from datetime import datetime, timedelta
from pathlib import Path

import pytest

from aloxcam.dispatch import EventSink
from aloxcam.errors import NO_ERROR, ErrorCluster
from aloxcam.eventlog import STEM, DailyFileHandler, EventLogs, setup_diagnostic_logging
from aloxcam.settings import HIGH_MAG, LOW_MAG, Folders, LoggingOptions
from golden import spec_vectors as sv

WHEN = datetime(2026, 10, 29, 16, 31, 5, 123456)
Z_LINE, CAM_LINE, INFO_LINE = sv.LOG_LINES


class Clock:
    """A settable ``now`` for EventLogs."""

    def __init__(self, when: datetime = WHEN) -> None:
        self.when = when

    def __call__(self) -> datetime:
        return self.when


def make_folders(tmp_path: Path, *, vision: bool = True, data: bool = True) -> Folders:
    return Folders(exe=tmp_path / "exe", settings=tmp_path / "settings",
                   vision=tmp_path / "vision" if vision else None, data=tmp_path / "data" if data else None)


def make_logs(tmp_path: Path, clock: Clock | None = None, **options: object) -> EventLogs:
    return EventLogs(make_folders(tmp_path), LoggingOptions(**options), now=clock or Clock())  # type: ignore[arg-type]


def lines_of(path: Path) -> list[str]:
    data = path.read_bytes()
    assert data.endswith(b"\r\n")
    return data.decode("cp1252").split("\r\n")[:-1]


# ---------------------------------------------------------------------------------------------- line format (09 §7.4)
def test_golden_error_line_z() -> None:
    v = Z_LINE
    assert v.line == "29/10/2026 - 16:31:05.123 - ERR (Z STAGE)-(9405) : Z move out of range (Move Absolute)"
    assert EventLogs.format_error_line(v.when, v.err, v.server_type, v.extra, v.command, v.camera_type) == v.line


def test_golden_error_line_camera() -> None:
    v = CAM_LINE.filled(-1074360316, "IMAQdx Open Camera.vi<ERR>\nInvalid camera session")
    assert v.line == ("29/10/2026 - 16:31:07.456 - ERR (LOW MAG)-Cam Setup(-1074360316) : "
                      "IMAQdx Open Camera.vi<ERR>\nInvalid camera sessionCam Setup (Setup)")
    assert EventLogs.format_error_line(v.when, v.err, v.server_type, v.extra, v.command, v.camera_type) == v.line


def test_golden_info_line() -> None:
    v = INFO_LINE
    assert v.line == "29/10/2026 - 16:32:00.789 - INFO -(0) :  (Move Absolute: Z Reference resetted)"
    assert EventLogs.format_info_line(v.when, v.err, v.server_type, v.extra, v.camera_type) == v.line


def test_error_tags_and_command_name() -> None:
    err = ErrorCluster(True, 1, "s")

    def line(server_type: str, command: str | None = "Cam Start", camera: str = HIGH_MAG) -> str:
        return EventLogs.format_error_line(WHEN, err, server_type, "x", command, camera).split(" - ", 2)[2]

    assert line("Camera") == "ERR (HI MAG)-Cam Start(1) : s (x)"
    assert line("Camera", camera=LOW_MAG) == "ERR (LOW MAG)-Cam Start(1) : s (x)"
    assert line("Camera", command=None) == "ERR (HI MAG)-(1) : s (x)"
    assert line("Z Stage") == "ERR (Z STAGE)-(1) : s (x)"           # no command name for other servers
    assert line("Monochromator") == "ERR (MONO WL)-(1) : s (x)"
    assert line("XY Stages") == "ERR (XY STAGE)-(1) : s (x)"
    assert line("Other") == "ERR -(1) : s (x)"


def test_info_labels_fixed_and_reproduced() -> None:
    def tag(camera: str, fix: bool, server: str = "Camera") -> str:
        return EventLogs.format_info_line(WHEN, NO_ERROR, server, "x", camera, fix_info_labels=fix).split(" - ")[2]

    assert tag(HIGH_MAG, True).startswith("INFO (HI)-(0)")
    assert tag(LOW_MAG, True).startswith("INFO (LO)-(0)")
    assert tag(HIGH_MAG, False).startswith("INFO (LO)-(0)")          # LabVIEW D11: swapped
    assert tag(LOW_MAG, False).startswith("INFO (HI)-(0)")
    assert tag(LOW_MAG, False, "Z Stage").startswith("INFO -(0)")


def test_decimal_separator_and_milliseconds() -> None:
    err = ErrorCluster(True, 1, "s")
    assert EventLogs.format_error_line(WHEN, err, "Z Stage", "x", None, HIGH_MAG, ",").startswith(
        "29/10/2026 - 16:31:05,123 - ")
    early = datetime(2026, 1, 2, 3, 4, 5, 999)
    assert EventLogs.format_info_line(early, err, "Z Stage", "x", HIGH_MAG).startswith("02/01/2026 - 03:04:05.000 - ")


# ---------------------------------------------------------------------------------------------- EventSink behaviour
def test_is_an_event_sink(tmp_path: Path) -> None:
    sink: EventSink = make_logs(tmp_path)
    assert sink.error(NO_ERROR, "Camera", "x") is NO_ERROR


def test_only_status_true_logged(tmp_path: Path) -> None:
    logs = make_logs(tmp_path)
    warning = ErrorCluster(False, 5, "warning only")
    assert logs.error(warning, "Z Stage", "x") is warning
    assert logs.error(NO_ERROR, "Camera", "x") is NO_ERROR
    assert not (tmp_path / "vision" / "logs").exists() or not any((tmp_path / "vision" / "logs").iterdir())
    assert logs.recent() == []
    assert logs.info(warning, "Z Stage", "Move Absolute: Z Reference resetted") is warning   # infos always written
    assert len(logs.recent()) == 1


def test_files_and_folders_09_7_3(tmp_path: Path) -> None:
    logs = make_logs(tmp_path)
    err = ErrorCluster(True, 9405, "Z move out of range")
    logs.error(err, "Z Stage", "Move Absolute")
    logs.error(ErrorCluster(True, 1, "c"), "Camera", "Setup", command="Cam Setup", camera_type=LOW_MAG)
    logs.info(NO_ERROR, "Z Stage", "Move Absolute: Z Reference resetted")
    vision, data = tmp_path / "vision" / "logs", tmp_path / "data" / "logs"
    assert sorted(p.name for p in vision.iterdir()) == ["261029_camera_events.log", "261029_z_events.log"]
    assert sorted(p.name for p in data.iterdir()) == ["261029_z_events.log"]
    assert lines_of(vision / "261029_z_events.log") == [
        "29/10/2026 - 16:31:05.123 - ERR (Z STAGE)-(9405) : Z move out of range (Move Absolute)"]
    assert lines_of(data / "261029_z_events.log") == [
        "29/10/2026 - 16:31:05.123 - INFO -(0) :  (Move Absolute: Z Reference resetted)"]
    assert lines_of(vision / "261029_camera_events.log") == [
        "29/10/2026 - 16:31:05.123 - ERR (LOW MAG)-Cam Setup(1) : c (Setup)"]


def test_path_helpers_and_stems(tmp_path: Path) -> None:
    logs = make_logs(tmp_path)
    assert dict(STEM) == {"Camera": "camera", "Monochromator": "wavelength", "XY Stages": "xy", "Z Stage": "z"}
    for server, stem in [*STEM.items(), ("Unknown", "camera")]:
        assert logs.error_log_path(server, WHEN) == tmp_path / "vision" / "logs" / f"261029_{stem}_events.log"
        assert logs.info_log_path(server, WHEN) == tmp_path / "data" / "logs" / f"261029_{stem}_events.log"
    nothing = EventLogs(make_folders(tmp_path, vision=False, data=False), LoggingOptions())
    assert nothing.error_log_path("Z Stage", WHEN) is None and nothing.info_log_path("Z Stage", WHEN) is None


def test_daily_rotation(tmp_path: Path) -> None:
    clock = Clock(datetime(2026, 12, 31, 23, 59, 59, 999000))
    logs = make_logs(tmp_path, clock)
    logs.error(ErrorCluster(True, 1, "a"), "Z Stage", "x")
    clock.when += timedelta(milliseconds=1)
    logs.error(ErrorCluster(True, 2, "b"), "Z Stage", "x")
    names = sorted(p.name for p in (tmp_path / "vision" / "logs").iterdir())
    assert names == ["261231_z_events.log", "270101_z_events.log"]


def test_get_heartbeat_printed_by_default(tmp_path: Path) -> None:
    logs = make_logs(tmp_path)
    logs.error(ErrorCluster(True, 7, "src"), "Camera", "Extra")
    assert logs.recent() == ["29/10/2026 - 16:31:05.123 - ERR (HI MAG)-Get Heartbeat(7) : src (Extra)"]


def test_crlf_cp1252_and_unencodable_characters(tmp_path: Path) -> None:
    logs = make_logs(tmp_path)
    logs.error(ErrorCluster(True, 1, "Caméra € Ω ✓"), "Z Stage", "x")
    data = (tmp_path / "vision" / "logs" / "261029_z_events.log").read_bytes()
    assert data.endswith(b" : Cam\xe9ra \x80 ? ? (x)\r\n")
    assert data.count(b"\r\n") == 1 and b"\n" not in data.replace(b"\r\n", b"")
    assert logs.recent()[0].endswith("Caméra € Ω ✓ (x)")             # the ring buffer keeps the text


def test_append_vs_offset_zero_09_7_3(tmp_path: Path) -> None:
    ex = sv.OFFSET_ZERO_EXAMPLE
    for append in (True, False):
        root = tmp_path / str(append)
        clock = Clock(ex.first.when)
        logs = make_logs(root, clock, append_errors=append)
        logs.error(ex.first.err, ex.first.server_type, ex.first.extra, command=ex.first.command,
                   camera_type=ex.first.camera_type)
        clock.when = ex.second.when
        logs.error(ex.second.err, ex.second.server_type, ex.second.extra, command=ex.second.command,
                   camera_type=ex.second.camera_type)
        data = (root / "vision" / "logs" / "261029_z_events.log").read_bytes()
        first, second = (ex.first.line + "\r\n").encode(), (ex.second.line + "\r\n").encode()
        if append:                                               # D10 fixed (default)
            assert data == first + second
        else:                                                    # LabVIEW: Set File Position(start, 0)
            assert data == second + first[len(second):]
            assert data.endswith(ex.expected_tail)
            assert data[len(second):] == ex.expected_tail


def test_info_always_appends(tmp_path: Path) -> None:
    logs = make_logs(tmp_path, append_errors=False)
    for _ in range(2):
        logs.info(NO_ERROR, "Z Stage", "Move Relative: Z Reference resetted")
    assert len(lines_of(tmp_path / "data" / "logs" / "261029_z_events.log")) == 2


def test_options_decimal_separator_and_labels(tmp_path: Path) -> None:
    logs = make_logs(tmp_path, decimal_separator=",", fix_info_labels=False)
    logs.info(NO_ERROR, "Camera", "x", camera_type=LOW_MAG)
    assert lines_of(tmp_path / "data" / "logs" / "261029_camera_events.log") == [
        "29/10/2026 - 16:31:05,123 - INFO (HI)-(0) :  (x)"]


def test_recent_ring_buffer(tmp_path: Path) -> None:
    logs = EventLogs(make_folders(tmp_path), LoggingOptions(), now=Clock(), recent_lines=3)
    for code in range(1, 6):
        logs.error(ErrorCluster(True, code, "s"), "Z Stage", "x")
    assert [line.split(" - ")[2] for line in logs.recent()] == [f"ERR (Z STAGE)-({c}) : s (x)" for c in (3, 4, 5)]
    assert len(lines_of(tmp_path / "vision" / "logs" / "261029_z_events.log")) == 5


# ---------------------------------------------------------------------------------------------- folders and failures
def test_ensure_dirs_creates_logs(tmp_path: Path) -> None:
    logs = make_logs(tmp_path)
    assert logs.ensure_dirs() == []
    assert (tmp_path / "vision" / "logs").is_dir() and (tmp_path / "data" / "logs").is_dir()
    assert logs.ensure_dirs() == []                              # idempotent


def test_ensure_dirs_disabled_and_unavailable(tmp_path: Path) -> None:
    logs = make_logs(tmp_path, create_log_dirs=False)
    warnings = logs.ensure_dirs()
    assert len(warnings) == 2 and all("create_log_dirs" in w for w in warnings)
    assert not (tmp_path / "vision" / "logs").exists()
    # LabVIEW behaviour: no folder → the line is lost silently (diagnostic warning only)
    assert logs.error(ErrorCluster(True, 1, "s"), "Z Stage", "x").code == 1
    assert not (tmp_path / "vision" / "logs").exists()
    none = EventLogs(make_folders(tmp_path, vision=False), LoggingOptions(), now=Clock())
    warnings = none.ensure_dirs()
    assert len(warnings) == 1 and "vision" in warnings[0]


def test_lines_written_without_ensure_dirs(tmp_path: Path) -> None:
    logs = make_logs(tmp_path)
    logs.error(ErrorCluster(True, 1, "s"), "Z Stage", "x")
    assert (tmp_path / "vision" / "logs" / "261029_z_events.log").exists()


def test_unwritable_folder_swallowed_one_warning(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    (tmp_path / "vision").mkdir()
    (tmp_path / "vision" / "logs").write_bytes(b"not a folder")
    logs = make_logs(tmp_path)
    assert len(logs.ensure_dirs()) == 1
    err = ErrorCluster(True, 1, "s")
    with caplog.at_level(logging.WARNING, logger="aloxcam.eventlog"):
        for _ in range(3):
            assert logs.error(err, "Z Stage", "x") is err
    failures = [r for r in caplog.records if r.name == "aloxcam.eventlog" and r.levelno == logging.WARNING]
    assert len(failures) == 1 and "261029_z_events.log" in failures[0].getMessage()
    assert len(logs.recent()) == 3
    logs.info(NO_ERROR, "Z Stage", "x")                          # the data log is unaffected
    assert (tmp_path / "data" / "logs" / "261029_z_events.log").exists()


def test_unavailable_folder_one_warning(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    logs = EventLogs(make_folders(tmp_path, vision=False), LoggingOptions(), now=Clock())
    with caplog.at_level(logging.WARNING, logger="aloxcam.eventlog"):
        for _ in range(2):
            logs.error(ErrorCluster(True, 1, "s"), "Z Stage", "x")
    assert len([r for r in caplog.records if r.name == "aloxcam.eventlog"]) == 1


def test_concurrent_writers_keep_whole_lines(tmp_path: Path) -> None:
    logs = EventLogs(make_folders(tmp_path), LoggingOptions(), now=datetime.now, recent_lines=10_000)
    threads_n, per_thread = 8, 50
    barrier = threading.Barrier(threads_n)

    def writer(t: int) -> None:
        barrier.wait()
        for i in range(per_thread):
            logs.error(ErrorCluster(True, t, f"thread {t} line {i} " + "x" * 200), "Z Stage", f"{t}/{i}")

    workers = [threading.Thread(target=writer, args=(t,)) for t in range(threads_n)]
    for w in workers:
        w.start()
    for w in workers:
        w.join()
    pattern = re.compile(r"^\d\d/\d\d/\d{4} - \d\d:\d\d:\d\d\.\d{3} - ERR \(Z STAGE\)-\((\d)\) : thread \1 line (\d+) "
                         r"x{200} \(\1/\2\)$")
    lines = [line for p in (tmp_path / "vision" / "logs").iterdir() for line in lines_of(p)]
    assert len(lines) == threads_n * per_thread
    assert all(pattern.match(line) for line in lines)
    assert len(logs.recent()) == threads_n * per_thread


# ---------------------------------------------------------------------------------------------- diagnostic log
@pytest.fixture
def restore_aloxcam_logger():
    logger = logging.getLogger("aloxcam")
    level, handlers = logger.level, list(logger.handlers)
    yield logger
    for h in list(logger.handlers):
        if h not in handlers:
            logger.removeHandler(h)
            h.close()
    logger.setLevel(level)


def test_setup_diagnostic_logging(tmp_path: Path, restore_aloxcam_logger: logging.Logger) -> None:
    folders = make_folders(tmp_path)
    handlers = setup_diagnostic_logging(folders, LoggingOptions(diag_level="WARNING"), stderr=False)
    assert len(handlers) == 1 and isinstance(handlers[0], DailyFileHandler)
    assert restore_aloxcam_logger.level == logging.WARNING
    logging.getLogger("aloxcam.test").warning("diagnostic é")
    logging.getLogger("aloxcam.test").info("not written")
    path = handlers[0].current_path
    assert path is not None and path.parent == tmp_path / "vision" / "logs"
    assert re.fullmatch(r"\d{6}_camera_server_diag\.log", path.name)
    text = path.read_text(encoding="utf-8")
    assert "WARNING aloxcam.test" in text and "diagnostic é" in text and "not written" not in text
    # idempotent: a second call replaces the handlers; --log-level overrides diag_level
    again = setup_diagnostic_logging(folders, LoggingOptions(), level="debug", stderr=True)
    assert len(again) == 2 and restore_aloxcam_logger.level == logging.DEBUG
    assert handlers[0] not in restore_aloxcam_logger.handlers
    assert all(h in restore_aloxcam_logger.handlers for h in again)


def test_setup_diagnostic_logging_without_vision(restore_aloxcam_logger: logging.Logger) -> None:
    assert setup_diagnostic_logging(None, LoggingOptions(), stderr=False) == []
    handlers = setup_diagnostic_logging(None, LoggingOptions(), stderr=True)
    assert len(handlers) == 1 and not isinstance(handlers[0], DailyFileHandler)


def test_daily_file_handler_switches_on_date(tmp_path: Path) -> None:
    handler = DailyFileHandler(tmp_path / "logs", "diag")
    handler.setFormatter(logging.Formatter("%(message)s"))
    try:
        for when, msg in [(datetime(2026, 10, 29, 23, 59, 59), "a"), (datetime(2026, 10, 29, 23, 59, 59, 900000), "b"),
                          (datetime(2026, 10, 30, 0, 0, 1), "c")]:
            record = logging.LogRecord("aloxcam.x", logging.INFO, __file__, 1, msg, None, None)
            record.created = when.timestamp()
            handler.handle(record)
        assert handler.current_path == tmp_path / "logs" / "261030_diag.log"
        assert (tmp_path / "logs" / "261029_diag.log").read_text(encoding="utf-8").splitlines() == ["a", "b"]
        assert (tmp_path / "logs" / "261030_diag.log").read_text(encoding="utf-8").splitlines() == ["c"]
    finally:
        handler.close()
