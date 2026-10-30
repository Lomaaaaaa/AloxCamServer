#!/usr/bin/env python3
r"""Check the PI C-663 Z stage controller over RS-232: read-only information, then optional guided motion tests.

Spec: docs/phase2-design.md §4.1. Background: 08 §3 (frames, errors), §8 (waits), §10 (macros DH / HOME), §13.6
(hardware checklist); docs/open-questions.md Q9, Q10, Q11, Q35; docs/answers/owner-answers.md §4-§5;
docs/phase2-notes.md ("Lab tool z_stage_check").

Run it on the PC that controls the stage, from the repository folder, **with the LabVIEW camera server stopped** (it
keeps the COM port open). Information first; the motion tests only when the stage area is safe::

    python tools\z_stage_check.py --alox-ini D:\alox\settings\alox.ini
    python tools\z_stage_check.py --alox-ini D:\alox\settings\alox.ini --motion

``--port COM4`` (and ``--baud``) gives the port directly. ``--fake`` runs everything against an emulated C-663 inside
this program (a rehearsal: no port is opened). The tool uses the server's own classes (``PiSerialStage`` on
``GcsSerialLink``), so it sends exactly the server's frames, and it writes a report
(``z_stage_report_<date>-<time>.json`` with every answer and every raw frame, plus a ``.txt`` summary) next to this
tool, or in ``--out``. That report is what to send back.

Information (the default): read-only
------------------------------------
These frames, and only these, are sent, in this order:

1. Opening the port sends the purge of LabVIEW's PI driver (``PI Open Interface``), exactly as LabVIEW does when it
   opens the port: ``XXX``, a 10 ms pause, ``*idn?``, ``err?``. ``XXX`` is not a command: the controller only latches
   error 2 ("Unknown command"), which ``err?`` reads and clears. The code read is reported (2 is normal).
2. The queries, axis 1: ``SAI?``, ``*IDN?``, ``ERR?``, ``CST? 1``, ``SVO? 1``, ``RON? 1``, ``TRS? 1``, ``LIM? 1``,
   ``TMN? 1``, ``TMX? 1``, ``FRF? 1``, ``POS? 1``, ``VEL? 1``, ``JON? 1``, ``RMC?``, ``MAC? DH``, ``MAC? HOME``,
   ``#5`` (the single byte 0x05: is the stage moving?), ``#7`` (the single byte 0x07: is the controller ready?),
   ``ERR?``.
3. Only when a query gets no usable answer: one more ``ERR?`` right after it, to read the controller's reason.

``ERR?`` reads **and clears** the controller's pending error; every value read is in the report. Nothing else is sent:
no command that moves the stage, changes a setting or stops a macro. Closing the port sends nothing.

Motion tests (``--motion``): the stage moves
---------------------------------------------
The information above is read first. Then the plan is printed (start position, allowed window, test speed, the DH
warning) and nothing moves until the operator types ``yes``. Each test is explained and needs its own ``yes``
(anything else skips it, ``quit`` skips the rest). The tests answer questionnaire Q35 and 08 §13.6:

1. POS? during a move (Q35 test 2): ``MOV 1 <target>``; ``#5`` and ``POS? 1`` every 50 ms until it stands still.
2. STP 1 during a move (Q35 test 1): ``MOV 1 <target>``; ``STP 1`` after 40 % of the move, ``ERR?`` at once (10
   expected); ``#5`` / ``POS? 1`` until it stands still (``#24`` if it still moves 1 s after ``STP 1``); ``POS? 1``,
   ``ERR?``, ``RMC?``.
3. MVR while a move runs (Q35 test 3): ``MOV 1 <target>``; ``POS? 1`` and ``MVR 1 <d>`` after 40 % of the move; wait;
   ``POS? 1``, ``ERR?``. Ends at "target + d" or at "position at that moment + d"?
4. MOV with the joystick on (Q35 test 4): ``JAX 1 1 1``, ``JON 1 1``, ``ERR?``, ``JON? 1``; ``MOV 1 <target>`` (0.2 mm);
   wait; ``POS? 1``, ``ERR?``; then ``JAX 1 1 1``, ``JON 1 0``, ``ERR?``, ``JON? 1`` (as LabVIEW's joystick commands).
5. VEL 0 (Q35 test 5): ``VEL? 1``, ``VEL 1 0.0000``, ``ERR?``, ``VEL? 1``, then ``VEL 1 <test speed>``, ``ERR?``,
   ``VEL? 1``. Nothing moves.
6. Only with ``--setup-twice`` (Q35 test 6, Q11): the server's ``Z Setup`` (``ZController.setup``, ``[stage]``
   defaults: reference at every set-up, FNL) twice, timed. **FNL drives the stage to its negative limit switch, out of
   the window**: an extra warning and a second ``yes`` are required. This is the only time the tool references.

Every test starts with ``ERR?`` (clears an older error), ``POS? 1`` and ``#5``, before its ``yes``. Once the first
test (or the move back) is confirmed: ``VEL 1 <test speed>``, ``ERR?``, ``VEL? 1``. At the end: optionally the move
back to the start position (``MOV 1 <start>``, after a ``yes``), the original speed if it was changed (``VEL 1
<original>``, ``ERR?``, ``VEL? 1``), ``RMC?`` and, when DH ran at the start and is stopped now, ``MAC START DH``,
``RMC?``, ``ERR?``; then ``POS? 1``, ``#5``, ``ERR?``. Declining everything therefore changes nothing on the
controller.

Safety rules (all enforced by the code below):

- ``--yes`` (answers every question) is refused unless ``--fake`` is given.
- Allowed window = intersection of the controller range ``[TMN? 1, TMX? 1]``, ``[zmin_DBL, zmax_DBL]`` of
  ``--alox-ini`` (when given) and the start position +/- ``--window`` (default 1 mm). A test whose moves do not fit is
  refused. While the stage moves, ``#5`` and ``POS? 1`` are read every 50 ms: a position outside the window (+0.05 mm)
  sends ``#24`` at once.
- Test speed ``--speed`` (default 1 mm/s; at most 5 mm/s and never above the controller's current speed, unless
  ``--fake``); the original speed is restored at the end.
- Every move waits for the end of the motion with a timeout. Ctrl+C, any error or a timeout sends ``#24`` (stop all
  axes) at once - and once more on a re-opened port when the link itself failed; ``JON 1 0`` right after it when the
  tool had switched the joystick on - then restores what it can (joystick off, speed). The tool never leaves the
  motion phase without attempting that stop, and never moves the stage back after it.
- ``DH``: whether it ran at the start (``RMC?``) is remembered; ``STP`` / ``#24`` stop it; it is restarted at the end
  (after a stop on error or Ctrl+C only when the operator types ``yes``). While DH runs, the controller starts ``HOME``
  (a move to Z = 0 mm) when digital input 1 is high: the operator is warned, the tool never starts ``HOME``, and the
  window check stops such a move.
- Never any reference move (FNL / FRF) except with ``--setup-twice``.

Exit code: 0 the tool ran to the end (findings are in the report), 1 it could not connect, lost the link, or the
motion tests were stopped (#24), 2 usage error.
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import platform
import re
import subprocess
import sys
import textwrap
import time
import traceback
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field, replace
from datetime import datetime
from pathlib import Path
from typing import Any, TextIO

try:                                    # a source checkout without ``pip install -e .``: use <repo>/src
    import aloxcam  # noqa: F401
except ImportError:                     # pragma: no cover - depends on how the tool is started
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from aloxcam import PHASE
from aloxcam import __version__ as ALOXCAM_VERSION
from aloxcam.config.alox_ini import load_alox_ini
from aloxcam.errors import Code, StageError
from aloxcam.settings import CameraOptions, SimulationOptions, StageOptions, ZStageSettings
from aloxcam.zstage.backend import format_gcs_float
from aloxcam.zstage.controller import KNOWN_MACROS, PURGE_OWN_ERROR, ZController, serial_from_idn
from aloxcam.zstage.fake_c663 import FAKE_IDENTITY, FakeC663Server
from aloxcam.zstage.gcs_errors import gcs_error_text
from aloxcam.zstage.gcs_serial import (
    BUSY_BYTE,
    DROPPED,
    FRAME_LOG_SIZE,
    PYSERIAL_MISSING,
    READY_BYTE,
    RX,
    TX,
    GcsSerialLink,
    pyserial_version,
)
from aloxcam.zstage.pi_serial import PURGE_FRAMES, LinkFactory, PiSerialStage
from aloxcam.zstage.sim import DH_MACRO_TEXT, HOME_MACRO_TEXT

__all__ = [
    "INFO_FRAMES",
    "INFO_STEPS",
    "TEST_SPECS",
    "Environment",
    "FrameLog",
    "Window",
    "advice",
    "compute_window",
    "main",
    "start_fake_server",
]

TOOL_NAME = "z_stage_check"
TOOL_VERSION = "1.0"
TOOL_DIR = Path(__file__).resolve().parent
REPORT_PREFIX = "z_stage_report_"

AXIS = "1"
JOYSTICK_ID = 1
JOYSTICK_AXIS = 1
DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT_MS = 5000               # the VISA timeout LabVIEW wires (08 §3.1)
DEFAULT_WINDOW_MM = 1.0
DEFAULT_SPEED = 1.0                     # mm/s
MAX_SPEED_REAL = 5.0                    # mm/s, real stage
MIN_SPEED = 0.05                        # mm/s: slower test moves would take minutes
MAX_WINDOW_REAL = 10.0                  # mm, real stage

POLL_S = 0.05                           # #5 + POS? 1 while the stage moves
MIN_MOVE_S = 0.6                        # a test move lasts at least this long, so that POS? can be read during it
MIN_MOVE_MM = 0.2
ACT_FRACTION = 0.4                      # STP / MVR are sent after this fraction of the move
AT_TARGET_MM = 0.01
WINDOW_MARGIN_MM = 0.05                 # the window check allows this much beyond the window (servo overshoot)
SETTLE_S = 0.3                          # a move that does not start: how long to wait before deciding so
STOP_GRACE_S = 1.0                      # after STP 1 the stage must stand still within this time, else #24
STANDSTILL_WAIT_S = 3.0                 # after #24: how long to watch for the stage to stand still
JOYSTICK_MOVE_MM = 0.2
MIN_JOYSTICK_MOVE_MM = 0.05
MOVE_TIMEOUT_MIN_S = 3.0                # move timeout = MIN + FACTOR * expected duration
MOVE_TIMEOUT_FACTOR = 3.0
SETUP_READY_TIMEOUT_S = 180.0           # --setup-twice: #7 wait for the end of referencing
MAX_TIMEOUTS_IN_A_ROW = 3               # info: stop asking a controller that no longer answers
FAKE_START_MM = 50.0                    # --fake: the emulated stage starts referenced at mid-travel (FRF)
DH_OUTPUT_MM = 5.05                     # the DH macro toggles digital output 1 at this Z (owner-answers §5)

EXIT_OK, EXIT_PROBLEM, EXIT_USAGE = 0, 1, 2
LINK_ERRORS = frozenset({Code.PI_SEND_ERROR, Code.PI_RECEIVE_ERROR, Code.PI_NOT_CONNECTED})
TEST_QUESTION = ("Type yes and press Enter to run this test (anything else skips it; quit skips all the remaining "
                 "tests):")
_SINGLE_BYTES = {0x05: "#5", 0x07: "#7", 0x18: "#24"}


class UsageError(Exception):
    """A command-line problem: exit code 2, nothing opened."""


class Refusal(Exception):
    """A test (or the motion phase) cannot run safely; nothing was sent for it."""


class SafetyStop(Exception):
    """The motion tests must stop. ``stop_sent``: #24 was already sent for this reason."""

    def __init__(self, reason: str, *, stop_sent: bool = False) -> None:
        super().__init__(reason)
        self.reason = reason
        self.stop_sent = stop_sent


# ---------------------------------------------------------------------------------------------- outside world
@dataclass
class Environment:
    """What the tool needs from the outside world; the tests replace parts of it (virtual time, a prepared fake)."""

    read_line: Callable[[], str] = input
    output: TextIO | None = None                        # None: sys.stdout
    sleep: Callable[[float], None] = time.sleep
    monotonic: Callable[[], float] = time.monotonic
    link_factory: LinkFactory | None = None             # None: GcsSerialLink
    fake_server: FakeC663Server | None = None           # --fake: use this one (not closed by the tool)


# ---------------------------------------------------------------------------------------------- frames
def frame_text(data: bytes) -> str:
    """Printable form of raw bytes: ASCII as is, ``\\n``, ``\\r``, ``\\\\``, other bytes as ``\\xHH``."""
    out: list[str] = []
    for byte in data:
        if byte == 0x0A:
            out.append("\\n")
        elif byte == 0x0D:
            out.append("\\r")
        elif byte == 0x5C:
            out.append("\\\\")
        elif 0x20 <= byte < 0x7F:
            out.append(chr(byte))
        else:
            out.append(f"\\x{byte:02x}")
    return "".join(out)


def command_text(data: bytes) -> str:
    """A sent frame as a command: ``"POS? 1"`` (LF removed), ``"#5"`` / ``"#7"`` / ``"#24"`` for the single bytes."""
    if len(data) == 1 and data[0] in _SINGLE_BYTES:
        return _SINGLE_BYTES[data[0]]
    return frame_text(data[:-1] if data.endswith(b"\n") else data)


@dataclass(frozen=True)
class Frame:
    t: float                            # seconds since the tool started
    direction: str                      # "tx" | "rx" | "drop"
    data: bytes


class FrameLog:
    """Every frame of every link opened by the tool, time-stamped (``GcsSerialLink.frames`` keeps only 512)."""

    def __init__(self, monotonic: Callable[[], float]) -> None:
        self._monotonic = monotonic
        self._t0 = monotonic()
        self.frames: list[Frame] = []

    def elapsed(self) -> float:
        return self._monotonic() - self._t0

    def add(self, direction: str, data: bytes) -> None:
        self.frames.append(Frame(round(self.elapsed(), 4), direction, bytes(data)))

    def mark(self) -> int:
        return len(self.frames)

    def since(self, mark: int) -> list[Frame]:
        return self.frames[mark:]

    def sent_since(self, mark: int) -> list[str]:
        return [command_text(f.data) for f in self.since(mark) if f.direction == TX]

    def as_records(self) -> list[dict[str, Any]]:
        records = []
        for index, f in enumerate(self.frames):
            record: dict[str, Any] = {"i": index, "t_s": f.t, "dir": f.direction, "text": frame_text(f.data),
                                      "hex": f.data.hex(" ")}
            if f.direction == TX:
                record["cmd"] = command_text(f.data)
            records.append(record)
        return records


class _RecordingFrames(collections.deque):  # type: ignore[type-arg]
    """Replaces ``GcsSerialLink.frames``: keeps the link's own bounded deque and copies every frame into the
    :class:`FrameLog`."""

    def __init__(self, log: FrameLog) -> None:
        super().__init__((), FRAME_LOG_SIZE)
        self._log = log

    def append(self, item: tuple[str, bytes]) -> None:
        super().append(item)
        direction, data = item
        self._log.add(direction, data)


def recording_factory(base: LinkFactory, log: FrameLog) -> LinkFactory:
    """A link factory whose links record every frame into ``log``."""

    def open_link(port: str, baudrate: int, timeout_s: float) -> GcsSerialLink:
        link = base(port, baudrate, timeout_s)
        link.frames = _RecordingFrames(log)
        return link

    return open_link


# ---------------------------------------------------------------------------------------------- console
class Console:
    """Plain console output and the operator's answers (recorded for the report)."""

    def __init__(self, output: TextIO, read_line: Callable[[], str], *, auto_yes: bool) -> None:
        self._out = output
        self._read_line = read_line
        self.auto_yes = auto_yes
        self.answers: list[dict[str, str]] = []

    def line(self, text: str = "") -> None:
        try:
            self._out.write(text + "\n")
        except UnicodeEncodeError:                     # a console that cannot show a character of an answer
            self._out.write(text.encode("ascii", "backslashreplace").decode("ascii") + "\n")
        self._out.flush()

    def section(self, title: str) -> None:
        self.line()
        self.line(f"=== {title} ===")

    def ask(self, question: str) -> str:
        """``"yes"`` only when the operator typed yes (any case); ``"quit"`` for quit / q / stop or no input at all;
        ``"skip"`` for anything else, including a bare Enter."""
        prompt = question.rstrip() + " "
        if self.auto_yes:
            self.line(prompt + "yes   (--yes)")
            raw = "yes"
        else:
            self._out.write(prompt)
            self._out.flush()
            try:
                raw = self._read_line()
            except EOFError:                            # no operator (input closed)
                self.line()
                raw = "<end of input>"
        text = raw.strip().lower()
        meaning = "yes" if text == "yes" else "quit" if text in ("quit", "q", "stop", "<end of input>") else "skip"
        self.answers.append({"question": question, "answer": raw, "meaning": meaning})
        return meaning


# ---------------------------------------------------------------------------------------------- guidance
_BUSY_WORDS = ("access is denied", "permissionerror", "permission denied", "exclusively lock", "resource busy",
               "being used by another", "errno 16", "errno 13")
_MISSING_WORDS = ("filenotfounderror", "cannot find the file", "no such file", "does not exist", "errno 2]")


def describe(exc: BaseException) -> str:
    if isinstance(exc, StageError):
        return f"error {exc.code}: {exc.source}"
    return f"{type(exc).__name__}: {exc}"


def advice(exc: StageError, port: str, timeout_ms: int = DEFAULT_TIMEOUT_MS) -> list[str]:
    """What the operator should check for a link error (simple words)."""
    low = exc.source.lower()
    if exc.code == Code.PI_OPEN_PORT:
        if PYSERIAL_MISSING.lower() in low or "pyserial" in low:
            return ["pyserial (the serial port library) is not installed for this Python.",
                    "Install it with this exact command, then run the tool again:",
                    f'    "{sys.executable}" -m pip install pyserial']
        if any(word in low for word in _BUSY_WORDS):
            return [f"{port} is busy: another program has it open.",
                    "Close the LabVIEW camera server (it keeps the COM port open) and any PI MikroMove or terminal",
                    "window, then try again."]
        if any(word in low for word in _MISSING_WORDS):
            return [f"{port} does not exist on this PC.",
                    "Check the port number: alox.ini [hardware.z] comport_I32 = <n> means COM<n>. Windows Device",
                    "Manager > Ports (COM & LPT) lists the Moxa ports."]
        return [f"{port} could not be opened. Check the port number, and close any program that uses it (the",
                "LabVIEW camera server keeps the COM port open)."]
    if exc.code == Code.PI_TIMEOUT:
        return [f"The controller did not answer within {timeout_ms} ms. Check:",
                "- the port number (COM<n>, alox.ini [hardware.z] comport_I32);",
                "- the Moxa serial server: power, network cable, and that its driver maps this COM port;",
                "- the C-663 controller is switched on and its RS-232 cable is plugged in;",
                "- the baud rate (115200; alox.ini baudrate_I32 = 3)."]
    if exc.code in (Code.PI_SEND_ERROR, Code.PI_RECEIVE_ERROR):
        return ["The serial link failed. Check the Moxa serial server, its network cable and the RS-232 cable,",
                "then try again."]
    if exc.code == Code.PI_DRIVER:
        return ["The answer was not in the expected format: the report has the raw bytes."]
    return []


# ---------------------------------------------------------------------------------------------- info steps
def _show_err(code: int) -> str:
    return f"{code}  {gcs_error_text(code) or 'unknown code'}"


def _flag(yes: str, no: str) -> Callable[[bool], str]:
    return lambda value: f"{int(bool(value))}  {yes if value else no}"


def _mm(value: float) -> str:
    return f"{value:.4f} mm"


def _show_rmc(value: str) -> str:
    return f"{value!r}  {'macro running' if value.strip() else 'no macro running'}"


def _normal_macro(text: str) -> list[str]:
    return [" ".join(line.split()).upper() for line in text.splitlines() if line.strip()]


def _same_macro(text: str, documented: str) -> bool:
    return _normal_macro(text) == _normal_macro(documented)


def _show_macro(documented: str) -> Callable[[str], str]:
    def show(text: str) -> str:
        lines = len([line for line in text.splitlines() if line.strip()])
        same = "same as" if _same_macro(text, documented) else "DIFFERENT from"
        return f"{lines} lines, {same} the text in docs/answers/owner-answers.md section 5"
    return show


@dataclass(frozen=True)
class InfoStep:
    """One read-only query of the information mode."""

    frame: str                                  # the frame as sent (documented in the module docstring)
    key: str                                    # report key
    read: Callable[[PiSerialStage], Any]
    show: Callable[[Any], str]


INFO_STEPS: tuple[InfoStep, ...] = (
    InfoStep("SAI?", "sai", lambda s: s.qsai(), lambda v: " ".join(v) or "(empty)"),
    InfoStep("*IDN?", "idn", lambda s: s.qidn(), str),
    InfoStep("ERR?", "err_first", lambda s: s.qerr(), _show_err),
    InfoStep("CST? 1", "cst", lambda s: s.qcst(AXIS), str),
    InfoStep("SVO? 1", "svo", lambda s: s.qsvo(AXIS), _flag("servo on", "servo OFF")),
    InfoStep("RON? 1", "ron", lambda s: s.qron(AXIS), _flag("reference mode on", "reference mode off")),
    InfoStep("TRS? 1", "trs", lambda s: s.qtrs(AXIS), _flag("the stage has a reference switch", "no reference switch")),
    InfoStep("LIM? 1", "lim", lambda s: s.qlim(AXIS), _flag("the stage has limit switches", "no limit switches")),
    InfoStep("TMN? 1", "tmn", lambda s: s.qtmn(AXIS), _mm),
    InfoStep("TMX? 1", "tmx", lambda s: s.qtmx(AXIS), _mm),
    InfoStep("FRF? 1", "frf", lambda s: s.qfrf(AXIS), _flag("referenced", "NOT referenced")),
    InfoStep("POS? 1", "pos", lambda s: s.qpos(AXIS), _mm),
    InfoStep("VEL? 1", "vel", lambda s: s.qvel(AXIS), lambda v: f"{v:.4f} mm/s"),
    InfoStep("JON? 1", "jon", lambda s: s.qjon(JOYSTICK_ID), lambda v: f"{v}  joystick {'ON' if v else 'off'}"),
    InfoStep("RMC?", "rmc", lambda s: s.qrmc(), _show_rmc),
    InfoStep("MAC? DH", "mac_dh", lambda s: s.qmac("DH"), _show_macro(DH_MACRO_TEXT)),
    InfoStep("MAC? HOME", "mac_home", lambda s: s.qmac("HOME"), _show_macro(HOME_MACRO_TEXT)),
    InfoStep("#5", "moving", lambda s: s.is_moving(), _flag("moving", "standing still")),
    InfoStep("#7", "ready", lambda s: s.is_controller_ready(),
             lambda v: f"0x{READY_BYTE:02X}  ready" if v else f"0x{BUSY_BYTE:02X}  busy"),
    InfoStep("ERR?", "err_last", lambda s: s.qerr(), _show_err),
)

#: Exactly what the information mode sends when every query is answered (the purge, then INFO_STEPS).
INFO_FRAMES: tuple[str, ...] = PURGE_FRAMES + tuple(step.frame for step in INFO_STEPS)

#: Values the motion tests need from the information mode.
_MOTION_NEEDS = ("sai", "svo", "frf", "tmn", "tmx", "pos", "vel", "rmc", "moving")   # JON? 1: joystick test only


# ---------------------------------------------------------------------------------------------- window
@dataclass(frozen=True)
class Window:
    """An allowed position range (mm), inclusive."""

    lo: float
    hi: float

    def contains(self, x: float, margin: float = 0.0) -> bool:
        return self.lo - margin <= x <= self.hi + margin

    def text(self) -> str:
        return f"{self.lo:.4f} .. {self.hi:.4f} mm"


def compute_window(start: float, half_width: float, tmn: float, tmx: float,
                   zrange: tuple[float, float] | None) -> tuple[Window, list[tuple[str, float, float]]]:
    """The allowed window: the intersection of the controller range, the alox.ini range (when given) and
    ``start +/- half_width``. Returns the window and the ranges it came from; :class:`Refusal` when the start
    position is outside one of the fixed ranges."""
    limits = [("controller travel range (TMN? 1 .. TMX? 1)", tmn, tmx)]
    if zrange is not None:
        limits.append(("alox.ini [hardware.z] zmin_DBL .. zmax_DBL", zrange[0], zrange[1]))
    if not tmn < tmx:
        raise Refusal(f"the controller travel range is empty (TMN? 1 = {tmn:.4f}, TMX? 1 = {tmx:.4f})")
    for name, lo, hi in limits:
        if not lo <= start <= hi:
            raise Refusal(f"the stage is at {start:.4f} mm, outside the {name} {lo:.4f} .. {hi:.4f} mm. Move it "
                          "inside with LabVIEW first.")
    limits.append((f"start position +/- {half_width:.4f} mm (--window)", start - half_width, start + half_width))
    return Window(max(lo for _, lo, _ in limits), min(hi for _, _, hi in limits)), limits


# ---------------------------------------------------------------------------------------------- report
def _jsonable(value: Any) -> Any:
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def _git_commit() -> str | None:
    try:
        done = subprocess.run(["git", "rev-parse", "--short=12", "HEAD"], cwd=TOOL_DIR, capture_output=True,
                              text=True, timeout=5, check=False)
    except (OSError, subprocess.SubprocessError):
        return None
    return done.stdout.strip() or None if done.returncode == 0 else None


class Report:
    """The report sent back by the owner: everything read, every answer, every frame."""

    def __init__(self, argv: Sequence[str]) -> None:
        self.started = datetime.now().astimezone()
        self.data: dict[str, Any] = {
            "report": f"{TOOL_NAME} report",
            "versions": {"tool": TOOL_VERSION, "aloxcam": ALOXCAM_VERSION, "phase": PHASE,
                         "python": platform.python_version(), "pyserial": pyserial_version(),
                         "platform": platform.platform(), "git_commit": _git_commit()},
            "started": self.started.isoformat(timespec="seconds"),
            "command_line": [sys.argv[0] if sys.argv else TOOL_NAME, *argv],
            "connection": {},
            "alox_ini": None,
            "info": {"queries": []},
            "motion": None,
            "findings": [],
            "problems": [],
        }

    def finding(self, ref: str, topic: str, answer: str) -> None:
        self.data["findings"].append({"ref": ref, "topic": topic, "answer": answer})

    def problem(self, text: str) -> None:
        self.data["problems"].append(text)


def report_paths(out: Path | None, started: datetime) -> tuple[Path, Path]:
    """``<folder>/z_stage_report_<yyyymmdd-hhmmss>.json`` and ``.txt``; ``out`` is a folder, or a ``.json`` /
    ``.txt`` file name (the other file takes the same name)."""
    stem = REPORT_PREFIX + started.strftime("%Y%m%d-%H%M%S")
    if out is None:
        base = TOOL_DIR / stem
    elif out.suffix.lower() in (".json", ".txt"):
        base = out.with_suffix("")
    else:
        base = out / stem
    return base.with_suffix(".json"), base.with_suffix(".txt")


def render_text(data: dict[str, Any], json_path: Path) -> str:
    """The readable ``.txt`` summary of the report."""
    lines: list[str] = []
    add = lines.append
    add("Z STAGE CHECK REPORT")
    add(f"Send this report back: {json_path}  (this .txt is the same report, easier to read)")
    add("")
    v = data["versions"]
    add(f"Tool        {TOOL_NAME} {v['tool']}, aloxcam {v['aloxcam']} (phase {v['phase']}), git {v['git_commit']}")
    add(f"            Python {v['python']}, pyserial {v['pyserial']}, {v['platform']}")
    add(f"Started     {data['started']}")
    add(f"Finished    {data.get('finished', '')}")
    add(f"Command     {' '.join(data['command_line'])}")
    result = data.get("result", {})
    add(f"Result      {result.get('summary', '')} (exit code {result.get('exit_code', '')})")
    add("")
    conn = data["connection"]
    add("Connection")
    add(f"  Port      {conn.get('port', '')} at {conn.get('baudrate', '')} Bd ({conn.get('source', '')}), "
        f"timeout {conn.get('timeout_ms', '')} ms")
    purge = conn.get("purge")
    if purge:
        if "err_code" in purge:
            add(f"  Purge     XXX, *idn?, err?: err? read {purge['err_code']} - {purge['note']}")
        else:
            add(f"  Purge     failed: {purge.get('error', '')}")
    ini = data.get("alox_ini")
    if ini:
        add(f"  alox.ini  {ini['path']}: COM{ini['comport_I32']}, {ini['baudrate']} Bd, zmin_DBL {ini['zmin_DBL']}, "
            f"zmax_DBL {ini['zmax_DBL']}, speed_DBL {ini['speed_DBL']}")
    add("")
    add("Information (read-only)")
    for q in data["info"]["queries"]:
        if q["ok"]:
            add(f"  {q['frame']:<10} {q['shown']}")
        else:
            add(f"  {q['frame']:<10} FAILED: {q['error']}")
            if q.get("err_after"):
                add(f"  {'':<10} ERR? afterwards: {q['err_after']}")
        add(f"  {'':<10} raw: {' | '.join(q['raw']) or '(nothing received)'}")
    add("")
    add("Findings")
    for f in data["findings"]:
        add(f"  [{f['ref']}] {f['topic']}: {f['answer']}")
    if not data["findings"]:
        add("  (none)")
    motion = data.get("motion")
    if motion:
        add("")
        add("Motion tests")
        if motion.get("refused"):
            add(f"  Not run: {motion['refused']}")
        elif motion.get("declined"):
            add("  Not run: the operator did not type yes.")
        else:
            if "start_position_mm" in motion:
                add(f"  Start position {motion['start_position_mm']:.4f} mm, window {motion['window']['text']}, "
                    f"test speed {motion['speed_mm_s']:.4f} mm/s (original {motion['original_speed_mm_s']:.4f})")
            add(f"  DH at the start: {'running' if motion.get('dh_at_start') else 'not running'}")
            for t in motion["tests"]:
                add(f"  - {t['title']} ({t['ref']}): {t.get('status', '')}")
                if t.get("answer"):
                    add(f"      {t['answer']}")
                if t.get("reason"):
                    add(f"      {t['reason']}")
                if t.get("sent"):
                    add(f"      sent: {', '.join(t['sent'])}")
            back = motion.get("move_back")
            if back:
                add(f"  Move back: {back.get('result', '')}")
            for r in motion.get("restore", []):
                add(f"  Restore {r['what']}: {r['result']}")
            for e in motion.get("emergency_stops", []):
                add(f"  EMERGENCY STOP at {e['t_s']:.2f} s: {e['reason']}")
                for a in e["attempts"]:
                    add(f"      attempt {a['attempt']}{' (port re-opened)' if a['reopened'] else ''}: {a['result']}")
            if motion.get("aborted"):
                add(f"  Stopped: {motion['aborted']}")
            if "end_position_mm" in motion:
                add(f"  End position {motion['end_position_mm']}")
    add("")
    add("Problems")
    for p in data["problems"]:
        add(f"  - {p}")
    if not data["problems"]:
        add("  (none)")
    answers = data.get("operator_answers") or []
    if answers:
        add("")
        add("Operator answers")
        for a in answers:
            add(f"  {a['question']} -> {a['answer']!r} ({a['meaning']})")
    add("")
    add("Frames (tx = sent by this PC, rx = answer, drop = input discarded before a command; t in s)")
    for f in data.get("frames", []):
        add(f"  {f['t_s']:9.3f} {f['dir']:<4} {f['text']}")
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------------------------- target
@dataclass(frozen=True)
class Target:
    """Where to connect, and the alox.ini values when ``--alox-ini`` is given."""

    port: str
    baudrate: int
    source: str
    z: ZStageSettings | None = None
    alox_ini: Path | None = None
    ini_warnings: tuple[str, ...] = ()

    @property
    def zrange(self) -> tuple[float, float] | None:
        if self.z is None:
            return None
        return min(self.z.min_pos, self.z.max_pos), max(self.z.min_pos, self.z.max_pos)

    @property
    def com_number(self) -> int:
        if self.z is not None:
            return self.z.com if self.z.com > 0 else 1
        match = re.fullmatch(r"COM(\d+)", self.port, re.IGNORECASE)
        return int(match.group(1)) if match else 1


def _normal_port(text: str) -> str:
    text = text.strip()
    if text.isdigit():
        return f"COM{int(text)}"
    if re.fullmatch(r"com\d+", text, re.IGNORECASE):
        return text.upper()
    return text


def check_args(args: argparse.Namespace) -> None:
    """Command-line rules (safety first); :class:`UsageError` when one is broken."""
    if args.yes and not args.fake:
        raise UsageError("--yes is only accepted together with --fake: on the real stage the operator must type "
                         "yes for every test")
    if args.fake and args.port:
        raise UsageError("--fake and --port cannot be used together (--fake opens no port)")
    if not (args.fake or args.port or args.alox_ini):
        raise UsageError("say which port to use: --alox-ini <path to alox.ini> or --port COM<n> "
                         "(or --fake for a rehearsal)")
    if args.setup_twice and not args.motion:
        raise UsageError("--setup-twice is a motion test: add --motion")
    if not (math.isfinite(args.speed) and args.speed > 0):
        raise UsageError("--speed must be a positive number of mm/s")
    if args.speed < MIN_SPEED:
        raise UsageError(f"--speed below {MIN_SPEED:g} mm/s is too slow for the tests")
    if not args.fake and args.speed > MAX_SPEED_REAL:
        raise UsageError(f"--speed above {MAX_SPEED_REAL:g} mm/s is refused on the real stage")
    if not (math.isfinite(args.window) and args.window > 0):
        raise UsageError("--window must be a positive number of mm")
    if not args.fake and args.window > MAX_WINDOW_REAL:
        raise UsageError(f"--window above {MAX_WINDOW_REAL:g} mm is refused on the real stage")
    if args.baud is not None and args.baud <= 0:
        raise UsageError("--baud must be a positive number")
    if args.timeout_ms <= 0:
        raise UsageError("--timeout-ms must be a positive number")
    if args.alox_ini is not None and not args.alox_ini.is_file():
        raise UsageError(f"alox.ini not found: {args.alox_ini}")


def resolve_target(args: argparse.Namespace) -> Target:
    """Port, baud rate and alox.ini values from the command line (the port of ``--fake`` is set later)."""
    z = None
    warnings: list[str] = []
    if args.alox_ini is not None:
        z = load_alox_ini(args.alox_ini, CameraOptions(), warn=warnings.append).z
    if args.fake:
        return Target("", args.baud or DEFAULT_BAUD, "--fake (emulated C-663)", z, args.alox_ini, tuple(warnings))
    if args.port:
        baud = args.baud or (z.baudrate if z is not None else DEFAULT_BAUD)
        return Target(_normal_port(args.port), baud, "--port", z, args.alox_ini, tuple(warnings))
    assert z is not None
    return Target(z.port_name, args.baud or z.baudrate, f"alox.ini {args.alox_ini}", z, args.alox_ini,
                  tuple(warnings))


def start_fake_server(*, reference_s: float = 0.2) -> FakeC663Server:
    """``--fake``: an emulated C-663 (0-100 mm, DH running) whose stage is referenced at mid-travel (FRF)."""
    options = SimulationOptions(stage_min_mm=0.0, stage_max_mm=100.0, stage_reference_s=reference_s,
                                stage_initially_referenced=True, stage_identity=FAKE_IDENTITY)
    server = FakeC663Server(options).start()
    model = server.controller.model
    model.frf(AXIS)                                     # to mid-travel (FAKE_START_MM) in reference_s
    deadline = time.monotonic() + reference_s + 5.0
    while not model.referenced and time.monotonic() < deadline:
        time.sleep(0.01)
    model.qerr()
    return server


# ---------------------------------------------------------------------------------------------- the check
class StageCheck:
    """The connection, the information mode and the report data (the motion tests: :class:`MotionSession`)."""

    def __init__(self, args: argparse.Namespace, target: Target, env: Environment, console: Console,
                 report: Report) -> None:
        self.args = args
        self.target = target
        self.env = env
        self.console = console
        self.report = report
        self.log = FrameLog(env.monotonic)
        self.stage = PiSerialStage(recording_factory(env.link_factory or GcsSerialLink, self.log), sleep=env.sleep)
        self.values: dict[str, Any] = {}
        self.purge_code: int | None = None
        self.info_mark = 0

    # ============================================================================================== connection
    def connect(self) -> bool:
        c, t = self.console, self.target
        conn = self.report.data["connection"]
        conn.update(port=t.port, baudrate=t.baudrate, source=t.source, timeout_ms=self.args.timeout_ms,
                    fake=bool(self.args.fake))
        c.line(f"Port {t.port} at {t.baudrate} Bd ({t.source}), answer timeout {self.args.timeout_ms} ms.")
        c.line("Opening the port. This sends XXX, *idn? and err?: the purge LabVIEW's PI driver also sends when it")
        c.line("opens the port (XXX is not a command; the controller only notes error 2, which err? reads and clears).")
        self.info_mark = self.log.mark()
        try:
            code = self.stage.connect_rs232(t.port, t.baudrate, self.args.timeout_ms)
        except StageError as exc:
            conn["purge"] = {"sent": list(self.log.sent_since(self.info_mark)), "error": describe(exc)}
            self._link_failure("Opening the port failed", exc)
            return False
        self.purge_code = code
        if code == PURGE_OWN_ERROR:
            note = "normal: the purge's own XXX sets error 2 (Unknown command)"
        elif code == 0:
            note = "XXX did not set an error on this controller (finding: 2 was expected)"
        else:
            note = (f"{gcs_error_text(code) or 'unknown code'}: an error was pending before the tool connected; the "
                    "purge read and cleared it (as LabVIEW's Z Setup does, 08 Z22)")
        conn["purge"] = {"sent": list(self.log.sent_since(self.info_mark)), "err_code": code, "note": note}
        c.line(f"  purge err?  {code}  ({note})")
        return True

    def _link_failure(self, what: str, exc: StageError) -> None:
        c = self.console
        c.line()
        c.line(f"{what}: {describe(exc)}")
        hints = advice(exc, self.target.port, self.args.timeout_ms)
        for hint in hints:
            c.line(hint)
        self.report.problem(f"{what}: {describe(exc)}" + (f" -- {' '.join(h.strip() for h in hints)}" if hints else ""))

    def close(self) -> None:
        try:
            self.stage.close()
        except Exception as exc:                        # never hide the report because of a close failure
            self.report.problem(f"closing the port failed: {describe(exc)}")

    # ============================================================================================== information
    def run_info(self) -> bool:
        """The read-only queries of :data:`INFO_STEPS`; False when the link was lost (the rest is not asked)."""
        c = self.console
        c.line("Reading the controller (nothing here moves the stage, changes a setting or stops a macro):")
        timeouts = 0
        for step in INFO_STEPS:
            entry = self._query(step)
            if entry["ok"]:
                timeouts = 0
                continue
            if entry.get("link_lost"):
                self.report.problem("the link was lost during the information queries: the remaining ones were "
                                    "not sent")
                return False
            timeouts = timeouts + 1 if entry.get("code") == Code.PI_TIMEOUT else 0
            if timeouts >= MAX_TIMEOUTS_IN_A_ROW:
                c.line(f"The controller did not answer {timeouts} queries in a row: the remaining ones are not sent.")
                self.report.problem("the controller stopped answering during the information queries")
                return False
        failed = sum(1 for q in self.report.data["info"]["queries"] if not q["ok"])
        self.report.data["info"]["failed"] = failed
        c.line(f"Information done: {len(INFO_STEPS)} queries, {failed} failed.")
        if failed:
            self.report.problem(f"{failed} information queries failed (see the report)")
        return True

    def _query(self, step: InfoStep) -> dict[str, Any]:
        c = self.console
        mark = self.log.mark()
        t0 = self.env.monotonic()
        entry: dict[str, Any] = {"frame": step.frame, "key": step.key}
        self.report.data["info"]["queries"].append(entry)
        try:
            value = step.read(self.stage)
        except StageError as exc:
            entry.update(ok=False, code=exc.code, error=describe(exc))
            entry["raw"] = [frame_text(f.data) for f in self.log.since(mark) if f.direction != TX]
            c.line(f"  {step.frame:<10} FAILED: {describe(exc)}")
            if exc.code in LINK_ERRORS:
                entry["link_lost"] = True
                self._link_failure(f"{step.frame} failed", exc)
                return entry
            entry["err_after"] = self._err_after()
            c.line(f"  {'':<10} ERR? afterwards: {entry['err_after']}")
            return entry
        finally:
            entry["ms"] = round((self.env.monotonic() - t0) * 1000.0, 1)
        self.values[step.key] = value
        entry.update(ok=True, value=_jsonable(value), shown=step.show(value),
                     raw=[frame_text(f.data) for f in self.log.since(mark) if f.direction != TX])
        c.line(f"  {step.frame:<10} {entry['shown']}")
        if step.key in ("mac_dh", "mac_home"):
            for line in str(value).splitlines():
                c.line(f"  {'':<12} {line}")
        if step.key == "err_first":
            c.line(f"  {'':<10} (reading ERR? clears the controller's error; the value read is in the report)")
        return entry

    def _err_after(self) -> str:
        """``ERR?`` right after a failed query: the controller's reason (also clears it)."""
        try:
            code = self.stage.qerr()
        except StageError as exc:
            return f"ERR? failed too: {describe(exc)}"
        return _show_err(code)

    # ============================================================================================== findings
    def info_findings(self) -> None:
        v, r = self.values, self.report
        if self.purge_code is not None:
            r.finding("08 §3.2, Z22", "Open purge (XXX, *idn?, err?)",
                      f"err? read {self.purge_code}: {self.report.data['connection']['purge']['note']}")
        if "idn" in v:
            r.finding("Q9", "*IDN? answer", repr(v["idn"]))
            r.finding("Q35 test 7", "Z Get Serial (Python server)",
                      f"the Python server's Z Get Serial (Type = FALSE) returns {serial_from_idn(v['idn'])!r} (the "
                      "3rd field of *IDN?); note what the LabVIEW server returns, with the LabVIEW client")
        if "cst" in v:
            r.finding("Q9", "Stage connected (CST? 1)", repr(v["cst"]))
        if "trs" in v and "lim" in v:
            how = "FRF 1 (reference switch)" if v["trs"] else "FNL 1 (negative limit switch)" if v["lim"] else \
                "nothing (neither switch)"
            r.finding("Q9", "Reference switch or limit switches",
                      f"TRS? 1 = {int(v['trs'])} (reference switch {'yes' if v['trs'] else 'no'}), LIM? 1 = "
                      f"{int(v['lim'])} (limit switches {'yes' if v['lim'] else 'no'}): the Python Z Setup "
                      f"references with {how}")
        for key, name, documented in (("mac_dh", "DH", DH_MACRO_TEXT), ("mac_home", "HOME", HOME_MACRO_TEXT)):
            if key in v:
                same = _same_macro(v[key], documented)
                r.finding("Q10", f"MAC? {name}", "same as docs/answers/owner-answers.md §5" if same else
                          f"DIFFERENT from docs/answers/owner-answers.md §5: {v[key]!r}")
        if "rmc" in v:
            r.finding("Q10", "Running macro (RMC?)", repr(v["rmc"]))
        formats = []
        for q in self.report.data["info"]["queries"]:
            if q["ok"] and q["frame"] in ("POS? 1", "VEL? 1", "TMX? 1", "FRF? 1", "JON? 1", "#5", "#7", "SAI?"):
                formats.append(f"{q['frame']} -> {' | '.join(q['raw'])}")
        if formats:
            r.finding("design §6", "Answer formats (raw)", "; ".join(formats))
        lf = self._ready_byte_lf()
        if lf is not None:
            r.finding("design §6", "#7 answer", "the ready byte is followed by an LF" if lf else
                      "a single byte, no LF after it")

    def _ready_byte_lf(self) -> bool | None:
        """Whether the ``#7`` answer of the information mode was followed by an LF (seen as dropped input)."""
        frames = self.log.since(self.info_mark)
        for i, f in enumerate(frames):
            if f.direction == TX and f.data == b"\x07":
                if i + 1 >= len(frames) or frames[i + 1].direction != RX:
                    return None
                rest = frames[i + 2:i + 3]
                return bool(rest) and rest[0].direction == DROPPED and rest[0].data.startswith(b"\n")
        return None

    # ============================================================================================== run
    def run(self) -> int:
        c = self.console
        c.section(f"Z stage check {TOOL_VERSION} (aloxcam {ALOXCAM_VERSION}) - information, read-only")
        if self.args.fake:
            c.line("REHEARSAL (--fake): an emulated C-663 inside this program; no serial port is opened.")
        for warning in self.target.ini_warnings:
            c.line(f"alox.ini warning: {warning}")
        if not self.connect():
            return EXIT_PROBLEM
        ok = self.run_info()
        self.info_findings()
        if not ok:
            return EXIT_PROBLEM
        if self.args.motion:
            aborted = MotionSession(self).run()
            if aborted:
                return EXIT_PROBLEM
        return EXIT_OK


# ---------------------------------------------------------------------------------------------- motion tests
@dataclass(frozen=True)
class CheckSpec:
    key: str
    title: str
    ref: str
    question: str


TEST_SPECS: tuple[CheckSpec, ...] = (
    CheckSpec("pos_during_move", "POS? during a move", "Q35 test 2",
             "Does POS? 1 give the real position while the stage moves, or the target?"),
    CheckSpec("stp_during_move", "STP 1 during a move", "Q35 test 1",
             "Does STP 1 (the frame of Z Stop) stop a moving stage? What does ERR? give afterwards?"),
    CheckSpec("mvr_during_move", "MVR while a move runs", "Q35 test 3",
             "Does MVR add the distance to the target of the running move, or to the position at that moment?"),
    CheckSpec("mov_with_joystick", "MOV with the joystick on", "Q35 test 4",
             "Is MOV refused while joystick 1 is on? Which error does ERR? give?"),
    CheckSpec("vel_zero", "VEL 0", "Q35 test 5",
             "Which error does VEL 1 0.0000 give, and does the speed change?"),
)
SETUP_TWICE_SPEC = CheckSpec("setup_twice", "Z Setup twice", "Q35 test 6, Q11",
                            "How long does the server's Z Setup take, does it reference (FNL) every time, and does #7 "
                            "answer busy while the axis is referencing?")


@dataclass(frozen=True)
class Sample:
    t: float                            # seconds since the command
    moving: bool
    pos: float


@dataclass
class Plan:
    lines: list[str]
    values: dict[str, float] = field(default_factory=dict)


def _same_speed(read: float, wanted: float) -> bool:
    return abs(read - wanted) <= max(1e-4, 1e-3 * abs(wanted))


class MotionSession:
    """The guided motion tests (design §4.1 "tests"). Every rule of the module docstring's "Safety rules" is here."""

    def __init__(self, check: StageCheck) -> None:
        self.check = check
        self.stage = check.stage
        self.console = check.console
        self.env = check.env
        self.args = check.args
        self.data: dict[str, Any] = {"requested": True, "tests": [], "emergency_stops": [], "restore": []}
        check.report.data["motion"] = self.data
        self.specs = TEST_SPECS + ((SETUP_TWICE_SPEC,) if self.args.setup_twice else ())
        self.half_width = float(self.args.window)
        self.armed = False
        self.velocity_touched = False
        self.joystick_touched = False
        self.after_setup = False
        self.quit = False
        self.start = 0.0
        self.speed = float(self.args.speed)
        self.original_speed = 0.0
        self.window = Window(0.0, 0.0)
        self.limits: list[tuple[str, float, float]] = []
        self.dh_at_start = False

    # ============================================================================================== small helpers
    def _now(self) -> float:
        return self.env.monotonic()

    def _timeout(self, distance: float) -> float:
        return MOVE_TIMEOUT_MIN_S + MOVE_TIMEOUT_FACTOR * abs(distance) / self.speed

    def _need_mm(self) -> float:
        return max(MIN_MOVE_MM, MIN_MOVE_S * self.speed)

    def _direction(self, pos: float) -> tuple[int, float]:
        """The side with more free travel inside the window: (+1 or -1, free travel in mm)."""
        up, down = self.window.hi - pos, pos - self.window.lo
        return (1, up) if up >= down else (-1, down)

    def _travel(self, pos: float) -> tuple[int, float]:
        sign, room = self._direction(pos)
        travel = min(room, self.half_width)
        need = self._need_mm()
        if travel < need:
            raise Refusal(f"the window is too small for this test: at {self.speed:.4f} mm/s it needs {need:.4f} mm "
                          f"of free travel on one side, but only {max(room, 0.0):.4f} mm is free (window "
                          f"{self.window.text()}). Use a larger --window or a lower --speed.")
        return sign, travel

    def _err(self) -> int:
        return self.stage.qerr()

    # ============================================================================================== watching a move
    def watch(self, t_cmd: float, allowed: Window, timeout_s: float, *, targets: Sequence[float] = (),
              until: Callable[[Sample], bool] | None = None, on_timeout: str = "stop") -> tuple[list[Sample], str]:
        """Read ``#5`` and ``POS? 1`` every :data:`POLL_S` until the stage stands still (at one of ``targets``, or
        :data:`SETTLE_S` after the command), or until ``until(sample)``. Returns the samples and ``"stopped"`` /
        ``"until"`` / ``"timeout"``. A position outside ``allowed`` (+ margin) sends ``#24`` and raises
        :class:`SafetyStop`; so does a timeout unless ``on_timeout == "return"``."""
        samples: list[Sample] = []
        while True:
            moving = self.stage.is_moving()
            pos = self.stage.qpos(AXIS)
            sample = Sample(round(self._now() - t_cmd, 4), moving, pos)
            samples.append(sample)
            if not allowed.contains(pos, WINDOW_MARGIN_MM):
                reason = f"the stage left the allowed range {allowed.text()}: POS? 1 = {pos:.4f} mm"
                self.emergency_stop(reason)
                raise SafetyStop(reason, stop_sent=True)
            if until is not None and until(sample):
                return samples, "until"
            if not moving and (not targets or any(abs(pos - t) <= AT_TARGET_MM for t in targets)
                               or sample.t >= SETTLE_S):
                return samples, "stopped"
            if sample.t > timeout_s:
                if on_timeout == "return":
                    return samples, "timeout"
                reason = f"timeout: the stage still moved {timeout_s:.1f} s after the command (POS? 1 = {pos:.4f} mm)"
                self.emergency_stop(reason)
                raise SafetyStop(reason, stop_sent=True)
            self.env.sleep(POLL_S)

    @staticmethod
    def _samples(samples: Sequence[Sample]) -> list[list[Any]]:
        return [[s.t, int(s.moving), s.pos] for s in samples]

    # ============================================================================================== stopping
    def _send_stop(self, reopen: bool) -> str | None:
        """``#24`` (after re-opening the port when ``reopen``); None when sent, else what went wrong."""
        try:
            if reopen:
                t = self.check.target
                self.stage.connect_rs232(t.port, t.baudrate, self.args.timeout_ms)
            self.stage.stop_all()
        except KeyboardInterrupt:
            return "interrupted by Ctrl+C"
        except Exception as exc:
            return describe(exc)
        return None

    def emergency_stop(self, reason: str, *, link_error: bool = False) -> bool:
        """Send ``#24`` (stop all axes) now. When the link itself failed (``link_error``) or the first ``#24`` could
        not be sent, re-open the port and send it again (twice at most). Then watch for standstill. Returns whether
        the stop is known to have reached the controller."""
        c = self.console
        c.line()
        c.line(f"!!! STOP: {reason}")
        c.line("!!! Sending #24 (stop all axes) now.")
        event: dict[str, Any] = {"t_s": round(self.check.log.elapsed(), 3), "reason": reason, "attempts": []}
        self.data["emergency_stops"].append(event)
        error = self._send_stop(reopen=False)
        event["attempts"].append({"attempt": 1, "reopened": False, "result": error or "#24 sent"})
        certain = error is None and not link_error
        if not certain:
            for attempt in (2, 3):
                c.line("!!! Not sure #24 arrived: re-opening the port and sending #24 again.")
                error = self._send_stop(reopen=True)
                event["attempts"].append({"attempt": attempt, "reopened": True, "result": error or "#24 sent"})
                if error is None:
                    certain = True
                    break
        event["sent"] = certain
        if certain:
            c.line(f"!!! #24 sent (attempt {len(event['attempts'])}).")
            if self.joystick_touched:                    # a joystick-driven drift: switch it off at once too
                try:
                    self.stage.jon(JOYSTICK_ID, False)
                    event["joystick_off"] = "JON 1 0 sent"
                    c.line("!!! JON 1 0 sent (joystick off).")
                except (KeyboardInterrupt, Exception) as exc:
                    event["joystick_off"] = f"failed: {describe(exc)}"
        if not certain:
            c.line("!!! COULD NOT CONFIRM THAT #24 REACHED THE CONTROLLER.")
            c.line("!!! If the stage moves, stop it by hand: switch off the C-663 controller (or use the machine's")
            c.line("!!! emergency stop).")
            self.check.report.problem(f"emergency stop ({reason}): #24 could not be confirmed")
            return False
        self._wait_standstill(event)
        return True

    def _wait_standstill(self, event: dict[str, Any]) -> None:
        c = self.console
        t0 = self._now()
        try:
            while True:
                if not self.stage.is_moving():
                    pos = self.stage.qpos(AXIS)
                    event.update(standstill=True, position_mm=pos)
                    c.line(f"The stage stands still at {pos:.4f} mm.")
                    return
                if self._now() - t0 > STANDSTILL_WAIT_S:
                    event["standstill"] = False
                    c.line("!!! The stage STILL MOVES after #24. Stop it by hand: switch off the C-663 controller.")
                    self.check.report.problem("the stage still moved after #24")
                    return
                self.env.sleep(POLL_S)
        except KeyboardInterrupt:
            event["standstill"] = "not checked (Ctrl+C)"
        except Exception as exc:
            event["standstill"] = f"could not be checked: {describe(exc)}"
            c.line(f"Could not check that the stage stands still: {describe(exc)}")

    def _guarded(self, step: Callable[[], None]) -> str | None:
        """Run ``step``; on Ctrl+C, any error or a :class:`SafetyStop`, send ``#24`` (unless already sent) and return
        the reason. None when ``step`` ended normally."""
        handled = False
        try:
            step()
            handled = True
            return None
        except SafetyStop as exc:
            if not exc.stop_sent:
                self.emergency_stop(exc.reason)
            handled = True
            return exc.reason
        except KeyboardInterrupt:
            self.emergency_stop("Ctrl+C pressed")
            handled = True
            return "Ctrl+C pressed by the operator"
        except Exception as exc:
            self.data.setdefault("errors", []).append(traceback.format_exc())
            link_error = isinstance(exc, StageError) and exc.code in LINK_ERRORS
            reason = f"unexpected error: {describe(exc)}"
            self.emergency_stop(reason, link_error=link_error)
            if isinstance(exc, StageError):
                for hint in advice(exc, self.check.target.port, self.args.timeout_ms):
                    self.console.line(hint)
            handled = True
            return reason
        finally:
            if not handled:                             # e.g. a second Ctrl+C inside the handlers above
                self.emergency_stop("the tool is stopping")

    # ============================================================================================== the session
    def _refusal(self) -> str | None:
        """Why the motion tests cannot run (from the information read), or None. Also sets the window and speed."""
        v = self.check.values
        missing = [step.frame for step in INFO_STEPS if step.key in _MOTION_NEEDS and step.key not in v]
        if missing:
            return f"these values could not be read: {', '.join(missing)}"
        if AXIS not in v["sai"]:
            return f"axis {AXIS} is not in the SAI? answer {v['sai']!r}"
        if not v["svo"]:
            return "the servo is off (SVO? 1 = 0): moves would be refused. Run Z Setup in LabVIEW first."
        if not v["frf"]:
            return ("the axis is not referenced (FRF? 1 = 0): moves would be refused (error 5). Run Z Setup in "
                    "LabVIEW first (it references the axis), then run this tool again.")
        if v.get("jon"):
            return ("the joystick is on (JON? 1 = 1): moves would be refused. Switch it off (LabVIEW Z Disable "
                    "Joystick), then run this tool again.")
        if v["moving"]:
            return "the stage is moving (#5 = 1)."
        if v["rmc"].strip() == "HOME":
            return "the HOME macro is running (it moves the stage to Z = 0 mm)."
        if not (math.isfinite(v["vel"]) and v["vel"] > 0):
            return f"the controller's speed is not a positive number (VEL? 1 = {v['vel']!r})"
        self.start = float(v["pos"])
        self.original_speed = float(v["vel"])
        self.speed = min(float(self.args.speed), self.original_speed) if not self.args.fake else float(self.args.speed)
        if self.speed < MIN_SPEED:
            return (f"the controller's speed is very low (VEL? 1 = {self.original_speed:.4f} mm/s) and the tool never "
                    "moves faster than it: set the normal speed with LabVIEW (Z Setup) first")
        self.dh_at_start = v["rmc"] in KNOWN_MACROS
        try:
            self.window, self.limits = compute_window(self.start, self.half_width, float(v["tmn"]), float(v["tmx"]),
                                                      self.check.target.zrange)
        except Refusal as exc:
            return str(exc)
        return None

    def run(self) -> str | None:
        """The whole motion phase; returns why it was stopped (#24 sent), or None."""
        c = self.console
        c.section("Motion tests - THE STAGE WILL MOVE")
        refusal = self._refusal()
        if refusal:
            c.line(f"Motion tests NOT run: {refusal}")
            self.data["refused"] = refusal
            self.check.report.problem(f"motion tests not run: {refusal}")
            return None
        self.data.update(start_position_mm=self.start, speed_mm_s=self.speed, original_speed_mm_s=self.original_speed,
                         dh_at_start=self.dh_at_start, window_half_width_mm=self.half_width,
                         window={"lo": self.window.lo, "hi": self.window.hi, "text": self.window.text(),
                                 "limits": [{"from": n, "lo": lo, "hi": hi} for n, lo, hi in self.limits]})
        self._print_plan()
        if c.ask("Type yes and press Enter to start the motion tests (anything else stops here):") != "yes":
            c.line("Motion tests not started. Nothing was moved.")
            self.data["declined"] = True
            return None
        try:                                                              # still where it was read?
            pos, moving = self.stage.qpos(AXIS), self.stage.is_moving()
        except StageError as exc:
            reason = f"the stage could not be read again ({describe(exc)}). Nothing was moved."
            c.line(f"Motion tests NOT run: {reason}")
            self.data["refused"] = reason
            self.check.report.problem(f"motion tests not run: {reason}")
            return None
        if moving or abs(pos - self.start) > AT_TARGET_MM:
            reason = (f"the stage moved since it was read (POS? 1 was {self.start:.4f} mm, now {pos:.4f} mm, "
                      f"#5 = {int(moving)}). Nothing was moved by the tool; run it again.")
            c.line(f"Motion tests NOT run: {reason}")
            self.data["refused"] = reason
            self.check.report.problem(f"motion tests not run: {reason}")
            return None
        abort: str | None = "the tool stopped unexpectedly"
        try:
            self.armed = True
            abort = self._guarded(self._run_tests)
            if abort is None:
                abort = self._guarded(self._move_back)
        finally:
            self._finish(abort)
        return abort

    def _print_plan(self) -> None:
        c, v = self.console, self.check.values
        c.line(f"Start position   {self.start:.4f} mm (POS? 1)")
        c.line(f"Allowed window   {self.window.text()}: the intersection of")
        for name, lo, hi in self.limits:
            c.line(f"                   {lo:.4f} .. {hi:.4f} mm  {name}")
        c.line(f"Test speed       {self.speed:.4f} mm/s (the controller's speed now: {self.original_speed:.4f} mm/s; "
               "it is restored at the end)")
        if self.speed < self.args.speed:
            c.line("                 (lower than --speed: the tool never moves faster than the controller's speed)")
        c.line("Tests (each one is explained first and needs its own yes):")
        for index, spec in enumerate(self.specs, 1):
            c.line(f"  {index}. {spec.title}  ({spec.ref})")
        c.line()
        if self.dh_at_start:
            c.line(f"About the DH macro: it is running in the controller (RMC? = {v['rmc']!r}). While DH runs, the")
            c.line("controller starts the HOME macro when digital input 1 is high (for example the HOME button): HOME")
            c.line("switches the joystick off and MOVES THE STAGE TO Z = 0 mm. This tool never starts HOME and never")
            c.line("touches digital input 1. Do not press the HOME button during the tests. If the stage leaves the")
            c.line("allowed window, the tool sends #24 (stop all axes) at once. STP and #24 also stop DH; the tool")
            c.line("restarts DH at the end.")
            if self.window.lo <= DH_OUTPUT_MM <= self.window.hi:
                c.line(f"The window crosses Z = {DH_OUTPUT_MM} mm: while DH runs, digital output 1 changes when the "
                       "stage crosses it.")
        else:
            c.line(f"No DH macro runs now (RMC? = {v['rmc']!r}): the tool will not start one at the end.")
        c.line()
        c.line("Before you type yes: make sure nothing on or near the stage can be hit or damaged by a move inside")
        c.line("the window, and keep the controller power switch (or the machine's emergency stop) within reach.")
        c.line("Press Ctrl+C at any time to stop: the tool sends #24 (stop all axes) and restores the speed.")

    def _run_tests(self) -> None:
        for index, spec in enumerate(self.specs, 1):
            if self.quit:
                self.data["tests"].append({"key": spec.key, "title": spec.title, "ref": spec.ref,
                                           "status": "skipped (quit)"})
                continue
            self._run_one(index, spec)

    def _ensure_speed(self) -> None:
        """``VEL 1 <test speed>``, ``ERR?``, ``VEL? 1`` before the first move (only once something was confirmed, so
        that declining every test changes nothing)."""
        if "set_speed" in self.data:
            return
        c = self.console
        c.line(f"Setting the test speed: VEL 1 {format_gcs_float(self.speed)}, then ERR? and VEL? 1.")
        self.velocity_touched = True
        self.stage.vel(AXIS, self.speed)
        err = self._err()
        read = self.stage.qvel(AXIS)
        self.data["set_speed"] = {"err": err, "vel_read": read}
        if err or not _same_speed(read, self.speed):
            raise SafetyStop(f"the test speed could not be set (ERR? = {err}, VEL? 1 = {read:.4f} mm/s)")
        c.line(f"Test speed set: VEL? 1 = {read:.4f} mm/s.")

    def _run_one(self, index: int, spec: CheckSpec) -> None:
        c = self.console
        c.section(f"Test {index} of {len(self.specs)}: {spec.title}  ({spec.ref})")
        c.line(f"Question: {spec.question}")
        entry: dict[str, Any] = {"key": spec.key, "title": spec.title, "ref": spec.ref, "question": spec.question,
                                 "status": "not started"}
        self.data["tests"].append(entry)
        entry["pending_error_before"] = self._err()
        pos = self.stage.qpos(AXIS)
        moving = self.stage.is_moving()
        entry["position_before_mm"] = pos
        if moving:
            raise SafetyStop("the stage is moving before the test (HOME macro?)")
        if spec.key != SETUP_TWICE_SPEC.key and not self.window.contains(pos, WINDOW_MARGIN_MM):
            raise SafetyStop(f"the stage is at {pos:.4f} mm, outside the window {self.window.text()}, before the test")
        try:
            plan: Plan = getattr(self, f"_plan_{spec.key}")(pos)
        except Refusal as exc:
            entry.update(status="refused", reason=str(exc))
            c.line(f"Test refused: {exc}")
            return
        entry["plan"] = plan.values
        c.line("What will happen:")
        for line in plan.lines:
            c.line(f"  {line}")
        if spec.key != SETUP_TWICE_SPEC.key:
            c.line(f"  Allowed window {self.window.text()}, speed {self.speed:.4f} mm/s.")
        answer = c.ask(TEST_QUESTION)
        if answer == "yes" and spec.key == SETUP_TWICE_SPEC.key:
            c.line("WARNING: this test drives the stage to its NEGATIVE LIMIT SWITCH, outside the window. The window")
            c.line("check does not apply while Z Setup runs. Make sure the whole travel down to the limit is free.")
            answer = c.ask("Type yes again to confirm the reference moves:")
        if answer == "quit":
            self.quit = True
        if answer != "yes":
            entry["status"] = "skipped"
            c.line("Skipped.")
            return
        mark = self.check.log.mark()
        self._ensure_speed()
        entry.update(status="running", started_s=round(self.check.log.elapsed(), 3))
        try:
            result: dict[str, Any] = getattr(self, f"_run_{spec.key}")(plan)
        finally:
            entry["frames"] = [mark, self.check.log.mark()]
            entry["sent"] = self.check.log.sent_since(mark)
        entry.update(result)
        entry["status"] = result.get("status", "done")
        c.line(f"Result: {result['answer']}")
        self.check.report.finding(spec.ref, spec.title, result["answer"])

    # ============================================================================================== test 1: POS?
    def _plan_pos_during_move(self, pos: float) -> Plan:
        sign, travel = self._travel(pos)
        target = pos + sign * travel
        return Plan([f"MOV 1 {format_gcs_float(target)}: from {pos:.4f} to {target:.4f} mm ({travel:.4f} mm, about "
                     f"{travel / self.speed:.1f} s).",
                     f"While it moves: #5 and POS? 1 every {POLL_S:.2f} s. Then POS? 1 and ERR?."],
                    {"start_mm": pos, "target_mm": target})

    def _run_pos_during_move(self, plan: Plan) -> dict[str, Any]:
        start, target = plan.values["start_mm"], plan.values["target_mm"]
        t = self._now()
        self.stage.mov(AXIS, target)
        samples, _ = self.watch(t, self.window, self._timeout(target - start), targets=(target,))
        final, err = self.stage.qpos(AXIS), self._err()
        lo, hi = sorted((start, target))
        moving = [s for s in samples if s.moving]
        between = {round(s.pos, 4) for s in moving if lo + AT_TARGET_MM < s.pos < hi - AT_TARGET_MM}
        at_target = [s for s in moving if abs(s.pos - target) <= AT_TARGET_MM]
        details = f"{len(moving)} readings while moving; final POS? 1 = {final:.4f} mm; ERR? = {_show_err(err)}"
        if not moving:
            status, answer = "inconclusive", f"the stage was never seen moving (#5 = 0). {details}"
        elif len(between) >= 2:
            status, answer = "done", (f"POS? 1 gives the REAL position: it changed during the move ({len(between)} "
                                      f"different values between start and target). {details}")
        elif len(at_target) == len(moving):
            status, answer = "done", f"POS? 1 gave the TARGET during the whole move. {details}"
        else:
            status, answer = "inconclusive", f"not clear from the readings. {details}"
        return {"status": status, "answer": answer, "final_mm": final, "err": err, "samples": self._samples(samples)}

    # ============================================================================================== test 2: STP
    def _plan_stp_during_move(self, pos: float) -> Plan:
        sign, travel = self._travel(pos)
        target = pos + sign * travel
        stop_at = ACT_FRACTION * travel / self.speed
        return Plan([f"MOV 1 {format_gcs_float(target)}: from {pos:.4f} to {target:.4f} mm ({travel:.4f} mm, about "
                     f"{travel / self.speed:.1f} s).",
                     f"After about {stop_at:.2f} s, while it moves: STP 1 (the frame of Z Stop), then at once ERR? "
                     "(error 10 is expected).",
                     f"Then #5 and POS? 1 until it stands still; if it still moves {STOP_GRACE_S:.0f} s after STP 1, "
                     "the tool sends #24.",
                     "Then POS? 1, ERR? and RMC? (STP may stop the DH macro)."],
                    {"start_mm": pos, "target_mm": target, "stop_at_s": stop_at})

    def _run_stp_during_move(self, plan: Plan) -> dict[str, Any]:
        start, target, stop_at = plan.values["start_mm"], plan.values["target_mm"], plan.values["stop_at_s"]
        t = self._now()
        self.stage.mov(AXIS, target)
        samples, why = self.watch(t, self.window, self._timeout(target - start), targets=(target,),
                                  until=lambda s: s.t >= stop_at)
        result: dict[str, Any] = {"samples": self._samples(samples)}
        if why != "until" or not samples[-1].moving:
            final, err = self.stage.qpos(AXIS), self._err()
            result.update(status="inconclusive", final_mm=final, err=err,
                          answer=f"the stage was not moving when STP 1 was due (final POS? 1 = {final:.4f} mm, ERR? = "
                                 f"{_show_err(err)}); STP 1 was not sent")
            return result
        at_stop = samples[-1].pos
        t_stop = self._now()
        self.stage.stp(AXIS)
        err_stp = self._err()                            # at once: a later #24 would replace a refusal code
        result["err_after_stp"] = err_stp
        after, why_after = self.watch(t_stop, self.window, STOP_GRACE_S, on_timeout="return")
        stopped = why_after == "stopped"
        result["samples_after_stp"] = self._samples(after)
        if not stopped:
            self.console.line(f"STP 1 did not stop the stage within {STOP_GRACE_S:.0f} s: sending #24.")
            t24 = self._now()
            self.stage.stop_all()
            more, _ = self.watch(t24, self.window, self._timeout(target - start))
            result["samples_after_24"] = self._samples(more)
        final, err, rmc = self.stage.qpos(AXIS), self._err(), self.stage.qrmc()
        result.update(final_mm=final, err=err, rmc_after=rmc, position_at_stp_mm=at_stop)
        tail = (f"ERR? right after STP 1 = {_show_err(err_stp)}; ERR? once it stood still = {_show_err(err)}; "
                f"RMC? afterwards = {rmc!r}")
        if not stopped:
            result.update(status="done", answer=f"STP 1 did NOT stop the stage within {STOP_GRACE_S:.0f} s; #24 was "
                                                f"sent; it stood still at {final:.4f} mm. {tail}")
        elif abs(final - target) <= AT_TARGET_MM:
            result.update(status="done", answer=f"STP 1 did NOT stop the stage: it went on to its target {target:.4f} "
                                                f"mm (refused, or too late; see the first ERR?). {tail}")
        else:
            result.update(status="done", answer=f"STP 1 STOPPED the stage: it stood still {after[-1].t:.2f} s after "
                                                f"STP 1, at {final:.4f} mm (target was {target:.4f} mm). {tail}")
        return result

    # ============================================================================================== test 3: MVR
    def _plan_mvr_during_move(self, pos: float) -> Plan:
        sign, travel = self._travel(pos)
        first = travel * 2.0 / 3.0
        target = pos + sign * first
        dist = float(format_gcs_float(sign * travel / 3.0))
        mvr_at = ACT_FRACTION * first / self.speed
        return Plan([f"MOV 1 {format_gcs_float(target)}: from {pos:.4f} to {target:.4f} mm ({first:.4f} mm, about "
                     f"{first / self.speed:.1f} s).",
                     f"After about {mvr_at:.2f} s, while it moves: POS? 1, then MVR 1 {format_gcs_float(dist)}.",
                     "Then #5 and POS? 1 until it stands still; POS? 1 and ERR?.",
                     f"If MVR adds to the target the stage ends at {target + dist:.4f} mm; if it adds to the position "
                     f"at that moment it ends between {min(pos, target) + dist:.4f} and "
                     f"{max(pos, target) + dist:.4f} mm."],
                    {"start_mm": pos, "target_mm": target, "mvr_mm": dist, "mvr_at_s": mvr_at})

    def _run_mvr_during_move(self, plan: Plan) -> dict[str, Any]:
        start, target = plan.values["start_mm"], plan.values["target_mm"]
        dist, mvr_at = plan.values["mvr_mm"], plan.values["mvr_at_s"]
        t = self._now()
        self.stage.mov(AXIS, target)
        samples, why = self.watch(t, self.window, self._timeout(target - start), targets=(target,),
                                  until=lambda s: s.t >= mvr_at)
        result: dict[str, Any] = {"samples": self._samples(samples)}
        if why != "until" or not samples[-1].moving:
            final, err = self.stage.qpos(AXIS), self._err()
            result.update(status="inconclusive", final_mm=final, err=err,
                          answer=f"the move ended before MVR could be sent (POS? 1 = {final:.4f} mm, ERR? = "
                                 f"{_show_err(err)}); MVR was not sent")
            return result
        at_mvr = samples[-1].pos
        by_target, by_position = target + dist, at_mvr + dist
        t_mvr = self._now()
        self.stage.mvr(AXIS, dist)
        after, _ = self.watch(t_mvr, self.window, self._timeout(abs(target - at_mvr) + abs(dist)),
                              targets=(by_target, by_position))
        final, err = self.stage.qpos(AXIS), self._err()
        result.update(samples_after_mvr=self._samples(after), final_mm=final, err=err, position_at_mvr_mm=at_mvr,
                      if_added_to_target_mm=by_target, if_added_to_position_mm=by_position)
        separation = abs(by_target - by_position)
        d_target, d_position = abs(final - by_target), abs(final - by_position)
        tail = (f"MVR 1 {format_gcs_float(dist)} was sent at {at_mvr:.4f} mm while moving to {target:.4f} mm; final "
                f"POS? 1 = {final:.4f} mm (target + d = {by_target:.4f}, position + d = {by_position:.4f}); "
                f"ERR? = {_show_err(err)}")
        if separation < 3 * AT_TARGET_MM:
            result.update(status="inconclusive", answer=f"the two answers are too close to tell apart. {tail}")
        elif d_target < separation / 3 and d_target < d_position:
            result.update(status="done", answer=f"MVR adds to the TARGET of the running move. {tail}")
        elif d_position < separation / 3:
            result.update(status="done", answer=f"MVR adds to the POSITION at that moment. {tail}")
        else:
            result.update(status="inconclusive", answer=f"the final position matches neither answer. {tail}")
        return result

    # ============================================================================================== test 4: joystick
    def _plan_mov_with_joystick(self, pos: float) -> Plan:
        if "jon" not in self.check.values:
            raise Refusal("JON? 1 could not be read in the information part, so the joystick state is unknown")
        sign, room = self._direction(pos)
        move = min(JOYSTICK_MOVE_MM, room)
        if move < MIN_JOYSTICK_MOVE_MM:
            raise Refusal(f"the window is too small for this test: it needs {MIN_JOYSTICK_MOVE_MM:.4f} mm of free "
                          f"travel, only {max(room, 0.0):.4f} mm is free (window {self.window.text()})")
        target = pos + sign * move
        return Plan(["JAX 1 1 1, JON 1 1, then ERR? and JON? 1: joystick 1 ON (as LabVIEW's Z Enable Joystick).",
                     f"MOV 1 {format_gcs_float(target)}: a {move:.4f} mm move while the joystick is on; #5 and POS? 1 "
                     "until it stands still; POS? 1 and ERR?.",
                     "JAX 1 1 1, JON 1 0, then ERR? and JON? 1: joystick OFF again (as Z Disable Joystick).",
                     "Do NOT touch the joystick during this test (it is on for about one second)."],
                    {"start_mm": pos, "target_mm": target})

    def _run_mov_with_joystick(self, plan: Plan) -> dict[str, Any]:
        start, target = plan.values["start_mm"], plan.values["target_mm"]
        self.joystick_touched = True
        self.stage.jax(JOYSTICK_ID, JOYSTICK_AXIS, AXIS)
        self.stage.jon(JOYSTICK_ID, True)
        err_on, jon_on = self._err(), self.stage.qjon(JOYSTICK_ID)
        result: dict[str, Any] = {"err_jon_on": err_on, "jon_after_on": jon_on}
        if err_on == 0 and jon_on:
            t = self._now()
            self.stage.mov(AXIS, target)
            samples, _ = self.watch(t, self.window, self._timeout(target - start), targets=(target,))
            final, err_mov = self.stage.qpos(AXIS), self._err()
            result.update(samples=self._samples(samples), final_mm=final, err_mov=err_mov)
        self.stage.jax(JOYSTICK_ID, JOYSTICK_AXIS, AXIS)
        self.stage.jon(JOYSTICK_ID, False)
        err_off, jon_off = self._err(), self.stage.qjon(JOYSTICK_ID)
        result.update(err_jon_off=err_off, jon_after_off=jon_off)
        if jon_off:
            raise SafetyStop(f"the joystick could not be switched off (JON? 1 = {jon_off}, ERR? = {err_off})")
        self.joystick_touched = False
        off = f"Joystick switched off again (JON? 1 = 0, ERR? = {_show_err(err_off)})."
        if "err_mov" not in result:
            result.update(status="inconclusive",
                          answer=f"the joystick could not be switched on (ERR? after JON 1 1 = {_show_err(err_on)}, "
                                 f"JON? 1 = {jon_on}), so MOV was not sent. {off}")
            return result
        final, err_mov = result["final_mm"], result["err_mov"]
        moved = abs(final - start) > AT_TARGET_MM
        verdict = (f"the stage MOVED to {final:.4f} mm" if moved else
                   f"MOV was REFUSED: the stage did not move (POS? 1 = {final:.4f} mm)")
        result.update(status="done", moved=moved,
                      answer=f"With the joystick on, {verdict}; ERR? after MOV = {_show_err(err_mov)}. {off}")
        return result

    # ============================================================================================== test 5: VEL 0
    def _plan_vel_zero(self, pos: float) -> Plan:
        speed = format_gcs_float(self.speed)
        return Plan(["VEL? 1, then VEL 1 0.0000 (speed zero; PI's manual says it should not be set to 0), ERR? and "
                     "VEL? 1.",
                     f"Then VEL 1 {speed}, ERR? and VEL? 1 restore the test speed. The stage does not move."])

    def _run_vel_zero(self, plan: Plan) -> dict[str, Any]:
        before = self.stage.qvel(AXIS)
        self.stage.vel(AXIS, 0.0)
        err, after = self._err(), self.stage.qvel(AXIS)
        self.stage.vel(AXIS, self.speed)
        err_restore, restored = self._err(), self.stage.qvel(AXIS)
        result = {"vel_before": before, "err": err, "vel_after": after, "err_restore": err_restore,
                  "vel_restored": restored}
        if not _same_speed(restored, self.speed):
            raise SafetyStop(f"the test speed could not be restored after VEL 0 (VEL? 1 = {restored:.4f} mm/s)")
        changed = "unchanged" if _same_speed(after, before) else "CHANGED"
        result.update(status="done", answer=f"VEL 1 0.0000 gave ERR? = {_show_err(err)}; VEL? 1 afterwards = "
                                            f"{after:.4f} mm/s ({changed}; it was {before:.4f}). Test speed restored "
                                            f"(VEL? 1 = {restored:.4f} mm/s).")
        return result

    # ============================================================================================== test 6: Z Setup x2
    def _setup_settings(self) -> tuple[ZStageSettings, StageOptions]:
        t = self.check.target
        lo, hi = t.zrange or (float(self.check.values["tmn"]), float(self.check.values["tmx"]))
        settings = ZStageSettings(com=t.com_number, baudrate=t.baudrate, axis=1, speed=self.speed,
                                  slow_speed=self.speed, min_pos=lo, max_pos=hi)
        options = StageOptions(ready_timeout_s=SETUP_READY_TIMEOUT_S, port_map={settings.port_name: t.port})
        return settings, options

    def _plan_setup_twice(self, pos: float) -> Plan:
        tmn = float(self.check.values["tmn"])
        return Plan(["Runs the Python server's Z Setup (ZController.setup with the [stage] defaults: reference at",
                     "every set-up, reference move FNL) twice in a row, and times each run. Each run:",
                     "  re-opens the port (XXX, *idn?, err?), *IDN?, SAI?, CST? 1, SVO 1 1, SVO? 1, RON? 1, TRS? 1,",
                     "  then FRF 1 (if TRS? 1 = 1) or FNL 1 (if LIM? 1 = 1), #7 every 50 ms until referencing ends,",
                     f"  FRF? 1, TMN? 1, TMX? 1, POS? 1, ERR?, then VEL 1 {format_gcs_float(self.speed)} and ERR?.",
                     f"FNL 1 DRIVES THE STAGE TO ITS NEGATIVE LIMIT SWITCH (about {tmn:.4f} mm), outside the window.",
                     f"Each run may take a while (the #7 wait stops with #24 after {SETUP_READY_TIMEOUT_S:.0f} s).",
                     "Afterwards the tool offers to move the stage back to its start position."],
                    {"start_mm": pos})

    def _run_setup_twice(self, plan: Plan) -> dict[str, Any]:
        settings, options = self._setup_settings()
        # A plain function, not time.sleep itself: ZController then sleeps with it (Ctrl+C works on Windows)
        # instead of a threading.Event wait.
        zc = ZController(self.stage, options, sleep=lambda s: self.env.sleep(s), monotonic=self.env.monotonic)
        runs = []
        self.after_setup = True                          # the stage leaves the window from here on
        for number in (1, 2):
            self.console.line(f"Z Setup {number} of 2 ...")
            mark = self.check.log.mark()
            t0 = self._now()
            zc.setup(settings)
            duration = self._now() - t0
            frames = self.check.log.since(mark)
            sent = [command_text(f.data) for f in frames if f.direction == TX]
            busy = sum(1 for f in frames if f.direction == RX and f.data == bytes([BUSY_BYTE]))
            ref = [cmd for cmd in sent if cmd.split()[0] in ("FNL", "FRF", "FPL")]
            pos, referenced = self.stage.qpos(AXIS), self.stage.qfrf(AXIS)
            runs.append({"run": number, "duration_s": round(duration, 3), "reference_frames": ref,
                         "ready_busy_answers": busy, "position_after_mm": pos, "frf_after": referenced,
                         "sent": sent})
            self.console.line(f"  done in {duration:.2f} s: reference frames {ref or 'none'}, #7 busy answers {busy}, "
                              f"POS? 1 = {pos:.4f} mm, FRF? 1 = {int(referenced)}")
        each = all(r["reference_frames"] for r in runs)
        answer = ("The Python Z Setup referenced the axis "
                  f"{'at both runs' if each else 'NOT at every run'}: "
                  + "; ".join(f"run {r['run']}: {', '.join(r['reference_frames']) or 'no reference frame'}, "
                              f"{r['duration_s']:.2f} s, #7 busy {r['ready_busy_answers']} times, ended at "
                              f"{r['position_after_mm']:.4f} mm" for r in runs)
                  + ". (Q35 test 6 asks about the LabVIEW server: watch its Z Setup on the machine.)")
        return {"status": "done", "answer": answer, "runs": runs}

    # ============================================================================================== move back
    def _move_back(self) -> None:
        c = self.console
        c.section("Move back to the start position")
        pos, moving = self.stage.qpos(AXIS), self.stage.is_moving()
        back: dict[str, Any] = {"from_mm": pos, "to_mm": self.start}
        self.data["move_back"] = back
        if moving:
            raise SafetyStop("the stage is moving before the move back (HOME macro?)")
        if abs(pos - self.start) <= AT_TARGET_MM:
            back["result"] = f"not needed: the stage is at its start position ({pos:.4f} mm)"
            c.line(f"The stage is at its start position ({pos:.4f} mm): no move needed.")
            return
        path = Window(min(pos, self.start), max(pos, self.start))
        dist = abs(self.start - pos)
        c.line(f"MOV 1 {format_gcs_float(self.start)}: from {pos:.4f} mm back to the start position ({dist:.4f} mm at "
               f"{self.speed:.4f} mm/s, about {dist / self.speed:.1f} s).")
        c.line(f"While it moves the stage must stay inside {path.text()}; otherwise #24 is sent.")
        if c.ask("Type yes to move the stage back to its start position (anything else leaves it here):") != "yes":
            back["result"] = f"declined: the stage stays at {pos:.4f} mm"
            c.line(f"The stage stays at {pos:.4f} mm.")
            return
        self._ensure_speed()
        t = self._now()
        self.stage.mov(AXIS, self.start)
        samples, _ = self.watch(t, path, self._timeout(dist), targets=(self.start,))
        final, err = self.stage.qpos(AXIS), self._err()
        back.update(samples=self._samples(samples), final_mm=final, err=err,
                    result=f"moved back: POS? 1 = {final:.4f} mm (start {self.start:.4f}), ERR? = {_show_err(err)}")
        c.line(f"Back at {final:.4f} mm (ERR? = {_show_err(err)}).")

    # ============================================================================================== end
    def _finish(self, abort: str | None) -> None:
        """Always runs after the tests: #24 if an abnormal end has not sent it yet, then joystick off, original
        speed, DH, final state. A Ctrl+C here skips the rest of the restore."""
        c = self.console
        if abort is not None and self.armed and not self.data["emergency_stops"]:
            self.emergency_stop(abort)
        c.section("End of the motion tests")
        if abort is not None:
            self.data["aborted"] = abort
            self.check.report.problem(f"motion tests stopped: {abort}")
            c.line(f"The motion tests were STOPPED: {abort}")
            for entry in self.data["tests"]:
                if entry.get("status") == "running":
                    entry["status"] = "aborted"
            c.line("After a stop the tool does NOT move the stage back: its position is shown below.")
        steps: list[tuple[str, Callable[[], str]]] = []
        if self.joystick_touched:
            steps.append(("joystick", self._restore_joystick))
        if self.velocity_touched:
            steps.append(("speed", self._restore_speed))
        steps.append(("DH macro", lambda: self._restart_dh(ask=abort is not None)))
        steps.append(("final state", self._final_state))
        for index, (what, step) in enumerate(steps):
            try:
                result = step()
            except KeyboardInterrupt:
                rest = ", ".join(w for w, _ in steps[index:])
                c.line(f"Ctrl+C: the rest of the restore is skipped ({rest}).")
                self.data["restore"].append({"what": what, "result": "skipped (Ctrl+C)"})
                self.check.report.problem(f"restore skipped by Ctrl+C: {rest}")
                break
            except Exception as exc:
                result = f"FAILED: {describe(exc)}"
                c.line(f"Could not restore the {what}: {describe(exc)}")
                self.check.report.problem(f"restore of the {what} failed: {describe(exc)}")
            self.data["restore"].append({"what": what, "result": result})
        if self.joystick_touched:
            c.line("The joystick may still be ON: switch it off with LabVIEW (Z Disable Joystick).")
        restored = self.data["restore"]
        if any(r["what"] == "speed" and not r["result"].startswith("restored") for r in restored):
            c.line(f"The speed may not be restored: set it with LabVIEW (Z Set Speed {self.original_speed:.4f}, or "
                   "Z Setup).")

    def _restore_joystick(self) -> str:
        self.console.line("Switching the joystick off: JAX 1 1 1, JON 1 0, ERR?, JON? 1.")
        self.stage.jax(JOYSTICK_ID, JOYSTICK_AXIS, AXIS)
        self.stage.jon(JOYSTICK_ID, False)
        err, jon = self._err(), self.stage.qjon(JOYSTICK_ID)
        if jon:
            return f"FAILED: JON? 1 = {jon}, ERR? = {err}"
        self.joystick_touched = False
        return f"switched off (JON? 1 = 0, ERR? = {err})"

    def _restore_speed(self) -> str:
        c = self.console
        c.line(f"Restoring the original speed: VEL 1 {format_gcs_float(self.original_speed)}, ERR?, VEL? 1.")
        self.stage.vel(AXIS, self.original_speed)
        err, read = self._err(), self.stage.qvel(AXIS)
        note = f"ERR? = {_show_err(err)}" + (" (10 is left over from the stop)" if err == 10 else "")
        if not _same_speed(read, float(format_gcs_float(self.original_speed))):
            c.line(f"The speed was NOT restored: VEL? 1 = {read:.4f} mm/s, {note}.")
            return f"FAILED: VEL? 1 = {read:.4f} mm/s, {note}"
        c.line(f"Speed restored: VEL? 1 = {read:.4f} mm/s ({note}).")
        return f"restored: VEL? 1 = {read:.4f} mm/s, {note}"

    def _restart_dh(self, *, ask: bool) -> str:
        c = self.console
        if not self.dh_at_start:
            c.line("DH macro: no macro ran at the start, so none is started.")
            return "not needed (no macro ran at the start)"
        rmc = self.stage.qrmc()
        if rmc in KNOWN_MACROS:
            c.line(f"DH macro: still running (RMC? = {rmc!r}), nothing to do.")
            return f"still running (RMC? = {rmc!r})"
        c.line("DH macro: it was running at the start and is stopped now (STP and #24 stop it).")
        c.line("Note: when digital input 1 is high, DH starts HOME, which MOVES THE STAGE TO Z = 0 mm.")
        if ask:
            c.line("The tests were stopped. LabVIEW restarts DH by itself at its next Z Get Status or Z Stop.")
            if c.ask("Type yes to restart DH now (anything else leaves it stopped):") != "yes":
                c.line("DH not restarted.")
                return "not restarted (the operator did not type yes)"
        c.line("Restarting DH, because it was running at the start: MAC START DH, then RMC? and ERR?.")
        self.stage.mac_start("DH")
        rmc_after, err = self.stage.qrmc(), self._err()
        c.line(f"DH: RMC? = {rmc_after!r}, ERR? = {_show_err(err)}.")
        return f"restarted: MAC START DH, RMC? = {rmc_after!r}, ERR? = {err}"

    def _final_state(self) -> str:
        pos, moving, err = self.stage.qpos(AXIS), self.stage.is_moving(), self._err()
        self.data["end_position_mm"] = pos
        self.console.line(f"The stage is at {pos:.4f} mm (start position {self.start:.4f} mm), "
                          f"{'MOVING' if moving else 'standing still'}; ERR? = {_show_err(err)}.")
        return f"POS? 1 = {pos:.4f} mm, #5 = {int(moving)}, ERR? = {err}"


# ---------------------------------------------------------------------------------------------- command line
def _help_epilog() -> str:
    purge = ", ".join(PURGE_FRAMES)
    queries = textwrap.fill(", ".join(step.frame for step in INFO_STEPS), width=100, initial_indent="  then: ",
                            subsequent_indent="        ", break_long_words=False, break_on_hyphens=False)
    return f"""\
information mode (the default) - read-only; exactly these frames are sent:
  opening the port: {purge}  (the purge LabVIEW's PI driver also sends when it opens
    the port: XXX is not a command, the controller only notes error 2, which err? reads and clears)
{queries}
  and, only after a query that gets no usable answer, one ERR? to read the controller's reason.
  ERR? reads AND clears the controller's error (the values read are in the report). Nothing moves the
  stage, changes a setting or stops a macro. Closing the port sends nothing.

motion tests (--motion) - the stage moves:
  POS? during a move, STP 1 during a move, MVR while a move runs, MOV with the joystick on, VEL 0, and
  with --setup-twice the server's Z Setup twice (it references: FNL drives the stage to its negative
  limit switch). Every test is explained first and needs a typed yes. Moves stay inside the window =
  [TMN? 1, TMX? 1] and alox.ini [zmin_DBL, zmax_DBL] and start position +/- --window. Ctrl+C, an error
  or a timeout sends #24 (stop all axes) at once, then the speed is restored. DH is restarted at the end.

examples (Windows, from the repository folder, LabVIEW camera server stopped):
  python tools\\z_stage_check.py --alox-ini D:\\alox\\settings\\alox.ini
  python tools\\z_stage_check.py --port COM4
  python tools\\z_stage_check.py --alox-ini D:\\alox\\settings\\alox.ini --motion
  python tools\\z_stage_check.py --fake --motion --yes            (rehearsal, no hardware)

The report (z_stage_report_<date>-<time>.json and .txt) is written next to this tool, or in --out.
Exit code: 0 done, 1 could not connect / link lost / motion tests stopped, 2 usage error.
"""


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="python tools\\z_stage_check.py",
        description="Check the PI C-663 Z stage controller (RS-232): read-only information, then optional guided "
                    "motion tests. Writes a report to send back.",
        epilog=_help_epilog(), formatter_class=argparse.RawDescriptionHelpFormatter)
    where = p.add_argument_group("controller")
    where.add_argument("--alox-ini", type=Path, metavar="PATH",
                       help="read the port (comport_I32), baud rate and zmin_DBL / zmax_DBL from [hardware.z] of "
                            "this alox.ini (only read, never written)")
    where.add_argument("--port", help="serial port: COM4 (or just 4), /dev/ttyUSB0, or socket://host:port; "
                                      "overrides alox.ini")
    where.add_argument("--baud", type=int, help=f"baud rate (default: alox.ini, else {DEFAULT_BAUD})")
    where.add_argument("--timeout-ms", type=int, default=DEFAULT_TIMEOUT_MS, metavar="MS",
                       help=f"answer timeout (default {DEFAULT_TIMEOUT_MS} ms, as LabVIEW)")
    where.add_argument("--fake", action="store_true",
                       help="rehearsal: an emulated C-663 inside this program (stage at 50 mm, referenced, DH "
                            "running); no port is opened")
    motion = p.add_argument_group("motion tests (THE STAGE MOVES)")
    motion.add_argument("--motion", action="store_true", help="after the information, run the guided motion tests")
    motion.add_argument("--window", type=float, default=DEFAULT_WINDOW_MM, metavar="MM",
                        help=f"moves stay within start position +/- MM (default {DEFAULT_WINDOW_MM:g}; at most "
                             f"{MAX_WINDOW_REAL:g} unless --fake)")
    motion.add_argument("--speed", type=float, default=DEFAULT_SPEED, metavar="MM_S",
                        help=f"test speed in mm/s (default {DEFAULT_SPEED:g}; at most {MAX_SPEED_REAL:g} and never "
                             "above the controller's speed, unless --fake)")
    motion.add_argument("--setup-twice", action="store_true",
                        help="also run the server's Z Setup twice and time it. It REFERENCES THE AXIS: FNL drives "
                             "the stage to its negative limit switch")
    motion.add_argument("--yes", action="store_true", help="answer yes to every question (only with --fake)")
    p.add_argument("--out", type=Path, metavar="PATH",
                   help="folder for the report (default: the folder of this tool), or a .json file name")
    return p


def _final_lines(console: Console, paths: tuple[Path, Path] | None, summary: str) -> None:
    console.line()
    console.line("=" * 78)
    console.line(f"Result: {summary}")
    if paths is None:
        console.line("The report could NOT be written (see the error above).")
    else:
        console.line("Report written:")
        console.line(f"  {paths[0]}")
        console.line(f"  {paths[1]}  (the same report, easier to read)")
        console.line(f"PLEASE SEND THIS FILE BACK: {paths[0]}")
    console.line("=" * 78)


def _write_report(report: Report, out: Path | None, console: Console) -> tuple[Path, Path] | None:
    json_path, txt_path = report_paths(out, report.started)
    candidates = [(json_path, txt_path)]
    if out is None:                                     # tools folder not writable: the current folder
        fallback = report_paths(Path.cwd(), report.started)
        if fallback != candidates[0]:
            candidates.append(fallback)
    for json_path, txt_path in candidates:
        try:
            json_path.parent.mkdir(parents=True, exist_ok=True)
            report.data["send_back"] = str(json_path)
            json_path.write_text(json.dumps(report.data, indent=2, ensure_ascii=False, default=str) + "\n",
                                 encoding="utf-8")
            txt_path.write_text(render_text(report.data, json_path), encoding="utf-8")
            return json_path, txt_path
        except OSError as exc:
            console.line(f"Could not write the report to {json_path.parent}: {exc}")
    return None


def main(argv: Sequence[str] | None = None, *, env: Environment | None = None) -> int:
    env = env or Environment()
    argv = list(sys.argv[1:] if argv is None else argv)
    parser = build_parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit as exc:                           # --help, or an argparse usage error (already printed)
        return int(exc.code) if isinstance(exc.code, int) else EXIT_USAGE
    console = Console(env.output or sys.stdout, env.read_line, auto_yes=bool(args.yes and args.fake))
    try:
        check_args(args)
        target = resolve_target(args)
    except UsageError as exc:
        console.line(f"error: {exc}")
        console.line("Run with --help to see the options.")
        return EXIT_USAGE
    report = Report(argv)
    report.data["options"] = {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(args).items()}
    if target.z is not None:
        z = target.z
        report.data["alox_ini"] = {"path": str(target.alox_ini), "comport_I32": z.com, "baudrate": z.baudrate,
                                   "zmin_DBL": z.min_pos, "zmax_DBL": z.max_pos, "speed_DBL": z.speed,
                                   "warnings": list(target.ini_warnings)}
    server: FakeC663Server | None = None
    check: StageCheck | None = None
    code = EXIT_PROBLEM
    t0 = env.monotonic()
    try:
        if args.fake:
            server = env.fake_server or start_fake_server()
            target = replace(target, port=server.url)
        check = StageCheck(args, target, env, console, report)
        code = check.run()
    except KeyboardInterrupt:
        console.line()
        console.line("Interrupted (Ctrl+C).")
        report.problem("interrupted by Ctrl+C")
        code = EXIT_PROBLEM
    except Exception as exc:
        console.line(f"Unexpected error: {describe(exc)}")
        report.problem(f"unexpected error: {describe(exc)}")
        report.data["traceback"] = traceback.format_exc()
        code = EXIT_PROBLEM
    finally:
        if check is not None:
            check.close()
            report.data["frames"] = check.log.as_records()
        if server is not None and env.fake_server is None:
            server.close()
        report.data["operator_answers"] = console.answers
        report.data["finished"] = datetime.now().astimezone().isoformat(timespec="seconds")
        report.data["duration_s"] = round(env.monotonic() - t0, 3)
        summary = {EXIT_OK: "done", EXIT_PROBLEM: "stopped early or a problem (see Problems)"}.get(code, str(code))
        report.data["result"] = {"exit_code": code, "summary": summary}
        paths = _write_report(report, args.out, console)
        _final_lines(console, paths, summary)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
