"""``PiSerialStage``: the real PI C-663 backend — :class:`StageBackend` over :class:`GcsSerialLink`.

Spec: docs/phase2-design.md §2–§3.2; 08 §3.2 (opening and purge), §3.3 (frame rules, answer parsing, driver pre-checks),
§3.4 (GCS frame catalogue), §3.5 (driver errors), §11 (error texts), §13.2 (driver VI → GCS frames); 08 Z7 (``STP 1``),
Z22 (purge); docs/answers/owner-answers.md §4 (C-663 through a Moxa virtual COM port, 115200 Bd).

Every method sends exactly the frames of the LabVIEW PI driver VI it stands for (08 §3.4) — nothing else: no ``ERR?``
after a command (``ZController`` adds the ``ERR?`` of ``VEL.vi`` / ``JON.vi``), no ``CSV?``, no ``SAI? ALL``. Numbers
use :func:`format_gcs_float` (4 decimals, ``.`` separator); modes are integers (``SVO 1 1``, ``JON 1 0``).

Answers are parsed like ``Assign values from string to axes.vi`` (08 §3.3): split the lines, remove every space, split
``axis=value`` at the first ``=``, pick the value of the requested axis. Any spacing and any Python number syntax is
accepted (``1=12.500000``, ``1 = 12.5``, ``1=1.25E+01``), because the C-663's exact formats are only known once the
lab has seen them (docs/phase2-design.md §6). An empty answer gives
``StageError(100, "<CMD>: Assign values: Input string empty or connected axes not defined")``.

Driver pre-checks (08 §3.3), with nothing sent: an empty axis gives 100 ``"<CMD> No axis specified"`` (per-VI texts);
an axis that is not in the ``SAI?`` list read on the current link gives 100 ``"Commanded axis not connected: <axis>"``.
Before ``SAI?`` has been read on a link (no ``Z Setup`` yet), any axis is accepted. A call without a link raises
``StageError(-4, "<CMD>: not connected")``, as :class:`aloxcam.zstage.sim.SimStage` does.
"""

from __future__ import annotations

import logging
import math
import re
import time
from collections.abc import Callable

from aloxcam.errors import Code, StageError
from aloxcam.zstage.backend import StageBackend, format_gcs_float
from aloxcam.zstage.controller import no_axis_text
from aloxcam.zstage.gcs_errors import gcs_error_text
from aloxcam.zstage.gcs_serial import (
    BUSY_BYTE,
    CHAR_MOVING,
    CHAR_READY,
    CHAR_STOP_ALL,
    READY_BYTE,
    GcsSerialLink,
)

__all__ = [
    "ASSIGN_VALUES_TEXT",
    "NO_ANSWER_ON_ERR",
    "OPEN_PAUSE_S",
    "PURGE_PAUSE_S",
    "PURGE_FRAMES",
    "UNKNOWN_READY_REPLY",
    "LinkFactory",
    "PiSerialStage",
    "axis_value",
    "parse_axis_values",
    "parse_error_code",
]

log = logging.getLogger(__name__)

#: ``Callable[[port, baudrate, timeout_s], GcsSerialLink]``; :class:`GcsSerialLink` itself by default.
LinkFactory = Callable[[str, int, float], GcsSerialLink]

OPEN_PAUSE_S = 0.05                     # VISA Close, wait 50 ms, VISA Open (08 §3.2 step 2)
PURGE_PAUSE_S = 0.01                    # XXX, wait 10 ms, *idn?, err? (08 §3.2 step 3)
PURGE_FRAMES = ("XXX", "*idn?", "err?")  # sent in this spelling by PI Open Interface of one system.vi
ASSIGN_VALUES_TEXT = "Assign values: Input string empty or connected axes not defined"
NO_ANSWER_ON_ERR = "No answer on ERR?"
UNKNOWN_READY_REPLY = "#7 Unknown reply"

_LEADING_INT = re.compile(r"\s*([+-]?\d+)")


# ---------------------------------------------------------------------------------------------- answer parsing
def parse_axis_values(answer: str) -> dict[str, str]:
    """``Assign values from string to axes.vi`` (08 §3.3): ``{axis: value}`` from ``"1=12.5 \\n2=3.0"``; every space
    is removed and each line split at its first ``=``; lines without ``=`` are ignored; the first value of an axis
    wins."""
    values: dict[str, str] = {}
    for line in answer.splitlines():
        text = "".join(line.split())
        key, sep, value = text.partition("=")
        if sep and key:
            values.setdefault(key, value)
    return values


def axis_value(answer: str, axis: str, command: str) -> str:
    """The value of ``axis`` in ``answer``; ``StageError(100, "<command>: Assign values: …")`` when the answer is empty
    or has no value for ``axis``."""
    if not answer.strip():
        raise StageError(Code.PI_DRIVER, f"{command}: {ASSIGN_VALUES_TEXT}")
    values = parse_axis_values(answer)
    if axis not in values:
        raise StageError(Code.PI_DRIVER, f"{command}: {ASSIGN_VALUES_TEXT} (axis {axis} not in the answer {answer!r})")
    return values[axis]


def parse_error_code(answer: str) -> int:
    """The leading integer of an ``ERR?`` answer (LabVIEW ``Decimal String To Number``); 0 when there is none."""
    match = _LEADING_INT.match(answer)
    return int(match.group(1)) if match else 0


def _number(text: str, command: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise StageError(Code.PI_DRIVER, f"{command}: Assign values: {text!r} is not a number") from None
    if math.isnan(value):
        raise StageError(Code.PI_DRIVER, f"{command}: Assign values: {text!r} is not a number")
    return value


# ---------------------------------------------------------------------------------------------- the backend
class PiSerialStage(StageBackend):
    """PI C-663 (GCS 2.0, one axis) over RS-232 with the frames of the LabVIEW PI driver (docs/phase2-design.md §3.2).

    ``link_factory(port, baudrate, timeout_s)`` opens the link (:class:`GcsSerialLink`; tests inject another one);
    ``sleep`` performs the 50 ms / 10 ms pauses of the open sequence. Not thread-safe: ``ZController`` serialises
    every call under its lock. :attr:`link` exposes the open link (its ``frames`` hold the raw bytes exchanged), and
    :meth:`raw_query` / :meth:`raw_send` send any other frame — both for lab tools (``tools/z_stage_check.py``), never
    used by the server.
    """

    def __init__(self, link_factory: LinkFactory = GcsSerialLink, *,
                 sleep: Callable[[float], None] = time.sleep) -> None:
        self._link_factory = link_factory
        self._sleep = sleep
        self._link: GcsSerialLink | None = None
        self._axes: tuple[str, ...] | None = None

    # ============================================================================================== state
    @property
    def link(self) -> GcsSerialLink | None:
        """The open link, or None."""
        return self._link

    @property
    def connected_axes(self) -> tuple[str, ...] | None:
        """The ``SAI?`` answer read on the current link (the driver's connected-axes list), None before ``SAI?``."""
        return self._axes

    @property
    def is_connected(self) -> bool:
        return self._link is not None

    # ============================================================================================== helpers
    def _require(self, command: str) -> GcsSerialLink:
        link = self._link
        if link is None:
            raise StageError(Code.PI_NOT_CONNECTED, f"{command}: not connected")
        return link

    def _check_axis(self, command: str, axis: str) -> None:
        """Driver pre-checks (08 §3.3): nothing is sent when they fail."""
        if not axis:
            raise StageError(Code.PI_DRIVER, no_axis_text(command))
        if self._axes is not None and axis not in self._axes:
            raise StageError(Code.PI_DRIVER, f"Commanded axis not connected: {axis}")

    def _command(self, command: str, axis: str, *values: str) -> None:
        """``"<command> <axis> <values…>"``, no answer."""
        link = self._require(command)
        self._check_axis(command, axis)
        link.send(" ".join((command, axis) + values))

    def _axis_answer(self, command: str, axis: str) -> str:
        """``"<command> <axis>"`` → the value of ``axis`` (spaces removed)."""
        link = self._require(command)
        self._check_axis(command, axis)
        return axis_value(link.query(f"{command} {axis}"), axis, command)

    def _axis_number(self, command: str, axis: str) -> float:
        return _number(self._axis_answer(command, axis), command)

    def _axis_bool(self, command: str, axis: str) -> bool:
        return _number(self._axis_answer(command, axis), command) != 0.0

    # ============================================================================================== link
    def connect_rs232(self, port: str, baudrate: int, timeout_ms: int) -> int:
        """``PI Open Interface of one system.vi`` (08 §3.2): close the open link, wait 50 ms, open ``port``, then the
        purge ``XXX`` / 10 ms / ``*idn?`` (answer discarded) / ``err?``. Returns the code ``err?`` read — on a real
        controller normally 2 (``Unknown command``), latched by the purge's own ``XXX``. ``StageError(-6)`` when the
        port cannot be opened; an error of the purge (e.g. −7, controller off) is raised with the link left open, as
        the VISA session stays open in LabVIEW."""
        self.close()
        self._sleep(OPEN_PAUSE_S)
        link = self._link_factory(str(port), int(baudrate), max(0.0, float(timeout_ms)) / 1000.0)
        self._link = link
        log.info("PI link open: %s at %d Bd (timeout %d ms)", port, int(baudrate), int(timeout_ms))
        xxx, idn, err = PURGE_FRAMES
        link.send(xxx)
        self._sleep(PURGE_PAUSE_S)
        link.query(idn)
        return parse_error_code(link.query(err))

    def close(self) -> bool:
        link, self._link = self._link, None
        self._axes = None
        if link is None:
            return False
        link.close()
        log.info("PI link closed: %s", link.port)
        return True

    # ============================================================================================== general
    def qerr(self) -> int:
        """``ERR?`` → code; an empty answer raises ``StageError(100, "No answer on ERR?")``."""
        answer = self._require("ERR?").query("ERR?")
        if not answer.strip():
            raise StageError(Code.PI_DRIVER, NO_ANSWER_ON_ERR)
        if _LEADING_INT.match(answer) is None:
            log.warning("ERR?: unexpected answer %r read as 0", answer)
        return parse_error_code(answer)

    def qidn(self) -> str:
        """``*IDN?`` → the identification, final LF removed (nothing else)."""
        return self._require("*IDN?").query("*IDN?")

    def qsai(self) -> list[str]:
        """``SAI?`` → the axis ids (one per line); kept as the connected-axes list of the pre-checks."""
        axes = tuple(self._require("SAI?").query("SAI?").split())
        self._axes = axes
        return list(axes)

    def qcst(self, axis: str) -> str:
        return self._axis_answer("CST?", axis)

    def svo(self, axis: str, on: bool) -> None:
        self._command("SVO", axis, str(int(bool(on))))

    def qsvo(self, axis: str) -> bool:
        return self._axis_bool("SVO?", axis)

    def qron(self, axis: str) -> bool:
        return self._axis_bool("RON?", axis)

    def qtrs(self, axis: str) -> bool:
        return self._axis_bool("TRS?", axis)

    def qlim(self, axis: str) -> bool:
        return self._axis_bool("LIM?", axis)

    def qtmn(self, axis: str) -> float:
        return self._axis_number("TMN?", axis)

    def qtmx(self, axis: str) -> float:
        return self._axis_number("TMX?", axis)

    # ============================================================================================== motion
    def mov(self, axis: str, pos: float) -> None:
        self._command("MOV", axis, format_gcs_float(pos))

    def mvr(self, axis: str, dist: float) -> None:
        self._command("MVR", axis, format_gcs_float(dist))

    def vel(self, axis: str, speed: float) -> None:
        self._command("VEL", axis, format_gcs_float(speed))

    def qvel(self, axis: str) -> float:
        return self._axis_number("VEL?", axis)

    def qpos(self, axis: str) -> float:
        return self._axis_number("POS?", axis)

    def qfrf(self, axis: str) -> bool:
        return self._axis_bool("FRF?", axis)

    def frf(self, axis: str) -> None:
        self._command("FRF", axis)

    def fnl(self, axis: str) -> None:
        self._command("FNL", axis)

    def fpl(self, axis: str) -> None:
        self._command("FPL", axis)

    def stp(self, axis: str | None) -> None:
        """``STP 1`` as wired in ALOX (08 Z7); ``None`` → ``STP``."""
        if axis is None:
            self._require("STP").send("STP")
            return
        self._command("STP", axis)

    # ============================================================================================== macros
    def qrmc(self) -> str:
        """``RMC?`` → the raw answer, final LF removed (``""`` when no macro runs; compared exactly, 08 §10)."""
        return self._require("RMC?").query("RMC?")

    def mac_start(self, name: str) -> None:
        link = self._require("MAC START")
        if not name:
            raise StageError(Code.PI_DRIVER, "MAC START no macro name specified")
        link.send(f"MAC START {name}")

    def qmac(self, name: str) -> str:
        """``MAC? <name>`` → the macro's lines joined by LF (the ``" \\n"`` continuation marks become ``"\\n"``)."""
        answer = self._require("MAC?").query(f"MAC? {name}".rstrip())
        return answer.replace(" \n", "\n")

    # ============================================================================================== joystick
    def jax(self, joystick_id: int, joystick_axis: int, axis: str) -> None:
        link = self._require("JAX")
        self._check_axis("JAX", axis)
        link.send(f"JAX {int(joystick_id)} {int(joystick_axis)} {axis}")

    def jon(self, joystick_id: int, enable: bool) -> None:
        self._require("JON").send(f"JON {int(joystick_id)} {int(bool(enable))}")

    def qjon(self, joystick_id: int) -> int:
        key = str(int(joystick_id))
        answer = self._require("JON?").query(f"JON? {key}")
        return int(_number(axis_value(answer, key, "JON?"), "JON?"))

    # ============================================================================================== one-byte commands
    def is_moving(self) -> bool:
        """``#5`` → hexadecimal bit mask (bit *i* = connected axis *i* moving); TRUE when any bit is set."""
        answer = self._require("#5").query_char(CHAR_MOVING)
        try:
            return int(answer.strip(), 16) != 0
        except ValueError:
            raise StageError(Code.PI_DRIVER, f"#5: unexpected answer {answer!r}") from None

    def is_controller_ready(self) -> bool:
        """``#7`` → 0xB1 ready / 0xB0 busy; anything else raises ``StageError(100, "#7 Unknown reply")``."""
        answer = self._require("#7").query_char(CHAR_READY)
        if answer == chr(READY_BYTE):
            return True
        if answer == chr(BUSY_BYTE):
            return False
        log.warning("#7: unknown reply %r", answer)
        raise StageError(Code.PI_DRIVER, UNKNOWN_READY_REPLY)

    def stop_all(self) -> None:
        self._require("#24").send_char(CHAR_STOP_ALL)

    # ============================================================================================== error texts
    def translate_error(self, code: int) -> str:
        return gcs_error_text(code)

    # ============================================================================================== lab tools
    def raw_query(self, cmd: str) -> str:
        """Send any text command and return its raw answer (final LF removed). For lab tools only."""
        words = cmd.split()
        return self._require(words[0] if words else "query").query(cmd)

    def raw_send(self, cmd: str) -> None:
        """Send any text command that has no answer. For lab tools only."""
        words = cmd.split()
        self._require(words[0] if words else "send").send(cmd)
