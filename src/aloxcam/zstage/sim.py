"""``SimStage``: a simulated PI C-663 controller with one axis (``"1"``), at GCS level.

Spec: design §3.5.3; 08 §3.4 (frame catalogue), §3.5 (errors latched until ``ERR?``), §4.4 (referencing), §9 (joystick),
§10 (macros); docs/answers/owner-answers.md §4–§5 (C-663; ``DH`` daemon; ``HOME`` started by ``DH`` on digital input 1;
``STP`` / ``#24`` stop running macros).

The model is deterministic for a given ``clock`` (seconds):

- **Link.** ``connect_rs232`` accepts any port; its purge returns and clears the latched error (08 Z22). Every other
  GCS call on a closed link raises ``StageError(-4, "<CMD>: not connected")``. The controller state (position,
  reference, servo, velocity, joystick, macros, latched error) survives closing and re-opening the link.
- **Errors are latched, not raised** (08 §3.5); only the last one is kept, as on PI controllers; :meth:`qerr`
  returns and clears it. ``MOV`` / ``MVR``: non-finite → 1, joystick on → 51, servo off or axis unreferenced → 5,
  target outside ``[stage_min_mm, stage_max_mm]`` → 7. ``STP`` / ``#24`` → 10; ``VEL`` ≤ 0 → 17 (non-finite → 1);
  ``MAC START`` while a macro runs → 1008, unknown macro → 20; an axis other than ``"1"`` → 15.
- **Motion** is linear at the current velocity from the position at command time; ``MVR`` adds to the last commanded
  target (GCS 2.0, 08 Z5). Numbers pass through the 4-decimal frame format. ``#5`` is TRUE until arrival.
- **Referencing.** ``FNL`` → ``stage_min_mm``, ``FPL`` → ``stage_max_mm``, ``FRF`` → mid-travel, each in
  ``stage_reference_s``; the axis is unreferenced meanwhile and ``#7`` answers "busy". ``RON?``/``LIM?`` TRUE,
  ``TRS?`` FALSE (so set-up uses the limit-switch branch, 08 §4.2 step 12), ``CST?`` ``"SIM-STAGE"``.
- **Macros** (owner-answers §5). ``DH`` runs from power-up (``dh_running``) and restarts itself for ever when
  ``stage_dh_macro_s`` ≤ 0; with ``stage_dh_macro_s`` > 0 it ends that many seconds after its start. ``STP`` / ``#24``
  stop it. :meth:`trigger_home` (digital input 1 high) makes a running ``DH`` start ``HOME``: joystick 1 off, move to
  0 mm, wait on target, then ``DH`` again. ``RMC?`` returns the running macro's name or ``""``.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass

from aloxcam.errors import Code, StageError
from aloxcam.settings import SimulationOptions
from aloxcam.zstage.backend import StageBackend, format_gcs_float
from aloxcam.zstage.gcs_errors import gcs_error_text

__all__ = [
    "DEFAULT_VELOCITY",
    "DH_MACRO_TEXT",
    "FRAME_LOG_SIZE",
    "HOME_MACRO_TEXT",
    "INJECTABLE_METHODS",
    "SIM_AXIS",
    "SIM_STAGE_NAME",
    "SimStage",
]

SIM_AXIS = "1"
SIM_STAGE_NAME = "SIM-STAGE"
DEFAULT_VELOCITY = 10.0                  # mm/s at power-up (the INI "normal speed" default)
FRAME_LOG_SIZE = 4096                    # bounded test log of the frames sent (a 10 ms #5 poll adds 100 per second)

#: Content of the controller macros as read with ``MAC?`` on the real machine (owner-answers §5).
DH_MACRO_TEXT = "\n".join((
    "JRC 5 POS? 1 < 5.05", "DIO 1 1", "DEL 1000", "DIO 1 0", "DEL 2000",
    "JRC 5 POS? 1 > 5.05", "DIO 1 0", "DEL 1000", "DIO 1 1", "DEL 2000",
    "JRC 2 DIO? 1 = 0", "MAC START HOME", "MAC START DH",
))
HOME_MACRO_TEXT = "\n".join(("JAX 1 1 1", "JON 1 0", "MOV 1 0", "WAC ONT? 1 = 1", "MAC START DH"))
_MACROS = {"DH": DH_MACRO_TEXT, "HOME": HOME_MACRO_TEXT}

# Controller error codes latched by the simulation (PI GCS, 08 §11).
_E_SYNTAX = 1
_E_UNREFERENCED = 5
_E_LIMITS = 7
_E_STOPPED = 10
_E_AXIS = 15
_E_PARAM = 17
_E_NO_MACRO = 20
_E_JOYSTICK = 51
_E_MACRO_RUNNING = 1008

_MOVE = "move"
_HOME = "home"
_REFERENCE_KINDS = frozenset({"FNL", "FPL", "FRF"})
_ASSIGN_VALUES = "Assign values: Input string empty or connected axes not defined"   # 08 §3.3

#: StageBackend methods whose next calls :meth:`SimStage.inject_failure` can make fail.
INJECTABLE_METHODS = frozenset(name for name in StageBackend.__abstractmethods__ if name != "is_connected")


@dataclass
class _Failure:
    error: StageError
    remaining: int


@dataclass(frozen=True)
class _Motion:
    """One linear segment: a move, a reference move or the ``HOME`` macro's move."""

    kind: str
    start: float
    target: float
    t0: float
    duration: float

    @property
    def end(self) -> float:
        return self.t0 + self.duration

    def done(self, now: float) -> bool:
        return now >= self.end

    def position(self, now: float) -> float:
        if self.duration <= 0.0 or now >= self.end:
            return self.target
        return self.start + (self.target - self.start) * max(0.0, (now - self.t0) / self.duration)


def _sent(value: float) -> float:
    """The value the controller receives: the number after the 4-decimal frame formatting (08 §3.3)."""
    return float(format_gcs_float(value))


class SimStage(StageBackend):
    """Simulated PI C-663 at GCS level (design §3.5.3). Internally locked, so test hooks may be read from any thread."""

    def __init__(self, options: SimulationOptions, *, clock: Callable[[], float] = time.monotonic,
                 dh_running: bool = True) -> None:
        self._options = options
        self._clock = clock
        self._lock = threading.RLock()
        self._lo, self._hi = sorted((float(options.stage_min_mm), float(options.stage_max_mm)))
        self._dh_at_power_up = dh_running
        self._failures: dict[str, list[_Failure]] = {}
        self._frames: deque[str] = deque(maxlen=FRAME_LOG_SIZE)
        # link
        self._connected = False
        self._port = ""
        self._baudrate = 0
        # controller state (kept across close / re-open, as on the real controller)
        self._error = 0
        self._position = min(max(0.0, self._lo), self._hi)
        self._velocity = DEFAULT_VELOCITY
        self._referenced = bool(options.stage_initially_referenced)
        self._servo = bool(options.stage_initially_referenced)
        self._joystick_axes: dict[int, tuple[int, str]] = {}
        self._joystick_on: dict[int, bool] = {}
        self._motion: _Motion | None = None
        self._macro = ""
        self._macro_end: float | None = None
        if dh_running:
            self._start_macro("DH", self._clock())

    # ============================================================================================== test hooks
    def inject_failure(self, method: str, error: StageError, *, times: int = 1) -> None:
        """Make the next ``times`` calls of the StageBackend method ``method`` raise ``error`` (before anything
        else, including the link check). :class:`ValueError` for an unknown method name or ``times < 1``."""
        if method not in INJECTABLE_METHODS:
            raise ValueError(f"not a StageBackend method: {method!r}")
        if times < 1:
            raise ValueError("times must be >= 1")
        with self._lock:
            self._failures.setdefault(method, []).append(_Failure(error, times))

    @property
    def error_register(self) -> int:
        """The latched controller error (0 = none), without clearing it."""
        with self._lock:
            return self._error

    @property
    def position(self) -> float:
        """The axis position now (mm)."""
        with self._lock:
            now = self._clock()
            self._advance(now)
            return self._current_position(now)

    @property
    def velocity(self) -> float:
        """The current velocity (mm/s)."""
        with self._lock:
            return self._velocity

    @property
    def referenced(self) -> bool:
        """``FRF?`` without a frame."""
        with self._lock:
            self._advance(self._clock())
            return self._referenced

    @property
    def running_macro(self) -> str:
        """``RMC?`` without a frame (``""`` when no macro runs)."""
        with self._lock:
            self._advance(self._clock())
            return self._macro

    @property
    def joystick_enabled(self) -> bool:
        """TRUE when any joystick is on."""
        with self._lock:
            return self._joystick_active()

    @property
    def port(self) -> tuple[str, int]:
        """``(port, baud rate)`` of the last ``connect_rs232``."""
        with self._lock:
            return self._port, self._baudrate

    @property
    def frames(self) -> list[str]:
        """The GCS frames sent on an open link, oldest first, without the LF (bounded to :data:`FRAME_LOG_SIZE`)."""
        with self._lock:
            return list(self._frames)

    def clear_frames(self) -> None:
        """Forget the recorded frames."""
        with self._lock:
            self._frames.clear()

    def trigger_home(self) -> bool:
        """Digital input 1 goes high (e.g. the hardware button): a running ``DH`` starts ``HOME``. Returns whether
        ``HOME`` started (``DH`` must be running, owner-answers §5)."""
        with self._lock:
            now = self._clock()
            self._advance(now)
            if self._macro != "DH":
                return False
            self._start_macro("HOME", now)
            return True

    def power_cycle(self) -> None:
        """Controller reset: axis unreferenced, servo and joystick off, motion stopped, error cleared, velocity back
        to :data:`DEFAULT_VELOCITY`, ``DH`` started again (when it runs at power-up). The link stays as it is."""
        with self._lock:
            now = self._clock()
            self._advance(now)
            self._position = self._current_position(now)
            self._motion = None
            self._referenced = self._servo = False
            self._joystick_on.clear()
            self._joystick_axes.clear()
            self._velocity = DEFAULT_VELOCITY
            self._error = 0
            self._macro, self._macro_end = "", None
            if self._dh_at_power_up:
                self._start_macro("DH", now)

    # ============================================================================================== internals
    def _injected(self, method: str) -> None:
        queue = self._failures.get(method)
        if queue:
            failure = queue[0]
            failure.remaining -= 1
            if failure.remaining <= 0:
                queue.pop(0)
            raise failure.error

    def _enter(self, method: str, frame: str) -> float:
        """Common entry of a GCS call: injected failure, link check, frame log, time advance. Returns now."""
        self._injected(method)
        if not self._connected:
            words = frame.split(" ")
            command = " ".join(words[:2]) if words[0] == "MAC" else words[0]     # "MAC START", "MOV", "#5", …
            raise StageError(Code.PI_NOT_CONNECTED, f"{command}: not connected")
        self._frames.append(frame)
        now = self._clock()
        self._advance(now)
        return now

    def _advance(self, now: float) -> None:
        """Finish the motion and the macro whose time is over."""
        motion = self._motion
        if motion is not None and motion.done(now):
            self._position = motion.target
            self._motion = None
            if motion.kind in _REFERENCE_KINDS:
                self._referenced = True
            elif motion.kind == _HOME and self._macro == "HOME":
                self._start_macro("DH", motion.end)          # HOME: WAC ONT? 1 = 1, then MAC START DH
        if self._macro and self._macro_end is not None and now >= self._macro_end:
            self._macro, self._macro_end = "", None

    def _current_position(self, now: float) -> float:
        return self._motion.position(now) if self._motion is not None else self._position

    def _commanded_target(self, now: float) -> float:
        if self._motion is not None and self._motion.kind in (_MOVE, _HOME):
            return self._motion.target
        return self._current_position(now)

    def _latch(self, code: int) -> None:
        self._error = code                                   # PI keeps only the last error

    def _joystick_active(self) -> bool:
        return any(self._joystick_on.values())

    def _axis_ok(self, axis: str | None) -> bool:
        if axis != SIM_AXIS:
            self._latch(_E_AXIS)
            return False
        return True

    def _query_axis(self, axis: str, command: str) -> None:
        """A query for an unknown axis: the controller answers nothing (error 15), the driver fails (08 §3.3)."""
        if not self._axis_ok(axis):
            raise StageError(Code.PI_DRIVER, f"{command}: {_ASSIGN_VALUES}")

    def _move_error(self, target: float) -> int:
        if not math.isfinite(target):
            return _E_SYNTAX
        if self._joystick_active():
            return _E_JOYSTICK
        if not self._servo or not self._referenced:
            return _E_UNREFERENCED
        if not self._lo <= target <= self._hi:
            return _E_LIMITS
        return 0

    def _start_move(self, target: float, now: float, kind: str = _MOVE) -> None:
        start = self._current_position(now)
        duration = abs(target - start) / self._velocity if self._velocity > 0 else 0.0
        self._motion = _Motion(kind, start, target, now, duration)

    def _stop(self, now: float) -> None:
        """``STP`` / ``#24``: stop the axis where it is, end the running macro, latch 10."""
        self._position = self._current_position(now)
        self._motion = None
        self._macro, self._macro_end = "", None
        self._latch(_E_STOPPED)

    def _start_macro(self, name: str, now: float) -> None:
        if name == "DH":
            lifetime = self._options.stage_dh_macro_s
            self._macro = "DH"
            self._macro_end = now + lifetime if lifetime > 0 else None
            return
        # HOME (owner-answers §5): JAX 1 1 1, JON 1 0, MOV 1 0, WAC ONT? 1 = 1, MAC START DH
        self._joystick_axes[1] = (1, SIM_AXIS)
        self._joystick_on[1] = False
        code = self._move_error(0.0)
        if code:
            self._latch(code)                                # a failing macro line ends the macro
            self._macro, self._macro_end = "", None
            return
        self._macro, self._macro_end = "HOME", None
        self._start_move(0.0, now, _HOME)

    def _reference(self, kind: str, destination: float, now: float) -> None:
        if self._joystick_active():
            self._latch(_E_JOYSTICK)
            return
        if not self._servo:
            self._latch(_E_UNREFERENCED)
            return
        start = self._current_position(now)
        self._referenced = False
        self._motion = _Motion(kind, start, destination, now, max(0.0, float(self._options.stage_reference_s)))

    # ============================================================================================== link
    def connect_rs232(self, port: str, baudrate: int, timeout_ms: int) -> int:
        with self._lock:
            self._connected = False                          # the driver closes the old resource first (08 §3.2)
            self._injected("connect_rs232")
            self._connected = True
            self._port, self._baudrate = port, int(baudrate)
            self._frames.extend(("XXX", "*idn?", "err?"))
            self._advance(self._clock())
            code, self._error = self._error, 0
            return code

    def close(self) -> bool:
        with self._lock:
            self._injected("close")
            was_connected, self._connected = self._connected, False
            return was_connected

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    # ============================================================================================== general
    def qerr(self) -> int:
        with self._lock:
            self._enter("qerr", "ERR?")
            code, self._error = self._error, 0
            return code

    def qidn(self) -> str:
        with self._lock:
            self._enter("qidn", "*IDN?")
            return self._options.stage_identity

    def qsai(self) -> list[str]:
        with self._lock:
            self._enter("qsai", "SAI?")
            return [SIM_AXIS]

    def qcst(self, axis: str) -> str:
        with self._lock:
            self._enter("qcst", f"CST? {axis}")
            self._query_axis(axis, "CST?")
            return SIM_STAGE_NAME

    def svo(self, axis: str, on: bool) -> None:
        with self._lock:
            now = self._enter("svo", f"SVO {axis} {int(bool(on))}")
            if not self._axis_ok(axis):
                return
            self._servo = bool(on)
            if not on and self._motion is not None:
                self._position = self._current_position(now)
                self._motion = None

    def qsvo(self, axis: str) -> bool:
        with self._lock:
            self._enter("qsvo", f"SVO? {axis}")
            self._query_axis(axis, "SVO?")
            return self._servo

    def qron(self, axis: str) -> bool:
        with self._lock:
            self._enter("qron", f"RON? {axis}")
            self._query_axis(axis, "RON?")
            return True

    def qtrs(self, axis: str) -> bool:
        with self._lock:
            self._enter("qtrs", f"TRS? {axis}")
            self._query_axis(axis, "TRS?")
            return False

    def qlim(self, axis: str) -> bool:
        with self._lock:
            self._enter("qlim", f"LIM? {axis}")
            self._query_axis(axis, "LIM?")
            return True

    def qtmn(self, axis: str) -> float:
        with self._lock:
            self._enter("qtmn", f"TMN? {axis}")
            self._query_axis(axis, "TMN?")
            return self._lo

    def qtmx(self, axis: str) -> float:
        with self._lock:
            self._enter("qtmx", f"TMX? {axis}")
            self._query_axis(axis, "TMX?")
            return self._hi

    # ============================================================================================== motion
    def mov(self, axis: str, pos: float) -> None:
        with self._lock:
            now = self._enter("mov", f"MOV {axis} {format_gcs_float(pos)}")
            if not self._axis_ok(axis):
                return
            target = _sent(pos)
            code = self._move_error(target)
            if code:
                self._latch(code)
                return
            self._start_move(target, now)

    def mvr(self, axis: str, dist: float) -> None:
        with self._lock:
            now = self._enter("mvr", f"MVR {axis} {format_gcs_float(dist)}")
            if not self._axis_ok(axis):
                return
            target = self._commanded_target(now) + _sent(dist)
            code = self._move_error(target)
            if code:
                self._latch(code)
                return
            self._start_move(target, now)

    def vel(self, axis: str, speed: float) -> None:
        with self._lock:
            now = self._enter("vel", f"VEL {axis} {format_gcs_float(speed)}")
            if not self._axis_ok(axis):
                return
            value = _sent(speed)
            if not math.isfinite(value):
                self._latch(_E_SYNTAX)
                return
            if value <= 0.0:
                self._latch(_E_PARAM)                        # "The velocity should not be set to 0" (08 §4.4)
                return
            self._velocity = value
            motion = self._motion
            if motion is not None and motion.kind in (_MOVE, _HOME):
                self._start_move(motion.target, now, motion.kind)   # the running move continues at the new speed

    def qvel(self, axis: str) -> float:
        with self._lock:
            self._enter("qvel", f"VEL? {axis}")
            self._query_axis(axis, "VEL?")
            return self._velocity

    def qpos(self, axis: str) -> float:
        with self._lock:
            now = self._enter("qpos", f"POS? {axis}")
            self._query_axis(axis, "POS?")
            return self._current_position(now)

    def qfrf(self, axis: str) -> bool:
        with self._lock:
            self._enter("qfrf", f"FRF? {axis}")
            self._query_axis(axis, "FRF?")
            return self._referenced

    def frf(self, axis: str) -> None:
        with self._lock:
            now = self._enter("frf", f"FRF {axis}")
            if self._axis_ok(axis):
                self._reference("FRF", (self._lo + self._hi) / 2.0, now)

    def fnl(self, axis: str) -> None:
        with self._lock:
            now = self._enter("fnl", f"FNL {axis}")
            if self._axis_ok(axis):
                self._reference("FNL", self._lo, now)

    def fpl(self, axis: str) -> None:
        with self._lock:
            now = self._enter("fpl", f"FPL {axis}")
            if self._axis_ok(axis):
                self._reference("FPL", self._hi, now)

    def stp(self, axis: str | None) -> None:
        with self._lock:
            now = self._enter("stp", "STP" if axis is None else f"STP {axis}")
            if axis is not None and not self._axis_ok(axis):
                return
            self._stop(now)

    # ============================================================================================== macros
    def qrmc(self) -> str:
        with self._lock:
            self._enter("qrmc", "RMC?")
            return self._macro

    def mac_start(self, name: str) -> None:
        with self._lock:
            now = self._enter("mac_start", f"MAC START {name}")
            if self._macro:
                self._latch(_E_MACRO_RUNNING)
                return
            if name not in _MACROS:
                self._latch(_E_NO_MACRO)
                return
            self._start_macro(name, now)

    def qmac(self, name: str) -> str:
        with self._lock:
            self._enter("qmac", f"MAC? {name}")
            text = _MACROS.get(name)
            if text is None:
                self._latch(_E_NO_MACRO)
                return ""
            return text

    # ============================================================================================== joystick
    def jax(self, joystick_id: int, joystick_axis: int, axis: str) -> None:
        with self._lock:
            self._enter("jax", f"JAX {int(joystick_id)} {int(joystick_axis)} {axis}")
            if self._axis_ok(axis):
                self._joystick_axes[int(joystick_id)] = (int(joystick_axis), axis)

    def jon(self, joystick_id: int, enable: bool) -> None:
        with self._lock:
            self._enter("jon", f"JON {int(joystick_id)} {int(bool(enable))}")
            self._joystick_on[int(joystick_id)] = bool(enable)

    def qjon(self, joystick_id: int) -> int:
        with self._lock:
            self._enter("qjon", f"JON? {int(joystick_id)}")
            return 1 if self._joystick_on.get(int(joystick_id), False) else 0

    # ============================================================================================== one-byte commands
    def is_moving(self) -> bool:
        with self._lock:
            self._enter("is_moving", "#5")
            return self._motion is not None

    def is_controller_ready(self) -> bool:
        with self._lock:
            self._enter("is_controller_ready", "#7")
            return not (self._motion is not None and self._motion.kind in _REFERENCE_KINDS)

    def stop_all(self) -> None:
        with self._lock:
            now = self._enter("stop_all", "#24")
            self._stop(now)

    # ============================================================================================== error texts
    def translate_error(self, code: int) -> str:
        with self._lock:
            self._injected("translate_error")
            return gcs_error_text(code)
