"""``ZController``: the single, locked link to the PI controller, its live session, set-up, reference check and waits.

Spec: 08 §2.2 (``Z Session``), §3.3 (driver pre-checks), §4.1–§4.3 (``Z Setup`` / ``Mercury_GCS_Configuration_Setup``),
§6 (``alox_z_check_axis_reference.vi``), §8 (``General wait for movement to stop``), §10 (macros), §11 (error
translation), §12 (one locked link, Z18), §13.3 (reference sketch ``PiStage``); 08 Z13, Z19, Z22; 01 §3.4.3 / 08 §2.4
(start-up macro check); design §2 (thread contract), §3.5.4.

Thread contract (design §2): every GCS request/answer — and the ``VEL`` + ``ERR?`` and ``JON`` + ``ERR?`` pairs — runs
under :attr:`ZController.lock` (an RLock); the waits take it per exchange and never across a sleep, except inside
:meth:`ZController.setup`, which holds it for the whole sequence. :meth:`ZController.cancel_waits` needs no lock.
``ZController`` never calls ``ServerState``, a ``ReplySink`` or an ``EventSink``. Phase 4 focus code uses the same
instance and the same live session (08 Z18 fix).
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

from aloxcam.errors import Code, StageError
from aloxcam.settings import StageOptions, ZStageSettings
from aloxcam.zstage.backend import StageBackend

__all__ = [
    "DH_MACRO",
    "KNOWN_MACROS",
    "READY_POLL_S",
    "SETUP_TIMEOUT_MS",
    "ZController",
    "ZSession",
    "serial_from_idn",
]

log = logging.getLogger(__name__)

DH_MACRO = "DH"
KNOWN_MACROS = ("DH", "HOME")            # RMC? answers counted as "macro running" (exact compare, 08 §10)
SETUP_TIMEOUT_MS = 5000                  # VISA timeout wired by ALOX (08 §3.1)
READY_POLL_S = 0.05                      # Wait for controller ready.vi: #7 every 50 ms (08 §4.2)

_NO_AXIS_TEXT = {                        # per-VI texts of the "No axis specified" pre-check (08 §3.3)
    "MOV": "MOV No axis specified",
    "MVR": "MVR no axis specified",
    "VEL": "VEL No axis or velocity specified",
    "STP": "STP no axis specified",
}
_WAIT_READY = "Wait for controller ready"
_WAIT_AXES = "Wait for axes to stop"


@dataclass(frozen=True)
class ZSession:
    """``alox_typedef_ext_zstage_session.ctl`` (08 §2.2); one live copy, owned by :class:`ZController` (08 Z18)."""

    system: int = 0
    axes: tuple[str, ...] = ()
    identity: str = ""
    serial_number: str = ""


def serial_from_idn(identity: str) -> str:
    """The 3rd comma-separated field of ``*IDN?``, stripped; ``""`` when absent (08 §13.3, Z21: inferred source of
    ``SN out``)."""
    parts = [p.strip() for p in identity.split(",")]
    return parts[2] if len(parts) > 2 else ""


class ZController:
    """The single link to the controller. Thread-safe: every GCS request/answer (and VEL+ERR?, JON+ERR? pairs) runs
    under ``lock``; set-up holds it for the whole sequence. Phase 4 focus code uses the same instance."""

    lock: threading.RLock

    def __init__(self, backend: StageBackend, options: StageOptions, *,
                 sleep: Callable[[float], None] = time.sleep,
                 monotonic: Callable[[], float] = time.monotonic) -> None:
        self.lock = threading.RLock()
        self._backend = backend
        self._options = options
        self._sleep = sleep
        self._monotonic = monotonic
        self._cancelled = threading.Event()
        self._session = ZSession()

    # ============================================================================================== state
    @property
    def session(self) -> ZSession:
        """The live session (immutable value; replaced as a whole)."""
        return self._session

    @property
    def options(self) -> StageOptions:
        """The ``[stage]`` options this controller was built with."""
        return self._options

    def reset_session(self) -> None:
        """``Z Cleanup``: the session back to its default ``{0, [], "", ""}`` (08 §5.3)."""
        with self.lock:
            self._session = ZSession()

    def close(self) -> bool:
        """``Close connection if open``; waits for a running exchange (under ``lock``); idempotent. Returns whether a
        link was open."""
        with self.lock:
            was_connected = self._backend.close()
        if was_connected:
            log.info("Z stage link closed")
        return was_connected

    def cancel_waits(self) -> None:
        """Shutdown only (``RuntimeParts.abort_waits``): from now on :meth:`wait_ready`, :meth:`general_wait` and
        :meth:`setup` raise ``StageError(9510, "<what>: aborted by server shutdown")`` at their next poll. No lock."""
        self._cancelled.set()
        log.warning("Z stage waits cancelled by the server shutdown")

    # ============================================================================================== helpers
    def _axis(self, command: str) -> str:
        """First connected axis; the driver VIs send nothing without one (08 §3.3: error 100)."""
        axes = self._session.axes
        if not axes:
            raise StageError(Code.PI_DRIVER, _NO_AXIS_TEXT.get(command, f"{command} No axis specified"))
        return axes[0]

    @staticmethod
    def _aborted(what: str) -> StageError:
        return StageError(Code.ABORTED_BY_SHUTDOWN, f"{what}: aborted by server shutdown")

    def _check_cancel(self, what: str) -> None:
        if self._cancelled.is_set():
            raise self._aborted(what)

    def _pause(self, seconds: float, what: str) -> None:
        """Sleep ``seconds``; with the default ``time.sleep`` the sleep is a ``threading.Event`` wait that
        :meth:`cancel_waits` interrupts. An injected ``sleep`` (tests) is called as is."""
        self._check_cancel(what)
        seconds = max(0.0, seconds)
        if self._sleep is time.sleep:
            if self._cancelled.wait(seconds):
                raise self._aborted(what)
        else:
            self._sleep(seconds)

    # ============================================================================================== error handling
    def check_error(self) -> None:
        """``ERR?.vi``: reads **and clears** the controller error; a non-zero code raises
        ``StageError(code, "Controller error")`` (08 §3.4–§3.5)."""
        with self.lock:
            code = self._backend.qerr()
        if code:
            raise StageError(code, "Controller error")

    def gcs_error_text(self, code: int) -> str:
        """``GCSTranslateError`` description of ``code``; ``""`` when unknown (08 §11)."""
        with self.lock:
            return self._backend.translate_error(code)

    def translate(self, err: StageError) -> StageError:
        """``GCSTranslateError.vi``: a new error with ``"\\nGCS Error: <text>"`` appended to the source when the code
        is known; ``err`` itself otherwise (08 §11, §13.3)."""
        text = self.gcs_error_text(err.code)
        return StageError(err.code, f"{err.source}\nGCS Error: {text}") if text else err

    # ============================================================================================== GCS commands
    def mov(self, pos: float) -> None:
        """``MOV 1 <pos>`` (no ``ERR?``, 08 §3.4)."""
        with self.lock:
            self._backend.mov(self._axis("MOV"), float(pos))

    def mvr(self, dist: float) -> None:
        """``MVR 1 <dist>`` (no ``ERR?``)."""
        with self.lock:
            self._backend.mvr(self._axis("MVR"), float(dist))

    def vel(self, speed: float) -> None:
        """``VEL.vi``: ``VEL 1 <speed>`` then ``ERR?`` (an older pending error is reported here, 08 §5.3)."""
        with self.lock:
            self._backend.vel(self._axis("VEL"), float(speed))
            self.check_error()

    def qvel(self) -> float:
        """``VEL? 1`` (mm/s)."""
        with self.lock:
            return float(self._backend.qvel(self._axis("VEL?")))

    def qpos(self) -> float:
        """``POS? 1`` (mm)."""
        with self.lock:
            return float(self._backend.qpos(self._axis("POS?")))

    def qfrf(self) -> bool:
        """``FRF? 1`` (TRUE = referenced)."""
        with self.lock:
            return bool(self._backend.qfrf(self._axis("FRF?")))

    def fnl(self) -> None:
        """``FNL 1`` (reference move to the negative limit)."""
        with self.lock:
            self._backend.fnl(self._axis("FNL"))

    def frf(self) -> None:
        """``FRF 1`` (reference move to the reference switch)."""
        with self.lock:
            self._backend.frf(self._axis("FRF"))

    def stp(self) -> None:
        """``STP 1`` as wired in ALOX (08 §5.3, Z7); the controller latches error 10 and stops its macros."""
        with self.lock:
            self._backend.stp(self._axis("STP"))

    def running_macros(self) -> str:
        """``RMC?`` (final LF stripped; ``""`` when no macro runs)."""
        with self.lock:
            return self._backend.qrmc()

    def mac_start(self, name: str) -> None:
        """``MAC START <name>`` (no ``ERR?``); an empty name fails like the driver (08 §3.3)."""
        if not name:
            raise StageError(Code.PI_DRIVER, "MAC START no macro name specified")
        with self.lock:
            self._backend.mac_start(name)

    def jax(self, joystick_id: int, joystick_axis: int) -> None:
        """``JAX <id> <joystick axis> 1`` (08 §9)."""
        with self.lock:
            self._backend.jax(int(joystick_id), int(joystick_axis), self._axis("JAX"))

    def jon(self, joystick_id: int, enable: bool) -> None:
        """``JON.vi``: ``JON <id> 1|0`` then ``ERR?`` (08 §9)."""
        with self.lock:
            self._backend.jon(int(joystick_id), bool(enable))
            self.check_error()

    def qjon(self, joystick_id: int) -> bool:
        """``JON? <id>`` → value ≠ 0 (08 §9)."""
        with self.lock:
            return self._backend.qjon(int(joystick_id)) != 0

    def is_moving(self) -> bool:
        """``#5``: TRUE while an axis moves."""
        with self.lock:
            return bool(self._backend.is_moving())

    # ============================================================================================== waits
    def wait_ready(self, timeout_s: float | None = None) -> None:
        """``Wait for controller ready.vi``: ``#7`` every 50 ms until ready (08 §4.2, Z13); ``StageError(-7)`` after
        ``timeout_s`` (default ``[stage] ready_timeout_s``); 9510 once :meth:`cancel_waits` was called."""
        timeout = self._options.ready_timeout_s if timeout_s is None else timeout_s
        t0 = self._monotonic()
        while True:
            self._check_cancel(_WAIT_READY)
            with self.lock:
                ready = self._backend.is_controller_ready()
            if ready:
                return
            if self._monotonic() - t0 > timeout:
                raise StageError(Code.PI_TIMEOUT, f"{_WAIT_READY}: Timeout.")
            self._pause(READY_POLL_S, _WAIT_READY)

    def general_wait(self, add_wait_ms: int = 0, polling_ms: int = 10, add_wait_only: bool = False,
                     timeout_s: float | None = None) -> None:
        """``General wait for movement to stop.vi`` (Mercury_GCS branch, 08 §8): unless ``add_wait_only``, poll ``#5``
        every ``polling_ms`` until no axis moves (``StageError(-7, "Wait for axes to stop.vi: Timeout.")`` after
        ``timeout_s``, default ``[stage] wait_timeout_s``); then wait ``add_wait_ms`` — also after a failed poll, but
        not after a cancellation (LabVIEW ``Stopped``). Negative times count as 0."""
        timeout = self._options.wait_timeout_s if timeout_s is None else timeout_s
        self._check_cancel(_WAIT_AXES)
        error: StageError | None = None
        if not add_wait_only:
            try:
                self._wait_axes_stopped(max(0, polling_ms) / 1000.0, timeout)
            except StageError as exc:
                if exc.code == Code.ABORTED_BY_SHUTDOWN:
                    raise
                error = exc
        if add_wait_ms > 0:
            self._pause(add_wait_ms / 1000.0, _WAIT_AXES)
        if error is not None:
            raise error

    def _wait_axes_stopped(self, poll_s: float, timeout: float) -> None:
        """``Wait for axes to stop.vi`` (08 §8): ``#5`` until no axis moves."""
        t0 = self._monotonic()
        while True:
            self._check_cancel(_WAIT_AXES)
            with self.lock:
                moving = self._backend.is_moving()
            if not moving:
                return
            if self._monotonic() - t0 > timeout:
                raise StageError(Code.PI_TIMEOUT, "Wait for axes to stop.vi: Timeout.")
            self._pause(poll_s, _WAIT_AXES)

    # ============================================================================================== set-up
    def setup(self, s: ZStageSettings) -> None:
        """``Z Setup``: ``Mercury_GCS_Configuration_Setup.vi`` (08 §4.2 steps 1–19, §13.3) then ``VEL <speed>`` +
        ``ERR?`` ("Set Normal Speed"). Holds ``lock`` for the whole sequence. The session is written even on failure
        (as far as it was read); an error of steps 1–18 is translated (step 19), the ``VEL`` error is not."""
        with self.lock:
            self._check_cancel("Z Setup")
            self._session = ZSession(system=s.axis)
            try:
                self._configuration_setup(s)
            except StageError as exc:
                translated = self.translate(exc)
                if translated is exc:
                    raise
                raise translated from exc
            self.vel(s.speed)

    def _configuration_setup(self, s: ZStageSettings) -> None:
        backend = self._backend
        port = self._options.port_map.get(s.port_name, s.port_name)
        log.info("Z Setup: opening %s at %d Bd (system %d)", port, s.baudrate, s.axis)
        discarded = backend.connect_rs232(port, s.baudrate, SETUP_TIMEOUT_MS)           # step 1 (purge)
        if discarded:
            log.warning("Z Setup: controller error %d (%s) discarded by the connect purge (08 Z22)",
                        discarded, backend.translate_error(discarded) or "unknown code")
        identity = backend.qidn()                                                       # step 2
        serial = serial_from_idn(identity)
        self._session = ZSession(s.axis, (), identity, serial)
        axes = tuple(str(a) for a in backend.qsai())                                    # step 4
        self._session = ZSession(s.axis, axes, identity, serial)
        axis = self._axis("CST?")
        if backend.qcst(axis).strip().upper() != "NOSTAGE":                             # step 5 (inferred test)
            backend.svo(axis, True)                                                     # steps 6-7
            backend.qsvo(axis)
            if backend.qron(axis):                                                      # step 9
                if self._options.always_reference or not backend.qfrf(axis):            # hidden "Check FRF?"
                    if backend.qtrs(axis):                                              # steps 10-11
                        backend.frf(axis)
                    elif backend.qlim(axis):                                            # steps 12-13
                        self._reference_move(axis)
                    self.wait_ready()                                                   # #7 polling (inferred)
                    backend.qfrf(axis)
            backend.qtmn(axis)                                                          # step 14 (unused)
            backend.qtmx(axis)
            backend.qpos(axis)                                                          # step 17
        self.check_error()                                                              # step 18

    def _reference_move(self, axis: str) -> None:
        move = self._options.reference_move.strip().upper()
        if move == "FPL":
            self._backend.fpl(axis)
            return
        if move != "FNL":
            log.warning("[stage] reference_move = %r is not FNL or FPL: FNL used", self._options.reference_move)
        self._backend.fnl(axis)

    # ============================================================================================== reference check
    def check_axis_reference(self, s: ZStageSettings) -> tuple[bool, bool, StageError | None]:
        """``alox_z_check_axis_reference.vi`` (08 §6): returns ``(Z INITED, REF WARNING?, error)``. ``FRF?`` TRUE →
        ``(True, False, None)``. A FALSE or failed ``FRF?`` closes the link and runs the whole :meth:`setup` (possibly a
        reference move): ``(True, True, None)`` or ``(False, True, error)``. Incoming errors are discarded."""
        with self.lock:
            try:
                referenced = self.qfrf()
            except StageError as exc:
                log.info("Z reference check: FRF? failed (%s): re-initialising", exc)
                referenced = False
            if referenced:
                return True, False, None
            log.warning("Z reference check: axis not referenced: closing and re-initialising the link (08 §6)")
            try:
                self.close()                                                            # "Close Z"; errors cleared
            except StageError as exc:
                log.debug("Z reference check: close failed (cleared): %s", exc)
            try:
                self.setup(s)                                                           # "Reinit Z" + normal speed
            except StageError as exc:
                return False, True, exc
            return True, True, None

    # ============================================================================================== start-up
    def startup_macro_check(self) -> None:
        """Start-up macro check (08 §2.4, 01 §3.4.3), only with ``[stage] startup_macro_check`` and an open link:
        ``RMC?``; when it answers ``"DH"`` or ``"HOME"``, ``MAC START DH`` (LabVIEW's reverse logic). Errors are
        discarded (``Clear Errors``)."""
        if not self._options.startup_macro_check:
            return
        with self.lock:
            if not self._backend.is_connected:
                return
            try:
                running = self._backend.qrmc()
                if running in KNOWN_MACROS:
                    self._backend.mac_start(DH_MACRO)
            except StageError as exc:
                log.debug("start-up macro check failed (cleared): %s", exc)
