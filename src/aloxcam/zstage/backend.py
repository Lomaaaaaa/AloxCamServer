"""``StageBackend``: the PI GCS controller at the level of the LabVIEW PI driver VIs the server uses.

Spec: 08 §3 (controller link, frame rules, GCS frame catalogue, driver errors), §13.2 (LabVIEW driver VI → pipython),
§13.5 (raw transport); design §3.5.1.

One method per driver VI. Contract for every implementation (``SimStage``; ``PiSerialStage``, the real C-663 on raw
pyserial, docs/phase2-design.md §3.2):

- numbers are sent with 4 decimals and a ``.`` separator (:func:`format_gcs_float`, LabVIEW ``%.;%.4f``);
- ``ERR?`` is **never** sent implicitly (pipython ``errcheck = False``): commands refused by the controller leave the
  error latched in the controller until the next explicit :meth:`StageBackend.qerr` (08 §3.5);
- failures raise :class:`aloxcam.errors.StageError` with the codes / texts of 08 §3.5: −4 ``"<CMD>: not connected"``
  when no link is open, −6 when the port cannot be opened, −7 on a read timeout, 100 for the driver checks;
- axis arguments are GCS axis ids (``"1"``); the "No axis specified" pre-checks (08 §3.3) are done by ``ZController``;
- not thread-safe by itself: ``ZController`` serialises every call under its lock (design §2).
"""

from __future__ import annotations

from abc import ABC, abstractmethod

__all__ = ["GCS_FLOAT_FORMAT", "StageBackend", "format_gcs_float"]

#: LabVIEW ``%.;%.4f`` (08 §3.3): 4 decimals, ``.`` separator; pipython ``floatformat = ".4f"`` (08 §13.2).
GCS_FLOAT_FORMAT = ".4f"


def format_gcs_float(value: float) -> str:
    """A number as the PI driver writes it in a frame: ``12.5`` → ``"12.5000"`` (08 §3.3)."""
    return format(float(value), GCS_FLOAT_FORMAT)


class StageBackend(ABC):
    """A PI GCS 2.0 controller (C-663, single axis in ALOX) behind one serial link (08 §3, §13.2)."""

    # ---------------------------------------------------------------------------------------------- link
    @abstractmethod
    def connect_rs232(self, port: str, baudrate: int, timeout_ms: int) -> int:
        """``PI Open Interface of one system.vi`` (RS232 branch, 08 §3.2): close an open link first, open ``port``,
        then the purge ``XXX`` / ``*IDN?`` / ``ERR?``. Returns the code the purge's ``ERR?`` discarded (0 if none).
        Raises ``StageError(-6, …)`` when the port cannot be opened."""

    @abstractmethod
    def close(self) -> bool:
        """``Close connection if open.vi``: returns whether a link was open. Never raises for a closed link."""

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """TRUE while a link is open."""

    # ---------------------------------------------------------------------------------------------- general
    @abstractmethod
    def qerr(self) -> int:
        """``ERR?`` → the controller error code (0 = none); reads **and clears** it (08 §3.5).
        An empty answer raises ``StageError(100, "No answer on ERR?")``."""

    @abstractmethod
    def qidn(self) -> str:
        """``*IDN?`` → identification, final LF stripped (08 §13.2)."""

    @abstractmethod
    def qsai(self) -> list[str]:
        """``SAI?`` → the connected axis ids, e.g. ``["1"]``."""

    @abstractmethod
    def qcst(self, axis: str) -> str:
        """``CST? <axis>`` → stage name (``"NOSTAGE"`` when none is connected)."""

    @abstractmethod
    def svo(self, axis: str, on: bool) -> None:
        """``SVO <axis> 1|0`` (servo mode)."""

    @abstractmethod
    def qsvo(self, axis: str) -> bool:
        """``SVO? <axis>``."""

    @abstractmethod
    def qron(self, axis: str) -> bool:
        """``RON? <axis>`` (reference mode on)."""

    @abstractmethod
    def qtrs(self, axis: str) -> bool:
        """``TRS? <axis>`` (stage has a reference switch)."""

    @abstractmethod
    def qlim(self, axis: str) -> bool:
        """``LIM? <axis>`` (stage has limit switches)."""

    @abstractmethod
    def qtmn(self, axis: str) -> float:
        """``TMN? <axis>`` (travel range minimum, mm)."""

    @abstractmethod
    def qtmx(self, axis: str) -> float:
        """``TMX? <axis>`` (travel range maximum, mm)."""

    # ---------------------------------------------------------------------------------------------- motion
    @abstractmethod
    def mov(self, axis: str, pos: float) -> None:
        """``MOV 1 12.5000``; no ``ERR?`` (08 §3.4)."""

    @abstractmethod
    def mvr(self, axis: str, dist: float) -> None:
        """``MVR 1 -0.2500``; no ``ERR?``."""

    @abstractmethod
    def vel(self, axis: str, speed: float) -> None:
        """``VEL 1 10.0000`` only (``ZController`` adds the ``ERR?`` of ``VEL.vi``)."""

    @abstractmethod
    def qvel(self, axis: str) -> float:
        """``VEL? <axis>`` (mm/s)."""

    @abstractmethod
    def qpos(self, axis: str) -> float:
        """``POS? <axis>`` (mm)."""

    @abstractmethod
    def qfrf(self, axis: str) -> bool:
        """``FRF? <axis>`` (axis referenced)."""

    @abstractmethod
    def frf(self, axis: str) -> None:
        """``FRF <axis>`` (reference move to the reference switch); end detected with ``#7``."""

    @abstractmethod
    def fnl(self, axis: str) -> None:
        """``FNL <axis>`` (reference move to the negative limit switch)."""

    @abstractmethod
    def fpl(self, axis: str) -> None:
        """``FPL <axis>`` (reference move to the positive limit switch)."""

    @abstractmethod
    def stp(self, axis: str | None) -> None:
        """``STP 1`` as wired in ALOX (08 §5.3, Z7); ``None`` → ``STP``. The controller latches error 10 and stops
        running macros (owner-answers §5)."""

    # ---------------------------------------------------------------------------------------------- macros
    @abstractmethod
    def qrmc(self) -> str:
        """``RMC?`` → running macro names, final LF stripped (``""`` when none)."""

    @abstractmethod
    def mac_start(self, name: str) -> None:
        """``MAC START <name>``; no ``ERR?`` (the controller latches 1008 when a macro already runs)."""

    @abstractmethod
    def qmac(self, name: str) -> str:
        """``MAC? <name>`` → the macro's content (08 §10, pipython ``qMAC``)."""

    # ---------------------------------------------------------------------------------------------- joystick
    @abstractmethod
    def jax(self, joystick_id: int, joystick_axis: int, axis: str) -> None:
        """``JAX <joystick> <joystick axis> <axis>``, e.g. ``JAX 1 1 1``."""

    @abstractmethod
    def jon(self, joystick_id: int, enable: bool) -> None:
        """``JON 1 1`` / ``JON 1 0``; no ``ERR?`` (``ZController`` adds it, as ``JON.vi`` does)."""

    @abstractmethod
    def qjon(self, joystick_id: int) -> int:
        """``JON? <joystick>`` → the joystick's mode (0 = off)."""

    # ---------------------------------------------------------------------------------------------- one-byte commands
    @abstractmethod
    def is_moving(self) -> bool:
        """``#5`` (0x05, no LF): TRUE when any connected axis is moving (08 §3.4, §8)."""

    @abstractmethod
    def is_controller_ready(self) -> bool:
        """``#7`` (0x07): TRUE on 0xB1 (ready), FALSE on 0xB0 (busy, e.g. referencing); any other answer raises
        ``StageError(100, "#7 Unknown reply")``."""

    @abstractmethod
    def stop_all(self) -> None:
        """``#24`` (0x18): stop all motion (and macros); the controller latches error 10."""

    # ---------------------------------------------------------------------------------------------- error texts
    @abstractmethod
    def translate_error(self, code: int) -> str:
        """``GCSTranslateError`` description of ``code``; ``""`` when unknown (08 §11). Needs no link."""
