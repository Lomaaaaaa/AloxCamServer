"""``GcsSerialLink``: the raw PI GCS 2.0 transport (RS-232, or a ``socket://`` URL) on pyserial.

Spec: docs/phase2-design.md §2 (raw pyserial, not pipython), §3.1; 08 §3.1 (serial parameters), §3.2 (opening: input
buffer cleared), §3.3 (frame rules), §3.5 (driver errors), §13.5 (raw transport sketch).

Frame rules (08 §3.3), reproduced from ``PI Send String.vi`` / ``PI Receive String.vi``:

- a text command is sent with an LF appended; the single-byte commands ``#5`` (0x05), ``#7`` (0x07) and ``#24`` (0x18)
  are sent alone, without LF (:meth:`GcsSerialLink.send_char`);
- an answer is read line by line until a line that does **not** end with ``" \\n"`` (GCS multi-line convention); only
  the final LF is removed, every other byte (including the continuation spaces) is kept;
- ``#7`` answers one byte (0xB1 ready, 0xB0 busy) and no LF: :meth:`GcsSerialLink.query_char` reads exactly one byte
  for it instead of a line.

Ports: anything ``serial.serial_for_url`` opens — ``"COM4"``, ``"/dev/ttyUSB0"``, ``"socket://127.0.0.1:5555"`` (the
:class:`aloxcam.zstage.fake_c663.FakeC663Server`, docs/phase2-design.md §3.3) — at 8 data bits, no parity, 1 stop bit,
no handshake, opened exclusively. pyserial is imported lazily: without it the link cannot be opened
(``StageError(-6, PYSERIAL_MISSING)``) but the rest of the server imports and runs.

Errors (08 §3.5): −6 when the port cannot be opened; −7 ``"Data Receive Timeout (RS-232, <port>, baud <b>)"`` when no
complete answer arrives within ``timeout_s`` (per line, like the VISA timeout); −2 ``"Data Send Error (RS-232, <port>,
baudrate <b>): …"`` when a write fails; −3 ``"Data Receive Error (RS-232, <port>, baud <b>): …"`` when a read fails
(for example the ``socket://`` peer closed the connection); −4 on a closed link.

Robustness (Python addition, no frame changes): before each command, bytes already waiting in the input buffer are
read and discarded (an answer that arrived after a −7 timeout would otherwise be taken for the answer of the next
query). A lone LF right after a ``#7`` answer is dropped silently (in case the real controller terminates that byte
with LF); anything else is logged at WARNING. Discarded bytes appear in :attr:`GcsSerialLink.frames` as ``"drop"``.

Every frame is logged at DEBUG on the ``aloxcam.zstage.serial`` logger. The link has no lock: it is used only under
``ZController.lock`` (phase 1 thread contract).
"""

from __future__ import annotations

import collections
import logging
import socket
from typing import Any

from aloxcam.errors import Code, StageError

__all__ = [
    "BUSY_BYTE",
    "CHAR_MOVING",
    "CHAR_READY",
    "CHAR_STOP_ALL",
    "DEFAULT_ENCODING",
    "DROPPED",
    "FRAME_LOG_SIZE",
    "PYSERIAL_MISSING",
    "READY_BYTE",
    "RX",
    "SERIAL_LOGGER",
    "SINGLE_BYTE_COMMANDS",
    "TX",
    "GcsSerialLink",
    "import_pyserial",
    "pyserial_version",
]

SERIAL_LOGGER = "aloxcam.zstage.serial"
log = logging.getLogger(SERIAL_LOGGER)

FRAME_LOG_SIZE = 512                    # (direction, bytes) pairs kept in GcsSerialLink.frames
DEFAULT_ENCODING = "cp1252"             # 08 §3.1: single-byte text; #7 answers 0xB0 / 0xB1
CHAR_MOVING = 0x05                      # #5: motion status, answers a hexadecimal bit mask + LF
CHAR_READY = 0x07                       # #7: controller ready, answers one byte without LF
CHAR_STOP_ALL = 0x18                    # #24: stop all axes (and macros), no answer
SINGLE_BYTE_COMMANDS = frozenset({CHAR_MOVING, CHAR_READY, CHAR_STOP_ALL})
READY_BYTE = 0xB1                       # #7 answer: ready
BUSY_BYTE = 0xB0                        # #7 answer: busy (e.g. referencing)
TX, RX, DROPPED = "tx", "rx", "drop"    # GcsSerialLink.frames directions
PYSERIAL_MISSING = "pyserial is not installed: pip install pyserial (or pip install aloxcam[stage])"

_LF = b"\n"
_CONTINUATION = b" \n"
_MAX_DISCARD = 65536                    # bound of one input-buffer drain
_ARROWS = {TX: ">", RX: "<", DROPPED: "x"}


def import_pyserial() -> Any:
    """The ``serial`` module of pyserial, imported on first use (docs/phase2-design.md §3.4). Raises
    ``StageError(-6, PYSERIAL_MISSING)`` when it is missing, or when another package named ``serial`` (not pyserial)
    is installed."""
    try:
        import serial
    except ImportError:
        raise StageError(Code.PI_OPEN_PORT, PYSERIAL_MISSING) from None
    if not hasattr(serial, "serial_for_url"):
        raise StageError(Code.PI_OPEN_PORT, f"{PYSERIAL_MISSING} (the installed 'serial' module is not pyserial)")
    return serial


def _no_nagle(ser: Any) -> None:
    """``socket://`` links (pyserial's socket transport keeps its socket in ``_socket``): send each frame at once, as a
    serial line does, instead of holding a short frame until the previous one is acknowledged (Nagle)."""
    sock = getattr(ser, "_socket", None)
    if sock is None:
        return
    try:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except (OSError, AttributeError) as exc:
        log.debug("TCP_NODELAY not set: %s", exc)


def pyserial_version() -> str | None:
    """pyserial's version text, or None when pyserial cannot be imported (``--check-config``)."""
    try:
        serial = import_pyserial()
    except StageError:
        return None
    return str(getattr(serial, "__version__", "unknown version"))


class GcsSerialLink:
    """One open serial link to a GCS controller (docs/phase2-design.md §3.1; 08 §3.1–§3.3, §13.5).

    Opening clears the input buffer. :attr:`frames` keeps the last :data:`FRAME_LOG_SIZE` ``(direction, bytes)``
    pairs: ``"tx"`` for every frame written, ``"rx"`` for every answer read (all the lines of a multi-line answer in
    one entry, or the partial bytes of a timed-out read), ``"drop"`` for discarded input.
    """

    port: str
    baudrate: int
    timeout_s: float
    encoding: str
    frames: collections.deque[tuple[str, bytes]]

    def __init__(self, port: str, baudrate: int, timeout_s: float, *, encoding: str = DEFAULT_ENCODING) -> None:
        self.port = str(port)
        self.baudrate = int(baudrate)
        self.timeout_s = max(0.0, float(timeout_s))
        self.encoding = encoding
        self.frames = collections.deque(maxlen=FRAME_LOG_SIZE)
        self._after_ready_byte = False
        self._serial: Any = None
        serial = import_pyserial()
        try:
            self._serial = serial.serial_for_url(
                self.port, baudrate=self.baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, xonxoff=False, rtscts=False, dsrdtr=False, timeout=self.timeout_s,
                write_timeout=self.timeout_s, exclusive=True)
        except (OSError, ValueError) as exc:         # serial.SerialException is an OSError
            raise StageError(Code.PI_OPEN_PORT, f"Open RS232 {self.port} @ {self.baudrate}: {exc}") from exc
        try:
            self._serial.reset_input_buffer()        # "read and discard the bytes already waiting" (08 §3.2)
        except (OSError, ValueError) as exc:
            self.close()
            raise StageError(Code.PI_OPEN_PORT, f"Open RS232 {self.port} @ {self.baudrate}: {exc}") from exc
        _no_nagle(self._serial)
        log.debug("%s: opened at %d Bd, 8N1, no handshake, timeout %.3f s", self.port, self.baudrate, self.timeout_s)

    # ============================================================================================== state
    @property
    def is_open(self) -> bool:
        """TRUE until :meth:`close`."""
        return self._serial is not None

    def close(self) -> None:
        """Close the port; idempotent, never raises."""
        ser, self._serial = self._serial, None
        if ser is None:
            return
        try:
            ser.close()
        except Exception as exc:                     # a port that disappeared: nothing left to release
            log.debug("%s: close failed: %s", self.port, exc)
        log.debug("%s: closed", self.port)

    def __enter__(self) -> GcsSerialLink:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # ============================================================================================== frames
    def send(self, cmd: str) -> None:
        """A text command with LF appended (``PI Send String.vi``)."""
        self._write((cmd + "\n").encode(self.encoding, errors="replace"))

    def send_char(self, code: int) -> None:
        """A single-byte command (``#5``, ``#7``, ``#24``): that byte alone, no LF."""
        if not 0 <= int(code) <= 0xFF:
            raise ValueError(f"not a byte: {code!r}")
        self._write(bytes([int(code)]))

    def receive(self) -> str:
        """One answer (``PI Receive String.vi``): lines until one that does not end with ``" \\n"``; the final LF is
        removed, nothing else. ``StageError(-7, "Data Receive Timeout (RS-232, <port>, baud <b>)")`` when a line does
        not complete within ``timeout_s``."""
        ser = self._require("receive")
        data = bytearray()
        while True:
            try:
                line = ser.read_until(_LF)
            except (OSError, ValueError) as exc:
                self._record_partial(data)
                raise self._receive_error(exc) from exc
            data += line
            if not line.endswith(_LF):
                self._record_partial(data)
                raise self._timeout()
            if not line.endswith(_CONTINUATION):
                break
        self._record(RX, bytes(data))
        return bytes(data[:-1]).decode(self.encoding, errors="replace")

    def query(self, cmd: str) -> str:
        """:meth:`send` then :meth:`receive`."""
        self.send(cmd)
        return self.receive()

    def query_char(self, code: int) -> str:
        """:meth:`send_char` then the answer: one byte for ``#7`` (no LF; −7 when it does not come), a line
        (:meth:`receive`) otherwise (``#5`` answers ``"<hex>\\n"``)."""
        self.send_char(code)
        if int(code) != CHAR_READY:
            return self.receive()
        ser = self._require("receive")
        try:
            byte = ser.read(1)
        except (OSError, ValueError) as exc:
            raise self._receive_error(exc) from exc
        if not byte:
            raise self._timeout()
        self._record(RX, bytes(byte))
        self._after_ready_byte = True
        return bytes(byte).decode(self.encoding, errors="replace")

    def discard_input(self) -> bytes:
        """Read and drop every byte already waiting; returns them (also recorded as ``"drop"``)."""
        ser = self._require("discard_input")
        try:
            data = self._read_waiting(ser)
            ser.reset_input_buffer()
        except (OSError, ValueError) as exc:
            raise self._receive_error(exc) from exc
        if data:
            self._record(DROPPED, data)
        return data

    # ============================================================================================== internals
    def _require(self, what: str) -> Any:
        ser = self._serial
        if ser is None:
            raise StageError(Code.PI_NOT_CONNECTED, f"{what}: not connected")
        return ser

    def _write(self, data: bytes) -> None:
        ser = self._require("send")
        self._drop_stale_input(ser)
        try:
            ser.write(data)
        except (OSError, ValueError) as exc:        # includes serial.SerialTimeoutException (write timeout)
            log.debug("%s: write of %r failed: %s", self.port, data, exc)
            raise StageError(Code.PI_SEND_ERROR,
                             f"Data Send Error (RS-232, {self.port}, baudrate {self.baudrate}): {exc}") from exc
        self._record(TX, data)

    def _drop_stale_input(self, ser: Any) -> None:
        """Before a command: discard what is already waiting (see the module docstring)."""
        after_ready, self._after_ready_byte = self._after_ready_byte, False
        try:
            data = self._read_waiting(ser)
        except (OSError, ValueError) as exc:        # e.g. the peer has gone: the write or read reports it
            log.debug("%s: input check before a command failed: %s", self.port, exc)
            return
        if not data:
            return
        self._record(DROPPED, data)
        if after_ready and data == _LF:
            log.debug("%s: LF after the #7 answer dropped", self.port)
        else:
            log.warning("%s: %d unexpected input byte(s) discarded before the next command: %r", self.port,
                        len(data), data[:80])

    @staticmethod
    def _read_waiting(ser: Any) -> bytes:
        data = bytearray()
        while len(data) < _MAX_DISCARD:
            waiting = ser.in_waiting
            if not waiting:
                break
            chunk = ser.read(min(int(waiting), _MAX_DISCARD - len(data)))
            if not chunk:
                break
            data += chunk
        return bytes(data)

    def _record(self, direction: str, data: bytes) -> None:
        self.frames.append((direction, data))
        log.debug("%s %s %r", self.port, _ARROWS[direction], data)

    def _record_partial(self, data: bytearray) -> None:
        if data:
            self._record(RX, bytes(data))

    def _timeout(self) -> StageError:
        return StageError(Code.PI_TIMEOUT, f"Data Receive Timeout (RS-232, {self.port}, baud {self.baudrate})")

    def _receive_error(self, exc: BaseException) -> StageError:
        log.debug("%s: read failed: %s", self.port, exc)
        return StageError(Code.PI_RECEIVE_ERROR,
                          f"Data Receive Error (RS-232, {self.port}, baud {self.baudrate}): {exc}")
