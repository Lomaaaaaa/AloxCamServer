"""``FakeC663`` / ``FakeC663Server``: an emulated PI C-663 that speaks the GCS ASCII protocol (tests and rehearsal).

Spec: docs/phase2-design.md §2 (``socket://`` testing), §3.3; 08 §3.3 (frame rules), §3.4 (frame catalogue and answer
formats), §3.5 (errors latched until ``ERR?``), §4.4 (referencing), §9 (joystick), §10 (macros);
docs/answers/owner-answers.md §5 (``DH`` / ``HOME``).

:class:`FakeC663` parses the bytes a GCS driver sends and drives the :class:`aloxcam.zstage.sim.SimStage` model with
them — motion, referencing, macros, joystick and the latched error all come from that model, nothing is duplicated
here. Answers use the documented formats (08 §3.4): ``1=12.500000`` (6 decimals) for numbers, ``1=1`` for booleans,
``0`` for ``ERR?``, the single byte 0xB1 / 0xB0 (no LF) for ``#7``, ``1`` / ``0`` + LF for ``#5``, multi-line answers
with ``" \\n"`` after every line but the last (``MAC? DH`` returns the owner's macro text), and an empty line for
``RMC?`` when no macro runs.

Errors: an unknown command latches 2 (so the ``XXX`` of the open purge latches 2, and the purge's ``err?`` reads it), a
malformed one latches 1, and the model latches the rest (5, 7, 10, 15, 17, 20, 51, 1008). A query that fails (an
unknown axis, an unknown macro, a syntax error) gets **no answer**, so the driver times out (−7).

:class:`FakeC663Server` serves it on a loopback TCP port, one connection at a time; the controller state survives
reconnections. ``GcsSerialLink("socket://127.0.0.1:<port>", …)`` — or ``[stage] port_map = COM4=socket://…`` in the
server — talks to it exactly as to the real controller. Command line, for rehearsal::

    python -m aloxcam.zstage.fake_c663 --port 5555 [--initially-referenced]

Fault injection: :attr:`FakeC663.answer_delay_s`, :meth:`FakeC663.drop_next_answer`, :meth:`FakeC663.fail_next`,
:meth:`FakeC663.bad_ready_byte` and :meth:`FakeC663.disconnect`. Test synchronisation:
:attr:`FakeC663.processed_bytes` / :meth:`FakeC663.wait_processed` (a command without answer has been executed once
its bytes are counted).
"""

from __future__ import annotations

import argparse
import logging
import select
import signal
import socket
import threading
import time
from collections import deque
from collections.abc import Callable, Sequence

from aloxcam.errors import StageError
from aloxcam.settings import SimulationOptions
from aloxcam.zstage.sim import SimStage

__all__ = [
    "DEFAULT_PORT",
    "FAKE_IDENTITY",
    "FRAME_LOG_SIZE",
    "FakeC663",
    "FakeC663Server",
    "main",
]

log = logging.getLogger("aloxcam.zstage.fake_c663")       # also when run as __main__ (python -m)

DEFAULT_PORT = 5555
FAKE_IDENTITY = "(c) aloxcam FakeC663, C-663.FAKE, 0000000663, 0.0.0"   # command-line default (*IDN?)
FRAME_LOG_SIZE = 4096                    # frames kept in FakeC663.frames (like SimStage.frames)

_E_SYNTAX = 1                            # Parameter syntax error
_E_UNKNOWN_COMMAND = 2                   # Unknown command
_SINGLE_BYTE = {0x05: "#5", 0x07: "#7", 0x18: "#24"}
_LF = 0x0A
_READY, _BUSY = b"\xb1", b"\xb0"
_BAD_READY = b"?"
_POLL_S = 0.05                           # server: how often a blocked select re-checks stop / disconnect


class _Syntax(Exception):
    """A malformed frame: error 1, no answer."""


Lines = list[str] | None                 # a handler's answer lines (None = no answer)


class FakeC663:
    """The GCS ASCII protocol on top of a :class:`SimStage` (docs/phase2-design.md §3.3). Thread-safe.

    ``options`` are the ``[simulation]`` stage parameters (travel range, reference time, identity, initial
    reference); ``clock`` drives the model (default ``time.monotonic``); ``dh_running`` starts the ``DH`` daemon at
    power-up as on the real machine. :attr:`model` is the SimStage itself, for test hooks (``position``,
    ``power_cycle()``, ``trigger_home()``, ``error_register``…).
    """

    answer_delay_s: float

    def __init__(self, options: SimulationOptions | None = None, *, clock: Callable[[], float] | None = None,
                 dh_running: bool = True, encoding: str = "cp1252") -> None:
        self._model = SimStage(options or SimulationOptions(), clock=clock or time.monotonic, dh_running=dh_running)
        self._model.connect_rs232("fake-c663", 115200, 0)     # the model's own link: always open
        self._model.clear_frames()
        self._encoding = encoding
        self._cond = threading.Condition(threading.RLock())
        self._buffer = bytearray()
        self._frames: deque[str] = deque(maxlen=FRAME_LOG_SIZE)
        self._processed = 0
        self._drop_next = False
        self._bad_ready = False
        self._failures: list[tuple[str, int]] = []
        self._disconnect = threading.Event()
        self.answer_delay_s = 0.0
        m = self._model
        self._handlers: dict[str, Callable[[list[str]], Lines]] = {
            "*IDN?": lambda a: [m.qidn()],
            "ERR?": lambda a: [str(m.qerr())],
            "SAI?": lambda a: m.qsai(),                       # "SAI?" and "SAI? ALL"
            "CST?": lambda a: [f"{ax}={m.qcst(ax)}" for ax in self._axes(a)],
            "SVO": lambda a: self._each_pair(a, lambda ax, v: m.svo(ax, self._flag(v))),
            "SVO?": lambda a: self._flags(a, m.qsvo),
            "RON?": lambda a: self._flags(a, m.qron),
            "TRS?": lambda a: self._flags(a, m.qtrs),
            "LIM?": lambda a: self._flags(a, m.qlim),
            "FRF?": lambda a: self._flags(a, m.qfrf),
            "TMN?": lambda a: self._numbers(a, m.qtmn),
            "TMX?": lambda a: self._numbers(a, m.qtmx),
            "POS?": lambda a: self._numbers(a, m.qpos),
            "VEL?": lambda a: self._numbers(a, m.qvel),
            "MOV": lambda a: self._each_pair(a, lambda ax, v: m.mov(ax, self._float(v))),
            "MVR": lambda a: self._each_pair(a, lambda ax, v: m.mvr(ax, self._float(v))),
            "VEL": lambda a: self._each_pair(a, lambda ax, v: m.vel(ax, self._float(v))),
            "FNL": lambda a: self._each_axis(a, m.fnl),
            "FRF": lambda a: self._each_axis(a, m.frf),
            "FPL": lambda a: self._each_axis(a, m.fpl),
            "STP": self._stp,
            "RMC?": lambda a: [m.qrmc()],
            "MAC START": self._mac_start,
            "MAC?": self._mac_query,
            "JAX": self._jax,
            "JON": lambda a: self._each_pair(a, lambda j, v: m.jon(self._int(j), self._flag(v))),
            "JON?": lambda a: [f"{j}={m.qjon(self._int(j))}" for j in (a or ["1"])],
        }

    # ============================================================================================== state and hooks
    @property
    def model(self) -> SimStage:
        """The controller model (test hooks: ``position``, ``referenced``, ``power_cycle()``, ``trigger_home()``…)."""
        return self._model

    @property
    def frames(self) -> list[str]:
        """Every frame received, oldest first: text commands without their LF (as sent, e.g. ``"*idn?"``), the
        single-byte commands as ``"#5"``, ``"#7"``, ``"#24"`` (the format of ``SimStage.frames``)."""
        with self._cond:
            return list(self._frames)

    def clear_frames(self) -> None:
        """Forget the recorded frames."""
        with self._cond:
            self._frames.clear()

    @property
    def processed_bytes(self) -> int:
        """How many bytes :meth:`feed` has processed since construction (all connections)."""
        with self._cond:
            return self._processed

    def wait_processed(self, count: int, timeout: float) -> bool:
        """Wait until :attr:`processed_bytes` ≥ ``count``; False on timeout."""
        with self._cond:
            return self._cond.wait_for(lambda: self._processed >= count, timeout)

    def drop_next_answer(self) -> None:
        """The next answer (of a query or of ``#5`` / ``#7``) is not sent: the driver times out."""
        with self._cond:
            self._drop_next = True

    def fail_next(self, command_prefix: str, code: int) -> None:
        """The next frame starting with ``command_prefix`` (case-insensitive; ``"MOV"``, ``"POS?"``, ``"#7"``…) is not
        executed: the controller latches ``code`` instead, and a query gets no answer."""
        with self._cond:
            self._failures.append((command_prefix.strip().upper(), int(code)))

    def bad_ready_byte(self) -> None:
        """The next ``#7`` answers ``"?"`` instead of 0xB1 / 0xB0 (the driver reports ``"#7 Unknown reply"``)."""
        with self._cond:
            self._bad_ready = True

    def disconnect(self) -> None:
        """Drop the current client connection (the cable is pulled); bytes not processed yet are lost. The controller
        state is kept and the server accepts the next connection. No effect when no client is connected."""
        self._disconnect.set()

    def take_disconnect_request(self) -> bool:
        """For the server: whether :meth:`disconnect` was called since the last call (and clear the request)."""
        requested = self._disconnect.is_set()
        self._disconnect.clear()
        return requested

    def reset_input(self) -> None:
        """For the server, on a new connection: a partial line of the previous one is forgotten."""
        with self._cond:
            self._buffer.clear()

    # ============================================================================================== protocol
    def feed(self, data: bytes) -> bytes:
        """Process received bytes (any chunking) and return the answer bytes. Text frames end at LF (a partial line
        is kept for the next call); 0x05, 0x07 and 0x18 are handled as soon as they arrive."""
        out = bytearray()
        with self._cond:
            for byte in bytes(data):
                if byte in _SINGLE_BYTE:
                    out += self._single(byte)
                elif byte == _LF:
                    line = bytes(self._buffer)
                    self._buffer.clear()
                    out += self._line(line)
                else:
                    self._buffer.append(byte)
            self._processed += len(data)
            self._cond.notify_all()
        return bytes(out)

    def _record(self, frame: str) -> bool:
        """Log the frame; True when an injected failure consumed it (``fail_next``)."""
        self._frames.append(frame)
        log.debug("< %s", frame)
        upper = frame.upper()
        for i, (prefix, code) in enumerate(self._failures):
            if upper.startswith(prefix):
                del self._failures[i]
                self._model.latch_error(code)
                log.info("injected failure: %r latched error %d", frame, code)
                return True
        return False

    def _emit(self, answer: bytes) -> bytes:
        if self._drop_next:
            self._drop_next = False
            log.info("injected fault: answer %r dropped", answer)
            return b""
        log.debug("> %r", answer)
        return answer

    def _single(self, byte: int) -> bytes:
        if self._record(_SINGLE_BYTE[byte]):
            return b""
        if byte == 0x05:                                      # #5: hexadecimal mask, bit 0 = axis 1
            return self._emit(b"1\n" if self._model.is_moving() else b"0\n")
        if byte == 0x07:                                      # #7: one byte, no LF
            if self._bad_ready:
                self._bad_ready = False
                return self._emit(_BAD_READY)
            return self._emit(_READY if self._model.is_controller_ready() else _BUSY)
        self._model.stop_all()                                # #24
        return b""

    def _line(self, raw: bytes) -> bytes:
        text = raw.decode(self._encoding, errors="replace").strip()
        if not text:
            return b""
        if self._record(text):
            return b""
        words = text.split()
        mnemonic, args = words[0].upper(), words[1:]
        if mnemonic == "MAC" and args:
            mnemonic, args = f"MAC {args[0].upper()}", args[1:]
        handler = self._handlers.get(mnemonic)
        if handler is None:
            self._model.latch_error(_E_UNKNOWN_COMMAND)
            return b""
        try:
            lines = handler(args)
        except _Syntax:
            self._model.latch_error(_E_SYNTAX)
            return b""
        except StageError as exc:                             # e.g. a query for an unknown axis: error 15 latched
            log.debug("%r: no answer (%s)", text, exc)
            return b""
        if lines is None:
            return b""
        return self._emit((" \n".join(lines) + "\n").encode(self._encoding, errors="replace"))

    # ============================================================================================== argument helpers
    def _axes(self, args: list[str]) -> list[str]:
        """The axes of a command; none = every connected axis."""
        return args or self._model.qsai()

    @staticmethod
    def _float(text: str) -> float:
        try:
            return float(text)
        except ValueError:
            raise _Syntax(text) from None

    @staticmethod
    def _int(text: str) -> int:
        try:
            return int(text)
        except ValueError:
            raise _Syntax(text) from None

    def _flag(self, text: str) -> bool:
        value = self._int(text)
        if value not in (0, 1):
            raise _Syntax(text)
        return bool(value)

    def _each_pair(self, args: list[str], apply: Callable[[str, str], object]) -> Lines:
        if not args or len(args) % 2:
            raise _Syntax(" ".join(args))
        for key, value in zip(args[0::2], args[1::2], strict=True):
            apply(key, value)
        return None

    def _each_axis(self, args: list[str], apply: Callable[[str], object]) -> Lines:
        for axis in self._axes(args):
            apply(axis)
        return None

    def _flags(self, args: list[str], read: Callable[[str], bool]) -> Lines:
        return [f"{ax}={int(read(ax))}" for ax in self._axes(args)]

    def _numbers(self, args: list[str], read: Callable[[str], float]) -> Lines:
        return [f"{ax}={read(ax):.6f}" for ax in self._axes(args)]

    # ============================================================================================== special commands
    def _stp(self, args: list[str]) -> Lines:
        if not args:
            self._model.stp(None)
        for axis in args:
            self._model.stp(axis)
        return None

    def _mac_start(self, args: list[str]) -> Lines:
        if len(args) != 1:
            raise _Syntax(" ".join(args))
        self._model.mac_start(args[0])
        return None

    def _mac_query(self, args: list[str]) -> Lines:
        if len(args) != 1:
            raise _Syntax(" ".join(args))
        text = self._model.qmac(args[0])                      # unknown macro: "" and error 20 latched
        return text.split("\n") if text else None

    def _jax(self, args: list[str]) -> Lines:
        if len(args) != 3:
            raise _Syntax(" ".join(args))
        self._model.jax(self._int(args[0]), self._int(args[1]), args[2])
        return None


class FakeC663Server:
    """A :class:`FakeC663` on a loopback TCP port, served by a daemon thread, one connection at a time
    (docs/phase2-design.md §3.3). ``port=0`` picks a free port; :attr:`url` is ``"socket://<host>:<port>"``. Use
    :meth:`start` / :meth:`close`, or ``with FakeC663Server(...) as server:``."""

    def __init__(self, options: SimulationOptions | None = None, *, clock: Callable[[], float] | None = None,
                 host: str = "127.0.0.1", port: int = 0, dh_running: bool = True) -> None:
        self.controller = FakeC663(options, clock=clock, dh_running=dh_running)
        self._host = host
        self._requested_port = int(port)
        self._listener: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._conn: socket.socket | None = None
        self._connections = 0

    # ============================================================================================== life cycle
    def start(self) -> FakeC663Server:
        """Bind and start serving (idempotent); returns ``self``."""
        if self._listener is not None:
            return self
        self._listener = socket.create_server((self._host, self._requested_port))
        self._thread = threading.Thread(target=self._run, name=f"fake-c663-{self.port}", daemon=True)
        self._thread.start()
        log.info("FakeC663 listening on %s", self.url)
        return self

    def close(self) -> None:
        """Stop serving and close every socket (idempotent)."""
        self._stop.set()
        with self._lock:
            conn = self._conn
        if conn is not None:
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(5.0)
            self._thread = None
        if self._listener is not None:
            self._listener.close()

    def __enter__(self) -> FakeC663Server:
        return self.start()

    def __exit__(self, *exc: object) -> None:
        self.close()

    # ============================================================================================== state
    @property
    def address(self) -> tuple[str, int]:
        """``(host, port)`` once started."""
        if self._listener is None:
            raise RuntimeError("FakeC663Server not started")
        host, port = self._listener.getsockname()[:2]
        return str(host), int(port)

    @property
    def port(self) -> int:
        return self.address[1]

    @property
    def url(self) -> str:
        """``"socket://<host>:<port>"``, for ``GcsSerialLink`` or ``[stage] port_map``."""
        host, port = self.address
        return f"socket://{host}:{port}"

    @property
    def connections(self) -> int:
        """How many connections were accepted so far."""
        with self._lock:
            return self._connections

    @property
    def connected(self) -> bool:
        """TRUE while a client is being served."""
        with self._lock:
            return self._conn is not None

    def disconnect(self) -> None:
        """:meth:`FakeC663.disconnect`."""
        self.controller.disconnect()

    # ============================================================================================== serving
    def _run(self) -> None:
        listener = self._listener
        assert listener is not None
        while not self._stop.is_set():
            try:
                readable, _, _ = select.select([listener], [], [], _POLL_S)
            except (OSError, ValueError):
                break
            if not readable:
                continue
            try:
                conn, peer = listener.accept()
            except OSError:
                continue
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)   # answers leave at once, as on a serial line
            self.controller.reset_input()
            self.controller.take_disconnect_request()        # a request made while nobody was connected
            with self._lock:
                self._conn = conn
                self._connections += 1
            log.info("FakeC663: client %s connected", peer)
            try:
                self._serve(conn)
            except Exception:                                 # never let the thread die silently
                log.exception("FakeC663: connection failed")
            finally:
                with self._lock:
                    self._conn = None
                conn.close()
                log.info("FakeC663: client %s disconnected", peer)

    def _serve(self, conn: socket.socket) -> None:
        controller = self.controller
        while not self._stop.is_set():
            if controller.take_disconnect_request():
                log.info("FakeC663: injected disconnect")
                return
            readable, _, _ = select.select([conn], [], [], _POLL_S)
            if not readable:
                continue
            if controller.take_disconnect_request():          # requested before these bytes: they are lost
                log.info("FakeC663: injected disconnect")
                return
            try:
                data = conn.recv(4096)
            except OSError:
                return
            if not data:
                return
            answer = controller.feed(data)
            if not answer:
                continue
            delay = controller.answer_delay_s
            if delay > 0 and self._stop.wait(delay):
                return
            try:
                conn.sendall(answer)
            except OSError:
                return


# ---------------------------------------------------------------------------------------------- command line
def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="python -m aloxcam.zstage.fake_c663",
        description="An emulated PI C-663 on a TCP port, for rehearsing the real-stage server: set "
                    "[stage] port_map = COM<n>=socket://127.0.0.1:<port> in alox_camera_server.ini.")
    p.add_argument("--host", default="127.0.0.1", help="listen address (default 127.0.0.1)")
    p.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"TCP port (default {DEFAULT_PORT}; 0 = any)")
    p.add_argument("--initially-referenced", action="store_true", help="the axis starts referenced, servo on")
    p.add_argument("--no-dh", action="store_true", help="the DH macro does not run at power-up")
    p.add_argument("--min-mm", type=float, default=0.0, help="travel range minimum (default 0)")
    p.add_argument("--max-mm", type=float, default=100.0, help="travel range maximum (default 100)")
    p.add_argument("--reference-s", type=float, default=2.0, help="duration of a reference move (default 2 s)")
    p.add_argument("--identity", default=FAKE_IDENTITY, help="*IDN? answer")
    p.add_argument("--answer-delay-ms", type=float, default=0.0, help="delay before every answer (default 0)")
    p.add_argument("--log-level", type=str.upper, default="INFO", choices=("DEBUG", "INFO", "WARNING"),
                   help="DEBUG shows every frame")
    return p


def main(argv: Sequence[str] | None = None) -> int:
    """Run a :class:`FakeC663Server` until Ctrl+C (or SIGTERM). Prints the URL to use in ``[stage] port_map``."""
    args = _parser().parse_args(argv)
    logging.basicConfig(level=args.log_level, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    options = SimulationOptions(stage_min_mm=args.min_mm, stage_max_mm=args.max_mm,
                                stage_initially_referenced=args.initially_referenced,
                                stage_reference_s=max(0.0, args.reference_s), stage_identity=args.identity)
    try:
        server = FakeC663Server(options, host=args.host, port=args.port, dh_running=not args.no_dh).start()
    except OSError as exc:
        print(f"cannot listen on {args.host}:{args.port}: {exc}", flush=True)
        return 1
    server.controller.answer_delay_s = max(0.0, args.answer_delay_ms) / 1000.0
    print(f"FakeC663 listening on {server.url}", flush=True)
    print(f"alox_camera_server.ini: [stage] port_map = COM<n>={server.url}  (n = alox.ini [hardware.z] comport_I32)",
          flush=True)
    print("Ctrl+C stops it.", flush=True)
    stopped = threading.Event()
    try:
        signal.signal(signal.SIGTERM, lambda _signum, _frame: stopped.set())   # `kill`: stop cleanly too
    except (ValueError, OSError):                              # not in the main thread
        pass
    try:
        while not stopped.wait(0.5):                           # a timed wait keeps Ctrl+C working on Windows
            pass
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
