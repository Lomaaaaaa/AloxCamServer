"""Phase 2 test doubles: a ``PiSerialStage`` on a ``FakeC663Server`` with ``SimStage``'s test hooks, and a scripted
socket peer.

Spec: docs/phase2-design.md §5 (the Z handler and ``ZController`` suites run through ``PiSerialStage`` +
``FakeC663Server`` as well as through ``SimStage``; ``GcsSerialLink`` / ``PiSerialStage`` against a socket peer).

- :class:`FakeStageProbe` is what ``make_sim`` returns for the ``"fake"`` parameter (tests/zstage/conftest.py). It is
  the real :class:`PiSerialStage` (frames on a real TCP socket, real parsing) plus the hooks the phase 1 tests use on a
  ``SimStage``: ``frames`` (what the fake controller received), ``inject_failure`` (raised before the call, as
  ``SimStage`` does), ``position``, ``power_cycle()``, … read from the fake's model.
- :class:`SyncedLink` makes a write return only once the fake has processed the bytes, so a command without answer
  (``MOV``, ``STP``) runs at the caller's fake time, exactly as a ``SimStage`` call would.
- :class:`ScriptedPeer` answers canned bytes per frame, for the answer formats the fake never produces.
"""

from __future__ import annotations

import select
import socket
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from aloxcam.errors import StageError
from aloxcam.zstage.fake_c663 import FakeC663, FakeC663Server
from aloxcam.zstage.gcs_serial import GcsSerialLink
from aloxcam.zstage.pi_serial import PiSerialStage
from aloxcam.zstage.sim import INJECTABLE_METHODS

SYNC_TIMEOUT_S = 5.0


class SyncedLink(GcsSerialLink):
    """A link to a :class:`FakeC663Server` whose writes wait until the fake has processed them. ``name`` is the port
    name the stage asked for (``COM2``): it is used in the error texts, as for a real port."""

    def __init__(self, url: str, baudrate: int, timeout_s: float, *, fake: FakeC663, name: str) -> None:
        self._fake = fake
        self._base = fake.processed_bytes
        self._written = 0
        super().__init__(url, baudrate, timeout_s)
        self.port = name

    def _write(self, data: bytes) -> None:
        super()._write(data)
        self._written += len(data)
        if not self._fake.wait_processed(self._base + self._written, SYNC_TIMEOUT_S):
            raise AssertionError(f"the fake C-663 did not process {data!r} within {SYNC_TIMEOUT_S} s")


@dataclass
class _Failure:
    error: StageError
    remaining: int


class FakeStageProbe(PiSerialStage):
    """``PiSerialStage`` → ``SyncedLink`` → ``FakeC663Server``, with ``SimStage``'s test hooks. Every port name
    (``COM2``, ``/dev/ttyUSB0``…) reaches the fake; :attr:`port` is the ``(port, baud rate)`` last asked for."""

    def __init__(self, server: FakeC663Server) -> None:
        self.server = server
        self.fake = server.controller
        self._requested: tuple[str, int] = ("", 0)
        self._failures: dict[str, list[_Failure]] = {}
        super().__init__(self._open_link, sleep=lambda _s: None)

    def _open_link(self, port: str, baudrate: int, timeout_s: float) -> GcsSerialLink:
        self._requested = (port, int(baudrate))
        return SyncedLink(self.server.url, baudrate, timeout_s, fake=self.fake, name=port)

    def shutdown(self) -> None:
        """Close the link and the server (fixture teardown)."""
        self._failures.clear()
        PiSerialStage.close(self)
        self.server.close()

    # ---------------------------------------------------------------------------------------- SimStage test hooks
    def inject_failure(self, method: str, error: StageError, *, times: int = 1) -> None:
        """``SimStage.inject_failure``: the next ``times`` calls of ``method`` raise ``error`` before anything else."""
        if method not in INJECTABLE_METHODS:
            raise ValueError(f"not a StageBackend method: {method!r}")
        self._failures.setdefault(method, []).append(_Failure(error, times))

    def _injected(self, method: str) -> None:
        queue = self._failures.get(method)
        if queue:
            failure = queue[0]
            failure.remaining -= 1
            if failure.remaining <= 0:
                queue.pop(0)
            raise failure.error

    def connect_rs232(self, port: str, baudrate: int, timeout_ms: int) -> int:
        PiSerialStage.close(self)                    # like SimStage: a failed open leaves the link closed
        self._injected("connect_rs232")
        return super().connect_rs232(port, baudrate, timeout_ms)

    @property
    def frames(self) -> list[str]:
        return self.fake.frames

    def clear_frames(self) -> None:
        self.fake.clear_frames()

    @property
    def error_register(self) -> int:
        return self.fake.model.error_register

    @property
    def position(self) -> float:
        return self.fake.model.position

    @property
    def velocity(self) -> float:
        return self.fake.model.velocity

    @property
    def referenced(self) -> bool:
        return self.fake.model.referenced

    @property
    def running_macro(self) -> str:
        return self.fake.model.running_macro

    @property
    def joystick_enabled(self) -> bool:
        return self.fake.model.joystick_enabled

    @property
    def port(self) -> tuple[str, int]:
        return self._requested

    def trigger_home(self) -> bool:
        return self.fake.model.trigger_home()

    def power_cycle(self) -> None:
        self.fake.model.power_cycle()


def _injectable(name: str) -> Callable[..., Any]:
    base = getattr(PiSerialStage, name)

    def method(self: FakeStageProbe, *args: Any, **kwargs: Any) -> Any:
        self._injected(name)
        return base(self, *args, **kwargs)

    method.__name__ = name
    method.__doc__ = base.__doc__
    return method


for _name in INJECTABLE_METHODS - {"connect_rs232"}:
    setattr(FakeStageProbe, _name, _injectable(_name))


# ---------------------------------------------------------------------------------------------- scripted peer
Answer = bytes | Sequence[bytes] | None
Responder = Callable[[bytes], Answer]


class ScriptedPeer:
    """A loopback TCP peer standing in for the controller (one connection at a time). Every complete frame — a text
    line with its LF, or one of the bytes 0x05 / 0x07 / 0x18 — is recorded in :attr:`frames` and answered with
    ``respond(frame)``: bytes, a sequence of chunks (sent 20 ms apart), or None (no answer). A mapping is looked up
    by frame. :attr:`received` holds every byte received."""

    def __init__(self, respond: Responder | Mapping[bytes, Answer] | None = None) -> None:
        if respond is None:
            respond = {}
        if isinstance(respond, Mapping):
            table = dict(respond)
            self._respond: Responder = lambda frame: table.get(frame)
        else:
            self._respond = respond
        self.frames: list[bytes] = []
        self.received = bytearray()
        self._lock = threading.Lock()
        self._listener = socket.create_server(("127.0.0.1", 0))
        self._stop = threading.Event()
        self._conn: socket.socket | None = None
        self._thread = threading.Thread(target=self._run, name="scripted-peer", daemon=True)
        self._thread.start()

    @property
    def url(self) -> str:
        host, port = self._listener.getsockname()[:2]
        return f"socket://{host}:{port}"

    def wait_frames(self, count: int, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if len(self.frames) >= count:
                    return True
            time.sleep(0.005)
        return False

    def wait_connected(self, timeout: float = 5.0) -> bool:
        """Wait until a client connection has been accepted."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._conn is not None:
                    return True
            time.sleep(0.005)
        return False

    def drop_connection(self) -> None:
        """Close the current connection from the peer side."""
        with self._lock:
            conn, self._conn = self._conn, None
        if conn is not None:
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:                                  # already closed by the serving thread
                pass

    def close(self) -> None:
        self._stop.set()
        self.drop_connection()
        self._thread.join(5.0)
        self._listener.close()

    def __enter__(self) -> ScriptedPeer:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def _run(self) -> None:
        while not self._stop.is_set():
            readable, _, _ = select.select([self._listener], [], [], 0.05)
            if not readable:
                continue
            try:
                conn, _ = self._listener.accept()
            except OSError:
                continue
            with self._lock:
                self._conn = conn
            try:
                self._serve(conn)
            finally:
                with self._lock:
                    if self._conn is conn:
                        self._conn = None
                conn.close()

    def _serve(self, conn: socket.socket) -> None:
        line = bytearray()
        while not self._stop.is_set():
            readable, _, _ = select.select([conn], [], [], 0.05)
            if not readable:
                continue
            try:
                data = conn.recv(4096)
            except OSError:
                return
            if not data:
                return
            for byte in data:
                with self._lock:
                    self.received.append(byte)
                if byte in (0x05, 0x07, 0x18):
                    frame = bytes([byte])
                elif byte == 0x0A:
                    frame = bytes(line) + b"\n"
                    line.clear()
                else:
                    line.append(byte)
                    continue
                with self._lock:
                    self.frames.append(frame)
                answer = self._respond(frame)
                if answer is None:
                    continue
                chunks = [answer] if isinstance(answer, bytes) else list(answer)
                try:
                    for i, chunk in enumerate(chunks):
                        if i:
                            time.sleep(0.02)
                        conn.sendall(chunk)
                except OSError:
                    return
