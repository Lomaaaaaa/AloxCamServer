"""Helpers shared by the WP4 server tests (not a test module): frames, replies, a fake codec, polling waits."""

from __future__ import annotations

import socket
import struct
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import Any

from aloxcam.clock import monotonic_tick
from aloxcam.dispatch import STOP, Outcome, ReplyKind, Request, WorkerContext
from aloxcam.errors import NO_ERROR, CodecError, ErrorCluster
from aloxcam.imagedata import ImageData
from aloxcam.lv.variant import EMPTY_VARIANT_HEADER, synth_variant_header, write_variant
from aloxcam.server.runtime import RuntimeParts
from aloxcam.settings import ServerConfig, ServerOptions
from aloxcam.testing import RecordingEvents, make_state, packaged_enums
from aloxcam.wire.enums import build_routing
from aloxcam.wire.messages import COMMANDS, PayloadCodec
from aloxcam.wire.protocol import (
    Datatypes,
    ResponseTimestamps,
    decode_response_data,
    encode_envelope,
    encode_frame,
    parse_reply_value,
)

ENUMS = packaged_enums()
DT = Datatypes.from_enums(ENUMS)
PAYLOADS = PayloadCodec(ENUMS)
TIMEOUT = 5.0


def code_of(command: str) -> int:
    return ENUMS.code("camera_command", command)


def envelope(command: str | int, data: Any = None, *, notifier: bytes = b"N1", ts: tuple[int, int, int] = (0, 0, 0),
             data_bytes: bytes | None = None) -> bytes:
    """A ``Command`` payload; ``data_bytes`` replaces the encoded data part (malformed payloads)."""
    if isinstance(command, int):
        code, spec = command, None
    else:
        code, spec = code_of(command), COMMANDS.get(command)
    if data_bytes is None:
        variant = PAYLOADS.variant_for(spec or COMMANDS["Exit"], data)      # unknown code: empty variant
    else:
        if spec is None or spec.payload is None:
            header = EMPTY_VARIANT_HEADER
        else:
            header = synth_variant_header(PAYLOADS.layout_of(spec.payload), ENUMS, bare=spec.payload.LV_BARE)
        variant = write_variant(header, data_bytes)
    return encode_envelope(code, notifier, variant, ENUMS, ts)


def command_frame(command: str | int, data: Any = None, **kw: Any) -> bytes:
    return encode_frame(DT.command, envelope(command, data, **kw))


def recv_exact(sock: socket.socket, n: int, timeout: float = 5.0) -> bytes | None:
    """``n`` bytes, or None on EOF; :class:`TimeoutError` after ``timeout``."""
    sock.settimeout(timeout)
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def read_frame(sock: socket.socket, timeout: float = 5.0) -> tuple[int, bytes] | None:
    """One whole frame ``(datatype, payload)``, or None on EOF."""
    header = recv_exact(sock, 6, timeout)
    if header is None:
        return None
    n, datatype = struct.unpack(">iH", header)
    payload = recv_exact(sock, n - 2, timeout) if n > 2 else b""
    if payload is None:
        raise AssertionError("EOF inside a frame")
    return datatype, payload


def read_reply(sock: socket.socket, kind: ReplyKind, timeout: float = 5.0
               ) -> tuple[bytes, Any, ErrorCluster, ResponseTimestamps]:
    """The next reply frame decoded with ``kind``: ``(notifier, value, error, timestamps)``."""
    frame = read_frame(sock, timeout)
    assert frame is not None, "connection closed instead of a reply"
    datatype, payload = frame
    assert datatype == DT.response
    rd = decode_response_data(payload)
    value, err = parse_reply_value(kind, rd.flattened)
    return rd.notifier, value, err, rd.ts


def assert_eof(sock: socket.socket, timeout: float = 5.0) -> None:
    """The peer closed the connection, without sending anything first."""
    sock.settimeout(timeout)
    try:
        data = sock.recv(65536)
    except ConnectionResetError:
        return
    assert data == b"", f"unexpected bytes before EOF: {data[:40]!r}"


def wait_until(predicate: Callable[[], bool], timeout: float = 5.0, interval: float = 0.005) -> bool:
    """Poll ``predicate`` until it is true or ``timeout`` expires (for state that has no event to wait on)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


class FakeCodec:
    """:class:`aloxcam.imagedata.ImageCodec` double: ``b"IMG" + pixels``; records calls; optional failure."""

    name = "fake"

    def __init__(self, error: Exception | None = None) -> None:
        self.calls: list[ImageData] = []
        self.error = error

    def flatten(self, image: ImageData) -> bytes:
        self.calls.append(image)
        if self.error is not None:
            raise self.error
        return b"IMG" + image.pixels.tobytes()


CODEC_ERROR = CodecError(9508, "NI Vision image flattening is not implemented in this phase (phase 3)")


# ---------------------------------------------------------------------------------------------- runtime fakes
@dataclass
class CameraCtx(WorkerContext):
    forward_to_z: Callable[[Request], None] = lambda req: None


class Fakes:
    """Fake handler tables recording the shutdown order; ``Z Wait End Move`` blocks until released or aborted."""

    def __init__(self) -> None:
        self.order: list[str] = []
        self.lock = threading.Lock()
        self.release_wait = threading.Event()
        self.wait_started = threading.Event()
        self.hardware_closed = threading.Event()
        self.z_exit = threading.Event()
        self.heartbeat_gate: threading.Event | None = None    # when set: Get Heartbeat blocks until the gate opens
        self.heartbeat_entered = threading.Event()

    def note(self, what: str) -> None:
        with self.lock:
            self.order.append(what)

    # camera worker
    def heartbeat(self, ctx: WorkerContext, req: Request) -> None:
        self.heartbeat_entered.set()
        if self.heartbeat_gate is not None:
            self.heartbeat_gate.wait(TIMEOUT)
        ctx.replies.u32(req, ctx.tick(), NO_ERROR)

    def camera_exit(self, ctx: CameraCtx, req: Request) -> Outcome:
        self.note("camera Exit")
        ctx.forward_to_z(Request.make_internal("Exit", COMMANDS["Exit"]))
        return STOP

    # Z worker
    def get_position(self, ctx: WorkerContext, req: Request) -> None:
        ctx.replies.dbl(req, 12.5, NO_ERROR)

    def wait_end_move(self, ctx: WorkerContext, req: Request) -> None:
        self.wait_started.set()
        self.release_wait.wait(TIMEOUT)
        self.note("Z wait ended")
        ctx.replies.ack(req, NO_ERROR)

    def z_exit_handler(self, ctx: WorkerContext, req: Request) -> Outcome:
        self.note("Z Exit")
        self.z_exit.set()
        return STOP

    def abort_waits(self) -> None:
        self.note("abort_waits")
        self.release_wait.set()

    def close_hardware(self) -> None:
        self.note("close_hardware")
        self.hardware_closed.set()


def make_parts(fakes: Fakes, *, shutdown_timeout_s: float = 2.0, z_exit: bool = True) -> RuntimeParts:
    config = replace(ServerConfig(), server=ServerOptions(shutdown_timeout_s=shutdown_timeout_s, io_timeout_s=2.0))
    z_handlers = {"Z Get Position": fakes.get_position, "Z Wait End Move": fakes.wait_end_move}
    if z_exit:
        z_handlers["Exit"] = fakes.z_exit_handler
    return RuntimeParts(
        config=config, host="127.0.0.1", port=0, enums=ENUMS, routing=build_routing(ENUMS), commands=COMMANDS,
        payloads=PAYLOADS, state=make_state(), events=RecordingEvents(), codec=FakeCodec(), tick=monotonic_tick,
        camera_handlers={"Get Heartbeat": fakes.heartbeat, "Exit": fakes.camera_exit}, z_handlers=z_handlers,
        make_camera_context=lambda base, fwd: CameraCtx(base.state, base.replies, base.events, base.compat, base.tick,
                                                        forward_to_z=fwd),
        make_z_context=lambda base: base, abort_waits=fakes.abort_waits, close_hardware=fakes.close_hardware)
