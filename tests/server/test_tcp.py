"""aloxcam.server.tcp: one client at a time, whole frames, closing rules, heartbeat frames (02 §3–§5, §9;
design §3.4.5). Loopback and ephemeral ports only."""

from __future__ import annotations

import queue
import socket
import struct
import time
from collections.abc import Iterator
from dataclasses import dataclass
from typing import Any

import pytest

from aloxcam.server.link import ClientLink
from aloxcam.server.tcp import TcpServer
from aloxcam.settings import CompatSwitches, ServerOptions
from aloxcam.state import ServerState
from aloxcam.testing import FixedClock, make_state
from aloxcam.wire.protocol import encode_frame
from server.wp4_helpers import DT, assert_eof, read_frame, wait_until

TIMEOUT = 5.0


class FakeRouter:
    """Records ``(payload, conn_id)`` of every routed command frame."""

    def __init__(self) -> None:
        self.frames: queue.Queue[tuple[bytes, int]] = queue.Queue()

    def route_frame(self, payload: bytes, conn_id: int) -> None:
        self.frames.put((payload, conn_id))

    def next(self, timeout: float = TIMEOUT) -> tuple[bytes, int]:
        return self.frames.get(timeout=timeout)

    def assert_nothing_routed(self) -> None:
        assert self.frames.empty(), "unexpected routed frame"


@dataclass
class Rig:
    server: TcpServer
    router: FakeRouter
    link: ClientLink
    state: ServerState
    address: tuple[str, int]

    def connect(self) -> socket.socket:
        sock = socket.create_connection(self.address, timeout=TIMEOUT)
        sock.settimeout(TIMEOUT)
        return sock


def _rig(*, compat: CompatSwitches | None = None, io_timeout_s: float = 2.0,
         max_payload_bytes: int = 1 << 20) -> Iterator[Rig]:
    router = FakeRouter()
    link = ClientLink()
    state = make_state()
    server = TcpServer(host="127.0.0.1", port=0, link=link, router=router, state=state,  # type: ignore[arg-type]
                       datatypes=DT, compat=compat or CompatSwitches(),
                       options=ServerOptions(io_timeout_s=io_timeout_s, max_payload_bytes=max_payload_bytes),
                       tick=FixedClock(42))
    address = server.bind()
    server.start()
    try:
        yield Rig(server, router, link, state, address)
    finally:
        server.stop()


@pytest.fixture
def rig() -> Iterator[Rig]:
    yield from _rig()


def cmd(payload: bytes) -> bytes:
    return encode_frame(DT.command, payload)


def test_bind_returns_the_ephemeral_address(rig: Rig) -> None:
    host, port = rig.address
    assert host == "127.0.0.1" and port > 0 and rig.server.address == rig.address


def test_command_frames_are_routed_with_the_connection_id(rig: Rig) -> None:
    with rig.connect() as client:
        client.sendall(cmd(b"first") + cmd(b"second"))           # back to back in one segment
        assert rig.router.next() == (b"first", 1)
        assert rig.router.next() == (b"second", 1)
        assert rig.link.current_id == 1


def test_header_split_across_segments(rig: Rig) -> None:
    frame = cmd(b"split payload")
    with rig.connect() as client:
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        for part in (frame[:1], frame[1:4], frame[4:7], frame[7:]):
            client.sendall(part)
            time.sleep(0.01)                                      # separate segments (not a synchronisation)
        assert rig.router.next() == (b"split payload", 1)


def test_idle_client_is_kept() -> None:
    for rig in _rig(io_timeout_s=0.1):
        with rig.connect() as client:
            assert wait_until(lambda: rig.state.connected)
            time.sleep(0.5)                                       # idle much longer than io_timeout_s
            client.sendall(cmd(b"still here"))
            assert rig.router.next() == (b"still here", 1)


def test_mid_frame_timeout_closes_the_connection() -> None:
    for rig in _rig(io_timeout_s=0.2):
        with rig.connect() as client:
            client.sendall(cmd(b"0123456789")[:9])                # header + 3 of 10 payload bytes
            assert_eof(client)
            rig.router.assert_nothing_routed()


def test_partial_header_timeout_closes_the_connection() -> None:
    for rig in _rig(io_timeout_s=0.2):
        with rig.connect() as client:
            client.sendall(b"\x00\x00")
            assert_eof(client)


@pytest.mark.parametrize("n", [1, 0, -5])
def test_n_below_2_closes_the_connection(rig: Rig, n: int) -> None:
    with rig.connect() as client:
        client.sendall(struct.pack(">iH", n, DT.command))
        assert_eof(client)
        rig.router.assert_nothing_routed()


def test_oversized_frame_closes_the_connection() -> None:
    for rig in _rig(max_payload_bytes=100):
        with rig.connect() as client:
            client.sendall(struct.pack(">iH", 2 + 101, DT.command))
            assert_eof(client)


def test_heartbeat_response_image_and_unknown_frames_are_ignored(rig: Rig) -> None:
    with rig.connect() as client:
        client.sendall(encode_frame(DT.heartbeat) + encode_frame(DT.response, b"resp") + encode_frame(DT.image, b"img")
                       + encode_frame(4, b"statut") + encode_frame(999, b"?") + cmd(b"after"))
        assert rig.router.next() == (b"after", 1)                 # connection kept, only the command routed
        rig.router.assert_nothing_routed()


def test_heartbeat_with_payload_closes_by_default(rig: Rig) -> None:
    with rig.connect() as client:
        client.sendall(encode_frame(DT.heartbeat, b"x"))
        assert_eof(client)


def test_heartbeat_with_payload_ignored_when_switched_off() -> None:
    for rig in _rig(compat=CompatSwitches(close_on_extra_heartbeat_data=False)):
        with rig.connect() as client:
            client.sendall(encode_frame(DT.heartbeat, b"extra") + cmd(b"next"))
            assert rig.router.next() == (b"next", 1)


def test_second_client_served_after_the_first_disconnects(rig: Rig) -> None:
    first = rig.connect()
    first.sendall(cmd(b"one"))
    assert rig.router.next() == (b"one", 1)
    second = rig.connect()                                        # completes in the backlog (02 §3)
    second.sendall(cmd(b"two"))
    time.sleep(0.2)
    rig.router.assert_nothing_routed()                            # not served while the first one is
    first.close()
    assert rig.router.next() == (b"two", 2)
    assert rig.link.current_id == 2
    second.close()


def test_connected_flag_and_link(rig: Rig) -> None:
    assert not rig.state.connected and rig.link.current_id is None
    client = rig.connect()
    assert wait_until(lambda: rig.state.connected and rig.link.current_id == 1)
    assert rig.state.server_state == "Read" and rig.state.heartbeat == 42
    assert rig.link.send(encode_frame(DT.response, b"reply"), 1)
    assert read_frame(client) == (DT.response, b"reply")
    client.close()
    assert wait_until(lambda: not rig.state.connected and rig.link.current_id is None)   # 01 A6 fix
    assert rig.state.server_state == "Listen"


def test_stop_closes_the_connection_and_the_listener() -> None:
    for rig in _rig():
        client = rig.connect()
        assert wait_until(lambda: rig.state.connected)
        started = time.monotonic()
        rig.server.stop()
        assert time.monotonic() - started < 2.5
        assert_eof(client)
        client.close()
        with pytest.raises(OSError):
            socket.create_connection(rig.address, timeout=1.0).close()


def test_bind_on_a_busy_port_raises(rig: Rig) -> None:
    router: Any = FakeRouter()
    other = TcpServer(host="127.0.0.1", port=rig.address[1], link=ClientLink(), router=router, state=make_state(),
                      datatypes=DT, compat=CompatSwitches(), options=ServerOptions(), tick=FixedClock())
    with pytest.raises(OSError):
        other.bind()
