"""aloxcam.server.link: whole frames under one lock, stale connections, failed writes, close (design §3.4.1)."""

from __future__ import annotations

import socket
import struct
import threading
from collections.abc import Iterator

import pytest

from aloxcam.server.link import ClientLink
from server.wp4_helpers import read_frame


def _frame(tag: int, seq: int, size: int) -> bytes:
    payload = struct.pack(">HH", tag, seq) + bytes([tag & 0xFF]) * size
    return struct.pack(">iH", len(payload) + 2, 1) + payload


@pytest.fixture
def pair() -> Iterator[tuple[socket.socket, socket.socket]]:
    a, b = socket.socketpair()
    a.settimeout(5.0)
    yield a, b
    a.close()
    b.close()


def test_concurrent_large_frames_arrive_whole(pair: tuple[socket.socket, socket.socket]) -> None:
    server_side, client_side = pair
    link = ClientLink()
    conn_id = link.attach(server_side)
    threads_n, frames_n, size = 8, 4, 200_000
    received: list[tuple[int, bytes]] = []

    def reader() -> None:
        for _ in range(threads_n * frames_n):
            frame = read_frame(client_side, timeout=10.0)
            assert frame is not None
            received.append(frame)

    def writer(tag: int) -> None:
        for seq in range(frames_n):
            assert link.send(_frame(tag, seq, size), conn_id)

    r = threading.Thread(target=reader)
    r.start()
    writers = [threading.Thread(target=writer, args=(t,)) for t in range(threads_n)]
    for w in writers:
        w.start()
    for w in writers:
        w.join(10)
    r.join(10)
    assert len(received) == threads_n * frames_n
    seen: dict[int, list[int]] = {}
    for datatype, payload in received:
        tag, seq = struct.unpack(">HH", payload[:4])
        assert datatype == 1 and len(payload) == 4 + size
        assert payload[4:] == bytes([tag & 0xFF]) * size, "frames interleaved"
        seen.setdefault(tag, []).append(seq)
    assert all(seqs == list(range(frames_n)) for seqs in seen.values()) and len(seen) == threads_n


def test_conn_ids_and_stale_replies_dropped(pair: tuple[socket.socket, socket.socket]) -> None:
    server_side, client_side = pair
    link = ClientLink()
    assert link.current_id is None
    first = link.attach(server_side)
    assert first == 1 and link.current_id == 1
    with pytest.raises(RuntimeError):
        link.attach(server_side)                      # the previous connection must be detached first
    link.detach(99)                                   # not current: no-op
    assert link.current_id == 1
    link.detach(first)
    assert link.current_id is None
    second = link.attach(server_side)
    assert second == 2
    assert link.send(b"stale", first) is False        # 02 D10
    assert link.send(b"internal", 0) is False         # internal requests are never replied
    assert link.send(_frame(7, 0, 3), second) is True
    assert read_frame(client_side) == (1, struct.pack(">HH", 7, 0) + b"\x07" * 3)


def test_send_without_connection_returns_false() -> None:
    assert ClientLink().send(b"x", 1) is False


class _FailingSocket:
    def __init__(self) -> None:
        self.sendall_calls = 0
        self.shutdowns: list[int] = []

    def sendall(self, data: bytes) -> None:
        self.sendall_calls += 1
        raise TimeoutError("timed out")

    def shutdown(self, how: int) -> None:
        self.shutdowns.append(how)


def test_failed_send_shuts_the_socket_down() -> None:
    sock = _FailingSocket()
    link = ClientLink()
    conn_id = link.attach(sock)  # type: ignore[arg-type]
    assert link.send(b"frame", conn_id) is False
    assert sock.shutdowns == [socket.SHUT_RDWR]
    assert link.send(b"frame", conn_id) is False      # the broken connection is not written again
    assert sock.sendall_calls == 1


def test_write_timeout_on_a_real_socket_gives_eof_to_the_reader() -> None:
    server_side, client_side = socket.socketpair()
    try:
        server_side.settimeout(0.1)                   # the peer never reads: sendall times out (02 D12)
        link = ClientLink()
        conn_id = link.attach(server_side)
        assert link.send(b"\x00" * (32 * 2**20), conn_id) is False
        client_side.settimeout(5.0)
        total = 0
        while chunk := client_side.recv(1 << 20):    # partial frame, then EOF (shutdown by the link)
            total += len(chunk)
        assert total < 32 * 2**20
    finally:
        server_side.close()
        client_side.close()


def test_close_drops_later_sends(pair: tuple[socket.socket, socket.socket]) -> None:
    server_side, client_side = pair
    link = ClientLink()
    conn_id = link.attach(server_side)
    assert link.send(_frame(1, 0, 1), conn_id)
    assert not link.closed
    link.close()
    assert link.closed
    assert link.send(_frame(1, 1, 1), conn_id) is False
    assert read_frame(client_side) is not None        # only the frame sent before close()
    client_side.settimeout(0.05)
    with pytest.raises(TimeoutError):
        client_side.recv(1)
