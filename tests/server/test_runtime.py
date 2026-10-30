"""aloxcam.server.runtime: start, the Exit / signal shutdown chain, dropped replies, Z wait cancellation
(01 §7, design §3.4.6). Fake handler tables only; loopback and ephemeral ports."""

from __future__ import annotations

import socket
import threading
from collections.abc import Iterator
from dataclasses import replace

import pytest

from aloxcam.dispatch import ReplyKind
from aloxcam.errors import NO_ERROR
from aloxcam.server.runtime import ServerRuntime
from aloxcam.wire.messages import ZWaitData
from server.wp4_helpers import TIMEOUT, Fakes, assert_eof, command_frame, make_parts, read_reply, wait_until


@pytest.fixture
def fakes() -> Fakes:
    return Fakes()


@pytest.fixture
def runtime(fakes: Fakes) -> Iterator[ServerRuntime]:
    rt = ServerRuntime(make_parts(fakes))
    yield rt
    if rt.started and not rt.wait(0):
        fakes.release_wait.set()
        rt.request_shutdown("test teardown")
        assert rt.wait(TIMEOUT)


def connect(rt: ServerRuntime) -> socket.socket:
    sock = socket.create_connection(rt.address, timeout=TIMEOUT)
    sock.settimeout(TIMEOUT)
    return sock


def test_start_serves_and_client_exit_runs_the_chain(runtime: ServerRuntime, fakes: Fakes) -> None:
    host, port = runtime.start()
    assert (host, port) == runtime.address and port > 0
    assert not runtime.shutdown_started and not runtime.wait(0)
    with connect(runtime) as client:
        client.sendall(command_frame("Get Heartbeat", notifier=b"HB"))
        notifier, value, err, _ = read_reply(client, ReplyKind.U32)
        assert notifier == b"HB" and err == NO_ERROR and isinstance(value, int)
        client.sendall(command_frame("Z Get Position", notifier=b"ZP"))
        assert read_reply(client, ReplyKind.DBL)[:3] == (b"ZP", 12.5, NO_ERROR)
        client.sendall(command_frame("Exit"))
        assert runtime.wait(TIMEOUT)
        assert_eof(client)                                     # no reply to Exit; connection closed
    assert runtime.shutdown_started and runtime.link.closed
    assert fakes.order == ["camera Exit", "Z Exit", "close_hardware"]
    assert runtime.state.commands_history[0] == "Exit"
    assert not runtime.state.connected


def test_request_shutdown_runs_the_same_chain(runtime: ServerRuntime, fakes: Fakes) -> None:
    runtime.start()
    runtime.request_shutdown("test")
    assert runtime.shutdown_started
    runtime.request_shutdown("again")                          # idempotent
    assert runtime.wait(TIMEOUT)
    assert fakes.order == ["camera Exit", "Z Exit", "close_hardware"]
    with pytest.raises(OSError):
        socket.create_connection(runtime.address, timeout=1.0).close()


def test_replies_after_shutdown_are_dropped(runtime: ServerRuntime, fakes: Fakes) -> None:
    runtime.start()
    with connect(runtime) as client:
        client.sendall(command_frame("Z Wait End Move", ZWaitData(), notifier=b"W"))
        assert fakes.wait_started.wait(TIMEOUT)
        client.sendall(command_frame("Exit"))
        assert wait_until(lambda: runtime.link.closed, TIMEOUT)
        fakes.release_wait.set()                               # the wait ends after the link was closed
        assert runtime.wait(TIMEOUT)
        assert_eof(client)                                     # its reply was lost, as in LabVIEW (01 §6.3)
    assert fakes.order == ["camera Exit", "Z wait ended", "Z Exit", "close_hardware"]


def test_z_handler_blocked_past_the_timeout_is_aborted(fakes: Fakes) -> None:
    runtime = ServerRuntime(make_parts(fakes, shutdown_timeout_s=0.2))
    runtime.start()
    with connect(runtime) as client:
        client.sendall(command_frame("Z Wait End Move", ZWaitData(), notifier=b"W"))
        assert fakes.wait_started.wait(TIMEOUT)
        client.sendall(command_frame("Exit"))
        assert runtime.wait(TIMEOUT)
    assert fakes.order == ["camera Exit", "abort_waits", "Z wait ended", "Z Exit", "close_hardware"]


def test_z_worker_without_exit_is_ended_after_the_timeout(fakes: Fakes) -> None:
    runtime = ServerRuntime(make_parts(fakes, shutdown_timeout_s=0.1, z_exit=False))
    runtime.start()
    runtime.request_shutdown()
    assert runtime.wait(TIMEOUT)
    assert runtime.z_worker.stopped.is_set()
    assert fakes.order == ["camera Exit", "abort_waits", "close_hardware"]


def test_shutdown_started_follows_the_camera_worker(runtime: ServerRuntime, fakes: Fakes) -> None:
    runtime.start()
    assert not runtime.shutdown_started
    with connect(runtime) as client:
        client.sendall(command_frame("Exit"))
        assert wait_until(lambda: runtime.shutdown_started, TIMEOUT)
    assert runtime.wait(TIMEOUT)


def test_background_abort_and_hardware_errors_are_contained(fakes: Fakes) -> None:
    aborted = threading.Event()

    class Background:
        def has_work(self) -> bool:
            return False

        def step(self) -> None:
            pass

        def abort(self) -> None:
            aborted.set()
            raise RuntimeError("abort failed")

    def broken_close() -> None:
        raise RuntimeError("cannot close")

    parts = replace(make_parts(fakes), camera_background=Background(), close_hardware=broken_close)
    runtime = ServerRuntime(parts)
    runtime.start()
    runtime.request_shutdown()
    assert runtime.wait(TIMEOUT) and aborted.is_set()


def test_start_twice_and_bind_failure(runtime: ServerRuntime, fakes: Fakes) -> None:
    runtime.start()
    with pytest.raises(RuntimeError):
        runtime.start()
    busy = ServerRuntime(replace(make_parts(Fakes()), port=runtime.address[1]))
    with pytest.raises(OSError):
        busy.start()
    assert not busy.started and busy.camera_worker.join(0)    # nothing was started
