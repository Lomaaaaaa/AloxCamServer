"""GcsSerialLink against a loopback socket peer (docs/phase2-design.md §3.1, §5; 08 §3.1–§3.3, §3.5, §13.5)."""

from __future__ import annotations

import logging
import socket
import sys
import threading
import time
import types
from collections.abc import Iterator

import pytest

from aloxcam.errors import Code, StageError
from aloxcam.zstage.gcs_serial import (
    FRAME_LOG_SIZE,
    PYSERIAL_MISSING,
    SERIAL_LOGGER,
    GcsSerialLink,
    import_pyserial,
    pyserial_version,
)
from zstage.fake_probe import ScriptedPeer

TIMEOUT_S = 0.2


@pytest.fixture
def peer() -> Iterator[ScriptedPeer]:
    answers: dict[bytes, object] = {
        b"POS? 1\n": b"  1=12.5\n",                          # leading spaces are kept
        b"SAI?\n": b"1 \n2 \n3\n",                           # multi-line: " \n" on every line but the last
        b"CHUNKS?\n": [b"1=1 \n2", b"=0 \n", b"3=1\n"],     # one answer in three TCP segments, 20 ms apart
        b"RMC?\n": b"\n",                                    # empty answer
        b"\x05": b"1\n",                                     # #5: hex mask + LF
        b"\x07": b"\xb1",                                    # #7: one byte, no LF
        b"HALF?\n": b"1=1 \n",                               # a continuation line, then nothing
        b"TEMP?\n": b"\xb0C\n",                              # cp1252: 0xB0 = degree sign
        b"\xe9t\xe9\n": b"ok\n",
    }
    with ScriptedPeer(answers) as p:   # type: ignore[arg-type]
        yield p


@pytest.fixture
def link(peer: ScriptedPeer) -> Iterator[GcsSerialLink]:
    with GcsSerialLink(peer.url, 115200, TIMEOUT_S) as lk:
        yield lk


def test_text_command_gets_an_lf(peer: ScriptedPeer, link: GcsSerialLink) -> None:
    link.send("MOV 1 12.5000")
    assert peer.wait_frames(1)
    assert peer.frames == [b"MOV 1 12.5000\n"]
    assert list(link.frames) == [("tx", b"MOV 1 12.5000\n")]


def test_single_byte_commands_have_no_lf(peer: ScriptedPeer, link: GcsSerialLink) -> None:
    link.send_char(0x18)                                     # #24: no answer
    assert peer.wait_frames(1)
    assert bytes(peer.received) == b"\x18"
    with pytest.raises(ValueError):
        link.send_char(256)


def test_receive_removes_only_the_final_lf(link: GcsSerialLink) -> None:
    assert link.query("POS? 1") == "  1=12.5"
    assert link.query("SAI?") == "1 \n2 \n3"                 # continuation spaces kept (08 §3.3)
    assert link.query("CHUNKS?") == "1=1 \n2=0 \n3=1"
    assert link.query("RMC?") == ""
    assert list(link.frames)[-2:] == [("tx", b"RMC?\n"), ("rx", b"\n")]


def test_hash5_reads_a_line_and_hash7_one_byte(peer: ScriptedPeer, link: GcsSerialLink) -> None:
    assert link.query_char(0x05) == "1"
    assert link.query_char(0x07) == "\xb1"                   # no LF awaited (a line read would end in -7)
    assert bytes(peer.received) == b"\x05\x07"
    assert list(link.frames)[-2:] == [("tx", b"\x07"), ("rx", b"\xb1")]


def test_timeout_is_minus_7(link: GcsSerialLink) -> None:
    t0 = time.monotonic()
    with pytest.raises(StageError) as info:
        link.query("SILENT?")
    assert time.monotonic() - t0 >= TIMEOUT_S * 0.9
    assert (info.value.code, info.value.source) == (-7, f"Data Receive Timeout (RS-232, {link.port}, baud 115200)")
    with pytest.raises(StageError) as info:                  # an unfinished multi-line answer times out too
        link.query("HALF?")
    assert info.value.code == Code.PI_TIMEOUT
    assert list(link.frames)[-1] == ("rx", b"1=1 \n")        # partial bytes recorded


def test_hash7_timeout() -> None:
    with ScriptedPeer({}) as silent, GcsSerialLink(silent.url, 9600, TIMEOUT_S) as quiet:
        with pytest.raises(StageError) as info:
            quiet.query_char(0x07)
        assert (info.value.code, info.value.source) == (-7, f"Data Receive Timeout (RS-232, {silent.url}, baud 9600)")


def test_cp1252_bytes(peer: ScriptedPeer, link: GcsSerialLink) -> None:
    assert link.query("TEMP?") == "°C"
    assert link.query("été") == "ok"
    assert b"\xe9t\xe9\n" in peer.frames


def test_lf_after_the_ready_byte_is_dropped_quietly(caplog: pytest.LogCaptureFixture) -> None:
    with ScriptedPeer({b"\x07": b"\xb0\n", b"ERR?\n": b"0\n"}) as peer, \
            GcsSerialLink(peer.url, 115200, TIMEOUT_S) as link:
        assert link.query_char(0x07) == "\xb0"
        time.sleep(0.05)                                     # the LF has arrived
        with caplog.at_level(logging.WARNING, logger=SERIAL_LOGGER):
            assert link.query("ERR?") == "0"                 # not the stray LF
        assert ("drop", b"\n") in link.frames
        assert caplog.records == []


def test_late_answer_is_discarded_before_the_next_command(caplog: pytest.LogCaptureFixture) -> None:
    sent = threading.Event()

    def respond(frame: bytes) -> bytes | None:
        if frame == b"SLOW?\n":
            time.sleep(TIMEOUT_S * 2)                        # answers after the driver gave up
            sent.set()
            return b"late\n"
        return b"0\n" if frame == b"ERR?\n" else None

    with ScriptedPeer(respond) as peer, GcsSerialLink(peer.url, 115200, TIMEOUT_S) as link:
        with pytest.raises(StageError):
            link.query("SLOW?")
        assert sent.wait(5.0)
        time.sleep(0.2)                                      # loopback delivery
        with caplog.at_level(logging.WARNING, logger=SERIAL_LOGGER):
            assert link.query("ERR?") == "0"                 # not "late": no answer shift
        assert ("drop", b"late\n") in link.frames
        assert any("discarded" in r.getMessage() for r in caplog.records)


def test_discard_input() -> None:
    with ScriptedPeer({b"\x07": b"\xb1junk"}) as peer, GcsSerialLink(peer.url, 115200, TIMEOUT_S) as link:
        assert link.query_char(0x07) == "\xb1"
        deadline = time.monotonic() + 2.0
        data = b""
        while not data and time.monotonic() < deadline:
            data = link.discard_input()
        assert data == b"junk" and link.frames[-1] == ("drop", b"junk")
        assert link.discard_input() == b""


def test_frames_are_bounded_and_logged(link: GcsSerialLink, caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.DEBUG, logger=SERIAL_LOGGER):
        link.send("VEL 1 1.0000")
    assert any(r.name == SERIAL_LOGGER and ">" in r.getMessage() and "VEL 1 1.0000" in r.getMessage()
               for r in caplog.records)
    for i in range(FRAME_LOG_SIZE + 10):
        link.send(f"MOV 1 {i}.0000")
    assert len(link.frames) == FRAME_LOG_SIZE and link.frames[-1] == ("tx", f"MOV 1 {FRAME_LOG_SIZE + 9}.0000\n"
                                                                       .encode())


def test_close_is_idempotent_and_a_closed_link_is_minus_4(link: GcsSerialLink) -> None:
    assert link.is_open
    link.close()
    link.close()
    assert not link.is_open
    for call in (lambda: link.send("ERR?"), link.receive, lambda: link.query_char(5), link.discard_input):
        with pytest.raises(StageError) as info:
            call()
        assert info.value.code == Code.PI_NOT_CONNECTED and info.value.source.endswith(": not connected")


def test_peer_gone_is_minus_3_or_minus_2() -> None:
    with ScriptedPeer({b"ERR?\n": b"0\n"}) as peer, GcsSerialLink(peer.url, 115200, TIMEOUT_S) as link:
        assert link.query("ERR?") == "0"
        assert peer.wait_connected()
        peer.drop_connection()
        time.sleep(0.05)
        with pytest.raises(StageError) as info:
            link.query("ERR?")
        assert info.value.code in (Code.PI_RECEIVE_ERROR, Code.PI_SEND_ERROR)
        assert info.value.source.startswith(("Data Receive Error (RS-232, ", "Data Send Error (RS-232, "))


def test_unopenable_port_is_minus_6() -> None:
    with socket.create_server(("127.0.0.1", 0)) as s:
        port = s.getsockname()[1]                            # closed below: connection refused
    with pytest.raises(StageError) as info:
        GcsSerialLink(f"socket://127.0.0.1:{port}", 115200, TIMEOUT_S)
    assert info.value.code == Code.PI_OPEN_PORT
    assert info.value.source.startswith(f"Open RS232 socket://127.0.0.1:{port} @ 115200: ")
    with pytest.raises(StageError) as info:
        GcsSerialLink("/dev/aloxcam-no-such-port" if sys.platform != "win32" else "COM255", 115200, TIMEOUT_S)
    assert info.value.code == -6


def test_without_pyserial_the_link_is_minus_6(monkeypatch: pytest.MonkeyPatch) -> None:
    assert pyserial_version() is not None
    monkeypatch.setitem(sys.modules, "serial", None)
    with pytest.raises(StageError) as info:
        GcsSerialLink("COM4", 115200, 5.0)
    assert (info.value.code, info.value.source) == (-6, PYSERIAL_MISSING)
    assert "pip install pyserial" in PYSERIAL_MISSING and "aloxcam[stage]" in PYSERIAL_MISSING
    assert pyserial_version() is None
    monkeypatch.setitem(sys.modules, "serial", types.ModuleType("serial"))    # the other "serial" package
    with pytest.raises(StageError) as info:
        import_pyserial()
    assert "not pyserial" in info.value.source
