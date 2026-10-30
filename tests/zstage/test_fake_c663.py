"""FakeC663 / FakeC663Server: the GCS ASCII protocol on top of SimStage, fault injection, the server and its command
line (docs/phase2-design.md §3.3, §5; 08 §3.3–§3.5; owner-answers §5)."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from collections.abc import Iterator
from pathlib import Path

import pytest

from aloxcam.errors import Code, StageError
from aloxcam.settings import SimulationOptions, StageOptions, ZStageSettings
from aloxcam.zstage.controller import ZController
from aloxcam.zstage.fake_c663 import FAKE_IDENTITY, FakeC663, FakeC663Server
from aloxcam.zstage.gcs_serial import GcsSerialLink
from aloxcam.zstage.pi_serial import PiSerialStage
from aloxcam.zstage.sim import DH_MACRO_TEXT

REPO = Path(__file__).resolve().parents[2]
IDENTITY = "(c) aloxcam SimStage, C-663-SIM, 0000000001, 0.0.0"


class Clock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


@pytest.fixture
def clock() -> Clock:
    return Clock()


@pytest.fixture
def fake(clock: Clock) -> FakeC663:
    return FakeC663(SimulationOptions(stage_reference_s=1.0), clock=clock)


def feed_text(fake: FakeC663, text: str) -> bytes:
    return fake.feed(text.encode("cp1252"))


# ---------------------------------------------------------------------------------------------- protocol
def test_answer_formats(fake: FakeC663) -> None:
    assert feed_text(fake, "*IDN?\n") == IDENTITY.encode() + b"\n"
    assert feed_text(fake, "ERR?\n") == b"0\n"
    assert feed_text(fake, "SAI?\n") == b"1\n" and feed_text(fake, "SAI? ALL\n") == b"1\n"
    assert feed_text(fake, "CST? 1\n") == b"1=SIM-STAGE\n"
    assert feed_text(fake, "POS? 1\n") == b"1=0.000000\n"
    assert feed_text(fake, "VEL? 1\n") == b"1=10.000000\n"
    assert feed_text(fake, "TMN? 1\n") == b"1=0.000000\n" and feed_text(fake, "TMX? 1\n") == b"1=25.000000\n"
    assert feed_text(fake, "FRF? 1\n") == b"1=0\n" and feed_text(fake, "SVO? 1\n") == b"1=0\n"
    assert (feed_text(fake, "RON? 1\n"), feed_text(fake, "TRS? 1\n"), feed_text(fake, "LIM? 1\n")) == \
        (b"1=1\n", b"1=0\n", b"1=1\n")
    assert feed_text(fake, "POS?\n") == b"1=0.000000\n"                     # no axis: every axis
    assert feed_text(fake, "JON? 1\n") == b"1=0\n"
    assert feed_text(fake, "RMC?\n") == b"DH\n"
    assert fake.feed(b"\x05") == b"0\n" and fake.feed(b"\x07") == b"\xb1"   # #7: one byte, no LF
    lines = DH_MACRO_TEXT.split("\n")
    assert feed_text(fake, "MAC? DH\n") == (" \n".join(lines) + "\n").encode()   # " \n" on all lines but the last
    assert feed_text(fake, "MOV 1 1.0000\n") == b""                          # commands have no answer


def test_rmc_answers_an_empty_line_without_macro(clock: Clock) -> None:
    fake = FakeC663(SimulationOptions(), clock=clock, dh_running=False)
    assert feed_text(fake, "RMC?\n") == b"\n"


def test_any_chunking_and_single_bytes_at_once(fake: FakeC663) -> None:
    whole = feed_text(fake, "*IDN?\nERR?\n")
    pieces = b"".join(fake.feed(bytes([b])) for b in b"*IDN?\nERR?\n")
    assert pieces == whole == IDENTITY.encode() + b"\n0\n"
    assert feed_text(fake, "POS") == b""                                   # a partial line is kept…
    assert fake.feed(b"\x05") == b"0\n"                                     # …while #5 is answered at once
    assert feed_text(fake, "? 1\n") == b"1=0.000000\n"
    assert fake.frames[-3:] == ["ERR?", "#5", "POS? 1"]
    fake.clear_frames()
    assert fake.frames == []
    assert fake.processed_bytes == 2 * len(b"*IDN?\nERR?\n") + len(b"POS\x05? 1\n")


def test_errors_unknown_xxx_syntax_and_axis(fake: FakeC663) -> None:
    for frame, code in (("XXX", 2), ("HLT 1", 2), ("MOV 1 abc", 1), ("MOV 1", 1), ("SVO 1 2", 1),
                        ("JAX 1 1", 1), ("MAC START", 1), ("MOV 2 1.0", 15)):
        assert feed_text(fake, frame + "\n") == b"", frame
        assert fake.model.error_register == code, frame
        fake.model.qerr()
    assert feed_text(fake, "err?\n") == b"0\n"                              # lower case, as in the purge
    assert feed_text(fake, "POS? 2\n") == b""                                # failed query: no answer, error 15
    assert feed_text(fake, "MAC? NOPE\n") == b"" and fake.model.qerr() == 20
    assert feed_text(fake, "JON? x\n") == b"" and fake.model.qerr() == 1
    assert feed_text(fake, "\n") == b"" and "" not in fake.frames             # empty lines are ignored


def test_motion_referencing_and_macros_come_from_the_model(fake: FakeC663, clock: Clock) -> None:
    feed_text(fake, "SVO 1 1\nFNL 1\n")
    assert fake.feed(b"\x07") == b"\xb0" and fake.feed(b"\x05") == b"1\n"     # referencing: busy, moving
    clock.now += 1.0
    assert fake.feed(b"\x07") == b"\xb1" and feed_text(fake, "FRF? 1\n") == b"1=1\n"
    feed_text(fake, "VEL 1 5.0000\nMOV 1 10.0000\n")
    clock.now += 1.0
    assert feed_text(fake, "POS? 1\n") == b"1=5.000000\n"
    feed_text(fake, "STP 1\n")
    assert feed_text(fake, "ERR?\nRMC?\n") == b"10\n\n"                      # STP stopped DH too
    feed_text(fake, "MAC START DH\nJAX 1 1 1\nJON 1 1\n")
    assert feed_text(fake, "RMC?\nJON? 1\n") == b"DH\n1=1\n"
    feed_text(fake, "MVR 1 1.0000\n")
    assert fake.model.error_register == 51                                    # joystick on (08 §9)
    fake.feed(b"\x18")
    assert fake.model.error_register == 10 and fake.frames[-1] == "#24"


# ---------------------------------------------------------------------------------------------- fault injection
def test_drop_next_answer_fail_next_and_bad_ready_byte(fake: FakeC663) -> None:
    fake.drop_next_answer()
    assert feed_text(fake, "ERR?\n") == b""
    assert feed_text(fake, "ERR?\n") == b"0\n"
    fake.fail_next("mov", 7)
    assert feed_text(fake, "MOV 1 3.0000\n") == b"" and fake.model.position == 0.0
    assert fake.model.error_register == 7
    fake.fail_next("POS?", 5)
    assert feed_text(fake, "POS? 1\n") == b"" and fake.model.qerr() == 5    # a failed query: no answer
    fake.fail_next("#7", 100)
    assert fake.feed(b"\x07") == b"" and fake.model.qerr() == 100
    fake.bad_ready_byte()
    assert fake.feed(b"\x07") == b"?"
    assert fake.feed(b"\x07") == b"\xb1"
    assert "MOV 1 3.0000" in fake.frames                                      # failed frames are still logged


@pytest.fixture
def server() -> Iterator[FakeC663Server]:
    with FakeC663Server(SimulationOptions(stage_initially_referenced=True)) as srv:
        yield srv


def open_stage(server: FakeC663Server, timeout_ms: int = 2000) -> PiSerialStage:
    stage = PiSerialStage(sleep=lambda _s: None)
    stage.connect_rs232(server.url, 115200, timeout_ms)
    return stage


def test_answer_delay_and_dropped_answer_time_out(server: FakeC663Server) -> None:
    stage = open_stage(server, timeout_ms=300)
    try:
        server.controller.answer_delay_s = 0.6
        with pytest.raises(StageError) as info:
            stage.qpos("1")
        assert info.value.code == Code.PI_TIMEOUT
        server.controller.answer_delay_s = 0.0
        time.sleep(1.0)                                                       # the late answer arrives (0.6 s)…
        assert stage.qvel("1") == 10.0                                        # …and is discarded, no shift
        server.controller.drop_next_answer()
        with pytest.raises(StageError) as info:
            stage.qerr()
        assert info.value.code == -7
        assert stage.qerr() == 0
    finally:
        stage.close()


def test_disconnect_then_reconnect_keeps_the_controller_state(server: FakeC663Server) -> None:
    stage = open_stage(server)
    try:
        stage.mov("1", 3.0)
        stage.stp("1")                                                        # error 10 pending, position kept
        assert server.controller.wait_processed(server.controller.processed_bytes, 1.0)
        server.disconnect()
        deadline = time.monotonic() + 5.0
        while server.connected and time.monotonic() < deadline:
            time.sleep(0.01)
        assert not server.connected
        with pytest.raises(StageError) as info:
            stage.qpos("1")
        assert info.value.code in (Code.PI_RECEIVE_ERROR, Code.PI_SEND_ERROR)
        assert stage.is_connected                                             # the link object stays until closed
        stage.connect_rs232(server.url, 115200, 2000)                          # Z Setup re-opens
        assert stage.qerr() == 0                                              # the purge read (and cleared) it
        assert server.connections == 2 and stage.qfrf("1") is True
    finally:
        stage.close()
    server.disconnect()                                                       # nobody connected: no effect…
    stage = open_stage(server)                                                # …on the next connection
    assert stage.qerr() == 0
    stage.close()


def test_server_life_cycle() -> None:
    srv = FakeC663Server()
    with pytest.raises(RuntimeError):
        _ = srv.url
    assert srv.start() is srv.start()                                         # idempotent
    assert srv.url == f"socket://127.0.0.1:{srv.port}" and srv.address == ("127.0.0.1", srv.port)
    assert srv.connections == 0 and not srv.connected
    with GcsSerialLink(srv.url, 115200, 2.0) as link:
        assert link.query("*IDN?") == IDENTITY
        assert srv.connected and srv.connections == 1
    srv.close()
    srv.close()


def test_one_connection_at_a_time(server: FakeC663Server) -> None:
    first = GcsSerialLink(server.url, 115200, 0.3)
    second = GcsSerialLink(server.url, 115200, 0.3)
    try:
        assert first.query("ERR?") == "0"
        with pytest.raises(StageError):
            second.query("ERR?")                                               # waits in the backlog
        first.close()
        assert second.query("ERR?") == "0"                                     # served once the first has gone
    finally:
        first.close()
        second.close()


def test_zcontroller_over_the_fake_uses_real_time() -> None:
    with FakeC663Server(SimulationOptions(stage_reference_s=0.1)) as srv:
        ctl = ZController(PiSerialStage(), StageOptions(port_map={"COM7": srv.url}))
        ctl.setup(ZStageSettings(com=7, speed=40.0))
        ctl.mov(2.0)
        ctl.general_wait()
        assert ctl.qpos() == 2.0 and srv.controller.model.position == 2.0
        assert ctl.close() is True


# ---------------------------------------------------------------------------------------------- command line
def test_command_line_rehearsal_server(tmp_path: Path) -> None:
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(REPO / "src"), os.environ.get("PYTHONPATH", "")]))
    process = subprocess.Popen(
        [sys.executable, "-m", "aloxcam.zstage.fake_c663", "--port", "0", "--initially-referenced",
         "--log-level", "WARNING"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, cwd=tmp_path)
    try:
        assert process.stdout is not None
        first = process.stdout.readline().decode()
        assert first.startswith("FakeC663 listening on socket://127.0.0.1:"), first
        url = first.split()[-1]
        assert f"[stage] port_map = COM<n>={url}" in process.stdout.readline().decode()
        with GcsSerialLink(url, 115200, 5.0) as link:
            assert link.query("*IDN?") == FAKE_IDENTITY
            assert link.query("FRF? 1") == "1=1" and link.query("TMX? 1") == "1=100.000000"
        if sys.platform == "win32":
            process.kill()
        else:
            process.send_signal(signal.SIGINT)                                 # Ctrl+C
            assert process.wait(10) == 0
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(10)


@pytest.mark.skipif(sys.platform == "win32", reason="POSIX signal delivery")
def test_command_line_stops_on_sigterm(tmp_path: Path) -> None:
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(REPO / "src"), os.environ.get("PYTHONPATH", "")]))
    process = subprocess.Popen([sys.executable, "-m", "aloxcam.zstage.fake_c663", "--port", "0", "--log-level",
                                "WARNING"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, cwd=tmp_path)
    try:
        assert process.stdout is not None
        assert process.stdout.readline().decode().startswith("FakeC663 listening on ")
        process.terminate()
        assert process.wait(10) == 0
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(10)
