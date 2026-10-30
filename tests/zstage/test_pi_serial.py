"""PiSerialStage: exact frames, purge, answer parsing, driver pre-checks and errors (docs/phase2-design.md §3.2, §5;
08 §3.2–§3.5, §11, §13.2; 08 Z7, Z22)."""

from __future__ import annotations

import logging
import socket
from collections.abc import Callable, Iterator
from typing import Any

import pytest

from aloxcam.errors import Code, StageError
from aloxcam.settings import SimulationOptions, StageOptions, ZStageSettings
from aloxcam.zstage.backend import StageBackend
from aloxcam.zstage.controller import ZController
from aloxcam.zstage.fake_c663 import FakeC663Server
from aloxcam.zstage.gcs_serial import GcsSerialLink
from aloxcam.zstage.pi_serial import (
    ASSIGN_VALUES_TEXT,
    OPEN_PAUSE_S,
    PURGE_PAUSE_S,
    PiSerialStage,
    axis_value,
    parse_axis_values,
    parse_error_code,
)
from aloxcam.zstage.sim import DH_MACRO_TEXT, HOME_MACRO_TEXT, SimStage
from zstage.fake_probe import ScriptedPeer

TIMEOUT_MS = 2000


@pytest.fixture
def fake() -> Iterator[FakeC663Server]:
    with FakeC663Server(SimulationOptions(stage_initially_referenced=True, stage_reference_s=0.0)) as server:
        yield server


def open_stage(url: str, *, sleeps: list[float] | None = None, read_axes: bool = True) -> PiSerialStage:
    stage = PiSerialStage(sleep=(sleeps.append if sleeps is not None else lambda _s: None))
    stage.connect_rs232(url, 115200, TIMEOUT_MS)
    if read_axes:
        stage.qsai()
    return stage


def tx(stage: PiSerialStage) -> list[bytes]:
    assert stage.link is not None
    return [data for direction, data in stage.link.frames if direction == "tx"]


# ---------------------------------------------------------------------------------------------- open and purge
def test_purge_sequence_and_returned_code(fake: FakeC663Server) -> None:
    sleeps: list[float] = []
    stage = open_stage(fake.url, sleeps=sleeps, read_axes=False)
    try:
        assert tx(stage) == [b"XXX\n", b"*idn?\n", b"err?\n"]           # 08 §3.2, this spelling
        assert sleeps == [OPEN_PAUSE_S, PURGE_PAUSE_S] == [0.05, 0.01]
        assert fake.controller.model.error_register == 0                 # err? cleared XXX's error 2
        assert stage.is_connected and stage.link is not None and stage.link.timeout_s == 2.0
        fake.controller.model.stp("1")                                   # a pending 10 is overwritten by XXX's 2
        assert stage.connect_rs232(fake.url, 115200, TIMEOUT_MS) == 2    # (only the last error is kept)
    finally:
        stage.close()


def test_purge_returns_the_code_err_answers() -> None:
    answers = {b"*idn?\n": b"PI C-663\n", b"err?\n": b"10\n"}
    with ScriptedPeer(answers) as peer:   # type: ignore[arg-type]
        stage = PiSerialStage(sleep=lambda _s: None)
        assert stage.connect_rs232(peer.url, 115200, TIMEOUT_MS) == 10
        assert peer.frames == [b"XXX\n", b"*idn?\n", b"err?\n"]
        stage.close()
    with ScriptedPeer({b"*idn?\n": b"PI\n", b"err?\n": b"\n"}) as peer:  # type: ignore[arg-type]
        stage = PiSerialStage(sleep=lambda _s: None)
        assert stage.connect_rs232(peer.url, 115200, TIMEOUT_MS) == 0    # discarded, not checked (08 §3.2)
        stage.close()


def test_connect_closes_the_previous_link(fake: FakeC663Server) -> None:
    stage = open_stage(fake.url)
    first = stage.link
    stage.connect_rs232(fake.url, 115200, TIMEOUT_MS)
    assert first is not None and not first.is_open and stage.link is not first
    assert stage.connected_axes is None                                  # SAI? not read on the new link yet
    assert stage.close() is True and stage.close() is False
    assert not stage.is_connected and stage.link is None


def test_link_factory_and_timeout() -> None:
    calls: list[tuple[str, int, float]] = []

    def factory(port: str, baudrate: int, timeout_s: float) -> GcsSerialLink:
        calls.append((port, baudrate, timeout_s))
        raise StageError(Code.PI_OPEN_PORT, f"Open RS232 {port}")

    stage = PiSerialStage(factory, sleep=lambda _s: None)
    with pytest.raises(StageError) as info:
        stage.connect_rs232("COM4", 115200, 5000)
    assert calls == [("COM4", 115200, 5.0)] and info.value.code == -6
    assert not stage.is_connected


def test_unopenable_port_is_minus_6() -> None:
    with socket.create_server(("127.0.0.1", 0)) as s:
        port = s.getsockname()[1]
    stage = PiSerialStage(sleep=lambda _s: None)
    with pytest.raises(StageError) as info:
        stage.connect_rs232(f"socket://127.0.0.1:{port}", 115200, TIMEOUT_MS)
    assert info.value.code == Code.PI_OPEN_PORT and not stage.is_connected


def test_purge_timeout_leaves_the_link_open() -> None:
    with ScriptedPeer({}) as peer:
        stage = PiSerialStage(sleep=lambda _s: None)
        with pytest.raises(StageError) as info:
            stage.connect_rs232(peer.url, 115200, 200)
        assert info.value.code == Code.PI_TIMEOUT
        assert info.value.source == f"Data Receive Timeout (RS-232, {peer.url}, baud 115200)"
        assert stage.is_connected                                        # the VISA session stays open in LabVIEW
        stage.close()


# ---------------------------------------------------------------------------------------------- frame catalogue
FRAMES: list[tuple[str, tuple[Any, ...], bytes]] = [             # 08 §3.4 and §4.2, axis "1"
    ("qerr", (), b"ERR?\n"),
    ("qidn", (), b"*IDN?\n"),
    ("qsai", (), b"SAI?\n"),
    ("qcst", ("1",), b"CST? 1\n"),
    ("svo", ("1", True), b"SVO 1 1\n"),
    ("svo", ("1", False), b"SVO 1 0\n"),
    ("qsvo", ("1",), b"SVO? 1\n"),
    ("qron", ("1",), b"RON? 1\n"),
    ("qtrs", ("1",), b"TRS? 1\n"),
    ("qlim", ("1",), b"LIM? 1\n"),
    ("qtmn", ("1",), b"TMN? 1\n"),
    ("qtmx", ("1",), b"TMX? 1\n"),
    ("mov", ("1", 12.5), b"MOV 1 12.5000\n"),
    ("mov", ("1", 1.23456), b"MOV 1 1.2346\n"),
    ("mvr", ("1", -0.25), b"MVR 1 -0.2500\n"),
    ("vel", ("1", 10.0), b"VEL 1 10.0000\n"),
    ("vel", ("1", 0.0), b"VEL 1 0.0000\n"),
    ("qvel", ("1",), b"VEL? 1\n"),
    ("qpos", ("1",), b"POS? 1\n"),
    ("qfrf", ("1",), b"FRF? 1\n"),
    ("frf", ("1",), b"FRF 1\n"),
    ("fnl", ("1",), b"FNL 1\n"),
    ("fpl", ("1",), b"FPL 1\n"),
    ("stp", ("1",), b"STP 1\n"),
    ("stp", (None,), b"STP\n"),
    ("qrmc", (), b"RMC?\n"),
    ("mac_start", ("DH",), b"MAC START DH\n"),
    ("qmac", ("DH",), b"MAC? DH\n"),
    ("jax", (1, 1, "1"), b"JAX 1 1 1\n"),
    ("jon", (1, True), b"JON 1 1\n"),
    ("jon", (1, False), b"JON 1 0\n"),
    ("qjon", (1,), b"JON? 1\n"),
    ("is_moving", (), b"\x05"),
    ("is_controller_ready", (), b"\x07"),
    ("stop_all", (), b"\x18"),
]


@pytest.mark.parametrize(("method", "args", "frame"), FRAMES, ids=[f"{m}{a}" for m, a, _ in FRAMES])
def test_exact_frame_of_every_method(fake: FakeC663Server, method: str, args: tuple[Any, ...], frame: bytes) -> None:
    stage = open_stage(fake.url)
    try:
        assert stage.link is not None
        stage.link.frames.clear()
        fake.controller.clear_frames()
        getattr(stage, method)(*args)
        assert tx(stage) == [frame]                                       # nothing else: no ERR?, no CSV?
    finally:
        stage.close()


def test_every_backend_method_is_in_the_frame_table() -> None:
    covered = {m for m, _, _ in FRAMES} | {"connect_rs232", "close", "is_connected", "translate_error"}
    assert covered == set(StageBackend.__abstractmethods__)


# ---------------------------------------------------------------------------------------------- answers
def test_answers_from_the_fake(fake: FakeC663Server) -> None:
    stage = open_stage(fake.url)
    try:
        assert stage.qidn() == "(c) aloxcam SimStage, C-663-SIM, 0000000001, 0.0.0"
        assert stage.qsai() == ["1"] and stage.connected_axes == ("1",)
        assert stage.qcst("1") == "SIM-STAGE"
        assert (stage.qsvo("1"), stage.qron("1"), stage.qtrs("1"), stage.qlim("1")) == (True, True, False, True)
        assert (stage.qtmn("1"), stage.qtmx("1")) == (0.0, 25.0)
        assert (stage.qpos("1"), stage.qvel("1"), stage.qfrf("1")) == (0.0, 10.0, True)
        assert stage.qerr() == 0 and stage.qjon(1) == 0
        assert stage.qrmc() == "DH"
        assert stage.qmac("DH") == DH_MACRO_TEXT and stage.qmac("HOME") == HOME_MACRO_TEXT
        assert stage.is_moving() is False and stage.is_controller_ready() is True
        stage.mov("1", 5.0)
        assert stage.is_moving() is True
        stage.stop_all()                                                 # #24: stops the move and DH, latches 10
        assert stage.qerr() == 10 and stage.qrmc() == ""
        stage.jax(1, 1, "1")
        stage.jon(1, True)
        assert stage.qjon(1) == 1
        assert stage.translate_error(10) == "Controller was stopped by command"
        assert stage.translate_error(-3) == "Error while receiving data" and stage.translate_error(4242) == ""
    finally:
        stage.close()


@pytest.mark.parametrize(("answer", "value"), [
    (b"1=12.500000\n", 12.5), (b"1 = 12.5\n", 12.5), (b"  1=1.25E+01\n", 12.5), (b"1=+12.5\n", 12.5),
    (b"1=12\n", 12.0), (b"1=-0.0001\n", -0.0001), (b"2=3.0 \n1=12.5\n", 12.5), (b"1=12.5 \n1=99\n", 12.5),
])
def test_any_axis_value_format_is_accepted(answer: bytes, value: float) -> None:
    with ScriptedPeer({b"*idn?\n": b"PI\n", b"err?\n": b"0\n", b"POS? 1\n": answer}) as peer:
        stage = open_stage(peer.url, read_axes=False)
        assert stage.qpos("1") == value
        stage.close()


@pytest.mark.parametrize(("answer", "source"), [
    (b"\n", f"POS?: {ASSIGN_VALUES_TEXT}"),
    (b"\t\n", f"POS?: {ASSIGN_VALUES_TEXT}"),                  # (" \n" would announce another line)
    (b"2=1.0\n", f"POS?: {ASSIGN_VALUES_TEXT} (axis 1 not in the answer '2=1.0')"),
    (b"1=abc\n", "POS?: Assign values: 'abc' is not a number"),
    (b"1=nan\n", "POS?: Assign values: 'nan' is not a number"),
])
def test_bad_answers_are_error_100(answer: bytes, source: str) -> None:
    with ScriptedPeer({b"*idn?\n": b"PI\n", b"err?\n": b"0\n", b"POS? 1\n": answer}) as peer:
        stage = open_stage(peer.url, read_axes=False)
        with pytest.raises(StageError) as info:
            stage.qpos("1")
        assert (info.value.code, info.value.source) == (100, source)
        stage.close()


def test_err_cst_sai_hash5_hash7_answers(caplog: pytest.LogCaptureFixture) -> None:
    script = {b"*idn?\n": b"PI\n", b"err?\n": b"0\n", b"ERR?\n": [b"1008\n"], b"CST? 1\n": b"1 = L-509.40SD\n",
              b"SAI?\n": b"1 \n2\n", b"\x05": b"0x3\n", b"\x07": b"\xb0", b"FRF? 1\n": b"1=0\n"}
    with ScriptedPeer(script) as peer:   # type: ignore[arg-type]
        stage = open_stage(peer.url, read_axes=False)
        assert stage.qerr() == 1008
        assert stage.qcst("1") == "L-509.40SD"
        assert stage.qsai() == ["1", "2"] and stage.connected_axes == ("1", "2")
        assert stage.is_moving() is True and stage.is_controller_ready() is False and stage.qfrf("1") is False
        stage.close()
    for answer, outcome in ((b"\n", "No answer on ERR?"), (b"abc\n", 0), (b" -7\n", -7)):
        with ScriptedPeer({b"*idn?\n": b"PI\n", b"err?\n": b"0\n", b"ERR?\n": answer}) as peer:
            stage = open_stage(peer.url, read_axes=False)
            if isinstance(outcome, str):
                with pytest.raises(StageError) as info:
                    stage.qerr()
                assert (info.value.code, info.value.source) == (100, outcome)
            else:
                with caplog.at_level(logging.WARNING, logger="aloxcam.zstage.pi_serial"):
                    assert stage.qerr() == outcome
            stage.close()
    assert any("unexpected answer 'abc'" in r.getMessage() for r in caplog.records)
    with ScriptedPeer({b"*idn?\n": b"PI\n", b"err?\n": b"0\n", b"\x05": b"zz\n"}) as peer:
        stage = open_stage(peer.url, read_axes=False)
        with pytest.raises(StageError) as info:
            stage.is_moving()
        assert (info.value.code, info.value.source) == (100, "#5: unexpected answer 'zz'")
        stage.close()


def test_unknown_ready_reply(fake: FakeC663Server) -> None:
    stage = open_stage(fake.url)
    try:
        fake.controller.bad_ready_byte()
        with pytest.raises(StageError) as info:
            stage.is_controller_ready()
        assert (info.value.code, info.value.source) == (100, "#7 Unknown reply")
        assert stage.is_controller_ready() is True                       # the next one is normal
    finally:
        stage.close()


def test_parsers() -> None:
    assert parse_axis_values("1=12.5 \n 2 = 3 \nnoise\n=5\n1=7") == {"1": "12.5", "2": "3"}
    assert axis_value("A=1", "A", "X?") == "1"
    assert parse_error_code(" 15") == 15 and parse_error_code("") == 0 and parse_error_code("x1") == 0


# ---------------------------------------------------------------------------------------------- pre-checks, -4
@pytest.mark.parametrize(("call", "text"), [
    (lambda s: s.mov("", 1.0), "MOV No axis specified"),
    (lambda s: s.mvr("", 1.0), "MVR no axis specified"),
    (lambda s: s.vel("", 1.0), "VEL No axis or velocity specified"),
    (lambda s: s.stp(""), "STP no axis specified"),
    (lambda s: s.qpos(""), "POS? No axis specified"),
    (lambda s: s.jax(1, 1, ""), "JAX No axis specified"),
    (lambda s: s.mac_start(""), "MAC START no macro name specified"),
    (lambda s: s.mov("2", 1.0), "Commanded axis not connected: 2"),
    (lambda s: s.qfrf("2"), "Commanded axis not connected: 2"),
    (lambda s: s.fnl("A"), "Commanded axis not connected: A"),
    (lambda s: s.jax(1, 1, "3"), "Commanded axis not connected: 3"),
])
def test_driver_pre_checks_send_nothing(fake: FakeC663Server, call: Callable[[PiSerialStage], object],
                                        text: str) -> None:
    stage = open_stage(fake.url)
    try:
        assert stage.link is not None
        stage.link.frames.clear()
        with pytest.raises(StageError) as info:
            call(stage)
        assert (info.value.code, info.value.source) == (100, text)
        assert list(stage.link.frames) == []
    finally:
        stage.close()


def test_any_axis_before_sai(fake: FakeC663Server) -> None:
    stage = open_stage(fake.url, read_axes=False)
    try:
        stage.mov("2", 1.0)                                              # no SAI? list yet: sent, refused (15)
        assert tx(stage)[-1] == b"MOV 2 1.0000\n"
        assert stage.qerr() == 15
    finally:
        stage.close()


NOT_CONNECTED = [
    (lambda s: s.qerr(), "ERR?"), (lambda s: s.qidn(), "*IDN?"), (lambda s: s.qsai(), "SAI?"),
    (lambda s: s.qcst("1"), "CST?"), (lambda s: s.svo("1", True), "SVO"), (lambda s: s.qsvo("1"), "SVO?"),
    (lambda s: s.qron("1"), "RON?"), (lambda s: s.qtrs("1"), "TRS?"), (lambda s: s.qlim("1"), "LIM?"),
    (lambda s: s.qtmn("1"), "TMN?"), (lambda s: s.qtmx("1"), "TMX?"), (lambda s: s.mov("1", 1.0), "MOV"),
    (lambda s: s.mvr("1", 1.0), "MVR"), (lambda s: s.vel("1", 1.0), "VEL"), (lambda s: s.qvel("1"), "VEL?"),
    (lambda s: s.qpos("1"), "POS?"), (lambda s: s.qfrf("1"), "FRF?"), (lambda s: s.frf("1"), "FRF"),
    (lambda s: s.fnl("1"), "FNL"), (lambda s: s.fpl("1"), "FPL"), (lambda s: s.stp("1"), "STP"),
    (lambda s: s.stp(None), "STP"), (lambda s: s.qrmc(), "RMC?"), (lambda s: s.mac_start("DH"), "MAC START"),
    (lambda s: s.qmac("DH"), "MAC?"), (lambda s: s.jax(1, 1, "1"), "JAX"), (lambda s: s.jon(1, True), "JON"),
    (lambda s: s.qjon(1), "JON?"), (lambda s: s.is_moving(), "#5"), (lambda s: s.is_controller_ready(), "#7"),
    (lambda s: s.stop_all(), "#24"),
]


@pytest.mark.parametrize(("call", "command"), NOT_CONNECTED, ids=[c for _, c in NOT_CONNECTED])
def test_not_connected_is_minus_4_as_in_simstage(call: Callable[[Any], object], command: str) -> None:
    errors = []
    for stage in (PiSerialStage(), SimStage(SimulationOptions())):
        with pytest.raises(StageError) as info:
            call(stage)
        errors.append((info.value.code, info.value.source))
    assert errors[0] == errors[1] == (-4, f"{command}: not connected")


# ---------------------------------------------------------------------------------------------- ZController
def test_connect_purge_own_error_is_expected(fake: FakeC663Server, caplog: pytest.LogCaptureFixture) -> None:
    ctl = ZController(PiSerialStage(sleep=lambda _s: None), StageOptions(port_map={"COM2": fake.url}))
    settings = ZStageSettings(com=2)
    ctl.setup(settings)
    ctl.stp()                                                            # 10 pending, then overwritten by XXX's 2
    with caplog.at_level(logging.DEBUG, logger="aloxcam.zstage.controller"):
        ctl.setup(settings)
    messages = [(r.levelno, r.getMessage()) for r in caplog.records]
    assert (logging.DEBUG, "Z Setup: the connect purge read error 2 (Unknown command), latched by its own XXX") \
        in messages
    assert not any(level >= logging.WARNING for level, _ in messages)
    assert fake.controller.model.error_register == 0
    ctl.close()


def test_raw_query_and_send_for_lab_tools(fake: FakeC663Server) -> None:
    stage = open_stage(fake.url)
    try:
        assert stage.raw_query("TMX? 1") == "1=25.000000"                 # the raw answer, for the check report
        stage.raw_send("VEL 1 5.0000")
        assert stage.qvel("1") == 5.0
        stage.raw_send("HLT 1")                                          # not modelled by the fake: error 2
        assert stage.qerr() == 2
    finally:
        stage.close()
    with pytest.raises(StageError) as info:
        stage.raw_query("TMX? 1")
    assert info.value.source == "TMX?: not connected"
