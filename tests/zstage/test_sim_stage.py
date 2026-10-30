"""SimStage: the simulated PI C-663 at GCS level (design §3.5.3, §4.2 WP5; 08 §3.4–§3.5, §4.4, §9, §10;
owner-answers §5)."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import pytest

from aloxcam.errors import Code, StageError
from aloxcam.zstage.backend import StageBackend, format_gcs_float
from aloxcam.zstage.gcs_errors import GCS_ERROR_TEXT, gcs_error_text
from aloxcam.zstage.sim import DH_MACRO_TEXT, HOME_MACRO_TEXT, INJECTABLE_METHODS, SIM_STAGE_NAME, SimStage

MakeSim = Callable[..., SimStage]
FakeTime = Any                                               # tests/zstage/conftest.py fixture


@pytest.fixture
def stage_kind() -> str:
    """These tests are about SimStage itself: ``make_sim`` builds a SimStage only (tests/zstage/conftest.py)."""
    return "sim"


def ready(make_sim: MakeSim, **overrides: object) -> SimStage:
    """A connected, referenced stage with servo on (the state after ``Z Setup``)."""
    sim = make_sim(stage_initially_referenced=True, **overrides)
    sim.connect_rs232("COM2", 115200, 5000)
    sim.clear_frames()
    return sim


# ---------------------------------------------------------------------------------------------- link
@pytest.mark.parametrize(("call", "command"), [
    (lambda s: s.qerr(), "ERR?"),
    (lambda s: s.qidn(), "*IDN?"),
    (lambda s: s.qpos("1"), "POS?"),
    (lambda s: s.mov("1", 1.0), "MOV"),
    (lambda s: s.vel("1", 1.0), "VEL"),
    (lambda s: s.stp("1"), "STP"),
    (lambda s: s.qrmc(), "RMC?"),
    (lambda s: s.mac_start("DH"), "MAC START"),
    (lambda s: s.qmac("DH"), "MAC?"),
    (lambda s: s.is_moving(), "#5"),
    (lambda s: s.is_controller_ready(), "#7"),
    (lambda s: s.stop_all(), "#24"),
    (lambda s: s.jon(1, True), "JON"),
])
def test_not_connected_raises_minus_4(make_sim: MakeSim, call: Callable[[SimStage], object], command: str) -> None:
    sim = make_sim()
    with pytest.raises(StageError) as info:
        call(sim)
    assert info.value.code == Code.PI_NOT_CONNECTED == -4
    assert info.value.source == f"{command}: not connected"
    assert sim.frames == []


def test_close_and_translate_need_no_link(make_sim: MakeSim) -> None:
    sim = make_sim()
    assert sim.close() is False
    assert sim.translate_error(10) == "Controller was stopped by command"
    assert sim.translate_error(9405) == ""
    assert isinstance(sim, StageBackend)


def test_connect_purge_returns_and_clears_the_latched_error(make_sim: MakeSim) -> None:
    sim = make_sim()
    assert sim.connect_rs232("COM4", 115200, 5000) == 0
    assert sim.is_connected and sim.port == ("COM4", 115200)
    assert sim.frames == ["XXX", "*idn?", "err?"]
    sim.stp("1")                                             # latches 10
    assert sim.close() is True and not sim.is_connected
    assert sim.close() is False
    assert sim.error_register == 10                          # controller state survives the closed link
    assert sim.connect_rs232("COM4", 115200, 5000) == 10      # discarded by the purge (08 Z22)
    assert sim.error_register == 0
    assert sim.connect_rs232("COM5", 9600, 5000) == 0         # re-open closes the old link first
    assert sim.port == ("COM5", 9600)


def test_identity_and_static_queries(make_sim: MakeSim) -> None:
    sim = make_sim(stage_identity="PI, C-663, 123, 1.0", stage_min_mm=1.0, stage_max_mm=24.0)
    sim.connect_rs232("COM2", 115200, 5000)
    sim.clear_frames()
    assert sim.qidn() == "PI, C-663, 123, 1.0"
    assert sim.qsai() == ["1"]
    assert sim.qcst("1") == SIM_STAGE_NAME == "SIM-STAGE"
    assert sim.qron("1") is True and sim.qlim("1") is True and sim.qtrs("1") is False
    assert (sim.qtmn("1"), sim.qtmx("1")) == (1.0, 24.0)
    assert sim.frames == ["*IDN?", "SAI?", "CST? 1", "RON? 1", "LIM? 1", "TRS? 1", "TMN? 1", "TMX? 1"]


# ---------------------------------------------------------------------------------------------- motion
def test_linear_motion_timing_and_is_moving(make_sim: MakeSim, fake_time: FakeTime) -> None:
    sim = ready(make_sim)
    sim.vel("1", 10.0)
    sim.mov("1", 10.0)
    assert sim.frames == ["VEL 1 10.0000", "MOV 1 10.0000"]
    assert sim.is_moving() is True
    fake_time.advance(0.5)
    assert sim.qpos("1") == pytest.approx(5.0)
    assert sim.is_moving() is True
    fake_time.advance(0.5)
    assert sim.is_moving() is False
    assert sim.qpos("1") == 10.0
    assert sim.error_register == 0


def test_numbers_pass_through_the_4_decimal_frame_format(make_sim: MakeSim, fake_time: FakeTime) -> None:
    sim = ready(make_sim)
    sim.mov("1", 12.345678)
    sim.mvr("1", -0.25)
    assert sim.frames == ["MOV 1 12.3457", "MVR 1 -0.2500"]
    fake_time.advance(10)
    assert sim.position == pytest.approx(12.0957)
    assert format_gcs_float(10) == "10.0000"


def test_mvr_adds_to_the_commanded_target(make_sim: MakeSim, fake_time: FakeTime) -> None:
    sim = ready(make_sim)
    sim.vel("1", 1.0)
    sim.mov("1", 10.0)
    fake_time.advance(2.0)                                   # at 2.0, still moving towards 10
    sim.mvr("1", 1.0)
    fake_time.advance(100)
    assert sim.qpos("1") == pytest.approx(11.0)


def test_vel_changes_the_speed_of_the_running_move(make_sim: MakeSim, fake_time: FakeTime) -> None:
    sim = ready(make_sim)
    sim.vel("1", 1.0)
    sim.mov("1", 10.0)
    fake_time.advance(2.0)
    sim.vel("1", 8.0)                                        # 8 mm left at 8 mm/s
    fake_time.advance(0.5)
    assert sim.qpos("1") == pytest.approx(6.0)
    fake_time.advance(0.5)
    assert sim.is_moving() is False and sim.qpos("1") == pytest.approx(10.0)
    assert sim.qvel("1") == 8.0 and sim.velocity == 8.0


# ---------------------------------------------------------------------------------------------- latched errors
def joystick_on(sim: SimStage) -> None:
    sim.jax(1, 1, "1")
    sim.jon(1, True)


def joystick_then(call: Callable[[SimStage], object]) -> Callable[[SimStage], None]:
    def act(sim: SimStage) -> None:
        joystick_on(sim)
        call(sim)
    return act


@pytest.mark.parametrize(("act", "code"), [
    (lambda s: s.mov("1", 30.0), 7),                         # outside [stage_min_mm, stage_max_mm]
    (lambda s: s.mvr("1", -1.0), 7),
    (lambda s: s.stp("1"), 10),
    (lambda s: s.stp(None), 10),
    (lambda s: s.stop_all(), 10),
    (lambda s: s.vel("1", 0.0), 17),
    (lambda s: s.vel("1", -2.0), 17),
    (lambda s: s.vel("1", float("nan")), 1),
    (lambda s: s.mov("1", float("nan")), 1),
    (joystick_then(lambda s: s.mov("1", 1.0)), 51),
    (joystick_then(lambda s: s.fnl("1")), 51),
    (lambda s: s.mac_start("DH"), 1008),                     # DH runs from power-up
    (lambda s: (s.stp("1"), s.qerr(), s.mac_start("XY")), 20),
    (lambda s: s.mov("2", 1.0), 15),
    (lambda s: s.svo("A", True), 15),
])
def test_controller_errors_are_latched_and_read_once(make_sim: MakeSim, act: Callable[[SimStage], object],
                                                     code: int) -> None:
    sim = ready(make_sim)
    act(sim)                                                 # nothing raised: the error is latched (08 §3.5)
    assert sim.error_register == code
    assert sim.qerr() == code
    assert sim.qerr() == 0
    assert gcs_error_text(code) != ""


def test_move_on_unreferenced_axis_latches_5_and_does_not_move(make_sim: MakeSim, fake_time: FakeTime) -> None:
    sim = make_sim()
    sim.connect_rs232("COM2", 115200, 5000)
    sim.svo("1", True)
    sim.mov("1", 5.0)
    assert sim.is_moving() is False
    assert sim.qerr() == 5
    sim.svo("1", False)                                      # servo off: same code, even when referenced
    sim.fnl("1")
    assert sim.qerr() == 5


def test_only_the_last_error_is_kept(make_sim: MakeSim) -> None:
    sim = ready(make_sim)
    sim.mov("1", 99.0)
    sim.vel("1", 0.0)
    assert sim.qerr() == 17


def test_query_for_unknown_axis_fails_like_the_driver(make_sim: MakeSim) -> None:
    sim = ready(make_sim)
    with pytest.raises(StageError) as info:
        sim.qpos("2")
    assert info.value.code == Code.PI_DRIVER
    assert "Assign values" in info.value.source
    assert sim.qerr() == 15


# ---------------------------------------------------------------------------------------------- referencing
@pytest.mark.parametrize(("command", "destination"), [("fnl", 2.0), ("fpl", 24.0), ("frf", 13.0)])
def test_reference_moves(make_sim: MakeSim, fake_time: FakeTime, command: str, destination: float) -> None:
    sim = make_sim(stage_min_mm=2.0, stage_max_mm=24.0, stage_reference_s=0.4)
    sim.connect_rs232("COM2", 115200, 5000)
    sim.svo("1", True)
    sim.clear_frames()
    getattr(sim, command)("1")
    assert sim.frames == [f"{command.upper()} 1"]
    assert sim.qfrf("1") is False
    assert sim.is_controller_ready() is False and sim.is_moving() is True
    sim.mov("1", 10.0)                                       # refused while referencing: unreferenced
    assert sim.qerr() == 5
    fake_time.advance(0.2)
    assert sim.is_controller_ready() is False
    fake_time.advance(0.2)
    assert sim.is_controller_ready() is True and sim.is_moving() is False
    assert sim.qfrf("1") is True
    assert sim.qpos("1") == destination


def test_stop_during_referencing_leaves_the_axis_unreferenced(make_sim: MakeSim, fake_time: FakeTime) -> None:
    sim = make_sim(stage_reference_s=1.0, stage_initially_referenced=True)
    sim.connect_rs232("COM2", 115200, 5000)
    sim.mov("1", 10.0)
    fake_time.advance(10)
    sim.fnl("1")
    fake_time.advance(0.5)
    sim.stop_all()
    assert sim.qpos("1") == pytest.approx(5.0)
    fake_time.advance(5)
    assert sim.qfrf("1") is False and sim.is_controller_ready() is True
    assert sim.qerr() == 10


def test_stp_stops_where_the_axis_is(make_sim: MakeSim, fake_time: FakeTime) -> None:
    sim = ready(make_sim)
    sim.vel("1", 10.0)
    sim.mov("1", 20.0)
    fake_time.advance(0.75)
    sim.stp("1")
    assert sim.frames[-1] == "STP 1"
    assert sim.is_moving() is False
    fake_time.advance(5)
    assert sim.qpos("1") == pytest.approx(7.5)


def test_initially_referenced_option(make_sim: MakeSim) -> None:
    assert make_sim().referenced is False
    assert make_sim(stage_initially_referenced=True).referenced is True


# ---------------------------------------------------------------------------------------------- macros
def test_dh_runs_as_a_daemon_until_stopped(make_sim: MakeSim, fake_time: FakeTime) -> None:
    sim = ready(make_sim)
    assert sim.qrmc() == "DH"
    fake_time.advance(3600)
    assert sim.qrmc() == "DH"                                # restarts itself for ever (owner-answers §5)
    sim.stp("1")                                             # STP stops running macros
    assert sim.qrmc() == ""
    sim.qerr()
    sim.mac_start("DH")
    assert sim.qrmc() == "DH" and sim.error_register == 0
    sim.stop_all()
    assert sim.qrmc() == ""


def test_dh_with_a_lifetime_ends(make_sim: MakeSim, fake_time: FakeTime) -> None:
    sim = make_sim(stage_dh_macro_s=2.0)
    sim.connect_rs232("COM2", 115200, 5000)
    assert sim.qrmc() == "DH"
    fake_time.advance(2.0)
    assert sim.qrmc() == ""
    sim.mac_start("DH")
    fake_time.advance(1.0)
    assert sim.qrmc() == "DH"
    fake_time.advance(1.0)
    assert sim.qrmc() == ""


def test_dh_not_running_at_power_up(make_sim: MakeSim) -> None:
    sim = make_sim(dh_running=False)
    sim.connect_rs232("COM2", 115200, 5000)
    assert sim.qrmc() == "" and sim.running_macro == ""


def test_macro_contents(make_sim: MakeSim) -> None:
    sim = ready(make_sim)
    assert sim.qmac("DH") == DH_MACRO_TEXT
    assert sim.qmac("HOME") == HOME_MACRO_TEXT
    assert DH_MACRO_TEXT.splitlines()[-2:] == ["MAC START HOME", "MAC START DH"]
    assert sim.qmac("NOPE") == "" and sim.qerr() == 20
    assert sim.frames == ["MAC? DH", "MAC? HOME", "MAC? NOPE", "ERR?"]


def test_home_trigger_moves_to_zero_then_restarts_dh(make_sim: MakeSim, fake_time: FakeTime) -> None:
    sim = ready(make_sim)
    sim.vel("1", 5.0)
    sim.mov("1", 10.0)
    fake_time.advance(5)
    joystick_on(sim)
    assert sim.trigger_home() is True
    assert sim.qrmc() == "HOME"
    assert sim.qjon(1) == 0                                  # HOME: JON 1 0
    fake_time.advance(1.0)
    assert sim.qpos("1") == pytest.approx(5.0) and sim.is_moving() is True
    fake_time.advance(1.0)
    assert sim.qpos("1") == 0.0 and sim.is_moving() is False
    assert sim.qrmc() == "DH"
    assert sim.error_register == 0


def test_home_needs_dh_and_a_referenced_axis(make_sim: MakeSim) -> None:
    sim = ready(make_sim, dh_running=False)
    assert sim.trigger_home() is False
    unreferenced = make_sim()
    unreferenced.connect_rs232("COM2", 115200, 5000)
    assert unreferenced.trigger_home() is True               # DH starts HOME, whose MOV fails
    assert unreferenced.running_macro == ""
    assert unreferenced.qerr() == 5


def test_power_cycle(make_sim: MakeSim, fake_time: FakeTime) -> None:
    sim = ready(make_sim)
    sim.vel("1", 50.0)
    sim.mov("1", 5.0)
    fake_time.advance(1)
    sim.stp("1")
    sim.power_cycle()
    assert sim.is_connected
    assert sim.qfrf("1") is False and sim.qsvo("1") is False
    assert sim.qerr() == 0 and sim.qvel("1") == 10.0
    assert sim.qrmc() == "DH"
    assert sim.qpos("1") == 5.0


# ---------------------------------------------------------------------------------------------- joystick
def test_joystick_on_off(make_sim: MakeSim) -> None:
    sim = ready(make_sim)
    assert sim.qjon(1) == 0
    joystick_on(sim)
    assert sim.qjon(1) == 1 and sim.joystick_enabled
    sim.jon(1, False)
    assert sim.qjon(1) == 0
    assert sim.frames == ["JON? 1", "JAX 1 1 1", "JON 1 1", "JON? 1", "JON 1 0", "JON? 1"]
    assert sim.error_register == 0


# ---------------------------------------------------------------------------------------------- test hooks
def test_inject_failure(make_sim: MakeSim) -> None:
    sim = make_sim()
    boom = StageError(-7, "Data Receive Timeout (RS-232, COM2, baud 115200)")
    sim.inject_failure("qpos", boom, times=2)
    sim.inject_failure("connect_rs232", StageError(-6, "cannot open COM9"))
    with pytest.raises(StageError) as info:
        sim.qpos("1")                                        # injection wins over the link check
    assert info.value is boom
    with pytest.raises(StageError) as info:
        sim.connect_rs232("COM9", 115200, 5000)
    assert info.value.code == -6 and not sim.is_connected
    sim.connect_rs232("COM2", 115200, 5000)
    with pytest.raises(StageError):
        sim.qpos("1")
    assert sim.qpos("1") == 0.0                              # used up


def test_inject_failure_validation(make_sim: MakeSim) -> None:
    sim = make_sim()
    with pytest.raises(ValueError):
        sim.inject_failure("is_connected", StageError(1, "x"))
    with pytest.raises(ValueError):
        sim.inject_failure("nope", StageError(1, "x"))
    with pytest.raises(ValueError):
        sim.inject_failure("qpos", StageError(1, "x"), times=0)
    assert "qerr" in INJECTABLE_METHODS and "translate_error" in INJECTABLE_METHODS


def test_gcs_error_table() -> None:
    for code in (0, 1, 2, 5, 7, 10, 15, 17, 51, 1008, -7, 100):
        assert GCS_ERROR_TEXT[code]
    assert gcs_error_text(-7) == "Timeout error"
    assert gcs_error_text(10) == "Controller was stopped by command"
    assert gcs_error_text(12345) == ""
