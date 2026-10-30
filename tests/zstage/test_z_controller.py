"""ZController on SimStage: set-up, pre-checks, ERR? pairs, waits, reference check, cancellation, locking
(design §3.5.4, §4.2 WP5; 08 §3.3, §4, §6, §8, §11, §13.3; 08 Z13, Z22)."""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from typing import Any

import pytest

from aloxcam.errors import Code, StageError
from aloxcam.settings import SimulationOptions, StageOptions, ZStageSettings
from aloxcam.zstage.controller import ZController, ZSession, serial_from_idn
from aloxcam.zstage.sim import SimStage

MakeSim = Callable[..., SimStage]
MakeController = Callable[..., ZController]
FakeTime = Any                                               # tests/zstage/conftest.py fixture

SETTINGS = ZStageSettings(com=2, baudrate=115200, axis=1, speed=12.5)
IDENTITY = "(c) aloxcam SimStage, C-663-SIM, 0000000001, 0.0.0"


def set_up(make_sim: MakeSim, make_controller: MakeController, **stage: Any) -> tuple[SimStage, ZController]:
    sim = make_sim(stage_initially_referenced=True)
    ctl = make_controller(sim, **stage)
    ctl.setup(SETTINGS)
    sim.clear_frames()
    return sim, ctl


# ---------------------------------------------------------------------------------------------- set-up
def test_setup_sequence_references_an_unreferenced_axis(make_sim: MakeSim, make_controller: MakeController,
                                                        fake_time: FakeTime) -> None:
    sim = make_sim(stage_reference_s=0.2)
    ctl = make_controller(sim)
    ctl.setup(SETTINGS)
    assert ctl.session == ZSession(system=1, axes=("1",), identity=IDENTITY, serial_number="0000000001")
    frames = sim.frames
    polls = frames.count("#7")
    assert polls >= 4                                        # 0.2 s of referencing polled every 50 ms
    assert frames == (["XXX", "*idn?", "err?", "*IDN?", "SAI?", "CST? 1", "SVO 1 1", "SVO? 1", "RON? 1", "FRF? 1",
                       "TRS? 1", "LIM? 1", "FNL 1"] + ["#7"] * polls
                      + ["FRF? 1", "TMN? 1", "TMX? 1", "POS? 1", "ERR?", "VEL 1 12.5000", "ERR?"])
    assert set(fake_time.sleeps) == {0.05}
    assert sim.port == ("COM2", 115200)
    assert sim.referenced and sim.velocity == 12.5 and sim.position == 0.0


def test_setup_skips_referencing_a_referenced_axis(make_sim: MakeSim, make_controller: MakeController) -> None:
    sim = make_sim(stage_initially_referenced=True)
    make_controller(sim).setup(SETTINGS)
    assert "FNL 1" not in sim.frames and "#7" not in sim.frames


@pytest.mark.parametrize(("move", "frame"), [("FNL", "FNL 1"), ("FPL", "FPL 1"), ("fpl", "FPL 1"), ("XYZ", "FNL 1")])
def test_always_reference_and_reference_move(make_sim: MakeSim, make_controller: MakeController, move: str,
                                             frame: str) -> None:
    sim = make_sim(stage_initially_referenced=True)
    make_controller(sim, always_reference=True, reference_move=move).setup(SETTINGS)   # owner Q11: every Z Setup
    assert frame in sim.frames and sim.referenced


def test_setup_port_map(make_sim: MakeSim, make_controller: MakeController) -> None:
    sim = make_sim()
    make_controller(sim, port_map={"COM4": "/dev/ttyUSB0"}).setup(ZStageSettings(com=4, baudrate=9600))
    assert sim.port == ("/dev/ttyUSB0", 9600)
    make_controller(sim).setup(ZStageSettings(com=0))
    assert sim.port == ("COM1", 115200)


def test_setup_failure_writes_the_session_and_translates(make_sim: MakeSim, make_controller: MakeController) -> None:
    sim = make_sim(stage_initially_referenced=True)
    sim.connect_rs232("COM2", 115200, 5000)
    sim.jax(1, 1, "1")
    sim.jon(1, True)                                         # joystick on: FNL refused with 51
    ctl = make_controller(sim, always_reference=True)
    with pytest.raises(StageError) as info:
        ctl.setup(SETTINGS)
    assert info.value.code == 51
    assert info.value.source == "Controller error\nGCS Error: Selected axis is controlled by joystick"
    assert ctl.session == ZSession(1, ("1",), IDENTITY, "0000000001")
    assert not any(f.startswith("VEL") for f in sim.frames)  # "Set Normal Speed" skipped


def test_setup_open_failure(make_sim: MakeSim, make_controller: MakeController) -> None:
    sim = make_sim()
    ctl = make_controller(sim)
    ctl.setup(SETTINGS)                                      # previous session...
    sim.inject_failure("connect_rs232", StageError(Code.PI_OPEN_PORT, "Open RS232 COM7"))
    with pytest.raises(StageError) as info:
        ctl.setup(ZStageSettings(axis=3))
    assert info.value.code == -6
    assert info.value.source == "Open RS232 COM7\nGCS Error: Error while opening port"
    assert ctl.session == ZSession(system=3)                 # ...replaced even on failure (08 §4.1)


def test_setup_partial_session_on_query_failure(make_sim: MakeSim, make_controller: MakeController) -> None:
    sim = make_sim()
    sim.inject_failure("qsai", StageError(Code.PI_DRIVER, "Assign values: Input string empty"))
    ctl = make_controller(sim)
    with pytest.raises(StageError) as info:
        ctl.setup(SETTINGS)
    assert info.value.source.endswith("\nGCS Error: PI LabVIEW driver reports error. See source control for details.")
    assert ctl.session == ZSession(1, (), IDENTITY, "0000000001")


def test_setup_vel_error_is_not_translated(make_sim: MakeSim, make_controller: MakeController) -> None:
    sim = make_sim(stage_initially_referenced=True)
    ctl = make_controller(sim)
    with pytest.raises(StageError) as info:
        ctl.setup(ZStageSettings(speed=0.0))
    assert (info.value.code, info.value.source) == (17, "Controller error")


def test_connect_purge_discarded_code_is_logged(make_sim: MakeSim, make_controller: MakeController,
                                                caplog: pytest.LogCaptureFixture) -> None:
    sim, ctl = set_up(make_sim, make_controller)
    ctl.stp()                                                # error 10 pending
    with caplog.at_level(logging.WARNING, logger="aloxcam.zstage.controller"):
        ctl.setup(SETTINGS)
    assert any("controller error 10" in r.getMessage() and "Z22" in r.getMessage() for r in caplog.records)
    assert sim.error_register == 0


def test_serial_from_idn() -> None:
    assert serial_from_idn(IDENTITY) == "0000000001"
    assert serial_from_idn("PI,C-663.12, 0123456789 ,02.01") == "0123456789"
    assert serial_from_idn("PI C-663") == ""
    assert serial_from_idn("") == ""


# ---------------------------------------------------------------------------------------------- pre-checks and pairs
@pytest.mark.parametrize(("call", "text"), [
    (lambda c: c.mov(1.0), "MOV No axis specified"),
    (lambda c: c.mvr(1.0), "MVR no axis specified"),
    (lambda c: c.vel(1.0), "VEL No axis or velocity specified"),
    (lambda c: c.stp(), "STP no axis specified"),
    (lambda c: c.qpos(), "POS? No axis specified"),
    (lambda c: c.qvel(), "VEL? No axis specified"),
    (lambda c: c.qfrf(), "FRF? No axis specified"),
    (lambda c: c.fnl(), "FNL No axis specified"),
    (lambda c: c.frf(), "FRF No axis specified"),
    (lambda c: c.jax(1, 1), "JAX No axis specified"),
    (lambda c: c.mac_start(""), "MAC START no macro name specified"),
])
def test_no_axis_pre_checks_send_nothing(make_sim: MakeSim, make_controller: MakeController,
                                         call: Callable[[ZController], object], text: str) -> None:
    sim = make_sim()
    sim.connect_rs232("COM2", 115200, 5000)                  # link open, but no session (no Z Setup)
    sim.clear_frames()
    with pytest.raises(StageError) as info:
        call(make_controller(sim))
    assert (info.value.code, info.value.source) == (100, text)
    assert sim.frames == []


def test_vel_and_jon_read_err(make_sim: MakeSim, make_controller: MakeController) -> None:
    sim, ctl = set_up(make_sim, make_controller)
    ctl.vel(5.0)
    ctl.jon(1, True)
    assert sim.frames == ["VEL 1 5.0000", "ERR?", "JON 1 1", "ERR?"]
    ctl.stp()                                                # error 10 pending: reported by the next ERR? reader
    with pytest.raises(StageError) as info:
        ctl.vel(6.0)
    assert (info.value.code, info.value.source) == (10, "Controller error")
    ctl.stp()
    with pytest.raises(StageError) as info:
        ctl.jon(1, False)
    assert info.value.code == 10
    assert ctl.qjon(1) is False
    assert sim.velocity == 6.0


def test_simple_commands(make_sim: MakeSim, make_controller: MakeController, fake_time: FakeTime) -> None:
    sim, ctl = set_up(make_sim, make_controller)
    ctl.mov(3.0)
    assert ctl.is_moving() is True
    fake_time.advance(10)
    assert ctl.is_moving() is False
    ctl.mvr(-1.0)
    fake_time.advance(10)
    assert ctl.qpos() == 2.0 and ctl.qvel() == 12.5 and ctl.qfrf() is True
    assert ctl.running_macros() == "DH"
    ctl.jax(1, 1)
    ctl.jon(1, True)
    assert ctl.qjon(1) is True
    assert sim.frames[:3] == ["MOV 1 3.0000", "#5", "#5"]
    assert "JAX 1 1 1" in sim.frames


def test_check_error_and_translate(make_sim: MakeSim, make_controller: MakeController) -> None:
    sim, ctl = set_up(make_sim, make_controller)
    ctl.check_error()
    ctl.mov(99.0)                                            # latched 7, no ERR? (03 C13)
    with pytest.raises(StageError) as info:
        ctl.check_error()
    err = info.value
    assert (err.code, err.source) == (7, "Controller error")
    translated = ctl.translate(err)
    assert translated.code == 7
    assert translated.source == "Controller error\nGCS Error: Position out of limits"
    unknown = StageError(9405, "Z move out of range")
    assert ctl.translate(unknown) is unknown
    assert ctl.gcs_error_text(-7) == "Timeout error"


# ---------------------------------------------------------------------------------------------- waits
def test_general_wait_until_stopped_then_additional_wait(make_sim: MakeSim, make_controller: MakeController,
                                                         fake_time: FakeTime) -> None:
    sim, ctl = set_up(make_sim, make_controller)
    ctl.vel(10.0)
    ctl.mov(1.0)                                             # 0.1 s
    sim.clear_frames()
    fake_time.sleeps.clear()
    ctl.general_wait(add_wait_ms=250, polling_ms=20)
    assert fake_time.sleeps[:-1] == [0.02] * len(fake_time.sleeps[:-1])
    assert fake_time.sleeps[-1] == 0.25
    assert set(sim.frames) == {"#5"} and len(sim.frames) == len(fake_time.sleeps)
    assert sim.position == 1.0


def test_general_wait_add_wait_only(make_sim: MakeSim, make_controller: MakeController, fake_time: FakeTime) -> None:
    sim, ctl = set_up(make_sim, make_controller)
    ctl.mov(10.0)
    fake_time.sleeps.clear()
    ctl.general_wait(add_wait_ms=100, add_wait_only=True)
    assert fake_time.sleeps == [0.1] and sim.frames == ["MOV 1 10.0000"]


def test_general_wait_timeout_then_additional_wait(make_sim: MakeSim, make_controller: MakeController,
                                                   fake_time: FakeTime) -> None:
    sim, ctl = set_up(make_sim, make_controller)
    ctl.vel(1.0)
    ctl.mov(20.0)                                            # 20 s
    fake_time.sleeps.clear()
    t0 = fake_time.now
    with pytest.raises(StageError) as info:
        ctl.general_wait(add_wait_ms=500, polling_ms=10, timeout_s=1.0)
    assert (info.value.code, info.value.source) == (Code.PI_TIMEOUT, "Wait for axes to stop.vi: Timeout.")
    assert fake_time.sleeps[-1] == 0.5                       # the additional wait also runs after a timeout (08 §8)
    assert fake_time.now - t0 == pytest.approx(1.5, abs=0.02)


def test_general_wait_default_timeout_is_the_option(make_sim: MakeSim, make_controller: MakeController,
                                                    fake_time: FakeTime) -> None:
    sim = make_sim(stage_initially_referenced=True)
    ctl = make_controller(sim, wait_timeout_s=0.45)
    ctl.setup(SETTINGS)
    ctl.vel(1.0)
    ctl.mov(20.0)
    t0 = fake_time.now
    with pytest.raises(StageError):
        ctl.general_wait(polling_ms=100)
    assert fake_time.now - t0 == pytest.approx(0.5)


def test_general_wait_poll_error_still_waits(make_sim: MakeSim, make_controller: MakeController,
                                             fake_time: FakeTime) -> None:
    sim, ctl = set_up(make_sim, make_controller)
    sim.inject_failure("is_moving", StageError(-7, "Data Receive Timeout (RS-232, COM2, baud 115200)"))
    with pytest.raises(StageError) as info:
        ctl.general_wait(add_wait_ms=40)
    assert info.value.code == -7
    assert fake_time.sleeps[-1] == 0.04


def test_wait_ready_timeout(make_sim: MakeSim, make_controller: MakeController, fake_time: FakeTime) -> None:
    sim, ctl = set_up(make_sim, make_controller)
    sim.svo("1", True)
    sim.fnl("1")
    sim.clear_frames()
    with pytest.raises(StageError) as info:
        ctl.wait_ready(timeout_s=0.1)
    assert info.value.code == -7 and info.value.source.endswith("Timeout.")
    assert set(sim.frames) == {"#7"}


def test_wait_ready_returns_once_referenced(make_sim: MakeSim, make_controller: MakeController,
                                            fake_time: FakeTime) -> None:
    sim, ctl = set_up(make_sim, make_controller)
    ctl.fnl()
    ctl.wait_ready()
    assert sim.referenced and fake_time.sleeps.count(0.05) >= 4


# ---------------------------------------------------------------------------------------------- reference check
def test_check_axis_reference_referenced(make_sim: MakeSim, make_controller: MakeController) -> None:
    sim, ctl = set_up(make_sim, make_controller)
    assert ctl.check_axis_reference(SETTINGS) == (True, False, None)
    assert sim.frames == ["FRF? 1"]


def test_check_axis_reference_reinitialises(make_sim: MakeSim, make_controller: MakeController) -> None:
    sim, ctl = set_up(make_sim, make_controller)
    sim.mov("1", 4.0)
    sim.stp("1")                                             # pending 10: discarded by the re-init purge (08 §6)
    sim.power_cycle()                                        # reference lost
    sim.stp("1")
    sim.clear_frames()
    assert ctl.check_axis_reference(SETTINGS) == (True, True, None)
    assert sim.frames[:4] == ["FRF? 1", "XXX", "*idn?", "err?"]
    assert "FNL 1" in sim.frames and sim.frames[-2:] == ["VEL 1 12.5000", "ERR?"]
    assert sim.referenced and sim.is_connected


def test_check_axis_reference_before_any_setup(make_sim: MakeSim, make_controller: MakeController) -> None:
    sim = make_sim()
    ctl = make_controller(sim)
    assert ctl.check_axis_reference(SETTINGS) == (True, True, None)   # failed FRF? → whole set-up
    assert ctl.session.axes == ("1",) and sim.referenced


def test_check_axis_reference_failed_reinit(make_sim: MakeSim, make_controller: MakeController) -> None:
    sim, ctl = set_up(make_sim, make_controller)
    sim.power_cycle()
    sim.inject_failure("connect_rs232", StageError(Code.PI_OPEN_PORT, "Open RS232 COM2"))
    inited, warning, err = ctl.check_axis_reference(SETTINGS)
    assert (inited, warning) == (False, True)
    assert err is not None and err.code == -6 and "GCS Error: Error while opening port" in err.source
    assert not sim.is_connected


# ---------------------------------------------------------------------------------------------- cancellation
def test_cancel_waits_ends_general_wait_within_one_poll(make_sim: MakeSim, make_controller: MakeController,
                                                       fake_time: FakeTime) -> None:
    sim, ctl = set_up(make_sim, make_controller)
    ctl.vel(1.0)
    ctl.mov(20.0)
    sim.clear_frames()
    fake_time.sleeps.clear()
    fake_time.on_sleep = lambda _s: ctl.cancel_waits() if len(fake_time.sleeps) == 3 else None
    with pytest.raises(StageError) as info:
        ctl.general_wait(add_wait_ms=1000, polling_ms=10)
    assert (info.value.code, info.value.source) == (Code.ABORTED_BY_SHUTDOWN,
                                                    "Wait for axes to stop: aborted by server shutdown")
    assert sim.frames == ["#5"] * 3                          # no poll after the cancellation
    assert fake_time.sleeps == [0.01] * 3                    # and no additional wait ("Stopped")


def test_cancel_waits_ends_wait_ready_and_blocks_later_waits(make_sim: MakeSim, make_controller: MakeController,
                                                            fake_time: FakeTime) -> None:
    sim, ctl = set_up(make_sim, make_controller)
    ctl.fnl()
    sim.clear_frames()
    fake_time.on_sleep = lambda _s: ctl.cancel_waits()
    with pytest.raises(StageError) as info:
        ctl.wait_ready()
    assert info.value.code == 9510 and info.value.source == "Wait for controller ready: aborted by server shutdown"
    assert sim.frames == ["#7"]
    for call in (lambda: ctl.general_wait(), lambda: ctl.general_wait(add_wait_ms=5, add_wait_only=True),
                 lambda: ctl.wait_ready(), lambda: ctl.setup(SETTINGS)):
        with pytest.raises(StageError) as info:
            call()
        assert info.value.code == Code.ABORTED_BY_SHUTDOWN


def test_cancel_waits_aborts_a_setup_in_its_reference_wait(make_sim: MakeSim, make_controller: MakeController,
                                                          fake_time: FakeTime) -> None:
    sim = make_sim(stage_reference_s=60.0)
    ctl = make_controller(sim)
    fake_time.on_sleep = lambda _s: ctl.cancel_waits()
    with pytest.raises(StageError) as info:
        ctl.setup(SETTINGS)
    assert info.value.code == 9510 and "aborted by server shutdown" in info.value.source


def test_cancel_waits_with_real_time() -> None:
    sim = SimStage(SimulationOptions(stage_initially_referenced=True))
    ctl = ZController(sim, StageOptions())                   # default sleep: an interruptible Event wait
    ctl.setup(SETTINGS)
    ctl.vel(0.001)
    ctl.mov(20.0)
    errors: list[StageError] = []

    def run() -> None:
        try:
            ctl.general_wait(add_wait_ms=60_000, polling_ms=10)
        except StageError as exc:
            errors.append(exc)

    worker = threading.Thread(target=run)
    worker.start()
    time.sleep(0.05)
    t0 = time.monotonic()
    ctl.cancel_waits()
    worker.join(2.0)
    assert not worker.is_alive()
    assert time.monotonic() - t0 < 0.5
    assert errors and errors[0].code == Code.ABORTED_BY_SHUTDOWN


# ---------------------------------------------------------------------------------------------- locking, close
class _BlockingSim(SimStage):
    """``POS?`` blocks until released: a long exchange in progress."""

    def __init__(self) -> None:
        super().__init__(SimulationOptions(stage_initially_referenced=True))
        self.block = False
        self.entered = threading.Event()
        self.release = threading.Event()

    def qpos(self, axis: str) -> float:
        if self.block:
            self.entered.set()
            assert self.release.wait(5.0)
        return super().qpos(axis)


def test_close_waits_for_a_running_exchange() -> None:
    sim = _BlockingSim()
    ctl = ZController(sim, StageOptions())
    ctl.setup(SETTINGS)
    sim.block = True
    reader = threading.Thread(target=ctl.qpos)
    reader.start()
    assert sim.entered.wait(2.0)
    closed: list[bool] = []
    closer = threading.Thread(target=lambda: closed.append(ctl.close()))
    closer.start()
    closer.join(0.2)
    assert closer.is_alive() and sim.is_connected            # close() waits for the POS? exchange
    sim.release.set()
    reader.join(2.0)
    closer.join(2.0)
    assert closed == [True] and not sim.is_connected
    assert ctl.close() is False                              # idempotent


def test_reset_session(make_sim: MakeSim, make_controller: MakeController) -> None:
    sim, ctl = set_up(make_sim, make_controller)
    assert ctl.session.axes == ("1",)
    ctl.reset_session()
    assert ctl.session == ZSession()
    assert ctl.options == StageOptions()


# ---------------------------------------------------------------------------------------------- start-up check
def test_startup_macro_check(make_sim: MakeSim, make_controller: MakeController) -> None:
    sim = make_sim()
    make_controller(sim, startup_macro_check=True).startup_macro_check()      # not connected: nothing
    sim.connect_rs232("COM2", 115200, 5000)
    sim.clear_frames()
    make_controller(sim).startup_macro_check()                                # switch off: nothing
    assert sim.frames == []
    make_controller(sim, startup_macro_check=True).startup_macro_check()      # DH runs → MAC START DH (reverse logic)
    assert sim.frames == ["RMC?", "MAC START DH"]
    assert sim.error_register == 1008                                         # refused, left pending as LabVIEW
    sim.stop_all()
    sim.qerr()
    sim.clear_frames()
    make_controller(sim, startup_macro_check=True).startup_macro_check()
    assert sim.frames == ["RMC?"]
