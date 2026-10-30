"""The 19 Z handlers + Exit with RecordingReplies / RecordingEvents on SimStage (design §3.5.5, §4.2 WP5; 03 §4.3, §7;
08 §5–§10; 10 §6 ``restart_dh_macro`` / ``implicit_z_reinit``)."""

from __future__ import annotations

import logging
import math
from collections.abc import Callable
from typing import Any

import pytest

from aloxcam.dispatch import SERVER_Z, STOP, Z_WORKER, FinishOrder, ReplyKind, Request, WorkerContext
from aloxcam.errors import NO_ERROR, Code, ErrorCluster, StageError
from aloxcam.settings import StageOptions
from aloxcam.testing import LoggedEvent, RecordingEvents, RecordingReplies, SentReply, make_request, make_worker_context
from aloxcam.wire.messages import (
    COMMANDS,
    ZGetSerialData,
    ZJoystickData,
    ZLimitData,
    ZMoveAbsData,
    ZMoveRelData,
    ZReferenceData,
    ZSetSpeedData,
    ZWaitData,
)
from aloxcam.zstage.controller import ZController, ZSession
from aloxcam.zstage.handlers import Z_HANDLERS, ZContext
from aloxcam.zstage.sim import SimStage

ZEnv = Any                                                   # tests/zstage/conftest.py
MakeEnv = Callable[..., ZEnv]
IDENTITY = "(c) aloxcam SimStage, C-663-SIM, 0000000001, 0.0.0"
OUT_OF_RANGE = ErrorCluster(True, 9405, "Z move out of range")

LABELS = {
    "Z Setup": "Setup", "Z Get Serial": "Get Serial", "Z Get Status": "Get Status", "Z Get Error": "Get Error",
    "Z Set Speed": "Set Speed", "Z Get Speed": "Get Speed", "Z Get Reference": "Get Z Referenced",
    "Z Do Reference": "Get Z Referenced", "Z Get Position": "Get Position", "Z Move Absolute": "Move Absolute",
    "Z Move Relative": "Move Relative", "Z Wait End Move": "Wait End Move", "Z Stop": "Stop",
    "Z Enable Joystick": "Enable Joystick", "Z Disable Joystick": "Disable Joystick",
    "Z Joystick Enabled ?": "Is Joystick Enabled ?", "Z Set Min Limit": "Z Set Min Limit",
    "Z Set Max Limit": "Z Set Max Limit", "Z Cleanup": "Cleanup",
}


def run(env: ZEnv, command: str, data: Any = None) -> SentReply | None:
    """Run one handler on a fresh client request; returns the recorded reply (None when nothing was recorded)."""
    env.replies.clear()
    req = make_request(command, data)
    outcome = Z_HANDLERS[command](env.ctx, req)
    assert outcome is None
    assert req.replied
    return env.replies.last


def setup_ok(env: ZEnv) -> None:
    reply = run(env, "Z Setup")
    assert reply is not None and reply.err == NO_ERROR
    env.sim.clear_frames()
    env.events.clear()


def z_events(env: ZEnv) -> list[LoggedEvent]:
    assert all(e.server_type == SERVER_Z and e.command is None for e in env.events.events)
    return env.events.events


def error_event(err: ErrorCluster, command: str) -> LoggedEvent:
    return LoggedEvent("ERROR", err, SERVER_Z, LABELS[command], None, "High Mag.")


# ---------------------------------------------------------------------------------------------- table
def test_table_has_the_19_z_commands_and_exit() -> None:
    z_commands = {name for name, spec in COMMANDS.items() if spec.worker == Z_WORKER}
    assert len(z_commands) == 19
    assert set(Z_HANDLERS) == z_commands | {"Exit"}
    assert all(COMMANDS[name].order is FinishOrder.Z and COMMANDS[name].extra == LABELS[name] for name in z_commands)


def test_exit_stops_without_reply_or_log(make_env: MakeEnv) -> None:
    env = make_env()
    req = Request.make_internal("Exit", COMMANDS["Exit"])
    assert Z_HANDLERS["Exit"](env.ctx, req) is STOP
    assert env.replies.replies == [] and env.events.events == [] and not req.replied
    assert env.sim.frames == []


def test_context_from_base(make_env: MakeEnv) -> None:
    env = make_env()
    base: WorkerContext = make_worker_context()
    ctx = ZContext.from_base(base, stage=env.stage, stage_options=StageOptions(wait_timeout_s=3.0))
    assert ctx.state is base.state and ctx.replies is base.replies and ctx.events is base.events
    assert ctx.compat is base.compat and ctx.tick is base.tick and ctx.stage is env.stage
    assert ctx.stage_options.wait_timeout_s == 3.0


# ---------------------------------------------------------------------------------------------- no link / no session
@pytest.mark.parametrize(("command", "data", "kind", "value", "code", "source"), [
    ("Z Get Status", None, ReplyKind.BOOL, False, -4, "ERR?: not connected"),
    ("Z Get Error", None, ReplyKind.STR, "Not connected (no port with given ID open)", -4,
     "ERR?: not connected\nGCS Error: Not connected (no port with given ID open)"),
    ("Z Set Speed", ZSetSpeedData(5.0), ReplyKind.ACK, None, 100, "VEL No axis or velocity specified"),
    ("Z Get Speed", None, ReplyKind.DBL, 0.0, 100, "VEL? No axis specified"),
    ("Z Get Reference", None, ReplyKind.BOOL, False, 100, "FRF? No axis specified"),
    ("Z Do Reference", ZReferenceData("FNL"), ReplyKind.ACK, None, 100,
     "FNL No axis specified\nGCS Error: PI LabVIEW driver reports error. See source control for details."),
    ("Z Get Position", None, ReplyKind.DBL, 0.0, 100, "POS? No axis specified"),
    ("Z Wait End Move", ZWaitData(), ReplyKind.ACK, None, -4, "#5: not connected"),
    ("Z Stop", None, ReplyKind.ACK, None, 100, "STP no axis specified"),
    ("Z Enable Joystick", ZJoystickData(), ReplyKind.ACK, None, 100, "JAX No axis specified"),
    ("Z Disable Joystick", ZJoystickData(), ReplyKind.ACK, None, 100, "JAX No axis specified"),
    ("Z Joystick Enabled ?", ZJoystickData(), ReplyKind.BOOL, False, -4, "JON?: not connected"),
    ("Z Cleanup", None, ReplyKind.ACK, None, -4, "RMC?: not connected"),
])
def test_commands_before_setup_reply_and_log_the_error(make_env: MakeEnv, command: str, data: Any,
                                                       kind: ReplyKind, value: Any, code: int, source: str) -> None:
    env = make_env()
    reply = run(env, command, data)
    err = ErrorCluster(True, code, source)
    assert reply is not None
    assert (reply.kind, reply.value, reply.err) == (kind, value, err)
    assert reply.sent is (kind is not ReplyKind.ACK)        # error ACKs suppressed by default (08 Z1)
    assert z_events(env) == [error_event(err, command)]
    assert env.ctx.state.error_out == err


@pytest.mark.parametrize("command", ["Z Set Speed", "Z Stop", "Z Cleanup", "Z Wait End Move"])
def test_error_acks_are_sent_with_send_error_replies(make_env: MakeEnv, command: str) -> None:
    env = make_env(send_error_replies=True)
    reply = run(env, command)
    assert reply is not None and reply.kind is ReplyKind.ACK and reply.err.status and reply.sent


# ---------------------------------------------------------------------------------------------- Z Setup / Get Serial
def test_z_setup(make_env: MakeEnv) -> None:
    env = make_env(sim={"stage_reference_s": 0.1})
    env.ctx.state.replace_z_settings(speed=7.5, com=3)
    reply = run(env, "Z Setup")
    assert reply is not None and (reply.kind, reply.err, reply.sent) == (ReplyKind.ACK, NO_ERROR, True)
    assert env.ctx.state.z_inited is True
    assert env.stage.session == ZSession(1, ("1",), IDENTITY, "0000000001")
    assert env.sim.port == ("COM3", 115200) and env.sim.referenced and env.sim.velocity == 7.5
    assert env.events.events == []


def test_z_setup_failure(make_env: MakeEnv) -> None:
    env = make_env()
    env.ctx.state.update(z_inited=True)
    env.sim.inject_failure("connect_rs232", StageError(Code.PI_OPEN_PORT, "Open RS232 COM2"))
    reply = run(env, "Z Setup")
    err = ErrorCluster(True, -6, "Open RS232 COM2\nGCS Error: Error while opening port")
    assert reply is not None and (reply.err, reply.sent) == (err, False)
    assert env.ctx.state.z_inited is False
    assert z_events(env) == [error_event(err, "Z Setup")]
    assert env.stage.session == ZSession(system=1)


def test_z_get_serial(make_env: MakeEnv) -> None:
    env = make_env()
    reply = run(env, "Z Get Serial", ZGetSerialData(type=False))
    assert reply is not None and (reply.kind, reply.value, reply.err) == (ReplyKind.STR, "", NO_ERROR)
    setup_ok(env)
    assert run(env, "Z Get Serial", ZGetSerialData(type=False)).value == "0000000001"   # type: ignore[union-attr]
    assert run(env, "Z Get Serial", ZGetSerialData(type=True)).value == IDENTITY        # type: ignore[union-attr]
    assert env.sim.frames == [] and env.events.events == []                            # no I/O


# ---------------------------------------------------------------------------------------------- Get Status / Stop
def test_z_get_status_with_dh_running(make_env: MakeEnv) -> None:
    env = make_env()
    setup_ok(env)
    reply = run(env, "Z Get Status")
    assert reply is not None and (reply.kind, reply.value, reply.err) == (ReplyKind.BOOL, True, NO_ERROR)
    assert env.sim.frames == ["ERR?", "RMC?"]
    assert (env.ctx.state.running_macros, env.ctx.state.macro_running) == ("DH", True)


@pytest.mark.parametrize("restart", [True, False])
def test_z_get_status_restarts_dh(make_env: MakeEnv, restart: bool) -> None:
    env = make_env(compat={"restart_dh_macro": restart}, sim={"dh_running": False})
    setup_ok(env)
    reply = run(env, "Z Get Status")
    assert reply is not None and (reply.value, reply.err) == (True, NO_ERROR)
    assert env.sim.frames == (["ERR?", "RMC?", "MAC START DH"] if restart else ["ERR?", "RMC?"])
    assert (env.ctx.state.running_macros, env.ctx.state.macro_running) == ("", False)
    assert env.sim.running_macro == ("DH" if restart else "")


@pytest.mark.parametrize("restart", [True, False])
def test_z_stop_then_get_status(make_env: MakeEnv, restart: bool) -> None:
    env = make_env(compat={"restart_dh_macro": restart})
    setup_ok(env)
    env.ctx.state.update(running_macros="DH", macro_running=True)
    reply = run(env, "Z Stop")
    assert reply is not None and (reply.kind, reply.err, reply.sent) == (ReplyKind.ACK, NO_ERROR, True)
    assert env.sim.frames == (["STP 1", "RMC?", "MAC START DH"] if restart else ["STP 1"])
    assert env.sim.running_macro == ("DH" if restart else "")
    assert (env.ctx.state.running_macros, env.ctx.state.macro_running) == ("DH", True)   # not updated by Z Stop
    env.sim.clear_frames()
    reply = run(env, "Z Get Status")                         # 08 Z6: error 10 left pending by STP
    err = ErrorCluster(True, 10, "Controller error")
    assert reply is not None and (reply.kind, reply.value, reply.err, reply.sent) == (ReplyKind.BOOL, False, err, True)
    assert env.sim.frames == ["ERR?"]                         # macro part skipped after the ERR? error
    assert (env.ctx.state.running_macros, env.ctx.state.macro_running) == ("", False)
    assert z_events(env) == [error_event(err, "Z Get Status")]
    assert run(env, "Z Get Status").value is True             # type: ignore[union-attr]  # ERR? cleared it


def test_z_stop_restart_while_another_macro_runs(make_env: MakeEnv) -> None:
    env = make_env()
    setup_ok(env)
    env.sim.trigger_home()                                   # HOME runs; STP stops it, DH is restarted
    reply = run(env, "Z Stop")
    assert reply is not None and reply.err == NO_ERROR
    assert env.sim.running_macro == "DH"


# ---------------------------------------------------------------------------------------------- Get Error
def test_z_get_error(make_env: MakeEnv) -> None:
    env = make_env()
    setup_ok(env)
    reply = run(env, "Z Get Error")
    assert reply is not None and (reply.kind, reply.value, reply.err) == (ReplyKind.STR, "", NO_ERROR)
    run(env, "Z Move Absolute", ZMoveAbsData(15.0))          # inside the software range…
    env.sim.mov("1", 30.0)                                   # …but a refused controller move latches 7
    reply = run(env, "Z Get Error")
    err = ErrorCluster(True, 7, "Controller error\nGCS Error: Position out of limits")
    assert reply is not None and (reply.value, reply.err, reply.sent) == ("Position out of limits", err, True)
    assert z_events(env) == [error_event(err, "Z Get Error")]


# ------------------------------------------------------------------------------------- speed, reference, position
def test_z_set_and_get_speed(make_env: MakeEnv) -> None:
    env = make_env()
    setup_ok(env)
    reply = run(env, "Z Set Speed", ZSetSpeedData(50.0))
    assert reply is not None and (reply.kind, reply.err) == (ReplyKind.ACK, NO_ERROR)
    assert env.sim.frames == ["VEL 1 50.0000", "ERR?"]
    assert env.ctx.state.z.speed == 10.0                     # settings unchanged (03 C14)
    reply = run(env, "Z Get Speed")
    assert reply is not None and (reply.kind, reply.value, reply.err) == (ReplyKind.DBL, 50.0, NO_ERROR)
    reply = run(env, "Z Set Speed", ZSetSpeedData(0.0))      # refused by the controller, read by VEL's ERR?
    assert reply is not None and reply.err == ErrorCluster(True, 17, "Controller error")
    assert z_events(env)[-1].extra == "Set Speed"


def test_z_get_reference(make_env: MakeEnv) -> None:
    env = make_env()
    setup_ok(env)
    reply = run(env, "Z Get Reference")
    assert reply is not None and (reply.kind, reply.value, reply.err) == (ReplyKind.BOOL, True, NO_ERROR)
    env.sim.power_cycle()
    assert run(env, "Z Get Reference").value is False        # type: ignore[union-attr]
    assert env.sim.frames == ["FRF? 1", "FRF? 1"]


def test_z_get_position_writes_z_pos_even_on_error(make_env: MakeEnv, fake_time: Any) -> None:
    env = make_env()
    setup_ok(env)
    run(env, "Z Move Absolute", ZMoveAbsData(4.0))
    fake_time.advance(10)
    env.ctx.state.update(z_pos=-1.0)
    reply = run(env, "Z Get Position")
    assert reply is not None and (reply.kind, reply.value, reply.err) == (ReplyKind.DBL, 4.0, NO_ERROR)
    assert env.ctx.state.z_pos == 4.0
    env.sim.inject_failure("qpos", StageError(-7, "Data Receive Timeout (RS-232, COM2, baud 115200)"))
    reply = run(env, "Z Get Position")
    assert reply is not None and (reply.value, reply.err.code, reply.sent) == (0.0, -7, True)
    assert env.ctx.state.z_pos == 0.0


# ---------------------------------------------------------------------------------------------- Do Reference
@pytest.mark.parametrize(("method", "frame"), [("FNL", "FNL 1"), ("FRF", "FRF 1")])
def test_z_do_reference(make_env: MakeEnv, method: str, frame: str) -> None:
    env = make_env(sim={"stage_reference_s": 0.1})
    setup_ok(env)
    reply = run(env, "Z Do Reference", ZReferenceData(method))
    assert reply is not None and (reply.kind, reply.err, reply.sent) == (ReplyKind.ACK, NO_ERROR, True)
    frames = env.sim.frames
    assert frames[0] == frame and frames[-1] == "ERR?"
    assert "#7" in frames and frames.index("#5") > max(i for i, f in enumerate(frames) if f == "#7")   # Z13: #7 then #5
    assert env.sim.referenced and env.events.events == []


def test_z_do_reference_unknown_item_runs_fnl(make_env: MakeEnv, caplog: pytest.LogCaptureFixture) -> None:
    env = make_env()
    setup_ok(env)
    with caplog.at_level(logging.WARNING, logger="aloxcam.zstage.handlers"):
        run(env, "Z Do Reference", ZReferenceData("#7"))
    assert env.sim.frames[0] == "FNL 1"
    assert any("#7" in r.getMessage() for r in caplog.records)


def test_z_do_reference_without_ready_wait(make_env: MakeEnv) -> None:
    env = make_env(stage=StageOptions(do_reference_waits_ready=False), sim={"stage_reference_s": 0.1})
    setup_ok(env)
    run(env, "Z Do Reference", ZReferenceData("FNL"))
    assert "#7" not in env.sim.frames and "#5" in env.sim.frames   # LabVIEW: #5 only


def test_z_do_reference_error_is_translated(make_env: MakeEnv) -> None:
    env = make_env()
    setup_ok(env)
    run(env, "Z Enable Joystick", ZJoystickData())
    reply = run(env, "Z Do Reference", ZReferenceData("FNL"))   # FNL refused (51), read by the final ERR?
    err = ErrorCluster(True, 51, "Controller error\nGCS Error: Selected axis is controlled by joystick")
    assert reply is not None and (reply.err, reply.sent) == (err, False)
    assert z_events(env) == [error_event(err, "Z Do Reference")]


# ---------------------------------------------------------------------------------------------- Move Absolute
def test_z_move_absolute(make_env: MakeEnv, fake_time: Any) -> None:
    env = make_env()
    setup_ok(env)
    env.ctx.state.update(z_inited=False)
    reply = run(env, "Z Move Absolute", ZMoveAbsData(12.5))
    assert reply is not None and (reply.kind, reply.err, reply.sent) == (ReplyKind.ACK, NO_ERROR, True)
    assert env.sim.frames == ["FRF? 1", "MOV 1 12.5000"]     # no wait, no ERR? (03 C13)
    assert env.ctx.state.z_pos == 12.5
    assert env.ctx.state.z_inited is True                    # from the reference check
    fake_time.advance(10)
    assert env.sim.position == 12.5


@pytest.mark.parametrize(("lo", "hi", "target", "ok"), [
    (0.0, 20.0, 0.0, True), (0.0, 20.0, 20.0, True), (0.0, 20.0, 20.0001, False), (0.0, 20.0, -0.0001, False),
    (20.0, 0.0, 10.0, True), (20.0, 0.0, 20.0, True), (20.0, 0.0, 21.0, False),   # reversed limits tolerated
    (0.0, 20.0, math.nan, False), (0.0, 20.0, math.inf, False),
])
def test_z_move_absolute_range(make_env: MakeEnv, lo: float, hi: float, target: float, ok: bool) -> None:
    env = make_env()
    setup_ok(env)
    env.ctx.state.replace_z_settings(min_pos=lo, max_pos=hi)
    env.ctx.state.update(z_pos=-1.0)
    reply = run(env, "Z Move Absolute", ZMoveAbsData(target))
    assert reply is not None
    if ok:
        assert reply.err == NO_ERROR and env.sim.frames[-1].startswith("MOV")
        assert env.ctx.state.z_pos == target
    else:
        assert (reply.err, reply.sent) == (OUT_OF_RANGE, False)
        assert env.sim.frames == ["FRF? 1"] and env.ctx.state.z_pos == -1.0
        assert z_events(env) == [error_event(OUT_OF_RANGE, "Z Move Absolute")]


def test_z_move_absolute_out_of_range_sent_with_send_error_replies(make_env: MakeEnv) -> None:
    env = make_env(send_error_replies=True)
    setup_ok(env)
    reply = run(env, "Z Move Absolute", ZMoveAbsData(30.0))
    assert reply is not None and (reply.err, reply.sent) == (OUT_OF_RANGE, True)


def test_z_move_absolute_implicit_reinit(make_env: MakeEnv) -> None:
    env = make_env()                                         # no Z Setup: FRF? fails → whole set-up first (08 §6)
    reply = run(env, "Z Move Absolute", ZMoveAbsData(5.0))
    assert reply is not None and reply.err == NO_ERROR
    frames = env.sim.frames
    assert frames[:4] == ["XXX", "*idn?", "err?", "*IDN?"] and "FNL 1" in frames and frames[-1] == "MOV 1 5.0000"
    assert env.events.events == [LoggedEvent("INFO", NO_ERROR, SERVER_Z, "Move Absolute: Z Reference resetted",
                                             None, "High Mag.")]
    assert env.ctx.state.z_inited is True and env.ctx.state.z_pos == 5.0
    assert env.stage.session.axes == ("1",)


def test_z_move_absolute_reinit_failure(make_env: MakeEnv) -> None:
    env = make_env()
    env.ctx.state.update(z_inited=True)
    env.sim.inject_failure("connect_rs232", StageError(Code.PI_OPEN_PORT, "Open RS232 COM2"))
    reply = run(env, "Z Move Absolute", ZMoveAbsData(5.0))
    err = ErrorCluster(True, -6, "Open RS232 COM2\nGCS Error: Error while opening port")
    assert reply is not None and reply.err == err            # target seen as 0.0: in range, the re-init error wins
    assert env.ctx.state.z_inited is False
    assert z_events(env) == [LoggedEvent("INFO", err, SERVER_Z, "Move Absolute: Z Reference resetted", None,
                                         "High Mag."), error_event(err, "Z Move Absolute")]
    env.ctx.state.replace_z_settings(min_pos=1.0)            # 0.0 now out of range: 9405 replaces the error
    env.sim.inject_failure("connect_rs232", StageError(Code.PI_OPEN_PORT, "Open RS232 COM2"))
    assert run(env, "Z Move Absolute", ZMoveAbsData(5.0)).err == OUT_OF_RANGE   # type: ignore[union-attr]


def test_z_move_absolute_without_implicit_reinit(make_env: MakeEnv) -> None:
    env = make_env(compat={"implicit_z_reinit": False})
    setup_ok(env)
    env.ctx.state.update(z_inited=True)
    env.sim.power_cycle()                                    # reference lost
    reply = run(env, "Z Move Absolute", ZMoveAbsData(5.0))
    err = ErrorCluster(True, 5, "Z axis not referenced")
    assert reply is not None and reply.err == err
    assert env.sim.frames == ["FRF? 1"]                      # no re-initialisation, no move
    assert env.ctx.state.z_inited is True                    # left alone by this branch
    assert z_events(env) == [error_event(err, "Z Move Absolute")]
    env.sim.inject_failure("qfrf", StageError(-7, "Data Receive Timeout (RS-232, COM2, baud 115200)"))
    reply = run(env, "Z Move Absolute", ZMoveAbsData(5.0))
    assert reply is not None and reply.err == ErrorCluster(True, -7, "Data Receive Timeout (RS-232, COM2, baud 115200)")
    assert "XXX" not in env.sim.frames


def test_z_move_absolute_with_implicit_reinit_after_power_cycle(make_env: MakeEnv) -> None:
    env = make_env()
    setup_ok(env)
    env.sim.power_cycle()
    reply = run(env, "Z Move Absolute", ZMoveAbsData(5.0))
    assert reply is not None and reply.err == NO_ERROR
    assert "XXX" in env.sim.frames and env.sim.frames[-1] == "MOV 1 5.0000"
    assert [e.level for e in env.events.events] == ["INFO"]


def test_z_move_refused_by_the_controller_is_acknowledged(make_env: MakeEnv) -> None:
    env = make_env()
    setup_ok(env)
    run(env, "Z Enable Joystick", ZJoystickData())
    reply = run(env, "Z Move Absolute", ZMoveAbsData(5.0))
    assert reply is not None and reply.err == NO_ERROR       # 08 Z4: MOV does not read ERR?
    assert env.sim.error_register == 51
    assert env.ctx.state.z_pos == 5.0


# ---------------------------------------------------------------------------------------------- Move Relative
def test_z_move_relative(make_env: MakeEnv, fake_time: Any) -> None:
    env = make_env()
    setup_ok(env)
    run(env, "Z Move Absolute", ZMoveAbsData(12.5))
    fake_time.advance(10)
    env.sim.clear_frames()
    reply = run(env, "Z Move Relative", ZMoveRelData(-2.5))
    assert reply is not None and (reply.kind, reply.err) == (ReplyKind.ACK, NO_ERROR)
    assert env.sim.frames == ["FRF? 1", "POS? 1", "MVR 1 -2.5000"]
    assert env.ctx.state.z_pos == 10.0
    fake_time.advance(10)
    assert env.sim.position == 10.0
    reply = run(env, "Z Move Relative", ZMoveRelData(10.5))   # 20.5 > 20
    assert reply is not None and reply.err == OUT_OF_RANGE
    assert run(env, "Z Move Relative", ZMoveRelData(math.nan)).err == OUT_OF_RANGE   # type: ignore[union-attr]
    assert run(env, "Z Move Relative", ZMoveRelData(10.0)).err == NO_ERROR   # type: ignore[union-attr]  # bound


def test_z_move_relative_pos_failure(make_env: MakeEnv) -> None:
    env = make_env()
    setup_ok(env)
    env.sim.inject_failure("qpos", StageError(-7, "timeout"))
    reply = run(env, "Z Move Relative", ZMoveRelData(1.0))
    assert reply is not None and reply.err == ErrorCluster(True, -7, "timeout")
    env.sim.inject_failure("qpos", StageError(-7, "timeout"))
    reply = run(env, "Z Move Relative", ZMoveRelData(-1.0))   # 0.0 - 1.0 out of range: 9405 replaces the error
    assert reply is not None and reply.err == OUT_OF_RANGE
    assert not any(f.startswith("MVR") for f in env.sim.frames)


def test_z_move_relative_reinit_info_line(make_env: MakeEnv) -> None:
    env = make_env()
    reply = run(env, "Z Move Relative", ZMoveRelData(2.0))
    assert reply is not None and reply.err == NO_ERROR
    assert [(e.level, e.extra) for e in env.events.events] == [("INFO", "Move Relative: Z Reference resetted")]
    assert env.ctx.state.z_pos == 2.0


def test_z_move_relative_without_implicit_reinit(make_env: MakeEnv) -> None:
    env = make_env(compat={"implicit_z_reinit": False})
    reply = run(env, "Z Move Relative", ZMoveRelData(2.0))   # no link: FRF? fails → that error (100: no axis)
    assert reply is not None and reply.err == ErrorCluster(True, 100, "FRF? No axis specified")
    assert env.sim.frames == [] and env.events.events[0].level == "ERROR"


# ---------------------------------------------------------------------------------------------- Wait End Move
def test_z_wait_end_move(make_env: MakeEnv, fake_time: Any) -> None:
    env = make_env()
    setup_ok(env)
    run(env, "Z Move Absolute", ZMoveAbsData(5.0))           # 0.5 s at 10 mm/s
    fake_time.sleeps.clear()
    reply = run(env, "Z Wait End Move", ZWaitData(add_wait_ms=200, polling_cycle_ms=10, add_wait_only=False))
    assert reply is not None and (reply.kind, reply.err, reply.sent) == (ReplyKind.ACK, NO_ERROR, True)
    assert env.sim.position == 5.0 and not env.sim.frames[-1].startswith("MOV")
    assert fake_time.sleeps[-1] == 0.2 and set(fake_time.sleeps[:-1]) == {0.01}


def test_z_wait_end_move_negative_times_count_as_zero(make_env: MakeEnv, fake_time: Any) -> None:
    env = make_env()
    setup_ok(env)
    run(env, "Z Move Absolute", ZMoveAbsData(1.0))
    fake_time.sleeps.clear()
    fake_time.on_sleep = lambda _s: fake_time.advance(0.01)   # a 0 ms poll still lets (fake) time pass
    reply = run(env, "Z Wait End Move", ZWaitData(add_wait_ms=-50, polling_cycle_ms=-10))
    assert reply is not None and reply.err == NO_ERROR
    assert env.sim.position == 1.0
    assert set(fake_time.sleeps) == {0.0}                    # polls of 0 ms, no additional wait


def test_z_wait_end_move_timeout(make_env: MakeEnv, fake_time: Any) -> None:
    env = make_env(stage=StageOptions(wait_timeout_s=0.3))
    setup_ok(env)
    run(env, "Z Set Speed", ZSetSpeedData(1.0))
    run(env, "Z Move Absolute", ZMoveAbsData(20.0))
    reply = run(env, "Z Wait End Move", ZWaitData(add_wait_ms=100))
    err = ErrorCluster(True, -7, "Wait for axes to stop.vi: Timeout.")
    assert reply is not None and (reply.err, reply.sent) == (err, False)
    assert fake_time.sleeps[-1] == 0.1                       # additional wait after the timeout
    assert z_events(env) == [error_event(err, "Z Wait End Move")]


def test_z_wait_end_move_add_wait_only(make_env: MakeEnv, fake_time: Any) -> None:
    env = make_env()
    setup_ok(env)
    reply = run(env, "Z Wait End Move", ZWaitData(add_wait_ms=30, add_wait_only=True))
    assert reply is not None and reply.err == NO_ERROR
    assert env.sim.frames == [] and fake_time.sleeps[-1] == 0.03


# ---------------------------------------------------------------------------------------------- joystick
def test_z_joystick_commands(make_env: MakeEnv) -> None:
    env = make_env()
    setup_ok(env)
    reply = run(env, "Z Enable Joystick", ZJoystickData(joystick_id=1, joystick_axis=1))
    assert reply is not None and (reply.kind, reply.err) == (ReplyKind.ACK, NO_ERROR)
    assert env.sim.frames == ["JAX 1 1 1", "JON 1 1", "ERR?"]
    reply = run(env, "Z Joystick Enabled ?", ZJoystickData())
    assert reply is not None and (reply.kind, reply.value, reply.err) == (ReplyKind.BOOL, True, NO_ERROR)
    env.sim.clear_frames()
    reply = run(env, "Z Disable Joystick", ZJoystickData())
    assert reply is not None and reply.err == NO_ERROR
    assert env.sim.frames == ["JAX 1 1 1", "JON 1 0", "ERR?"]   # JAX is sent again
    assert run(env, "Z Joystick Enabled ?", ZJoystickData()).value is False   # type: ignore[union-attr]


def test_z_joystick_reports_a_pending_error(make_env: MakeEnv) -> None:
    env = make_env()
    setup_ok(env)
    run(env, "Z Stop")
    reply = run(env, "Z Enable Joystick", ZJoystickData())
    assert reply is not None and reply.err == ErrorCluster(True, 10, "Controller error")
    assert z_events(env)[-1].extra == "Enable Joystick"


# ---------------------------------------------------------------------------------------------- limits
@pytest.mark.parametrize(("command", "field"), [("Z Set Min Limit", "min_pos"), ("Z Set Max Limit", "max_pos")])
def test_z_set_limits(make_env: MakeEnv, command: str, field: str) -> None:
    env = make_env()
    reply = run(env, command, ZLimitData(0.5))
    assert reply is not None and (reply.kind, reply.err, reply.sent) == (ReplyKind.ACK, NO_ERROR, True)
    assert getattr(env.ctx.state.z, field) == 0.5
    assert env.sim.frames == []                              # memory only
    for bad in (math.nan, math.inf, -math.inf):
        reply = run(env, command, ZLimitData(bad))
        err = ErrorCluster(True, 1, f"{command}: value must be finite")
        assert reply is not None and (reply.err, reply.sent) == (err, False)
        assert getattr(env.ctx.state.z, field) == 0.5        # unchanged
    assert z_events(env)[-1] == error_event(ErrorCluster(True, 1, f"{command}: value must be finite"), command)


def test_z_limits_apply_to_the_next_move(make_env: MakeEnv) -> None:
    env = make_env()
    setup_ok(env)
    assert run(env, "Z Move Absolute", ZMoveAbsData(22.0)).err == OUT_OF_RANGE   # type: ignore[union-attr]
    run(env, "Z Set Max Limit", ZLimitData(24.0))
    assert run(env, "Z Move Absolute", ZMoveAbsData(22.0)).err == NO_ERROR      # type: ignore[union-attr]


# ---------------------------------------------------------------------------------------------- cleanup
def test_z_cleanup_default_error_ack(make_env: MakeEnv) -> None:
    env = make_env()
    setup_ok(env)
    reply = run(env, "Z Cleanup")
    err = ErrorCluster(True, -4, "RMC?: not connected")
    assert reply is not None and (reply.kind, reply.err, reply.sent) == (ReplyKind.ACK, err, False)
    assert z_events(env) == [error_event(err, "Z Cleanup")]
    assert not env.sim.is_connected and env.stage.session == ZSession()
    assert env.ctx.state.z_inited is False                   # 08 Z9 fix
    assert run(env, "Z Get Serial", ZGetSerialData(type=True)).value == ""   # type: ignore[union-attr]


def test_z_cleanup_without_dh_restart(make_env: MakeEnv) -> None:
    env = make_env(compat={"restart_dh_macro": False})
    setup_ok(env)
    reply = run(env, "Z Cleanup")
    assert reply is not None and (reply.err, reply.sent) == (NO_ERROR, True)
    assert env.events.events == [] and env.sim.frames == []
    assert env.ctx.state.z_inited is False and env.stage.session == ZSession()


def test_z_setup_after_cleanup(make_env: MakeEnv) -> None:
    env = make_env()
    setup_ok(env)
    run(env, "Z Cleanup")
    setup_ok(env)
    assert env.ctx.state.z_inited is True and env.stage.session.axes == ("1",)


# ---------------------------------------------------------------------------------------------- shared controller
def test_handlers_use_the_live_session_of_the_shared_controller(make_env: MakeEnv) -> None:
    env = make_env()
    other = ZController(env.sim, StageOptions())             # an independent controller has no session
    assert other.session == ZSession()
    setup_ok(env)
    assert isinstance(env.ctx.stage, ZController) and env.ctx.stage.session.identity == IDENTITY
    assert isinstance(env.ctx.replies, RecordingReplies) and isinstance(env.ctx.events, RecordingEvents)
    assert isinstance(env.sim, SimStage)
