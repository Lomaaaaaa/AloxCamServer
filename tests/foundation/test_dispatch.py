"""aloxcam.dispatch finish helpers with the recording doubles (03 §1.1, §4.2–§4.5, design §3.1.5, §3.1.7).

Requests are built with explicit ``spec`` / ``code``: these tests never import WP2.
"""

from __future__ import annotations

import numpy as np
import pytest

from aloxcam.dispatch import (
    CAMERA_WORKER,
    CONTINUE,
    REQUEUE,
    SERVER_CAMERA,
    SERVER_Z,
    STOP,
    Z_WORKER,
    CommandSpec,
    DataMode,
    FinishOrder,
    Outcome,
    ReplyKind,
    Request,
    finish_camera,
    finish_z,
    reject_undecodable,
    reply_default_value,
    send_reply,
)
from aloxcam.errors import NO_ERROR, ErrorCluster
from aloxcam.imagedata import ImageData
from aloxcam.settings import HIGH_MAG, LOW_MAG, CompatSwitches
from aloxcam.testing import (
    FixedClock,
    LoggedEvent,
    RecordingEvents,
    RecordingReplies,
    make_request,
    make_worker_context,
)


def spec(name: str, reply: ReplyKind, order: FinishOrder, *, worker: str = CAMERA_WORKER, extra: str = "Extra",
         suffix: str | None = "Suffix") -> CommandSpec:
    return CommandSpec(name=name, worker=worker, payload=None, data_mode=DataMode.STRICT, reply=reply, order=order,
                       extra=extra, suffix=suffix)


SET_EXPOSURE = spec("Cam Set Exposure", ReplyKind.ACK, FinishOrder.A, extra="Set Exposure", suffix="")
CAM_SETUP = spec("Cam Setup", ReplyKind.ACK, FinishOrder.A, extra="Setup", suffix=None)
CAM_STOP = spec("Cam Stop", ReplyKind.ACK, FinishOrder.A, extra="Stop", suffix="Cam stop")
GET_STATUS = spec("Cam Get Acquisition Status", ReplyKind.BOOL, FinishOrder.A, extra="Get Status",
                  suffix="Cam Acquisition status")
GET_EXPOSURE = spec("Cam Get Exposure", ReplyKind.DBL, FinishOrder.A, extra="Get Exposure", suffix=None)
SET_LED = spec("Cam Set Led", ReplyKind.ACK, FinishOrder.RAW_FIRST, extra="Set LED", suffix="Cam Set Led")
GET_LED = spec("Cam Get Led", ReplyKind.BOOL, FinishOrder.RAW_FIRST, extra="Get LED", suffix="Cam Get Led")
HEARTBEAT = spec("Get Heartbeat", ReplyKind.U32, FinishOrder.RAW_FIRST, extra="Heartbeat", suffix="Get Heartbeat")
AF_DATA = spec("Cam Get Autofocus Data", ReplyKind.DBL2D, FinishOrder.RAW_FIRST, extra="Autofocus Data",
               suffix="Cam Get Autofocus Data")
STREAM = spec("Cam Stream Next Image", ReplyKind.IMAGE, FinishOrder.CUSTOM, extra="Get Image Start", suffix=None)
GET_IMAGE = spec("Cam Get Image", ReplyKind.NONE, FinishOrder.CUSTOM, extra="", suffix="Cam Get Image")
Z_MOVE = spec("Z Move Absolute", ReplyKind.ACK, FinishOrder.Z, worker=Z_WORKER, extra="Move Absolute", suffix=None)
Z_SERIAL = spec("Z Get Serial", ReplyKind.STR, FinishOrder.Z, worker=Z_WORKER, extra="Get Serial", suffix=None)
Z_STATUS = spec("Z Get Status", ReplyKind.BOOL, FinishOrder.Z, worker=Z_WORKER, extra="Get Status", suffix=None)

ERR = ErrorCluster(True, -1074360316, "IMAQdx error ")
WARN = ErrorCluster(False, 3, "warning")
UNDECODABLE = ErrorCluster(True, 91, "Variant To Data in alox_camera_server.vi")


def req(s: CommandSpec, *, code: int = 7, conn_id: int = 1, decode_error: ErrorCluster | None = None) -> Request:
    return make_request(s.name, spec=s, code=code, conn_id=conn_id, decode_error=decode_error)


def replies(ctx) -> RecordingReplies:  # type: ignore[no-untyped-def]
    assert isinstance(ctx.replies, RecordingReplies)
    return ctx.replies


def events(ctx) -> RecordingEvents:  # type: ignore[no-untyped-def]
    assert isinstance(ctx.events, RecordingEvents)
    return ctx.events


# ---------------------------------------------------------------------------------------------- order A
def test_order_a_ack_error_decorated_logged_and_suppressed() -> None:
    ctx = make_worker_context()
    r = req(CAM_STOP)
    out = finish_camera(ctx, r, None, ERR, camera_type=LOW_MAG)
    decorated = ErrorCluster(True, ERR.code, "IMAQdx error Cam stop")
    assert out == decorated and ctx.state.error_out == decorated
    assert events(ctx).events == [LoggedEvent("ERROR", decorated, SERVER_CAMERA, "Stop", "Cam Stop", LOW_MAG)]
    (sent,) = replies(ctx).replies
    assert (sent.kind, sent.err, sent.sent) == (ReplyKind.ACK, decorated, False)    # send_error_replies FALSE
    assert r.replied


def test_order_a_ack_success_sent_not_logged() -> None:
    ctx = make_worker_context()
    out = finish_camera(ctx, req(CAM_STOP), None, NO_ERROR, camera_type=HIGH_MAG)
    assert out == NO_ERROR and events(ctx).events == []
    assert [(r.kind, r.err, r.sent) for r in replies(ctx).replies] == [(ReplyKind.ACK, NO_ERROR, True)]


def test_order_a_error_ack_sent_with_send_error_replies() -> None:
    ctx = make_worker_context(send_error_replies=True)
    finish_camera(ctx, req(CAM_STOP), None, ERR, camera_type=HIGH_MAG)
    assert replies(ctx).replies[0].sent


def test_order_a_typed_reply_carries_decorated_error() -> None:
    ctx = make_worker_context()
    out = finish_camera(ctx, req(GET_STATUS), False, ERR, camera_type=LOW_MAG)
    decorated = ErrorCluster(True, ERR.code, "IMAQdx error Cam Acquisition status")
    assert out == decorated
    (sent,) = replies(ctx).replies
    assert (sent.kind, sent.value, sent.err, sent.sent) == (ReplyKind.BOOL, False, decorated, True)
    assert events(ctx).events[0].err == decorated


def test_order_a_warning_not_decorated_not_logged_but_sent() -> None:
    ctx = make_worker_context()
    out = finish_camera(ctx, req(CAM_STOP), None, WARN, camera_type=HIGH_MAG)
    assert out is WARN and events(ctx).events == []
    assert replies(ctx).replies[0].sent and replies(ctx).replies[0].err is WARN


def test_dynamic_suffix_extra_and_camera_type_defaults() -> None:
    ctx = make_worker_context()
    finish_camera(ctx, req(CAM_SETUP), None, ERR, camera_type=None)        # suffix None → "", camera → HIGH_MAG
    assert events(ctx).events[0].err == ERR and events(ctx).events[0].camera_type == HIGH_MAG
    ctx = make_worker_context()
    finish_camera(ctx, req(CAM_SETUP), None, ERR, camera_type=LOW_MAG, suffix="Cam Setup", extra="Other")
    ev = events(ctx).events[0]
    assert ev.err.source == "IMAQdx error Cam Setup" and ev.extra == "Other" and ev.command == "Cam Setup"


def test_set_exposure_empty_suffix() -> None:
    ctx = make_worker_context()
    assert finish_camera(ctx, req(SET_EXPOSURE), None, ERR, camera_type=HIGH_MAG) == ERR


def test_force_sends_error_ack_in_order_a() -> None:
    ctx = make_worker_context()
    stub = ErrorCluster(True, 9507, "not implemented in this phase (phase 4)")
    af = spec("Cam Autofocus", ReplyKind.ACK, FinishOrder.A, extra="Autofocus", suffix="Cam Autofocus")
    out = finish_camera(ctx, req(af), None, stub, camera_type=HIGH_MAG, force=True)
    assert out.source == "not implemented in this phase (phase 4)Cam Autofocus"
    assert replies(ctx).replies[0].sent and replies(ctx).replies[0].err == out


# ---------------------------------------------------------------------------------------------- RAW_FIRST
def test_raw_first_ack_sends_raw_logs_decorated() -> None:
    ctx = make_worker_context(send_error_replies=True)
    out = finish_camera(ctx, req(SET_LED), None, ERR, camera_type=LOW_MAG)
    assert replies(ctx).replies[0].err == ERR                               # raw error replied
    decorated = ErrorCluster(True, ERR.code, "IMAQdx error Cam Set Led")
    assert out == decorated and events(ctx).events[0].err == decorated
    assert ctx.state.error_out == decorated


def test_raw_first_typed_sends_raw_logs_nothing() -> None:
    ctx = make_worker_context()
    out = finish_camera(ctx, req(GET_LED), True, ERR, camera_type=LOW_MAG)
    (sent,) = replies(ctx).replies
    assert (sent.kind, sent.value, sent.err, sent.sent) == (ReplyKind.BOOL, True, ERR, True)
    assert out == NO_ERROR and events(ctx).events == [] and ctx.state.error_out == NO_ERROR


def test_raw_first_heartbeat_and_dbl2d() -> None:
    ctx = make_worker_context(clock=FixedClock(123456))
    finish_camera(ctx, req(HEARTBEAT), ctx.tick(), NO_ERROR, camera_type=None)
    finish_camera(ctx, req(AF_DATA), None, NO_ERROR, camera_type=HIGH_MAG)       # None → 0×0 default
    hb, af = replies(ctx).replies
    assert (hb.kind, hb.value) == (ReplyKind.U32, 123456)
    assert af.kind is ReplyKind.DBL2D and af.value.shape == (0, 0)


# ---------------------------------------------------------------------------------------------- Z
def test_finish_z_no_decoration_command_none() -> None:
    ctx = make_worker_context()
    err = ErrorCluster(True, 9405, "Z move out of range")
    out = finish_z(ctx, req(Z_MOVE), None, err)
    assert out == err and ctx.state.error_out == err
    assert events(ctx).events == [LoggedEvent("ERROR", err, SERVER_Z, "Move Absolute", None, HIGH_MAG)]
    assert (replies(ctx).replies[0].err, replies(ctx).replies[0].sent) == (err, False)


def test_finish_z_typed_and_extra_override() -> None:
    ctx = make_worker_context()
    finish_z(ctx, req(Z_SERIAL), "SN1", NO_ERROR, extra="Custom")
    assert [(r.kind, r.value, r.sent) for r in replies(ctx).replies] == [(ReplyKind.STR, "SN1", True)]
    assert events(ctx).events == []


# ---------------------------------------------------------------------------------------------- reject_undecodable
@pytest.mark.parametrize(
    ("s", "kind", "value", "sent_err", "logged"),
    [
        (CAM_STOP, ReplyKind.ACK, None, "Variant To Data in alox_camera_server.viCam stop", True),
        (CAM_SETUP, ReplyKind.ACK, None, "Variant To Data in alox_camera_server.vi", True),
        (GET_EXPOSURE, ReplyKind.DBL, 0.0, "Variant To Data in alox_camera_server.vi", True),
        (GET_STATUS, ReplyKind.BOOL, False, "Variant To Data in alox_camera_server.viCam Acquisition status", True),
        (SET_LED, ReplyKind.ACK, None, "Variant To Data in alox_camera_server.vi", True),     # raw sent, out logged
        (GET_LED, ReplyKind.BOOL, False, "Variant To Data in alox_camera_server.vi", False),  # typed: nothing logged
        (STREAM, ReplyKind.IMAGE, None, "Variant To Data in alox_camera_server.vi", True),    # CUSTOM → A
        (Z_MOVE, ReplyKind.ACK, None, "Variant To Data in alox_camera_server.vi", True),
        (Z_SERIAL, ReplyKind.STR, "", "Variant To Data in alox_camera_server.vi", True),
        (Z_STATUS, ReplyKind.BOOL, False, "Variant To Data in alox_camera_server.vi", True),
    ],
)
def test_reject_undecodable_per_kind(s: CommandSpec, kind: ReplyKind, value: object, sent_err: str,
                                     logged: bool) -> None:
    ctx = make_worker_context(send_error_replies=True)
    r = req(s, decode_error=UNDECODABLE)
    reject_undecodable(ctx, r)
    (sent,) = replies(ctx).replies
    assert sent.kind is kind and sent.value == value and sent.err.source == sent_err and sent.err.code == 91
    assert r.replied
    if logged:
        (ev,) = events(ctx).events
        assert ev.extra == (s.extra or s.name) and ev.err.code == 91
        assert ev.server_type == (SERVER_Z if s.worker == Z_WORKER else SERVER_CAMERA)
        if s.worker == CAMERA_WORKER:
            assert ev.camera_type == HIGH_MAG and ev.command == s.name
    else:
        assert events(ctx).events == []


def test_reject_undecodable_reply_none_logs_only() -> None:
    ctx = make_worker_context(send_error_replies=True)
    reject_undecodable(ctx, req(GET_IMAGE, decode_error=UNDECODABLE))
    assert replies(ctx).replies == []
    (ev,) = events(ctx).events
    assert ev.extra == "Cam Get Image"                   # spec.extra empty → spec.name
    assert ev.err.source == "Variant To Data in alox_camera_server.viCam Get Image"


def test_reject_undecodable_never_forced() -> None:
    ctx = make_worker_context()                          # send_error_replies FALSE
    reject_undecodable(ctx, req(CAM_STOP, decode_error=UNDECODABLE))
    assert replies(ctx).replies[0].sent is False


def test_reject_undecodable_without_spec_is_dropped() -> None:
    ctx = make_worker_context()
    r = Request(command="#99", code=99, spec=None, conn_id=1, decode_error=UNDECODABLE)
    reject_undecodable(ctx, r)
    assert replies(ctx).replies == [] and events(ctx).events == []


# ---------------------------------------------------------------------------------------------- recording doubles
def test_recording_replies_suppression_warning_force() -> None:
    rr = RecordingReplies(send_error_replies=False)
    a, b, c, d = (req(CAM_STOP) for _ in range(4))
    assert rr.ack(a, ERR) is ERR
    assert rr.ack(b, WARN) is WARN
    assert rr.ack(c, ERR, force=True) is ERR
    assert rr.ack(d, NO_ERROR) is NO_ERROR
    assert [r.sent for r in rr.replies] == [False, True, True, True]
    assert len(rr.sent_replies) == 3 and rr.last is not None and rr.last.req is d
    rr2 = RecordingReplies(send_error_replies=True)
    rr2.ack(req(CAM_STOP), ERR)
    assert rr2.replies[0].sent


def test_every_reply_sink_method_sets_replied_and_returns() -> None:
    rr = RecordingReplies()
    image = ImageData(np.zeros((2, 3), np.uint8))
    calls = [
        (lambda r: rr.ack(r, ERR), ERR),
        (lambda r: rr.string(r, "s", ERR), NO_ERROR),
        (lambda r: rr.boolean(r, True, ERR), NO_ERROR),
        (lambda r: rr.dbl(r, 1.5, ERR), NO_ERROR),
        (lambda r: rr.u32(r, 7, ERR), NO_ERROR),
        (lambda r: rr.dbl_2d(r, np.zeros((1, 2)), ERR), NO_ERROR),
        (lambda r: rr.image(r, image, NO_ERROR), NO_ERROR),
        (lambda r: rr.error_reply(r, ReplyKind.ACK, ERR), ERR),
        (lambda r: rr.error_reply(r, ReplyKind.DBL, ERR), NO_ERROR),
        (lambda r: rr.error_reply(r, ReplyKind.NONE, ERR), ERR),
    ]
    for call, expected in calls:
        r = req(CAM_STOP)
        assert call(r) == expected
        assert r.replied
    assert len(rr.replies) == len(calls) - 1             # error_reply(NONE) sends nothing
    dbl_default = rr.replies[-1]
    assert (dbl_default.kind, dbl_default.value, dbl_default.err) == (ReplyKind.DBL, 0.0, ERR)


def test_error_reply_force_and_conn_zero() -> None:
    rr = RecordingReplies()
    rr.error_reply(req(CAM_STOP), ReplyKind.ACK, ErrorCluster(True, 9506, "Unknown command #99"), force=True)
    rr.error_reply(req(CAM_STOP), ReplyKind.ACK, ERR)
    internal = Request.make_internal("Exit", None)
    rr.ack(internal, NO_ERROR)
    rr.u32(req(HEARTBEAT, conn_id=0), 1, NO_ERROR)
    assert [r.sent for r in rr.replies] == [True, False, False, False]
    assert internal.replied


def test_recording_events_semantics() -> None:
    ev = RecordingEvents()
    assert ev.error(NO_ERROR, SERVER_CAMERA, "x") is NO_ERROR
    assert ev.error(WARN, SERVER_CAMERA, "x") is WARN
    assert ev.error(ERR, SERVER_CAMERA, "Setup", command="Cam Setup", camera_type=LOW_MAG) is ERR
    assert ev.error(ERR, SERVER_CAMERA, "y") is ERR
    assert ev.info(NO_ERROR, SERVER_Z, "Move Absolute: Z Reference resetted") is NO_ERROR
    assert ev.events == [
        LoggedEvent("ERROR", ERR, SERVER_CAMERA, "Setup", "Cam Setup", LOW_MAG),
        LoggedEvent("ERROR", ERR, SERVER_CAMERA, "y", "Get Heartbeat", HIGH_MAG),
        LoggedEvent("INFO", NO_ERROR, SERVER_Z, "Move Absolute: Z Reference resetted", None, HIGH_MAG),
    ]


# ---------------------------------------------------------------------------------------------- misc
def test_send_reply_none_and_defaults() -> None:
    ctx = make_worker_context()
    assert send_reply(ctx, req(GET_IMAGE), None, ERR) is ERR
    assert replies(ctx).replies == []
    assert send_reply(ctx, req(GET_EXPOSURE), None, ERR) == NO_ERROR
    assert replies(ctx).replies[0].value == 0.0
    with pytest.raises(ValueError):
        send_reply(ctx, Request(command="x", code=1, spec=None), None, NO_ERROR)


def test_reply_default_values() -> None:
    assert reply_default_value(ReplyKind.STR) == ""
    assert reply_default_value(ReplyKind.BOOL) is False
    assert reply_default_value(ReplyKind.DBL) == 0.0
    assert reply_default_value(ReplyKind.U32) == 0
    assert reply_default_value(ReplyKind.DBL2D).shape == (0, 0)
    assert reply_default_value(ReplyKind.IMAGE) is None
    assert reply_default_value(ReplyKind.ACK) is None


def test_request_helpers_and_outcomes() -> None:
    r = Request.make_internal("Exit", None)
    assert (r.code, r.conn_id, r.internal, r.replied) == (-1, 0, True, False)
    r2 = make_request("Cam Stop", "data", spec=CAM_STOP, code=12, ts=(1, 2, 3), notifier=b"N\x81")
    assert (r2.code, r2.data, r2.request_ts, r2.sending_ts, r2.receiving_ts) == (12, "data", 1, 2, 3)
    assert r2.notifier == b"N\x81" and r2.conn_id == 1 and r2.spec is CAM_STOP
    assert CONTINUE == Outcome() and STOP.stop and REQUEUE.requeue and not CONTINUE.stop


def test_make_worker_context_compat_override() -> None:
    compat = CompatSwitches(send_error_replies=True, restart_dh_macro=False)
    ctx = make_worker_context(compat=compat, send_error_replies=False)
    assert ctx.compat.send_error_replies is False and ctx.compat.restart_dh_macro is False
    assert replies(ctx).send_error_replies is False
    assert ctx.tick() == 0
