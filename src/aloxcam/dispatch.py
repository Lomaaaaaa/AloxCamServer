"""The handler contract: requests, command specs, outcomes, reply / event sinks and the finish helpers.

Spec: 03 §1.1 (reply kinds), §4.1–§4.5 (decoration, logging and reply order), 04 §4.1 / §5.1 (``Camera Queue Data``,
data modes), 02 §10 rule 3 (at most one reply per request), 10 §6 carve-out (a).

Every handler finishes a request with :func:`finish_camera` (camera worker) or :func:`finish_z` (Z worker), which
decorate, log and reply in the order of 03 §4.2 / §4.3 and record the logged error in ``ServerState.error_out``.
"""

from __future__ import annotations

import logging
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from enum import Enum
from typing import Any, Protocol

import numpy as np

from aloxcam.clock import TickFn
from aloxcam.errors import Code, ErrorCluster, decorate
from aloxcam.imagedata import ImageData
from aloxcam.settings import HIGH_MAG, CompatSwitches
from aloxcam.state import ServerState

__all__ = [
    "CAMERA_WORKER",
    "CONTINUE",
    "REQUEUE",
    "SERVER_CAMERA",
    "SERVER_Z",
    "STOP",
    "Z_WORKER",
    "BackgroundStepper",
    "CommandSpec",
    "DataMode",
    "EventSink",
    "FinishOrder",
    "Handler",
    "HandlerTable",
    "Outcome",
    "ReplyKind",
    "ReplySink",
    "Request",
    "WorkerContext",
    "finish_camera",
    "finish_z",
    "reject_undecodable",
    "reply_default_value",
    "send_reply",
]

log = logging.getLogger(__name__)

CAMERA_WORKER = "camera"
Z_WORKER = "z"
SERVER_CAMERA = "Camera"                 # Log Server Error "Server Type" (09 §7.1)
SERVER_Z = "Z Stage"


class ReplyKind(Enum):
    """Reply kinds of 03 §1.1 / 02 §8.3; the value is the kind's short name."""

    NONE = "none"
    ACK = "ACK"
    STR = "STR"
    BOOL = "BOOL"
    DBL = "DBL"
    U32 = "U32"
    DBL2D = "DBL2D"
    IMAGE = "IMAGE"


class DataMode(Enum):
    """How a command's ``Command Data`` is decoded (04 §5.1)."""

    NOT_READ = "not_read"        # LabVIEW never reads Command Data: any bytes accepted, data = None
    UNUSED = "unused"            # LabVIEW decodes {Extra Param} but never uses it: any bytes accepted, data = None
    STRICT = "strict"            # decoded with the payload layout; failure → Request.decode_error


class FinishOrder(Enum):
    """Order of decoration, logging and reply (03 §4.2, §4.3)."""

    A = "A"                      # decorate → log → reply (variant A, A')
    RAW_FIRST = "raw_first"      # reply raw error → decorate reply-out → log (variants B, C, D)
    Z = "z"                      # reply → log, no decoration (03 §4.3)
    CUSTOM = "custom"            # handler does it itself (variant E)


@dataclass(frozen=True)
class CommandSpec:
    """Static description of one ``Camera Command`` item (design §3.2.5 registry)."""

    name: str                    # exact enum item label
    worker: str                  # CAMERA_WORKER | Z_WORKER (loop that has the case)
    payload: type | None         # wire.messages dataclass; None = no payload type
    data_mode: DataMode
    reply: ReplyKind
    order: FinishOrder
    extra: str                   # Log Server Error "Extra String" (03 §5–§7)
    suffix: str | None           # camera source suffix; None = computed by the handler
    client_routable: bool = True  # False: Cam Autofocus Run / Cam Refocus Run (03 C16)
    spec_ref: str = ""           # e.g. "05 §4.2"


@dataclass
class Request:
    """Internal form of ``Camera Queue Data`` (04 §4.1) after decoding; travels by reference through the queues."""

    command: str                 # item label; "#<code>" when the code has no item
    code: int                    # wire code; -1 for internal requests
    spec: CommandSpec | None
    notifier: bytes = b""        # opaque, echoed byte for byte (02 §10 rule 1)
    data: Any = None             # decoded payload dataclass, or None
    raw_data: bytes = b""        # variant data bytes as received
    decode_error: ErrorCluster | None = None
    request_ts: int = 0
    sending_ts: int = 0
    receiving_ts: int = 0        # set by the router on the camera route (02 §7.1)
    conn_id: int = 0             # ClientLink generation; 0 = internal (never replied)
    internal: bool = False
    requeue_count: int = 0
    replied: bool = False        # set by every ReplySink method (at most one reply per request)

    @classmethod
    def make_internal(cls, command: str, spec: CommandSpec | None) -> Request:
        """An internal request (forwarded ``Exit``, shutdown ``Exit``): code -1, conn_id 0, never replied."""
        return cls(command=command, code=-1, spec=spec, conn_id=0, internal=True)


@dataclass(frozen=True)
class Outcome:
    """What the worker does after a handler returns (01 §5.3–§5.4)."""

    stop: bool = False           # Exit
    requeue: bool = False        # put the same Request back at the tail of the worker queue


CONTINUE = Outcome()
STOP = Outcome(stop=True)
REQUEUE = Outcome(requeue=True)


class ReplySink(Protocol):
    """Reply path (``server.reply.Replier``, ``testing.RecordingReplies``); thread-safe.

    Every method sets ``req.replied = True`` (also when the frame is suppressed or dropped). ``force=True`` sends an
    error ACK even when ``send_error_replies`` is FALSE: used ONLY for 9506 (``reply_unknown_commands``) and the
    Phase-1 stubs 9507 (design §7). ``ack`` returns ``err`` (the ACK VI's error out); typed replies return NO_ERROR.
    """

    def ack(self, req: Request, err: ErrorCluster, *, force: bool = False) -> ErrorCluster: ...
    def string(self, req: Request, value: str, err: ErrorCluster) -> ErrorCluster: ...
    def boolean(self, req: Request, value: bool, err: ErrorCluster) -> ErrorCluster: ...
    def dbl(self, req: Request, value: float, err: ErrorCluster) -> ErrorCluster: ...
    def u32(self, req: Request, value: int, err: ErrorCluster) -> ErrorCluster: ...
    def dbl_2d(self, req: Request, value: np.ndarray, err: ErrorCluster) -> ErrorCluster: ...
    def image(self, req: Request, image: ImageData | None, err: ErrorCluster) -> ErrorCluster: ...

    def error_reply(self, req: Request, kind: ReplyKind, err: ErrorCluster, *,
                    force: bool = False) -> ErrorCluster:
        """The kind's default value with ``err``; kind NONE sends nothing."""
        ...


class EventSink(Protocol):
    """LabVIEW-format event logs (``eventlog.EventLogs``, ``testing.RecordingEvents``); thread-safe (09 §7)."""

    def error(self, err: ErrorCluster, server_type: str, extra: str, *, command: str | None = "Get Heartbeat",
              camera_type: str = HIGH_MAG) -> ErrorCluster:
        """``Log Server Error``: writes only if ``err.status``; returns ``err``."""
        ...

    def info(self, err: ErrorCluster, server_type: str, extra: str, *,
             camera_type: str = HIGH_MAG) -> ErrorCluster:
        """``Log Server Infos``: always writes; returns ``err``."""
        ...


Handler = Callable[[Any, Request], Outcome | None]       # (context, request); None = CONTINUE
HandlerTable = Mapping[str, Handler]                     # command label → handler


class BackgroundStepper(Protocol):
    """Phase 4 focus stepping (06 §9.5); None in Phase 1."""

    def has_work(self) -> bool: ...
    def step(self) -> None: ...
    def abort(self) -> None: ...


@dataclass
class WorkerContext:
    """Base context of a worker; ``CameraContext`` / ``ZContext`` subclass it."""

    state: ServerState
    replies: ReplySink
    events: EventSink
    compat: CompatSwitches
    tick: TickFn


# ---------------------------------------------------------------------------------------------- finish helpers
def reply_default_value(kind: ReplyKind) -> Any:
    """LabVIEW default value of a reply kind (03 §1.1): ``""``, FALSE, 0.0, 0, a 0×0 array, None (image, ACK)."""
    if kind is ReplyKind.STR:
        return ""
    if kind is ReplyKind.BOOL:
        return False
    if kind is ReplyKind.DBL:
        return 0.0
    if kind is ReplyKind.U32:
        return 0
    if kind is ReplyKind.DBL2D:
        return np.empty((0, 0))
    return None


def _spec(req: Request) -> CommandSpec:
    if req.spec is None:
        raise ValueError(f"request {req.command!r} has no CommandSpec")
    return req.spec


def send_reply(ctx: WorkerContext, req: Request, value: Any, err: ErrorCluster, *,
               force: bool = False) -> ErrorCluster:
    """Reply with the kind of ``req.spec.reply`` (03 §1.1) and return the reply VI's error out.

    NONE sends nothing and returns ``err``; ACK returns ``err``; typed replies return NO_ERROR. A ``value`` of None
    is replaced by the kind's default value. ``force`` is passed to ``ack()`` only.
    """
    kind = _spec(req).reply
    if kind is ReplyKind.NONE:
        return err
    if kind is ReplyKind.ACK:
        return ctx.replies.ack(req, err, force=force)
    if value is None and kind is not ReplyKind.IMAGE:
        value = reply_default_value(kind)
    if kind is ReplyKind.STR:
        return ctx.replies.string(req, value, err)
    if kind is ReplyKind.BOOL:
        return ctx.replies.boolean(req, value, err)
    if kind is ReplyKind.DBL:
        return ctx.replies.dbl(req, value, err)
    if kind is ReplyKind.U32:
        return ctx.replies.u32(req, value, err)
    if kind is ReplyKind.DBL2D:
        return ctx.replies.dbl_2d(req, value, err)
    return ctx.replies.image(req, value, err)


def finish_camera(ctx: WorkerContext, req: Request, value: Any, err: ErrorCluster, *, camera_type: str | None,
                  suffix: str | None = None, extra: str | None = None,
                  order: FinishOrder | None = None, force: bool = False) -> ErrorCluster:
    """Finish a camera-worker command (03 §4.1–§4.2, §4.5); returns the logged error (also put in ``error_out``).

    Order A (and CUSTOM): ``e = decorate(err, suffix)`` → ``events.error(e, …)`` → reply ``e``.
    RAW_FIRST: reply ``err`` → ``e = decorate(reply_out, suffix)`` → ``events.error(e, …)`` (typed replies return
    NO_ERROR, so nothing is logged for them). Z: reply → log, no decoration.
    Defaults come from ``req.spec``: ``suffix = spec.suffix or ""``, ``extra = spec.extra or spec.name``,
    ``order = spec.order``. ``camera_type`` None → ``HIGH_MAG`` (LabVIEW unwired input); pass the real type
    whenever known (09 D15 fix).
    """
    spec = _spec(req)
    order = order or spec.order
    suffix = suffix if suffix is not None else (spec.suffix or "")
    extra = extra if extra is not None else (spec.extra or spec.name)
    cam = camera_type or HIGH_MAG
    if order is FinishOrder.RAW_FIRST:
        out = send_reply(ctx, req, value, err, force=force)
        e = decorate(out, suffix)
        ctx.events.error(e, SERVER_CAMERA, extra, command=req.command, camera_type=cam)
    elif order is FinishOrder.Z:
        send_reply(ctx, req, value, err, force=force)
        e = err
        ctx.events.error(e, SERVER_CAMERA, extra, command=req.command, camera_type=cam)
    else:                                         # A, A' and CUSTOM handlers finishing like A
        e = decorate(err, suffix)
        ctx.events.error(e, SERVER_CAMERA, extra, command=req.command, camera_type=cam)
        send_reply(ctx, req, value, e, force=force)
    ctx.state.set_error_out(e)
    return e


def finish_z(ctx: WorkerContext, req: Request, value: Any, err: ErrorCluster, *,
             extra: str | None = None) -> ErrorCluster:
    """Finish a Z-worker command (03 §4.3): reply ``err``, then ``events.error(err, "Z Stage", extra,
    command=None)``; no decoration. ``extra`` defaults to ``spec.extra or spec.name``. Returns ``err``."""
    spec = _spec(req)
    extra = extra if extra is not None else (spec.extra or spec.name)
    send_reply(ctx, req, value, err)
    ctx.events.error(err, SERVER_Z, extra, command=None)
    ctx.state.set_error_out(err)
    return err


def reject_undecodable(ctx: WorkerContext, req: Request) -> None:
    """Finish a request whose payload or envelope did not decode, with ``req.decode_error`` (04 T1, design §2).

    Value = the kind's default. Camera → :func:`finish_camera` with ``spec.order`` (CUSTOM and Z → A), suffix
    ``spec.suffix or ""``, camera type HIGH_MAG (unknown); Z → :func:`finish_z`. Reply kind NONE → log only.
    ``extra = spec.extra or spec.name``. Never forced (10 §6 carve-out (a): follows ``send_error_replies``).
    The worker also uses it for a handler exception (after setting ``decode_error = cluster_of(exc)``).
    """
    err = req.decode_error
    if err is None:
        err = ErrorCluster(True, Code.INTERNAL, f"{req.command}: rejected without a decode error")
    spec = req.spec
    if spec is None:
        log.warning("undecodable request %s without CommandSpec dropped: %s", req.command, err)
        return
    value = reply_default_value(spec.reply)
    extra = spec.extra or spec.name
    if spec.worker == Z_WORKER:
        finish_z(ctx, req, value, err, extra=extra)
        return
    order = spec.order if spec.order in (FinishOrder.A, FinishOrder.RAW_FIRST) else FinishOrder.A
    finish_camera(ctx, req, value, err, camera_type=HIGH_MAG, suffix=spec.suffix or "", extra=extra, order=order)
