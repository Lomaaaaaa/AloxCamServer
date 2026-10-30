"""Test doubles shared by every package: a fixed clock, recording reply / event sinks and request builders.

Spec: docs/phase1-design.md §3.1.7; 03 §1.1 and 02 §8.4 (reply suppression), 09 §7.1 (event-log semantics),
09 §4 / §5.6 (reference ``alox_setup.ini`` / ``alox.ini`` tree).

:class:`RecordingReplies` reproduces the return / suppression / ``force`` / ``replied`` semantics of
``server.reply.Replier`` without a socket; :class:`RecordingEvents` reproduces ``Log Server Error`` (records only
``status`` TRUE) and ``Log Server Infos`` (always records). WP2 is imported lazily (``make_request``,
``packaged_enums``), so WP1 tests that pass ``spec`` and ``code`` explicitly never need it.
"""

from __future__ import annotations

import functools
import threading
from dataclasses import dataclass, replace
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from aloxcam.dispatch import CommandSpec, ReplyKind, Request, WorkerContext, reply_default_value
from aloxcam.errors import NO_ERROR, ErrorCluster
from aloxcam.imagedata import ImageData
from aloxcam.settings import HIGH_MAG, AloxSettings, CompatSwitches, ServerOptions
from aloxcam.state import ServerState

if TYPE_CHECKING:
    from aloxcam.wire.enums import WireEnums

__all__ = [
    "REFERENCE_ALOX_INI",
    "AloxTree",
    "FixedClock",
    "LoggedEvent",
    "RecordingEvents",
    "RecordingReplies",
    "SentReply",
    "make_request",
    "make_state",
    "make_worker_context",
    "packaged_enums",
    "write_alox_tree",
]


class FixedClock:
    """A :data:`aloxcam.clock.TickFn` that only moves when told to (U32 wrap-around kept)."""

    def __init__(self, start: int = 0) -> None:
        self._value = start & 0xFFFFFFFF
        self._lock = threading.Lock()

    def __call__(self) -> int:
        with self._lock:
            return self._value

    def set(self, value: int) -> None:
        """Set the tick to ``value`` (masked to U32)."""
        with self._lock:
            self._value = value & 0xFFFFFFFF

    def advance(self, ms: int) -> None:
        """Advance by ``ms`` milliseconds, wrapping at 2**32."""
        with self._lock:
            self._value = (self._value + ms) & 0xFFFFFFFF


@dataclass
class SentReply:
    """One call recorded by :class:`RecordingReplies`.

    ``sent`` is False when the frame would not reach the client: an error ACK suppressed by
    ``send_error_replies`` (02 §8.4), or a request with ``conn_id == 0`` (internal, never replied).
    """

    kind: ReplyKind
    req: Request
    value: Any
    err: ErrorCluster
    sent: bool


class RecordingReplies:
    """:class:`aloxcam.dispatch.ReplySink` that records replies instead of sending frames; thread-safe."""

    def __init__(self, send_error_replies: bool = False) -> None:
        self.send_error_replies = send_error_replies
        self.replies: list[SentReply] = []
        self._lock = threading.Lock()

    # -- helpers (not part of ReplySink)
    @property
    def sent_replies(self) -> list[SentReply]:
        """The replies that reached the client (``sent`` TRUE)."""
        with self._lock:
            return [r for r in self.replies if r.sent]

    @property
    def last(self) -> SentReply | None:
        """The last recorded reply, or None."""
        with self._lock:
            return self.replies[-1] if self.replies else None

    def clear(self) -> None:
        """Forget every recorded reply."""
        with self._lock:
            self.replies.clear()

    def _record(self, kind: ReplyKind, req: Request, value: Any, err: ErrorCluster, sent: bool) -> None:
        with self._lock:
            self.replies.append(SentReply(kind, req, value, err, sent and req.conn_id != 0))

    # -- ReplySink
    def ack(self, req: Request, err: ErrorCluster, *, force: bool = False) -> ErrorCluster:
        req.replied = True
        suppressed = err.status and not force and not self.send_error_replies
        self._record(ReplyKind.ACK, req, None, err, not suppressed)
        return err

    def _typed(self, kind: ReplyKind, req: Request, value: Any, err: ErrorCluster) -> ErrorCluster:
        req.replied = True
        self._record(kind, req, value, err, True)
        return NO_ERROR

    def string(self, req: Request, value: str, err: ErrorCluster) -> ErrorCluster:
        return self._typed(ReplyKind.STR, req, value, err)

    def boolean(self, req: Request, value: bool, err: ErrorCluster) -> ErrorCluster:
        return self._typed(ReplyKind.BOOL, req, value, err)

    def dbl(self, req: Request, value: float, err: ErrorCluster) -> ErrorCluster:
        return self._typed(ReplyKind.DBL, req, value, err)

    def u32(self, req: Request, value: int, err: ErrorCluster) -> ErrorCluster:
        return self._typed(ReplyKind.U32, req, value, err)

    def dbl_2d(self, req: Request, value: np.ndarray, err: ErrorCluster) -> ErrorCluster:
        return self._typed(ReplyKind.DBL2D, req, value, err)

    def image(self, req: Request, image: ImageData | None, err: ErrorCluster) -> ErrorCluster:
        return self._typed(ReplyKind.IMAGE, req, image, err)

    def error_reply(self, req: Request, kind: ReplyKind, err: ErrorCluster, *,
                    force: bool = False) -> ErrorCluster:
        if kind is ReplyKind.NONE:
            req.replied = True
            return err
        if kind is ReplyKind.ACK:
            return self.ack(req, err, force=force)
        return self._typed(kind, req, reply_default_value(kind), err)


@dataclass
class LoggedEvent:
    """One call recorded by :class:`RecordingEvents`; ``level`` is ``"ERROR"`` or ``"INFO"``."""

    level: str
    err: ErrorCluster
    server_type: str
    extra: str
    command: str | None
    camera_type: str


class RecordingEvents:
    """:class:`aloxcam.dispatch.EventSink` that records events; ``error()`` records only ``status`` TRUE."""

    def __init__(self) -> None:
        self.events: list[LoggedEvent] = []
        self._lock = threading.Lock()

    def error(self, err: ErrorCluster, server_type: str, extra: str, *, command: str | None = "Get Heartbeat",
              camera_type: str = HIGH_MAG) -> ErrorCluster:
        if err.status:
            with self._lock:
                self.events.append(LoggedEvent("ERROR", err, server_type, extra, command, camera_type))
        return err

    def info(self, err: ErrorCluster, server_type: str, extra: str, *,
             camera_type: str = HIGH_MAG) -> ErrorCluster:
        with self._lock:
            self.events.append(LoggedEvent("INFO", err, server_type, extra, None, camera_type))
        return err

    def clear(self) -> None:
        """Forget every recorded event."""
        with self._lock:
            self.events.clear()


@functools.lru_cache(maxsize=1)
def packaged_enums() -> WireEnums:
    """The packaged wire enum table (``aloxcam/data/wire_enums.toml``), loaded once (needs WP2)."""
    from aloxcam.wire.enums import WireEnums

    return WireEnums.load()


def make_request(command: str, data: Any = None, *, notifier: bytes = b"N1", conn_id: int = 1,
                 ts: tuple[int, int, int] = (0, 0, 0), decode_error: ErrorCluster | None = None,
                 spec: CommandSpec | None = None, code: int | None = None) -> Request:
    """A client request as the router would queue it.

    ``spec`` / ``code`` given → used as is. Otherwise ``spec`` is looked up in ``wire.messages.COMMANDS`` (None
    when the name is unknown) and ``code`` in the packaged enum table (``"#<n>"`` → n; :class:`ValueError` for an
    unknown label). Both lookups import WP2 lazily.
    """
    if spec is None:
        from aloxcam.wire.messages import COMMANDS

        spec = COMMANDS.get(command)
    if code is None:
        code = packaged_enums().code("camera_command", command)
    return Request(command=command, code=code, spec=spec, notifier=notifier, data=data, decode_error=decode_error,
                   request_ts=ts[0], sending_ts=ts[1], receiving_ts=ts[2], conn_id=conn_id)


def make_state(**overrides: Any) -> ServerState:
    """A ``ServerState`` from ``AloxSettings()`` defaults; ``options=`` sizes the histories, other keywords are
    applied with ``ServerState.update``."""
    options = overrides.pop("options", ServerOptions())
    state = ServerState.from_settings(AloxSettings(), options)
    if overrides:
        state.update(**overrides)
    return state


def make_worker_context(*, compat: CompatSwitches | None = None, clock: FixedClock | None = None,
                        send_error_replies: bool | None = None) -> WorkerContext:
    """A ``WorkerContext`` with :class:`RecordingReplies`, :class:`RecordingEvents`, :func:`make_state` and a
    :class:`FixedClock`. ``send_error_replies`` overrides the value in ``compat``."""
    compat = compat or CompatSwitches()
    if send_error_replies is not None:
        compat = replace(compat, send_error_replies=send_error_replies)
    return WorkerContext(state=make_state(), replies=RecordingReplies(send_error_replies=compat.send_error_replies),
                         events=RecordingEvents(), compat=compat, tick=clock or FixedClock())


# ---------------------------------------------------------------------------------------------- INI tree
#: The 09 §5.6 reference ``alox.ini``: every key at its hard-coded default (``AloxSettings()``).
REFERENCE_ALOX_INI = """\
; <settings>\\alox.ini — hard-coded defaults of alox_camera_server.vi (stage 2)
[general]
core_I32 = 1
setLEDcam_I32 = 0

[log]
maxlines_I32 = 1000

[hardware.camera]
port_I32 = 503
lightStatusInverter_BOOL = TRUE

[hardware.camera.highmag]
name_STR = "HighMag"
buffers_I32 = 5
timeout_I32 = 5000
flatfieldcorrection_BOOL = FALSE
flatfieldimage_PATH =
darkfieldimage_PATH =
flatfieldfactor_DBL = 1
exposure_STR = "CameraAttributes::Exposure::Value"
gain_STR = "CameraAttributes::Gain::Value"
exposuretype_STR = "DBL"
gaintype_STR = "I64"
hflip_BOOL = FALSE
vflip_BOOL = FALSE
framerate_STR = "CameraAttributes::AcquisitionControl::AcquisitionFrameRate"
frameratetype_STR = "DBL"
LimitMinExposure_DBL = 177
waitled_U32 = 0
invertled_BOOL = FALSE
cameraconnectortype_STR = "gige"
setLEDline_I32 = 0
setBULBline_I32 = 0
; frameratemax_DBL is not read for High Mag

[hardware.camera.lowmag]
name_STR = "LowMag"
buffers_I32 = 5
timeout_I32 = 5000
flatfieldcorrection_BOOL = FALSE
flatfieldimage_PATH =
darkfieldimage_PATH =
flatfieldfactor_DBL = 1
exposure_STR = "CameraAttributes::Exposure::Value"
gain_STR = "CameraAttributes::Gain::Value"
exposuretype_STR = "DBL"
gaintype_STR = "I64"
hflip_BOOL = FALSE
vflip_BOOL = FALSE
framerate_STR = "CameraAttributes::AcquisitionControl::AcquisitionFrameRate"
frameratetype_STR = "DBL"
frameratemax_DBL = 24.6
LimitMinExposure_DBL = 177
waitled_U32 = 0
invertled_BOOL = TRUE
cameraconnectortype_STR = "gige"

[hardware.z]
comport_I32 = 2
baudrate_I32 = 3
zaxis_I32 = 1
speed_DBL = 10
slowspeed_DBL = 1
zmin_DBL = 0
zmax_DBL = 20
"""


@dataclass(frozen=True)
class AloxTree:
    """A temporary installation: ``exe/alox_setup.ini`` pointing at ``settings/``, ``vision/`` and ``data/``."""

    root: Path
    exe: Path
    settings: Path
    vision: Path
    data: Path
    setup_ini: Path
    alox_ini: Path


def write_alox_tree(root: Path, *, alox_ini: str = REFERENCE_ALOX_INI) -> AloxTree:
    """Create ``root/{exe,settings,vision,data}``, ``exe/alox_setup.ini`` (``[folders]`` = POSIX paths inside
    ``root``) and ``settings/alox.ini`` (CRLF, cp1252). ``logs`` folders and ``alox_camera_server.ini`` are not
    created (tests add them as needed)."""
    root = Path(root)
    exe, settings, vision, data = (root / name for name in ("exe", "settings", "vision", "data"))
    for folder in (exe, settings, vision, data):
        folder.mkdir(parents=True, exist_ok=True)
    setup_ini = exe / "alox_setup.ini"
    setup_text = f"[folders]\nsettings = {settings}\nvision = {vision}\ndata = {data}\n"
    setup_ini.write_bytes(setup_text.replace("\n", "\r\n").encode("cp1252"))
    ini = settings / "alox.ini"
    ini.write_bytes(alox_ini.replace("\r\n", "\n").replace("\n", "\r\n").encode("cp1252"))
    return AloxTree(root=root, exe=exe, settings=settings, vision=vision, data=data, setup_ini=setup_ini,
                    alox_ini=ini)
