"""``Replier``: the :class:`aloxcam.dispatch.ReplySink` that builds ``Response`` frames and writes them on the link.

Spec: 02 §8.1–§8.4 (``HW TCP Response Data``, reply timestamps, ``Response (Flattened)`` per kind, which replies are
really sent), 02 §8.5 (image replies), 03 §1.1 (reply kinds), 01 §6 (response path), 02 §10 rule 3 (at most one
reply per request), 10 §6 (``send_error_replies``); docs/phase1-design.md §3.4.2, §7.

- **ACK** (``Enqueue Send Error Cluster Reponse.vi``): an error (``status`` TRUE) is not sent unless
  ``compat.send_error_replies`` or ``force``; warnings and successes are sent. Returns ``err``.
- **Typed replies** (STR, BOOL, DBL, U32, DBL2D, IMAGE): always sent, the error travels as data. Return NO_ERROR.
- **Timestamps** (02 §8.2): all 0 except ``TCP Sending`` = ``tick()`` just before the write; image replies also
  copy the request's three timestamps and take ``Pre-`` / ``Post-flattening`` around the codec call.

Every method sets ``req.replied = True`` first, also when the frame is suppressed or dropped; requests with
``conn_id == 0`` (internal) are never sent. Thread-safe: the only shared object is the :class:`ClientLink`.
"""

from __future__ import annotations

import logging
from dataclasses import replace
from typing import Any

import numpy as np

from aloxcam.clock import TickFn
from aloxcam.dispatch import ReplyKind, Request, reply_default_value
from aloxcam.errors import NO_ERROR, CodecError, ErrorCluster, cluster_of
from aloxcam.imagedata import ImageCodec, ImageData
from aloxcam.server.link import ClientLink
from aloxcam.settings import CompatSwitches
from aloxcam.wire.protocol import Datatypes, ResponseTimestamps, flatten_reply, response_frame

__all__ = ["Replier"]

log = logging.getLogger(__name__)


class Replier:
    """Implements :class:`aloxcam.dispatch.ReplySink` on top of a :class:`ClientLink` (02 §8)."""

    def __init__(self, link: ClientLink, datatypes: Datatypes, codec: ImageCodec, compat: CompatSwitches,
                 tick: TickFn, *, encoding: str = "cp1252") -> None:
        self.link = link
        self.datatypes = datatypes
        self.codec = codec
        self.compat = compat
        self.tick = tick
        self.encoding = encoding

    # ------------------------------------------------------------------ ReplySink
    def ack(self, req: Request, err: ErrorCluster, *, force: bool = False) -> ErrorCluster:
        """Error-cluster reply (02 §8.4): an error is suppressed unless ``send_error_replies`` or ``force``;
        returns ``err`` (the ACK VI's error out)."""
        req.replied = True
        if err.status and not force and not self.compat.send_error_replies:
            log.debug("error ACK suppressed (send_error_replies = FALSE): %s %r", req.command, err)
            return err
        self._send(req, ReplyKind.ACK, None, err, ResponseTimestamps())
        return err

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
        """Image reply (02 §8.2, §8.5; camera worker only): ``Pre-flattening`` tick → ``codec.flatten`` (skipped on
        error: the value is ``b""``) → ``Post-flattening`` tick; a codec failure becomes the reply's error with
        ``b""``. The request's three timestamps are copied. Returns NO_ERROR."""
        req.replied = True
        pre = self.tick()                 # LabVIEW's Tick Count nodes have no error input: taken on error too
        data = b""
        if not err.status:
            if image is None:
                log.warning("%s: image reply without an image; empty image sent", req.command)
            else:
                try:
                    data = self.codec.flatten(image)
                except CodecError as exc:
                    err = exc.cluster()
                except Exception as exc:  # a codec bug must not cost the client its reply
                    log.exception("%s: image codec %r failed", req.command, getattr(self.codec, "name", "?"))
                    err = cluster_of(exc)
        post = self.tick()
        ts = ResponseTimestamps(request=req.request_ts, sending=req.sending_ts, receiving=req.receiving_ts,
                                pre_flatten=pre, post_flatten=post)
        self._send(req, ReplyKind.IMAGE, data, err, ts)
        return NO_ERROR

    def error_reply(self, req: Request, kind: ReplyKind, err: ErrorCluster, *,
                    force: bool = False) -> ErrorCluster:
        """The kind's default value with ``err`` (03 §1.1); ACK honours ``send_error_replies`` unless ``force``;
        NONE sends nothing and returns ``err``."""
        if kind is ReplyKind.NONE:
            req.replied = True
            return err
        if kind is ReplyKind.ACK:
            return self.ack(req, err, force=force)
        if kind is ReplyKind.IMAGE:
            return self.image(req, None, err)
        return self._typed(kind, req, reply_default_value(kind), err)

    # ------------------------------------------------------------------ helpers
    def _typed(self, kind: ReplyKind, req: Request, value: Any, err: ErrorCluster) -> ErrorCluster:
        req.replied = True
        self._send(req, kind, value, err, ResponseTimestamps())
        return NO_ERROR

    def _send(self, req: Request, kind: ReplyKind, value: Any, err: ErrorCluster, ts: ResponseTimestamps) -> bool:
        """Flatten ``{value, error}``, stamp ``TCP Sending`` and write the frame (02 §8.1–§8.3)."""
        if req.conn_id == 0:
            log.debug("%s: internal request, %s reply not sent", req.command, kind.value)
            return False
        try:
            flattened = flatten_reply(kind, value, err, encoding=self.encoding, errors="replace")
        except Exception as exc:        # unencodable value (handler bug): reply the kind's default with the error
            log.exception("%s: cannot flatten the %s reply value; default value sent", req.command, kind.value)
            default = b"" if kind is ReplyKind.IMAGE else reply_default_value(kind)
            flattened = flatten_reply(kind, default, cluster_of(exc), encoding=self.encoding, errors="replace")
        frame = response_frame(self.datatypes, req.notifier, flattened, replace(ts, tcp_sending=self.tick()))
        sent = self.link.send(frame, req.conn_id)
        if not sent:
            log.debug("%s: %s reply for connection %d not delivered", req.command, kind.value, req.conn_id)
        return sent
