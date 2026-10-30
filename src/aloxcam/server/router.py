"""``Router``: decodes a ``Command`` payload, records it, and routes it to the camera or Z worker.

Spec: 02 §7.1–§7.6 (envelope, routing, malformed commands), 03 §3.1–§3.4 (routing rule, router side effects,
values that produce nothing), 01 §5.2.2, 01 A8 / A20, 04 T1 / T8, 10 §6 carve-out (a); docs/phase1-design.md §2,
§3.4.3, §7.

``route_frame``:

1. ``decode_envelope``. An undecodable envelope whose command code and notifier were decoded, with a routed code of a
   known command, is queued with ``decode_error = {TRUE, 116, "Unflatten From String in alox_camera_server.vi"}``
   (timestamps 0, no data: the worker rejects it like an undecodable payload); any other undecodable envelope is
   logged (ERROR, hex excerpt) and dropped.
2. ``state.record_command(name)`` (unknown codes included, as ``"#<code>"``).
3. Not routed → WARNING; with ``reply_unknown_commands`` a forced ``{TRUE, 9506, "Unknown command <name>"}`` reply.
4. Known command → payload decoded strictly; a failure gives ``decode_error = {TRUE, 91, "Variant To Data in
   alox_camera_server.vi"}``. A routed item that is not one of the known commands → ``Request(spec=None)``.
5. Camera route: ``receiving_ts = tick()`` (the Z route keeps the client's value); queued without blocking.

Called only from the TCP thread.
"""

from __future__ import annotations

import logging
from collections.abc import Callable, Mapping

from aloxcam.clock import TickFn
from aloxcam.dispatch import CAMERA_WORKER, CommandSpec, ReplyKind, ReplySink, Request
from aloxcam.errors import Code, ErrorCluster, ProtocolError
from aloxcam.settings import CompatSwitches
from aloxcam.state import ServerState
from aloxcam.wire.enums import RoutingTable, WireEnums
from aloxcam.wire.messages import PayloadCodec
from aloxcam.wire.protocol import EnvelopeError, decode_envelope

__all__ = ["ENVELOPE_ERROR", "PAYLOAD_ERROR", "Router", "unknown_command_error"]

log = logging.getLogger(__name__)

#: Error of a command whose envelope did not decode (02 §7.6, 04 T8): LabVIEW's ``Unflatten From String``.
ENVELOPE_ERROR = ErrorCluster(True, Code.UNFLATTEN_FAILED, "Unflatten From String in alox_camera_server.vi")
#: Error of a known command whose ``Command Data`` did not decode (04 T1): LabVIEW's ``Variant To Data``.
PAYLOAD_ERROR = ErrorCluster(True, Code.VARIANT_TYPE_MISMATCH, "Variant To Data in alox_camera_server.vi")

_HEX_EXCERPT = 64


def unknown_command_error(name: str) -> ErrorCluster:
    """``{TRUE, 9506, "Unknown command <name>"}`` (``reply_unknown_commands``, 10 §6)."""
    return ErrorCluster(True, Code.UNKNOWN_COMMAND, f"Unknown command {name}")


def _hex_excerpt(payload: bytes) -> str:
    text = payload[:_HEX_EXCERPT].hex(" ")
    return text + (f" … ({len(payload)} bytes)" if len(payload) > _HEX_EXCERPT else "")


class Router:
    """Envelope decoding, ``Commands History``, routing and receiving timestamp (02 §7, 03 §3). TCP thread only."""

    def __init__(self, *, enums: WireEnums, routing: RoutingTable, commands: Mapping[str, CommandSpec],
                 payloads: PayloadCodec, state: ServerState, replies: ReplySink, compat: CompatSwitches,
                 tick: TickFn, camera_put: Callable[[Request], None], z_put: Callable[[Request], None]) -> None:
        self.enums = enums
        self.routing = routing
        self.commands = commands
        self.payloads = payloads
        self.state = state
        self.replies = replies
        self.compat = compat
        self.tick = tick
        self._camera_put = camera_put
        self._z_put = z_put

    def route_frame(self, payload: bytes, conn_id: int) -> Request | None:
        """Decode, record and route one ``Command`` payload; returns the queued :class:`Request` or None (dropped
        or unknown command)."""
        try:
            env = decode_envelope(payload, self.enums)
        except EnvelopeError as exc:
            return self._envelope_error(exc, payload, conn_id)
        name = self.routing.name(env.code)
        self.state.record_command(name)
        req = Request(command=name, code=env.code, spec=self.commands.get(name), notifier=env.notifier,
                      raw_data=env.data, request_ts=env.request_ts, sending_ts=env.sending_ts,
                      receiving_ts=env.receiving_ts, conn_id=conn_id)
        route = self.routing.route(env.code)
        if route is None:
            self._unknown(req)
            return None
        if req.spec is not None:
            try:
                req.data = self.payloads.decode(req.spec, env.data)
            except Exception as exc:        # ProtocolError normally; anything else is a codec bug, rejected alike
                if isinstance(exc, ProtocolError):
                    log.warning("%s (connection %d): undecodable Command Data rejected (91): %s", name, conn_id,
                                exc.source)
                else:
                    log.exception("%s (connection %d): payload decoder failed; rejected (91)", name, conn_id)
                req.decode_error = PAYLOAD_ERROR
        return self._dispatch(req, route)

    # ------------------------------------------------------------------ helpers
    def _dispatch(self, req: Request, route: str) -> Request:
        if route == CAMERA_WORKER:
            req.receiving_ts = self.tick()  # "Store receiving timestamp for image statistics" (02 §7.1)
            self._camera_put(req)
        else:
            self._z_put(req)
        return req

    def _envelope_error(self, exc: EnvelopeError, payload: bytes, conn_id: int) -> Request | None:
        """02 §7.6 / 04 T8: reject with 116 when the code and notifier decoded and name a routed known command."""
        code, notifier = exc.code_value, exc.notifier
        if code is not None and notifier is not None:
            name = self.routing.name(code)
            route = self.routing.route(code)
            spec = self.commands.get(name)
            if route is not None and spec is not None:
                log.warning("%s (connection %d): undecodable command envelope rejected (116): %s", name, conn_id,
                            exc.source)
                self.state.record_command(name)
                req = Request(command=name, code=code, spec=spec, notifier=notifier, decode_error=ENVELOPE_ERROR,
                              conn_id=conn_id)
                return self._dispatch(req, route)
        log.error("connection %d: undecodable command frame dropped (%s); payload: %s", conn_id, exc.source,
                  _hex_excerpt(payload))
        return None

    def _unknown(self, req: Request) -> None:
        """Router ``Default`` (03 §3.4, 01 A8): no reply unless ``reply_unknown_commands`` (forced 9506 ACK or the
        command's own reply kind, design §7)."""
        log.warning("unknown command %s (code %d, connection %d): not routed", req.command, req.code, req.conn_id)
        if not self.compat.reply_unknown_commands:
            return
        spec = req.spec
        kind = spec.reply if spec is not None and spec.reply is not ReplyKind.NONE else ReplyKind.ACK
        self.replies.error_reply(req, kind, unknown_command_error(req.command), force=True)
