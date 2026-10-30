"""TCP frames, the inbound ``Camera Queue Data`` envelope and the outbound ``HW TCP Response Data`` payloads.

Spec: 02 §5 (frame layout, ``HW TCP Datatype``), §7.1 / §7.4 (envelope and its parse), §7.6 (malformed commands),
§8.1–§8.3 (response data, timestamps, ``Response (Flattened)`` per reply kind), §8.6 (worked examples), §10
(notifier correlation), §11.1 (reference codec).

Frame: ``I32 N (= 2 + payload length) · U16 datatype · payload``. Envelope: ``command code (camera_command size) ·
notifier (I32 + bytes) · variant (header · data · U32 0) · 3 × U32``. Response data: ``notifier · Response
(Flattened) · 6 × U32``. The notifier is opaque bytes, echoed byte for byte (02 §10 rule 1).
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Any

from aloxcam.dispatch import ReplyKind
from aloxcam.errors import Code, ErrorCluster, ProtocolError
from aloxcam.lv.flatten import Reader, Writer
from aloxcam.lv.variant import VariantHeader, read_variant_header
from aloxcam.wire.enums import WireEnums

__all__ = [
    "ENVELOPE_TRAILER_SIZE",
    "FRAME_HEADER_SIZE",
    "CommandEnvelope",
    "Datatypes",
    "EnvelopeError",
    "FrameHeader",
    "ResponseData",
    "ResponseTimestamps",
    "decode_envelope",
    "decode_response_data",
    "encode_envelope",
    "encode_frame",
    "encode_response_data",
    "flatten_reply",
    "heartbeat_frame",
    "parse_header",
    "parse_reply_value",
    "response_frame",
]

FRAME_HEADER_SIZE = 6
ENVELOPE_TRAILER_SIZE = 16              # U32 attribute count + 3 × U32 timestamps (02 §7.4)
_DATATYPE_ITEMS = ("Command", "Response", "Image", "Heartbeat")


@dataclass(frozen=True)
class Datatypes:
    """``HW TCP Datatype`` codes from the enum table (02 §5.2)."""

    command: int
    response: int
    image: int
    heartbeat: int

    @classmethod
    def from_enums(cls, enums: WireEnums) -> Datatypes:
        """Codes of the four items the server knows (``Statut`` is not used by the server)."""
        return cls(*(enums.code("tcp_datatype", item) for item in _DATATYPE_ITEMS))

    def name_of(self, code: int) -> str | None:
        """``"Command"``, ``"Response"``, ``"Image"`` or ``"Heartbeat"``; None for any other code."""
        for name, value in zip(_DATATYPE_ITEMS, (self.command, self.response, self.image, self.heartbeat), strict=True):
            if value == code:
                return name
        return None


@dataclass(frozen=True)
class FrameHeader:
    """The 6 header bytes, decoded (02 §5.1)."""

    payload_len: int
    datatype: int


def encode_frame(datatype: int, payload: bytes = b"") -> bytes:
    """``>iH`` header + payload (02 §5.1, §5.4)."""
    try:
        return struct.pack(">iH", 2 + len(payload), datatype) + payload
    except struct.error as exc:
        raise ValueError(f"cannot frame datatype {datatype} with {len(payload)} payload bytes: {exc}") from None


def parse_header(header6: bytes, max_payload: int) -> FrameHeader:
    """Decode the header; :class:`ProtocolError` if ``N < 2`` or ``N - 2 > max_payload`` (02 §5.5, D5, D6)."""
    if len(header6) != FRAME_HEADER_SIZE:
        raise ProtocolError(Code.UNFLATTEN_FAILED, f"frame header must be 6 bytes, got {len(header6)}")
    n, datatype = struct.unpack(">iH", header6)
    if n < 2:
        raise ProtocolError(Code.UNFLATTEN_FAILED, f"bad frame length N = {n} (< 2)")
    if n - 2 > max_payload:
        raise ProtocolError(Code.UNFLATTEN_FAILED, f"frame payload of {n - 2} bytes exceeds {max_payload}")
    return FrameHeader(payload_len=n - 2, datatype=datatype)


def heartbeat_frame(datatypes: Datatypes, payload: bytes = b"") -> bytes:
    """``00 00 00 02 tt tt`` (02 §5.4); a payload makes the LabVIEW server close the connection (02 §9)."""
    return encode_frame(datatypes.heartbeat, payload)


# ---------------------------------------------------------------------------------------------- inbound envelope
@dataclass(frozen=True)
class CommandEnvelope:
    """``Camera Queue Data`` as flattened (02 §7.1); the command code is a raw int (never mapped here)."""

    code: int
    notifier: bytes
    variant: VariantHeader
    data: bytes                           # variant data part (between the header and the U32 attribute count)
    request_ts: int
    sending_ts: int
    receiving_ts: int


class EnvelopeError(ProtocolError):
    """Undecodable envelope (code 116), carrying what was decoded before the failure (design §3.4.3)."""

    def __init__(self, source: str, *, code_value: int | None = None, notifier: bytes | None = None) -> None:
        super().__init__(Code.UNFLATTEN_FAILED, source)
        self.code_value = code_value      # command code, when it was decoded (may exceed the item list)
        self.notifier = notifier          # notifier name, when it was decoded


def decode_envelope(payload: bytes, enums: WireEnums) -> CommandEnvelope:
    """Parse a ``Command`` payload (02 §7.4): code, notifier, variant header walk, data up to ``len - 16``,
    attribute count 0, then 3 × U32. Every failure raises :class:`EnvelopeError` with what was decoded."""
    r = Reader(payload)
    try:
        code = r.uint(enums.size("camera_command"))
    except ProtocolError as exc:
        raise EnvelopeError(f"command code: {exc.source}") from None
    try:
        notifier = r.string_bytes()
    except ProtocolError as exc:
        raise EnvelopeError(f"notifier name: {exc.source}", code_value=code) from None
    try:
        header = read_variant_header(r)
    except ProtocolError as exc:
        raise EnvelopeError(f"variant header: {exc.source}", code_value=code, notifier=notifier) from None
    if len(payload) - r.pos < ENVELOPE_TRAILER_SIZE:
        raise EnvelopeError(f"{len(payload) - r.pos} bytes after the variant header, at least "
                            f"{ENVELOPE_TRAILER_SIZE} expected", code_value=code, notifier=notifier)
    end = len(payload) - ENVELOPE_TRAILER_SIZE
    attributes, request_ts, sending_ts, receiving_ts = struct.unpack(">IIII", payload[end:])
    if attributes != 0:
        raise EnvelopeError(f"variant attribute count {attributes} (0 expected)", code_value=code, notifier=notifier)
    return CommandEnvelope(code=code, notifier=notifier, variant=header, data=payload[r.pos:end],
                           request_ts=request_ts, sending_ts=sending_ts, receiving_ts=receiving_ts)


def encode_envelope(code: int, notifier: bytes | str, variant: bytes, enums: WireEnums,
                    ts: tuple[int, int, int] = (0, 0, 0)) -> bytes:
    """Flatten ``Camera Queue Data`` (client / test side); ``variant`` is already flattened (header + data + U32 0)."""
    w = Writer().uint(code, enums.size("camera_command")).string(notifier).raw(variant)
    for t in ts:
        w.u32(t)
    return w.bytes()


# ---------------------------------------------------------------------------------------------- outbound replies
@dataclass(frozen=True)
class ResponseTimestamps:
    """``HW TCP Response Data`` fields 2–7 in the order of 02 §8.1 (inferred, A9)."""

    request: int = 0
    sending: int = 0
    receiving: int = 0
    pre_flatten: int = 0
    post_flatten: int = 0
    tcp_sending: int = 0

    def as_tuple(self) -> tuple[int, int, int, int, int, int]:
        """The six values in wire order."""
        return (self.request, self.sending, self.receiving, self.pre_flatten, self.post_flatten, self.tcp_sending)


def flatten_reply(kind: ReplyKind, value: Any, err: ErrorCluster, *, encoding: str = "cp1252",
                  errors: str = "replace") -> bytes:
    """``Response (Flattened)`` of one reply kind (02 §8.3): ``{value, error}``, or the error alone for ACK.

    ACK ignores ``value``; IMAGE ``value`` = flattened image bytes (``b""`` on error); a ``value`` of None writes
    the kind's default. Text (STR value, error source) is encoded with ``errors`` (``"replace"`` by default: a
    source built from an OS message must never make the reply fail). NONE → :class:`ValueError`.
    """
    w = Writer(encoding=encoding, errors=errors)
    if kind is ReplyKind.NONE:
        raise ValueError("reply kind NONE has no flattened form")
    if kind is ReplyKind.STR:
        w.string("" if value is None else value)
    elif kind is ReplyKind.BOOL:
        w.bool(bool(value))
    elif kind is ReplyKind.DBL:
        w.dbl(0.0 if value is None else value)
    elif kind is ReplyKind.U32:
        w.u32(0 if value is None else value)
    elif kind is ReplyKind.DBL2D:
        w.dbl_2d([] if value is None else value)
    elif kind is ReplyKind.IMAGE:
        w.string(b"" if value is None else bytes(value))
    return w.error(err).bytes()


def encode_response_data(notifier: bytes, flattened: bytes, ts: ResponseTimestamps) -> bytes:
    """``HW TCP Response Data`` (02 §8.1): notifier, ``Response (Flattened)``, 6 × U32."""
    w = Writer().string(notifier).string(flattened)
    for t in ts.as_tuple():
        w.u32(t)
    return w.bytes()


def response_frame(datatypes: Datatypes, notifier: bytes, flattened: bytes, ts: ResponseTimestamps) -> bytes:
    """A whole ``Response`` frame (02 §5.4, §8.6)."""
    return encode_frame(datatypes.response, encode_response_data(notifier, flattened, ts))


@dataclass(frozen=True)
class ResponseData:
    """``HW TCP Response Data`` as decoded by a client."""

    notifier: bytes
    flattened: bytes
    ts: ResponseTimestamps


def decode_response_data(payload: bytes) -> ResponseData:
    """Client side: decode a ``Response`` payload; it must be consumed exactly (:class:`ProtocolError`)."""
    r = Reader(payload)
    notifier = r.string_bytes()
    flattened = r.string_bytes()
    ts = ResponseTimestamps(*(r.u32() for _ in range(6)))
    if r.remaining:
        raise ProtocolError(Code.UNFLATTEN_FAILED, f"response data: {r.remaining} unexpected trailing bytes")
    return ResponseData(notifier=notifier, flattened=flattened, ts=ts)


def parse_reply_value(kind: ReplyKind, flattened: bytes, *, encoding: str = "cp1252") -> tuple[Any, ErrorCluster]:
    """Client side inverse of :func:`flatten_reply`; must consume exactly. ACK value = None, IMAGE value = bytes."""
    r = Reader(flattened, encoding=encoding)
    value: Any
    if kind is ReplyKind.NONE:
        raise ValueError("reply kind NONE has no flattened form")
    if kind is ReplyKind.ACK:
        value = None
    elif kind is ReplyKind.STR:
        value = r.string()
    elif kind is ReplyKind.BOOL:
        value = r.bool()
    elif kind is ReplyKind.DBL:
        value = r.dbl()
    elif kind is ReplyKind.U32:
        value = r.u32()
    elif kind is ReplyKind.DBL2D:
        value = r.dbl_2d()
    else:
        value = r.string_bytes()
    err = r.error()
    if r.remaining:
        raise ProtocolError(Code.UNFLATTEN_FAILED, f"{kind.value} reply: {r.remaining} unexpected trailing bytes")
    return value, err
