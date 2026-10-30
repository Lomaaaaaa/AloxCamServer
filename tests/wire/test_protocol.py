"""aloxcam.wire.protocol: frames, envelopes and replies against the 02 §5.4 / §7.5 / §8.6 vectors."""

from __future__ import annotations

import struct

import numpy as np
import pytest

from aloxcam.dispatch import ReplyKind
from aloxcam.errors import NO_ERROR, ErrorCluster, ProtocolError
from aloxcam.lv.variant import EMPTY_VARIANT_HEADER, write_variant
from aloxcam.testing import packaged_enums
from aloxcam.wire import messages
from aloxcam.wire.messages import COMMANDS, PayloadCodec
from aloxcam.wire.protocol import (
    FRAME_HEADER_SIZE,
    Datatypes,
    EnvelopeError,
    FrameHeader,
    ResponseTimestamps,
    decode_envelope,
    decode_response_data,
    encode_envelope,
    encode_frame,
    encode_response_data,
    flatten_reply,
    heartbeat_frame,
    parse_header,
    parse_reply_value,
    response_frame,
)
from golden import spec_vectors as sv

ENUMS = packaged_enums()
DT = Datatypes.from_enums(ENUMS)
MAX = 256 * 2**20


def test_datatypes_from_enums() -> None:
    assert (DT.command, DT.response, DT.image, DT.heartbeat) == (0, 1, 2, 3)
    assert DT.name_of(DT.response) == "Response" and DT.name_of(4) is None and DT.name_of(99) is None


def test_heartbeat_frame_02_5_4() -> None:
    assert heartbeat_frame(DT) == sv.fill(sv.HEARTBEAT_FRAME, tt=ENUMS.code("tcp_datatype", "Heartbeat"))
    assert heartbeat_frame(DT, b"x") == encode_frame(DT.heartbeat, b"x") == sv.h("00 00 00 03 00 03 78")


def test_decode_and_reencode_the_02_7_5_b_frame() -> None:
    v = sv.Z_SET_MIN_LIMIT_FRAME
    frame = sv.fill(v.template, tt=ENUMS.code("tcp_datatype", v.datatype), cc=ENUMS.code("camera_command", v.command))
    header = parse_header(frame[:FRAME_HEADER_SIZE], MAX)
    assert header == FrameHeader(payload_len=len(frame) - 6, datatype=DT.command) and header.payload_len == 50
    env = decode_envelope(frame[6:], ENUMS)
    assert ENUMS.label("camera_command", env.code) == v.command and env.code == 40
    assert env.notifier == v.notifier
    assert (env.request_ts, env.sending_ts, env.receiving_ts) == v.ts
    assert env.variant.version == v.variant_version and env.variant.top_type_code == v.top_type_code
    assert env.variant.raw == sv.h(sv.BARE_DBL_VARIANT_HEADER)
    assert env.data == sv.h(v.data_hex)
    codec = PayloadCodec(ENUMS)
    obj = codec.decode(COMMANDS[v.command], env.data)
    assert obj == getattr(messages, v.payload_class)(**v.payload_fields)
    variant = codec.variant_for(COMMANDS[v.command], obj)
    rebuilt = encode_frame(DT.command, encode_envelope(env.code, v.notifier, variant, ENUMS, ts=v.ts))
    assert rebuilt == frame
    assert encode_envelope(env.code, "Z#17", codec.variant_for(COMMANDS[v.command], obj), ENUMS, ts=v.ts) == frame[6:]


def test_get_heartbeat_reply_57_bytes() -> None:
    v = sv.GET_HEARTBEAT_REPLY
    expected = sv.fill(v.template, tt=ENUMS.code("tcp_datatype", v.datatype))
    assert len(expected) == v.size == 57
    frame = response_frame(DT, v.notifier, flatten_reply(v.kind, v.value, v.err), ResponseTimestamps(*v.ts))
    assert frame == expected
    rd = decode_response_data(frame[6:])
    assert rd.notifier == v.notifier and rd.ts.as_tuple() == v.ts
    assert parse_reply_value(v.kind, rd.flattened) == (123456, NO_ERROR)
    assert encode_response_data(v.notifier, rd.flattened, rd.ts) == frame[6:]


@pytest.mark.parametrize("row", sv.RESPONSE_FLATTENED, ids=lambda r: r.name)
def test_response_flattened_rows_02_8_6(row: sv.FlattenedReplyVector) -> None:
    assert len(sv.RESPONSE_FLATTENED) == 9
    data = sv.h(row.hex)
    assert flatten_reply(row.kind, row.value, row.err) == data
    value, err = parse_reply_value(row.kind, data)
    assert err == row.err
    if row.kind is ReplyKind.DBL2D:
        assert np.array_equal(value, np.asarray(row.value))
    elif row.kind is ReplyKind.ACK:
        assert value is None
    else:
        assert value == row.value


@pytest.mark.parametrize(
    ("kind", "value"),
    [(ReplyKind.ACK, None), (ReplyKind.STR, "Kamera é"), (ReplyKind.BOOL, True), (ReplyKind.DBL, -0.25),
     (ReplyKind.U32, 2**32 - 1), (ReplyKind.DBL2D, np.empty((0, 2))), (ReplyKind.DBL2D, np.empty((0, 0))),
     (ReplyKind.DBL2D, np.arange(6.0).reshape(3, 2)), (ReplyKind.IMAGE, b"ALXIMG01\x00\x81")],
)
def test_parse_reply_value_inverts_flatten_reply(kind: ReplyKind, value: object) -> None:
    for err in (NO_ERROR, ErrorCluster(True, 9405, "Z move out of range"), ErrorCluster(False, 3, "warn")):
        got, got_err = parse_reply_value(kind, flatten_reply(kind, value, err))
        assert got_err == err
        if isinstance(value, np.ndarray):
            assert got.shape == value.shape and np.array_equal(got, value)
        else:
            assert got == value


def test_flatten_reply_defaults_and_errors() -> None:
    assert flatten_reply(ReplyKind.ACK, "ignored", NO_ERROR) == sv.h("00 00000000 00000000")
    assert flatten_reply(ReplyKind.DBL2D, None, NO_ERROR)[:8] == bytes(8)
    assert flatten_reply(ReplyKind.IMAGE, None, NO_ERROR)[:4] == bytes(4)
    assert flatten_reply(ReplyKind.STR, None, NO_ERROR)[:4] == bytes(4)
    with pytest.raises(ValueError):
        flatten_reply(ReplyKind.NONE, None, NO_ERROR)
    with pytest.raises(ValueError):
        parse_reply_value(ReplyKind.NONE, b"")


def test_flatten_reply_replaces_unencodable_characters() -> None:
    data = flatten_reply(ReplyKind.ACK, None, ErrorCluster(True, 9501, "fichier Δ introuvable"))
    assert data.endswith(b"fichier ? introuvable")
    assert flatten_reply(ReplyKind.STR, "Δ", NO_ERROR)[:5] == b"\x00\x00\x00\x01?"
    with pytest.raises(ValueError):
        flatten_reply(ReplyKind.STR, "Δ", NO_ERROR, errors="strict")


def test_parse_reply_value_must_consume_exactly() -> None:
    with pytest.raises(ProtocolError):
        parse_reply_value(ReplyKind.ACK, sv.h("00 00000000 00000000 00"))
    with pytest.raises(ProtocolError):
        decode_response_data(encode_response_data(b"n", b"", ResponseTimestamps()) + b"\x00")


def test_parse_header() -> None:
    assert parse_header(sv.h("00 00 00 02 00 03"), MAX) == FrameHeader(0, 3)
    for bad in ("00 00 00 01 00 00", "ff ff ff ff 00 00", "00 00 00 00 00 00"):
        with pytest.raises(ProtocolError):
            parse_header(sv.h(bad), MAX)
    with pytest.raises(ProtocolError):
        parse_header(struct.pack(">iH", 2 + 1001, 0), 1000)
    assert parse_header(struct.pack(">iH", 2 + 1000, 0), 1000).payload_len == 1000
    with pytest.raises(ProtocolError):
        parse_header(b"\x00\x00\x00\x02\x00", MAX)
    with pytest.raises(ValueError):
        encode_frame(70000)


def _envelope(code: int = 40, notifier: bytes = b"Z#17", variant: bytes | None = None,
              ts: tuple[int, int, int] = (1, 2, 3)) -> bytes:
    if variant is None:
        variant = write_variant(sv.h(sv.BARE_DBL_VARIANT_HEADER), sv.h("3f e0 00 00 00 00 00 00"))
    return encode_envelope(code, notifier, variant, ENUMS, ts)


def test_code_beyond_the_item_list_decodes_as_raw_int() -> None:
    env = decode_envelope(_envelope(code=32767, variant=write_variant(EMPTY_VARIANT_HEADER, b"")), ENUMS)
    assert env.code == 32767 and env.data == b""
    assert decode_envelope(_envelope(code=65535), ENUMS).code == 65535


def test_envelope_error_carries_what_was_decoded() -> None:
    good = _envelope()
    header_end = 2 + 8 + 16
    cases = {
        "broken variant header": good[:10] + b"\x18\x00\x80\x00\x00\x00\x00\x01\x00\x02\x00\x0a" + good[22:],
        "attribute count != 0": good[:-16] + b"\x00\x00\x00\x01" + good[-12:],
        "payload < 16 bytes after the header": good[:header_end + 15],
        "header cut": good[:header_end - 1],
    }
    for name, payload in cases.items():
        with pytest.raises(EnvelopeError) as info:
            decode_envelope(payload, ENUMS)
        assert info.value.code == 116, name
        assert info.value.code_value == 40 and info.value.notifier == b"Z#17", name
    with pytest.raises(EnvelopeError) as info:
        decode_envelope(good[:2] + b"\x00\x00\x00\x09Z#", ENUMS)       # notifier truncated
    assert info.value.code_value == 40 and info.value.notifier is None
    with pytest.raises(EnvelopeError) as info:
        decode_envelope(b"\x00", ENUMS)                                 # code truncated
    assert info.value.code_value is None and info.value.notifier is None
    assert isinstance(info.value, ProtocolError)


def test_envelope_notifier_bytes_are_opaque_and_size_follows_the_table() -> None:
    notifier = b"py-\x81\x8d\x8f\x90\x9d"
    env = decode_envelope(_envelope(notifier=notifier), ENUMS)
    assert env.notifier == notifier
    from aloxcam.wire.enums import WireEnums
    u8 = WireEnums.load(sizes={"camera_command": 1})
    payload = encode_envelope(40, b"n", write_variant(EMPTY_VARIANT_HEADER, b""), u8)
    assert payload[0] == 40 and payload[1:5] == b"\x00\x00\x00\x01"
    assert decode_envelope(payload, u8).code == 40


def test_response_timestamps_order() -> None:
    ts = ResponseTimestamps(request=1, sending=2, receiving=3, pre_flatten=4, post_flatten=5, tcp_sending=6)
    data = encode_response_data(b"N1", b"", ts)
    assert struct.unpack(">6I", data[-24:]) == (1, 2, 3, 4, 5, 6)
    assert decode_response_data(data).ts == ts
