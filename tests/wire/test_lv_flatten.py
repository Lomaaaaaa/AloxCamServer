"""aloxcam.lv.flatten: Reader / Writer kinds, layouts and truncation (02 §6, 04 §3, design §3.2.1)."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import pytest

from aloxcam.dispatch import ReplyKind
from aloxcam.errors import ConfigError, ErrorCluster, ProtocolError
from aloxcam.lv.flatten import KINDS, Reader, Writer, flatten, parse_layout, unflatten, validate_layout
from aloxcam.lv.path import PATH_UNC, LVPath
from aloxcam.testing import packaged_enums
from aloxcam.wire import messages
from aloxcam.wire.messages import COMMANDS, PayloadCodec
from aloxcam.wire.protocol import decode_envelope, decode_response_data, parse_header, parse_reply_value
from golden import spec_vectors as sv

ENUMS = packaged_enums()

SAMPLES: dict[str, list[object]] = {
    "bool": [False, True],
    "u8": [0, 255], "i8": [-128, 127], "u16": [0, 65535], "i16": [-32768, 32767],
    "u32": [0, 2**32 - 1], "i32": [-(2**31), 2**31 - 1], "u64": [0, 2**64 - 1], "i64": [-(2**63), 2**63 - 1],
    "sgl": [0.0, 1.5, -2.25], "dbl": [0.0, 1500.0, -1e-300, math.inf],
    "str": ["", "hello", "Kamera é"],
    "path": [LVPath(), LVPath(0, ("C", "f.png")), LVPath(2, ()), LVPath(PATH_UNC, ("srv", "s")), LVPath(1, ("", "x"))],
    "i32[]": [[], [1, -2, 3, 2**31 - 1]],
    "dbl[][]": [np.empty((0, 0)), np.empty((0, 2)), np.array([[1.0, 0.5], [2.0, 0.75]])],
    "err": [ErrorCluster(), ErrorCluster(True, -1074360316, "Cam Acquisition status"), ErrorCluster(False, 3, "w")],
}


@dataclass(frozen=True)
class One:
    v: object


def _equal(a: object, b: object) -> bool:
    if isinstance(a, np.ndarray):
        return isinstance(b, np.ndarray) and a.shape == b.shape and bool(np.array_equal(a, b))
    return a == b


@pytest.mark.parametrize("kind", KINDS)
def test_every_kind_round_trips(kind: str) -> None:
    for value in SAMPLES[kind]:
        data = flatten(One(value), (("v", kind),), ENUMS)
        r = Reader(data)
        got = unflatten(One, r, (("v", kind),), ENUMS)
        assert r.remaining == 0
        assert _equal(got.v, value), (kind, value, got.v)


def test_enum_kind_round_trips_with_size() -> None:
    data = flatten(One("Low Mag."), (("v", "enum:camera_type"),), ENUMS)
    assert data == b"\x00\x01"
    assert unflatten(One, Reader(data), (("v", "enum:camera_type"),), ENUMS).v == "Low Mag."
    assert flatten(One("#7"), (("v", "enum:camera_type"),), ENUMS) == b"\x00\x07"
    with pytest.raises(ProtocolError):
        unflatten(One, Reader(b"\x00\x07"), (("v", "enum:camera_type"),), ENUMS)
    with pytest.raises(ValueError):
        flatten(One("Mid Mag."), (("v", "enum:camera_type"),), ENUMS)


def test_big_endian_bytes() -> None:
    w = Writer().u16(0x0102).i32(-2).u32(0xBFF69004).dbl(1500.0).sgl(1.0).bool(True).string("hello")
    assert w.bytes().hex() == "0102" "fffffffe" "bff69004" "4097700000000000" "3f800000" "01" "0000000568656c6c6f"
    assert Writer().error(ErrorCluster(True, 9405, "Z Move Absolute")).bytes().hex() == \
        "01000024bd0000000f5a204d6f7665204162736f6c757465"


def test_notifier_bytes_are_opaque() -> None:
    raw = bytes([0x81, 0x8D, 0x8F, 0x90, 0x9D]) + b"py-1\xff"
    data = Writer().string(raw).bytes()
    assert data == b"\x00\x00\x00\x0a" + raw
    assert Reader(data).string_bytes() == raw
    assert "�" in Reader(data).string()     # decoded text replaces the undefined cp1252 bytes


@pytest.mark.parametrize("byte", [1, 2, 0x7F, 0x80, 0xFF])
def test_any_non_zero_bool_byte_is_true(byte: int) -> None:
    assert Reader(bytes([byte])).bool() is True
    assert Reader(b"\x00").bool() is False


def test_dbl_2d_keeps_empty_shapes() -> None:
    for shape in ((0, 0), (0, 2), (3, 0)):
        data = Writer().dbl_2d(np.empty(shape)).bytes()
        assert data == shape[0].to_bytes(4, "big") + shape[1].to_bytes(4, "big")
        got = Reader(data).dbl_2d()
        assert got.shape == shape and got.dtype == np.float64
    assert Writer().dbl_2d([]).bytes() == bytes(8)
    with pytest.raises(ValueError):
        Writer().dbl_2d([1.0, 2.0])


def test_reader_errors() -> None:
    with pytest.raises(ProtocolError) as info:
        Reader(b"\x00\x01").take(3)
    assert info.value.code == 116 and "need 3 bytes at offset 0, have 2" in info.value.source
    with pytest.raises(ProtocolError):
        Reader(b"\xff\xff\xff\xff").string_bytes()        # negative length
    with pytest.raises(ProtocolError):
        Reader(b"\xff\xff\xff\xfe").i32_array()
    with pytest.raises(ProtocolError):
        Reader(b"\x00\x00\x00\x01\xff\xff\xff\xff").dbl_2d()
    with pytest.raises(ProtocolError):
        Reader(b"\x7f\xff\xff\xff\x7f\xff\xff\xff").dbl_2d()   # absurd size fails before allocating
    with pytest.raises(ValueError):
        Reader(b"\x00\x00\x00").uint(3)
    r = Reader(b"\x01\x02\x03", 1)
    assert r.remaining == 2 and r.uint(1) == 2 and r.pos == 2


@pytest.mark.parametrize(
    ("method", "value"),
    [("u8", 256), ("u8", -1), ("i8", 128), ("u16", 65536), ("i16", -32769), ("u32", 2**32), ("u32", -1),
     ("i32", 2**31), ("u64", -1), ("i64", 2**63)],
)
def test_out_of_range_integer_writes_raise(method: str, value: int) -> None:
    with pytest.raises(ValueError):
        getattr(Writer(), method)(value)


def test_uint_sizes_and_string_encoding() -> None:
    assert Writer().uint(5, 1).uint(5, 2).uint(5, 4).bytes() == b"\x05\x00\x05\x00\x00\x00\x05"
    with pytest.raises(ValueError):
        Writer().uint(256, 1)
    with pytest.raises(ValueError):
        Writer().uint(1, 8)
    with pytest.raises(ValueError):
        Writer().string("Δ")                                 # strict by default
    assert Writer(errors="replace").string("Δ").bytes() == b"\x00\x00\x00\x01?"


def test_parse_and_validate_layout() -> None:
    layout = parse_layout(" camera_type : enum:camera_type, camera_name:str,number_of_buffers:i32 ")
    assert layout == (("camera_type", "enum:camera_type"), ("camera_name", "str"), ("number_of_buffers", "i32"))
    for bad in ("", "a", "a:nope", "1a:i32", "a:enum:", "a:i32,,b:i32"):
        with pytest.raises(ConfigError):
            parse_layout(bad)
    cls = messages.CameraInitData
    validate_layout(cls, cls.LAYOUT_UNBUNDLE)
    with pytest.raises(ConfigError):
        validate_layout(cls, cls.LAYOUT_PANEL[:3])
    with pytest.raises(ConfigError):
        validate_layout(cls, cls.LAYOUT_PANEL + (("camera_name", "str"),))
    with pytest.raises(ConfigError):
        validate_layout(cls, (("camera_type", "u128"),) + cls.LAYOUT_PANEL[1:])


# ---------------------------------------------------------------------------------------------- truncations
def _truncations(data: bytes) -> list[bytes]:
    return [data[:n] for n in range(len(data))]


@pytest.mark.parametrize("vector", sv.PAYLOAD_VECTORS, ids=lambda v: v.name)
def test_every_truncation_of_every_payload_vector_raises(vector: sv.PayloadVector) -> None:
    cls = getattr(messages, vector.payload_class)
    codec = PayloadCodec(ENUMS, init_data_fallback=True)
    if vector.layout:
        codec = PayloadCodec(ENUMS, {cls.LV_TYPEDEF: ", ".join(f"{a}:{k}" for a, k in getattr(cls, vector.layout))})
    spec = next(s for s in COMMANDS.values() if s.payload is cls)
    data = sv.fill(vector.hex, **vector.code_values(ENUMS))
    codec.decode(spec, data)
    for truncated in _truncations(data):
        with pytest.raises(ProtocolError):
            codec.decode(spec, truncated)


@pytest.mark.parametrize("vector", sv.PATH_VECTORS, ids=lambda v: v.name)
def test_every_truncation_of_every_path_vector_raises(vector: sv.PathVector) -> None:
    assert vector.hex is not None
    for truncated in _truncations(sv.h(vector.hex)):
        with pytest.raises(ProtocolError):
            Reader(truncated).path()


def test_every_truncation_of_the_command_frame_raises() -> None:
    v = sv.Z_SET_MIN_LIMIT_FRAME
    frame = sv.fill(v.template, tt=ENUMS.code("tcp_datatype", v.datatype), cc=ENUMS.code("camera_command", v.command))
    payload = frame[6:]
    codec = PayloadCodec(ENUMS)
    spec = COMMANDS[v.command]
    for truncated in _truncations(payload):
        with pytest.raises(ProtocolError):
            codec.decode(spec, decode_envelope(truncated, ENUMS).data)
    for truncated in _truncations(frame[:6]):
        with pytest.raises(ProtocolError):
            parse_header(truncated, 1 << 20)


def test_every_truncation_of_every_reply_vector_raises() -> None:
    reply = sv.fill(sv.GET_HEARTBEAT_REPLY.template, tt=ENUMS.code("tcp_datatype", "Response"))
    for truncated in _truncations(reply[6:]):
        with pytest.raises(ProtocolError):
            decode_response_data(truncated)
    for row in sv.RESPONSE_FLATTENED:
        for truncated in _truncations(sv.h(row.hex)):
            with pytest.raises(ProtocolError):
                parse_reply_value(row.kind, truncated)
    assert ReplyKind.ACK in {row.kind for row in sv.RESPONSE_FLATTENED}
