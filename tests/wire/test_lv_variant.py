"""aloxcam.lv.variant: header walk and synthetic type descriptors (02 §7.3–§7.5, design §3.2.3)."""

from __future__ import annotations

import struct

import pytest

from aloxcam.errors import ProtocolError
from aloxcam.lv.flatten import Reader
from aloxcam.lv.variant import (
    EMPTY_VARIANT_HEADER,
    LV2018_VERSION,
    TypeCode,
    read_variant_header,
    synth_variant_header,
    write_variant,
)
from aloxcam.wire.enums import WireEnums
from aloxcam.testing import packaged_enums
from aloxcam.wire import messages
from golden import spec_vectors as sv

# The spec examples (02 §7.5, 04 §5.7) were written with a U16 Camera Type; the real wire uses U32 (capture),
# which tests/wire/test_capture_vectors.py covers. These tests check the codec mechanics against the spec bytes.
ENUMS = WireEnums.load(sizes={"camera_type": 2})


def _parse(header: bytes):  # type: ignore[no-untyped-def]
    r = Reader(header + b"tail")
    vh = read_variant_header(r)
    assert r.remaining == 4 and vh.raw == header
    return vh


def test_bare_dbl_header_is_the_02_7_5_b_golden() -> None:
    header = synth_variant_header((("value_mm", "dbl"),), ENUMS, bare=True)
    assert header == sv.h(sv.BARE_DBL_VARIANT_HEADER)
    vh = _parse(header)
    assert vh.version == LV2018_VERSION == 0x18008000
    assert vh.type_descriptors == (b"\x00\x04\x00\x0a",) and vh.top_indices == (0,)
    assert vh.top_type_code == TypeCode.DBL


def test_empty_variant_header() -> None:
    assert EMPTY_VARIANT_HEADER == sv.h("18 00 80 00 00 00 00 01 00 04 00 00 00 01 00 00")
    assert synth_variant_header(None, ENUMS) == EMPTY_VARIANT_HEADER
    assert _parse(EMPTY_VARIANT_HEADER).top_type_code == TypeCode.VOID
    assert write_variant(EMPTY_VARIANT_HEADER, b"") == EMPTY_VARIANT_HEADER + bytes(4)
    assert write_variant(b"H", b"D") == b"HD\x00\x00\x00\x00"


def test_synthetic_cluster_header_parses() -> None:
    vh = _parse(synth_variant_header(messages.CameraSetFlatFieldData.LV_LAYOUT, ENUMS))
    codes = [td[3] for td in vh.type_descriptors]
    assert codes == [TypeCode.ENUM_U16, TypeCode.BOOL, TypeCode.PATH, TypeCode.PATH, TypeCode.DBL, TypeCode.CLUSTER]
    assert vh.top_indices == (5,) and vh.top_type_code == TypeCode.CLUSTER
    cluster = vh.type_descriptors[5]
    assert struct.unpack(">H5H", cluster[4:]) == (5, 0, 1, 2, 3, 4)
    for td in vh.type_descriptors:
        assert struct.unpack(">H", td[:2])[0] == len(td) and len(td) % 2 == 0


def test_synthetic_enum_td_lists_items() -> None:
    vh = _parse(synth_variant_header((("method", "enum:z_reference_method"),), ENUMS, bare=True))
    (td,) = vh.type_descriptors
    assert td[3] == TypeCode.ENUM_U16
    assert td[4:] == b"\x00\x02\x03FNL\x03FRF"            # U16 count + Pascal strings (already even)
    odd = _parse(synth_variant_header((("v", "enum:camera_type"),), ENUMS, bare=True)).type_descriptors[0]
    assert odd[4:] == b"\x00\x02\x09High Mag.\x08Low Mag.\x00"   # padded to an even size
    from aloxcam.wire.enums import WireEnums
    u8 = WireEnums.load(sizes={"camera_type": 1})
    assert _parse(synth_variant_header((("v", "enum:camera_type"),), u8, bare=True)).top_type_code == TypeCode.ENUM_U8


def test_synthetic_array_headers_parse() -> None:
    vh = _parse(synth_variant_header(messages.CameraAutofocusData.LV_LAYOUT, ENUMS))
    array = next(td for td in vh.type_descriptors if td[3] == TypeCode.ARRAY)
    ndims, dim, element = struct.unpack(">HiH", array[4:])
    assert (ndims, dim) == (1, -1) and vh.type_descriptors[element][3] == TypeCode.I32
    vh2 = _parse(synth_variant_header((("a", "dbl[][]"), ("e", "err"), ("s", "str")), ENUMS))
    arr2 = next(td for td in vh2.type_descriptors if td[3] == TypeCode.ARRAY)
    assert struct.unpack(">HiiH", arr2[4:])[:3] == (2, -1, -1)
    assert vh2.top_type_code == TypeCode.CLUSTER


def test_every_payload_class_has_a_parseable_header() -> None:
    for cls in messages.PAYLOAD_CLASSES:
        vh = _parse(synth_variant_header(cls.LV_LAYOUT, ENUMS, bare=cls.LV_BARE))
        assert vh.top_type_code is not None


@pytest.mark.parametrize("size", [0, 2, 3, 0x8004, 0xFFFF])
def test_bad_td_sizes_rejected(size: int) -> None:
    header = struct.pack(">IIH", LV2018_VERSION, 1, size) + b"\x00\x0a" + b"\x00" * 8 + b"\x00\x01\x00\x00"
    with pytest.raises(ProtocolError):
        read_variant_header(Reader(header))


def test_bad_top_index_rejected() -> None:
    header = struct.pack(">II", LV2018_VERSION, 1) + b"\x00\x04\x00\x0a" + struct.pack(">HH", 1, 1)
    with pytest.raises(ProtocolError):
        read_variant_header(Reader(header))


def test_truncated_or_absurd_headers_rejected() -> None:
    golden = sv.h(sv.BARE_DBL_VARIANT_HEADER)
    for n in range(len(golden)):
        with pytest.raises(ProtocolError):
            read_variant_header(Reader(golden[:n]))
    with pytest.raises(ProtocolError):
        read_variant_header(Reader(struct.pack(">II", LV2018_VERSION, 0xFFFFFFFF) + bytes(16)))


def test_zero_top_level_types_allowed() -> None:
    header = struct.pack(">II", LV2018_VERSION, 1) + b"\x00\x04\x00\x00" + b"\x00\x00"
    vh = read_variant_header(Reader(header))
    assert vh.top_indices == () and vh.top_type_code is None
