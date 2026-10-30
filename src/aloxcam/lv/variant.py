"""Flattened LabVIEW variants: header parsing and synthetic type descriptors (layout inferred).

Spec: 02 §7.3 (flattened variant format, inferred), §7.4 (parsing and emitting ``Command Data``), §7.5 (b) (golden
bare-DBL header), §11.1 (``read_variant_header`` / ``write_variant`` reference sketch).

Layout: ``U32 version · U32 TD count · TDs · U16 top-level count + U16 indices · data · U32 attribute count``. Each
type descriptor (TD) is ``U16 size (whole TD) · U8 flags · U8 type code · content``. Synthetic headers are only for
our test client and simulator: they are well formed for :func:`read_variant_header` but are **not** claimed to equal
LabVIEW's bytes, except the bare-DBL header shown in 02 §7.5 (b). A captured template is preferred (02 §7.4).
"""

from __future__ import annotations

import struct
from collections.abc import Sequence
from dataclasses import dataclass
from enum import IntEnum

from aloxcam.errors import Code, ProtocolError
from aloxcam.lv.flatten import EnumCodec, Layout, Reader, enum_id_of

__all__ = [
    "EMPTY_VARIANT_HEADER",
    "LV2018_VERSION",
    "TypeCode",
    "VariantHeader",
    "read_variant_header",
    "synth_variant_header",
    "write_variant",
]

LV2018_VERSION = 0x18008000


class TypeCode(IntEnum):
    """Type codes of LabVIEW type descriptors (02 §7.3, inferred)."""

    VOID = 0x00
    I8 = 0x01
    I16 = 0x02
    I32 = 0x03
    I64 = 0x04
    U8 = 0x05
    U16 = 0x06
    U32 = 0x07
    U64 = 0x08
    SGL = 0x09
    DBL = 0x0A
    ENUM_U8 = 0x15
    ENUM_U16 = 0x16
    ENUM_U32 = 0x17
    BOOL = 0x21
    STRING = 0x30
    PATH = 0x32
    ARRAY = 0x40
    CLUSTER = 0x50
    VARIANT = 0x53
    TYPEDEF = 0xF1


@dataclass(frozen=True)
class VariantHeader:
    """The part of a flattened variant before its data (02 §7.3)."""

    version: int
    type_descriptors: tuple[bytes, ...]   # each TD including its size word
    top_indices: tuple[int, ...]
    raw: bytes                            # version … top-level list, exactly as received

    @property
    def top_type_code(self) -> int | None:
        """Type code of the first top-level TD, or None when there is no top-level type."""
        if not self.top_indices:
            return None
        return self.type_descriptors[self.top_indices[0]][3]


def read_variant_header(r: Reader) -> VariantHeader:
    """Walk version, TD list (by size words) and top-level list (02 §7.4, §11.1).

    A TD size below 4 or with bit 15 set (32-bit size form) raises :class:`ProtocolError`, as does a top-level
    index that is not below the TD count.
    """
    start = r.pos
    version = r.u32()
    n_td = r.u32()
    if 4 * n_td > r.remaining:
        raise ProtocolError(Code.UNFLATTEN_FAILED, f"variant: {n_td} type descriptors cannot fit in "
                                                   f"{r.remaining} bytes (offset {r.pos})")
    tds: list[bytes] = []
    for _ in range(n_td):
        pos = r.pos
        size_word = r.take(2)
        size = struct.unpack(">H", size_word)[0]
        if size < 4 or size & 0x8000:
            raise ProtocolError(Code.UNFLATTEN_FAILED, f"variant: unsupported type descriptor size {size:#06x} "
                                                       f"at offset {pos}")
        tds.append(size_word + r.take(size - 2))
    count = r.u16()
    indices = tuple(r.u16() for _ in range(count))
    if any(i >= n_td for i in indices):
        raise ProtocolError(Code.UNFLATTEN_FAILED, f"variant: top-level index out of range {indices} "
                                                   f"({n_td} type descriptors)")
    return VariantHeader(version, tuple(tds), indices, r.data[start:r.pos])


def write_variant(header: bytes, data: bytes) -> bytes:
    """``header ‖ data ‖ U32 0`` (no attributes; 02 §7.4)."""
    return header + data + b"\x00\x00\x00\x00"


# ---------------------------------------------------------------------------------------------- synthetic headers
_SCALAR_CODES = {
    "bool": TypeCode.BOOL, "i8": TypeCode.I8, "i16": TypeCode.I16, "i32": TypeCode.I32, "i64": TypeCode.I64,
    "u8": TypeCode.U8, "u16": TypeCode.U16, "u32": TypeCode.U32, "u64": TypeCode.U64, "sgl": TypeCode.SGL,
    "dbl": TypeCode.DBL,
}
_ENUM_CODES = {1: TypeCode.ENUM_U8, 2: TypeCode.ENUM_U16, 4: TypeCode.ENUM_U32}


def _scalar(code: int) -> bytes:
    return struct.pack(">HBB", 4, 0, code)


def _string_like(code: int) -> bytes:
    return struct.pack(">HBBi", 8, 0, code, -1)


def _array(element_index: int, ndims: int) -> bytes:
    body = struct.pack(">H", ndims) + struct.pack(">i", -1) * ndims + struct.pack(">H", element_index)
    return struct.pack(">HBB", 4 + len(body), 0, TypeCode.ARRAY) + body


def _cluster(indices: Sequence[int]) -> bytes:
    body = struct.pack(f">H{len(indices)}H", len(indices), *indices)
    return struct.pack(">HBB", 4 + len(body), 0, TypeCode.CLUSTER) + body


def _enum_items(enums: EnumCodec, enum_id: str) -> tuple[str, ...]:
    table = getattr(enums, "table", None)
    if callable(table):
        return tuple(table(enum_id).items)
    items: list[str] = []
    for code in range(2 ** (8 * enums.size(enum_id))):     # probe codes until the table ends
        try:
            label = enums.label(enum_id, code)
        except (ProtocolError, ValueError, KeyError):
            break
        if label.startswith("#"):
            break
        items.append(label)
    return tuple(items)


def _enum(enums: EnumCodec, enum_id: str) -> bytes:
    items = _enum_items(enums, enum_id)
    body = bytearray(struct.pack(">H", len(items)))
    for item in items:
        b = item.encode("cp1252")
        if len(b) > 255:
            raise ValueError(f"enum item longer than 255 bytes: {item[:40]!r}…")
        body.append(len(b))
        body += b
    if (4 + len(body)) % 2:
        body.append(0)
    return struct.pack(">HBB", 4 + len(body), 0, _ENUM_CODES[enums.size(enum_id)]) + bytes(body)


def _add_kind(tds: list[bytes], kind: str, enums: EnumCodec) -> int:
    """Append the TD(s) of one field (elements first, container last); return the field's TD index."""
    enum_id = enum_id_of(kind)
    if enum_id is not None:
        tds.append(_enum(enums, enum_id))
    elif kind in _SCALAR_CODES:
        tds.append(_scalar(_SCALAR_CODES[kind]))
    elif kind == "str":
        tds.append(_string_like(TypeCode.STRING))
    elif kind == "path":
        tds.append(_string_like(TypeCode.PATH))
    elif kind in ("i32[]", "dbl[][]"):
        element = _add_kind(tds, "i32" if kind == "i32[]" else "dbl", enums)
        tds.append(_array(element, 1 if kind == "i32[]" else 2))
    elif kind == "err":
        elements = [_add_kind(tds, k, enums) for k in ("bool", "i32", "str")]
        tds.append(_cluster(elements))
    else:
        raise ValueError(f"unknown kind {kind!r}")
    return len(tds) - 1


def synth_variant_header(layout: Layout | None, enums: EnumCodec, *, bare: bool = False,
                         version: int = LV2018_VERSION) -> bytes:
    """A synthetic variant header for ``layout`` (02 §7.3 inferred layout; design §3.2.3).

    ``None`` → void TD (an empty variant). Element TDs come first, the cluster TD last, and the top-level index is
    the cluster. ``bare=True`` (Show UI, Z Do Reference, Z Set Min/Max Limit) emits the single field TD as the
    top-level type. No labels, no typedef wrappers.
    """
    tds: list[bytes] = []
    if layout is None:
        tds.append(_scalar(TypeCode.VOID))
        top = 0
    elif bare:
        if len(layout) != 1:
            raise ValueError(f"a bare payload has exactly one field, got {len(layout)}")
        top = _add_kind(tds, layout[0][1], enums)
    else:
        elements = [_add_kind(tds, kind, enums) for _, kind in layout]
        tds.append(_cluster(elements))
        top = len(tds) - 1
    return struct.pack(">II", version, len(tds)) + b"".join(tds) + struct.pack(">HH", 1, top)


#: ``version · 1 TD (00 04 00 00, void) · top-level list [0]``: the header of an empty variant.
EMPTY_VARIANT_HEADER = struct.pack(">II", LV2018_VERSION, 1) + _scalar(TypeCode.VOID) + struct.pack(">HH", 1, 0)
