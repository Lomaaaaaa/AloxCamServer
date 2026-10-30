"""LabVIEW ``Flatten To String`` / ``Unflatten From String``: big-endian Reader / Writer and layout-driven clusters.

Spec: 02 §6 (flattening rules), 02 §11.1 (``lvwire`` reference codec), 04 §3 (field types), 04 §9.2 (``lvtypes``
layout-driven (un)flatten).

A *layout* is ``((attribute, kind), …)`` in **cluster** order; kinds are listed in :data:`KINDS` plus
``enum:<enum_id>`` (an unsigned integer of the enum's size, carried as its item label above this module).
"""

from __future__ import annotations

import dataclasses
import operator
import re
import struct
from collections.abc import Callable, Iterable
from typing import Any, Protocol, TypeVar

import numpy as np

from aloxcam.errors import Code, ConfigError, ErrorCluster, ProtocolError
from aloxcam.lv.path import LVPath, read_path, write_path

__all__ = [
    "DEFAULT_ENCODING",
    "ENUM_PREFIX",
    "KINDS",
    "EnumCodec",
    "Kind",
    "Layout",
    "Reader",
    "Writer",
    "enum_id_of",
    "flatten",
    "parse_layout",
    "read_value",
    "unflatten",
    "validate_layout",
    "write_value",
]

DEFAULT_ENCODING = "cp1252"
Kind = str
Layout = tuple[tuple[str, Kind], ...]     # ((attribute, kind), …) in CLUSTER order

ENUM_PREFIX = "enum:"
#: Every kind except ``enum:<enum_id>`` (design §3.2.1 kinds table).
KINDS = ("bool", "u8", "i8", "u16", "i16", "u32", "i32", "u64", "i64", "sgl", "dbl", "str", "path", "i32[]",
         "dbl[][]", "err")

_INT_FORMATS = {"u8": ">B", "i8": ">b", "u16": ">H", "i16": ">h", "u32": ">I", "i32": ">i", "u64": ">Q", "i64": ">q"}
_INT_RANGES = {k: ((0, 2 ** (8 * struct.calcsize(f)) - 1) if k[0] == "u"
                   else (-(2 ** (8 * struct.calcsize(f) - 1)), 2 ** (8 * struct.calcsize(f) - 1) - 1))
               for k, f in _INT_FORMATS.items()}
_UINT_BY_SIZE = {1: "u8", 2: "u16", 4: "u32"}
_IDENTIFIER = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")

T = TypeVar("T")


class EnumCodec(Protocol):
    """Enum code ↔ label translation (implemented by ``wire.enums.WireEnums``)."""

    def size(self, enum_id: str) -> int:
        """Representation in bytes: 1, 2 or 4."""
        ...

    def label(self, enum_id: str, code: int) -> str:
        """Item label; :class:`ProtocolError` if out of range (``"#<code>"`` for lenient enums)."""
        ...

    def code(self, enum_id: str, label: str) -> int:
        """Item code; :class:`ValueError` for unknown labels; ``"#<n>"`` → n."""
        ...


def _short(n: int, pos: int, have: int) -> ProtocolError:
    return ProtocolError(Code.UNFLATTEN_FAILED, f"need {n} bytes at offset {pos}, have {have}")


class Reader:
    """Big-endian reader over flattened bytes; every short read raises ``ProtocolError(116, …)`` (02 §6)."""

    def __init__(self, data: bytes, pos: int = 0, *, encoding: str = DEFAULT_ENCODING) -> None:
        self.data = bytes(data)
        self.pos = pos
        self.encoding = encoding

    @property
    def remaining(self) -> int:
        """Bytes left after :attr:`pos`."""
        return len(self.data) - self.pos

    def take(self, n: int) -> bytes:
        """The next ``n`` bytes."""
        if n < 0 or self.pos + n > len(self.data):
            raise _short(n, self.pos, self.remaining)
        b = self.data[self.pos:self.pos + n]
        self.pos += n
        return b

    def _int(self, kind: str) -> int:
        fmt = _INT_FORMATS[kind]
        value: int = struct.unpack(fmt, self.take(struct.calcsize(fmt)))[0]
        return value

    def bool(self) -> bool:
        """LabVIEW Boolean: any non-zero byte is TRUE (02 §6)."""
        return self.take(1) != b"\x00"

    def u8(self) -> int:
        return self._int("u8")

    def i8(self) -> int:
        return self._int("i8")

    def u16(self) -> int:
        return self._int("u16")

    def i16(self) -> int:
        return self._int("i16")

    def u32(self) -> int:
        return self._int("u32")

    def i32(self) -> int:
        return self._int("i32")

    def u64(self) -> int:
        return self._int("u64")

    def i64(self) -> int:
        return self._int("i64")

    def sgl(self) -> float:
        value: float = struct.unpack(">f", self.take(4))[0]
        return value

    def dbl(self) -> float:
        value: float = struct.unpack(">d", self.take(8))[0]
        return value

    def uint(self, size: int) -> int:
        """Unsigned integer of ``size`` bytes (1 | 2 | 4): an enum value."""
        try:
            return self._int(_UINT_BY_SIZE[size])
        except KeyError:
            raise ValueError(f"unsupported integer size {size}") from None

    def string_bytes(self) -> bytes:
        """I32 length + raw bytes (opaque: notifier names, 02 §10 rule 1)."""
        pos = self.pos
        n = self.i32()
        if n < 0:
            raise ProtocolError(Code.UNFLATTEN_FAILED, f"negative string length {n} at offset {pos}")
        return self.take(n)

    def string(self) -> str:
        """I32 length + text decoded with :attr:`encoding` (``errors="replace"``)."""
        return self.string_bytes().decode(self.encoding, "replace")

    def error(self) -> ErrorCluster:
        """Error cluster: U8 status · I32 code · I32 length + source (04 §4.3)."""
        return ErrorCluster(self.bool(), self.i32(), self.string())

    def i32_array(self) -> list[int]:
        """1-D I32 array: I32 count + count × I32."""
        pos = self.pos
        n = self.i32()
        if n < 0:
            raise ProtocolError(Code.UNFLATTEN_FAILED, f"negative array size {n} at offset {pos}")
        return list(struct.unpack(f">{n}i", self.take(4 * n)))

    def dbl_2d(self) -> np.ndarray:
        """2-D DBL array: I32 rows, I32 cols, row-major DBL; shape kept (0×0 vs 0×2, 02 §8.3)."""
        pos = self.pos
        rows, cols = self.i32(), self.i32()
        if rows < 0 or cols < 0:
            raise ProtocolError(Code.UNFLATTEN_FAILED, f"negative 2-D array size {rows}×{cols} at offset {pos}")
        raw = self.take(8 * rows * cols)
        return np.frombuffer(raw, dtype=">f8").astype(np.float64).reshape(rows, cols)

    def path(self) -> LVPath:
        """``PTH0`` path (04 §3.2)."""
        return read_path(self)


class Writer:
    """Big-endian writer; every method returns ``self`` (02 §6).

    Integer writers raise :class:`ValueError` when out of range. :meth:`string` accepts ``str`` (encoded with
    ``errors``: ``"strict"`` → :class:`ValueError` on unencodable text; the server's outbound path uses
    ``"replace"``) or bytes (written as is).
    """

    def __init__(self, *, encoding: str = DEFAULT_ENCODING, errors: str = "strict") -> None:
        self.encoding = encoding
        self.errors = errors
        self._buf = bytearray()

    def raw(self, b: bytes) -> Writer:
        """Append bytes as they are."""
        self._buf += b
        return self

    def _int(self, kind: str, value: int) -> Writer:
        v = operator.index(value)
        lo, hi = _INT_RANGES[kind]
        if not lo <= v <= hi:
            raise ValueError(f"{v} out of range for {kind} [{lo}, {hi}]")
        return self.raw(struct.pack(_INT_FORMATS[kind], v))

    def bool(self, value: bool) -> Writer:
        return self.raw(b"\x01" if value else b"\x00")

    def u8(self, value: int) -> Writer:
        return self._int("u8", value)

    def i8(self, value: int) -> Writer:
        return self._int("i8", value)

    def u16(self, value: int) -> Writer:
        return self._int("u16", value)

    def i16(self, value: int) -> Writer:
        return self._int("i16", value)

    def u32(self, value: int) -> Writer:
        return self._int("u32", value)

    def i32(self, value: int) -> Writer:
        return self._int("i32", value)

    def u64(self, value: int) -> Writer:
        return self._int("u64", value)

    def i64(self, value: int) -> Writer:
        return self._int("i64", value)

    def sgl(self, value: float) -> Writer:
        return self.raw(struct.pack(">f", float(value)))

    def dbl(self, value: float) -> Writer:
        return self.raw(struct.pack(">d", float(value)))

    def uint(self, value: int, size: int) -> Writer:
        """Unsigned integer of ``size`` bytes (1 | 2 | 4)."""
        try:
            kind = _UINT_BY_SIZE[size]
        except KeyError:
            raise ValueError(f"unsupported integer size {size}") from None
        return self._int(kind, value)

    def string(self, value: str | bytes) -> Writer:
        """I32 length + bytes."""
        b = value.encode(self.encoding, self.errors) if isinstance(value, str) else bytes(value)
        if len(b) > _INT_RANGES["i32"][1]:
            raise ValueError("string longer than 2**31 - 1 bytes")
        return self.i32(len(b)).raw(b)

    def error(self, err: ErrorCluster) -> Writer:
        """Error cluster: U8 status · I32 code · I32 length + source."""
        return self.bool(err.status).i32(err.code).string(err.source)

    def i32_array(self, values: Iterable[int]) -> Writer:
        """1-D I32 array: I32 count + count × I32."""
        items = list(values)
        self.i32(len(items))
        for v in items:
            self.i32(v)
        return self

    def dbl_2d(self, value: Any) -> Writer:
        """2-D DBL array (array-like, shape kept): I32 rows, I32 cols, row-major DBL. An empty 1-D input is 0×0."""
        arr = np.asarray(value, dtype=np.float64)
        if arr.ndim == 1 and arr.size == 0:
            arr = arr.reshape(0, 0)
        if arr.ndim != 2:
            raise ValueError(f"2-D array expected, got shape {arr.shape}")
        rows, cols = arr.shape
        return self.i32(rows).i32(cols).raw(np.ascontiguousarray(arr, dtype=">f8").tobytes())

    def path(self, p: LVPath) -> Writer:
        """``PTH0`` path (04 §3.2)."""
        return write_path(self, p)

    def bytes(self) -> bytes:
        """Everything written so far."""
        return bytes(self._buf)


# ---------------------------------------------------------------------------------------------- layouts
def enum_id_of(kind: Kind) -> str | None:
    """The enum id of an ``enum:<enum_id>`` kind, else None."""
    return kind[len(ENUM_PREFIX):] if kind.startswith(ENUM_PREFIX) else None


_READERS: dict[str, Callable[[Reader], Any]] = {
    "bool": Reader.bool, "u8": Reader.u8, "i8": Reader.i8, "u16": Reader.u16, "i16": Reader.i16,
    "u32": Reader.u32, "i32": Reader.i32, "u64": Reader.u64, "i64": Reader.i64, "sgl": Reader.sgl,
    "dbl": Reader.dbl, "str": Reader.string, "path": Reader.path, "i32[]": Reader.i32_array,
    "dbl[][]": Reader.dbl_2d, "err": Reader.error,
}
_WRITERS: dict[str, Callable[[Writer, Any], Writer]] = {
    "bool": Writer.bool, "u8": Writer.u8, "i8": Writer.i8, "u16": Writer.u16, "i16": Writer.i16,
    "u32": Writer.u32, "i32": Writer.i32, "u64": Writer.u64, "i64": Writer.i64, "sgl": Writer.sgl,
    "dbl": Writer.dbl, "str": Writer.string, "path": Writer.path, "i32[]": Writer.i32_array,
    "dbl[][]": Writer.dbl_2d, "err": Writer.error,
}


def read_value(r: Reader, kind: Kind, enums: EnumCodec) -> Any:
    """Read one value of ``kind``."""
    enum_id = enum_id_of(kind)
    if enum_id is not None:
        return enums.label(enum_id, r.uint(enums.size(enum_id)))
    try:
        reader = _READERS[kind]
    except KeyError:
        raise ValueError(f"unknown kind {kind!r}") from None
    return reader(r)


def write_value(w: Writer, kind: Kind, value: Any, enums: EnumCodec) -> Writer:
    """Write one value of ``kind``."""
    enum_id = enum_id_of(kind)
    if enum_id is not None:
        return w.uint(enums.code(enum_id, value), enums.size(enum_id))
    try:
        writer = _WRITERS[kind]
    except KeyError:
        raise ValueError(f"unknown kind {kind!r}") from None
    return writer(w, value)


def unflatten(cls: type[T], r: Reader, layout: Layout, enums: EnumCodec) -> T:
    """``Unflatten From String`` of one cluster: read the fields in ``layout`` (cluster) order and build ``cls``."""
    values = {attr: read_value(r, kind, enums) for attr, kind in layout}
    return cls(**values)


def flatten(obj: Any, layout: Layout, enums: EnumCodec, w: Writer | None = None) -> bytes:
    """``Flatten To String`` of one cluster (data only). Writes into ``w`` when given and returns its whole content."""
    w = w if w is not None else Writer()
    for attr, kind in layout:
        write_value(w, kind, getattr(obj, attr), enums)
    return w.bytes()


def _check_kind(kind: Kind, where: str) -> None:
    enum_id = enum_id_of(kind)
    if enum_id is not None:
        if not enum_id.strip() or enum_id != enum_id.strip():
            raise ConfigError(Code.LV_INVALID_INPUT, f"{where}: bad enum kind {kind!r}")
        return
    if kind not in KINDS:
        raise ConfigError(Code.LV_INVALID_INPUT, f"{where}: unknown kind {kind!r}")


def parse_layout(text: str) -> Layout:
    """Parse ``"attr:kind, attr:kind"`` (``[wire.layout]``); the kind may contain ``:`` for enums.

    Raises :class:`ConfigError` for empty text, a missing ``:``, a bad attribute name or an unknown kind.
    """
    if not text or not text.strip():
        raise ConfigError(Code.LV_INVALID_INPUT, "layout: empty")
    out: list[tuple[str, Kind]] = []
    for item in text.split(","):
        attr, sep, kind = item.strip().partition(":")
        attr, kind = attr.strip(), kind.strip()
        if not sep or not _IDENTIFIER.match(attr):
            raise ConfigError(Code.LV_INVALID_INPUT, f"layout: bad item {item.strip()!r} (expected attr:kind)")
        if kind.startswith(ENUM_PREFIX):
            kind = ENUM_PREFIX + kind[len(ENUM_PREFIX):].strip()
        _check_kind(kind, f"layout item {attr!r}")
        out.append((attr, kind))
    return tuple(out)


def validate_layout(cls: type, layout: Layout) -> None:
    """:class:`ConfigError` unless ``layout`` is a permutation of the dataclass fields of ``cls`` with valid kinds."""
    names = [attr for attr, _ in layout]
    fields = [f.name for f in dataclasses.fields(cls)]
    if len(set(names)) != len(names) or sorted(names) != sorted(fields):
        raise ConfigError(Code.LV_INVALID_INPUT,
                          f"layout of {cls.__name__}: fields {names} are not a permutation of {fields}")
    for attr, kind in layout:
        _check_kind(kind, f"layout of {cls.__name__}, field {attr!r}")
