"""Minimal PNG reader / writer (zlib + numpy) for the simulation folder and the flat-field correction images.

Spec: 05 §8.2 (``IMAQ Read Image And Vision Info 2`` with ``Use File Path Image Type``: gray 8 → Mono8, gray 16 →
Mono16, RGB → luminance, anything else → -9011), 05 §7.3 (FFC / DFC files); design §3.6.8.

Reads non-interlaced gray 8/16, RGB 8/16 and RGBA 8/16 with all five filter types. Palette, gray+alpha, bit depths
below 8 and interlaced files raise :class:`PngUnsupported`; malformed files raise :class:`PngError`. Writes
single-channel 8/16-bit images with filter 0 (``None``).
"""

from __future__ import annotations

import struct
import zlib
from pathlib import Path

import numpy as np

__all__ = ["PNG_SIGNATURE", "PngError", "PngUnsupported", "read_png", "write_png"]

PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"

_CHANNELS = {0: 1, 2: 3, 4: 2, 6: 4, 3: 1}          # colour type → samples per pixel
_VALID_DEPTHS = {0: (1, 2, 4, 8, 16), 2: (8, 16), 3: (1, 2, 4, 8), 4: (8, 16), 6: (8, 16)}
_SUPPORTED = {0, 2, 6}                               # gray, RGB, RGBA


class PngError(Exception):
    """Malformed or truncated PNG file."""


class PngUnsupported(PngError):
    """A valid PNG in a format this reader does not decode (palette, gray+alpha, < 8 bits, interlaced)."""

    def __init__(self, message: str, *, color_type: int, bit_depth: int) -> None:
        super().__init__(message)
        self.color_type = color_type
        self.bit_depth = bit_depth


def _chunks(data: bytes) -> list[tuple[bytes, bytes]]:
    if not data.startswith(PNG_SIGNATURE):
        raise PngError("not a PNG file (bad signature)")
    pos = len(PNG_SIGNATURE)
    chunks: list[tuple[bytes, bytes]] = []
    while True:
        if pos + 8 > len(data):
            raise PngError("truncated chunk header")
        length, ctype = struct.unpack(">I4s", data[pos:pos + 8])
        end = pos + 8 + length
        if end + 4 > len(data):
            raise PngError(f"chunk {ctype!r} truncated")
        body = data[pos + 8:end]
        (crc,) = struct.unpack(">I", data[end:end + 4])
        if zlib.crc32(ctype + body) & 0xFFFFFFFF != crc:
            raise PngError(f"chunk {ctype!r}: CRC mismatch")
        chunks.append((ctype, body))
        pos = end + 4
        if ctype == b"IEND":
            return chunks


def _unfilter_avg(cur: bytearray, prior: bytes, bpp: int) -> None:
    for i in range(bpp):
        cur[i] = (cur[i] + (prior[i] >> 1)) & 0xFF
    for i in range(bpp, len(cur)):
        cur[i] = (cur[i] + ((cur[i - bpp] + prior[i]) >> 1)) & 0xFF


def _unfilter_paeth(cur: bytearray, prior: bytes, bpp: int) -> None:
    """Paeth predictor with ``|p − a| = |b − c|``, ``|p − b| = |a − c|``, ``|p − c| = |a + b − 2c|``."""
    for i in range(bpp):
        cur[i] = (cur[i] + prior[i]) & 0xFF            # a = c = 0 → predictor b
    for i in range(bpp, len(cur)):
        a = cur[i - bpp]
        b = prior[i]
        c = prior[i - bpp]
        pb = a - c
        pa = b - c
        pc = pa + pb
        if pa < 0:
            pa = -pa
        if pb < 0:
            pb = -pb
        if pc < 0:
            pc = -pc
        if pa <= pb and pa <= pc:
            cur[i] = (cur[i] + a) & 0xFF
        elif pb <= pc:
            cur[i] = (cur[i] + b) & 0xFF
        else:
            cur[i] = (cur[i] + c) & 0xFF


def _unfilter(raw: bytes, height: int, stride: int, bpp: int) -> np.ndarray:
    """Undo the per-scanline filters (PNG spec §9); returns ``(height, stride)`` uint8."""
    if len(raw) != height * (stride + 1):
        raise PngError(f"image data has {len(raw)} bytes, expected {height * (stride + 1)}")
    out = np.empty((height, stride), dtype=np.uint8)
    prior = np.zeros(stride, dtype=np.uint8)
    for y in range(height):
        start = y * (stride + 1)
        ftype = raw[start]
        line = np.frombuffer(raw, dtype=np.uint8, count=stride, offset=start + 1)
        if ftype == 0:
            row = line.copy()
        elif ftype == 1:
            row = np.empty(stride, dtype=np.uint8)
            for c in range(bpp):
                row[c::bpp] = np.cumsum(line[c::bpp], dtype=np.uint8)
        elif ftype == 2:
            row = line + prior                       # uint8 arithmetic wraps modulo 256
        elif ftype in (3, 4):                        # sequential along the row: per-byte loop
            cur = bytearray(line.tobytes())
            (_unfilter_avg if ftype == 3 else _unfilter_paeth)(cur, prior.tobytes(), bpp)
            row = np.frombuffer(bytes(cur), dtype=np.uint8)
        else:
            raise PngError(f"row {y}: unknown filter type {ftype}")
        out[y] = row
        prior = out[y]
    return out


def read_png(path: Path | str) -> np.ndarray:
    """Read a PNG file: gray 8/16 → ``(h, w)`` uint8/uint16; RGB/RGBA 8 → ``(h, w, 3|4)`` uint8; RGB/RGBA 16 →
    ``(h, w, 3|4)`` uint16. :class:`OSError` when the file cannot be read; :class:`PngError` /
    :class:`PngUnsupported` otherwise."""
    data = Path(path).read_bytes()
    chunks = _chunks(data)
    if not chunks or chunks[0][0] != b"IHDR" or len(chunks[0][1]) != 13:
        raise PngError("missing or malformed IHDR chunk")
    width, height, depth, ctype, compression, filter_method, interlace = struct.unpack(">IIBBBBB", chunks[0][1])
    if width == 0 or height == 0:
        raise PngError(f"invalid image size {width} x {height}")
    if ctype not in _VALID_DEPTHS or depth not in _VALID_DEPTHS[ctype]:
        raise PngError(f"invalid colour type {ctype} / bit depth {depth}")
    if compression != 0 or filter_method != 0 or interlace not in (0, 1):
        raise PngError("unknown compression, filter or interlace method")
    if ctype not in _SUPPORTED or depth < 8 or interlace:
        raise PngUnsupported(f"unsupported PNG format (colour type {ctype}, bit depth {depth}, interlace "
                             f"{interlace})", color_type=ctype, bit_depth=depth)
    idat = b"".join(body for name, body in chunks if name == b"IDAT")
    if not idat:
        raise PngError("no IDAT chunk")
    try:
        raw = zlib.decompress(idat)
    except zlib.error as exc:
        raise PngError(f"image data cannot be decompressed: {exc}") from None
    channels = _CHANNELS[ctype]
    sample_bytes = depth // 8
    bpp = channels * sample_bytes
    rows = _unfilter(raw, height, width * bpp, bpp)
    if sample_bytes == 2:
        pixels = rows.view(">u2").astype(np.uint16)
    else:
        pixels = rows
    shape = (height, width) if channels == 1 else (height, width, channels)
    return np.ascontiguousarray(pixels.reshape(shape))


def _chunk(ctype: bytes, body: bytes) -> bytes:
    return struct.pack(">I", len(body)) + ctype + body + struct.pack(">I", zlib.crc32(ctype + body) & 0xFFFFFFFF)


def write_png(path: Path | str, array: np.ndarray) -> None:
    """Write a 2-D uint8 / uint16 array as a gray 8 / 16-bit PNG (filter 0, one IDAT); :class:`ValueError` for
    other arrays."""
    a = np.asarray(array)
    if a.ndim != 2 or a.dtype not in (np.uint8, np.uint16) or a.shape[0] == 0 or a.shape[1] == 0:
        raise ValueError(f"write_png: a non-empty 2-D uint8 or uint16 array is required, got {a.dtype} {a.shape}")
    height, width = a.shape
    depth = 8 if a.dtype == np.uint8 else 16
    rows = a.astype(">u2").view(np.uint8).reshape(height, width * 2) if depth == 16 else a
    raw = np.hstack([np.zeros((height, 1), dtype=np.uint8), np.ascontiguousarray(rows)]).tobytes()
    ihdr = struct.pack(">IIBBBBB", width, height, depth, 0, 0, 0, 0)
    Path(path).write_bytes(PNG_SIGNATURE + _chunk(b"IHDR", ihdr) + _chunk(b"IDAT", zlib.compress(raw, 6))
                           + _chunk(b"IEND", b""))
