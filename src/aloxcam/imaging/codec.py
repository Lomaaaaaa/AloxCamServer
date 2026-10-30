"""Image codecs: the replacements of ``IMAQ Flatten Image to String`` used by IMAGE replies.

Spec: 02 §8.5 (image replies, NI-proprietary format), 05 §6.2 (flatten at reply time), 05 §7.5 (custom data);
design §3.6.9.

:class:`ProvisionalCodec` writes a documented, **non-NI** format so that the server is usable and testable until
Phase 3 (the real client cannot decode it). All big-endian::

    "ALXIMG01" · U16 NI image type · U16 0 · I32 width · I32 height · I32 n
    · n × (LabVIEW string key, LabVIEW string value) · I32 pixel byte count · pixels row-major

(LabVIEW string = I32 length + bytes, cp1252). :class:`NiVisionCodec` is the Phase-3 stub. This module uses
``struct`` only (no ``aloxcam.lv`` import).
"""

from __future__ import annotations

import ctypes
import ctypes.util
import logging
import struct
import sys

import numpy as np

from aloxcam.errors import Code, CodecError, ConfigError, ProtocolError
from aloxcam.imagedata import NI_IMAGE_I16, NI_IMAGE_RGB, NI_IMAGE_SGL, NI_IMAGE_U8, NI_IMAGE_U16, ImageCodec, ImageData
from aloxcam.settings import ImageOptions

__all__ = ["PROVISIONAL_MAGIC", "NiVisionCodec", "ProvisionalCodec", "decode_provisional", "make_codec"]

log = logging.getLogger(__name__)

PROVISIONAL_MAGIC = b"ALXIMG01"
_TEXT_ENCODING = "cp1252"

#: NI image type → (big-endian dtype, samples per pixel).
_PIXEL_FORMATS: dict[int, tuple[str, int]] = {
    NI_IMAGE_U8: (">u1", 1),
    NI_IMAGE_I16: (">i2", 1),
    NI_IMAGE_SGL: (">f4", 1),
    NI_IMAGE_RGB: (">u1", 4),
    NI_IMAGE_U16: (">u2", 1),
}


def _lv_string(text: str) -> bytes:
    b = text.encode(_TEXT_ENCODING, "replace")
    return struct.pack(">i", len(b)) + b


class ProvisionalCodec:
    """:class:`aloxcam.imagedata.ImageCodec` writing the provisional ``ALXIMG01`` format (design §3.6.9)."""

    name = "provisional"

    def flatten(self, image: ImageData) -> bytes:
        """Flatten ``image``; :class:`CodecError` (9508) for a pixel type without an NI image type."""
        try:
            ni_type = image.ni_image_type
        except ValueError as exc:
            raise CodecError(Code.IMAGE_CODEC_UNAVAILABLE, f"provisional image codec: {exc}") from None
        dtype, _ = _PIXEL_FORMATS[ni_type]
        pixels = np.ascontiguousarray(image.pixels).astype(dtype, copy=False).tobytes()
        parts = [PROVISIONAL_MAGIC, struct.pack(">HHiii", ni_type, 0, image.width, image.height,
                                                len(image.custom_data))]
        for key, value in image.custom_data.items():
            parts.append(_lv_string(str(key)))
            parts.append(_lv_string(str(value)))
        parts.append(struct.pack(">i", len(pixels)))
        parts.append(pixels)
        return b"".join(parts)


class _Cursor:
    def __init__(self, data: bytes) -> None:
        self.data = data
        self.pos = 0

    def take(self, n: int, what: str) -> bytes:
        if n < 0 or self.pos + n > len(self.data):
            raise ProtocolError(Code.UNFLATTEN_FAILED,
                                f"provisional image: {what} needs {n} bytes at offset {self.pos}, "
                                f"have {len(self.data) - self.pos}")
        out = self.data[self.pos:self.pos + n]
        self.pos += n
        return out

    def unpack(self, fmt: str, what: str) -> tuple[int, ...]:
        return struct.unpack(fmt, self.take(struct.calcsize(fmt), what))

    def string(self, what: str) -> str:
        (n,) = self.unpack(">i", what)
        if n < 0:
            raise ProtocolError(Code.UNFLATTEN_FAILED, f"provisional image: negative {what} length {n}")
        return self.take(n, what).decode(_TEXT_ENCODING, "replace")


def decode_provisional(data: bytes) -> ImageData:
    """Inverse of :meth:`ProvisionalCodec.flatten` (client side); :class:`ProtocolError` (116) on malformed data."""
    c = _Cursor(bytes(data))
    if c.take(len(PROVISIONAL_MAGIC), "magic") != PROVISIONAL_MAGIC:
        raise ProtocolError(Code.UNFLATTEN_FAILED, "provisional image: bad magic (not an ALXIMG01 image)")
    ni_type, _reserved, width, height, count = c.unpack(">HHiii", "header")
    if ni_type not in _PIXEL_FORMATS:
        raise ProtocolError(Code.UNFLATTEN_FAILED, f"provisional image: unknown NI image type {ni_type}")
    if width < 0 or height < 0 or count < 0:
        raise ProtocolError(Code.UNFLATTEN_FAILED,
                            f"provisional image: negative size or key count ({width}, {height}, {count})")
    custom: dict[str, str] = {}
    for _ in range(count):
        key = c.string("custom-data key")
        custom[key] = c.string("custom-data value")
    (nbytes,) = c.unpack(">i", "pixel byte count")
    dtype, samples = _PIXEL_FORMATS[ni_type]
    expected = width * height * samples * np.dtype(dtype).itemsize
    if nbytes != expected:
        raise ProtocolError(Code.UNFLATTEN_FAILED,
                            f"provisional image: {nbytes} pixel bytes for {width} x {height} (expected {expected})")
    raw = c.take(nbytes, "pixels")
    if c.pos != len(c.data):
        raise ProtocolError(Code.UNFLATTEN_FAILED,
                            f"provisional image: {len(c.data) - c.pos} unexpected bytes after the pixels")
    shape: tuple[int, ...] = (height, width) if samples == 1 else (height, width, samples)
    pixels = np.frombuffer(raw, dtype=dtype).astype(np.dtype(dtype).newbyteorder("=")).reshape(shape)
    return ImageData(pixels, custom)


class NiVisionCodec:
    """Stub of the NI Vision codec (Phase 3: ``imaging/nivision.py`` calls NI Vision through ``ctypes``)."""

    name = "nivision"

    def __init__(self, library: str = "") -> None:
        self.library = library
        self._available: bool | None = None

    def is_available(self) -> bool:
        """True when ``ctypes`` can load the NI Vision library (Windows only; the result is cached)."""
        if self._available is None:
            self._available = False
            if sys.platform == "win32":
                name = self.library or ctypes.util.find_library("nivision") or "nivision.dll"
                try:
                    ctypes.CDLL(name)
                    self._available = True
                except OSError as exc:
                    log.debug("NI Vision library %r cannot be loaded: %s", name, exc)
        return self._available

    def flatten(self, image: ImageData) -> bytes:
        """Phase 1: always :class:`CodecError` (9508)."""
        raise CodecError(Code.IMAGE_CODEC_UNAVAILABLE,
                         "NI Vision image flattening is not implemented in this phase (phase 3)")


def make_codec(options: ImageOptions) -> ImageCodec:
    """The codec named by ``[image] codec`` (``provisional`` | ``nivision``, case-insensitive);
    :class:`ConfigError` for other names; WARNING when the NI Vision library is not available."""
    name = options.codec.strip().lower()
    if name == ProvisionalCodec.name:
        return ProvisionalCodec()
    if name == NiVisionCodec.name:
        codec = NiVisionCodec(options.nivision_library)
        if not codec.is_available():
            log.warning("[image] codec = nivision: the NI Vision library %r is not available; every image reply "
                        "will carry error %d", options.nivision_library or "nivision", Code.IMAGE_CODEC_UNAVAILABLE)
        else:
            log.warning("[image] codec = nivision: image flattening is not implemented in this phase; every image "
                        "reply will carry error %d", Code.IMAGE_CODEC_UNAVAILABLE)
        return codec
    raise ConfigError(Code.LV_INVALID_INPUT, f"[image] codec: unknown codec {options.codec!r} (provisional | nivision)")
