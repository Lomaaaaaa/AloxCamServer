"""``ImageData`` (an IMAQ image with its custom-data keys) and the pluggable ``ImageCodec`` protocol.

Spec: 02 §8.5 (``IMAQ Flatten Image to String``), 05 §7.5 (custom data).
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

__all__ = ["NI_IMAGE_I16", "NI_IMAGE_RGB", "NI_IMAGE_SGL", "NI_IMAGE_U16", "NI_IMAGE_U8", "ImageCodec", "ImageData"]

NI_IMAGE_U8 = 0
NI_IMAGE_I16 = 1
NI_IMAGE_SGL = 2
NI_IMAGE_RGB = 4
NI_IMAGE_U16 = 7

_GRAY_TYPES = {np.dtype(np.uint8): NI_IMAGE_U8, np.dtype(np.int16): NI_IMAGE_I16,
               np.dtype(np.float32): NI_IMAGE_SGL, np.dtype(np.uint16): NI_IMAGE_U16}


@dataclass(frozen=True, eq=False)
class ImageData:
    """An image as the camera worker hands it to the reply path (02 §8.5).

    ``pixels`` is 2-D ``(rows, cols)`` uint8 | uint16 | int16 | float32, or ``(rows, cols, 4)`` uint8 RGBA.
    Equality compares dtype, shape, pixel values and custom data.
    """

    pixels: np.ndarray
    custom_data: Mapping[str, str] = field(default_factory=dict)   # IMAQ custom-data keys, insertion order kept

    @property
    def width(self) -> int:
        """Number of columns."""
        return int(self.pixels.shape[1])

    @property
    def height(self) -> int:
        """Number of rows."""
        return int(self.pixels.shape[0])

    @property
    def ni_image_type(self) -> int:
        """``NI_IMAGE_*`` code of the pixel type; :class:`ValueError` for other dtypes or shapes."""
        px = self.pixels
        if px.ndim == 2 and px.dtype in _GRAY_TYPES:
            return _GRAY_TYPES[px.dtype]
        if px.ndim == 3 and px.shape[2] == 4 and px.dtype == np.uint8:
            return NI_IMAGE_RGB
        raise ValueError(f"no NI image type for pixels of dtype {px.dtype} and shape {px.shape}")

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, ImageData):
            return NotImplemented
        a, b = self.pixels, other.pixels
        return (a.dtype == b.dtype and a.shape == b.shape and bool(np.array_equal(a, b))
                and list(self.custom_data.items()) == list(other.custom_data.items()))

    __hash__ = None  # type: ignore[assignment]


class ImageCodec(Protocol):
    """The ``IMAQ Flatten Image to String`` replacement (02 §8.5). Called from the camera worker only."""

    name: str

    def flatten(self, image: ImageData) -> bytes:
        """Flattened image bytes for an IMAGE reply; raises :class:`aloxcam.errors.CodecError`."""
        ...
