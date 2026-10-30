"""aloxcam.imagedata.ImageData (02 §8.5, design §3.1.6)."""

from __future__ import annotations

import numpy as np
import pytest

from aloxcam.imagedata import NI_IMAGE_I16, NI_IMAGE_RGB, NI_IMAGE_SGL, NI_IMAGE_U8, NI_IMAGE_U16, ImageData


@pytest.mark.parametrize(
    ("dtype", "shape", "expected"),
    [
        (np.uint8, (3, 5), NI_IMAGE_U8),
        (np.int16, (3, 5), NI_IMAGE_I16),
        (np.float32, (3, 5), NI_IMAGE_SGL),
        (np.uint16, (3, 5), NI_IMAGE_U16),
        (np.uint8, (3, 5, 4), NI_IMAGE_RGB),
    ],
)
def test_ni_image_type(dtype: type, shape: tuple[int, ...], expected: int) -> None:
    assert ImageData(np.zeros(shape, dtype)).ni_image_type == expected


@pytest.mark.parametrize(("dtype", "shape"), [(np.float64, (2, 2)), (np.int32, (2, 2)), (np.uint8, (2, 2, 3)),
                                              (np.uint16, (2, 2, 4)), (np.uint8, (4,))])
def test_ni_image_type_rejects(dtype: type, shape: tuple[int, ...]) -> None:
    with pytest.raises(ValueError):
        ImageData(np.zeros(shape, dtype)).ni_image_type  # noqa: B018


def test_width_height() -> None:
    img = ImageData(np.zeros((480, 640), np.uint8))
    assert (img.width, img.height) == (640, 480)
    rgba = ImageData(np.zeros((2, 3, 4), np.uint8))
    assert (rgba.width, rgba.height) == (3, 2)


def test_custom_data_order_and_equality() -> None:
    a = ImageData(np.arange(6, dtype=np.uint8).reshape(2, 3), {"buffer": "1", "name": "HighMag"})
    b = ImageData(np.arange(6, dtype=np.uint8).reshape(2, 3), {"buffer": "1", "name": "HighMag"})
    assert list(a.custom_data) == ["buffer", "name"]
    assert a == b
    assert a != ImageData(np.arange(6, dtype=np.uint8).reshape(2, 3), {"name": "HighMag", "buffer": "1"})
    assert a != ImageData(np.arange(6, dtype=np.uint16).reshape(2, 3), dict(a.custom_data))
    assert ImageData(np.zeros((1, 1), np.uint8)).custom_data == {}
