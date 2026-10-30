"""aloxcam.imaging.codec: the provisional ``ALXIMG01`` format, the NI Vision stub and ``make_codec`` (02 §8.5,
design §3.6.9)."""

from __future__ import annotations

import logging
import sys

import numpy as np
import pytest

from aloxcam.errors import Code, CodecError, ConfigError, ProtocolError
from aloxcam.imagedata import ImageData
from aloxcam.imaging.codec import PROVISIONAL_MAGIC, NiVisionCodec, ProvisionalCodec, decode_provisional, make_codec
from aloxcam.settings import ImageOptions


def test_golden_header_2x3_u8_with_two_keys() -> None:
    pixels = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.uint8)            # 2 rows × 3 columns
    image = ImageData(pixels, {"buffer": "7", "type": "High-Mag"})
    expected = bytes.fromhex(
        "414c58494d473031"          # "ALXIMG01"
        "0000" "0000"               # NI image type U8 = 0, reserved
        "00000003" "00000002"       # width 3, height 2
        "00000002"                  # 2 custom-data keys
        "00000006" "627566666572" "00000001" "37"                  # "buffer" → "7"
        "00000004" "74797065" "00000008" "486967682d4d6167"        # "type" → "High-Mag"
        "00000006" "010203040506"   # pixel bytes, row-major
    )
    assert ProvisionalCodec().flatten(image) == expected
    assert decode_provisional(expected) == image


@pytest.mark.parametrize("dtype", [np.uint8, np.uint16, np.int16, np.float32])
def test_round_trip_gray(dtype: type) -> None:
    rng = np.random.default_rng(7)
    pixels = (rng.random((4, 5)) * 1000).astype(dtype)
    image = ImageData(pixels, {"name": "HighMag", "simulation": "FALSE"})
    data = ProvisionalCodec().flatten(image)
    back = decode_provisional(data)
    assert back == image
    assert back.pixels.dtype == np.dtype(dtype)
    assert data[8:10] == image.ni_image_type.to_bytes(2, "big")


def test_u16_pixels_are_big_endian() -> None:
    data = ProvisionalCodec().flatten(ImageData(np.array([[0x1234]], dtype=np.uint16)))
    assert data.endswith(b"\x00\x00\x00\x02\x12\x34")


def test_round_trip_rgba_and_views() -> None:
    rgba = np.arange(2 * 3 * 4, dtype=np.uint8).reshape(2, 3, 4)
    assert decode_provisional(ProvisionalCodec().flatten(ImageData(rgba))) == ImageData(rgba)
    view = np.arange(12, dtype=np.uint8).reshape(3, 4)[::-1, ::-1]
    assert decode_provisional(ProvisionalCodec().flatten(ImageData(view))) == ImageData(view)


def test_custom_data_text_is_cp1252_with_replacement() -> None:
    image = ImageData(np.zeros((1, 1), np.uint8), {"name": "caméra €", "x": "日本"})
    back = decode_provisional(ProvisionalCodec().flatten(image))
    assert back.custom_data == {"name": "caméra €", "x": "??"}


def test_unsupported_pixel_type_is_a_codec_error() -> None:
    with pytest.raises(CodecError) as info:
        ProvisionalCodec().flatten(ImageData(np.zeros((2, 2), np.int32)))
    assert info.value.code == Code.IMAGE_CODEC_UNAVAILABLE


def test_decode_errors() -> None:
    good = ProvisionalCodec().flatten(ImageData(np.zeros((2, 2), np.uint8), {"k": "v"}))
    for bad in (b"", b"ALXIMG02" + good[8:], good[:-1], good + b"\x00", good[:20]):
        with pytest.raises(ProtocolError) as info:
            decode_provisional(bad)
        assert info.value.code == Code.UNFLATTEN_FAILED
    wrong_count = bytearray(good)
    wrong_count[-8:-4] = (5).to_bytes(4, "big")
    with pytest.raises(ProtocolError, match="pixel bytes"):
        decode_provisional(bytes(wrong_count))
    unknown_type = bytearray(good)
    unknown_type[8:10] = (3).to_bytes(2, "big")
    with pytest.raises(ProtocolError, match="image type"):
        decode_provisional(bytes(unknown_type))
    assert good.startswith(PROVISIONAL_MAGIC)


def test_nivision_stub() -> None:
    codec = NiVisionCodec("does-not-exist.dll")
    assert codec.name == "nivision"
    if sys.platform != "win32":
        assert codec.is_available() is False
    with pytest.raises(CodecError) as info:
        codec.flatten(ImageData(np.zeros((1, 1), np.uint8)))
    assert info.value.code == Code.IMAGE_CODEC_UNAVAILABLE == 9508
    assert info.value.source == "NI Vision image flattening is not implemented in this phase (phase 3)"


def test_make_codec(caplog: pytest.LogCaptureFixture) -> None:
    assert isinstance(make_codec(ImageOptions()), ProvisionalCodec)
    assert make_codec(ImageOptions(codec=" Provisional ")).name == "provisional"
    with caplog.at_level(logging.WARNING, logger="aloxcam.imaging.codec"):
        codec = make_codec(ImageOptions(codec="nivision", nivision_library="nivision.dll"))
    assert isinstance(codec, NiVisionCodec) and codec.library == "nivision.dll"
    assert any("nivision" in r.getMessage() for r in caplog.records)
    with pytest.raises(ConfigError):
        make_codec(ImageOptions(codec="png"))
