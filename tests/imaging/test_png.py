"""aloxcam.imaging.png: hand-made PNG files (every filter type) and the gray writer (05 §8.2, design §3.6.8)."""

from __future__ import annotations

import struct
import zlib
from collections.abc import Sequence
from pathlib import Path

import numpy as np
import pytest

from aloxcam.imaging.png import PNG_SIGNATURE, PngError, PngUnsupported, read_png, write_png

COLOR_CHANNELS = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}


def chunk(ctype: bytes, body: bytes) -> bytes:
    return struct.pack(">I", len(body)) + ctype + body + struct.pack(">I", zlib.crc32(ctype + body) & 0xFFFFFFFF)


def paeth(a: int, b: int, c: int) -> int:
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    return b if pb <= pc else c


def filter_row(ftype: int, row: bytes, prior: bytes, bpp: int) -> bytes:
    """Independent PNG filter encoder (PNG spec §9.2)."""
    out = bytearray()
    for i, x in enumerate(row):
        a = row[i - bpp] if i >= bpp else 0
        b = prior[i]
        c = prior[i - bpp] if i >= bpp else 0
        pred = {0: 0, 1: a, 2: b, 3: (a + b) // 2, 4: paeth(a, b, c)}[ftype]
        out.append((x - pred) & 0xFF)
    return bytes(out)


def make_png(pixels: np.ndarray, color_type: int, bit_depth: int, filters: Sequence[int] = (0,), *,
             interlace: int = 0, extra: Sequence[bytes] = ()) -> bytes:
    h, w = pixels.shape[:2]
    channels = COLOR_CHANNELS[color_type]
    if bit_depth == 16:
        rows = np.ascontiguousarray(pixels).astype(">u2").view(np.uint8).reshape(h, -1)
    else:
        rows = np.ascontiguousarray(pixels).astype(np.uint8).reshape(h, -1)
    bpp = channels * max(1, bit_depth // 8)
    raw = bytearray()
    prior = bytes(rows.shape[1])
    for y in range(h):
        row = rows[y].tobytes()
        ftype = filters[y % len(filters)]
        raw.append(ftype)
        raw += filter_row(ftype, row, prior, bpp)
        prior = row
    ihdr = struct.pack(">IIBBBBB", w, h, bit_depth, color_type, 0, 0, interlace)
    return (PNG_SIGNATURE + chunk(b"IHDR", ihdr) + b"".join(extra) + chunk(b"IDAT", zlib.compress(bytes(raw)))
            + chunk(b"IEND", b""))


@pytest.fixture
def rng() -> np.random.Generator:
    return np.random.default_rng(1234)


def write(tmp_path: Path, data: bytes, name: str = "img.png") -> Path:
    path = tmp_path / name
    path.write_bytes(data)
    return path


@pytest.mark.parametrize("ftype", [0, 1, 2, 3, 4])
@pytest.mark.parametrize(("color_type", "depth", "dtype", "channels"), [
    (0, 8, np.uint8, 1), (0, 16, np.uint16, 1), (2, 8, np.uint8, 3), (6, 8, np.uint8, 4), (2, 16, np.uint16, 3),
    (6, 16, np.uint16, 4),
])
def test_each_filter_type_decodes(tmp_path: Path, rng: np.random.Generator, ftype: int, color_type: int, depth: int,
                                  dtype: type, channels: int) -> None:
    top = 255 if depth == 8 else 65535
    shape = (5, 7) if channels == 1 else (5, 7, channels)
    pixels = rng.integers(0, top + 1, size=shape).astype(dtype)
    got = read_png(write(tmp_path, make_png(pixels, color_type, depth, (ftype,))))
    assert got.dtype == dtype and got.shape == shape
    assert np.array_equal(got, pixels)


def test_mixed_filters_and_ancillary_chunks(tmp_path: Path, rng: np.random.Generator) -> None:
    pixels = rng.integers(0, 256, size=(10, 9, 3)).astype(np.uint8)
    data = make_png(pixels, 2, 8, (0, 1, 2, 3, 4), extra=[chunk(b"tEXt", b"Comment\x00hello")])
    assert np.array_equal(read_png(write(tmp_path, data)), pixels)


def test_hand_made_rgba_values(tmp_path: Path) -> None:
    pixels = np.array([[[255, 0, 0, 255], [0, 255, 0, 128]], [[0, 0, 255, 0], [10, 20, 30, 40]]], dtype=np.uint8)
    got = read_png(write(tmp_path, make_png(pixels, 6, 8, (4,))))
    assert got.tolist() == pixels.tolist()


@pytest.mark.parametrize("dtype", [np.uint8, np.uint16])
def test_write_read_round_trip_gray(tmp_path: Path, rng: np.random.Generator, dtype: type) -> None:
    top = np.iinfo(dtype).max
    pixels = rng.integers(0, int(top) + 1, size=(13, 17)).astype(dtype)
    path = tmp_path / "gray.png"
    write_png(path, pixels)
    got = read_png(path)
    assert got.dtype == dtype and np.array_equal(got, pixels)
    raw = path.read_bytes()
    assert raw.startswith(PNG_SIGNATURE) and raw[12:16] == b"IHDR"
    assert raw[24] == (8 if dtype is np.uint8 else 16) and raw[25] == 0      # bit depth, colour type gray


def test_write_png_accepts_views(tmp_path: Path) -> None:
    pixels = np.arange(24, dtype=np.uint8).reshape(4, 6)[::-1, ::-1]
    write_png(tmp_path / "v.png", pixels)
    assert np.array_equal(read_png(tmp_path / "v.png"), pixels)


@pytest.mark.parametrize("bad", [np.zeros((2, 2), np.float32), np.zeros((2, 2, 3), np.uint8),
                                 np.zeros((0, 3), np.uint8), np.zeros(4, np.uint8)])
def test_write_png_rejects_other_arrays(tmp_path: Path, bad: np.ndarray) -> None:
    with pytest.raises(ValueError):
        write_png(tmp_path / "x.png", bad)


@pytest.mark.parametrize(("color_type", "depth", "shape", "interlace"), [
    (3, 8, (2, 2), 0),            # palette
    (4, 8, (2, 2, 2), 0),         # gray + alpha
    (0, 4, (2, 2), 0),            # < 8 bits
    (0, 8, (2, 2), 1),            # interlaced
])
def test_unsupported_formats(tmp_path: Path, color_type: int, depth: int, shape: tuple[int, ...],
                             interlace: int) -> None:
    data = make_png(np.zeros(shape, np.uint8), color_type, 8 if depth == 4 else depth, interlace=interlace)
    if depth == 4:                                     # rewrite the IHDR bit depth (the body is never decoded)
        ihdr = struct.pack(">IIBBBBB", shape[1], shape[0], 4, color_type, 0, 0, 0)
        data = PNG_SIGNATURE + chunk(b"IHDR", ihdr) + data[len(PNG_SIGNATURE) + 25:]
    with pytest.raises(PngUnsupported) as info:
        read_png(write(tmp_path, data))
    assert info.value.color_type == color_type
    assert info.value.bit_depth == depth
    assert isinstance(info.value, PngError)


def test_malformed_files(tmp_path: Path) -> None:
    good = make_png(np.zeros((3, 3), np.uint8), 0, 8)
    with pytest.raises(PngError, match="signature"):
        read_png(write(tmp_path, b"GIF89a" + good[6:]))
    corrupted = bytearray(good)
    corrupted[-20] ^= 0xFF                              # inside the IDAT body
    with pytest.raises(PngError, match="CRC"):
        read_png(write(tmp_path, bytes(corrupted)))
    with pytest.raises(PngError):
        read_png(write(tmp_path, good[:40]))
    no_ihdr = PNG_SIGNATURE + chunk(b"IEND", b"")
    with pytest.raises(PngError, match="IHDR"):
        read_png(write(tmp_path, no_ihdr))
    bad_depth = PNG_SIGNATURE + chunk(b"IHDR", struct.pack(">IIBBBBB", 2, 2, 4, 2, 0, 0, 0)) + chunk(b"IEND", b"")
    with pytest.raises(PngError, match="invalid colour type"):
        read_png(write(tmp_path, bad_depth))
    short = make_png(np.zeros((3, 3), np.uint8), 0, 8)
    ihdr_wrong_size = PNG_SIGNATURE + chunk(b"IHDR", struct.pack(">IIBBBBB", 4, 3, 8, 0, 0, 0, 0))
    with pytest.raises(PngError, match="expected"):
        read_png(write(tmp_path, ihdr_wrong_size + short[len(PNG_SIGNATURE) + 25:]))
    bad_filter = PNG_SIGNATURE + chunk(b"IHDR", struct.pack(">IIBBBBB", 1, 1, 8, 0, 0, 0, 0)) + chunk(
        b"IDAT", zlib.compress(b"\x09\x00")) + chunk(b"IEND", b"")
    with pytest.raises(PngError, match="filter"):
        read_png(write(tmp_path, bad_filter))


def test_missing_file_raises_oserror(tmp_path: Path) -> None:
    with pytest.raises(OSError):
        read_png(tmp_path / "absent.png")
