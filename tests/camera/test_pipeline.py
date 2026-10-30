"""aloxcam.camera.pipeline: flip, flat-field, custom data and the PNG-folder simulation (05 §7, §8;
design §3.6.4)."""

from __future__ import annotations

import struct
import time
import zlib
from pathlib import Path

import numpy as np
import pytest

from aloxcam.camera.pipeline import (
    CUSTOM_DATA_KEYS,
    SimulationError,
    build_custom_data,
    flat_field_correct,
    flip_image,
    list_simulation_files,
    load_correction_image,
    lv_timestamp_text,
    simulate_next_image,
    to_u8_like_simulation,
)
from aloxcam.errors import CameraError, Code
from aloxcam.imaging.png import PNG_SIGNATURE, write_png
from aloxcam.settings import HIGH_MAG, LOW_MAG, CameraSettings
from aloxcam.testing import FixedClock

IMG = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.uint8)


# ---------------------------------------------------------------------------------------------- flip
def test_flip_semantics() -> None:
    assert flip_image(IMG, False, False) is IMG
    assert flip_image(IMG, True, False).tolist() == [[4, 5, 6], [1, 2, 3]]       # H = top ↔ bottom
    assert flip_image(IMG, False, True).tolist() == [[3, 2, 1], [6, 5, 4]]       # V = left ↔ right
    assert flip_image(IMG, True, True).tolist() == [[6, 5, 4], [3, 2, 1]]        # central (180°)


# ---------------------------------------------------------------------------------------------- flat-field
def test_flat_field_formula_on_a_2x2_example() -> None:
    img = np.array([[10, 20], [30, 40]], dtype=np.uint8)
    flat = np.array([[110, 210], [160, 250]], dtype=np.uint8)
    dark = np.array([[0, 10], [10, 0]], dtype=np.uint8)
    # F − D = [[110, 200], [150, 250]], m = 177.5, k·m = 266.25
    # (I − D)·k·m/(F − D) = [[24.2045, 13.3125], [35.5, 42.6]] → half-even rounding
    out = flat_field_correct(img, flat, dark, 1.5)
    assert out.dtype == np.uint8
    assert out.tolist() == [[24, 13], [36, 43]]


def test_flat_field_saturation_zero_denominator_and_factor() -> None:
    img = np.array([[200, 50, 10]], dtype=np.uint8)
    flat = np.array([[10, 7, 30]], dtype=np.uint8)
    dark = np.array([[0, 7, 0]], dtype=np.uint8)                # F == D on the second pixel
    # F − D = [10, 0, 30], m = 40/3, k·m = 26.67: [533 → 255 (saturated), 0 (F = D), 8.9 → 9]
    assert flat_field_correct(img, flat, dark, 2.0).tolist() == [[255, 0, 9]]
    assert flat_field_correct(img, flat, dark, 0.0).tolist() == [[0, 0, 0]]       # factor 0 → black (05 #18)
    u16 = flat_field_correct(np.array([[1000]], np.uint16), np.array([[500]], np.uint16),
                             np.array([[0]], np.uint16), 0.1)
    assert u16.tolist() == [[round(1000 * float(np.float32(0.1)))]]               # factor coerced to SGL


def test_flat_field_errors() -> None:
    img = np.zeros((2, 2), np.uint8)
    for flat, dark in ((None, img), (img, None)):
        with pytest.raises(CameraError) as info:
            flat_field_correct(img, flat, dark, 1.0)
        assert info.value.code == Code.VISION_OPERATION
    with pytest.raises(CameraError, match="does not match"):
        flat_field_correct(img, np.zeros((2, 3), np.uint8), img, 1.0)
    with pytest.raises(CameraError, match="does not match"):
        flat_field_correct(img, img, np.zeros((2, 2), np.uint16), 1.0)


# ---------------------------------------------------------------------------------------------- custom data
def test_lv_timestamp_text() -> None:
    now = 1_700_000_000.125
    expected = time.strftime("%H:%M:%S", time.localtime(now))
    assert lv_timestamp_text(now) == expected + ":.125"
    assert lv_timestamp_text(now, ",") == expected + ":,125"
    assert lv_timestamp_text(1_700_000_000.0).endswith(":.000")


def test_build_custom_data() -> None:
    now = 1_700_000_000.5
    settings = CameraSettings.default_low_mag()
    data = build_custom_data(42, settings, LOW_MAG, now=now)
    assert tuple(data) == CUSTOM_DATA_KEYS
    assert data == {"buffer": "42", "timestamp": lv_timestamp_text(now), "name": "LowMag", "type": "Low-Mag",
                    "simulation": "FALSE", "flatfield": "FALSE"}
    hm = build_custom_data(0, CameraSettings(camera_name="X", flat_field=True), HIGH_MAG, now=now,
                           decimal_separator=",")
    assert hm["type"] == "High-Mag" and hm["flatfield"] == "TRUE" and hm["name"] == "X"
    assert hm["timestamp"].endswith(":,500")


# ---------------------------------------------------------------------------------------------- pixel types
def test_to_u8_like_simulation() -> None:
    u8 = np.array([[1, 2]], np.uint8)
    assert to_u8_like_simulation(u8) is u8
    assert to_u8_like_simulation(np.array([[0x1234, 0xFFFF, 0x00FF]], np.uint16)).tolist() == [[0x12, 0xFF, 0x00]]
    rgb = np.array([[[255, 0, 0], [0, 255, 0], [0, 0, 255], [10, 20, 30]]], np.uint8)
    # 0.299·255 = 76.245, 0.587·255 = 149.685, 0.114·255 = 29.07, 0.299·10 + 0.587·20 + 0.114·30 = 18.12
    assert to_u8_like_simulation(rgb).tolist() == [[76, 150, 29, 18]]
    rgba = np.concatenate([rgb, np.full((1, 4, 1), 7, np.uint8)], axis=2)
    assert to_u8_like_simulation(rgba).tolist() == [[76, 150, 29, 18]]
    for bad in (np.zeros((2, 2), np.float32), np.zeros((2, 2, 3), np.uint16), np.zeros((2, 2, 2), np.uint8)):
        with pytest.raises(CameraError) as info:
            to_u8_like_simulation(bad)
        assert info.value.code == Code.SIM_INVALID_TYPE == -9011
        assert info.value.source == "Invalid Type"


# ---------------------------------------------------------------------------------------------- simulation folder
def _rgb16_png(path: Path) -> None:
    def chunk(t: bytes, b: bytes) -> bytes:
        return struct.pack(">I", len(b)) + t + b + struct.pack(">I", zlib.crc32(t + b) & 0xFFFFFFFF)
    ihdr = struct.pack(">IIBBBBB", 1, 1, 16, 2, 0, 0, 0)
    path.write_bytes(PNG_SIGNATURE + chunk(b"IHDR", ihdr) + chunk(b"IDAT", zlib.compress(b"\x00" + bytes(6)))
                     + chunk(b"IEND", b""))


@pytest.fixture
def sim_folder(tmp_path: Path) -> Path:
    folder = tmp_path / "sim"
    folder.mkdir()
    write_png(folder / "b.png", np.full((2, 3), 2, np.uint8))
    write_png(folder / "A.PNG", np.full((4, 5), 0x0100 * 9, np.uint16))           # 16-bit → >> 8 = 9
    write_png(folder / "c.png", np.full((3, 3), 3, np.uint8))
    (folder / "notes.txt").write_text("not an image")
    (folder / "d.png").mkdir()                                                     # a folder, not a file
    return folder


def test_list_simulation_files(sim_folder: Path, tmp_path: Path) -> None:
    assert list_simulation_files(sim_folder) == ["A.PNG", "b.png", "c.png"]
    with pytest.raises(CameraError) as info:
        list_simulation_files(tmp_path / "missing")
    assert info.value.code == Code.IMAGE_FILE


def test_timer_zero_cycles_through_the_files(sim_folder: Path) -> None:
    clock = FixedClock(1000)
    index, t = 0, 0
    shapes = []
    for _ in range(4):
        img, index, t = simulate_next_image(sim_folder, 0, index, t, clock)
        shapes.append(img.shape)
        assert img.dtype == np.uint8
        clock.advance(1)
    assert shapes == [(4, 5), (2, 3), (3, 3), (4, 5)]
    assert index == 1 and t == 1003


def test_first_file_is_mono16_shifted(sim_folder: Path) -> None:
    img, _, _ = simulate_next_image(sim_folder, 0, 0, 0, FixedClock(5))
    assert int(img[0, 0]) == 9


def test_timer_repeats_the_file_until_elapsed(sim_folder: Path) -> None:
    clock = FixedClock(1200)
    img, index, t = simulate_next_image(sim_folder, 500, 1, 1000, clock)          # 1200 < 1500: not elapsed
    assert img.shape == (2, 3) and (index, t) == (1, 1000)
    clock.set(1500)                                                                # ≥: elapsed
    img, index, t = simulate_next_image(sim_folder, 500, index, t, clock)
    assert img.shape == (2, 3) and (index, t) == (2, 1500)
    clock.set(1700)
    img, index, t = simulate_next_image(sim_folder, 500, index, t, clock)
    assert img.shape == (3, 3) and (index, t) == (2, 1500)


def test_index_wraps_modulo_the_file_count(sim_folder: Path) -> None:
    img, index, _ = simulate_next_image(sim_folder, 0, 7, 0, FixedClock(10))      # 7 mod 3 = 1
    assert img.shape == (2, 3) and index == 2


def test_empty_folder_fails_and_advances_like_labview(tmp_path: Path) -> None:
    clock = FixedClock(50)
    with pytest.raises(SimulationError) as info:
        simulate_next_image(tmp_path, 0, 0, 0, clock)                             # x mod 0 = x
    assert info.value.code == Code.IMAGE_FILE
    assert (info.value.index_out, info.value.time_out) == (1, 50)


def test_bad_files(sim_folder: Path) -> None:
    (sim_folder / "0bad.png").write_bytes(b"not a png")                            # sorts first
    with pytest.raises(SimulationError) as info:
        simulate_next_image(sim_folder, 0, 0, 0, FixedClock(9))
    assert info.value.code == Code.IMAGE_FILE and info.value.index_out == 1
    (sim_folder / "0bad.png").unlink()
    _rgb16_png(sim_folder / "0rgb16.png")
    with pytest.raises(SimulationError) as info:
        simulate_next_image(sim_folder, 0, 0, 0, FixedClock(9))
    assert info.value.code == Code.SIM_INVALID_TYPE and info.value.source == "Invalid Type"
    assert info.value.index_out == 1


def test_load_correction_image(tmp_path: Path) -> None:
    write_png(tmp_path / "ffc.png", np.full((2, 2), 9, np.uint8))
    assert load_correction_image(tmp_path / "ffc.png").tolist() == [[9, 9], [9, 9]]
    with pytest.raises(CameraError) as info:
        load_correction_image(tmp_path / "missing.png")
    assert info.value.code == Code.IMAGE_FILE and "missing.png" in info.value.source
    (tmp_path / "bad.png").write_bytes(b"xx")
    with pytest.raises(CameraError) as info:
        load_correction_image(tmp_path / "bad.png")
    assert info.value.code == Code.IMAGE_FILE
