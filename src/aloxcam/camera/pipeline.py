"""Image processing applied to acquired frames: flip, flat-field correction, custom data, and the PNG-folder
simulation.

Spec: 05 §7 (processing chain: flip → flat-field → custom data; §7.2 flip, §7.3 flat-field, §7.4 casts,
§7.5 custom data), 05 §8 (``alox_camera_server_simulation.vi``), 05 #19–#22; design §3.6.4, §3.6.8.
"""

from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np

from aloxcam.clock import U32_MASK, TickFn
from aloxcam.errors import TEXT, CameraError, Code
from aloxcam.imaging.png import PngError, PngUnsupported, read_png
from aloxcam.settings import HIGH_MAG, CameraSettings

__all__ = [
    "CUSTOM_DATA_KEYS",
    "SimulationError",
    "build_custom_data",
    "flat_field_correct",
    "flip_image",
    "list_simulation_files",
    "load_correction_image",
    "lv_timestamp_text",
    "simulate_next_image",
    "to_u8_like_simulation",
]

CUSTOM_DATA_KEYS = ("buffer", "timestamp", "name", "type", "simulation", "flatfield")

_LUMINANCE = np.array([0.299, 0.587, 0.114])         # inferred NI weights (05 §8.2)


class SimulationError(CameraError):
    """A failed simulation read that still advanced the file index (05 §8.2: ``Index Out`` / ``Time Out`` are
    computed before the read and do not depend on its error)."""

    def __init__(self, code: int, source: str, *, index_out: int, time_out: int) -> None:
        super().__init__(code, source)
        self.index_out = index_out
        self.time_out = time_out


def flip_image(img: np.ndarray, horizontal: bool, vertical: bool) -> np.ndarray:
    """``alox_vision_flip_image.vi`` (05 §7.2): H → top↔bottom, V → left↔right, both → 180° (a view)."""
    if horizontal and vertical:
        return img[::-1, ::-1]
    if horizontal:
        return img[::-1, :]
    if vertical:
        return img[:, ::-1]
    return img


def flat_field_correct(img: np.ndarray, flat: np.ndarray | None, dark: np.ndarray | None,
                       factor: float) -> np.ndarray:
    """``IMAQ Flat Field Correction`` with the inferred formula of 05 §7.3::

        out = (I − D) × k × m / (F − D),   m = mean(F − D),   k = SGL(factor)

    rounded half to even and saturated to the source type; pixels where ``F = D`` give 0 (unknown in NI).
    A missing correction image or a size / type mismatch raises ``CameraError(9509, …)``.
    """
    if flat is None or dark is None:
        missing = "flat-field" if flat is None else "dark-field"
        raise CameraError(Code.VISION_OPERATION, f"IMAQ Flat Field Correction: no {missing} image loaded")
    for label, ref in (("flat-field", flat), ("dark-field", dark)):
        if ref.shape != img.shape or ref.dtype != img.dtype:
            raise CameraError(Code.VISION_OPERATION,
                              f"IMAQ Flat Field Correction: {label} image {ref.dtype} {ref.shape} does not match the "
                              f"image {img.dtype} {img.shape}")
    k = float(np.float32(factor))                    # DBL → SGL coercion (05 §7.4)
    i, f, d = (a.astype(np.float64) for a in (img, flat, dark))
    den = f - d
    m = float(den.mean()) if den.size else 0.0
    out = np.zeros_like(i)
    np.divide((i - d) * (k * m), den, out=out, where=den != 0)
    if np.issubdtype(img.dtype, np.integer):
        info = np.iinfo(img.dtype)
        return np.clip(np.rint(out), info.min, info.max).astype(img.dtype)
    return out.astype(img.dtype)


def lv_timestamp_text(now: float, decimal_separator: str = ".") -> str:
    """Custom-data ``timestamp``: ``Format Date/Time String "%H:%M:%S:%3u"`` in local time, ``%3u`` written as the
    decimal separator + 3 digits (05 §7.5, #20, inferred): ``"HH:MM:SS:.mmm"``."""
    ms = math.floor(now * 1000) % 1000
    return time.strftime("%H:%M:%S", time.localtime(now)) + ":" + decimal_separator + f"{ms:03d}"


def build_custom_data(buffer_number: int, settings: CameraSettings, camera_type: str, *, now: float,
                      decimal_separator: str = ".") -> dict[str, str]:
    """The six ``IMAQ Write Custom Data`` keys of the camera path, in write order (05 §7.5)."""
    return {
        "buffer": f"{int(buffer_number) & U32_MASK:d}",
        "timestamp": lv_timestamp_text(now, decimal_separator),
        "name": settings.camera_name,
        "type": "High-Mag" if camera_type == HIGH_MAG else "Low-Mag",
        "simulation": "FALSE",
        "flatfield": "TRUE" if settings.flat_field else "FALSE",
    }


def to_u8_like_simulation(img: np.ndarray) -> np.ndarray:
    """Pixel-type handling of the simulation VI (05 §8.2): Mono8 unchanged; Mono16 (``uint16`` / ``int16``)
    ``>> 8``; RGB(A) 8-bit → luminance ``0.299 R + 0.587 G + 0.114 B`` rounded; anything else →
    ``CameraError(-9011, "Invalid Type")``."""
    if img.ndim == 2 and img.dtype == np.uint8:
        return img
    if img.ndim == 2 and img.dtype in (np.uint16, np.int16):
        return (img.astype(np.uint16) >> 8).astype(np.uint8)
    if img.ndim == 3 and img.dtype == np.uint8 and img.shape[2] in (3, 4):
        lum = img[..., :3].astype(np.float64) @ _LUMINANCE
        return np.clip(np.rint(lum), 0, 255).astype(np.uint8)
    raise CameraError(Code.SIM_INVALID_TYPE, TEXT[Code.SIM_INVALID_TYPE])


def list_simulation_files(folder: Path) -> list[str]:
    """``List Folder(folder, "*.png")``: file names with a ``.png`` suffix (any case), in case-insensitive name
    order (inferred, 05 §8.2). An unreadable folder raises ``CameraError(9501, …)``."""
    try:
        names = [p.name for p in Path(folder).iterdir() if p.suffix.lower() == ".png" and p.is_file()]
    except OSError as exc:
        raise CameraError(Code.IMAGE_FILE, f"List Folder {folder}: {exc.strerror or exc}") from None
    return sorted(names, key=lambda n: (n.lower(), n))


def simulate_next_image(folder: Path, timer_ms: int, index_in: int, time_in: int,
                        tick: TickFn) -> tuple[np.ndarray, int, int]:
    """One call of ``alox_camera_server_simulation.vi`` (05 §8.2) → ``(U8 image, index out, time out)``.

    The folder is re-listed on every call. When ``tick() >= (time_in + timer_ms) mod 2**32`` the file
    ``index_in mod n`` is served and the index advances (``x mod 0 = x``, as LabVIEW); otherwise the same index is
    served again. A failed read raises :class:`SimulationError` carrying the advanced index / time: an empty folder
    or an index out of range → 9501, an unreadable file → 9501, an unsupported pixel type → -9011.
    """
    files = list_simulation_files(folder)
    n = len(files)
    now = tick() & U32_MASK
    if now >= ((time_in + timer_ms) & U32_MASK):
        out = index_in % n if n else index_in
        nxt = (out + 1) % n if n else out + 1
        next_time = now
    else:
        out, nxt, next_time = index_in, index_in, time_in
    nxt &= U32_MASK

    def fail(code: int, source: str) -> SimulationError:
        return SimulationError(code, source, index_out=nxt, time_out=next_time)

    if not 0 <= out < n:
        raise fail(Code.IMAGE_FILE, f"IMAQ Read Image And Vision Info 2: no PNG file at index {out} in {folder} "
                                    f"({n} file(s))")
    path = Path(folder) / files[out]
    try:
        img = read_png(path)
    except PngUnsupported:
        raise fail(Code.SIM_INVALID_TYPE, TEXT[Code.SIM_INVALID_TYPE]) from None
    except PngError as exc:
        raise fail(Code.IMAGE_FILE, f"IMAQ Read Image And Vision Info 2: {path}: {exc}") from None
    except OSError as exc:
        raise fail(Code.IMAGE_FILE, f"IMAQ Read Image And Vision Info 2: {path}: {exc.strerror or exc}") from None
    try:
        u8 = to_u8_like_simulation(img)
    except CameraError as exc:
        raise fail(exc.code, exc.source) from None
    return u8, nxt, next_time


def load_correction_image(path: Path) -> np.ndarray:
    """Read a flat-field / dark-field correction file (05 §7.3) as stored (PNG only in Phase 1); any read failure
    → ``CameraError(9501, "<path>: …")``."""
    try:
        return read_png(path)
    except PngError as exc:
        raise CameraError(Code.IMAGE_FILE, f"{path}: {exc}") from None
    except OSError as exc:
        raise CameraError(Code.IMAGE_FILE, f"{path}: {exc.strerror or exc}") from None
