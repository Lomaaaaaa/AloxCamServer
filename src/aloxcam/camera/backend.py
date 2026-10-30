"""The camera driver interface (the NI-IMAQdx replacement) and a reusable IMAQdx-like frame ring.

Spec: 05 §2 (camera model, attributes), §4 (session lifecycle), §6.4 (buffer numbering, ``Buffer Number`` mode and
``OverwriteMode`` "get newest"), §12.3–§12.4 (IMAQdx → Python equivalents, frame-ring sketch); design §3.6.1.

A backend mirrors the IMAQdx calls the LabVIEW server makes: :class:`CameraBackend` is the driver (``Open Camera``,
release at shutdown), :class:`CameraDevice` one IMAQdx session. Attribute names are IMAQdx paths
(``CameraAttributes::<Category>::<Feature>``); a real backend (Phase 3: IDS peak, VmbPy or Harvesters) maps them to
GenICam feature names and keeps a :class:`FrameRing` filled by its own grab thread. A command attribute
(``TriggerSoftware``) is executed by writing ``BOOL True``.

Threading: devices are called from the camera worker only (plus ``close()`` from the shutdown path once the worker
has stopped); implementations guard their own grab thread. :class:`FrameRing` is thread-safe.
"""

from __future__ import annotations

import dataclasses
import threading
import time
from abc import ABC, abstractmethod
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum

import numpy as np

from aloxcam.clock import U32_MASK

__all__ = ["CameraBackend", "CameraDevice", "Frame", "FrameRing", "ValueType"]


class ValueType(Enum):
    """IMAQdx property-node value accessors (05 §5.1): ``ValueDBL``, ``ValueI64``, ``ValueU32``, ``ValueBool``,
    ``ValueString``."""

    DBL = "DBL"
    I64 = "I64"
    U32 = "U32"
    BOOL = "BOOL"
    STRING = "STRING"


@dataclass(frozen=True)
class Frame:
    """One acquired buffer (05 §6.4)."""

    number: int                           # cumulative buffer number (U32), 0 = first frame after configure
    pixels: np.ndarray                    # owned 2-D copy
    t_monotonic: float


class CameraBackend(ABC):
    """The camera driver, ~ NI-IMAQdx (05 §2, §12.3)."""

    @abstractmethod
    def open_camera(self, name: str) -> CameraDevice:
        """``IMAQdx Open Camera`` in ``Controller`` mode (05 §4.2); :class:`aloxcam.errors.CameraError` if the
        camera is unknown or cannot be opened."""

    @abstractmethod
    def close(self) -> None:
        """Release the driver at shutdown (05 #10 fix); idempotent."""


class CameraDevice(ABC):
    """One open camera, ~ one IMAQdx session (05 §4). Every failure raises :class:`aloxcam.errors.CameraError`."""

    name: str

    @abstractmethod
    def bus_type(self) -> str:
        """IMAQdx session property ``BusType`` (display only, 05 §4.2)."""

    @abstractmethod
    def serial_number(self) -> tuple[int, int]:
        """``(SerialNumberHigh, SerialNumberLow)`` (U32, display only)."""

    @abstractmethod
    def set_packet_size(self, size: int) -> None:
        """Session property ``PacketSize`` (GigE only, 05 §4.2: 8996)."""

    @abstractmethod
    def configure(self, continuous: bool, num_buffers: int) -> None:
        """``IMAQdx Configure Acquisition`` (05 §4.3): ``continuous`` = Grab, else Sequence of ``num_buffers``
        frames; buffer numbering restarts at 0."""

    @abstractmethod
    def unconfigure(self) -> None:
        """``IMAQdx Unconfigure Acquisition`` (05 §4.6)."""

    @abstractmethod
    def close(self) -> None:
        """``IMAQdx Close Camera``: stops a running acquisition first, releases the camera; idempotent."""

    @abstractmethod
    def start(self) -> None:
        """``IMAQdx Start Acquisition`` (05 §4.4)."""

    @abstractmethod
    def stop(self) -> None:
        """``IMAQdx Stop Acquisition`` (05 §4.5)."""

    @abstractmethod
    def acquisition_in_progress(self) -> bool:
        """Session property ``AcquisitionInProgress`` (05 §4.7); stays TRUE after a Sequence completed."""

    @abstractmethod
    def last_buffer_number(self) -> int | None:
        """Session property ``LastBufferNumber`` (cumulative, U32); None before the first frame (05 §6.4)."""

    @abstractmethod
    def buffer_count(self) -> int:
        """Session property ``BufferCount`` (only printed in the 9009 text, 05 §6.3)."""

    @abstractmethod
    def get_buffer(self, number: int | None, timeout_ms: int) -> Frame:
        """``IMAQdx Get Image2`` in ``Buffer Number`` mode: ``number`` None or already overwritten → the newest
        frame (``OverwriteMode`` "get newest", 05 §6.4); waits up to ``timeout_ms`` for a future buffer."""

    @abstractmethod
    def get_next(self, timeout_ms: int) -> Frame:
        """``Get Image2`` mode ``Next``: the buffer after the last one returned (Phase-4 focus grabs, 05 §6.1)."""

    @abstractmethod
    def get_last_new(self, timeout_ms: int) -> Frame:
        """``Get Image2`` mode ``Last New``: the newest buffer not returned yet, waiting for one (05 §6.5)."""

    @abstractmethod
    def read_attribute(self, name: str, vtype: ValueType) -> float | int | bool | str:
        """``ActiveAttribute ← name`` then read ``Value<vtype>`` (05 §5.1)."""

    @abstractmethod
    def write_attribute(self, name: str, vtype: ValueType, value: float | int | bool | str) -> None:
        """``ActiveAttribute ← name`` then write ``Value<vtype>`` (05 §5.1); ``BOOL True`` executes a command."""

    @abstractmethod
    def read_attribute_max(self, name: str) -> float:
        """``ActiveAttribute ← name`` then read ``MaxDBL`` (``Cam Set Exposure`` on HM, 05 §5.2)."""


class FrameRing:
    """IMAQdx buffer ring with a cumulative counter (05 §6.4, §12.4); a building block for backends.

    The grab side calls :meth:`push`; the camera worker reads with the three ``Get Image2`` modes. Lookups wait on a
    condition variable: they return None on timeout, or as soon as ``alive()`` turns FALSE (acquisition stopped).
    ``alive`` is called with the ring's lock held, so it must not take a lock that is held around :meth:`push`.
    Internal numbers are unbounded; reported numbers are masked to U32.
    """

    def __init__(self, num_buffers: int = 5) -> None:
        self._cond = threading.Condition()
        self._frames: deque[tuple[int, Frame]] = deque(maxlen=max(1, num_buffers))
        self._num_buffers = max(1, num_buffers)
        self._next = 0
        self._cursor: int | None = None       # number of the last buffer returned (Next / Last New)

    # ------------------------------------------------------------------ grab side
    def reset(self, num_buffers: int) -> None:
        """Empty ring of ``num_buffers`` slots, numbering from 0 (``Configure Acquisition``)."""
        with self._cond:
            self._num_buffers = max(1, num_buffers)
            self._frames = deque(maxlen=self._num_buffers)
            self._next = 0
            self._cursor = None
            self._cond.notify_all()

    def push(self, pixels: np.ndarray, t_monotonic: float) -> Frame:
        """Store a new frame (the ring keeps its own reference to ``pixels``) and wake the waiters."""
        with self._cond:
            n = self._next
            frame = Frame(n & U32_MASK, pixels, t_monotonic)
            self._frames.append((n, frame))
            self._next = n + 1
            self._cond.notify_all()
            return frame

    def interrupt(self) -> None:
        """Wake every waiter so that it re-checks ``alive()`` (acquisition stopped or device closed)."""
        with self._cond:
            self._cond.notify_all()

    # ------------------------------------------------------------------ properties
    @property
    def num_buffers(self) -> int:
        """Number of slots."""
        with self._cond:
            return self._num_buffers

    @property
    def frames_acquired(self) -> int:
        """Frames pushed since the last :meth:`reset`."""
        with self._cond:
            return self._next

    @property
    def last_number(self) -> int | None:
        """``LastBufferNumber``: the newest frame's number (U32), None before the first frame."""
        with self._cond:
            return self._frames[-1][1].number if self._frames else None

    # ------------------------------------------------------------------ worker side
    def get_buffer(self, number: int | None, timeout_s: float, *,
                   alive: Callable[[], bool] | None = None) -> Frame | None:
        """``Buffer Number`` mode: frame ``number`` if still in the ring, else the newest (None → newest)."""
        with self._cond:
            if not self._wait(lambda: bool(self._frames) and (number is None or self._frames[-1][0] >= number),
                              timeout_s, alive):
                return None
            chosen = self._frames[-1]
            if number is not None:
                chosen = next((item for item in self._frames if item[0] == number), chosen)
            return self._returned(chosen)

    def get_next(self, timeout_s: float, *, alive: Callable[[], bool] | None = None) -> Frame | None:
        """``Next`` mode: the buffer after the last one returned (buffer 0 first); overwritten → newest."""
        with self._cond:
            target = 0 if self._cursor is None else self._cursor + 1
            if not self._wait(lambda: bool(self._frames) and self._frames[-1][0] >= target, timeout_s, alive):
                return None
            chosen = next((item for item in self._frames if item[0] == target), self._frames[-1])
            return self._returned(chosen)

    def get_last_new(self, timeout_s: float, *, alive: Callable[[], bool] | None = None) -> Frame | None:
        """``Last New`` mode: the newest buffer if it was not returned yet, else wait for a newer one."""
        with self._cond:
            if not self._wait(lambda: bool(self._frames) and (self._cursor is None
                                                               or self._frames[-1][0] > self._cursor),
                              timeout_s, alive):
                return None
            return self._returned(self._frames[-1])

    # ------------------------------------------------------------------ helpers (lock held)
    def _wait(self, ready: Callable[[], bool], timeout_s: float, alive: Callable[[], bool] | None) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_s)
        while not ready():
            if alive is not None and not alive():
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            self._cond.wait(remaining)
        return True

    def _returned(self, item: tuple[int, Frame]) -> Frame:
        n, frame = item
        self._cursor = n
        return dataclasses.replace(frame, pixels=frame.pixels.copy())
