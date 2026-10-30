"""``SimCameraBackend`` / ``SimCameraDevice``: a hardware-free IMAQdx-like camera for ``--simulate`` and the tests.

Spec: 05 §4 (session lifecycle), §4.3 (Grab vs Sequence, triggered acquisition), §5 / §2.1 (attributes), §6.4
(buffer numbering), §9 (digital I/O lines); design §3.6.2.

Behaviour: frames are ``image_height × image_width`` of ``pixel_type`` (``U8`` | ``U16``): a horizontal gradient
scaled by the exposure (1.0 at 10000), with the cumulative frame number written big-endian into the low byte of
the first 4 pixels of row 0 (:func:`frame_number_of` reads it back). Continuous (Grab): a grab thread
``aloxcam-simcam-<name>`` produces frames at the ``AcquisitionFrameRate`` attribute (≤ ``max_frame_rate``); one-shot
(Sequence): ``num_buffers`` frames per start, then the counter stops while ``acquisition_in_progress`` stays TRUE.
``TriggerMode = "On"``: no free-running frames, one frame per ``TriggerSoftware``. ``manual=True``: no grab thread,
frames only through :meth:`SimCameraDevice.emit_frame` (and software triggers). A camera is opened exclusively: a
second ``open_camera`` of an open name fails. Failures raise ``CameraError(9502, …)``; a closed device raises
``-1074360316`` ("invalid camera session").
"""

from __future__ import annotations

import dataclasses
import logging
import math
import threading
import time
import zlib
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any

import numpy as np

from aloxcam.camera.attributes import (
    DEVICE_TEMPERATURE,
    LINE_INVERTER,
    LINE_MODE,
    LINE_SELECTOR,
    LINE_SOURCE,
    NUM_BUFFERS,
    TRIGGER_MODE,
    TRIGGER_SOFTWARE,
    USER_OUTPUT_SELECTOR,
    USER_OUTPUT_VALUE,
)
from aloxcam.camera.backend import CameraBackend, CameraDevice, Frame, FrameRing, ValueType
from aloxcam.clock import U32_MASK
from aloxcam.errors import CameraError, Code
from aloxcam.settings import SimulationOptions

__all__ = [
    "SIM_EXPOSURE",
    "SIM_EXPOSURE_TIME",
    "SIM_FRAME_RATE",
    "SIM_GAIN",
    "SIM_GAIN_ANALOG",
    "SimCameraBackend",
    "SimCameraDevice",
    "frame_number_of",
]

log = logging.getLogger(__name__)

SIM_EXPOSURE = "CameraAttributes::Exposure::Value"
SIM_EXPOSURE_TIME = "CameraAttributes::AcquisitionControl::ExposureTime"
SIM_GAIN = "CameraAttributes::Gain::Value"
SIM_GAIN_ANALOG = "CameraAttributes::AnalogControl::Gain"
SIM_FRAME_RATE = "CameraAttributes::AcquisitionControl::AcquisitionFrameRate"

_EXPOSURE_REFERENCE = 10000.0           # exposure giving the full gradient
_NUMERIC = (ValueType.DBL, ValueType.I64, ValueType.U32)

#: CameraDevice method → IMAQdx operation named in error sources.
_OPERATIONS: Mapping[str, str] = MappingProxyType({
    "bus_type": "IMAQdx Property Node (BusType)",
    "serial_number": "IMAQdx Property Node (SerialNumber)",
    "set_packet_size": "IMAQdx Property Node (PacketSize)",
    "configure": "IMAQdx Configure Acquisition",
    "unconfigure": "IMAQdx Unconfigure Acquisition",
    "close": "IMAQdx Close Camera",
    "start": "IMAQdx Start Acquisition",
    "stop": "IMAQdx Stop Acquisition",
    "acquisition_in_progress": "IMAQdx Property Node (AcquisitionInProgress)",
    "last_buffer_number": "IMAQdx Property Node (LastBufferNumber)",
    "buffer_count": "IMAQdx Property Node (BufferCount)",
    "get_buffer": "IMAQdx Get Image2",
    "get_next": "IMAQdx Get Image2",
    "get_last_new": "IMAQdx Get Image2",
    "read_attribute": "IMAQdx Property Node",
    "write_attribute": "IMAQdx Property Node",
    "read_attribute_max": "IMAQdx Property Node",
})


def frame_number_of(pixels: np.ndarray) -> int:
    """The frame number a simulated frame carries in the low bytes of its first 4 pixels (row 0, unflipped)."""
    row = np.asarray(pixels)[0, :4].astype(np.int64) & 0xFF
    n = 0
    for b in row:
        n = (n << 8) | int(b)
    return n


def _sim_error(source: str) -> CameraError:
    return CameraError(Code.CAMERA_SDK, source)


@dataclass
class _Node:
    """One simulated GenICam feature."""

    kind: str                            # "float" | "int" | "bool" | "str" | "command"
    value: Any = None
    minimum: float | None = None
    maximum: float | None = None
    readonly: bool = False
    choices: tuple[str, ...] = ()
    selector: str | None = None          # value kept per value of this selector node


def _node_map(options: SimulationOptions) -> dict[str, _Node]:
    max_rate = float(options.max_frame_rate) if options.max_frame_rate > 0 else 100.0
    rate = min(max(float(options.frame_rate), 0.1), max_rate) if options.frame_rate > 0 else max_rate
    return {
        SIM_EXPOSURE: _Node("float", _EXPOSURE_REFERENCE, 1.0, 10_000_000.0),
        SIM_GAIN: _Node("float", 0.0, 0.0, 48.0),
        SIM_FRAME_RATE: _Node("float", rate, 0.1, max_rate),
        TRIGGER_MODE: _Node("str", "Off", choices=("Off", "On")),
        TRIGGER_SOFTWARE: _Node("command"),
        DEVICE_TEMPERATURE: _Node("float", float(options.temperature_c), readonly=True),
        LINE_SELECTOR: _Node("int", 0, 0, 3),
        LINE_MODE: _Node("int", 0, 0, 1, selector=LINE_SELECTOR),
        LINE_SOURCE: _Node("int", 0, 0, 15, selector=LINE_SELECTOR),
        LINE_INVERTER: _Node("bool", False, selector=LINE_SELECTOR),
        USER_OUTPUT_SELECTOR: _Node("int", 0, 0, 3),
        USER_OUTPUT_VALUE: _Node("bool", False, selector=USER_OUTPUT_SELECTOR),
    }


_ALIASES: Mapping[str, str] = MappingProxyType({SIM_EXPOSURE_TIME: SIM_EXPOSURE, SIM_GAIN_ANALOG: SIM_GAIN})


class SimCameraDevice(CameraDevice):
    """One simulated camera session. Thread-safe (one lock; the grab thread produces frames under it)."""

    def __init__(self, name: str, options: SimulationOptions, *, clock: Callable[[], float] = time.monotonic,
                 manual: bool = False) -> None:
        self.name = name
        self._options = options
        self._clock = clock
        self._manual = manual
        self._dtype = np.dtype(np.uint16) if options.pixel_type.strip().upper() == "U16" else np.dtype(np.uint8)
        self._lock = threading.RLock()
        self._nodes = _node_map(options)
        self._indexed: dict[tuple[str, int], Any] = {}
        self._attribute_failures: dict[str, CameraError] = {}
        self._method_failures: dict[str, CameraError] = {}
        self._ring = FrameRing(NUM_BUFFERS)
        self._configured = False
        self._continuous = True
        self._num_buffers = 0
        self._sequence_count = 0
        self._acquiring = False
        self._closed = False
        self._packet_size = 0
        self._serial = (0, zlib.crc32(name.encode("utf-8", "replace")) & U32_MASK)
        self._grab: threading.Thread | None = None
        self._grab_stop = threading.Event()
        self._gradient: tuple[float, np.ndarray] | None = None

    # ------------------------------------------------------------------ test hooks and introspection
    @property
    def closed(self) -> bool:
        """TRUE once :meth:`close` ran."""
        return self._closed

    @property
    def configured(self) -> bool:
        """TRUE between :meth:`configure` and :meth:`unconfigure` / :meth:`close`."""
        return self._configured

    @property
    def continuous(self) -> bool:
        """The last ``configure(continuous=…)`` value."""
        return self._continuous

    @property
    def packet_size(self) -> int:
        """The last ``set_packet_size`` value (0 = never written)."""
        return self._packet_size

    @property
    def frames_acquired(self) -> int:
        """Frames produced since the last :meth:`configure`."""
        return self._ring.frames_acquired

    def attribute_value(self, name: str, selector: int | None = None) -> Any:
        """Current value of a node (per-line nodes: of line / output ``selector``, default the selected one)."""
        with self._lock:
            key, node = self._node(name)
            if node.selector is None:
                return node.value
            sel = int(self._nodes[node.selector].value) if selector is None else selector
            return self._indexed.get((key, sel), node.value)

    def emit_frame(self) -> Frame:
        """Test hook: produce one frame now (as the sensor would, whatever the trigger mode).

        ``CameraError(9502, …)`` when not acquiring or when a one-shot sequence is complete.
        """
        with self._lock:
            if self._closed:
                raise CameraError(Code.IMAQDX_INVALID_SESSION, f"emit_frame {self.name}: invalid camera session")
            if not self._acquiring:
                raise _sim_error(f"emit_frame {self.name}: acquisition not in progress (simulation)")
            frame = self._produce_locked()
            if frame is None:
                raise _sim_error(f"emit_frame {self.name}: sequence of {self._num_buffers} frames complete "
                                 f"(simulation)")
            return dataclasses.replace(frame, pixels=frame.pixels.copy())

    def fail_attribute(self, name: str, error: CameraError | None) -> None:
        """Inject (``error``) or clear (None) a failure of every read / write of attribute ``name``."""
        key = _ALIASES.get(name, name)
        with self._lock:
            if error is None:
                self._attribute_failures.pop(key, None)
            else:
                self._attribute_failures[key] = error

    def fail_method(self, method: str, error: CameraError | None) -> None:
        """Inject (``error``) or clear (None) a failure of a :class:`CameraDevice` method (e.g. ``"configure"``).
        An injected ``close`` failure is raised after the device was closed."""
        if method not in _OPERATIONS:
            raise ValueError(f"fail_method: unknown CameraDevice method {method!r}")
        with self._lock:
            if error is None:
                self._method_failures.pop(method, None)
            else:
                self._method_failures[method] = error

    # ------------------------------------------------------------------ CameraDevice: session
    def bus_type(self) -> str:
        with self._lock:
            self._check("bus_type")
            return "Simulator"

    def serial_number(self) -> tuple[int, int]:
        with self._lock:
            self._check("serial_number")
            return self._serial

    def set_packet_size(self, size: int) -> None:
        with self._lock:
            self._check("set_packet_size")
            if not 576 <= int(size) <= 16384:
                raise _sim_error(f"{_OPERATIONS['set_packet_size']}: packet size {size} out of range (simulation)")
            self._packet_size = int(size)

    def configure(self, continuous: bool, num_buffers: int) -> None:
        with self._lock:
            self._check("configure")
            if self._acquiring:
                raise _sim_error(f"{_OPERATIONS['configure']}: acquisition in progress (simulation)")
            if num_buffers < 1:
                raise _sim_error(f"{_OPERATIONS['configure']}: invalid number of buffers {num_buffers} (simulation)")
            self._configured = True
            self._continuous = bool(continuous)
            self._num_buffers = int(num_buffers)
            self._ring.reset(self._num_buffers)

    def unconfigure(self) -> None:
        with self._lock:
            self._check("unconfigure")
            thread = self._halt_locked()
            self._configured = False
        self._join(thread)

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            thread = self._halt_locked()
            self._closed = True
            self._configured = False
            failure = self._method_failures.get("close")
        self._join(thread)
        self._ring.interrupt()
        if failure is not None:
            raise failure

    def start(self) -> None:
        with self._lock:
            self._check("start")
            if not self._configured:
                raise _sim_error(f"{_OPERATIONS['start']}: acquisition not configured (simulation)")
            if self._acquiring:
                return
            self._acquiring = True
            self._sequence_count = 0
            if not self._manual:
                self._grab_stop = threading.Event()
                self._grab = threading.Thread(target=self._grab_loop, args=(self._grab_stop,),
                                              name=f"aloxcam-simcam-{self.name}", daemon=True)
                self._grab.start()

    def stop(self) -> None:
        with self._lock:
            self._check("stop")
            thread = self._halt_locked()
        self._join(thread)

    def acquisition_in_progress(self) -> bool:
        with self._lock:
            self._check("acquisition_in_progress")
            return self._acquiring

    def last_buffer_number(self) -> int | None:
        with self._lock:
            self._check("last_buffer_number")
            return self._ring.last_number

    def buffer_count(self) -> int:
        with self._lock:
            self._check("buffer_count")
            return self._num_buffers if self._configured else 0

    # ------------------------------------------------------------------ CameraDevice: images
    def get_buffer(self, number: int | None, timeout_ms: int) -> Frame:
        self._before_get("get_buffer")
        return self._after_get(self._ring.get_buffer(number, timeout_ms / 1000.0, alive=self._alive), timeout_ms)

    def get_next(self, timeout_ms: int) -> Frame:
        self._before_get("get_next")
        return self._after_get(self._ring.get_next(timeout_ms / 1000.0, alive=self._alive), timeout_ms)

    def get_last_new(self, timeout_ms: int) -> Frame:
        self._before_get("get_last_new")
        return self._after_get(self._ring.get_last_new(timeout_ms / 1000.0, alive=self._alive), timeout_ms)

    # ------------------------------------------------------------------ CameraDevice: attributes
    def read_attribute(self, name: str, vtype: ValueType) -> float | int | bool | str:
        with self._lock:
            self._check("read_attribute")
            key, node = self._node(name)
            self._attribute_failure(key)
            if node.kind == "command":
                raise _sim_error(f"{name}: command attribute is not readable (simulation)")
            value = self.attribute_value(key)
            if node.kind in ("float", "int"):
                if vtype is ValueType.DBL:
                    return float(value)
                if vtype in (ValueType.I64, ValueType.U32):
                    n = round(float(value))
                    return max(n, 0) if vtype is ValueType.U32 else n
            elif node.kind == "bool" and vtype is ValueType.BOOL:
                return bool(value)
            elif node.kind == "str" and vtype is ValueType.STRING:
                return str(value)
            raise self._wrong_type(name, node, vtype)

    def write_attribute(self, name: str, vtype: ValueType, value: float | int | bool | str) -> None:
        with self._lock:
            self._check("write_attribute")
            key, node = self._node(name)
            self._attribute_failure(key)
            if node.readonly:
                raise _sim_error(f"{name}: read-only attribute (simulation)")
            if node.kind == "command":
                if vtype is not ValueType.BOOL or not isinstance(value, (bool, np.bool_)):
                    raise self._wrong_type(name, node, vtype)
                if value:
                    self._execute(key)
                return
            self._store(key, node, self._coerce(name, node, vtype, value))

    def read_attribute_max(self, name: str) -> float:
        with self._lock:
            self._check("read_attribute_max")
            key, node = self._node(name)
            self._attribute_failure(key)
            if node.kind not in ("float", "int") or node.maximum is None:
                raise _sim_error(f"{name}: attribute has no maximum (simulation)")
            return float(node.maximum)

    # ------------------------------------------------------------------ internals (lock held unless stated)
    def _check(self, method: str) -> None:
        if self._closed:
            raise CameraError(Code.IMAQDX_INVALID_SESSION, f"{_OPERATIONS[method]}: invalid camera session")
        failure = self._method_failures.get(method)
        if failure is not None:
            raise failure

    def _alive(self) -> bool:                                     # lock-free: called under the ring's lock
        return self._acquiring and not self._closed

    def _before_get(self, method: str) -> None:
        with self._lock:
            self._check(method)
            if not self._acquiring:
                raise _sim_error(f"{_OPERATIONS[method]}: acquisition not in progress")

    def _after_get(self, frame: Frame | None, timeout_ms: int) -> Frame:   # without the lock
        if frame is not None:
            return frame
        if not self._alive():
            raise _sim_error("IMAQdx Get Image2: acquisition stopped while waiting (simulation)")
        raise _sim_error(f"IMAQdx Get Image2: timeout after {timeout_ms} ms (simulation)")

    def _node(self, name: str) -> tuple[str, _Node]:
        key = _ALIASES.get(name, name)
        node = self._nodes.get(key)
        if node is None:
            raise _sim_error(f"{name}: unknown attribute (simulation)")
        return key, node

    def _attribute_failure(self, key: str) -> None:
        failure = self._attribute_failures.get(key)
        if failure is not None:
            raise failure

    @staticmethod
    def _wrong_type(name: str, node: _Node, vtype: ValueType) -> CameraError:
        return _sim_error(f"{name}: Value{vtype.value} does not match a {node.kind} attribute (simulation)")

    def _coerce(self, name: str, node: _Node, vtype: ValueType, value: Any) -> Any:
        if node.kind in ("float", "int"):
            if vtype not in _NUMERIC or isinstance(value, (bool, np.bool_, str)):
                raise self._wrong_type(name, node, vtype)
            x = float(value)
            if not math.isfinite(x):
                raise _sim_error(f"{name}: invalid value {x} (simulation)")
            v: Any = round(x) if node.kind == "int" else x
            if (node.minimum is not None and v < node.minimum) or (node.maximum is not None and v > node.maximum):
                raise _sim_error(f"{name}: value {v} out of range [{node.minimum}, {node.maximum}] (simulation)")
            return v
        if node.kind == "bool":
            if vtype is not ValueType.BOOL or not isinstance(value, (bool, np.bool_)):
                raise self._wrong_type(name, node, vtype)
            return bool(value)
        if vtype is not ValueType.STRING or not isinstance(value, str):
            raise self._wrong_type(name, node, vtype)
        if node.choices and value not in node.choices:
            raise _sim_error(f"{name}: invalid value {value!r} (simulation; one of {', '.join(node.choices)})")
        return value

    def _store(self, key: str, node: _Node, value: Any) -> None:
        if node.selector is None:
            node.value = value
        else:
            self._indexed[(key, int(self._nodes[node.selector].value))] = value

    def _execute(self, key: str) -> None:
        if key == TRIGGER_SOFTWARE and self._acquiring and self._nodes[TRIGGER_MODE].value == "On":
            self._produce_locked()

    def _may_produce(self) -> bool:
        return (self._acquiring and not self._closed
                and (self._continuous or self._sequence_count < self._num_buffers))

    def _produce_locked(self) -> Frame | None:
        if not self._may_produce():
            return None
        frame = self._ring.push(self._pixels(self._ring.frames_acquired), self._clock())
        self._sequence_count += 1
        return frame

    def _pixels(self, number: int) -> np.ndarray:
        exposure = float(self._nodes[SIM_EXPOSURE].value)
        if self._gradient is None or self._gradient[0] != exposure:
            top = float(np.iinfo(self._dtype).max)
            width = max(1, self._options.image_width)
            ramp = np.linspace(0.0, top, width) * (exposure / _EXPOSURE_REFERENCE)
            self._gradient = (exposure, np.rint(np.clip(ramp, 0.0, top)).astype(self._dtype))
        row = self._gradient[1]
        img = np.repeat(row[np.newaxis, :], max(1, self._options.image_height), axis=0)
        n = number & U32_MASK
        for i in range(min(4, img.shape[1])):
            img[0, i] = (n >> (8 * (3 - i))) & 0xFF
        return img

    def _halt_locked(self) -> threading.Thread | None:
        """Stop the acquisition (no join: the grab thread needs the lock to finish its pass)."""
        if not self._acquiring:
            return None
        self._acquiring = False
        self._grab_stop.set()
        thread, self._grab = self._grab, None
        self._ring.interrupt()
        return thread

    @staticmethod
    def _join(thread: threading.Thread | None) -> None:                 # without the lock
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)

    def _grab_loop(self, stop: threading.Event) -> None:
        while not stop.is_set():
            with self._lock:
                if not self._acquiring:
                    return
                rate = float(self._nodes[SIM_FRAME_RATE].value)
                if self._nodes[TRIGGER_MODE].value == "Off":
                    self._produce_locked()
            stop.wait(1.0 / rate if rate > 0 else 0.1)


class SimCameraBackend(CameraBackend):
    """Simulated NI-IMAQdx driver (design §3.6.2). Thread-safe."""

    def __init__(self, options: SimulationOptions, *, clock: Callable[[], float] = time.monotonic,
                 manual: bool = False) -> None:
        self.options = options
        self._clock = clock
        self._manual = manual
        self._lock = threading.Lock()
        self._devices: dict[str, SimCameraDevice] = {}
        self._closed = False
        if options.pixel_type.strip().upper() not in ("U8", "U16"):
            log.warning("[simulation] pixel_type %r is not U8 or U16: U8 is used", options.pixel_type)

    @property
    def devices(self) -> Mapping[str, SimCameraDevice]:
        """The devices opened so far, by camera name (the last one per name; closed ones included)."""
        with self._lock:
            return MappingProxyType(dict(self._devices))

    @property
    def closed(self) -> bool:
        """TRUE once :meth:`close` ran."""
        return self._closed

    def open_camera(self, name: str) -> SimCameraDevice:
        """``IMAQdx Open Camera``: ``CameraError(9502, …)`` for a name outside ``[simulation] camera_names`` (when
        set), a camera that is already open, or a released driver."""
        with self._lock:
            if self._closed:
                raise _sim_error("IMAQdx Open Camera: camera driver released (simulation)")
            if self.options.camera_names and name not in self.options.camera_names:
                raise _sim_error(f"IMAQdx Open Camera: camera '{name}' not found (simulation)")
            old = self._devices.get(name)
            if old is not None and not old.closed:
                raise _sim_error(f"IMAQdx Open Camera: camera '{name}' is already open (simulation)")
            device = SimCameraDevice(name, self.options, clock=self._clock, manual=self._manual)
            self._devices[name] = device
            return device

    def close(self) -> None:
        """Close every open device; idempotent."""
        with self._lock:
            self._closed = True
            devices = list(self._devices.values())
        for device in devices:
            try:
                device.close()
            except CameraError as exc:
                log.warning("closing simulated camera %r failed: %s", device.name, exc)
