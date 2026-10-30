"""``ServerState``: the front-panel objects of the main VI used as shared memory, behind one lock.

Spec: 01 §8 (front-panel state), §9.2 (unsynchronised shared memory, A16), §10.1 (one ``ServerState`` with an RLock),
01 A11–A13 (bounded, time-stamped histories); 03 §3.3 (``Commands History``).

Thread-safety: every method takes :attr:`ServerState.lock` (a leaf lock: nothing else is called while it is held).
Settings objects are immutable, so a reader holding a returned object never sees a torn value.
"""

from __future__ import annotations

import dataclasses
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

from aloxcam.errors import NO_ERROR, ErrorCluster
from aloxcam.settings import (
    CAMERA_TYPES,
    HIGH_MAG,
    LOW_MAG,
    AloxSettings,
    CameraSettings,
    LightSettings,
    ServerOptions,
    ZStageSettings,
)

__all__ = ["ServerState"]

_CAMERA_FIELD = {HIGH_MAG: "high_mag", LOW_MAG: "low_mag"}


@dataclass
class ServerState:
    """Front-panel objects used as shared memory, with one RLock (01 §8, A16). Readers get values or copies."""

    high_mag: CameraSettings
    low_mag: CameraSettings
    light: LightSettings
    z: ZStageSettings
    options: ServerOptions = field(default_factory=ServerOptions)    # history sizes
    # status (01 §8.3) — written through update()/helpers only
    connected: bool = False
    server_state: str = "Listen"
    heartbeat: int = 0                   # TCP-loop liveness tick (not the Get Heartbeat reply)
    last_command: str = ""
    error_out: ErrorCluster = NO_ERROR
    inited: dict[str, bool] = field(default_factory=lambda: dict.fromkeys(CAMERA_TYPES, False))
    acq_state: dict[str, str] = field(default_factory=lambda: dict.fromkeys(CAMERA_TYPES, "Start"))
    temperature: dict[str, float] = field(default_factory=lambda: dict.fromkeys(CAMERA_TYPES, 0.0))
    bus_type: dict[str, str] = field(default_factory=lambda: dict.fromkeys(CAMERA_TYPES, ""))
    serial_high: int = 0
    serial_low: int = 0
    max_framerate_display: float = 0.0
    max_fr_real: int = 0
    led_cmd: bool = False
    led_status: bool = False
    light_trace: str = ""
    show_ui: bool = False
    cam_log_data: object | None = None   # wire.messages.CamLogData (stored, never read: 03 C5)
    z_inited: bool = False
    z_pos: float = 0.0
    running_macros: str = ""
    macro_running: bool = False
    # histories: created in __post_init__ with maxlen from `options`
    commands_history: deque[str] = field(init=False, repr=False)                        # newest first
    error_history: deque[tuple[float, ErrorCluster]] = field(init=False, repr=False)
    temperature_history: deque[tuple[float, float, float]] = field(init=False, repr=False)   # (time, HM, LM)
    lock: threading.RLock = field(default_factory=threading.RLock, repr=False, compare=False)

    def __post_init__(self) -> None:
        self.commands_history = deque(maxlen=max(0, self.options.commands_history))
        self.error_history = deque(maxlen=max(0, self.options.error_history))
        self.temperature_history = deque(maxlen=max(0, self.options.temperature_history))

    @classmethod
    def from_settings(cls, alox: AloxSettings,
                      options: ServerOptions = ServerOptions()) -> ServerState:  # noqa: B008 (frozen)
        """Initial state from the loaded ``alox.ini`` settings (01 §3.4)."""
        return cls(high_mag=alox.high_mag, low_mag=alox.low_mag, light=alox.light, z=alox.z, options=options)

    # ------------------------------------------------------------------ settings
    def camera_settings(self, camera_type: str) -> CameraSettings:
        """``High Mag Settings`` / ``Low Mag Settings``; :class:`KeyError` for unknown camera types."""
        name = _CAMERA_FIELD[camera_type]
        with self.lock:
            settings: CameraSettings = getattr(self, name)
            return settings

    def replace_camera_settings(self, camera_type: str, **changes: Any) -> CameraSettings:
        """Replace the camera's settings object with a modified copy and return the new object (05 §7.2, §7.3)."""
        name = _CAMERA_FIELD[camera_type]
        with self.lock:
            new: CameraSettings = dataclasses.replace(getattr(self, name), **changes)
            setattr(self, name, new)
            return new

    def replace_z_settings(self, **changes: Any) -> ZStageSettings:
        """Replace ``Z Stage Settings`` with a modified copy and return it (``Z Set Min/Max Limit``, 03 §7.17)."""
        with self.lock:
            self.z = dataclasses.replace(self.z, **changes)
            return self.z

    # ------------------------------------------------------------------ status and histories
    def record_command(self, name: str) -> None:
        """Prepend to ``Commands History`` (newest first, bounded) and set ``last_command`` (03 §3.3)."""
        with self.lock:
            self.commands_history.appendleft(name)
            self.last_command = name

    def set_error_out(self, err: ErrorCluster) -> None:
        """Write the front-panel ``error out`` (03 §4.4)."""
        with self.lock:
            self.error_out = err

    def record_camera_iteration(self) -> None:
        """Append ``(time, error_out)`` and ``(time, HM °C, LM °C)`` samples (01 §5.3; bounded: A11/A12 fix)."""
        with self.lock:
            now = time.time()
            self.error_history.append((now, self.error_out))
            self.temperature_history.append((now, self.temperature[HIGH_MAG], self.temperature[LOW_MAG]))

    def update(self, **fields: Any) -> None:
        """Atomic multi-field write; :class:`AttributeError` on unknown names (nothing is written then)."""
        unknown = [name for name in fields if name not in _UPDATABLE]
        if unknown:
            raise AttributeError(f"ServerState has no updatable field(s) {', '.join(sorted(unknown))}")
        with self.lock:
            for name, value in fields.items():
                setattr(self, name, value)

    def update_camera(self, camera_type: str, *, inited: bool | None = None, acq_state: str | None = None,
                      temperature: float | None = None, bus_type: str | None = None) -> None:
        """Per-camera status indicators (01 §8.3); :class:`KeyError` for unknown camera types."""
        if camera_type not in _CAMERA_FIELD:
            raise KeyError(camera_type)
        with self.lock:
            if inited is not None:
                self.inited[camera_type] = inited
            if acq_state is not None:
                self.acq_state[camera_type] = acq_state
            if temperature is not None:
                self.temperature[camera_type] = temperature
            if bus_type is not None:
                self.bus_type[camera_type] = bus_type

    def snapshot(self) -> dict[str, Any]:
        """Copies of all fields (diagnostics, tests): dicts copied, histories as lists, no lock."""
        with self.lock:
            out: dict[str, Any] = {}
            for f in dataclasses.fields(self):
                if f.name == "lock":
                    continue
                value = getattr(self, f.name)
                if isinstance(value, dict):
                    value = dict(value)
                elif isinstance(value, deque):
                    value = list(value)
                out[f.name] = value
            return out


#: Fields ``update()`` may write: every constructor field except the lock (histories are not replaceable).
_UPDATABLE = frozenset(f.name for f in dataclasses.fields(ServerState) if f.init and f.name != "lock")
