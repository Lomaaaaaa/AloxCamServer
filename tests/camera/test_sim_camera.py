"""aloxcam.camera.sim (``manual=True`` unless stated) and the backend's ``FrameRing`` (05 §4, §6.4;
design §3.6.2)."""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest

from aloxcam.camera.attributes import (
    DEVICE_TEMPERATURE,
    LINE_INVERTER,
    LINE_SELECTOR,
    TRIGGER_MODE,
    TRIGGER_SOFTWARE,
    USER_OUTPUT_SELECTOR,
    USER_OUTPUT_VALUE,
)
from aloxcam.camera.backend import FrameRing, ValueType
from aloxcam.camera.sim import (
    SIM_EXPOSURE,
    SIM_EXPOSURE_TIME,
    SIM_FRAME_RATE,
    SIM_GAIN,
    SIM_GAIN_ANALOG,
    SimCameraBackend,
    SimCameraDevice,
    frame_number_of,
)
from aloxcam.errors import CameraError, Code
from aloxcam.settings import SimulationOptions

SMALL = SimulationOptions(image_width=8, image_height=6, max_frame_rate=100.0, frame_rate=10.0, temperature_c=41.5)


def opened(options: SimulationOptions = SMALL, *, continuous: bool = True, buffers: int = 5,
           start: bool = True) -> SimCameraDevice:
    dev = SimCameraBackend(options, manual=True).open_camera("cam0")
    dev.configure(continuous, buffers)
    if start:
        dev.start()
    return dev


def code_of(exc: pytest.ExceptionInfo[CameraError]) -> int:
    return exc.value.code


# ---------------------------------------------------------------------------------------------- backend
def test_unknown_camera_name() -> None:
    backend = SimCameraBackend(SimulationOptions(camera_names=("cam0", "cam1")), manual=True)
    with pytest.raises(CameraError) as info:
        backend.open_camera("cam7")
    assert info.value.code == Code.CAMERA_SDK
    assert info.value.source == "IMAQdx Open Camera: camera 'cam7' not found (simulation)"
    assert backend.open_camera("cam1").name == "cam1"


def test_any_name_opens_exclusively_and_reopens_after_close() -> None:
    backend = SimCameraBackend(SMALL, manual=True)
    dev = backend.open_camera("anything")
    with pytest.raises(CameraError, match="already open"):
        backend.open_camera("anything")
    dev.close()
    dev.close()                                                   # idempotent
    again = backend.open_camera("anything")
    assert again is not dev and backend.devices["anything"] is again


def test_backend_close_closes_devices_and_refuses_open() -> None:
    backend = SimCameraBackend(SMALL, manual=True)
    a, b = backend.open_camera("a"), backend.open_camera("b")
    a.configure(True, 5)
    a.start()
    backend.close()
    assert a.closed and b.closed and backend.closed
    with pytest.raises(CameraError, match="released"):
        backend.open_camera("c")
    backend.close()


def test_session_properties() -> None:
    dev = opened(start=False)
    assert dev.bus_type() == "Simulator"
    high, low = dev.serial_number()
    assert high == 0 and 0 <= low <= 0xFFFFFFFF
    dev.set_packet_size(8996)
    assert dev.packet_size == 8996
    with pytest.raises(CameraError):
        dev.set_packet_size(10)
    assert dev.buffer_count() == 5 and dev.configured and dev.continuous


def test_closed_device_is_an_invalid_session() -> None:
    dev = opened()
    dev.close()
    for call in (dev.bus_type, dev.start, dev.acquisition_in_progress, dev.last_buffer_number,
                 lambda: dev.read_attribute(SIM_EXPOSURE, ValueType.DBL), dev.emit_frame):
        with pytest.raises(CameraError) as info:
            call()
        assert info.value.code == Code.IMAQDX_INVALID_SESSION
        assert info.value.source.endswith("invalid camera session")


# ---------------------------------------------------------------------------------------------- acquisition
def test_numbering_from_zero_and_last_buffer_number() -> None:
    dev = opened()
    assert dev.last_buffer_number() is None
    frames = [dev.emit_frame() for _ in range(3)]
    assert [f.number for f in frames] == [0, 1, 2]
    assert [frame_number_of(f.pixels) for f in frames] == [0, 1, 2]
    assert dev.last_buffer_number() == 2
    assert frames[0].pixels.shape == (6, 8) and frames[0].pixels.dtype == np.uint8
    dev.stop()
    dev.configure(True, 5)                                      # numbering restarts after configure
    dev.start()
    assert dev.last_buffer_number() is None
    assert dev.emit_frame().number == 0


def test_configure_and_start_rules() -> None:
    dev = opened(start=False)
    dev.unconfigure()
    with pytest.raises(CameraError, match="not configured"):
        dev.start()
    dev.configure(True, 5)
    dev.start()
    dev.start()                                                 # already running: no-op
    with pytest.raises(CameraError, match="in progress"):
        dev.configure(True, 5)
    with pytest.raises(CameraError, match="not in progress"):
        opened(start=False).emit_frame()


def test_one_shot_stops_after_num_buffers() -> None:
    dev = opened(continuous=False)
    for _ in range(5):
        dev.emit_frame()
    with pytest.raises(CameraError, match="sequence of 5 frames complete"):
        dev.emit_frame()
    assert dev.last_buffer_number() == 4
    assert dev.acquisition_in_progress() is True
    dev.stop()
    assert dev.acquisition_in_progress() is False


def test_trigger_mode_one_frame_per_software_trigger() -> None:
    dev = opened()
    dev.write_attribute(TRIGGER_SOFTWARE, ValueType.BOOL, True)       # TriggerMode Off: no effect
    assert dev.last_buffer_number() is None
    dev.write_attribute(TRIGGER_MODE, ValueType.STRING, "On")
    dev.write_attribute(TRIGGER_SOFTWARE, ValueType.BOOL, True)
    dev.write_attribute(TRIGGER_SOFTWARE, ValueType.BOOL, True)
    assert dev.last_buffer_number() == 1
    dev.write_attribute(TRIGGER_SOFTWARE, ValueType.BOOL, False)      # FALSE does not execute
    assert dev.last_buffer_number() == 1
    with pytest.raises(CameraError):
        dev.read_attribute(TRIGGER_SOFTWARE, ValueType.BOOL)
    with pytest.raises(CameraError, match="invalid value"):
        dev.write_attribute(TRIGGER_MODE, ValueType.STRING, "Maybe")


def test_ring_overwrite_and_buffer_number_mode() -> None:
    dev = opened()
    for _ in range(3):
        dev.emit_frame()
    assert dev.get_buffer(0, 100).number == 0                  # buffer 0 still in the ring: the stale first frame
    for _ in range(4):
        dev.emit_frame()                                        # 0..6: ring holds 2..6
    assert dev.get_buffer(0, 100).number == 6                  # overwritten → newest ("get newest")
    assert dev.get_buffer(3, 100).number == 3
    assert dev.get_buffer(None, 100).number == 6
    frame = dev.get_buffer(4, 100)
    assert frame_number_of(frame.pixels) == 4
    frame.pixels[:] = 0                                         # returned frames are copies
    assert frame_number_of(dev.get_buffer(4, 100).pixels) == 4


def test_get_modes_wait_and_time_out() -> None:
    dev = opened()
    with pytest.raises(CameraError) as info:
        dev.get_buffer(None, 20)
    assert info.value.code == Code.CAMERA_SDK and "timeout" in info.value.source
    dev.emit_frame()
    dev.emit_frame()
    assert dev.get_last_new(20).number == 1
    with pytest.raises(CameraError, match="timeout"):
        dev.get_last_new(20)                                    # nothing new since
    dev.emit_frame()
    assert dev.get_last_new(20).number == 2
    with pytest.raises(CameraError, match="timeout"):
        dev.get_next(20)                                        # after buffer 2 was returned, 'Next' waits for 3
    dev.emit_frame()
    assert dev.get_next(20).number == 3


def test_get_next_sequence_from_zero() -> None:
    dev = opened()
    for _ in range(3):
        dev.emit_frame()
    assert [dev.get_next(20).number for _ in range(3)] == [0, 1, 2]


def test_waiting_get_returns_frame_emitted_by_another_thread() -> None:
    dev = opened()
    timer = threading.Timer(0.05, dev.emit_frame)
    timer.start()
    try:
        assert dev.get_last_new(2000).number == 0
    finally:
        timer.join()


def test_get_when_not_acquiring() -> None:
    dev = opened(start=False)
    for call in (lambda: dev.get_buffer(0, 10), lambda: dev.get_next(10), lambda: dev.get_last_new(10)):
        with pytest.raises(CameraError) as info:
            call()
        assert info.value.source == "IMAQdx Get Image2: acquisition not in progress"


def test_gradient_scaled_by_exposure_and_u16() -> None:
    dev = opened(SimulationOptions(image_width=11, image_height=3))
    full = dev.emit_frame().pixels
    assert full[1, -1] == 255 and full[1, 0] == 0
    dev.write_attribute(SIM_EXPOSURE, ValueType.DBL, 5000.0)
    half = dev.emit_frame().pixels
    assert half[1, -1] == 128                                   # rint(255 / 2)
    dev16 = opened(SimulationOptions(image_width=5, image_height=2, pixel_type="U16"))
    px = dev16.emit_frame().pixels
    assert px.dtype == np.uint16 and px[1, -1] == 65535


# ---------------------------------------------------------------------------------------------- attributes
def test_numeric_attributes_and_aliases() -> None:
    dev = opened()
    assert dev.read_attribute(SIM_EXPOSURE, ValueType.DBL) == 10000.0
    dev.write_attribute(SIM_EXPOSURE_TIME, ValueType.DBL, 1500.0)
    assert dev.read_attribute(SIM_EXPOSURE, ValueType.DBL) == 1500.0
    assert dev.read_attribute(SIM_EXPOSURE, ValueType.I64) == 1500
    dev.write_attribute(SIM_GAIN, ValueType.I64, 2)
    assert dev.read_attribute(SIM_GAIN_ANALOG, ValueType.DBL) == 2.0
    dev.write_attribute(SIM_GAIN, ValueType.U32, 3)
    assert dev.read_attribute(SIM_GAIN, ValueType.U32) == 3
    assert dev.read_attribute(DEVICE_TEMPERATURE, ValueType.DBL) == 41.5
    assert dev.read_attribute(SIM_FRAME_RATE, ValueType.DBL) == 10.0
    assert dev.read_attribute_max(SIM_FRAME_RATE) == 100.0
    dev.write_attribute(SIM_FRAME_RATE, ValueType.DBL, 100.0)


@pytest.mark.parametrize(("name", "vtype", "value", "match"), [
    (SIM_FRAME_RATE, ValueType.DBL, 100.5, "out of range"),
    (SIM_GAIN, ValueType.DBL, -1.0, "out of range"),
    (SIM_EXPOSURE, ValueType.STRING, "1500", "does not match"),
    (SIM_EXPOSURE, ValueType.BOOL, True, "does not match"),
    (SIM_EXPOSURE, ValueType.DBL, float("nan"), "invalid value"),
    (LINE_INVERTER, ValueType.DBL, 1.0, "does not match"),
    (TRIGGER_MODE, ValueType.U32, 1, "does not match"),
    (DEVICE_TEMPERATURE, ValueType.DBL, 20.0, "read-only"),
    ("CameraAttributes::Nope::Value", ValueType.DBL, 1.0, "unknown attribute"),
])
def test_attribute_write_errors(name: str, vtype: ValueType, value: object, match: str) -> None:
    dev = opened()
    with pytest.raises(CameraError, match=match) as info:
        dev.write_attribute(name, vtype, value)                 # type: ignore[arg-type]
    assert info.value.code == Code.CAMERA_SDK


def test_attribute_read_errors() -> None:
    dev = opened()
    with pytest.raises(CameraError, match="does not match"):
        dev.read_attribute(SIM_EXPOSURE, ValueType.STRING)
    with pytest.raises(CameraError, match="does not match"):
        dev.read_attribute(TRIGGER_MODE, ValueType.DBL)
    with pytest.raises(CameraError, match="no maximum"):
        dev.read_attribute_max(TRIGGER_MODE)
    with pytest.raises(CameraError, match="unknown attribute"):
        dev.read_attribute("CameraAttributes::Nope::Value", ValueType.DBL)


def test_digital_io_values_per_line_and_output() -> None:
    dev = opened()
    dev.write_attribute(LINE_SELECTOR, ValueType.U32, 2)
    dev.write_attribute(LINE_INVERTER, ValueType.BOOL, True)
    dev.write_attribute(LINE_SELECTOR, ValueType.U32, 3)
    assert dev.read_attribute(LINE_INVERTER, ValueType.BOOL) is False
    assert dev.attribute_value(LINE_INVERTER, 2) is True
    dev.write_attribute(USER_OUTPUT_SELECTOR, ValueType.U32, 1)
    dev.write_attribute(USER_OUTPUT_VALUE, ValueType.BOOL, True)
    dev.write_attribute(USER_OUTPUT_SELECTOR, ValueType.U32, 0)
    assert dev.read_attribute(USER_OUTPUT_VALUE, ValueType.BOOL) is False
    assert dev.attribute_value(USER_OUTPUT_VALUE, 1) is True
    with pytest.raises(CameraError, match="out of range"):
        dev.write_attribute(LINE_SELECTOR, ValueType.U32, 4)


def test_injected_failures() -> None:
    dev = opened()
    boom = CameraError(-1074360000, "injected")
    dev.fail_attribute(SIM_EXPOSURE_TIME, boom)                 # alias → same node
    with pytest.raises(CameraError) as info:
        dev.read_attribute(SIM_EXPOSURE, ValueType.DBL)
    assert info.value is boom
    with pytest.raises(CameraError):
        dev.write_attribute(SIM_EXPOSURE, ValueType.DBL, 2000.0)
    with pytest.raises(CameraError):
        dev.read_attribute_max(SIM_EXPOSURE)
    dev.fail_attribute(SIM_EXPOSURE, None)
    assert dev.read_attribute(SIM_EXPOSURE, ValueType.DBL) == 10000.0
    dev.fail_method("last_buffer_number", boom)
    with pytest.raises(CameraError):
        dev.last_buffer_number()
    dev.fail_method("last_buffer_number", None)
    assert dev.last_buffer_number() is None
    dev.fail_method("close", boom)
    with pytest.raises(CameraError):
        dev.close()
    assert dev.closed                                           # an injected close failure still closes
    with pytest.raises(ValueError):
        dev.fail_method("explode", boom)


# ---------------------------------------------------------------------------------------------- grab thread
def test_grab_thread_produces_frames_and_stops() -> None:
    backend = SimCameraBackend(SimulationOptions(image_width=4, image_height=2, frame_rate=200.0,
                                                 max_frame_rate=1000.0))
    dev = backend.open_camera("cam0")
    dev.configure(True, 5)
    dev.start()
    try:
        names = {t.name for t in threading.enumerate()}
        assert "aloxcam-simcam-cam0" in names
        frame = dev.get_buffer(2, 2000)                         # waits for buffer 2
        assert frame.number == 2
    finally:
        dev.stop()
    count = dev.frames_acquired
    time.sleep(0.03)
    assert dev.frames_acquired == count
    assert "aloxcam-simcam-cam0" not in {t.name for t in threading.enumerate()}
    dev.close()


def test_grab_thread_respects_trigger_mode_and_sequence() -> None:
    dev = SimCameraBackend(SimulationOptions(image_width=4, image_height=2, frame_rate=500.0,
                                             max_frame_rate=1000.0)).open_camera("cam0")
    dev.configure(False, 3)
    dev.start()
    try:
        deadline = time.monotonic() + 2.0
        while dev.frames_acquired < 3 and time.monotonic() < deadline:
            time.sleep(0.005)
        time.sleep(0.02)
        assert dev.frames_acquired == 3 and dev.acquisition_in_progress()
    finally:
        dev.close()


# ---------------------------------------------------------------------------------------------- FrameRing
def test_frame_ring_basics() -> None:
    ring = FrameRing(3)
    assert ring.last_number is None and ring.num_buffers == 3
    assert ring.get_buffer(None, 0.0) is None
    for i in range(4):
        ring.push(np.full((1, 1), i, np.uint8), float(i))
    assert ring.last_number == 3 and ring.frames_acquired == 4
    frame = ring.get_buffer(0, 0.0)
    assert frame is not None and frame.number == 3              # 0 overwritten → newest
    assert ring.get_buffer(1, 0.0).number == 1                  # type: ignore[union-attr]
    assert ring.get_buffer(9, 0.01) is None                     # future buffer: times out
    stopped = ring.get_last_new(5.0, alive=lambda: False)       # returns at once when not alive
    assert stopped is not None                                  # … unless a frame is ready
    ring.reset(2)
    assert ring.last_number is None and ring.num_buffers == 2
    assert ring.get_next(5.0, alive=lambda: False) is None
