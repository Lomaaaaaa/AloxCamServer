"""aloxcam.camera.handlers: every camera-worker command with the recording doubles and a manual ``SimCameraBackend``
(03 §5–§6, 05, 06 §3; design §3.6.7, §4.2)."""

from __future__ import annotations

import dataclasses
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from aloxcam.camera.acquisition import AcqState
from aloxcam.camera.attributes import (
    DEVICE_TEMPERATURE,
    LINE_INVERTER,
    LINE_MODE,
    LINE_SOURCE,
    TRIGGER_MODE,
    USER_OUTPUT_VALUE,
)
from aloxcam.camera.backend import ValueType
from aloxcam.camera.handlers import CAMERA_HANDLERS, NOT_IMPLEMENTED_TEXT, CameraContext, close_cameras
from aloxcam.camera.pipeline import CUSTOM_DATA_KEYS
from aloxcam.camera.session import make_sessions
from aloxcam.camera.sim import SIM_EXPOSURE, SIM_FRAME_RATE, SIM_GAIN, SimCameraBackend, SimCameraDevice
from aloxcam.dispatch import REQUEUE, STOP, ReplyKind, Request
from aloxcam.errors import NO_ERROR, CameraError, Code, ErrorCluster
from aloxcam.imagedata import ImageData
from aloxcam.imaging.png import write_png
from aloxcam.lv.path import LVPath
from aloxcam.settings import (
    HIGH_MAG,
    LOW_MAG,
    PLUG_USB,
    AloxSettings,
    CameraOptions,
    CompatSwitches,
    LightSettings,
    SimulationOptions,
)
from aloxcam.state import ServerState
from aloxcam.testing import FixedClock, RecordingEvents, RecordingReplies, make_request
from aloxcam.wire.messages import (
    CASES,
    CamLogData,
    CameraAutofocusData,
    CameraFlipData,
    CameraGainData,
    CameraGetAutofocusData,
    CameraGetExposureData,
    CameraGetGainData,
    CameraGetImageData,
    CameraGetLedData,
    CameraInitData,
    CameraRefocusData,
    CameraReleaseData,
    CameraSetExposureData,
    CameraSetFlatFieldData,
    CameraSetLedData,
    CameraStartData,
    CameraStatusData,
    CameraStopData,
    CameraStreamData,
    CameraTriggerSoftData,
    ShowUIData,
)

NOW = 1_700_000_000.5
SIM = SimulationOptions(image_width=8, image_height=6, max_frame_rate=100.0, temperature_c=36.5)
INVALID_SESSION = Code.IMAQDX_INVALID_SESSION


@dataclass
class Rig:
    ctx: CameraContext
    backend: SimCameraBackend
    replies: RecordingReplies
    events: RecordingEvents
    clock: FixedClock
    sleeps: list[float] = field(default_factory=list)
    forwarded: list[Request] = field(default_factory=list)

    def run(self, command: str, data: Any = None) -> tuple[Any, Request]:
        req = make_request(command, data)
        return CAMERA_HANDLERS[command](self.ctx, req), req

    def device(self, camera_type: str = HIGH_MAG) -> SimCameraDevice:
        dev = self.ctx.sessions[camera_type].device
        assert isinstance(dev, SimCameraDevice)
        return dev

    def setup(self, camera_type: str = HIGH_MAG, name: str | None = None, *, continuous: bool = True,
              start: bool = False) -> SimCameraDevice:
        name = name or ("cam0" if camera_type == HIGH_MAG else "cam1")
        self.run("Cam Setup", CameraInitData(camera_type, name, 5, continuous))
        if start:
            self.run("Cam Start", CameraStartData(camera_type, False))
        self.replies.clear()
        self.events.clear()
        return self.device(camera_type)


def make_rig(*, compat: CompatSwitches | None = None, alox: AloxSettings | None = None,
             camera_options: CameraOptions | None = None, sim: SimulationOptions = SIM) -> Rig:
    compat = compat or CompatSwitches()
    clock = FixedClock(1000)
    replies = RecordingReplies(send_error_replies=compat.send_error_replies)
    events = RecordingEvents()
    backend = SimCameraBackend(sim, manual=True)
    sessions = make_sessions(compat=compat, tick=clock, now=lambda: NOW)
    rig = Rig(ctx=None, backend=backend, replies=replies, events=events, clock=clock)   # type: ignore[arg-type]
    rig.ctx = CameraContext(state=ServerState.from_settings(alox or AloxSettings()), replies=replies, events=events,
                            compat=compat, tick=clock, backend=backend, sessions=sessions,
                            forward_to_z=rig.forwarded.append, camera_options=camera_options or CameraOptions(),
                            sleep=rig.sleeps.append)
    return rig


def with_settings(camera_type: str, **changes: Any) -> AloxSettings:
    base = AloxSettings()
    if camera_type == HIGH_MAG:
        return dataclasses.replace(base, high_mag=dataclasses.replace(base.high_mag, **changes))
    return dataclasses.replace(base, low_mag=dataclasses.replace(base.low_mag, **changes))


@pytest.fixture
def rig() -> Rig:
    return make_rig()


# ---------------------------------------------------------------------------------------------- table, general
def test_handler_table_covers_every_camera_case() -> None:
    assert set(CAMERA_HANDLERS) == {name for name, worker in CASES.items() if worker == "camera"}
    assert "Cam Autofocus Run" not in CAMERA_HANDLERS and "Cam Refocus Run" not in CAMERA_HANDLERS


def test_get_heartbeat(rig: Rig) -> None:
    rig.clock.set(123456)
    outcome, _ = rig.run("Get Heartbeat")
    assert outcome is None
    last = rig.replies.last
    assert last is not None and (last.kind, last.value, last.err, last.sent) == (ReplyKind.U32, 123456, NO_ERROR, True)
    assert rig.events.events == []


def test_show_ui_and_log(rig: Rig) -> None:
    rig.run("Show UI", ShowUIData(True))
    assert rig.ctx.state.show_ui is True
    assert rig.replies.last.kind is ReplyKind.ACK and rig.replies.last.err == NO_ERROR   # type: ignore[union-attr]
    log_data = CamLogData(log=True, max_lines=10)
    rig.run("Log", log_data)
    assert rig.ctx.state.cam_log_data == log_data
    assert rig.replies.last.sent                                # type: ignore[union-attr]


def test_exit_forwards_an_internal_exit(rig: Rig) -> None:
    outcome, req = rig.run("Exit")
    assert outcome is STOP
    assert rig.replies.replies == [] and rig.events.events == [] and not req.replied
    (fwd,) = rig.forwarded
    assert (fwd.command, fwd.code, fwd.conn_id, fwd.internal) == ("Exit", -1, 0, True)


def test_unknown_camera_type_label_is_rejected_as_undecodable(rig: Rig) -> None:
    with pytest.raises(CameraError) as info:
        rig.run("Cam Stop", CameraStopData("Medium Mag."))
    assert info.value.code == Code.VARIANT_TYPE_MISMATCH


# ---------------------------------------------------------------------------------------------- Cam Setup
def test_cam_setup_high_mag(rig: Rig) -> None:
    rig.clock.set(4321)
    rig.run("Cam Setup", CameraInitData(HIGH_MAG, "cam0", 5, True))
    last = rig.replies.last
    assert last is not None and (last.kind, last.err, last.sent) == (ReplyKind.ACK, NO_ERROR, True)
    dev = rig.device()
    assert dev.configured and dev.continuous and dev.buffer_count() == 5 and dev.packet_size == 8996
    state = rig.ctx.state
    assert state.inited[HIGH_MAG] and state.acq_state[HIGH_MAG] == "Stopped" and state.bus_type[HIGH_MAG] == "Simulator"
    assert (state.serial_high, state.serial_low) == dev.serial_number()
    session = rig.ctx.sessions[HIGH_MAG]
    assert session.acquisition_state is AcqState.STOPPED and session.simulation_time == 4321
    assert rig.events.events == []


def test_cam_setup_buffer_count_warning(rig: Rig, caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.WARNING, logger="aloxcam.camera.handlers"):
        rig.run("Cam Setup", CameraInitData(HIGH_MAG, "cam0", 7, True))
    assert any("Number of buffers = 7" in r.getMessage() for r in caplog.records)
    assert rig.device().buffer_count() == 5                     # always 5 (05 #8)
    caplog.clear()
    with caplog.at_level(logging.WARNING, logger="aloxcam.camera.handlers"):
        rig.run("Cam Setup", CameraInitData(LOW_MAG, "cam1", 0, False))
        rig.run("Cam Setup", CameraInitData(LOW_MAG, "cam1", 5, False))
    assert not any("Number of buffers" in r.getMessage() for r in caplog.records)
    assert not rig.device(LOW_MAG).continuous


def test_cam_setup_reopen_closes_the_previous_session(rig: Rig) -> None:
    first = rig.setup(HIGH_MAG, "cam0", start=True)
    rig.run("Cam Setup", CameraInitData(HIGH_MAG, "cam0", 5, True))
    assert first.closed                                         # 05 #9 fix: no "already open" failure
    assert rig.replies.last.err == NO_ERROR                     # type: ignore[union-attr]
    assert rig.device() is not first


def test_cam_setup_open_failure_suffixes() -> None:
    rig = make_rig(sim=dataclasses.replace(SIM, camera_names=("cam0",)))
    s = "IMAQdx Open Camera: camera 'nope' not found (simulation)"
    rig.run("Cam Setup", CameraInitData(HIGH_MAG, "nope", 5, True))
    assert rig.replies.last.sent is True                        # type: ignore[union-attr]  (error ACK sent, as LabVIEW)
    (ev,) = rig.events.events
    assert ev.err == ErrorCluster(True, Code.CAMERA_SDK, s + "Cam Setup")
    assert (ev.extra, ev.command, ev.camera_type) == ("Setup", "Cam Setup", HIGH_MAG)
    assert rig.ctx.state.inited[HIGH_MAG] is False and rig.ctx.sessions[HIGH_MAG].device is None
    rig.events.clear()
    rig.run("Cam Setup", CameraInitData(LOW_MAG, "nope", 5, True))
    assert rig.events.events[0].err.source == s + "Cam Setup" + s + s + s       # chained Configure, PacketSize, Open
    assert rig.events.events[0].camera_type == LOW_MAG


class _FailingBackend(SimCameraBackend):
    """Injects a method failure into every device it opens."""

    def __init__(self, method: str, error: CameraError) -> None:
        super().__init__(SIM, manual=True)
        self.method, self.error = method, error

    def open_camera(self, name: str) -> SimCameraDevice:
        dev = super().open_camera(name)
        dev.fail_method(self.method, self.error)
        return dev


def test_cam_setup_packet_size_failure_chains_to_configure() -> None:
    rig = make_rig()
    rig.ctx.backend = _FailingBackend("set_packet_size", CameraError(-1074360299, "pkt"))
    rig.run("Cam Setup", CameraInitData(LOW_MAG, "cam1", 5, True))
    (ev,) = rig.events.events
    assert ev.err == ErrorCluster(True, -1074360299, "pkt" + "Cam Setup" + "pkt" + "pkt")
    assert rig.ctx.state.inited[LOW_MAG] is False
    assert not rig.device(LOW_MAG).configured                   # Configure skipped (error-chained)


def test_cam_setup_configure_failure() -> None:
    rig = make_rig()
    rig.ctx.backend = _FailingBackend("configure", CameraError(-1074360298, "cfg"))
    rig.run("Cam Setup", CameraInitData(HIGH_MAG, "cam0", 5, True))
    assert rig.events.events[0].err.source == "cfgCam Setup"
    assert rig.ctx.state.inited[HIGH_MAG] is False
    assert rig.ctx.sessions[HIGH_MAG].device is not None        # kept so that Cam Cleanup can close it


def test_cam_setup_usb_plug_type() -> None:
    rig = make_rig(alox=with_settings(LOW_MAG, plug_type=PLUG_USB))
    rig.run("Cam Setup", CameraInitData(LOW_MAG, "cam1", 5, True))
    assert rig.sleeps == [0.1] and rig.device(LOW_MAG).packet_size == 0
    rig2 = make_rig(alox=with_settings(HIGH_MAG, plug_type=PLUG_USB))
    rig2.run("Cam Setup", CameraInitData(HIGH_MAG, "cam0", 5, True))
    assert rig2.sleeps == [] and rig2.device().packet_size == 0


def test_cam_setup_loads_flat_field_images(tmp_path: Path) -> None:
    write_png(tmp_path / "ffc.png", np.full((6, 8), 50, np.uint8))
    write_png(tmp_path / "dfc.png", np.full((6, 8), 5, np.uint8))
    (tmp_path / "bad.png").write_bytes(b"broken")
    rig = make_rig(alox=with_settings(HIGH_MAG, flat_field=True, flat_field_path=tmp_path / "ffc.png",
                                      dark_current_path=tmp_path / "missing.png"))
    rig.run("Cam Setup", CameraInitData(HIGH_MAG, "cam0", 5, True))
    session = rig.ctx.sessions[HIGH_MAG]
    assert session.ffc_image is not None and session.ffc_image.shape == (6, 8)
    assert session.dfc_image is None                            # missing file: null image, no error
    assert rig.replies.last.err == NO_ERROR                     # type: ignore[union-attr]
    rig = make_rig(alox=with_settings(HIGH_MAG, flat_field=True, flat_field_path=tmp_path / "bad.png",
                                      dark_current_path=tmp_path / "dfc.png"))
    rig.run("Cam Setup", CameraInitData(HIGH_MAG, "cam0", 5, True))
    (ev,) = rig.events.events
    assert ev.err.code == Code.IMAGE_FILE and ev.err.source.endswith("Cam Setup")
    assert rig.ctx.state.inited[HIGH_MAG] is True               # FFC/DFC errors do not affect Inited
    assert rig.ctx.sessions[HIGH_MAG].dfc_image is not None


# ---------------------------------------------------------------------------------------------- lifecycle
def test_cam_start_trigger_mode_and_suffix(rig: Rig) -> None:
    dev = rig.setup(HIGH_MAG)
    rig.run("Cam Start", CameraStartData(HIGH_MAG, True))
    assert dev.acquisition_in_progress() and dev.attribute_value(TRIGGER_MODE) == "On"
    assert rig.replies.last.err == NO_ERROR                     # type: ignore[union-attr]
    rig.run("Cam Start", CameraStartData(LOW_MAG, False))     # LM never set up
    (ev,) = rig.events.events
    assert ev.err == ErrorCluster(True, INVALID_SESSION,
                                  "IMAQdx Start Acquisition: invalid camera sessionCamStart TriggerModeLM")
    assert (ev.extra, ev.camera_type) == ("Start", LOW_MAG)


def test_cam_start_uses_the_camera_own_plug_type() -> None:
    alox = with_settings(HIGH_MAG, plug_type=PLUG_USB)
    alox = dataclasses.replace(alox, light=LightSettings(camera_sort_led="GiGe"))      # lights on a GigE camera
    rig = make_rig(alox=alox)
    dev = rig.setup(HIGH_MAG)
    rig.run("Cam Start", CameraStartData(HIGH_MAG, True))
    assert dev.acquisition_in_progress() and dev.attribute_value(TRIGGER_MODE) == "Off"   # USB: not written (05 #12)


def test_cam_stop_before_setup_and_after_start(rig: Rig) -> None:
    rig.run("Cam Stop", CameraStopData(HIGH_MAG))
    assert rig.replies.last.err == NO_ERROR and rig.replies.last.sent          # type: ignore[union-attr]
    dev = rig.setup(HIGH_MAG, start=True)
    rig.ctx.sessions[HIGH_MAG].acquisition_state = AcqState.ACQUIRE
    rig.run("Cam Stop", CameraStopData(HIGH_MAG))
    assert not dev.acquisition_in_progress()
    assert rig.ctx.sessions[HIGH_MAG].acquisition_state is AcqState.STOPPED       # 05 #11 fix
    dev.fail_method("stop", CameraError(-1, "stop failed"))
    dev.start()
    rig.run("Cam Stop", CameraStopData(HIGH_MAG))
    assert rig.events.events[-1].err.source == "stop failedCam stop"


def test_cam_cleanup(rig: Rig) -> None:
    dev = rig.setup(HIGH_MAG, start=True)
    session = rig.ctx.sessions[HIGH_MAG]
    session.focus.focus_progress = 50.0
    rig.run("Cam Cleanup", CameraReleaseData(HIGH_MAG))
    assert dev.closed and session.device is None and session.focus.focus_progress == 0.0
    assert session.acquisition_state is AcqState.START
    assert rig.ctx.state.inited[HIGH_MAG] is False
    assert rig.replies.last.err == NO_ERROR                     # type: ignore[union-attr]
    rig.run("Cam Cleanup", CameraReleaseData(HIGH_MAG))         # again: invalid session, error ACK suppressed
    assert rig.replies.last.sent is True                        # type: ignore[union-attr]
    assert rig.events.events[-1].err.code == INVALID_SESSION
    assert rig.events.events[-1].err.source.endswith("Cam Cleanup")


def test_get_acquisition_status(rig: Rig) -> None:
    rig.run("Cam Get Acquisition Status", CameraStatusData(HIGH_MAG))
    last = rig.replies.last
    assert last is not None and (last.kind, last.value, last.err) == (ReplyKind.BOOL, False, NO_ERROR)
    dev = rig.setup(HIGH_MAG, start=True)
    rig.run("Cam Get Acquisition Status", CameraStatusData(HIGH_MAG))
    assert rig.replies.last.value is True                       # type: ignore[union-attr]
    dev.fail_method("acquisition_in_progress", CameraError(-5, "boom"))
    rig.run("Cam Get Acquisition Status", CameraStatusData(HIGH_MAG))
    last = rig.replies.last
    assert last is not None and last.value is False
    assert last.err == ErrorCluster(True, -5, "boomCam Acquisition status")      # order A: decorated reply


def test_software_trigger_always_ok(rig: Rig) -> None:
    rig.run("Cam Software Trigger", CameraTriggerSoftData(HIGH_MAG))              # not set up
    assert rig.replies.last.err == NO_ERROR and rig.replies.last.sent            # type: ignore[union-attr]
    dev = rig.setup(HIGH_MAG)
    rig.run("Cam Start", CameraStartData(HIGH_MAG, True))
    rig.run("Cam Software Trigger", CameraTriggerSoftData(HIGH_MAG))
    assert dev.last_buffer_number() == 0
    rig.events.clear()
    usb = make_rig(alox=with_settings(HIGH_MAG, plug_type=PLUG_USB))
    dev2 = usb.setup(HIGH_MAG)
    dev2.write_attribute(TRIGGER_MODE, ValueType.STRING, "On")
    dev2.start()
    usb.run("Cam Software Trigger", CameraTriggerSoftData(HIGH_MAG))
    assert dev2.last_buffer_number() is None                    # USB: nothing sent
    assert rig.events.events == [] and usb.events.events == []
    assert usb.replies.last.err == NO_ERROR                     # type: ignore[union-attr]


# ---------------------------------------------------------------------------------------------- exposure, gain
def test_set_exposure_clamps_and_writes_max_framerate_on_hm(rig: Rig) -> None:
    dev = rig.setup(HIGH_MAG)
    dev.write_attribute(SIM_FRAME_RATE, ValueType.DBL, 10.0)
    rig.run("Cam Set Exposure", CameraSetExposureData(HIGH_MAG, 1500.0))
    assert rig.replies.last.err == NO_ERROR                     # type: ignore[union-attr]
    assert dev.attribute_value(SIM_EXPOSURE) == 1500.0
    assert dev.attribute_value(SIM_FRAME_RATE) == 100.0        # MaxDBL written (exposure_sets_max_framerate)
    assert rig.ctx.state.max_fr_real == 100
    assert rig.ctx.state.max_framerate_display == 0.0           # clamp(633.3, 0.5, HM Max Framerate 0) → 0
    rig.run("Cam Set Exposure", CameraSetExposureData(HIGH_MAG, 3_000_000.0))
    assert dev.attribute_value(SIM_EXPOSURE) == 2_000_000.0    # upper limit 2 000 000


def test_set_exposure_low_mag(rig: Rig) -> None:
    dev = rig.setup(LOW_MAG)
    rig.run("Cam Set Exposure", CameraSetExposureData(LOW_MAG, 100.0))
    assert dev.attribute_value(SIM_EXPOSURE) == 177.0          # lower limit (LimitMinExposure_DBL)
    assert dev.attribute_value(SIM_FRAME_RATE) == 10.0         # DBL frame-rate type: nothing written
    rig.run("Cam Get Exposure", CameraGetExposureData(LOW_MAG))
    assert rig.replies.last.value == 177.0                      # type: ignore[union-attr]
    lm_i64 = make_rig(alox=with_settings(LOW_MAG, framerate_type="I64"))
    dev = lm_i64.setup(LOW_MAG)
    lm_i64.run("Cam Set Exposure", CameraSetExposureData(LOW_MAG, 100.0))
    assert dev.attribute_value(SIM_FRAME_RATE) == 25.0         # clamp(5367, 0.5, 24.6) = 24.6 → I64 25


def test_set_exposure_fixed_framerate_behaviour() -> None:
    rig = make_rig(compat=CompatSwitches(exposure_sets_max_framerate=False))
    dev = rig.setup(HIGH_MAG)
    rig.run("Cam Set Exposure", CameraSetExposureData(HIGH_MAG, 100_000.0))
    assert dev.attribute_value(SIM_FRAME_RATE) == pytest.approx(9.5)             # 0.95 / 0.1 s, bound = MaxDBL
    assert rig.ctx.state.max_framerate_display == pytest.approx(9.5)
    rig.run("Cam Set Exposure", CameraSetExposureData(HIGH_MAG, 1000.0))
    assert dev.attribute_value(SIM_FRAME_RATE) == 100.0        # 950 → MaxDBL 100
    capped = make_rig(compat=CompatSwitches(exposure_sets_max_framerate=False),
                      camera_options=CameraOptions(hm_max_framerate=5.0))
    dev = capped.setup(HIGH_MAG)
    capped.run("Cam Set Exposure", CameraSetExposureData(HIGH_MAG, 1000.0))
    assert dev.attribute_value(SIM_FRAME_RATE) == 5.0          # bound = [camera] hm_max_framerate


def test_set_exposure_errors_replace_the_source() -> None:
    rig = make_rig(alox=with_settings(LOW_MAG, framerate_type="I64"))
    dev = rig.setup(LOW_MAG)
    dev.fail_attribute(SIM_EXPOSURE, CameraError(-1074360201, "driver text"))
    rig.run("Cam Set Exposure", CameraSetExposureData(LOW_MAG, 1000.0))
    (ev,) = rig.events.events
    assert ev.err == ErrorCluster(True, -1074360201, "CameraAttributes::Exposure::Value")   # replaced, no suffix
    assert rig.replies.last.sent is True                        # type: ignore[union-attr]
    rig.events.clear()
    dev.fail_attribute(SIM_EXPOSURE, None)
    dev.fail_attribute(SIM_FRAME_RATE, CameraError(-1074360202, "fr text"))
    rig.run("Cam Set Exposure", CameraSetExposureData(LOW_MAG, 1000.0))
    assert rig.events.events[0].err == ErrorCluster(
        True, -1074360202, "CameraAttributes::AcquisitionControl::AcquisitionFrameRate24.600000")


def test_set_exposure_non_finite_does_not_touch_the_camera(rig: Rig) -> None:
    dev = rig.setup(HIGH_MAG)
    rig.run("Cam Set Exposure", CameraSetExposureData(HIGH_MAG, float("nan")))
    assert rig.events.events[0].err == ErrorCluster(True, 1, "CameraAttributes::Exposure::Value: invalid value")
    assert dev.attribute_value(SIM_EXPOSURE) == 10000.0


def test_get_exposure_error_suffix(rig: Rig) -> None:
    rig.run("Cam Get Exposure", CameraGetExposureData(HIGH_MAG))
    last = rig.replies.last
    assert last is not None and last.kind is ReplyKind.DBL and last.value == 0.0 and last.sent
    assert last.err == ErrorCluster(True, INVALID_SESSION, "IMAQdx Property Node: invalid camera session"
                                    "CameraAttributes::Exposure::ValueDBL")
    assert rig.events.events[0].extra == "Get Exposure"


def test_set_and_get_gain_rounding(rig: Rig) -> None:
    dev = rig.setup(HIGH_MAG)
    rig.run("Cam Set Gain", CameraGainData(HIGH_MAG, 2.5))
    assert dev.attribute_value(SIM_GAIN) == 2.0                # I64: half to even
    rig.run("Cam Get Gain", CameraGetGainData(HIGH_MAG))
    assert rig.replies.last.value == 2.0                        # type: ignore[union-attr]
    rig.run("Cam Set Gain", CameraGainData(HIGH_MAG, 3.5))
    assert dev.attribute_value(SIM_GAIN) == 4.0
    rig.run("Cam Set Gain", CameraGainData(HIGH_MAG, 99.0))   # no clamp: the camera rejects it
    assert rig.events.events[-1].err.source.endswith("CameraAttributes::Gain::ValueI64")
    rig.run("Cam Set Gain", CameraGainData(HIGH_MAG, float("inf")))
    assert rig.events.events[-1].err == ErrorCluster(
        True, 1, "CameraAttributes::Gain::Value: invalid valueCameraAttributes::Gain::ValueI64")


# ---------------------------------------------------------------------------------------------- streaming
def stream(rig: Rig, data: CameraStreamData) -> tuple[Any, Request]:
    req = make_request("Cam Stream Next Image", data)
    return CAMERA_HANDLERS["Cam Stream Next Image"](rig.ctx, req), req


def test_stream_requeue_then_image_with_custom_data(rig: Rig) -> None:
    dev = rig.setup(HIGH_MAG, start=True)
    data = CameraStreamData(HIGH_MAG)
    req = make_request("Cam Stream Next Image", data)
    handler = CAMERA_HANDLERS["Cam Stream Next Image"]
    assert handler(rig.ctx, req) is REQUEUE
    assert rig.replies.replies == [] and rig.ctx.sessions[HIGH_MAG].acquisition_state is AcqState.ACQUIRE
    assert rig.ctx.state.acq_state[HIGH_MAG] == "Acquire" and rig.ctx.state.temperature[HIGH_MAG] == 36.5
    assert handler(rig.ctx, req) is REQUEUE                     # still no frame
    frame = dev.emit_frame()
    assert handler(rig.ctx, req) is None
    (reply,) = rig.replies.replies
    assert reply.kind is ReplyKind.IMAGE and reply.err == NO_ERROR and reply.req is req
    image = reply.value
    assert isinstance(image, ImageData) and np.array_equal(image.pixels, frame.pixels)
    assert tuple(image.custom_data) == CUSTOM_DATA_KEYS
    assert image.custom_data["type"] == "High-Mag" and image.custom_data["flatfield"] == "FALSE"
    assert image.custom_data["buffer"] == "0" and image.custom_data["simulation"] == "FALSE"
    session = rig.ctx.sessions[HIGH_MAG]
    assert session.acquisition_state is AcqState.STOPPED and session.display_image is session.src_image
    assert rig.ctx.state.acq_state[HIGH_MAG] == "Stopped" and rig.events.events == []


def test_stream_low_mag_type_and_timeout(rig: Rig) -> None:
    dev = rig.setup(LOW_MAG, start=True)
    outcome, req = stream(rig, CameraStreamData(LOW_MAG))
    assert outcome is REQUEUE
    dev.emit_frame()
    CAMERA_HANDLERS["Cam Stream Next Image"](rig.ctx, req)
    assert rig.replies.last.value.custom_data["type"] == "Low-Mag"                # type: ignore[union-attr]
    outcome, req = stream(rig, CameraStreamData(LOW_MAG, acquisition_timeout_ms=100))
    rig.clock.advance(101)
    assert CAMERA_HANDLERS["Cam Stream Next Image"](rig.ctx, req) is None
    last = rig.replies.last
    assert last is not None and last.value is None and last.err.code == Code.GET_IMAGE_TIMEOUT
    (ev,) = rig.events.events
    assert (ev.extra, ev.camera_type, ev.command) == ("Get Image Acquisition", LOW_MAG, "Cam Stream Next Image")


def test_stream_on_a_camera_that_is_not_set_up(rig: Rig) -> None:
    outcome, _ = stream(rig, CameraStreamData(HIGH_MAG))
    assert outcome is None
    last = rig.replies.last
    assert last is not None and last.kind is ReplyKind.IMAGE and last.value is None and last.sent
    assert last.err == ErrorCluster(True, INVALID_SESSION, "IMAQdx Property Node: invalid camera session"
                                    + DEVICE_TEMPERATURE)
    assert rig.events.events[0].extra == "Get Image Start"


@pytest.mark.parametrize("fails_image", [True, False])
def test_stream_temperature_failure(fails_image: bool, caplog: pytest.LogCaptureFixture) -> None:
    rig = make_rig(compat=CompatSwitches(temperature_error_fails_image=fails_image))
    dev = rig.setup(HIGH_MAG, start=True)
    dev.fail_attribute(DEVICE_TEMPERATURE, CameraError(-1074360100, "no temp"))
    with caplog.at_level(logging.WARNING, logger="aloxcam.camera.handlers"):
        outcome, req = stream(rig, CameraStreamData(HIGH_MAG))
    if fails_image:
        assert outcome is None
        last = rig.replies.last
        assert last is not None and last.value is None
        assert last.err == ErrorCluster(True, -1074360100, "no temp" + DEVICE_TEMPERATURE)
    else:
        assert outcome is REQUEUE
        assert any("temperature" in r.getMessage() for r in caplog.records)
        dev.emit_frame()
        CAMERA_HANDLERS["Cam Stream Next Image"](rig.ctx, req)
        assert rig.replies.last.err == NO_ERROR                 # type: ignore[union-attr]
        assert isinstance(rig.replies.last.value, ImageData)    # type: ignore[union-attr]


def test_stream_simulation_mode(rig: Rig, tmp_path: Path) -> None:
    write_png(tmp_path / "a.png", np.full((3, 5), 7, np.uint8))
    write_png(tmp_path / "b.png", np.full((2, 2), 9, np.uint8))
    rig.setup(LOW_MAG, start=True)
    data = CameraStreamData(LOW_MAG, simulation=True, simulation_path=LVPath.from_local(tmp_path))
    outcome, req = stream(rig, data)
    assert outcome is REQUEUE                                   # Start never returns Finished
    assert CAMERA_HANDLERS["Cam Stream Next Image"](rig.ctx, req) is None
    image = rig.replies.last.value                              # type: ignore[union-attr]
    assert image.pixels.shape == (3, 5) and image.custom_data == {}
    outcome, req = stream(rig, data)
    CAMERA_HANDLERS["Cam Stream Next Image"](rig.ctx, req)
    assert rig.replies.last.value.pixels.shape == (2, 2)       # type: ignore[union-attr]


def test_stream_after_cleanup_gives_invalid_session(rig: Rig) -> None:
    rig.setup(HIGH_MAG, start=True)
    rig.run("Cam Cleanup", CameraReleaseData(HIGH_MAG))
    stream(rig, CameraStreamData(HIGH_MAG))
    assert rig.replies.last.err.code == INVALID_SESSION        # type: ignore[union-attr]


def test_cam_get_image(rig: Rig) -> None:
    _, req = rig.run("Cam Get Image", CameraGetImageData(HIGH_MAG))              # not set up: error cleared
    assert rig.replies.replies == [] and rig.events.events == [] and not req.replied
    assert rig.ctx.state.error_out == NO_ERROR
    dev = rig.setup(HIGH_MAG)
    rig.run("Cam Get Image", CameraGetImageData(HIGH_MAG))    # not acquiring
    assert rig.ctx.state.error_out == ErrorCluster(
        True, Code.CAMERA_SDK, "IMAQdx Get Image2: acquisition not in progressCam Get Image")
    assert rig.replies.replies == [] and rig.events.events == []
    rig.ctx.state.replace_camera_settings(HIGH_MAG, vertical_flip=True)
    dev.start()
    frame = dev.emit_frame()
    rig.run("Cam Get Image", CameraGetImageData(HIGH_MAG))
    session = rig.ctx.sessions[HIGH_MAG]
    assert np.array_equal(session.display_image, frame.pixels[:, ::-1])
    assert rig.ctx.state.error_out == NO_ERROR
    session.simulation = True
    session.display_image = None
    dev.emit_frame()
    rig.run("Cam Get Image", CameraGetImageData(HIGH_MAG))    # simulation: nothing
    assert session.display_image is None


def test_cam_get_images_array_does_nothing(rig: Rig) -> None:
    outcome, req = rig.run("Cam Get Images Array")
    assert outcome is None and not req.replied and rig.replies.replies == [] and rig.events.events == []


# ---------------------------------------------------------------------------------------------- flip, flat field
def test_cam_flip(rig: Rig) -> None:
    rig.run("Cam Flip", CameraFlipData(LOW_MAG, False, True))
    lm = rig.ctx.state.camera_settings(LOW_MAG)
    assert (lm.horizontal_flip, lm.vertical_flip) == (False, True)
    assert rig.ctx.state.camera_settings(HIGH_MAG).vertical_flip is False
    assert rig.replies.last.err == NO_ERROR                     # type: ignore[union-attr]


def test_set_flatfield_loads_images(rig: Rig, tmp_path: Path) -> None:
    write_png(tmp_path / "ffc.png", np.full((6, 8), 40, np.uint8))
    write_png(tmp_path / "dfc.png", np.full((6, 8), 4, np.uint8))
    dev = rig.setup(HIGH_MAG, start=True)
    session = rig.ctx.sessions[HIGH_MAG]
    session.src_image = np.zeros((1, 1), np.uint8)
    rig.run("Cam Set FlatField", CameraSetFlatFieldData(HIGH_MAG, True, LVPath.from_local(tmp_path / "ffc.png"),
                                                        LVPath.from_local(tmp_path / "dfc.png"), 1.0))
    settings = rig.ctx.state.camera_settings(HIGH_MAG)
    assert settings.flat_field and settings.flat_field_path == tmp_path / "ffc.png"
    assert settings.dark_current_path == tmp_path / "dfc.png" and settings.flat_field_factor == 1.0
    assert session.src_image is None and session.ffc_image is not None and session.dfc_image is not None
    assert rig.replies.last.err == NO_ERROR                     # type: ignore[union-attr]
    outcome, req = stream(rig, CameraStreamData(HIGH_MAG))
    dev.emit_frame()
    CAMERA_HANDLERS["Cam Stream Next Image"](rig.ctx, req)
    assert rig.replies.last.value.custom_data["flatfield"] == "TRUE"             # type: ignore[union-attr]
    rig.run("Cam Set FlatField", CameraSetFlatFieldData(HIGH_MAG, False, LVPath.from_local(tmp_path / "ffc.png")))
    assert session.ffc_image is None and session.dfc_image is None              # disabled: null images


@pytest.mark.parametrize("check", [False, True])
def test_set_flatfield_missing_file(check: bool, tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    rig = make_rig(compat=CompatSwitches(flatfield_check_on_set=check))
    write_png(tmp_path / "dfc.png", np.full((6, 8), 4, np.uint8))
    missing = tmp_path / "missing.png"
    with caplog.at_level(logging.WARNING, logger="aloxcam.camera.handlers"):
        rig.run("Cam Set FlatField", CameraSetFlatFieldData(HIGH_MAG, True, LVPath.from_local(missing),
                                                            LVPath.from_local(tmp_path / "dfc.png"), 1.0))
    assert any("does not exist" in r.getMessage() for r in caplog.records)      # always warned
    assert rig.ctx.state.camera_settings(HIGH_MAG).flat_field is True
    if check:
        (ev,) = rig.events.events
        assert ev.err == ErrorCluster(True, Code.IMAGE_FILE, f"{missing}: flat-field file not foundCam Set Field")
        assert rig.replies.last.sent is True                    # type: ignore[union-attr]
    else:
        assert rig.events.events == [] and rig.replies.last.err == NO_ERROR      # type: ignore[union-attr]


def test_set_flatfield_non_finite_factor(rig: Rig) -> None:
    rig.run("Cam Set FlatField", CameraSetFlatFieldData(HIGH_MAG, True, correction_factor=float("nan")))
    assert rig.events.events[0].err.source == "Correction Factor: invalid valueCam Set Field"
    assert rig.ctx.state.camera_settings(HIGH_MAG).flat_field is False          # unchanged


# ---------------------------------------------------------------------------------------------- LED
def test_set_led_gige_truth_table_and_wait() -> None:
    rig = make_rig(alox=with_settings(HIGH_MAG, wait_led_ms=50))
    dev = rig.setup(HIGH_MAG)
    rig.run("Cam Set Led", CameraSetLedData(HIGH_MAG, True))
    assert dev.attribute_value(USER_OUTPUT_VALUE, 0) is True    # LEDStatusInverter TRUE: value = ledcmd
    assert dev.attribute_value(LINE_SOURCE, 1) == 1
    assert rig.sleeps == [0.05]                                 # output changed
    assert rig.ctx.state.led_cmd is True and rig.ctx.state.light_trace == "gi"
    assert rig.replies.last.err == NO_ERROR                     # type: ignore[union-attr]
    rig.run("Cam Set Led", CameraSetLedData(HIGH_MAG, True))
    assert rig.sleeps == [0.05]                                 # unchanged: no wait
    rig.run("Cam Get Led", CameraGetLedData(HIGH_MAG))
    last = rig.replies.last
    assert last is not None and (last.kind, last.value, last.err) == (ReplyKind.BOOL, True, NO_ERROR)
    assert rig.ctx.state.led_status is True


def test_led_invert_and_status_inverter() -> None:
    rig = make_rig()
    dev = rig.setup(LOW_MAG)                                    # LM Invert LED = TRUE
    rig.run("Cam Set Led", CameraSetLedData(LOW_MAG, True))
    assert rig.ctx.state.led_cmd is False and dev.attribute_value(USER_OUTPUT_VALUE, 0) is False
    rig.run("Cam Get Led", CameraGetLedData(LOW_MAG))
    assert rig.replies.last.value is True                       # XOR Invert LED
    alox = dataclasses.replace(AloxSettings(), light=LightSettings(led_status_inverter=False))
    rig2 = make_rig(alox=alox)
    dev2 = rig2.setup(HIGH_MAG)
    rig2.run("Cam Set Led", CameraSetLedData(HIGH_MAG, True))
    assert dev2.attribute_value(USER_OUTPUT_VALUE, 0) is False  # target = NOT ledcmd
    rig2.run("Cam Get Led", CameraGetLedData(HIGH_MAG))
    assert rig2.replies.last.value is False                     # type: ignore[union-attr]


def test_set_led_usb_lines() -> None:
    light = LightSettings(camera_sort_led=PLUG_USB, hm_led_line=2, hm_bulb_line=3, led_status_inverter=True,
                          bulb_status_inverter=False)
    rig = make_rig(alox=dataclasses.replace(with_settings(HIGH_MAG, wait_led_ms=20), light=light))
    dev = rig.setup(HIGH_MAG)
    rig.run("Cam Set Led", CameraSetLedData(HIGH_MAG, True))
    assert dev.attribute_value(LINE_INVERTER, 2) is True        # LED line: ledcmd (inverter TRUE)
    assert dev.attribute_value(LINE_INVERTER, 3) is False       # bulb line: NOT ledcmd (inverter FALSE)
    assert [dev.attribute_value(LINE_MODE, n) for n in (2, 3)] == [1, 1]
    assert [dev.attribute_value(LINE_SOURCE, n) for n in (2, 3)] == [1, 1]
    assert rig.sleeps == [] and rig.ctx.state.light_trace == "us"                 # switching on: no wait
    rig.run("Cam Set Led", CameraSetLedData(HIGH_MAG, False))
    assert rig.sleeps == [0.02]                                 # switching off: wait
    rig.run("Cam Get Led", CameraGetLedData(HIGH_MAG))
    assert dev.attribute_value(LINE_SOURCE, 2) == 0             # Cam Get Led writes LineSource = 0 (05 #23)
    assert rig.replies.last.value is False                      # type: ignore[union-attr]


def test_set_led_error_raw_reply_then_decorated_log() -> None:
    rig = make_rig(alox=with_settings(HIGH_MAG, wait_led_ms=10))
    rig.run("Cam Set Led", CameraSetLedData(HIGH_MAG, True))   # not set up
    last = rig.replies.last
    assert last is not None and last.sent is True                # the capture shows LabVIEW sending exactly this
    assert last.err == ErrorCluster(True, INVALID_SESSION, "IMAQdx Property Node: invalid camera session")
    (ev,) = rig.events.events
    assert ev.err.source == "IMAQdx Property Node: invalid camera sessionCam Set Led" and ev.extra == "Set LED"
    assert rig.sleeps == [0.01]                                 # the wait runs even after the I/O error
    rig.run("Cam Get Led", CameraGetLedData(HIGH_MAG))
    last = rig.replies.last
    assert last is not None and last.value is False and last.err.code == INVALID_SESSION and last.sent
    assert len(rig.events.events) == 1                          # typed reply: nothing more logged (variant D)


# ---------------------------------------------------------------------------------------------- focus shells
@pytest.mark.parametrize(("command", "payload", "suffix", "extra", "state_field"), [
    ("Cam Autofocus", CameraAutofocusData(LOW_MAG), "Cam Autofocus", "Autofocus", "af_state"),
    ("Cam Refocus", CameraRefocusData(LOW_MAG), "Cam Refocus", "Refocus", "rf_state"),
])
def test_focus_stubs_are_forced_9507(rig: Rig, command: str, payload: Any, suffix: str, extra: str,
                                     state_field: str) -> None:
    focus = rig.ctx.sessions[LOW_MAG].focus
    focus.last_focus_data = np.ones((3, 2))
    focus.focus_progress = 42.0
    rig.run(command, payload)
    expected = ErrorCluster(True, Code.NOT_IMPLEMENTED_IN_PHASE, NOT_IMPLEMENTED_TEXT + suffix)
    last = rig.replies.last
    assert last is not None and last.kind is ReplyKind.ACK and last.err == expected
    assert last.sent is True                                    # forced although send_error_replies = FALSE
    (ev,) = rig.events.events
    assert (ev.err, ev.extra, ev.camera_type, ev.command) == (expected, extra, LOW_MAG, command)
    assert getattr(focus, state_field) == "Error"
    assert focus.last_focus_data.shape == (0, 0) and focus.focus_progress == 0.0
    assert focus.last_error == expected


def test_get_autofocus_data_and_progress(rig: Rig) -> None:
    rig.run("Cam Get Autofocus Data", CameraGetAutofocusData(HIGH_MAG))
    last = rig.replies.last
    assert last is not None and last.kind is ReplyKind.DBL2D and last.value.shape == (0, 0) and last.sent
    rig.run("Cam Get Autofocus Progress", CameraGetAutofocusData(HIGH_MAG))
    assert (rig.replies.last.kind, rig.replies.last.value) == (ReplyKind.DBL, 0.0)  # type: ignore[union-attr]
    focus = rig.ctx.sessions[LOW_MAG].focus
    focus.last_focus_data = np.array([[1.0, 2.0], [1.1, 3.0]])
    focus.focus_progress = 100.0
    rig.run("Cam Get Autofocus Data", CameraGetAutofocusData(LOW_MAG))
    assert rig.replies.last.value.tolist() == [[1.0, 2.0], [1.1, 3.0]]          # type: ignore[union-attr]
    rig.run("Cam Get Autofocus Progress", CameraGetAutofocusData(LOW_MAG))
    assert rig.replies.last.value == 100.0                      # type: ignore[union-attr]
    assert rig.events.events == []


# ---------------------------------------------------------------------------------------------- context, shutdown
def test_make_sessions_are_independent() -> None:
    sessions = make_sessions(compat=CompatSwitches(request_buffer_zero=False, strict_new_frame=True),
                             tick=FixedClock())
    assert set(sessions) == {HIGH_MAG, LOW_MAG}
    hm, lm = sessions[HIGH_MAG], sessions[LOW_MAG]
    assert hm.state_machine is not lm.state_machine and hm.focus is not lm.focus
    assert hm.state_machine.request_buffer_zero is False and hm.state_machine.strict_new_frame is True
    with pytest.raises(CameraError) as info:
        hm.require_device("op")
    assert info.value.code == INVALID_SESSION and info.value.source == "op: invalid camera session"


def test_from_base_and_close_cameras(rig: Rig) -> None:
    ctx = CameraContext.from_base(rig.ctx, backend=rig.backend, sessions=rig.ctx.sessions,
                                  forward_to_z=rig.forwarded.append, camera_options=CameraOptions(),
                                  decimal_separator=",")
    assert ctx.state is rig.ctx.state and ctx.decimal_separator == "," and ctx.focus is None and ctx.stage is None
    hm = rig.setup(HIGH_MAG, start=True)
    lm = rig.setup(LOW_MAG)
    lm.fail_method("close", CameraError(-1, "close failed"))
    close_cameras(rig.ctx.sessions)
    assert hm.closed and lm.closed
    assert all(s.device is None for s in rig.ctx.sessions.values())
    close_cameras(rig.ctx.sessions)                             # nothing left: no error


def test_finish_records_error_out(rig: Rig) -> None:
    rig.run("Cam Cleanup", CameraReleaseData(LOW_MAG))
    assert rig.ctx.state.error_out.code == INVALID_SESSION
    rig.run("Get Heartbeat")
    assert rig.ctx.state.error_out == NO_ERROR
