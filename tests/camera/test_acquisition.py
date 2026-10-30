"""aloxcam.camera.acquisition: the per-camera ``Get Image`` state machine (05 §6.3–§6.4, §12.7; design §3.6.5)."""

from __future__ import annotations

import dataclasses
from pathlib import Path

import numpy as np
import pytest

from aloxcam.camera.acquisition import AcqState, GetImageStateMachine
from aloxcam.camera.handlers import CAMERA_HANDLERS, CameraContext
from aloxcam.camera.pipeline import CUSTOM_DATA_KEYS, lv_timestamp_text
from aloxcam.camera.session import CameraSession, make_sessions
from aloxcam.camera.sim import SimCameraBackend, SimCameraDevice, frame_number_of
from aloxcam.dispatch import REQUEUE
from aloxcam.errors import NO_ERROR, CameraError, Code, ErrorCluster
from aloxcam.imaging.png import write_png
from aloxcam.lv.path import LVPath
from aloxcam.settings import (
    HIGH_MAG,
    LOW_MAG,
    AloxSettings,
    CameraOptions,
    CameraSettings,
    CompatSwitches,
    SimulationOptions,
)
from aloxcam.state import ServerState
from aloxcam.testing import FixedClock, RecordingEvents, RecordingReplies, make_request
from aloxcam.wire.messages import CameraStreamData

NOW = 1_700_000_000.25
SETTINGS = CameraSettings.default_high_mag()
STREAM = CameraStreamData()
S, A, F, E = AcqState.START, AcqState.ACQUIRE, AcqState.FINISHED, AcqState.ERROR


def make(*, buffer_zero: bool = True, strict: bool = False, start_tick: int = 10_000,
         device: bool = True) -> tuple[CameraSession, SimCameraDevice, FixedClock]:
    clock = FixedClock(start_tick)
    dev = SimCameraBackend(SimulationOptions(image_width=8, image_height=6), manual=True).open_camera("cam0")
    dev.configure(True, 5)
    dev.start()
    machine = GetImageStateMachine(request_buffer_zero=buffer_zero, strict_new_frame=strict, tick=clock,
                                   now=lambda: NOW)
    return CameraSession(HIGH_MAG, machine, device=dev if device else None), dev, clock


def step(session: CameraSession, state: AcqState, stream: CameraStreamData = STREAM,
         settings: CameraSettings = SETTINGS, error_in: ErrorCluster = NO_ERROR) -> tuple[AcqState, ErrorCluster]:
    return session.state_machine.step(state, stream, session, settings, error_in)


def test_transitions_start_acquire_finished() -> None:
    session, dev, _ = make()
    assert step(session, S) == (A, NO_ERROR)
    assert session.state_machine.t0 == 10_000 and session.state_machine.t0_buf == -1
    assert step(session, A) == (A, NO_ERROR)                    # no new buffer yet
    dev.emit_frame()
    assert step(session, A) == (F, NO_ERROR)
    assert session.src_image is not None and session.src_image.shape == (6, 8)
    assert tuple(session.custom_data) == CUSTOM_DATA_KEYS
    assert session.custom_data["buffer"] == "0"
    assert session.custom_data["type"] == "High-Mag"
    assert session.custom_data["timestamp"] == lv_timestamp_text(NOW)
    assert session.custom_data["name"] == "HighMag"


@pytest.mark.parametrize("state", [F, E, AcqState.STOPPED])
def test_terminal_states_pass_through(state: AcqState) -> None:
    session, _, _ = make()
    assert step(session, state) == (state, NO_ERROR)


def test_error_in_returns_error_and_runs_nothing() -> None:
    session, dev, _ = make()
    dev.emit_frame()
    err = ErrorCluster(True, 42, "temperature")
    for state in (S, A):
        assert step(session, state, error_in=err) == (E, err)
    assert session.src_image is None and session.state_machine.t0 == 0
    warning = ErrorCluster(False, 7, "warning")
    assert step(session, S, error_in=warning) == (A, NO_ERROR)


def test_timeout_9009_exact_text() -> None:
    session, _, clock = make()
    step(session, S)
    clock.advance(5000)
    assert step(session, A) == (A, NO_ERROR)                    # elapsed 5000 is not > 5000
    clock.advance(1)
    assert step(session, A) == (E, ErrorCluster(
        True, 9009, "get image error (timeout)lastbuffer:0/ buffercount:5/ buffer t-1:4294967295"))


def test_timeout_text_with_frames_and_custom_timeout() -> None:
    session, dev, clock = make()
    dev.emit_frame()
    dev.emit_frame()
    step(session, S)
    assert session.state_machine.t0_buf == 1
    clock.advance(101)
    stream = dataclasses.replace(STREAM, acquisition_timeout_ms=100)
    assert step(session, A, stream) == (E, ErrorCluster(
        True, 9009, "get image error (timeout)lastbuffer:1/ buffercount:5/ buffer t-1:1"))


def test_elapsed_time_wraps_at_u32() -> None:
    session, _, clock = make(start_tick=0xFFFFFFF0)
    step(session, S)
    clock.set(0x10)                                             # 32 ms later, across the wrap
    assert step(session, A) == (A, NO_ERROR)
    negative = dataclasses.replace(STREAM, acquisition_timeout_ms=-1)               # I32 → U32: never times out
    clock.advance(10_000_000)
    assert step(session, A, negative) == (A, NO_ERROR)


def test_request_buffer_zero_true_returns_buffer_zero_during_the_first_frames() -> None:
    session, dev, _ = make(buffer_zero=True)
    dev.emit_frame()                                            # buffer 0 before Start
    step(session, S)
    dev.emit_frame()
    assert step(session, A) == (F, NO_ERROR)
    assert session.custom_data["buffer"] == "0"                 # the stale first frame (05 §6.4)
    assert frame_number_of(session.src_image) == 0              # type: ignore[arg-type]
    for _ in range(4):
        dev.emit_frame()                                        # 0..5: buffer 0 overwritten
    step(session, S)
    dev.emit_frame()
    step(session, A)
    assert session.custom_data["buffer"] == "6"                 # newest


def test_request_buffer_zero_false_returns_the_newest() -> None:
    session, dev, _ = make(buffer_zero=False)
    dev.emit_frame()
    step(session, S)
    dev.emit_frame()
    dev.emit_frame()
    assert step(session, A) == (F, NO_ERROR)
    assert session.custom_data["buffer"] == "2"


def test_strict_new_frame() -> None:
    session, dev, _ = make(buffer_zero=False, strict=True)
    dev.emit_frame()
    step(session, S)                                            # t0_buf = 0
    dev.emit_frame()                                            # 1 = t0_buf + 1: may predate the request
    assert step(session, A) == (A, NO_ERROR)
    dev.emit_frame()
    assert step(session, A) == (F, NO_ERROR)
    assert session.custom_data["buffer"] == "2"
    lax, dev2, _ = make(buffer_zero=False, strict=False)
    dev2.emit_frame()
    step(lax, S)
    dev2.emit_frame()
    assert step(lax, A) == (F, NO_ERROR)


def test_start_without_a_camera_session() -> None:
    session, _, _ = make(device=False)
    state, err = step(session, S)
    assert state is E
    assert err == ErrorCluster(True, Code.IMAQDX_INVALID_SESSION, "IMAQdx Property Node: invalid camera session")


def test_acquire_read_errors_are_ignored_then_9009() -> None:
    session, dev, clock = make()
    step(session, S)
    dev.emit_frame()
    boom = CameraError(-1074360000, "LastBufferNumber broken")
    dev.fail_method("last_buffer_number", boom)
    dev.fail_method("buffer_count", boom)
    assert step(session, A) == (A, NO_ERROR)                    # 05 #27: errors ignored
    clock.advance(5001)
    assert step(session, A)[1].source.endswith("lastbuffer:0/ buffercount:0/ buffer t-1:4294967295")


def test_get_image_failure_is_returned() -> None:
    session, dev, _ = make()
    step(session, S)
    dev.emit_frame()
    boom = CameraError(-1074360311, "Get Image2 broken")
    dev.fail_method("get_buffer", boom)
    assert step(session, A) == (E, boom.cluster())


def test_flip_and_flat_field_in_the_camera_path() -> None:
    session, dev, _ = make()
    step(session, S)
    frame = dev.emit_frame()
    flipped = dataclasses.replace(SETTINGS, horizontal_flip=True)
    assert step(session, A, settings=flipped) == (F, NO_ERROR)
    assert np.array_equal(session.src_image, frame.pixels[::-1, :])
    step(session, S)
    dev.emit_frame()
    ff = dataclasses.replace(SETTINGS, flat_field=True)
    state, err = step(session, A, settings=ff)                  # no FFC / DFC loaded
    assert state is E and err.code == Code.VISION_OPERATION
    session.ffc_image = np.full((6, 8), 20, np.uint8)
    session.dfc_image = np.full((6, 8), 10, np.uint8)
    step(session, S)
    dev.emit_frame()
    assert step(session, A, settings=ff) == (F, NO_ERROR)
    assert session.custom_data["flatfield"] == "TRUE"


@pytest.fixture
def sim_folder(tmp_path: Path) -> Path:
    write_png(tmp_path / "1.png", np.full((3, 4), 1, np.uint8))
    write_png(tmp_path / "2.png", np.full((5, 2), 2, np.uint8))
    return tmp_path


def test_simulation_path(sim_folder: Path) -> None:
    session, _, clock = make()
    session.simulation_index, session.simulation_time = 5, 7
    session.custom_data = {"stale": "key"}
    stream = CameraStreamData(camera_type=HIGH_MAG, simulation=True, simulation_path=LVPath.from_local(sim_folder))
    assert step(session, S, stream) == (A, NO_ERROR)
    assert session.simulation and session.simulation_index == 0 and session.simulation_time == 10_000  # first call
    assert session.state_machine.t0_buf == 0
    flipped = dataclasses.replace(SETTINGS, horizontal_flip=True, vertical_flip=True)
    clock.advance(3)
    assert step(session, A, stream, flipped) == (F, NO_ERROR)
    assert session.src_image.tolist() == np.full((3, 4), 1).tolist()   # type: ignore[union-attr]  (no flip)
    assert session.custom_data == {}
    assert (session.simulation_index, session.simulation_time) == (1, 10_003)
    assert step(session, S, stream) == (A, NO_ERROR)            # not a first call: index kept
    assert step(session, A, stream) == (F, NO_ERROR)
    assert session.src_image.shape == (5, 2)                    # type: ignore[union-attr]


def test_simulation_errors(tmp_path: Path, sim_folder: Path) -> None:
    session, _, _ = make()
    no_path = CameraStreamData(simulation=True)
    step(session, S, no_path)
    state, err = step(session, A, no_path)
    assert state is E and err.code == Code.IMAGE_FILE and "<empty path>" in err.source
    empty = tmp_path / "empty"
    empty.mkdir()
    stream = CameraStreamData(simulation=True, simulation_path=LVPath.from_local(empty))
    step(session, S, stream)
    state, err = step(session, A, stream)
    assert state is E and err.code == Code.IMAGE_FILE
    assert session.simulation_index == 1                        # advanced as in LabVIEW (x mod 0 = x)
    ok = CameraStreamData(simulation=True, simulation_path=LVPath.from_local(sim_folder))
    step(session, S, ok)
    ff = dataclasses.replace(SETTINGS, flat_field=True)
    assert step(session, A, ok, ff)[1].code == Code.VISION_OPERATION


def test_independent_t0_per_camera() -> None:
    clock = FixedClock(0)
    sessions = make_sessions(compat=CompatSwitches(), tick=clock)
    hm, lm = sessions[HIGH_MAG], sessions[LOW_MAG]
    assert hm.state_machine is not lm.state_machine
    backend = SimCameraBackend(SimulationOptions(image_width=4, image_height=2), manual=True)
    for name, session in (("a", hm), ("b", lm)):
        session.device = backend.open_camera(name)
        session.device.configure(True, 5)
        session.device.start()
    step(hm, S)
    clock.advance(3000)
    step(lm, S)
    clock.advance(2001)
    assert step(hm, A)[0] is E                                  # 5001 ms since HM's Start
    assert step(lm, A) == (A, NO_ERROR)                         # 2001 ms since LM's Start
    assert hm.state_machine.t0 == 0 and lm.state_machine.t0 == 3000


def test_reset() -> None:
    session, _, _ = make()
    step(session, S)
    session.state_machine.reset()
    assert (session.state_machine.t0, session.state_machine.t0_buf) == (0, -1)


# ---------------------------------------------------------------------------------------------- 05 §6.2 table
def _stream_rig(clock: FixedClock) -> tuple[CameraContext, SimCameraDevice, RecordingReplies]:
    replies = RecordingReplies()
    backend = SimCameraBackend(SimulationOptions(image_width=4, image_height=2), manual=True)
    ctx = CameraContext(state=ServerState.from_settings(AloxSettings()), replies=replies, events=RecordingEvents(),
                        compat=CompatSwitches(), tick=clock, backend=backend,
                        sessions=make_sessions(compat=CompatSwitches(), tick=clock, now=lambda: NOW),
                        forward_to_z=lambda r: None, camera_options=CameraOptions())
    dev = backend.open_camera("cam0")
    dev.configure(True, 5)
    dev.start()
    ctx.sessions[HIGH_MAG].device = dev
    return ctx, dev, replies


@pytest.mark.parametrize(("before", "prepare", "outcome", "after", "reply"), [
    (A, "frame", None, AcqState.STOPPED, "image"),          # Acquire → Finished: image reply
    (A, "nothing", "requeue", A, None),                     # Acquire → Acquire: re-enqueue, no reply yet
    (A, "timeout", None, AcqState.STOPPED, "error"),        # Acquire → Error: null image + error
    (AcqState.STOPPED, "frame", "requeue", A, None),        # anything else → Start → Acquire: re-enqueue
    (S, "frame", "requeue", A, None),
    (F, "frame", "requeue", A, None),
    (E, "frame", "requeue", A, None),
    (AcqState.STOPPED, "no-session", None, AcqState.STOPPED, "error"),   # Start → Error: null image + error
])
def test_stream_transition_table(before: AcqState, prepare: str, outcome: str | None, after: AcqState,
                                 reply: str | None) -> None:
    clock = FixedClock(1000)
    ctx, dev, replies = _stream_rig(clock)
    session = ctx.sessions[HIGH_MAG]
    session.acquisition_state = before
    session.state_machine.t0, session.state_machine.t0_buf = 1000, -1
    if prepare == "frame":
        dev.emit_frame()
    elif prepare == "timeout":
        clock.advance(5001)
    elif prepare == "no-session":
        session.device = None
    req = make_request("Cam Stream Next Image", CameraStreamData(HIGH_MAG))
    result = CAMERA_HANDLERS["Cam Stream Next Image"](ctx, req)
    assert (result is REQUEUE) == (outcome == "requeue")
    assert session.acquisition_state is after
    assert ctx.state.acq_state[HIGH_MAG] == after.value
    if reply is None:
        assert replies.replies == [] and not req.replied
    else:
        (sent,) = replies.replies
        assert sent.kind.value == "IMAGE" and sent.sent
        assert (sent.value is not None) == (reply == "image")
        assert sent.err.status == (reply == "error")
