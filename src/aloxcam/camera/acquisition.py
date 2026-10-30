"""``alox_camera_server_get_image_state_machine.vi``: one step of the per-request acquisition state machine.

Spec: 05 §6.3 (interface, outer error case, ``Start`` / ``Acquire`` camera and simulation), §6.4 (calling contract,
which frame is returned), §12.7 (reference sketch), 05 #1–#3, #21, #27; 10 §6 (``request_buffer_zero``,
``strict_new_frame``); design §3.6.5.

The LabVIEW VI is non-reentrant, so its ``t0`` / ``t0 Buffer Number`` registers are shared by both cameras
(owner-answers §3). The Python server keeps **one instance per camera** (05 #3 fix, design §7). The 1 ms ``Wait``
of the VI is not reproduced here: the worker waits before re-processing a re-queued request (design §3.4.4).
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from enum import Enum
from typing import TYPE_CHECKING

import numpy as np

from aloxcam.camera.attributes import GET_IMAGE_TIMEOUT_MS, PROPERTY_NODE
from aloxcam.camera.pipeline import (
    SimulationError,
    build_custom_data,
    flat_field_correct,
    flip_image,
    simulate_next_image,
)
from aloxcam.clock import U32_MASK, TickFn, tick_diff
from aloxcam.errors import NO_ERROR, TEXT, CameraError, Code, ErrorCluster
from aloxcam.settings import CameraSettings

if TYPE_CHECKING:
    from aloxcam.camera.session import CameraSession
    from aloxcam.wire.messages import CameraStreamData

__all__ = ["AcqState", "GetImageStateMachine"]

log = logging.getLogger(__name__)

class AcqState(str, Enum):
    """``typedef_ext_camera_get_image_state.ctl`` (05 §3.4); the value is the item label."""

    START = "Start"
    ACQUIRE = "Acquire"
    FINISHED = "Finished"
    ERROR = "Error"
    STOPPED = "Stopped"


class GetImageStateMachine:
    """The acquisition state machine of one camera (05 §6.3); touched by the camera worker only.

    ``t0`` is the tick of the last ``Start``; ``t0_buf`` the ``LastBufferNumber`` sampled there (-1 when no frame
    existed, so that buffer 0 counts as new; 0 in simulation). ``now`` gives the wall-clock time of the custom-data
    ``timestamp`` (tests inject it).
    """

    def __init__(self, *, request_buffer_zero: bool, strict_new_frame: bool, tick: TickFn,
                 decimal_separator: str = ".", now: Callable[[], float] = time.time) -> None:
        self.request_buffer_zero = request_buffer_zero
        self.strict_new_frame = strict_new_frame
        self.tick = tick
        self.decimal_separator = decimal_separator
        self.now = now
        self.t0: int = 0
        self.t0_buf: int = -1

    def reset(self) -> None:
        """Forget ``t0`` / ``t0_buf`` (``Cam Cleanup``)."""
        self.t0 = 0
        self.t0_buf = -1

    def step(self, state: AcqState, stream: CameraStreamData, session: CameraSession, settings: CameraSettings,
             error_in: ErrorCluster) -> tuple[AcqState, ErrorCluster]:
        """One call of the VI (05 §6.3) → ``(State Out, error out)``; writes ``session.src_image``,
        ``session.custom_data`` and the simulation fields in place.

        ``error_in`` with ``status`` TRUE → ``(Error, error_in)``, nothing else runs. ``Start`` never returns
        ``Finished``. ``Finished`` / ``Error`` / ``Stopped`` pass through unchanged.
        """
        if error_in.status:
            return AcqState.ERROR, error_in
        state = AcqState(state)
        if state is AcqState.START:
            return self._start(stream, session)
        if state is AcqState.ACQUIRE:
            if session.simulation:
                return self._acquire_simulation(stream, session, settings)
            return self._acquire_camera(stream, session, settings)
        return state, error_in

    # ------------------------------------------------------------------ states
    def _start(self, stream: CameraStreamData, session: CameraSession) -> tuple[AcqState, ErrorCluster]:
        if stream.simulation and not session.simulation:          # "First Call ?"
            session.simulation_index = 0
            session.simulation_time = self.tick() & U32_MASK
        session.simulation = stream.simulation
        session.simulation_path = stream.simulation_path
        self.t0 = self.tick() & U32_MASK
        if stream.simulation:
            self.t0_buf = 0
            return AcqState.ACQUIRE, NO_ERROR
        try:
            last = session.require_device(PROPERTY_NODE).last_buffer_number()
        except CameraError as exc:                                   # this read's error IS wired (05 §6.3)
            self.t0_buf = 0
            return AcqState.ERROR, exc.cluster()
        self.t0_buf = -1 if last is None else int(last)
        return AcqState.ACQUIRE, NO_ERROR

    def _acquire_camera(self, stream: CameraStreamData, session: CameraSession,
                        settings: CameraSettings) -> tuple[AcqState, ErrorCluster]:
        dev = session.device
        last: int | None = None
        count = 0
        if dev is None:
            log.debug("%s: Acquire without a camera session (LastBufferNumber / BufferCount unreadable)",
                      session.camera_type)
        else:
            try:                                                     # read errors IGNORED here (05 #27)
                last = dev.last_buffer_number()
            except CameraError as exc:
                log.debug("%s: LastBufferNumber read failed (ignored, 05 #27): %s", session.camera_type, exc)
            try:
                count = dev.buffer_count()
            except CameraError as exc:
                log.debug("%s: BufferCount read failed (ignored, 05 #27): %s", session.camera_type, exc)
        if last is None or not self._is_new(int(last)):
            elapsed = tick_diff(self.tick(), self.t0)
            if elapsed > (stream.acquisition_timeout_ms & U32_MASK):
                msg = (f"lastbuffer:{0 if last is None else last}/ buffercount:{count}"
                       f"/ buffer t-1:{self.t0_buf & U32_MASK}")
                return AcqState.ERROR, ErrorCluster(True, Code.GET_IMAGE_TIMEOUT,
                                                    TEXT[Code.GET_IMAGE_TIMEOUT] + msg)
            return AcqState.ACQUIRE, NO_ERROR                       # the {FALSE, 0, "lastbuffer:…"} diag: 05 #21
        assert dev is not None
        try:
            frame = dev.get_buffer(0 if self.request_buffer_zero else None, GET_IMAGE_TIMEOUT_MS)
            img = flip_image(frame.pixels, settings.horizontal_flip, settings.vertical_flip)
            if settings.flat_field:
                img = flat_field_correct(img, session.ffc_image, session.dfc_image, settings.flat_field_factor)
        except CameraError as exc:
            return AcqState.ERROR, exc.cluster()
        session.src_image = np.ascontiguousarray(img)
        session.custom_data = build_custom_data(frame.number, settings, stream.camera_type, now=self.now(),
                                                decimal_separator=self.decimal_separator)
        return AcqState.FINISHED, NO_ERROR

    def _acquire_simulation(self, stream: CameraStreamData, session: CameraSession,
                            settings: CameraSettings) -> tuple[AcqState, ErrorCluster]:
        session.simulation = stream.simulation
        session.simulation_path = stream.simulation_path
        folder = stream.simulation_path.to_local()
        if folder is None:
            text = stream.simulation_path.to_windows() or "<empty path>"
            return AcqState.ERROR, ErrorCluster(True, Code.IMAGE_FILE,
                                                f"List Folder: simulation folder {text} is not available")
        try:
            img, index, t = simulate_next_image(folder, stream.simulation_timer_ms, session.simulation_index,
                                                session.simulation_time, self.tick)
        except SimulationError as exc:
            session.simulation_index, session.simulation_time = exc.index_out, exc.time_out
            return AcqState.ERROR, exc.cluster()
        except CameraError as exc:
            return AcqState.ERROR, exc.cluster()
        session.simulation_index, session.simulation_time = index, t
        if settings.flat_field:                                      # no flip, no custom data (05 #22)
            try:
                img = flat_field_correct(img, session.ffc_image, session.dfc_image, settings.flat_field_factor)
            except CameraError as exc:
                return AcqState.ERROR, exc.cluster()
        session.src_image = img
        session.custom_data = {}
        return AcqState.FINISHED, NO_ERROR

    def _is_new(self, last: int) -> bool:
        """"Buffer Available": ``last > t0_buf``; ``last >= t0_buf + 2`` with ``strict_new_frame`` (05 #2)."""
        if self.strict_new_frame:
            return last >= self.t0_buf + 2
        return last > self.t0_buf
