"""The camera session registers of the camera loop (one per camera) and their focus fields.

Spec: 05 §3.2 (``alox_typedef_ext_camera_session.ctl``), §4.6 (reset by ``Cam Cleanup``), 06 §2.5 (focus fields),
06 §9.5 (``FocusFields``); design §3.6.6.

Thread contract (design §2): a session is touched by the camera worker thread only (the shutdown thread may use it
once the camera worker has stopped).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from aloxcam.camera.acquisition import AcqState, GetImageStateMachine
from aloxcam.camera.backend import CameraDevice
from aloxcam.clock import TickFn
from aloxcam.errors import CameraError, Code, ErrorCluster
from aloxcam.lv.path import LVPath
from aloxcam.settings import CAMERA_TYPES, CompatSwitches

__all__ = ["CameraSession", "FocusFields", "make_sessions"]


@dataclass
class FocusFields:
    """``AF State``, ``RF State``, ``Last Focus Data`` and ``Focus Progress (%)`` of a session (06 §2.5).

    ``last_focus_data`` and ``focus_progress`` are shared by autofocus and refocus. ``last_error`` is new (not in
    LabVIEW): the last focus error, for logs (06 §9.5).
    """

    af_state: str = "Start"
    rf_state: str = "Start"
    last_focus_data: np.ndarray = field(default_factory=lambda: np.empty((0, 0)))
    focus_progress: float = 0.0
    last_error: ErrorCluster | None = None


@dataclass
class CameraSession:
    """One camera session register (05 §3.2); touched only by the camera worker thread.

    ``src_image`` is the IMAQ ``SRC Image`` (None until a frame was acquired), ``ffc_image`` / ``dfc_image`` the
    correction images (None = null image), ``display_image`` the front-panel ``Image`` (diagnostics only).
    """

    camera_type: str
    state_machine: GetImageStateMachine
    device: CameraDevice | None = None
    src_image: np.ndarray | None = None
    ffc_image: np.ndarray | None = None
    dfc_image: np.ndarray | None = None
    custom_data: dict[str, str] = field(default_factory=dict)
    display_image: np.ndarray | None = None
    simulation: bool = False
    simulation_path: LVPath = LVPath()
    simulation_index: int = 0
    simulation_time: int = 0
    acquisition_state: AcqState = AcqState.START
    focus: FocusFields = field(default_factory=FocusFields)

    def require_device(self, operation: str) -> CameraDevice:
        """The open device; ``CameraError(-1074360316, "<operation>: invalid camera session")`` when there is none
        (IMAQdx error on a session that is not open, 05 §10.1)."""
        if self.device is None:
            raise CameraError(Code.IMAQDX_INVALID_SESSION, f"{operation}: invalid camera session")
        return self.device

    def reset(self) -> None:
        """``Cam Cleanup``: the session constant (typedef defaults, 05 §4.6 / 03 §5.8) and a reset state machine.
        The device must have been closed by the caller."""
        self.device = None
        self.src_image = None
        self.ffc_image = None
        self.dfc_image = None
        self.custom_data = {}
        self.display_image = None
        self.simulation = False
        self.simulation_path = LVPath()
        self.simulation_index = 0
        self.simulation_time = 0
        self.acquisition_state = AcqState.START
        self.focus = FocusFields()
        self.state_machine.reset()


def make_sessions(*, compat: CompatSwitches, tick: TickFn, decimal_separator: str = ".",
                  now: Callable[[], float] | None = None) -> dict[str, CameraSession]:
    """One :class:`CameraSession` with its **own** :class:`GetImageStateMachine` per camera type (05 #3 fix).

    ``now`` (optional) is the wall clock of the custom-data timestamps (tests).
    """
    sessions: dict[str, CameraSession] = {}
    for camera_type in CAMERA_TYPES:
        kwargs = {} if now is None else {"now": now}
        machine = GetImageStateMachine(request_buffer_zero=compat.request_buffer_zero,
                                       strict_new_frame=compat.strict_new_frame, tick=tick,
                                       decimal_separator=decimal_separator, **kwargs)
        sessions[camera_type] = CameraSession(camera_type=camera_type, state_machine=machine)
    return sessions
