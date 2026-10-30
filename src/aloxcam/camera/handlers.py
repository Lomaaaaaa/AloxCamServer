"""The camera-worker command handlers (camera and general commands) and the camera-worker context.

Spec: 03 §4.1–§4.2 (execution pattern, reply / decoration order), 03 §5 (general and camera commands), 03 §6 and
06 §3 (autofocus / refocus command shells), 05 §4–§9 (camera behaviour), 05 #9, #11, #12, #26 (fixes), 10 §6
(``[compat]`` switches); design §3.6.7, §7.

Every handler selects the session and the settings by the decoded ``Camera Type`` label, turns
:class:`~aloxcam.errors.CameraError` into error clusters with ``cluster()`` and finishes through
:func:`aloxcam.dispatch.finish_camera`. Non-finite numeric inputs give ``{TRUE, 1, "<attribute>: invalid value"}``
without touching the camera. ``Cam Autofocus`` / ``Cam Refocus`` are Phase-1 stubs (9507, always replied); the
internal ``… Run`` commands have no handler (never routed, 03 C16).
"""

from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, TypeVar

import numpy as np

from aloxcam.camera.acquisition import AcqState
from aloxcam.camera.attributes import (
    DEVICE_TEMPERATURE,
    EXPOSURE_UPPER_LIMIT,
    FRAMERATE_LOWER_LIMIT,
    GET_IMAGE_TIMEOUT_MS,
    GIGE_PACKET_SIZE,
    LINE_INVERTER,
    LINE_MODE,
    LINE_SELECTOR,
    LINE_SOURCE,
    NUM_BUFFERS,
    PROPERTY_NODE,
    TRIGGER_MODE,
    TRIGGER_SOFTWARE,
    USER_OUTPUT_SELECTOR,
    USER_OUTPUT_VALUE,
    format_lv_double,
    in_range_and_coerce,
    lv_to_int,
    read_typed,
    value_kind,
    write_typed,
)
from aloxcam.camera.backend import CameraBackend, ValueType
from aloxcam.camera.pipeline import flip_image, load_correction_image
from aloxcam.camera.session import CameraSession
from aloxcam.clock import U32_MASK
from aloxcam.dispatch import REQUEUE, STOP, FinishOrder, HandlerTable, Outcome, Request, WorkerContext, finish_camera
from aloxcam.errors import (
    NO_ERROR,
    CameraError,
    Code,
    ErrorCluster,
    clear_errors,
    decorate,
    merge_errors,
    replace_source,
)
from aloxcam.imagedata import ImageData
from aloxcam.lv.path import LVPath
from aloxcam.settings import HIGH_MAG, LOW_MAG, PLUG_USB, CameraOptions, CameraSettings
from aloxcam.wire.messages import (
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
    CamLogData,
    ShowUIData,
)

if TYPE_CHECKING:
    from aloxcam.zstage.controller import ZController

__all__ = ["CAMERA_HANDLERS", "NOT_IMPLEMENTED_TEXT", "CameraContext", "close_cameras"]

log = logging.getLogger(__name__)

#: Source text of the Phase-1 focus stubs (code 9507; the command suffix is appended).
NOT_IMPLEMENTED_TEXT = "not implemented in this phase (phase 4)"

_P = TypeVar("_P")


@dataclass
class CameraContext(WorkerContext):
    """Context of the camera worker (design §3.6.7)."""

    backend: CameraBackend
    sessions: Mapping[str, CameraSession]          # "High Mag." / "Low Mag."
    forward_to_z: Callable[[Request], None]
    camera_options: CameraOptions
    decimal_separator: str = "."
    stage: ZController | None = None               # Phase 4 focus jobs
    focus: Any | None = None                       # reserved: Phase 4 FocusController (06 §9.5); None in Phase 1
    sleep: Callable[[float], None] = time.sleep    # Wait LED, LM USB 100 ms (tests patch it)

    @classmethod
    def from_base(cls, base: WorkerContext, *, backend: CameraBackend, sessions: Mapping[str, CameraSession],
                  forward_to_z: Callable[[Request], None], camera_options: CameraOptions,
                  **extra: Any) -> CameraContext:
        """A camera context sharing the state, sinks, switches and tick of ``base`` (``RuntimeParts``)."""
        return cls(state=base.state, replies=base.replies, events=base.events, compat=base.compat, tick=base.tick,
                   backend=backend, sessions=sessions, forward_to_z=forward_to_z, camera_options=camera_options,
                   **extra)


# ---------------------------------------------------------------------------------------------- helpers
def _payload(req: Request, cls: type[_P]) -> _P:
    """The decoded payload; its typedef defaults when the request carries none (internal / test requests)."""
    if req.data is None:
        return cls()
    if not isinstance(req.data, cls):
        raise CameraError(Code.VARIANT_TYPE_MISMATCH,
                          f"Variant To Data in alox_camera_server.vi: {cls.__name__} expected for {req.command}")
    return req.data


def _camera(ctx: CameraContext, camera_type: str) -> tuple[CameraSession, CameraSettings]:
    """Session and settings of ``camera_type``; a label without a session (enum override) is out-of-contract
    input (10 §6 carve-out (a)): ``CameraError(91, …)``."""
    session = ctx.sessions.get(camera_type)
    if session is None:
        raise CameraError(Code.VARIANT_TYPE_MISMATCH,
                          f"Variant To Data in alox_camera_server.vi: unknown Camera Type {camera_type!r}")
    return session, ctx.state.camera_settings(camera_type)


def _invalid(attribute: str) -> ErrorCluster:
    """``{TRUE, 1, "<attribute>: invalid value"}`` for a non-finite numeric input (design §3.6.7)."""
    return ErrorCluster(True, Code.LV_INVALID_INPUT, f"{attribute}: invalid value")


def _attempt(action: Callable[[], object]) -> ErrorCluster:
    """Run one camera access; its :class:`CameraError` as a cluster, else NO_ERROR."""
    try:
        action()
    except CameraError as exc:
        return exc.cluster()
    return NO_ERROR


def _read_max(session: CameraSession, attribute: str) -> float | None:
    """``MaxDBL`` of ``attribute``; None when the read fails (the error is ignored, 05 §5.2)."""
    try:
        return session.require_device(PROPERTY_NODE).read_attribute_max(attribute)
    except CameraError as exc:
        log.debug("%s: MaxDBL of %s not readable (ignored, 05 §5.2): %s", session.camera_type, attribute, exc)
        return None


def _to_u32(x: float) -> int:
    return lv_to_int(x, ValueType.U32) if not math.isnan(x) else 0


def _close_device(session: CameraSession, reason: str) -> None:
    """Close the session's device (05 #9 fix: before a re-open) and forget it; errors are logged."""
    device = session.device
    session.device = None
    if device is None:
        return
    try:
        device.close()
    except CameraError as exc:
        log.warning("%s: %s: closing the previous camera session failed: %s", session.camera_type, reason, exc)


def _load_correction(enabled: bool, path: Path | None) -> tuple[np.ndarray | None, ErrorCluster]:
    """FFC / DFC load of ``Cam Setup`` / ``Cam Set FlatField`` (05 §4.2, §7.3): only when enabled, the path is
    not empty and the file exists; otherwise a null image and no error."""
    if not enabled or path is None or not path.exists():
        return None, NO_ERROR
    try:
        return load_correction_image(path), NO_ERROR
    except CameraError as exc:
        return None, exc.cluster()


# ---------------------------------------------------------------------------------------------- general commands
def _get_heartbeat(ctx: CameraContext, req: Request) -> None:
    """03 §5.1: U32 = server tick, always "no error"."""
    finish_camera(ctx, req, ctx.tick() & U32_MASK, NO_ERROR, camera_type=None)


def _show_ui(ctx: CameraContext, req: Request) -> None:
    """03 §5.2: no GUI in Phase 1; the flag is kept in ``ServerState.show_ui``."""
    data = _payload(req, ShowUIData)
    ctx.state.update(show_ui=bool(data.show))
    finish_camera(ctx, req, None, NO_ERROR, camera_type=None)


def _log(ctx: CameraContext, req: Request) -> None:
    """03 §5.3: store ``Cam Log Data`` (never read, 03 C5)."""
    ctx.state.update(cam_log_data=_payload(req, CamLogData))
    finish_camera(ctx, req, None, NO_ERROR, camera_type=None)


def _exit(ctx: CameraContext, req: Request) -> Outcome:
    """03 §5.4: forward an internal ``Exit`` to the Z worker and stop; no reply, no log."""
    ctx.forward_to_z(Request.make_internal("Exit", req.spec))
    return STOP


# ---------------------------------------------------------------------------------------------- session lifecycle
def _cam_setup(ctx: CameraContext, req: Request) -> None:
    """05 §4.2: Open → [GigE] PacketSize / [USB, LM] 100 ms → Configure (5 buffers) → FFC / DFC."""
    data = _payload(req, CameraInitData)
    camera_type = data.camera_type
    session, settings = _camera(ctx, camera_type)
    if data.number_of_buffers not in (0, NUM_BUFFERS):
        log.warning("Cam Setup %s: Number of buffers = %d is ignored, the ring always has %d buffers (05 #8)",
                    camera_type, data.number_of_buffers, NUM_BUFFERS)
    _close_device(session, "Cam Setup")
    device = None
    e_open = NO_ERROR
    try:
        device = ctx.backend.open_camera(data.camera_name)
    except CameraError as exc:
        e_open = exc.cluster()
    bus = ""
    if device is not None:
        try:                                                     # display only, errors ignored
            bus = device.bus_type()
        except CameraError as exc:
            log.debug("Cam Setup %s: BusType not readable: %s", camera_type, exc)
        if camera_type == HIGH_MAG:
            try:
                high, low = device.serial_number()
                ctx.state.update(serial_high=high & U32_MASK, serial_low=low & U32_MASK)
            except CameraError as exc:
                log.debug("Cam Setup %s: serial number not readable: %s", camera_type, exc)
    e_packet = e_open                                            # error-chained: Open → PacketSize → Configure
    if settings.plug_type == PLUG_USB:
        if camera_type == LOW_MAG:
            ctx.sleep(0.1)                                       # Wait (ms) 100: no error input
    elif device is not None and not e_open.status:
        e_packet = _attempt(lambda: device.set_packet_size(GIGE_PACKET_SIZE))
    e_config = e_packet
    if device is not None and not e_packet.status:
        e_config = _attempt(lambda: device.configure(data.continuous_mode, NUM_BUFFERS))
    ctx.state.update_camera(camera_type, inited=not e_config.status, acq_state=AcqState.STOPPED.value,
                            bus_type=bus)
    ffc, e_ffc = _load_correction(settings.flat_field, settings.flat_field_path)
    dfc, e_dfc = _load_correction(settings.flat_field, settings.dark_current_path)
    session.device = device
    session.src_image = None                                     # IMAQ Create: a new, empty SRC image
    session.ffc_image, session.dfc_image = ffc, dfc
    session.acquisition_state = AcqState.STOPPED
    session.simulation_time = ctx.tick() & U32_MASK
    err = merge_errors(e_config, e_ffc, e_dfc)
    suffix = "Cam Setup"
    if camera_type == LOW_MAG:                                   # + sources of the Configure, PacketSize, Open errors
        suffix += e_config.source + e_packet.source + e_open.source
    finish_camera(ctx, req, None, err, camera_type=camera_type, suffix=suffix)


def _cam_start(ctx: CameraContext, req: Request) -> None:
    """05 §4.4: [GigE] ``TriggerMode`` → ``Start Acquisition`` (runs even if the trigger write failed). The bus
    path comes from the camera's own plug type for both cameras (05 #12 fix)."""
    data = _payload(req, CameraStartData)
    camera_type = data.camera_type
    session, settings = _camera(ctx, camera_type)
    e_trigger = NO_ERROR
    if settings.plug_type != PLUG_USB:
        value = "On" if data.triggered else "Off"
        e_trigger = _attempt(lambda: session.require_device(PROPERTY_NODE).write_attribute(
            TRIGGER_MODE, ValueType.STRING, value))
    e_start = _attempt(lambda: session.require_device("IMAQdx Start Acquisition").start())
    err = merge_errors(e_start, e_trigger)
    suffix = "CamStart TriggerMode" + ("HM" if camera_type == HIGH_MAG else "LM")
    finish_camera(ctx, req, None, err, camera_type=camera_type, suffix=suffix)


def _cam_stop(ctx: CameraContext, req: Request) -> None:
    """05 §4.5: stop if ``AcquisitionInProgress`` (a failed read counts as FALSE and is discarded). Also sets the
    session's acquisition state to ``Stopped`` (05 #11 fix)."""
    data = _payload(req, CameraStopData)
    camera_type = data.camera_type
    session, _ = _camera(ctx, camera_type)
    err = NO_ERROR
    device = session.device
    running = False
    try:
        running = session.require_device(PROPERTY_NODE).acquisition_in_progress()
    except CameraError as exc:
        log.debug("Cam Stop %s: AcquisitionInProgress not readable (discarded, 05 §4.5): %s", camera_type, exc)
    if running and device is not None:
        err = _attempt(device.stop)
    session.acquisition_state = AcqState.STOPPED
    ctx.state.update_camera(camera_type, acq_state=AcqState.STOPPED.value)
    finish_camera(ctx, req, None, err, camera_type=camera_type)


def _cam_cleanup(ctx: CameraContext, req: Request) -> None:
    """05 §4.6: ``Inited`` FALSE first, Unconfigure → Close (Close runs whatever Unconfigure did), session reset."""
    data = _payload(req, CameraReleaseData)
    camera_type = data.camera_type
    session, _ = _camera(ctx, camera_type)
    ctx.state.update_camera(camera_type, inited=False)
    device = session.device
    if device is None:
        err = ErrorCluster(True, Code.IMAQDX_INVALID_SESSION, "IMAQdx Unconfigure Acquisition: invalid camera session")
    else:
        err = merge_errors(_attempt(device.unconfigure), _attempt(device.close))
    session.reset()
    finish_camera(ctx, req, None, err, camera_type=camera_type)


def _cam_get_acquisition_status(ctx: CameraContext, req: Request) -> None:
    """05 §4.7: BOOL ``AcquisitionInProgress``; "invalid camera session" (-1074360316) cleared."""
    data = _payload(req, CameraStatusData)
    camera_type = data.camera_type
    session, _ = _camera(ctx, camera_type)
    value = False
    err = NO_ERROR
    try:
        value = session.require_device(PROPERTY_NODE).acquisition_in_progress()
    except CameraError as exc:
        err = exc.cluster()
    err, _ = clear_errors(err, Code.IMAQDX_INVALID_SESSION)
    finish_camera(ctx, req, bool(value), err, camera_type=camera_type)


def _cam_software_trigger(ctx: CameraContext, req: Request) -> None:
    """05 §4.8: [GigE] ``TriggerSoftware``; every error is cleared, the reply is always "no error"."""
    data = _payload(req, CameraTriggerSoftData)
    camera_type = data.camera_type
    session, settings = _camera(ctx, camera_type)
    if settings.plug_type != PLUG_USB:
        err = _attempt(lambda: session.require_device(PROPERTY_NODE).write_attribute(
            TRIGGER_SOFTWARE, ValueType.BOOL, True))
        if err.status:
            log.warning("Cam Software Trigger %s: error cleared as in LabVIEW (05 §4.8): %s", camera_type, err)
    finish_camera(ctx, req, None, NO_ERROR, camera_type=camera_type)


# ---------------------------------------------------------------------------------------------- attributes
def _cam_set_exposure(ctx: CameraContext, req: Request) -> None:
    """05 §5.2: clamp, write the exposure, then the frame rate (HM: ``MaxDBL``, LM: ``fr_calc`` for I64/U32 only;
    ``exposure_sets_max_framerate`` FALSE: HM writes the computed rate). Error sources are replaced."""
    data = _payload(req, CameraSetExposureData)
    camera_type = data.camera_type
    session, s = _camera(ctx, camera_type)
    if not math.isfinite(data.exposure):
        finish_camera(ctx, req, None, _invalid(s.exposure_attribute), camera_type=camera_type)
        return
    exposure = in_range_and_coerce(data.exposure, s.exposure_lower_limit, EXPOSURE_UPPER_LIMIT)
    e_exposure = replace_source(_attempt(lambda: write_typed(session.require_device(PROPERTY_NODE),
                                                             s.exposure_attribute, s.exposure_type, exposure)),
                                s.exposure_attribute)
    computed = 0.95 * (1.0 / (exposure / 1_000_000.0))
    fr_calc = in_range_and_coerce(computed, FRAMERATE_LOWER_LIMIT, s.max_framerate)
    e_rate = NO_ERROR
    if camera_type == HIGH_MAG:
        if ctx.compat.exposure_sets_max_framerate:               # LabVIEW: the camera maximum is written
            maxdbl = _read_max(session, s.framerate_attribute)
            target = 0.0 if maxdbl is None else maxdbl           # failed MaxDBL read → LabVIEW default 0
            ctx.state.update(max_framerate_display=fr_calc, max_fr_real=_to_u32(target))
        else:                                                    # fixed behaviour (10 §6, Q16)
            upper = ctx.camera_options.hm_max_framerate
            if upper <= 0:
                maxdbl = _read_max(session, s.framerate_attribute)
                upper = math.inf if maxdbl is None else maxdbl
                if maxdbl is not None:
                    ctx.state.update(max_fr_real=_to_u32(maxdbl))
            fr_calc = target = in_range_and_coerce(computed, FRAMERATE_LOWER_LIMIT, upper)
            ctx.state.update(max_framerate_display=fr_calc)
        e_rate = _attempt(lambda: write_typed(session.require_device(PROPERTY_NODE), s.framerate_attribute,
                                              s.framerate_type, target))
    elif value_kind(s.framerate_type) is not ValueType.DBL:      # LM: nothing is written for DBL (05 §5.2)
        e_rate = _attempt(lambda: write_typed(session.require_device(PROPERTY_NODE), s.framerate_attribute,
                                              s.framerate_type, fr_calc))
    e_rate = replace_source(e_rate, s.framerate_attribute + format_lv_double(fr_calc, ctx.decimal_separator))
    finish_camera(ctx, req, None, merge_errors(e_exposure, e_rate), camera_type=camera_type)


def _get_attribute(ctx: CameraContext, req: Request, camera_type: str, which: str) -> None:
    """05 §5.3–§5.4: read with the configured accessor, DBL reply (0.0 on error), suffix = attribute + type."""
    session, s = _camera(ctx, camera_type)
    attribute, type_str = ((s.exposure_attribute, s.exposure_type) if which == "exposure"
                           else (s.gain_attribute, s.gain_type))
    value = 0.0
    err = NO_ERROR
    try:
        value = read_typed(session.require_device(PROPERTY_NODE), attribute, type_str)
    except CameraError as exc:
        err = exc.cluster()
    finish_camera(ctx, req, value, err, camera_type=camera_type, suffix=attribute + type_str)


def _cam_get_exposure(ctx: CameraContext, req: Request) -> None:
    """05 §5.3."""
    _get_attribute(ctx, req, _payload(req, CameraGetExposureData).camera_type, "exposure")


def _cam_get_gain(ctx: CameraContext, req: Request) -> None:
    """05 §5.4 (variant A)."""
    _get_attribute(ctx, req, _payload(req, CameraGetGainData).camera_type, "gain")


def _cam_set_gain(ctx: CameraContext, req: Request) -> None:
    """05 §5.4: no clamping; I64 / U32 round half to even and saturate."""
    data = _payload(req, CameraGainData)
    camera_type = data.camera_type
    session, s = _camera(ctx, camera_type)
    if not math.isfinite(data.gain):
        err = _invalid(s.gain_attribute)
    else:
        err = _attempt(lambda: write_typed(session.require_device(PROPERTY_NODE), s.gain_attribute, s.gain_type,
                                           data.gain))
    finish_camera(ctx, req, None, err, camera_type=camera_type, suffix=s.gain_attribute + s.gain_type)


# ---------------------------------------------------------------------------------------------- images
def _cam_stream_next_image(ctx: CameraContext, req: Request) -> Outcome | None:
    """05 §6.2: temperature read, one state-machine step, then re-queue (``Acquire``), reply the image
    (``Finished``) or reply a null image with the error. Exactly one IMAGE reply per request."""
    data = _payload(req, CameraStreamData)
    camera_type = data.camera_type
    session, settings = _camera(ctx, camera_type)
    e_temp = NO_ERROR
    try:
        temperature = session.require_device(PROPERTY_NODE).read_attribute(DEVICE_TEMPERATURE, ValueType.DBL)
        ctx.state.update_camera(camera_type, temperature=float(temperature))
    except CameraError as exc:
        e_temp = decorate(exc.cluster(), DEVICE_TEMPERATURE)
    if e_temp.status and not ctx.compat.temperature_error_fails_image:
        log.warning("Cam Stream Next Image %s: temperature read failed, image still acquired "
                    "(temperature_error_fails_image = FALSE): %s", camera_type, e_temp)
        e_temp = NO_ERROR
    from_acquire = session.acquisition_state is AcqState.ACQUIRE
    state, err = session.state_machine.step(AcqState.ACQUIRE if from_acquire else AcqState.START, data, session,
                                            settings, e_temp)
    outcome: Outcome | None = None
    if state is AcqState.ACQUIRE:                                # "Image Not Ready / Re-Enqueue command"
        session.acquisition_state = AcqState.ACQUIRE
        ctx.state.set_error_out(err)
        outcome = REQUEUE
    elif state is AcqState.FINISHED and from_acquire:            # "Image Acquired / Return it"
        session.display_image = session.src_image
        image = ImageData(session.src_image, dict(session.custom_data)) if session.src_image is not None else None
        finish_camera(ctx, req, image, err, camera_type=camera_type, suffix="", extra="Enqueue Image Data",
                      order=FinishOrder.RAW_FIRST)               # real camera type in the log (05 #26 fix)
        session.acquisition_state = AcqState.STOPPED
    else:                                                        # "Acquisition Error / Return Immediately"
        extra = "Get Image Acquisition" if from_acquire else "Get Image Start"
        finish_camera(ctx, req, None, err, camera_type=camera_type, suffix="", extra=extra, order=FinishOrder.A)
        session.acquisition_state = AcqState.STOPPED
    ctx.state.update_camera(camera_type, acq_state=session.acquisition_state.value)
    return outcome


def _cam_get_image(ctx: CameraContext, req: Request) -> None:
    """05 §6.5: ``Get Image2 Last New`` + flip into the display; no reply, no event log; the error (−1074360316
    cleared, suffix ``"Cam Get Image"``) goes to ``ServerState.error_out`` only. Nothing in simulation."""
    data = _payload(req, CameraGetImageData)
    session, settings = _camera(ctx, data.camera_type)
    if session.simulation:
        return
    err = NO_ERROR
    try:
        frame = session.require_device("IMAQdx Get Image2").get_last_new(GET_IMAGE_TIMEOUT_MS)
        image = np.ascontiguousarray(flip_image(frame.pixels, settings.horizontal_flip, settings.vertical_flip))
        session.src_image = image                                # Get Image2 into SRC, flipped in place
        session.display_image = image
    except CameraError as exc:
        err = exc.cluster()
    err, _ = clear_errors(err, Code.IMAQDX_INVALID_SESSION)
    suffix = req.spec.suffix if req.spec is not None and req.spec.suffix else "Cam Get Image"
    ctx.state.set_error_out(decorate(err, suffix))


def _cam_get_images_array(ctx: CameraContext, req: Request) -> None:
    """03 §5.17: "TO BE DEFINED": nothing, no reply, no log."""


# ---------------------------------------------------------------------------------------------- processing settings
def _cam_flip(ctx: CameraContext, req: Request) -> None:
    """05 §7.2: software flip flags (memory only)."""
    data = _payload(req, CameraFlipData)
    _camera(ctx, data.camera_type)
    ctx.state.replace_camera_settings(data.camera_type, horizontal_flip=bool(data.horizontal_flip),
                                      vertical_flip=bool(data.vertical_flip))
    finish_camera(ctx, req, None, NO_ERROR, camera_type=data.camera_type)


def _cam_set_flatfield(ctx: CameraContext, req: Request) -> None:
    """05 §7.3: settings ← {Enable, FFC Path, DFC Path, Factor}; SRC re-created; FFC / DFC reloaded when enabled
    and the files exist. A missing file is logged (WARNING) and, with ``flatfield_check_on_set``, returned as
    ``{TRUE, 9501, "<path>: flat-field file not found"}``."""
    data = _payload(req, CameraSetFlatFieldData)
    camera_type = data.camera_type
    session, _ = _camera(ctx, camera_type)
    if not math.isfinite(data.correction_factor):
        finish_camera(ctx, req, None, _invalid("Correction Factor"), camera_type=camera_type)
        return
    ffc_path, dfc_path = data.ffc_path.to_local(), data.dfc_path.to_local()
    settings = ctx.state.replace_camera_settings(camera_type, flat_field=bool(data.enable), flat_field_path=ffc_path,
                                                 dark_current_path=dfc_path,
                                                 flat_field_factor=float(data.correction_factor))
    session.src_image = None
    loaded: list[tuple[np.ndarray | None, ErrorCluster]] = []
    for label, lv_path, path in (("flat-field", data.ffc_path, ffc_path), ("dark-field", data.dfc_path, dfc_path)):
        image, err = _load_correction(settings.flat_field, path)
        if settings.flat_field and (path is None or not path.exists()):
            text = str(path) if path is not None else (_path_text(lv_path))
            log.warning("Cam Set FlatField %s: %s correction enabled but %s does not exist; every acquisition will "
                        "fail in the correction (05 #17)", camera_type, label, text)
            if ctx.compat.flatfield_check_on_set:
                err = ErrorCluster(True, Code.IMAGE_FILE, f"{text}: flat-field file not found")
        loaded.append((image, err))
    (session.ffc_image, e_ffc), (session.dfc_image, e_dfc) = loaded
    finish_camera(ctx, req, None, merge_errors(e_ffc, e_dfc), camera_type=camera_type)


def _path_text(path: LVPath) -> str:
    text = path.to_windows()
    return text if text else "<empty path>"


# ---------------------------------------------------------------------------------------------- LED / bulb
def _cam_set_led(ctx: CameraContext, req: Request) -> None:
    """05 §9.2, §12.8: bus path from ``Camera Sort: LED``, session from the client's camera, line numbers from
    the HM section; ``Wait LED`` (USB: when switching off; GigE: when the output changes) even after an I/O
    error. Order RAW_FIRST: the raw error is replied, then decorated and logged."""
    data = _payload(req, CameraSetLedData)
    camera_type = data.camera_type
    session, settings = _camera(ctx, camera_type)
    light = ctx.state.light
    led_cmd = (not data.led) if settings.invert_led else bool(data.led)
    off = not led_cmd
    ctx.state.update(led_cmd=led_cmd)
    if light.camera_sort_led == PLUG_USB:
        trace = "us"

        def usb() -> None:
            device = session.require_device(PROPERTY_NODE)
            for line, inverter in ((light.hm_led_line, light.led_status_inverter),
                                   (light.hm_bulb_line, light.bulb_status_inverter)):
                device.write_attribute(LINE_SELECTOR, ValueType.U32, int(line) & U32_MASK)
                device.write_attribute(LINE_MODE, ValueType.U32, 1)
                device.write_attribute(LINE_SOURCE, ValueType.U32, 1)
                device.write_attribute(LINE_INVERTER, ValueType.BOOL, led_cmd if inverter else off)

        err = _attempt(usb)
        wait = off
    else:
        trace = "gi"
        target = led_cmd if light.led_status_inverter else off
        previous = [False]                                       # a failed read returns FALSE (inferred)

        def gige() -> None:
            device = session.require_device(PROPERTY_NODE)
            device.write_attribute(LINE_SELECTOR, ValueType.U32, 1)
            device.write_attribute(LINE_SOURCE, ValueType.U32, 1)
            device.write_attribute(USER_OUTPUT_SELECTOR, ValueType.U32, 0)
            previous[0] = bool(device.read_attribute(USER_OUTPUT_VALUE, ValueType.BOOL))
            device.write_attribute(USER_OUTPUT_VALUE, ValueType.BOOL, target)

        err = _attempt(gige)
        wait = previous[0] != target
    ctx.state.update(light_trace=trace)
    if wait and settings.wait_led_ms > 0:
        ctx.sleep(settings.wait_led_ms / 1000.0)
    finish_camera(ctx, req, None, err, camera_type=camera_type)


def _cam_get_led(ctx: CameraContext, req: Request) -> None:
    """05 §9.3: USB writes ``LineSource = 0`` then reads ``LineInverter``; GigE reads ``UserOutputValue``.
    BOOL = value XOR ``Invert LED``, with the raw error (variant D)."""
    data = _payload(req, CameraGetLedData)
    camera_type = data.camera_type
    session, settings = _camera(ctx, camera_type)
    light = ctx.state.light
    value = [False]

    def read() -> None:
        device = session.require_device(PROPERTY_NODE)
        if light.camera_sort_led == PLUG_USB:
            device.write_attribute(LINE_SELECTOR, ValueType.U32, int(light.hm_led_line) & U32_MASK)
            device.write_attribute(LINE_MODE, ValueType.U32, 1)
            device.write_attribute(LINE_SOURCE, ValueType.U32, 0)
            value[0] = bool(device.read_attribute(LINE_INVERTER, ValueType.BOOL))
        else:
            device.write_attribute(LINE_SELECTOR, ValueType.U32, 1)
            device.write_attribute(LINE_SOURCE, ValueType.U32, 1)
            device.write_attribute(USER_OUTPUT_SELECTOR, ValueType.U32, 0)
            value[0] = bool(device.read_attribute(USER_OUTPUT_VALUE, ValueType.BOOL))

    err = _attempt(read)
    ctx.state.update(led_status=value[0])
    reply = (not value[0]) if settings.invert_led else value[0]
    finish_camera(ctx, req, reply, err, camera_type=camera_type)


# ---------------------------------------------------------------------------------------------- focus (Phase-1 shells)
def _focus_stub(ctx: CameraContext, req: Request, kind: str) -> None:
    """06 §3.1 shell: ``{TRUE, 9507, "not implemented in this phase (phase 4)"}`` in order A, ACK forced (design
    §7); the focus fields as after a failed start (state ``Error``, data 0×0, progress 0)."""
    data = _payload(req, CameraAutofocusData if kind == "AF" else CameraRefocusData)
    camera_type = data.camera_type
    session, _ = _camera(ctx, camera_type)
    err = ErrorCluster(True, Code.NOT_IMPLEMENTED_IN_PHASE, NOT_IMPLEMENTED_TEXT)
    focus = session.focus
    if kind == "AF":
        focus.af_state = "Error"
    else:
        focus.rf_state = "Error"
    focus.last_focus_data = np.empty((0, 0))
    focus.focus_progress = 0.0
    focus.last_error = finish_camera(ctx, req, None, err, camera_type=camera_type, force=True)


def _cam_autofocus(ctx: CameraContext, req: Request) -> None:
    """06 §3.1 (Phase-1 stub)."""
    _focus_stub(ctx, req, "AF")


def _cam_refocus(ctx: CameraContext, req: Request) -> None:
    """06 §3.1 (Phase-1 stub)."""
    _focus_stub(ctx, req, "RF")


def _cam_get_autofocus_data(ctx: CameraContext, req: Request) -> None:
    """06 §3.3: DBL2D ``Last Focus Data`` (N × 2 ``[Z, score]``; 0×0 before any run), variant B."""
    data = _payload(req, CameraGetAutofocusData)
    session, _ = _camera(ctx, data.camera_type)
    finish_camera(ctx, req, np.array(session.focus.last_focus_data, dtype=np.float64), NO_ERROR,
                  camera_type=data.camera_type)


def _cam_get_autofocus_progress(ctx: CameraContext, req: Request) -> None:
    """06 §3.3: DBL ``Focus Progress (%)``, variant B."""
    data = _payload(req, CameraGetAutofocusData)
    session, _ = _camera(ctx, data.camera_type)
    finish_camera(ctx, req, float(session.focus.focus_progress), NO_ERROR, camera_type=data.camera_type)


# ---------------------------------------------------------------------------------------------- table, shutdown
#: Every camera-worker command of ``wire.messages.COMMANDS`` except the internal ``… Run`` commands.
CAMERA_HANDLERS: HandlerTable = MappingProxyType({
    "Get Heartbeat": _get_heartbeat,
    "Show UI": _show_ui,
    "Log": _log,
    "Exit": _exit,
    "Cam Setup": _cam_setup,
    "Cam Start": _cam_start,
    "Cam Stop": _cam_stop,
    "Cam Cleanup": _cam_cleanup,
    "Cam Get Acquisition Status": _cam_get_acquisition_status,
    "Cam Software Trigger": _cam_software_trigger,
    "Cam Set Exposure": _cam_set_exposure,
    "Cam Get Exposure": _cam_get_exposure,
    "Cam Set Gain": _cam_set_gain,
    "Cam Get Gain": _cam_get_gain,
    "Cam Stream Next Image": _cam_stream_next_image,
    "Cam Get Image": _cam_get_image,
    "Cam Get Images Array": _cam_get_images_array,
    "Cam Flip": _cam_flip,
    "Cam Set FlatField": _cam_set_flatfield,
    "Cam Set Led": _cam_set_led,
    "Cam Get Led": _cam_get_led,
    "Cam Autofocus": _cam_autofocus,
    "Cam Refocus": _cam_refocus,
    "Cam Get Autofocus Data": _cam_get_autofocus_data,
    "Cam Get Autofocus Progress": _cam_get_autofocus_progress,
})


def close_cameras(sessions: Mapping[str, CameraSession]) -> None:
    """Shutdown (05 #10 fix): stop and close every open device, errors logged; the sessions keep no device."""
    for camera_type, session in sessions.items():
        device = session.device
        if device is None:
            continue
        session.device = None
        try:
            if device.acquisition_in_progress():
                device.stop()
        except Exception as exc:                                 # noqa: BLE001 - shutdown must go on
            log.warning("shutdown: stopping the %s camera failed: %s", camera_type, exc)
        try:
            device.close()
        except Exception as exc:                                 # noqa: BLE001
            log.warning("shutdown: closing the %s camera failed: %s", camera_type, exc)
