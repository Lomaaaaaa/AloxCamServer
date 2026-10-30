"""The Z consumer loop cases: ``ZContext`` and the 19 Z command handlers plus ``Exit``.

Spec: 03 §4.3 (Z pattern: reply → log, no decoration), §7 (Z commands); 08 §5 (command details), §6 (reference check
before moves), §7 (software range, 9405), §8 (wait end move), §9 (joystick), §10 (``DH`` restart), §13.4 (reference
sketch); 10 §6 (``restart_dh_macro``, ``implicit_z_reinit``); 09 §10.5 (code 5 proposal); design §3.5.5.

Every handler starts from "no error" (LabVIEW: no ``error in`` on the first node) and finishes with
:func:`aloxcam.dispatch.finish_z`. A failed step skips the later ones, as a LabVIEW error chain does; query handlers
reply the type default (0.0, FALSE, ``""``) next to the error. Payloads were decoded strictly by the router: a request
that did not decode never reaches a handler (design §2, 08 Z10 fix), so the "VTD error" paths of 08 §5.3 do not exist.
"""

from __future__ import annotations

import logging
import math
from collections.abc import Callable
from dataclasses import dataclass, fields
from types import MappingProxyType
from typing import Any

from aloxcam.dispatch import SERVER_Z, STOP, HandlerTable, Outcome, Request, WorkerContext, finish_z
from aloxcam.errors import NO_ERROR, Code, ErrorCluster, StageError
from aloxcam.settings import StageOptions, ZStageSettings
from aloxcam.wire.messages import (
    ZGetSerialData,
    ZJoystickData,
    ZLimitData,
    ZMoveAbsData,
    ZMoveRelData,
    ZReferenceData,
    ZSetSpeedData,
    ZWaitData,
)
from aloxcam.zstage.controller import DH_MACRO, KNOWN_MACROS, ZController

__all__ = ["NOT_REFERENCED_TEXT", "REFERENCE_RESET_INFO", "Z_HANDLERS", "ZContext"]

log = logging.getLogger(__name__)

NOT_REFERENCED_TEXT = "Z axis not referenced"            # code 5 when implicit_z_reinit = FALSE (09 §10.5)
REFERENCE_RESET_INFO = "{label}: Z Reference resetted"   # Log Server Infos text after a re-initialisation (08 §6)
_OUT_OF_RANGE = ErrorCluster(True, Code.Z_OUT_OF_RANGE, "Z move out of range")


@dataclass
class ZContext(WorkerContext):
    """Context of the Z worker: the shared controller link and the ``[stage]`` options."""

    stage: ZController
    stage_options: StageOptions

    @classmethod
    def from_base(cls, base: WorkerContext, *, stage: ZController, stage_options: StageOptions) -> ZContext:
        """A ``ZContext`` sharing every field of ``base`` (``RuntimeParts.make_z_context``)."""
        shared = {f.name: getattr(base, f.name) for f in fields(WorkerContext)}
        return cls(**shared, stage=stage, stage_options=stage_options)


# ---------------------------------------------------------------------------------------------- helpers
def _payload(req: Request, cls: type[Any]) -> Any:
    """The decoded payload, or the typedef default when the request carries none (internal / test requests)."""
    return req.data if isinstance(req.data, cls) else cls()


def _run(*steps: Callable[[], object]) -> ErrorCluster:
    """Run ``steps`` in order; the first ``StageError`` stops the chain (LabVIEW error-in semantics)."""
    try:
        for step in steps:
            step()
    except StageError as exc:
        return exc.cluster()
    return NO_ERROR


def _query(ctx: ZContext, req: Request, read: Callable[[], Any], default: Any) -> None:
    """One query: its value, or ``default`` next to the error (``Index Array`` of an empty array, 08 §5.3)."""
    try:
        value, err = read(), NO_ERROR
    except StageError as exc:
        value, err = default, exc.cluster()
    finish_z(ctx, req, value, err)


def _restart_dh(stage: ZController) -> str:
    """``RMC?`` then ``MAC START DH`` unless ``DH`` or ``HOME`` runs (exact compare, 08 §10). Returns the answer."""
    running = stage.running_macros()
    if running not in KNOWN_MACROS:
        stage.mac_start(DH_MACRO)
    return running


def _reference_check(ctx: ZContext, z: ZStageSettings, label: str) -> ErrorCluster:
    """The reference check before a move (08 §6, §13.4 ``_check_reference``); returns the error chained into the
    move. ``implicit_z_reinit`` TRUE: ``check_axis_reference`` (``state.z_inited`` ← its ``Z INITED``; an info line
    after a re-initialisation). FALSE: ``FRF?`` only — FALSE → ``{TRUE, 5, "Z axis not referenced"}``, a failed
    query → its error; no re-initialisation."""
    if not ctx.compat.implicit_z_reinit:
        try:
            if ctx.stage.qfrf():
                return NO_ERROR
        except StageError as exc:
            return exc.cluster()
        return ErrorCluster(True, Code.GCS_UNREFERENCED, NOT_REFERENCED_TEXT)
    inited, warning, failure = ctx.stage.check_axis_reference(z)
    ctx.state.update(z_inited=inited)
    err = failure.cluster() if failure is not None else NO_ERROR
    if warning:
        ctx.events.info(err, SERVER_Z, REFERENCE_RESET_INFO.format(label=label))
    return err


def _in_range(z: ZStageSettings, target: float) -> bool:
    """Inclusive software range ``[min(Min, Max), max(Min, Max)]`` (LabVIEW ``Max & Min``, 08 §7); NaN is out."""
    lo, hi = sorted((z.min_pos, z.max_pos))
    return lo <= target <= hi


def _move(ctx: ZContext, z: ZStageSettings, target: float, err: ErrorCluster,
          send: Callable[[], None]) -> ErrorCluster:
    """Range check, then the move when nothing failed before; 9405 replaces any earlier error (08 §5.3)."""
    if not _in_range(z, target):
        return _OUT_OF_RANGE
    if err.status:
        return err
    try:
        send()
    except StageError as exc:
        return exc.cluster()
    ctx.state.update(z_pos=target)
    return NO_ERROR


# ---------------------------------------------------------------------------------------------- the 19 Z cases
def _z_setup(ctx: ZContext, req: Request) -> None:
    """``Z Setup`` (08 §4.1, 03 §7.2): set-up + normal speed; ``z_inited`` ← no error."""
    err = _run(lambda: ctx.stage.setup(ctx.state.z))
    ctx.state.update(z_inited=not err.status)
    finish_z(ctx, req, None, err)


def _z_get_serial(ctx: ZContext, req: Request) -> None:
    """``Z Get Serial`` (08 §5.3, 03 §7.3): no I/O; ``Type`` TRUE → identity, FALSE → serial number."""
    data: ZGetSerialData = _payload(req, ZGetSerialData)
    session = ctx.stage.session
    finish_z(ctx, req, session.identity if data.type else session.serial_number, NO_ERROR)


def _z_get_status(ctx: ZContext, req: Request) -> None:
    """``Z Get Status`` (08 §5.3, 03 §7.4): ``ERR?`` (reads and clears), ``RMC?``, ``MAC START DH`` when neither
    ``DH`` nor ``HOME`` runs (``restart_dh_macro``). BOOL = ``ERR?`` returned 0."""
    ok, running, err = False, "", NO_ERROR
    try:
        ctx.stage.check_error()
        ok = True
        running = ctx.stage.running_macros()
        if ctx.compat.restart_dh_macro and running not in KNOWN_MACROS:
            ctx.stage.mac_start(DH_MACRO)
    except StageError as exc:
        err = exc.cluster()
    ctx.state.update(running_macros=running, macro_running=running in KNOWN_MACROS)
    finish_z(ctx, req, ok, err)


def _z_get_error(ctx: ZContext, req: Request) -> None:
    """``Z Get Error`` (08 §5.3, §11): ``ERR?`` + ``GCSTranslateError``; STR = the GCS text or ``""``."""
    value, err = "", NO_ERROR
    try:
        ctx.stage.check_error()
    except StageError as exc:
        value, err = ctx.stage.gcs_error_text(exc.code), ctx.stage.translate(exc).cluster()
    finish_z(ctx, req, value, err)


def _z_set_speed(ctx: ZContext, req: Request) -> None:
    """``Z Set Speed`` (08 §5.3): ``VEL`` + ``ERR?``; the settings keep the INI speed (03 C14)."""
    data: ZSetSpeedData = _payload(req, ZSetSpeedData)
    finish_z(ctx, req, None, _run(lambda: ctx.stage.vel(data.speed_mm_s)))


def _z_get_speed(ctx: ZContext, req: Request) -> None:
    """``Z Get Speed`` (08 §5.3): ``VEL?``; 0.0 on error."""
    _query(ctx, req, ctx.stage.qvel, 0.0)


def _z_get_reference(ctx: ZContext, req: Request) -> None:
    """``Z Get Reference`` (08 §5.3): ``FRF?``; FALSE on error. Log label ``"Get Z Referenced"``."""
    _query(ctx, req, ctx.stage.qfrf, False)


def _z_do_reference(ctx: ZContext, req: Request) -> None:
    """``Z Do Reference`` (08 §5.3, Z13): ``FRF`` for the label ``"FRF"``, else ``FNL``; with
    ``do_reference_waits_ready`` ``#7`` polling first, then the ``#5`` wait (10 ms), then ``ERR?``; error translated.
    Log label ``"Get Z Referenced"`` (sic, kept: 08 Z15)."""
    data: ZReferenceData = _payload(req, ZReferenceData)
    stage, options = ctx.stage, ctx.stage_options
    if data.method == "FRF":
        start = stage.frf
    else:
        if data.method.startswith("#"):
            log.warning("Z Do Reference: reference method %s is not an item of the enum table: FNL used", data.method)
        start = stage.fnl
    steps: list[Callable[[], object]] = [start]
    if options.do_reference_waits_ready:
        steps.append(lambda: stage.wait_ready(options.ready_timeout_s))
    steps += [lambda: stage.general_wait(polling_ms=10, timeout_s=options.wait_timeout_s), stage.check_error]
    try:
        for step in steps:
            step()
        err = NO_ERROR
    except StageError as exc:
        err = stage.translate(exc).cluster()
    finish_z(ctx, req, None, err)


def _z_get_position(ctx: ZContext, req: Request) -> None:
    """``Z Get Position`` (08 §5.3): ``POS?``; 0.0 on error; ``z_pos`` written even on error."""
    try:
        value, err = ctx.stage.qpos(), NO_ERROR
    except StageError as exc:
        value, err = 0.0, exc.cluster()
    ctx.state.update(z_pos=value)
    finish_z(ctx, req, value, err)


def _z_move_absolute(ctx: ZContext, req: Request) -> None:
    """``Z Move Absolute`` (08 §5.3, §6, §7; 03 §7.11): reference check, inclusive range check (9405), ``MOV``. No wait,
    no ``ERR?`` (03 C13). After a failed reference check the range test sees 0.0 (chained ``Variant To Data``
    skipped, inferred)."""
    data: ZMoveAbsData = _payload(req, ZMoveAbsData)
    z = ctx.state.z
    err = _reference_check(ctx, z, "Move Absolute")
    target = 0.0 if err.status else data.pos_mm
    finish_z(ctx, req, None, _move(ctx, z, target, err, lambda: ctx.stage.mov(target)))


def _z_move_relative(ctx: ZContext, req: Request) -> None:
    """``Z Move Relative`` (08 §5.3, §6, §7; 03 §7.12): reference check, ``POS?`` (0.0 if it failed or was skipped),
    range check of ``POS? + move`` (9405), ``MVR move``. No wait, no ``ERR?``."""
    data: ZMoveRelData = _payload(req, ZMoveRelData)
    z = ctx.state.z
    err = _reference_check(ctx, z, "Move Relative")
    current = 0.0
    if not err.status:
        try:
            current = ctx.stage.qpos()
        except StageError as exc:
            err = exc.cluster()
    move = data.move_mm
    finish_z(ctx, req, None, _move(ctx, z, current + move, err, lambda: ctx.stage.mvr(move)))


def _z_wait_end_move(ctx: ZContext, req: Request) -> None:
    """``Z Wait End Move`` (08 §8): general wait; negative times → 0; timeout ``[stage] wait_timeout_s``."""
    data: ZWaitData = _payload(req, ZWaitData)
    err = _run(lambda: ctx.stage.general_wait(add_wait_ms=max(0, data.add_wait_ms),
                                              polling_ms=max(0, data.polling_cycle_ms),
                                              add_wait_only=data.add_wait_only,
                                              timeout_s=ctx.stage_options.wait_timeout_s))
    finish_z(ctx, req, None, err)


def _z_stop(ctx: ZContext, req: Request) -> None:
    """``Z Stop`` (08 §5.3, 03 §7.14): ``STP 1`` (error 10 stays pending); with ``restart_dh_macro`` ``RMC?`` and
    ``MAC START DH`` (``STP`` stopped it). The macro indicators are not updated."""
    steps: list[Callable[[], object]] = [ctx.stage.stp]
    if ctx.compat.restart_dh_macro:
        steps.append(lambda: _restart_dh(ctx.stage))
    finish_z(ctx, req, None, _run(*steps))


def _joystick(ctx: ZContext, req: Request, enable: bool) -> None:
    """``Z Enable/Disable Joystick`` (08 §9): ``JAX <id> <axis> 1`` then ``JON <id> 1|0`` + ``ERR?``."""
    data: ZJoystickData = _payload(req, ZJoystickData)
    err = _run(lambda: ctx.stage.jax(data.joystick_id, data.joystick_axis),
               lambda: ctx.stage.jon(data.joystick_id, enable))
    finish_z(ctx, req, None, err)


def _z_enable_joystick(ctx: ZContext, req: Request) -> None:
    """``Z Enable Joystick`` (08 §9)."""
    _joystick(ctx, req, True)


def _z_disable_joystick(ctx: ZContext, req: Request) -> None:
    """``Z Disable Joystick`` (08 §9): ``JAX`` is sent again."""
    _joystick(ctx, req, False)


def _z_joystick_enabled(ctx: ZContext, req: Request) -> None:
    """``Z Joystick Enabled ?`` (08 §9): BOOL = ``JON? <id>`` ≠ 0; FALSE on error."""
    data: ZJoystickData = _payload(req, ZJoystickData)
    _query(ctx, req, lambda: ctx.stage.qjon(data.joystick_id), False)


def _set_limit(ctx: ZContext, req: Request, field_name: str) -> None:
    """``Z Set Min/Max Limit`` (08 §5.3, 03 §7.17, C15): in memory only; a non-finite value is refused with
    ``{TRUE, 1, "<Command>: value must be finite"}`` and the limit is kept."""
    data: ZLimitData = _payload(req, ZLimitData)
    if not math.isfinite(data.value_mm):
        err = ErrorCluster(True, Code.LV_INVALID_INPUT, f"{req.command}: value must be finite")
    else:
        ctx.state.replace_z_settings(**{field_name: float(data.value_mm)})
        err = NO_ERROR
    finish_z(ctx, req, None, err)


def _z_set_min_limit(ctx: ZContext, req: Request) -> None:
    """``Z Set Min Limit``."""
    _set_limit(ctx, req, "min_pos")


def _z_set_max_limit(ctx: ZContext, req: Request) -> None:
    """``Z Set Max Limit``."""
    _set_limit(ctx, req, "max_pos")


def _z_cleanup(ctx: ZContext, req: Request) -> None:
    """``Z Cleanup`` (08 §5.3, Z9; 03 §7.18): close the link; with ``restart_dh_macro`` ``RMC?`` on the closed link
    fails (−4) and the ACK carries that error, as LabVIEW; FALSE → nothing after the close, success ACK. Session
    reset; ``z_inited`` ← FALSE (08 Z9 fix, not client-visible)."""
    steps: list[Callable[[], object]] = [ctx.stage.close]
    if ctx.compat.restart_dh_macro:
        steps.append(lambda: _restart_dh(ctx.stage))
    err = _run(*steps)
    ctx.stage.reset_session()
    ctx.state.update(z_inited=False)
    finish_z(ctx, req, None, err)


def _z_exit(ctx: ZContext, req: Request) -> Outcome:
    """``Exit`` (Z case, 03 §7.19): stop the loop; no reply, no log, no close (the shutdown chain closes the link)."""
    return STOP


Z_HANDLERS: HandlerTable = MappingProxyType({
    "Z Setup": _z_setup,
    "Z Get Serial": _z_get_serial,
    "Z Get Status": _z_get_status,
    "Z Get Error": _z_get_error,
    "Z Set Speed": _z_set_speed,
    "Z Get Speed": _z_get_speed,
    "Z Get Reference": _z_get_reference,
    "Z Do Reference": _z_do_reference,
    "Z Get Position": _z_get_position,
    "Z Move Absolute": _z_move_absolute,
    "Z Move Relative": _z_move_relative,
    "Z Wait End Move": _z_wait_end_move,
    "Z Stop": _z_stop,
    "Z Enable Joystick": _z_enable_joystick,
    "Z Disable Joystick": _z_disable_joystick,
    "Z Joystick Enabled ?": _z_joystick_enabled,
    "Z Set Min Limit": _z_set_min_limit,
    "Z Set Max Limit": _z_set_max_limit,
    "Z Cleanup": _z_cleanup,
    "Exit": _z_exit,
})
