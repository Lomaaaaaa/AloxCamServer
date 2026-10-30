"""``Command Data`` payload dataclasses, the payload codec and the registry of the 46 ``Camera Command`` items.

Spec: 04 §5 (command payloads, data modes, worked examples §5.7), 04 §7.1 (``Camera Init Data`` order), 04 §9.2
(``lvtypes`` sketch), 03 §2 (catalogue), 03 §5–§7 (extra strings and suffixes), 05 §10.2, 06 §2.1;
docs/answers/owner-answers.md §2 (confirmed cluster orders); design §3.2.5.

Each payload class carries ``LV_TYPEDEF`` (typedef label = ``[wire.layout]`` key), ``LV_LAYOUT`` (working cluster
order) and ``LV_BARE`` (a bare value, not a cluster). Enum fields hold item labels, defaulting to the typedef's
default item. Integer widths are I32 as assumed by 04 §7.2 A5. Payload instances are frozen: they travel between
threads inside :class:`aloxcam.dispatch.Request`.
"""

from __future__ import annotations

import dataclasses
import logging
from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, ClassVar

import numpy as np

from aloxcam.dispatch import CAMERA_WORKER, Z_WORKER, CommandSpec, DataMode, FinishOrder
from aloxcam.dispatch import ReplyKind as R
from aloxcam.errors import Code, ConfigError, ErrorCluster, ProtocolError
from aloxcam.lv.flatten import (
    ENUM_PREFIX,
    Layout,
    Reader,
    Writer,
    enum_id_of,
    flatten,
    parse_layout,
    unflatten,
    validate_layout,
)
from aloxcam.lv.path import LVPath
from aloxcam.lv.variant import EMPTY_VARIANT_HEADER, read_variant_header, synth_variant_header, write_variant
from aloxcam.settings import HIGH_MAG
from aloxcam.wire.enums import WireEnums

__all__ = [
    "CASES",
    "COMMANDS",
    "PAYLOAD_CLASSES",
    "CamLogData",
    "CameraAutofocusData",
    "CameraFlipData",
    "CameraGainData",
    "CameraGetAutofocusData",
    "CameraGetExposureData",
    "CameraGetGainData",
    "CameraGetImageData",
    "CameraGetLedData",
    "CameraInitData",
    "CameraOnlyData",
    "CameraRefocusData",
    "CameraReleaseData",
    "CameraSetExposureData",
    "CameraSetFlatFieldData",
    "CameraSetLedData",
    "CameraStartData",
    "CameraStatusData",
    "CameraStopData",
    "CameraStreamData",
    "CameraTriggerSoftData",
    "PayloadCodec",
    "ShowUIData",
    "ZExtraParamData",
    "ZGetSerialData",
    "ZJoystickData",
    "ZLimitData",
    "ZMoveAbsData",
    "ZMoveRelData",
    "ZReferenceData",
    "ZSetSpeedData",
    "ZWaitData",
    "payload_from_mapping",
    "payload_to_mapping",
]

log = logging.getLogger(__name__)

CT = ("camera_type", "enum:camera_type")


# ---------------------------------------------------------------------------------------------- camera payloads
@dataclass(frozen=True)
class CameraOnlyData:
    """Base of the nine single-field typedefs ``{Camera Type}`` (04 §5.2)."""

    camera_type: str = HIGH_MAG
    LV_TYPEDEF: ClassVar[str] = ""
    LV_LAYOUT: ClassVar[Layout] = (CT,)
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class CameraGetImageData(CameraOnlyData):
    """``typedef_ext_camera_getimage_data.ctl`` — ``Cam Get Image``."""

    LV_TYPEDEF = "Camera Get Image Data"


@dataclass(frozen=True)
class CameraStatusData(CameraOnlyData):
    """``typedef_ext_camera_status_data.ctl`` — ``Cam Get Acquisition Status``."""

    LV_TYPEDEF = "Camera Status Data"


@dataclass(frozen=True)
class CameraTriggerSoftData(CameraOnlyData):
    """``typedef_ext_camera_trigsoft_data.ctl`` — ``Cam Software Trigger``."""

    LV_TYPEDEF = "Camera Trigger Soft Data"


@dataclass(frozen=True)
class CameraGetExposureData(CameraOnlyData):
    """``typedef_ext_camera_getexposure_data.ctl`` — ``Cam Get Exposure``."""

    LV_TYPEDEF = "Camera Get Exposure Data"


@dataclass(frozen=True)
class CameraGetGainData(CameraOnlyData):
    """``typedef_ext_camera_getgain_data.ctl`` — ``Cam Get Gain``."""

    LV_TYPEDEF = "Camera Get Gain Data"


@dataclass(frozen=True)
class CameraStopData(CameraOnlyData):
    """``typedef_ext_camera_stop_data.ctl`` — ``Cam Stop``."""

    LV_TYPEDEF = "Camera Stop Data"


@dataclass(frozen=True)
class CameraReleaseData(CameraOnlyData):
    """``typedef_ext_camera_release_data.ctl`` — ``Cam Cleanup``."""

    LV_TYPEDEF = "Camera Release Data"


@dataclass(frozen=True)
class CameraGetAutofocusData(CameraOnlyData):
    """``typedef_ext_camera_getautofocusdata.ctl`` — ``Cam Get Autofocus Data`` / ``… Progress``."""

    LV_TYPEDEF = "Camera Get Autofocus Data"


@dataclass(frozen=True)
class CameraGetLedData(CameraOnlyData):
    """``typedef_ext_camera_getled_data.ctl`` — ``Cam Get Led``."""

    LV_TYPEDEF = "Camera Get LED Data"


@dataclass(frozen=True)
class CameraInitData:
    """``typedef_ext_camera_init_data.ctl`` — ``Cam Setup`` (04 §5.3). Panel order confirmed (owner-answers §2)."""

    camera_type: str = HIGH_MAG
    camera_name: str = ""                     # IMAQdx camera name
    number_of_buffers: int = 0                # ignored by the server (5 buffers, constant)
    continuous_mode: bool = False
    LV_TYPEDEF: ClassVar[str] = "Camera Init Data"
    LAYOUT_PANEL: ClassVar[Layout] = (CT, ("camera_name", "str"), ("number_of_buffers", "i32"),
                                      ("continuous_mode", "bool"))
    LAYOUT_UNBUNDLE: ClassVar[Layout] = (CT, ("continuous_mode", "bool"), ("number_of_buffers", "i32"),
                                         ("camera_name", "str"))    # old assumption, optional fallback only
    LV_LAYOUT: ClassVar[Layout] = LAYOUT_PANEL
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class CameraStartData:
    """``typedef_ext_camera_start_data.ctl`` — ``Cam Start``."""

    camera_type: str = HIGH_MAG
    triggered: bool = False
    LV_TYPEDEF: ClassVar[str] = "Camera Start Data"
    LV_LAYOUT: ClassVar[Layout] = (CT, ("triggered", "bool"))
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class CameraSetExposureData:
    """``typedef_ext_camera_setexposure_data.ctl`` — ``Cam Set Exposure``."""

    camera_type: str = HIGH_MAG
    exposure: float = 0.0
    LV_TYPEDEF: ClassVar[str] = "Camera Set Exposure Data"
    LV_LAYOUT: ClassVar[Layout] = (CT, ("exposure", "dbl"))
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class CameraGainData:
    """``typedef_ext_camera_gain_data.ctl`` — ``Cam Set Gain``."""

    camera_type: str = HIGH_MAG
    gain: float = 0.0
    LV_TYPEDEF: ClassVar[str] = "Camera Gain Data"
    LV_LAYOUT: ClassVar[Layout] = (CT, ("gain", "dbl"))
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class CameraSetFlatFieldData:
    """``typedef_ext_camera_setflatfield_data.ctl`` (``Camera Ext Set FlatField``) — ``Cam Set FlatField``."""

    camera_type: str = HIGH_MAG
    enable: bool = False
    ffc_path: LVPath = LVPath()
    dfc_path: LVPath = LVPath()
    correction_factor: float = 0.0
    LV_TYPEDEF: ClassVar[str] = "Camera Ext Set FlatField"
    LV_LAYOUT: ClassVar[Layout] = (CT, ("enable", "bool"), ("ffc_path", "path"), ("dfc_path", "path"),
                                   ("correction_factor", "dbl"))
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class CameraFlipData:
    """``typedef_ext_camera_flip_data.ctl`` — ``Cam Flip``."""

    camera_type: str = HIGH_MAG
    horizontal_flip: bool = False
    vertical_flip: bool = False
    LV_TYPEDEF: ClassVar[str] = "Camera Flip Data"
    LV_LAYOUT: ClassVar[Layout] = (CT, ("horizontal_flip", "bool"), ("vertical_flip", "bool"))
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class CameraSetLedData:
    """``typedef_ext_camera_setled_data.ctl`` — ``Cam Set Led``."""

    camera_type: str = HIGH_MAG
    led: bool = False
    LV_TYPEDEF: ClassVar[str] = "Camera Set LED Data"
    LV_LAYOUT: ClassVar[Layout] = (CT, ("led", "bool"))
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class CameraStreamData:
    """``typedef_ext_camera_stream_data.ctl`` — ``Cam Stream Next Image``."""

    camera_type: str = HIGH_MAG
    simulation: bool = False
    simulation_path: LVPath = LVPath()        # folder of PNG files
    simulation_timer_ms: int = 0              # U32
    acquisition_timeout_ms: int = 5000        # I32 (inferred)
    LV_TYPEDEF: ClassVar[str] = "Camera Stream Data"
    LV_LAYOUT: ClassVar[Layout] = (CT, ("simulation", "bool"), ("simulation_path", "path"),
                                   ("simulation_timer_ms", "u32"), ("acquisition_timeout_ms", "i32"))
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class CamLogData:
    """``typedef_ext_camera_log_data.ctl`` — ``Log`` (stored, never used: 03 C5)."""

    log: bool = False
    path: LVPath = LVPath()
    max_lines: int = 0                        # I32 (inferred)
    max_count: int = 0                        # I32 (inferred)
    LV_TYPEDEF: ClassVar[str] = "Cam Log Data"
    LV_LAYOUT: ClassVar[Layout] = (("log", "bool"), ("path", "path"), ("max_lines", "i32"), ("max_count", "i32"))
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class ShowUIData:
    """``Show UI``: a bare Boolean, not a cluster (04 §5.4)."""

    show: bool = False
    LV_TYPEDEF: ClassVar[str] = "Show UI"
    LV_LAYOUT: ClassVar[Layout] = (("show", "bool"),)
    LV_BARE: ClassVar[bool] = True


@dataclass(frozen=True)
class CameraAutofocusData:
    """``typedef_ext_camera_autofocus_data.ctl`` — ``Cam Autofocus`` / ``… Run`` (panel order, confirmed)."""

    camera_type: str = HIGH_MAG
    autofocus_speed: str = "Slow"
    autofocus_method: str = "FFT"
    duration_s: int = 0
    z_min: float = 0.0
    z_max: float = 0.0
    step: float = 0.0
    subsampling: float = 0.0                  # DBL (inferred, A7)
    roi: list[int] = field(default_factory=list)   # [] or [Left, Top, Right, Bottom]
    acquisition_timeout_ms: int = 5000        # not used by the server
    LV_TYPEDEF: ClassVar[str] = "Camera Autofocus Data"
    LV_LAYOUT: ClassVar[Layout] = (CT, ("autofocus_speed", "enum:autofocus_speed"),
                                   ("autofocus_method", "enum:autofocus_method"), ("duration_s", "i32"),
                                   ("z_min", "dbl"), ("z_max", "dbl"), ("step", "dbl"), ("subsampling", "dbl"),
                                   ("roi", "i32[]"), ("acquisition_timeout_ms", "i32"))
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class CameraRefocusData:
    """``typedef_ext_camera_refocus_data.ctl`` — ``Cam Refocus`` / ``Cam Refocus Run`` (panel order, confirmed)."""

    camera_type: str = HIGH_MAG
    autofocus_method: str = "FFT"
    refocus_method: str = "Fin"               # ignored by the server
    steps: int = 0
    step_mm: float = 0.0
    fin_roi_width: int = 0
    fin_roi_height: int = 0
    oxyd_x_pos: int = 0
    oxyd_y_pos: int = 0
    oxyd_diam: int = 0
    acquisition_timeout_ms: int = 5000        # not used by the server
    wait_ms: int = 0
    save_images: bool = False
    folder: LVPath = LVPath()
    LV_TYPEDEF: ClassVar[str] = "Camera Refocus Data"
    LV_LAYOUT: ClassVar[Layout] = (CT, ("autofocus_method", "enum:autofocus_method"),
                                   ("refocus_method", "enum:refocus_method"), ("steps", "i32"), ("step_mm", "dbl"),
                                   ("fin_roi_width", "i32"), ("fin_roi_height", "i32"), ("oxyd_x_pos", "i32"),
                                   ("oxyd_y_pos", "i32"), ("oxyd_diam", "i32"), ("acquisition_timeout_ms", "i32"),
                                   ("wait_ms", "i32"), ("save_images", "bool"), ("folder", "path"))
    LV_BARE: ClassVar[bool] = False


# ---------------------------------------------------------------------------------------------- Z payloads
@dataclass(frozen=True)
class ZGetSerialData:
    """``typedef_ext_z_getserial_data.ctl`` — ``Z Get Serial``: TRUE → Identity, FALSE → Serial Number."""

    type: bool = False
    LV_TYPEDEF: ClassVar[str] = "Z Stage Get Serial Data"
    LV_LAYOUT: ClassVar[Layout] = (("type", "bool"),)
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class ZSetSpeedData:
    """``typedef_ext_z_set_speed_data.ctl`` — ``Z Set Speed``."""

    speed_mm_s: float = 0.0
    LV_TYPEDEF: ClassVar[str] = "Z Stage Set Speed Data"
    LV_LAYOUT: ClassVar[Layout] = (("speed_mm_s", "dbl"),)
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class ZExtraParamData:
    """``typedef_ext_z_get_speed_data.ctl`` / ``…_stop_data.ctl`` ``{Extra Param}`` ("not used, reserved")."""

    extra_param: str = ""
    LV_TYPEDEF: ClassVar[str] = "Z Stage Get Speed Data"
    LV_LAYOUT: ClassVar[Layout] = (("extra_param", "str"),)
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class ZMoveAbsData:
    """``typedef_ext_z_move_absolute_data.ctl`` — ``Z Move Absolute``."""

    pos_mm: float = 0.0
    LV_TYPEDEF: ClassVar[str] = "Z Move Abs Data"
    LV_LAYOUT: ClassVar[Layout] = (("pos_mm", "dbl"),)
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class ZMoveRelData:
    """``typedef_ext_z_move_relative_data.ctl`` — ``Z Move Relative``."""

    move_mm: float = 0.0
    LV_TYPEDEF: ClassVar[str] = "Z Stage Move Rel Data"
    LV_LAYOUT: ClassVar[Layout] = (("move_mm", "dbl"),)
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class ZWaitData:
    """``typedef_ext_z_move_wait_data.ctl`` — ``Z Wait End Move`` (panel order, confirmed)."""

    add_wait_ms: int = 0
    polling_cycle_ms: int = 10
    add_wait_only: bool = False
    LV_TYPEDEF: ClassVar[str] = "Z Stage Wait Data"
    LV_LAYOUT: ClassVar[Layout] = (("add_wait_ms", "i32"), ("polling_cycle_ms", "i32"), ("add_wait_only", "bool"))
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class ZJoystickData:
    """``typedef_ext_z_joystick_data.ctl`` — ``Z Enable/Disable Joystick``, ``Z Joystick Enabled ?``."""

    joystick_id: int = 1
    joystick_axis: int = 1
    LV_TYPEDEF: ClassVar[str] = "Z Stage Joystick Data"
    LV_LAYOUT: ClassVar[Layout] = (("joystick_id", "i32"), ("joystick_axis", "i32"))
    LV_BARE: ClassVar[bool] = False


@dataclass(frozen=True)
class ZReferenceData:
    """``Z Do Reference``: a bare enum (FNL default, FRF; lenient: any other value runs FNL, 08 §5.3)."""

    method: str = "FNL"
    LV_TYPEDEF: ClassVar[str] = "Z Reference Method"
    LV_LAYOUT: ClassVar[Layout] = (("method", "enum:z_reference_method"),)
    LV_BARE: ClassVar[bool] = True


@dataclass(frozen=True)
class ZLimitData:
    """``Z Set Min Limit`` / ``Z Set Max Limit``: a bare DBL."""

    value_mm: float = 0.0
    LV_TYPEDEF: ClassVar[str] = "Z Limit"
    LV_LAYOUT: ClassVar[Layout] = (("value_mm", "dbl"),)
    LV_BARE: ClassVar[bool] = True


#: Every payload class (each LV_TYPEDEF is a ``[wire.layout]`` key).
PAYLOAD_CLASSES: tuple[type[Any], ...] = (
    CameraGetImageData, CameraStatusData, CameraTriggerSoftData, CameraGetExposureData, CameraGetGainData,
    CameraStopData, CameraReleaseData, CameraGetAutofocusData, CameraGetLedData, CameraInitData, CameraStartData,
    CameraSetExposureData, CameraGainData, CameraSetFlatFieldData, CameraFlipData, CameraSetLedData,
    CameraStreamData, CamLogData, ShowUIData, CameraAutofocusData, CameraRefocusData, ZGetSerialData, ZSetSpeedData,
    ZExtraParamData, ZMoveAbsData, ZMoveRelData, ZWaitData, ZJoystickData, ZReferenceData, ZLimitData,
)


# ---------------------------------------------------------------------------------------------- codec
def _bad(message: str) -> ConfigError:
    return ConfigError(Code.LV_INVALID_INPUT, message)


class PayloadCodec:
    """Decodes / encodes ``Command Data`` with the working (or overridden) cluster orders (04 §5, §7).

    Thread-safe after construction (immutable). ``layout_overrides`` maps typedef labels (``[wire.layout]`` keys,
    compared lower-cased) to layout text; ``templates`` maps command labels to captured variant headers (bytes or
    hex text, 02 §7.4).
    """

    def __init__(self, enums: WireEnums, layout_overrides: Mapping[str, str] = {}, *,
                 init_data_fallback: bool = False, encoding: str = "cp1252",
                 templates: Mapping[str, bytes] = {}) -> None:
        self.enums = enums
        self.init_data_fallback = init_data_fallback
        self.encoding = encoding
        by_typedef = {cls.LV_TYPEDEF.lower(): cls for cls in PAYLOAD_CLASSES}
        layouts: dict[type[Any], Layout] = {cls: cls.LV_LAYOUT for cls in PAYLOAD_CLASSES}
        for key, text in layout_overrides.items():
            cls = by_typedef.get(str(key).strip().lower())
            if cls is None:
                raise _bad(f"[wire.layout] {key!r}: no payload typedef has this label")
            layout = parse_layout(text)
            validate_layout(cls, layout)
            if cls.LV_BARE and len(layout) != 1:
                raise _bad(f"[wire.layout] {key!r}: a bare payload has exactly one field")
            layouts[cls] = layout
        for cls, layout in layouts.items():
            for attr, kind in layout:
                enum_id = enum_id_of(kind)
                if enum_id is not None and enum_id not in enums.tables:
                    raise _bad(f"layout of {cls.LV_TYPEDEF!r}, field {attr!r}: unknown enum id {enum_id!r}")
        self._layouts = MappingProxyType(layouts)
        self._headers = MappingProxyType({cls: synth_variant_header(layout, enums, bare=cls.LV_BARE)
                                          for cls, layout in layouts.items()})
        self._templates = MappingProxyType({name: _template_bytes(name, t) for name, t in templates.items()})

    def layout_of(self, cls: type[Any]) -> Layout:
        """The cluster order used for ``cls``."""
        try:
            return self._layouts[cls]
        except KeyError:
            layout: Layout = cls.LV_LAYOUT
            return layout

    def _init_layouts(self) -> tuple[Layout, ...]:
        primary = self.layout_of(CameraInitData)
        if not self.init_data_fallback:
            return (primary,)
        others = tuple(lay for lay in (CameraInitData.LAYOUT_PANEL, CameraInitData.LAYOUT_UNBUNDLE)
                       if lay != primary)
        return (primary,) + others

    def _decode_exact(self, cls: type[Any], data: bytes, layout: Layout) -> Any:
        r = Reader(data, encoding=self.encoding)
        obj = unflatten(cls, r, layout, self.enums)
        if r.remaining:
            raise ProtocolError(Code.UNFLATTEN_FAILED, f"{cls.LV_TYPEDEF}: {r.remaining} unexpected data bytes")
        return obj

    def decode(self, spec: CommandSpec, data: bytes) -> Any:
        """Decode ``Command Data`` (04 §5.1): STRICT must consume exactly ``len(data)`` (else
        :class:`ProtocolError`); NOT_READ / UNUSED return None without looking at the bytes.

        ``Camera Init Data``: the configured order first, then the other built-in order if ``init_data_fallback``
        (WARNING log when the other one matched, 04 §7.1).
        """
        if spec.data_mode is not DataMode.STRICT or spec.payload is None:
            return None
        cls = spec.payload
        if cls is not CameraInitData:
            return self._decode_exact(cls, data, self.layout_of(cls))
        layouts = self._init_layouts()
        first_error: ProtocolError | None = None
        for i, layout in enumerate(layouts):
            try:
                obj = self._decode_exact(cls, data, layout)
            except ProtocolError as exc:
                first_error = first_error or exc
                continue
            if i:
                log.warning("Camera Init Data matched the fallback field order %s, not the configured one "
                            "(04 §7.1)", [attr for attr, _ in layout])
            return obj
        assert first_error is not None
        raise first_error

    def encode(self, obj: Any) -> bytes:
        """The data part of ``obj`` (client / simulator side; unencodable text raises :class:`ValueError`)."""
        return flatten(obj, self.layout_of(type(obj)), self.enums, Writer(encoding=self.encoding))

    def variant_for(self, spec: CommandSpec, obj: Any | None) -> bytes:
        """A whole flattened variant: header (``templates[spec.name]`` if present, else synthetic) + data + U32 0.

        NOT_READ → ``EMPTY_VARIANT_HEADER`` + U32 0; UNUSED → ``ZExtraParamData()`` when ``obj`` is None;
        STRICT with ``obj`` None → the payload's defaults.
        """
        if spec.data_mode is DataMode.NOT_READ or spec.payload is None:
            return write_variant(self._templates.get(spec.name, EMPTY_VARIANT_HEADER), b"")
        cls: type[Any] = spec.payload
        if obj is None:
            obj = cls()
        header = self._templates.get(spec.name) or self._headers.get(cls)
        if header is None:
            header = synth_variant_header(self.layout_of(cls), self.enums, bare=getattr(cls, "LV_BARE", False))
        return write_variant(header, self.encode(obj))


def _template_bytes(name: str, template: bytes | str) -> bytes:
    raw = bytes.fromhex("".join(template.split())) if isinstance(template, str) else bytes(template)
    r = Reader(raw)
    try:
        read_variant_header(r)
    except ProtocolError as exc:
        raise _bad(f"variant template of {name!r}: {exc.source}") from None
    if r.remaining:
        raise _bad(f"variant template of {name!r}: {r.remaining} bytes after the top-level type list")
    return raw


# ---------------------------------------------------------------------------------------------- mappings
def _from_plain(kind: str, value: Any, where: str) -> Any:
    if kind.startswith(ENUM_PREFIX):
        if isinstance(value, bool) or not isinstance(value, (str, int)):
            raise ValueError(f"{where}: enum label or integer code expected, got {value!r}")
        return value if isinstance(value, str) else f"#{value}"
    if kind == "path":
        if isinstance(value, LVPath):
            return value
        if isinstance(value, str):
            return LVPath.parse_user(value)
        if isinstance(value, Mapping):
            return LVPath(int(value.get("kind", 0)), tuple(str(c) for c in value.get("components", ())))
        raise ValueError(f"{where}: path text or {{kind, components}} expected, got {value!r}")
    if kind == "bool":
        if not isinstance(value, (bool, int)):
            raise ValueError(f"{where}: boolean expected, got {value!r}")
        return bool(value)
    if kind in ("sgl", "dbl"):
        return float(value)
    if kind == "str":
        return str(value)
    if kind == "i32[]":
        return [int(v) for v in value]
    if kind == "dbl[][]":
        return np.asarray(value, dtype=np.float64)
    if kind == "err":
        if isinstance(value, ErrorCluster):
            return value
        return ErrorCluster(bool(value.get("status", False)), int(value.get("code", 0)), str(value.get("source", "")))
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{where}: integer expected, got {value!r}")
    return value


def payload_from_mapping(cls: type[Any], mapping: Mapping[str, Any]) -> Any:
    """Build a payload from plain values (replay scripts, TOML): paths accept text (:meth:`LVPath.parse_user`) or
    ``{kind, components}``; enums accept labels or integer codes (→ ``"#<n>"``); missing keys keep their defaults;
    unknown keys raise :class:`ValueError`."""
    kinds = dict(cls.LV_LAYOUT)
    unknown = set(mapping) - set(kinds)
    if unknown:
        raise ValueError(f"{cls.__name__}: unknown field(s) {', '.join(sorted(unknown))}")
    values = {name: _from_plain(kinds[name], value, f"{cls.__name__}.{name}") for name, value in mapping.items()}
    return cls(**values)


def _to_plain(value: Any) -> Any:
    if isinstance(value, LVPath):
        return {"kind": value.kind, "components": list(value.components)}
    if isinstance(value, ErrorCluster):
        return {"status": value.status, "code": value.code, "source": value.source}
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (list, tuple)):
        return [_to_plain(v) for v in value]
    return value


def payload_to_mapping(obj: Any) -> dict[str, Any]:
    """Plain values of a payload (JSON-friendly): paths as ``{kind, components}``, arrays as lists."""
    return {f.name: _to_plain(getattr(obj, f.name)) for f in dataclasses.fields(obj)}


# ---------------------------------------------------------------------------------------------- registry
_S, _N, _U = DataMode.STRICT, DataMode.NOT_READ, DataMode.UNUSED
_A, _RAW, _Z, _CUSTOM = FinishOrder.A, FinishOrder.RAW_FIRST, FinishOrder.Z, FinishOrder.CUSTOM
_CAM, _ZW = CAMERA_WORKER, Z_WORKER

_REGISTRY: tuple[CommandSpec, ...] = (
    CommandSpec("Get Heartbeat", _CAM, None, _N, R.U32, _RAW, "Heartbeat", "Get Heartbeat", spec_ref="03 §5.1"),
    CommandSpec("Show UI", _CAM, ShowUIData, _S, R.ACK, _A, "Show UI", "Cam Show UI", spec_ref="03 §5.2"),
    CommandSpec("Log", _CAM, CamLogData, _S, R.ACK, _A, "Log", "Cam Log", spec_ref="03 §5.3"),
    CommandSpec("Exit", _CAM, None, _N, R.NONE, _CUSTOM, "", None, spec_ref="03 §5.4"),
    CommandSpec("Cam Setup", _CAM, CameraInitData, _S, R.ACK, _A, "Setup", None, spec_ref="05 §4.2"),
    CommandSpec("Cam Start", _CAM, CameraStartData, _S, R.ACK, _A, "Start", None, spec_ref="05 §4.4"),
    CommandSpec("Cam Stop", _CAM, CameraStopData, _S, R.ACK, _A, "Stop", "Cam stop", spec_ref="05 §4.5"),
    CommandSpec("Cam Cleanup", _CAM, CameraReleaseData, _S, R.ACK, _A, "Cleanup", "Cam Cleanup", spec_ref="05 §4.6"),
    CommandSpec("Cam Get Acquisition Status", _CAM, CameraStatusData, _S, R.BOOL, _A, "Get Status",
                "Cam Acquisition status", spec_ref="05 §4.7"),
    CommandSpec("Cam Software Trigger", _CAM, CameraTriggerSoftData, _S, R.ACK, _A, "Soft Trigger", "",
                spec_ref="05 §4.8"),
    CommandSpec("Cam Set Exposure", _CAM, CameraSetExposureData, _S, R.ACK, _A, "Set Exposure", "",
                spec_ref="05 §5.2"),
    CommandSpec("Cam Get Exposure", _CAM, CameraGetExposureData, _S, R.DBL, _A, "Get Exposure", None,
                spec_ref="05 §5.3"),
    CommandSpec("Cam Set Gain", _CAM, CameraGainData, _S, R.ACK, _A, "Set Gain", None, spec_ref="05 §5.4"),
    CommandSpec("Cam Get Gain", _CAM, CameraGetGainData, _S, R.DBL, _A, "Get Gain", None, spec_ref="05 §5.4"),
    CommandSpec("Cam Stream Next Image", _CAM, CameraStreamData, _S, R.IMAGE, _CUSTOM, "Get Image Start", None,
                spec_ref="05 §6.2"),
    CommandSpec("Cam Get Image", _CAM, CameraGetImageData, _S, R.NONE, _CUSTOM, "", "Cam Get Image",
                spec_ref="05 §6.5"),
    CommandSpec("Cam Get Images Array", _CAM, None, _N, R.NONE, _CUSTOM, "", None, spec_ref="03 §5.17"),
    CommandSpec("Cam Flip", _CAM, CameraFlipData, _S, R.ACK, _A, "Flip", "Cam Flip", spec_ref="05 §7.2"),
    CommandSpec("Cam Set FlatField", _CAM, CameraSetFlatFieldData, _S, R.ACK, _A, "Set FlatField", "Cam Set Field",
                spec_ref="05 §7.3"),
    CommandSpec("Cam Set Led", _CAM, CameraSetLedData, _S, R.ACK, _RAW, "Set LED", "Cam Set Led", spec_ref="05 §9.2"),
    CommandSpec("Cam Get Led", _CAM, CameraGetLedData, _S, R.BOOL, _RAW, "Get LED", "Cam Get Led", spec_ref="05 §9.3"),
    CommandSpec("Cam Autofocus", _CAM, CameraAutofocusData, _S, R.ACK, _A, "Autofocus", "Cam Autofocus",
                spec_ref="06 §3.1"),
    CommandSpec("Cam Refocus", _CAM, CameraRefocusData, _S, R.ACK, _A, "Refocus", "Cam Refocus", spec_ref="06 §3.1"),
    CommandSpec("Cam Autofocus Run", _CAM, CameraAutofocusData, _S, R.NONE, _CUSTOM, "Autofocus Run", "Autofocus",
                client_routable=False, spec_ref="06 §3.2"),
    CommandSpec("Cam Refocus Run", _CAM, CameraRefocusData, _S, R.NONE, _CUSTOM, "Refocus Run", "Cam Refocus Run",
                client_routable=False, spec_ref="06 §3.2"),
    CommandSpec("Cam Get Autofocus Data", _CAM, CameraGetAutofocusData, _S, R.DBL2D, _RAW, "Autofocus Data",
                "Cam Get Autofocus Data", spec_ref="06 §3.3"),
    CommandSpec("Cam Get Autofocus Progress", _CAM, CameraGetAutofocusData, _S, R.DBL, _RAW, "Autofocus Progress",
                "Cam Get Autofocus Progress", spec_ref="06 §3.3"),
    CommandSpec("Z Setup", _ZW, None, _N, R.ACK, _Z, "Setup", None, spec_ref="08 §4.1"),
    CommandSpec("Z Get Serial", _ZW, ZGetSerialData, _S, R.STR, _Z, "Get Serial", None, spec_ref="08 §5.3"),
    CommandSpec("Z Get Status", _ZW, None, _N, R.BOOL, _Z, "Get Status", None, spec_ref="08 §5.3"),
    CommandSpec("Z Get Error", _ZW, None, _N, R.STR, _Z, "Get Error", None, spec_ref="08 §5.3"),
    CommandSpec("Z Set Speed", _ZW, ZSetSpeedData, _S, R.ACK, _Z, "Set Speed", None, spec_ref="08 §5.3"),
    CommandSpec("Z Get Speed", _ZW, ZExtraParamData, _U, R.DBL, _Z, "Get Speed", None, spec_ref="08 §5.3"),
    CommandSpec("Z Get Reference", _ZW, ZExtraParamData, _U, R.BOOL, _Z, "Get Z Referenced", None,
                spec_ref="08 §5.3"),
    CommandSpec("Z Do Reference", _ZW, ZReferenceData, _S, R.ACK, _Z, "Get Z Referenced", None, spec_ref="08 §5.3"),
    CommandSpec("Z Get Position", _ZW, ZExtraParamData, _U, R.DBL, _Z, "Get Position", None, spec_ref="08 §5.3"),
    CommandSpec("Z Move Absolute", _ZW, ZMoveAbsData, _S, R.ACK, _Z, "Move Absolute", None, spec_ref="08 §5.3, §6"),
    CommandSpec("Z Move Relative", _ZW, ZMoveRelData, _S, R.ACK, _Z, "Move Relative", None, spec_ref="08 §5.3, §6"),
    CommandSpec("Z Wait End Move", _ZW, ZWaitData, _S, R.ACK, _Z, "Wait End Move", None, spec_ref="08 §8"),
    CommandSpec("Z Stop", _ZW, ZExtraParamData, _U, R.ACK, _Z, "Stop", None, spec_ref="08 §5.3"),
    CommandSpec("Z Enable Joystick", _ZW, ZJoystickData, _S, R.ACK, _Z, "Enable Joystick", None, spec_ref="08 §9"),
    CommandSpec("Z Disable Joystick", _ZW, ZJoystickData, _S, R.ACK, _Z, "Disable Joystick", None, spec_ref="08 §9"),
    CommandSpec("Z Joystick Enabled ?", _ZW, ZJoystickData, _S, R.BOOL, _Z, "Is Joystick Enabled ?", None,
                spec_ref="08 §9"),
    CommandSpec("Z Set Min Limit", _ZW, ZLimitData, _S, R.ACK, _Z, "Z Set Min Limit", None, spec_ref="08 §5.3"),
    CommandSpec("Z Set Max Limit", _ZW, ZLimitData, _S, R.ACK, _Z, "Z Set Max Limit", None, spec_ref="08 §5.3"),
    CommandSpec("Z Cleanup", _ZW, None, _N, R.ACK, _Z, "Cleanup", None, spec_ref="08 §5.3"),
)

#: The 46 commands of design §3.2.5, in table order.
COMMANDS: Mapping[str, CommandSpec] = MappingProxyType({spec.name: spec for spec in _REGISTRY})
#: label → worker for every command that has a handler (excludes the two internal ``… Run`` commands).
CASES: Mapping[str, str] = MappingProxyType({spec.name: spec.worker for spec in _REGISTRY if spec.client_routable})
