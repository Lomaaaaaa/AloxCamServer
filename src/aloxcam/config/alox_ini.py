"""``<settings>/alox.ini`` → :class:`aloxcam.settings.AloxSettings`: the 53 ``Read Key`` calls of stage 2.

Spec: 09 §5.1–§5.3 (key schema, defaults, derived values), §5.6 (reference file), §6.6 (tolerant loader pattern),
D2/D4/D5/D6/D7; 01 §3.4 (stage 2); 05 §9.1 (lights); design §3.3.3.

Each key is read on its own: an invalid value takes that key's default plus a warning, and never affects another key
(D4 fix: the LabVIEW error chains of 09 §5.4 are not reproduced). A missing file gives every default plus a warning and
is not created (D2). The file is never written (Q23).
"""

from __future__ import annotations

import logging
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any

from aloxcam.config.niconfig import ConfigTypeInvalid, NIConfig
from aloxcam.lv.path import localize_path_text
from aloxcam.settings import (
    PLUG_GIGE,
    PLUG_USB,
    AloxSettings,
    CameraOptions,
    CameraSettings,
    LightSettings,
    ZStageSettings,
)

__all__ = ["ALOX_INI_KEYS", "ALOX_INI_NAME", "BAUD_BY_INDEX", "IniKey", "load_alox_ini"]

log = logging.getLogger(__name__)

ALOX_INI_NAME = "alox.ini"


@dataclass(frozen=True)
class IniKey:
    """One ``Read Key`` call of stage 2 (09 §5.2): ``kind`` is the NI instance (the key's suffix), ``default`` the
    INI default (before derivation), ``target`` the dotted :class:`AloxSettings` field it feeds."""

    section: str
    key: str
    kind: str
    default: Any
    target: str


_HM = "hardware.camera.highmag"
_LM = "hardware.camera.lowmag"
_CAM = "hardware.camera"
_Z = "hardware.z"


def _camera_keys(section: str, cam: str, *, low_mag: bool) -> tuple[IniKey, ...]:
    """Frames 1 (HM, 19 keys) and 2 (LM, 20 keys) of stage 2, in 09 §5.2 order."""
    rows: list[tuple[str, str, Any, str]] = [
        ("name_STR", "STR", "LowMag" if low_mag else "HighMag", "camera_name"),
        ("buffers_I32", "I32", 5, "buffers"),
        ("timeout_I32", "I32", 5000, "timeout"),
        ("flatfieldcorrection_BOOL", "BOOL", False, "flat_field"),
        ("flatfieldimage_PATH", "PATH", "", "flat_field_path"),
        ("darkfieldimage_PATH", "PATH", "", "dark_current_path"),
        ("flatfieldfactor_DBL", "DBL", 1.0, "flat_field_factor"),
        ("exposure_STR", "STR", "CameraAttributes::Exposure::Value", "exposure_attribute"),
        ("gain_STR", "STR", "CameraAttributes::Gain::Value", "gain_attribute"),
        ("exposuretype_STR", "STR", "DBL", "exposure_type"),
        ("gaintype_STR", "STR", "I64", "gain_type"),
        ("hflip_BOOL", "BOOL", False, "horizontal_flip"),
        ("vflip_BOOL", "BOOL", False, "vertical_flip"),
        ("framerate_STR", "STR", "CameraAttributes::AcquisitionControl::AcquisitionFrameRate", "framerate_attribute"),
        ("frameratetype_STR", "STR", "DBL", "framerate_type"),
    ]
    if low_mag:                                         # D5: never read for High Mag
        rows.append(("frameratemax_DBL", "DBL", 24.6, "max_framerate"))
    rows += [
        ("LimitMinExposure_DBL", "DBL", 177.0, "exposure_lower_limit"),
        ("waitled_U32", "U32", 0, "wait_led_ms"),
        ("invertled_BOOL", "BOOL", low_mag, "invert_led"),
        ("cameraconnectortype_STR", "STR", "gige", "plug_type"),
    ]
    return tuple(IniKey(section, key, kind, default, f"{cam}.{field}") for key, kind, default, field in rows)


#: Exactly the 53 ``Read Key`` calls of 09 §5.2, in the order of 09 §5.1 frames 0, 1, 2, 3, 5 (design §3.3.3).
ALOX_INI_KEYS: tuple[IniKey, ...] = (
    IniKey(_CAM, "port_I32", "I32", 503, "port"),
    IniKey("general", "core_I32", "I32", 1, "core"),
    IniKey("log", "maxlines_I32", "I32", 1000, "max_lines"),
    *_camera_keys(_HM, "high_mag", low_mag=False),
    *_camera_keys(_LM, "low_mag", low_mag=True),
    IniKey(_Z, "comport_I32", "I32", 2, "z.com"),
    IniKey(_Z, "baudrate_I32", "I32", 3, "z.baudrate"),
    IniKey(_Z, "zaxis_I32", "I32", 1, "z.axis"),
    IniKey(_Z, "speed_DBL", "DBL", 10.0, "z.speed"),
    IniKey(_Z, "slowspeed_DBL", "DBL", 1.0, "z.slow_speed"),
    IniKey(_Z, "zmin_DBL", "DBL", 0.0, "z.min_pos"),
    IniKey(_Z, "zmax_DBL", "DBL", 20.0, "z.max_pos"),
    IniKey(_HM, "setLEDline_I32", "I32", 0, "light.hm_led_line"),
    IniKey(_HM, "setBULBline_I32", "I32", 0, "light.hm_bulb_line"),
    IniKey("general", "setLEDcam_I32", "I32", 0, "light.set_led_cam"),
    IniKey(_CAM, "lightStatusInverter_BOOL", "BOOL", True, "light.led_status_inverter"),
)

#: ``baudrate_I32`` is an index (09 §5.3); any other value gives 9600 (D7, warned).
BAUD_BY_INDEX: Mapping[int, int] = MappingProxyType({0: 9600, 1: 19200, 2: 38400, 3: 115200})


def _read(cfg: NIConfig, k: IniKey) -> Any:
    if k.kind == "I32":
        return cfg.read_int(k.section, k.key, k.default)
    if k.kind == "U32":
        return cfg.read_int(k.section, k.key, k.default, unsigned=True)
    if k.kind == "DBL":
        return cfg.read_float(k.section, k.key, k.default)
    if k.kind == "BOOL":
        return cfg.read_bool(k.section, k.key, k.default)
    if k.kind == "STR":
        return cfg.read_str(k.section, k.key, k.default)
    if k.kind == "PATH":
        return cfg.read_path(k.section, k.key, k.default)
    raise ValueError(f"unknown NI key kind {k.kind!r}")


def _plug_type(text: str) -> str:
    """``cameraconnectortype_STR`` (09 §5.3, D6 fix): ``usb`` in any case → ``USB``, anything else ``GiGe``."""
    return PLUG_USB if text.lower() == "usb" else PLUG_GIGE


def load_alox_ini(path: Path, camera_options: CameraOptions, *, warn: Callable[[str], None]) -> AloxSettings:
    """Load ``alox.ini`` (09 §5) into :class:`AloxSettings`; never raises for INI content.

    Rules (design §3.3.3): missing file → all defaults + warning, not created (D2); invalid value → that key's default
    + warning (D4); ``port_I32`` outside 0…65535 → 503 + warning; ``baudrate_I32`` index → rate, other values → 9600 +
    warning (D7); ``cameraconnectortype_STR`` case-insensitive (D6); paths localized, unmappable → None + warning; HM
    ``max_framerate`` = ``camera_options.hm_max_framerate`` (D5); lights derived per 09 §5.3, with
    ``bulb_status_inverter`` from ``camera_options``.
    """
    path = Path(path)
    try:
        cfg = NIConfig.load(path)
    except FileNotFoundError:
        warn(f"{path}: not found; every alox.ini key takes its default (the file is not created)")
        cfg = NIConfig.parse("")
    except OSError as exc:
        warn(f"{path}: cannot be read ({exc}); every alox.ini key takes its default")
        cfg = NIConfig.parse("")

    values: dict[str, Any] = {}
    for k in ALOX_INI_KEYS:
        try:
            value = _read(cfg, k)
        except ConfigTypeInvalid:
            warn(f"{path}: [{k.section}] {k.key} = {cfg.raw(k.section, k.key)!r} is not a valid {k.kind}; "
                 f"default {k.default!r} used")
            value = k.default
        if cfg.raw(k.section, k.key) is None:
            log.debug("%s: [%s] %s missing; default %r used", path, k.section, k.key, k.default)
        values[k.target] = value
    if cfg.raw(_HM, "frameratemax_DBL") is not None:
        log.debug("%s: [%s] frameratemax_DBL is not read for High Mag (09 D5); [camera] hm_max_framerate = %r used",
                  path, _HM, camera_options.hm_max_framerate)

    port = values["port"]
    if not 0 <= port <= 0xFFFF:
        warn(f"{path}: [{_CAM}] port_I32 = {port} is outside 0...65535; default 503 used")
        port = 503
    index = values["z.baudrate"]
    baudrate = BAUD_BY_INDEX.get(index)
    if baudrate is None:
        warn(f"{path}: [{_Z}] baudrate_I32 = {index} is not an index 0-3 (0=9600, 1=19200, 2=38400, 3=115200); "
             "9600 used as LabVIEW does")
        baudrate = 9600

    def camera(cam: str, section: str, **extra: Any) -> CameraSettings:
        fields = {target.partition(".")[2]: value for target, value in values.items() if target.startswith(cam + ".")}
        fields["plug_type"] = _plug_type(fields["plug_type"])
        for name, key in (("flat_field_path", "flatfieldimage_PATH"), ("dark_current_path", "darkfieldimage_PATH")):
            text = fields[name]
            fields[name] = localize_path_text(text)
            if text and fields[name] is None:
                warn(f"{path}: [{section}] {key} = {text!r} cannot be mapped to a local path "
                     "(set ALOX_DRIVE_<LETTER>); empty path used")
        fields.update(extra)
        return CameraSettings(**fields)

    high_mag = camera("high_mag", _HM, max_framerate=camera_options.hm_max_framerate)
    low_mag = camera("low_mag", _LM)
    set_led_cam = values["light.set_led_cam"]
    use_lm = set_led_cam != 0
    light = LightSettings(
        set_led_cam=set_led_cam,
        camera_sort_led=(low_mag if use_lm else high_mag).plug_type,
        connector="LM" if use_lm else "HM",
        hm_led_line=values["light.hm_led_line"],
        hm_bulb_line=values["light.hm_bulb_line"],
        led_status_inverter=values["light.led_status_inverter"],
        bulb_status_inverter=camera_options.bulb_status_inverter,
    )
    z = ZStageSettings(com=values["z.com"], baudrate=baudrate, axis=values["z.axis"], speed=values["z.speed"],
                       slow_speed=values["z.slow_speed"], min_pos=values["z.min_pos"], max_pos=values["z.max_pos"])
    return AloxSettings(port=port, core=values["core"], max_lines=values["max_lines"], high_mag=high_mag,
                        low_mag=low_mag, light=light, z=z)
