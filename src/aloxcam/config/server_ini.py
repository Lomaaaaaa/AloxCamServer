"""``<settings>/alox_camera_server.ini`` → :class:`aloxcam.settings.ServerConfig`, including the ``[compat]`` registry.

Spec: 10 §6 (``[compat]`` registry, ``alox.ini`` never written), 02 §11.3; design §3.3.4, §6 and Appendix A (template).

The file is Python-only (never read by LabVIEW) and read with the NI rules of :mod:`aloxcam.config.niconfig`. A missing
file gives :class:`ServerConfig` defaults; an invalid value gives the field's default plus a warning; an unknown
section or key is warned and ignored. Nothing here raises for file content: ``[wire.items]``, ``[wire.size]`` and
``[wire.layout]`` values are validated later by :mod:`aloxcam.wire.enums` / :mod:`aloxcam.wire.messages`.
"""

from __future__ import annotations

import codecs
import dataclasses
import logging
import math
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from aloxcam.config.niconfig import ConfigTypeInvalid, NIConfig
from aloxcam.lv.path import localize_path_text
from aloxcam.settings import (
    CameraOptions,
    CompatSwitches,
    ImageOptions,
    LoggingOptions,
    ServerConfig,
    ServerOptions,
    SimulationOptions,
    StageOptions,
    WireOptions,
)

__all__ = ["SERVER_INI_NAME", "SERVER_INI_TEMPLATE", "load_server_config", "write_template"]

log = logging.getLogger(__name__)

SERVER_INI_NAME = "alox_camera_server.ini"

#: Design Appendix A, verbatim; :func:`load_server_config` turns it into ``ServerConfig()`` without a warning.
SERVER_INI_TEMPLATE = """\
; alox_camera_server.ini - Python-only settings of the ALOX camera server (never read by LabVIEW).
; NI INI rules (docs/09 6): case-insensitive, first occurrence wins, ';' starts a comment.
[server]
port =
io_timeout_s = 5.0
max_payload_mb = 256
stream_poll_ms = 1
tick_source = monotonic
shutdown_timeout_s = 10
commands_history = 500
error_history = 1000
temperature_history = 10000

[compat]
send_error_replies = TRUE
reply_unknown_commands = FALSE
listen_address = 127.0.0.1
inbound_pause_ms = 0
close_on_extra_heartbeat_data = TRUE
exposure_sets_max_framerate = TRUE
strict_new_frame = FALSE
request_buffer_zero = TRUE
temperature_error_fails_image = TRUE
flatfield_check_on_set = FALSE
focus_reject_concurrent_start = FALSE
focus_validate_params = FALSE
af_check_z_limits = FALSE
restart_dh_macro = TRUE
implicit_z_reinit = TRUE

[wire]
enums_file =
encoding = cp1252
init_data_fallback = FALSE

[wire.items]
; camera_command = Get Heartbeat, Cam Setup, Cam Start, ...

[wire.size]
; camera_command = 2

[wire.layout]
; Camera Init Data = camera_type:enum:camera_type, camera_name:str, number_of_buffers:i32, continuous_mode:bool

[image]
codec = provisional
nivision_library =

[logging]
append_errors = TRUE
fix_info_labels = TRUE
decimal_separator = .
create_log_dirs = TRUE
encoding = cp1252
diag_level = INFO
diag_file_stem = camera_server_diag

[camera]
hm_max_framerate = 0
bulb_status_inverter = FALSE

[stage]
always_reference = TRUE
reference_move = FNL
ready_timeout_s = 300
wait_timeout_s = 300
do_reference_waits_ready = TRUE
startup_macro_check = FALSE
port_map =

[simulation]
enabled = FALSE
camera_names =
image_width = 640
image_height = 480
pixel_type = U8
frame_rate = 10
max_frame_rate = 100
temperature_c = 35.0
stage_min_mm = 0
stage_max_mm = 25
stage_initially_referenced = FALSE
stage_reference_s = 0.2
stage_identity = "(c) aloxcam SimStage, C-663-SIM, 0000000001, 0.0.0"
stage_dh_macro_s = 0
"""

_MIB = 2**20
_LOG_LEVELS = ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL")


# ---------------------------------------------------------------------------------------------- value checks
# A check returns the (normalized) value or raises ValueError with the reason.
def _choice(*allowed: str, upper: bool = False) -> Callable[[Any], Any]:
    def check(value: Any) -> Any:
        v = str(value).strip()
        v = v.upper() if upper else v.lower()
        if v not in allowed:
            raise ValueError(f"expected one of {', '.join(allowed)}")
        return v
    return check


def _at_least(lo: float, *, strict: bool = False) -> Callable[[Any], Any]:
    def check(value: Any) -> Any:
        if strict and not value > lo:
            raise ValueError(f"must be > {lo:g}")
        if not strict and not value >= lo:
            raise ValueError(f"must be >= {lo:g}")
        return value
    return check


def _port(value: Any) -> Any:
    if not 0 <= value <= 0xFFFF:
        raise ValueError("must be 0...65535")
    return value


def _not_empty(value: Any) -> Any:
    if not str(value).strip():
        raise ValueError("must not be empty")
    return str(value).strip()


def _encoding(value: Any) -> Any:
    try:
        codecs.lookup(value)
    except LookupError:
        raise ValueError("unknown encoding") from None
    return value


def _one_char(value: Any) -> Any:
    if len(value) != 1:
        raise ValueError("must be a single character")
    return value


@dataclass(frozen=True)
class _Key:
    """One INI key: ``kind`` selects the reader (bool, int, float, str, path, list); ``attr`` the dataclass field."""

    key: str
    kind: str
    attr: str = ""
    check: Callable[[Any], Any] | None = None

    @property
    def field(self) -> str:
        return self.attr or self.key


#: Fixed sections: section → (options dataclass, keys). ``[wire.items]``, ``[wire.size]`` and ``[wire.layout]`` have
#: free keys (handled by :func:`_wire_tables`).
_SCHEMA: dict[str, tuple[type[Any], tuple[_Key, ...]]] = {
    "server": (ServerOptions, (
        _Key("port", "port"),                                       # special: empty → None
        _Key("io_timeout_s", "float", check=_at_least(0, strict=True)),
        _Key("max_payload_mb", "float", "max_payload_bytes", _at_least(0, strict=True)),   # special: × 2**20
        _Key("stream_poll_ms", "float", check=_at_least(0)),
        _Key("tick_source", "str", check=_choice("monotonic", "windows")),
        _Key("shutdown_timeout_s", "float", check=_at_least(0)),
        _Key("commands_history", "int", check=_at_least(0)),
        _Key("error_history", "int", check=_at_least(0)),
        _Key("temperature_history", "int", check=_at_least(0)),
    )),
    "compat": (CompatSwitches, (
        _Key("send_error_replies", "bool"),
        _Key("reply_unknown_commands", "bool"),
        _Key("listen_address", "str", check=_not_empty),
        _Key("inbound_pause_ms", "int", check=_at_least(0)),
        _Key("close_on_extra_heartbeat_data", "bool"),
        _Key("exposure_sets_max_framerate", "bool"),
        _Key("strict_new_frame", "bool"),
        _Key("request_buffer_zero", "bool"),
        _Key("temperature_error_fails_image", "bool"),
        _Key("flatfield_check_on_set", "bool"),
        _Key("focus_reject_concurrent_start", "bool"),
        _Key("focus_validate_params", "bool"),
        _Key("af_check_z_limits", "bool"),
        _Key("restart_dh_macro", "bool"),
        _Key("implicit_z_reinit", "bool"),
    )),
    "wire": (WireOptions, (
        _Key("enums_file", "path"),                                 # empty → None
        _Key("encoding", "str", check=_encoding),
        _Key("init_data_fallback", "bool"),
    )),
    "image": (ImageOptions, (
        _Key("codec", "str", check=_choice("provisional", "nivision")),
        _Key("nivision_library", "rawstr"),
    )),
    "logging": (LoggingOptions, (
        _Key("append_errors", "bool"),
        _Key("fix_info_labels", "bool"),
        _Key("decimal_separator", "str", check=_one_char),
        _Key("create_log_dirs", "bool"),
        _Key("encoding", "str", check=_encoding),
        _Key("diag_level", "str", check=_choice(*_LOG_LEVELS, upper=True)),
        _Key("diag_file_stem", "str", check=_not_empty),
    )),
    "camera": (CameraOptions, (
        _Key("hm_max_framerate", "float", check=_at_least(0)),
        _Key("bulb_status_inverter", "bool"),
    )),
    "stage": (StageOptions, (
        _Key("always_reference", "bool"),
        _Key("reference_move", "str", check=_choice("FNL", "FPL", upper=True)),
        _Key("ready_timeout_s", "float", check=_at_least(0, strict=True)),
        _Key("wait_timeout_s", "float", check=_at_least(0, strict=True)),
        _Key("do_reference_waits_ready", "bool"),
        _Key("startup_macro_check", "bool"),
        _Key("port_map", "port_map"),
    )),
    "simulation": (SimulationOptions, (
        _Key("enabled", "bool"),
        _Key("camera_names", "list"),
        _Key("image_width", "int", check=_at_least(1)),
        _Key("image_height", "int", check=_at_least(1)),
        _Key("pixel_type", "str", check=_choice("U8", "U16", upper=True)),
        _Key("frame_rate", "float", check=_at_least(0, strict=True)),
        _Key("max_frame_rate", "float", check=_at_least(0, strict=True)),
        _Key("temperature_c", "float"),
        _Key("stage_min_mm", "float"),
        _Key("stage_max_mm", "float"),
        _Key("stage_initially_referenced", "bool"),
        _Key("stage_reference_s", "float", check=_at_least(0)),
        _Key("stage_identity", "str"),
        _Key("stage_dh_macro_s", "float", check=_at_least(0)),
    )),
}
_WIRE_TABLES = ("wire.items", "wire.size", "wire.layout")
_KNOWN_SECTIONS = frozenset(_SCHEMA) | frozenset(_WIRE_TABLES)


class _Reader:
    """Reads one file with per-key fallback and warnings."""

    def __init__(self, cfg: NIConfig, path: Path, warn: Callable[[str], None]) -> None:
        self.cfg, self.path, self.warn = cfg, path, warn

    def bad(self, section: str, key: str, reason: str, default: Any) -> None:
        self.warn(f"{self.path}: [{section}] {key} = {self.cfg.raw(section, key)!r} {reason}; "
                  f"default {default!r} used")

    def read(self, section: str, k: _Key, default: Any) -> Any:
        """The value of ``k`` (``default`` when the key is absent or invalid)."""
        cfg = self.cfg
        if cfg.raw(section, k.key) is None:
            return default
        try:
            value = self._convert(section, k, default)
            if k.check is not None and value is not None:
                value = k.check(value)
        except ConfigTypeInvalid:
            self.bad(section, k.key, f"is not a valid {k.kind}", default)
            return default
        except ValueError as exc:
            self.bad(section, k.key, str(exc), default)
            return default
        return value

    def _convert(self, section: str, k: _Key, default: Any) -> Any:
        cfg, key = self.cfg, k.key
        if k.kind == "bool":
            return cfg.read_bool(section, key, default)
        if k.kind == "int":
            return cfg.read_int(section, key, default)
        if k.kind == "float":
            value = cfg.read_float(section, key, default)
            if not math.isfinite(value):
                raise ValueError("must be a finite number")
            return value
        if k.kind == "str":
            return cfg.read_str(section, key, default)
        if k.kind == "rawstr":
            return cfg.read_path(section, key, default)     # quotes removed, backslashes kept (library paths)
        if k.kind == "list":
            return cfg.read_list(section, key, default)
        if k.kind == "port":                                # [server] port: empty = alox.ini port_I32
            if cfg.read_path(section, key, "").strip() == "":
                return None
            return _port(cfg.read_int(section, key, 0))
        if k.kind == "path":                                # empty = none; localized like every INI path
            text = cfg.read_path(section, key, "").strip()
            if not text:
                return None
            local = localize_path_text(text)
            if local is None:
                raise ValueError("cannot be mapped to a local path (set ALOX_DRIVE_<LETTER>)")
            return local
        if k.kind == "port_map":
            return self._port_map(section, key)
        raise AssertionError(k.kind)

    def _port_map(self, section: str, key: str) -> dict[str, str]:
        """``COM2=/dev/ttyUSB0, COM3=/dev/ttyS2`` (backslashes kept); a bad item is warned and skipped."""
        result: dict[str, str] = {}
        for item in self.cfg.read_path(section, key, "").split(","):
            item = item.strip()
            if not item:
                continue
            name, sep, target = item.partition("=")
            name, target = name.strip().upper(), target.strip()
            if not sep or not name or not target:
                self.warn(f"{self.path}: [{section}] {key}: item {item!r} ignored (expected COM<n>=<device>)")
                continue
            result.setdefault(name, target)
        return result


def _options(reader: _Reader, section: str) -> Any:
    cls, keys = _SCHEMA[section]
    defaults = cls()
    values: dict[str, Any] = {}
    for k in keys:
        if k.key == "max_payload_mb":
            mb = reader.read(section, k, defaults.max_payload_bytes / _MIB)
            values[k.field] = int(round(mb * _MIB))
        else:
            values[k.field] = reader.read(section, k, getattr(defaults, k.field))
    known = {k.key.lower() for k in keys}
    for key in reader.cfg.keys(section):
        if key.lower() not in known:
            reader.warn(f"{reader.path}: [{section}] unknown key {key!r} ignored")
    return dataclasses.replace(defaults, **values)


def _wire_tables(reader: _Reader) -> dict[str, dict[str, Any]]:
    """``[wire.items]`` (lists), ``[wire.size]`` (ints), ``[wire.layout]`` (raw text); keys stripped and lower-cased."""
    cfg, path, warn = reader.cfg, reader.path, reader.warn
    items: dict[str, tuple[str, ...]] = {}
    sizes: dict[str, int] = {}
    layouts: dict[str, str] = {}
    for key in cfg.keys("wire.items"):
        labels = cfg.read_list("wire.items", key)
        if labels:
            items.setdefault(key.strip().lower(), labels)
        else:
            warn(f"{path}: [wire.items] {key} is empty; ignored")
    for key in cfg.keys("wire.size"):
        try:
            sizes.setdefault(key.strip().lower(), cfg.read_int("wire.size", key, 0))
        except ConfigTypeInvalid:
            warn(f"{path}: [wire.size] {key} = {cfg.raw('wire.size', key)!r} is not an integer; ignored")
    for key in cfg.keys("wire.layout"):
        text = cfg.read_str("wire.layout", key, "").strip()
        if text:
            layouts.setdefault(key.strip().lower(), text)
        else:
            warn(f"{path}: [wire.layout] {key} is empty; ignored")
    return {"enum_items": items, "enum_sizes": sizes, "layouts": layouts}


def load_server_config(path: Path, *, warn: Callable[[str], None]) -> ServerConfig:
    """Load ``alox_camera_server.ini`` (design §3.3.4, Appendix A); never raises for file content.

    Missing file → ``ServerConfig()`` (INFO). Invalid value → its default + warning. Unknown section or key → warning.
    ``[server] max_payload_mb`` → ``max_payload_bytes`` (× 2**20); empty ``[server] port`` → None; ``[wire]
    enums_file`` empty → None, else a localized path (relative paths are resolved later against ``<settings>``).
    """
    path = Path(path)
    try:
        cfg = NIConfig.load(path)
    except FileNotFoundError:
        log.info("%s not found; built-in defaults used for every Python-only setting", path)
        return ServerConfig()
    except OSError as exc:
        warn(f"{path}: cannot be read ({exc}); built-in defaults used")
        return ServerConfig()

    reader = _Reader(cfg, path, warn)
    for section in cfg.sections():
        if section.lower() not in _KNOWN_SECTIONS:
            warn(f"{path}: unknown section [{section}] ignored")
    wire = dataclasses.replace(_options(reader, "wire"), **_wire_tables(reader))
    return ServerConfig(server=_options(reader, "server"), compat=_options(reader, "compat"), wire=wire,
                        logging=_options(reader, "logging"), image=_options(reader, "image"),
                        camera=_options(reader, "camera"), stage=_options(reader, "stage"),
                        simulation=_options(reader, "simulation"))


def write_template(path: Path) -> None:
    """Write :data:`SERVER_INI_TEMPLATE` (CRLF, ASCII) to ``path``; :class:`FileExistsError` if it exists."""
    data = SERVER_INI_TEMPLATE.replace("\n", "\r\n").encode("ascii")
    with open(path, "xb") as f:
        f.write(data)
