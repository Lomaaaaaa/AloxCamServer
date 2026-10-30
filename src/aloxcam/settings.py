"""Every typed settings object of the server, defined once (``alox.ini`` and ``alox_camera_server.ini``).

Spec: 09 §4–§5 (folders, ``alox.ini`` keys and defaults), 05 §3.1 (camera properties), 08 §2.1 (Z stage properties),
10 §6 (``[compat]`` registry), docs/phase1-design.md §3.1.3 and Appendix A.

Defaults are the ``alox.ini`` defaults (09 §5.2/§5.6), not the typedef defaults. All classes are frozen: run-time
changes (``Cam Flip``, ``Cam Set FlatField``, ``Z Set Min/Max Limit``) replace whole objects through ``ServerState``.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

__all__ = [
    "CAMERA_TYPES",
    "HIGH_MAG",
    "LOW_MAG",
    "PLUG_GIGE",
    "PLUG_USB",
    "AloxSettings",
    "CameraOptions",
    "CameraSettings",
    "CompatSwitches",
    "Folders",
    "ImageOptions",
    "LightSettings",
    "LoggingOptions",
    "ServerConfig",
    "ServerOptions",
    "SimulationOptions",
    "StageOptions",
    "WireOptions",
    "ZStageSettings",
]

HIGH_MAG = "High Mag."
LOW_MAG = "Low Mag."
CAMERA_TYPES = (HIGH_MAG, LOW_MAG)
PLUG_GIGE = "GiGe"
PLUG_USB = "USB"


@dataclass(frozen=True)
class CameraSettings:
    """``alox_typedef_ext_camera_properties.ctl`` (04 §8.1, 05 §3.1) with the ``alox.ini`` defaults (09 §5.2)."""

    camera_name: str = "HighMag"         # custom-data "name" only
    buffers: int = 5                     # not used
    timeout: int = 5000                  # not used
    flat_field: bool = False
    flat_field_path: Path | None = None  # localized (lv.path.localize_path_text); None = empty path / unmappable
    dark_current_path: Path | None = None
    flat_field_factor: float = 1.0
    exposure_attribute: str = "CameraAttributes::Exposure::Value"
    gain_attribute: str = "CameraAttributes::Gain::Value"
    exposure_type: str = "DBL"           # "DBL" | "I64" | "U32" (case-sensitive; anything else = DBL)
    gain_type: str = "I64"
    horizontal_flip: bool = False
    vertical_flip: bool = False
    framerate_attribute: str = "CameraAttributes::AcquisitionControl::AcquisitionFrameRate"
    framerate_type: str = "DBL"
    max_framerate: float = 0.0           # HM: CameraOptions.hm_max_framerate (unknown LV const); LM: frameratemax_DBL
    wait_led_ms: int = 0
    invert_led: bool = False
    plug_type: str = PLUG_GIGE
    exposure_lower_limit: float = 177.0

    @classmethod
    def default_high_mag(cls) -> CameraSettings:
        """09 §5.6 High Mag defaults: name ``HighMag``, ``invert_led`` FALSE, ``max_framerate`` 0.0."""
        return cls(camera_name="HighMag", invert_led=False, max_framerate=0.0)

    @classmethod
    def default_low_mag(cls) -> CameraSettings:
        """09 §5.6 Low Mag defaults: name ``LowMag``, ``invert_led`` TRUE, ``max_framerate`` 24.6."""
        return cls(camera_name="LowMag", invert_led=True, max_framerate=24.6)


@dataclass(frozen=True)
class LightSettings:
    """LED / bulb wiring (05 §9.1, 01 §3.4.2 frame 5, 09 §5.3)."""

    set_led_cam: int = 0
    camera_sort_led: str = PLUG_GIGE     # plug type of the camera that drives the lights (derived)
    connector: str = "HM"                # "HM" | "LM" (display)
    hm_led_line: int = 0
    hm_bulb_line: int = 0
    led_status_inverter: bool = True     # [hardware.camera] lightStatusInverter_BOOL
    bulb_status_inverter: bool = False   # CameraOptions.bulb_status_inverter (saved LV default unknown)


@dataclass(frozen=True)
class ZStageSettings:
    """``alox_typedef_ext_zstage_properties.ctl`` (08 §2.1); ``baudrate`` holds the real rate (04 T7)."""

    com: int = 2
    baudrate: int = 115200               # real rate (decoded from the INI index)
    axis: int = 1                        # PI "System no."
    speed: float = 10.0
    slow_speed: float = 1.0
    min_pos: float = 0.0
    max_pos: float = 20.0

    @property
    def port_name(self) -> str:
        """``COM<com>``; ``com <= 0`` gives ``COM1`` as in ``Z Setup`` (09 §5.2)."""
        return f"COM{self.com if self.com > 0 else 1}"


@dataclass(frozen=True)
class AloxSettings:
    """Everything ``alox.ini`` provides (09 §5.2)."""

    port: int = 503
    core: int = 1
    max_lines: int = 1000
    high_mag: CameraSettings = field(default_factory=CameraSettings.default_high_mag)
    low_mag: CameraSettings = field(default_factory=CameraSettings.default_low_mag)
    light: LightSettings = field(default_factory=LightSettings)
    z: ZStageSettings = field(default_factory=ZStageSettings)


@dataclass(frozen=True)
class Folders:
    """Resolved folders (09 §3), resolved once at start-up (09 D9)."""

    exe: Path
    settings: Path
    vision: Path | None                  # None = unresolvable on this OS → error event log disabled (warning)
    data: Path | None


@dataclass(frozen=True)
class CompatSwitches:
    """The complete ``[compat]`` registry of 10 §6; defaults = LabVIEW-observable behaviour."""

    send_error_replies: bool = False
    reply_unknown_commands: bool = False
    listen_address: str = "127.0.0.1"
    inbound_pause_ms: int = 0
    close_on_extra_heartbeat_data: bool = True
    exposure_sets_max_framerate: bool = True
    strict_new_frame: bool = False
    request_buffer_zero: bool = True
    temperature_error_fails_image: bool = True
    flatfield_check_on_set: bool = False
    focus_reject_concurrent_start: bool = False   # parsed in Phase 1, used in Phase 4
    focus_validate_params: bool = False           # idem
    af_check_z_limits: bool = False               # idem
    restart_dh_macro: bool = True
    implicit_z_reinit: bool = True


@dataclass(frozen=True)
class ServerOptions:
    """``[server]`` (02 §5.5, D6, D12; 01 §10)."""

    port: int | None = None              # None = alox.ini port_I32
    io_timeout_s: float = 5.0            # socket timeout for payload reads and whole-frame writes (02 §5.5, D12)
    max_payload_bytes: int = 256 * 2**20  # frames with N-2 above this close the connection (02 D6)
    stream_poll_ms: float = 1.0          # pause before re-processing a re-enqueued request when nothing else is queued
    tick_source: str = "monotonic"
    shutdown_timeout_s: float = 10.0
    commands_history: int = 500
    error_history: int = 1000
    temperature_history: int = 10000


@dataclass(frozen=True)
class WireOptions:
    """``[wire]``, ``[wire.items]``, ``[wire.size]``, ``[wire.layout]`` (04 §6, §7; design §6 rows 1–3)."""

    enums_file: Path | None = None
    enum_items: Mapping[str, tuple[str, ...]] = field(default_factory=dict)   # enum_id → ordered labels
    enum_sizes: Mapping[str, int] = field(default_factory=dict)               # enum_id → 1 | 2 | 4
    layouts: Mapping[str, str] = field(default_factory=dict)                  # typedef label → raw layout text
    encoding: str = "cp1252"
    init_data_fallback: bool = False     # Camera Init Data: also try the old Unbundle order (order confirmed)


@dataclass(frozen=True)
class LoggingOptions:
    """``[logging]`` (09 §7, D10, D11, D13)."""

    append_errors: bool = True           # 09 D10 fix; FALSE reproduces the offset-0 overwrite
    fix_info_labels: bool = True         # 09 D11 fix
    decimal_separator: str = "."         # LabVIEW %3u separator (system locale)
    create_log_dirs: bool = True         # 09 D13 fix
    encoding: str = "cp1252"
    diag_level: str = "INFO"
    diag_file_stem: str = "camera_server_diag"


@dataclass(frozen=True)
class ImageOptions:
    """``[image]`` (02 §8.5)."""

    codec: str = "provisional"           # "provisional" | "nivision"
    nivision_library: str = ""           # ctypes library name/path for the Phase-3 codec


@dataclass(frozen=True)
class CameraOptions:
    """``[camera]``: Python-only values the printouts do not give (05 §3.1, §9.1)."""

    hm_max_framerate: float = 0.0
    bulb_status_inverter: bool = False


@dataclass(frozen=True)
class StageOptions:
    """``[stage]`` (08 §2.4, §4.3, §8, Z13)."""

    always_reference: bool = False       # hidden "Check FRF?" option (08 §4.3): FALSE = reference only if FRF? is FALSE
    reference_move: str = "FNL"          # "FNL" | "FPL"
    ready_timeout_s: float = 300.0
    wait_timeout_s: float = 300.0        # General wait for movement to stop (08 §8, Z19)
    do_reference_waits_ready: bool = True  # 08 Z13 fix: #7 then #5 after FNL/FRF
    startup_macro_check: bool = False    # 01 §3.4.3 / 08 §2.4 (A17 default: skip)
    port_map: Mapping[str, str] = field(default_factory=dict)   # "COM2" → "/dev/ttyUSB0" (Phase 2)


@dataclass(frozen=True)
class SimulationOptions:
    """``[simulation]``: ``SimCameraBackend`` / ``SimStage`` parameters."""

    enabled: bool = False                # also set by --simulate
    camera_names: tuple[str, ...] = ()   # empty = any name opens
    image_width: int = 640
    image_height: int = 480
    pixel_type: str = "U8"               # "U8" | "U16"
    frame_rate: float = 10.0
    max_frame_rate: float = 100.0
    temperature_c: float = 35.0
    stage_min_mm: float = 0.0
    stage_max_mm: float = 25.0
    stage_initially_referenced: bool = False
    stage_reference_s: float = 0.2
    stage_identity: str = "(c) aloxcam SimStage, C-663-SIM, 0000000001, 0.0.0"
    stage_dh_macro_s: float = 0.0


@dataclass(frozen=True)
class ServerConfig:
    """The whole ``<settings>/alox_camera_server.ini`` (design Appendix A)."""

    server: ServerOptions = field(default_factory=ServerOptions)
    compat: CompatSwitches = field(default_factory=CompatSwitches)
    wire: WireOptions = field(default_factory=WireOptions)
    logging: LoggingOptions = field(default_factory=LoggingOptions)
    image: ImageOptions = field(default_factory=ImageOptions)
    camera: CameraOptions = field(default_factory=CameraOptions)
    stage: StageOptions = field(default_factory=StageOptions)
    simulation: SimulationOptions = field(default_factory=SimulationOptions)
