# Phase 1 design: protocol skeleton (implementation contract)

**Status:** contract for Phase 1 of [10-migration-plan.md](10-migration-plan.md) §5 ("Protocol skeleton"). Several developers
implement it **in parallel without talking to each other**: everything that crosses a file boundary is fixed here. Behaviour is
specified by the spec chapters (`docs/01`…`docs/10`, cited as "01 §10.3", "05 #12", "08 Z13", "Q17"); this document only fixes
the Python structure, the public APIs, the decisions the spec leaves open, the work split and the tests. Where this document and a
spec chapter disagree on *behaviour*, the spec chapter wins unless this document lists the point as a decision (§6, §7).

**Fixed decisions** (not revisited here): Python ≥ 3.11, src layout, setuptools, runtime deps = stdlib + numpy; threads +
`queue.Queue` (01 §10, 02 §11.2); workers write replies through `ClientLink.send()`; wire enums in a provisional package data file;
the `[compat]` registry of 10 §6 in `<settings>/alox_camera_server.ini` (`alox.ini` is read-only); pluggable `ImageCodec`;
`CameraBackend` / `StageBackend` interfaces with `SimCamera` / `SimStage`; every command of 03 implemented at API level except the
focus jobs and metrics (Phase 4), which reply "not implemented in this phase".

## 0. Conventions for implementers

| Rule | Detail |
|---|---|
| Language level | `from __future__ import annotations` in every module; full type hints; `@dataclass` (frozen where stated); `enum`; no third-party imports except `numpy`. |
| Module docstring | First line = responsibility; then `Spec:` with the sections implemented, e.g. `Spec: 02 §5, §7.4, §8.` Public functions/classes cite their section in the docstring. |
| Logging | `log = logging.getLogger(__name__)` (all loggers under `aloxcam.*`). **No `print`** except in `aloxcam/app.py` (only for `--check-config` / `--version`) and `tools/`. |
| LabVIEW event logs vs diagnostics | LabVIEW-format event logs go **only** through `EventSink` (§3.3.5). Everything else (warnings, connections, tracebacks) goes to the stdlib `logging` "diagnostic" log (09 §7.7). |
| Errors | Inside the server: exceptions from `aloxcam.errors`. At the TCP/log boundary: `ErrorCluster`. Never invent codes outside §3.1.1. |
| Enum values in Python | Wire enums are carried as their **item label** (`str`, e.g. `"High Mag."`, `"FNL"`) everywhere above the codec. Codes exist only inside `aloxcam.lv`/`aloxcam.wire`. |
| Ticks | Every LabVIEW `Tick Count (ms)` is a `TickFn` (`Callable[[], int]`, U32) injected through contexts; never call `time.monotonic()` for protocol timestamps directly. |
| Thread names | `aloxcam-tcp`, `aloxcam-camera`, `aloxcam-z`, `aloxcam-shutdown`, `aloxcam-simcam-<name>`. All daemon threads. |
| Public API | Names listed in this document are the public API (`__all__`). Anything else is private (`_name`) and may change. Adding public names is allowed; changing listed signatures is not. |
| Tests | `pytest`; run with `/home/user/aloxcamserver/.venv/bin/python -m pytest`. Unique test file basenames. Tests never need hardware, network access other than loopback, or real time > 2 s per test (use `FixedClock`/fake clocks). |

---

## 1. Package layout

`WP` = owning work package (§4). Every file has exactly one owner.

```text
pyproject.toml                         WP1  setuptools project, package data, pytest config
README.md                              WP7  (add a "Phase 1 quick start" section only)
src/aloxcam/
  __init__.py                          WP1  __version__ = "0.1.0", PHASE = 1
  __main__.py                          WP4  `python -m aloxcam` → app.main()                                  01 §3, §7
  app.py                               WP4  CLI, configuration loading order, assembly, signals, exit codes   01 §3, §10.1
  errors.py                            WP1  Code, ErrorCluster, AloxError hierarchy, LabVIEW error helpers     09 §8–§10
  clock.py                             WP1  U32 tick sources and arithmetic                                  01 §1.2, 02 §1.2
  settings.py                          WP1  all typed settings dataclasses (alox.ini + server ini)             09 §4–§5, 10 §6
  state.py                             WP1  ServerState (replaces front-panel shared memory)                  01 §8, §10.1
  dispatch.py                          WP1  Request, Outcome, ReplyKind, CommandSpec, contexts, sinks, finish helpers  03 §1.1, §4
  imagedata.py                         WP1  ImageData, ImageCodec protocol                                   02 §8.5
  testing.py                           WP1  test doubles: FixedClock, RecordingReplies, RecordingEvents, make_request
  data/wire_enums.toml                 WP2  PROVISIONAL wire enum tables (§3.2.4)                             04 §6, Q2
  lv/__init__.py                       WP2
  lv/flatten.py                        WP2  big-endian Reader/Writer, layout-driven cluster (un)flatten       02 §6, 04 §3
  lv/path.py                           WP2  LVPath (PTH0), path text ↔ local Path                             04 §3.2, 09 §6.3
  lv/variant.py                        WP2  variant header parse, synthetic type descriptors                 02 §7.3–§7.4
  wire/__init__.py                     WP2
  wire/enums.py                        WP2  WireEnums loader/merger, routing table, routing self-check       03 §3, 04 §6.2
  wire/messages.py                     WP2  payload dataclasses, PayloadCodec, COMMANDS registry (46 entries) 04 §4–§5, 03 §2
  wire/protocol.py                     WP2  frames, envelope, reply payloads, client-side reply parsing      02 §5, §7, §8
  config/__init__.py                   WP3
  config/niconfig.py                   WP3  NI config.llb-compatible INI reader                              09 §6
  config/folders.py                    WP3  exe folder, alox_setup.ini → Folders                             09 §3–§4
  config/alox_ini.py                   WP3  alox.ini → AloxSettings (53 keys)                                09 §5, 01 §3.4
  config/server_ini.py                 WP3  alox_camera_server.ini → ServerConfig (incl. [compat])           10 §6, 02 §11.3
  eventlog.py                          WP3  EventLogs (LabVIEW event logs) + diagnostic logging setup        09 §7
  server/__init__.py                   WP4
  server/link.py                       WP4  ClientLink: one client, whole-frame sends under a lock           01 §10.2–10.3, 02 D10/D12
  server/reply.py                      WP4  Replier: reply frames per kind, send_error_replies, timestamps    02 §8, 03 §1.1
  server/router.py                     WP4  envelope decode, routing, history, receiving timestamp           02 §7, 03 §3
  server/worker.py                     WP4  consumer thread (camera / Z), re-enqueue path, Exit             01 §5.3–5.4, §10
  server/tcp.py                        WP4  listener + reader thread                                         02 §3–§5, §9
  server/runtime.py                    WP4  ServerRuntime: start, shutdown chain                            01 §7, §10.3
  zstage/__init__.py                   WP5
  zstage/backend.py                    WP5  StageBackend ABC (GCS level)                                     08 §3, §13.2
  zstage/gcs_errors.py                 WP5  minimal GCS error text table (used by SimStage)                  08 §11
  zstage/sim.py                        WP5  SimStage: simulated Mercury C-863 at GCS level
  zstage/controller.py                 WP5  ZController: locked link + session, set-up, ref check, waits     08 §4, §6, §8, §12, §13.3
  zstage/handlers.py                   WP5  ZContext + the 19 Z handlers + Exit                              03 §7, 08 §5, §13.4
  camera/__init__.py                   WP6
  camera/backend.py                    WP6  CameraBackend / CameraDevice ABCs, ValueType, Frame              05 §2, §12.3
  camera/sim.py                        WP6  SimCameraBackend / SimCameraDevice
  camera/attributes.py                 WP6  typed attribute access, LabVIEW numeric coercion, attribute names 05 §2.1, §5.1
  camera/pipeline.py                   WP6  flip, flat-field, custom data, PNG-folder simulation             05 §7, §8
  camera/acquisition.py                WP6  AcqState, per-camera GetImageStateMachine                         05 §6.3–6.4, §12.7
  camera/session.py                    WP6  CameraSession, FocusFields                                       05 §3.2, 06 §2.5
  camera/handlers.py                   WP6  CameraContext + camera/general handlers (+ Phase-4 stubs)        03 §5–§6, 05, 06 §3
  imaging/__init__.py                  WP6
  imaging/png.py                       WP6  minimal PNG reader/writer (zlib + numpy)                         05 §8.2
  imaging/codec.py                     WP6  ProvisionalCodec, NiVisionCodec stub, make_codec, decode_provisional  02 §8.5
  client.py                            WP7  AloxClient: protocol client library (tests + replay tool)        02 §10
tools/
  replay_client.py                     WP7  scripted client: send commands, check/record/compare replies    10 §7
  scripts/phase1_all_commands.toml     WP7  every command once, default [compat]
  scripts/phase1_error_replies.toml    WP7  send_error_replies / reply_unknown_commands = TRUE
  scripts/smoke.toml                   WP7  heartbeat + Exit
tests/
  conftest.py                          WP1  shared fixtures (fixed_clock, tmp_alox_tree)
  golden/spec_vectors.py               WP2  every worked hex example of the spec, as data (§4.3)
  foundation/test_*.py                 WP1
  wire/test_*.py                       WP2
  config/test_*.py, config/data/*.ini  WP3
  server/test_*.py                     WP4
  zstage/test_*.py                     WP5
  camera/test_*.py, imaging/test_*.py  WP6
  tools/test_replay_client.py          WP7
  integration/test_*.py                WP7
```

Reserved for later phases (do **not** create in Phase 1): `zstage/pi_backend.py` (Phase 2, pipython `StageBackend`),
`camera/<sdk>_backend.py` (Phase 3), `imaging/nivision.py` (Phase 3, real ctypes codec), `focus/{metrics,autofocus,refocus,controller}.py`
(Phase 4).

### 1.1 `pyproject.toml` (WP1)

```toml
[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[project]
name = "aloxcam"
version = "0.1.0"
description = "Python re-implementation of the ALOX Camera Server (LabVIEW)"
requires-python = ">=3.11"
dependencies = ["numpy>=1.24"]

[project.optional-dependencies]
dev = ["pytest>=7.4"]            # pipython / camera SDK extras are added in phases 2-3

[project.scripts]
aloxcam = "aloxcam.app:main"

[tool.setuptools.packages.find]
where = ["src"]

[tool.setuptools.package-data]
aloxcam = ["data/*.toml"]

[tool.pytest.ini_options]
testpaths = ["tests"]
pythonpath = ["src", "tools", "tests"]
addopts = "--import-mode=importlib"
markers = ["integration: end-to-end tests with a live server on loopback", "slow: > 2 s"]
```

---

## 2. Runtime architecture (summary)

```text
 client ⇄ socket ── tcp thread (server/tcp.py): accept 1 client, read whole frames
                       │ Command frame → Router.route_frame(): decode envelope, map code → name, history,
                       │                 strict payload decode, route by RoutingTable
                       ├──────────────▶ camera Worker (queue.Queue, unbounded) ─▶ camera handlers ─▶ CameraBackend ×2
                       │                    ▲ Outcome.requeue → put at tail (internal path, never blocks)
                       │                    │ "Exit" → forward internal Exit to Z worker, stop
                       └──────────────▶ Z Worker (queue.Queue, unbounded) ─────▶ Z handlers ─▶ ZController ─▶ StageBackend
 replies: handlers → ReplySink (server/reply.py Replier) → ClientLink.send(frame, conn_id) [one lock, whole frames]
 shared:  ServerState (RLock), EventLogs (lock), ZController (RLock per GCS exchange; Phase 4 focus code shares it)
```

- **Routing** is computed once from the enum table (03 §3.5, 04 T2); the two internal `… Run` commands are never routed (03 C16).
- **Undecodable payloads** of a known command are not executed: the worker replies with the command's reply kind carrying
  `{TRUE, 91, "Variant To Data in alox_camera_server.vi"}` + the command's suffix (camera commands), subject to `send_error_replies`
  (10 §6 carve-out (a), 04 T1). Undecodable envelopes are logged and dropped (02 §7.6).
- **Shutdown** (01 §7, A10 fix): `Exit` → camera worker forwards `Exit` to the Z worker and stops → runtime closes the link (later
  replies dropped, as LabVIEW loses them), stops the listener, waits for the Z worker, closes cameras and stage. Ctrl+C/SIGTERM post an
  internal `Exit` to the camera worker (same chain).

---

## 3. Module APIs

Signatures are normative. "Raises" lists the only exception types callers must handle.

### 3.1 WP1 — foundation (shared types)

#### 3.1.1 `aloxcam/errors.py` (09 §8–§10)

```python
class Code(IntEnum):
    """Every code the server produces or tests (09 §9, §10.5). Values are final."""
    NO_ERROR = 0
    LV_INVALID_INPUT = 1                # LabVIEW "An input parameter is invalid" (non-finite numeric inputs)
    EXTRA_DATA_FOUND = -1               # HW TCP Extract TCP Data (02 §5.3) — diagnostic only
    PI_NOT_CONNECTED = -4               # Python choice (08 §13.3): GCS call with no open link
    PI_OPEN_PORT = -6                   # Python choice (08 §13.3): serial port open failure
    PI_TIMEOUT = -7                     # "Wait for axes to stop.vi: Timeout." / serial read timeout
    GCS_UNREFERENCED = 5                # proposal, implicit_z_reinit = FALSE (09 §10.5)
    TCP_TIMEOUT = 56
    VARIANT_TYPE_MISMATCH = 91          # undecodable Command Data (04 T1)
    PI_DRIVER = 100                     # PI LabVIEW driver checks ("<CMD> No axis specified", "No answer on ERR?")
    UNFLATTEN_FAILED = 116              # malformed/truncated flattened data (ProtocolError default)
    GET_IMAGE_TIMEOUT = 9009
    SIM_INVALID_TYPE = -9011            # negative, as printed (09 D16)
    REFOCUS_JOYSTICK = 9031
    Z_POSITION_TIMEOUT = 9401
    Z_OUT_OF_RANGE = 9405
    INTERNAL = 9500                     # unexpected exception (bug)            — 95xx block: 09 §10.5 proposal
    IMAGE_FILE = 9501                   # image file read/write failed
    CAMERA_SDK = 9502                   # camera driver error without IMAQdx equivalent (incl. SimCamera)
    FOCUS_METRIC = 9503                 # Phase 4
    FOCUS_INVALID_PARAMS = 9504         # Phase 4, focus_validate_params
    FOCUS_JOB_RUNNING = 9505            # Phase 4, focus_reject_concurrent_start
    UNKNOWN_COMMAND = 9506              # reply_unknown_commands = TRUE
    NOT_IMPLEMENTED_IN_PHASE = 9507     # Phase-1 stubs (Cam Autofocus, Cam Refocus)
    IMAGE_CODEC_UNAVAILABLE = 9508      # ImageCodec cannot flatten (NI Vision stub)
    VISION_OPERATION = 9509             # NI Vision operation failed, NI code unknown (flat-field size mismatch…)
    IMAQDX_INVALID_SESSION = -1074360316

TEXT: Mapping[int, str]   # fixed source texts: 9405 "Z move out of range", 9009 "get image error (timeout)",
                          # -9011 "Invalid Type", 9031 "'refocus' warning (joystick enabled)", 9401 "z stage get position timeout"

@dataclass(frozen=True)
class ErrorCluster:                     # 04 §4.3; the only error form on the wire or in event logs
    status: bool = False
    code: int = 0
    source: str = ""
    @property
    def is_warning(self) -> bool: ...   # not status and code != 0
    @property
    def ok(self) -> bool: ...           # not status

NO_ERROR: ErrorCluster                  # ErrorCluster()

class AloxError(Exception):
    def __init__(self, code: int, source: str) -> None: ...   # attributes .code, .source
    def cluster(self) -> ErrorCluster: ...                     # ErrorCluster(True, code, source)
class ConfigError(AloxError): ...       # fatal configuration problems only (tolerated INI problems are warnings)
class ProtocolError(AloxError): ...     # framing / flattening; code normally Code.UNFLATTEN_FAILED
class CameraError(AloxError): ...
class CodecError(CameraError): ...      # ImageCodec failures
class StageError(AloxError): ...
class FocusError(AloxError): ...        # Phase 4

def decorate(err: ErrorCluster, suffix: str) -> ErrorCluster: ...          # append only when status (03 §1, 09 §10.2)
def replace_source(err: ErrorCluster, source: str) -> ErrorCluster: ...    # only when status (Cam Set Exposure, 05 §5.2)
def merge_errors(*errs: ErrorCluster) -> ErrorCluster: ...                # first error, else first warning, else NO_ERROR
def clear_errors(err: ErrorCluster, specific_code: int = 0) -> tuple[ErrorCluster, bool]: ...
def error_cluster_from_error_code(code: int, message: str = "", *, is_warning: bool = False,
                                  error_in: ErrorCluster = NO_ERROR, caller: str = "") -> ErrorCluster: ...
def cluster_of(exc: BaseException) -> ErrorCluster: ...   # AloxError → its cluster; anything else → (True, 9500, f"{type}: {exc}")
```

Thread-safety: pure values/functions.

#### 3.1.2 `aloxcam/clock.py`

```python
U32_MASK = 0xFFFFFFFF
TickFn = Callable[[], int]                       # returns a U32 millisecond tick (LabVIEW Tick Count)
def monotonic_tick() -> int: ...                 # int(time.monotonic() * 1000) & U32_MASK (01 §10.1)
def windows_tick() -> int: ...                   # GetTickCount via ctypes; falls back to monotonic_tick off Windows
def make_tick_source(name: str) -> TickFn: ...   # "monotonic" | "windows"; ConfigError otherwise (Q32 e)
def tick_add(t: int, ms: int) -> int: ...        # U32 wrap-around addition
def tick_diff(later: int, earlier: int) -> int: ...  # (later - earlier) & U32_MASK
```

#### 3.1.3 `aloxcam/settings.py` — every settings type, defined once

Defaults are the **alox.ini defaults** (09 §5.2/§5.6), not the typedef defaults. All dataclasses are `frozen=True`; run-time changes
(`Cam Flip`, `Cam Set FlatField`, `Z Set Min/Max Limit`) replace whole objects through `ServerState`.

```python
HIGH_MAG = "High Mag."; LOW_MAG = "Low Mag."; CAMERA_TYPES = (HIGH_MAG, LOW_MAG)
PLUG_GIGE = "GiGe"; PLUG_USB = "USB"

@dataclass(frozen=True)
class CameraSettings:                    # alox_typedef_ext_camera_properties.ctl (04 §8.1, 05 §3.1)
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
    max_framerate: float = 0.0           # HM: CameraOptions.hm_max_framerate (unknown LV constant); LM: frameratemax_DBL
    wait_led_ms: int = 0
    invert_led: bool = False
    plug_type: str = PLUG_GIGE
    exposure_lower_limit: float = 177.0
    @classmethod
    def default_high_mag(cls) -> "CameraSettings": ...   # name "HighMag", invert_led False, max_framerate 0.0
    @classmethod
    def default_low_mag(cls) -> "CameraSettings": ...    # name "LowMag", invert_led True, max_framerate 24.6

@dataclass(frozen=True)
class LightSettings:                     # 05 §9.1, 01 §3.4.2 frame 5
    set_led_cam: int = 0
    camera_sort_led: str = PLUG_GIGE     # plug type of the camera that drives the lights (derived)
    connector: str = "HM"                # "HM" | "LM" (display)
    hm_led_line: int = 0
    hm_bulb_line: int = 0
    led_status_inverter: bool = True     # [hardware.camera] lightStatusInverter_BOOL
    bulb_status_inverter: bool = False   # CameraOptions.bulb_status_inverter (saved LV default unknown)

@dataclass(frozen=True)
class ZStageSettings:                    # alox_typedef_ext_zstage_properties.ctl (08 §2.1)
    com: int = 2
    baudrate: int = 115200               # real rate (decoded from the INI index)
    axis: int = 1                        # PI "System no."
    speed: float = 10.0
    slow_speed: float = 1.0
    min_pos: float = 0.0
    max_pos: float = 20.0
    @property
    def port_name(self) -> str: ...      # f"COM{com if com > 0 else 1}"

@dataclass(frozen=True)
class AloxSettings:                      # everything alox.ini provides (09 §5.2)
    port: int = 503
    core: int = 1
    max_lines: int = 1000
    high_mag: CameraSettings = field(default_factory=CameraSettings.default_high_mag)
    low_mag: CameraSettings = field(default_factory=CameraSettings.default_low_mag)
    light: LightSettings = field(default_factory=LightSettings)
    z: ZStageSettings = field(default_factory=ZStageSettings)

@dataclass(frozen=True)
class Folders:                           # 09 §3
    exe: Path
    settings: Path
    vision: Path | None                  # None = unresolvable on this OS → error event log disabled (warning)
    data: Path | None

@dataclass(frozen=True)
class CompatSwitches:                    # 10 §6 — the complete registry; defaults = LabVIEW-observable behaviour
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
class ServerOptions:                     # [server]
    port: int | None = None              # None = alox.ini port_I32
    io_timeout_s: float = 5.0            # socket timeout for payload reads and whole-frame writes (02 §5.5, D12)
    max_payload_bytes: int = 256 * 2**20 # frames with N-2 above this close the connection (02 D6)
    stream_poll_ms: float = 1.0          # pause before re-processing a re-enqueued request when nothing else is queued
    tick_source: str = "monotonic"
    shutdown_timeout_s: float = 10.0
    commands_history: int = 500
    error_history: int = 1000
    temperature_history: int = 10000

@dataclass(frozen=True)
class WireOptions:                       # [wire], [wire.items], [wire.size], [wire.layout]
    enums_file: Path | None = None
    enum_items: Mapping[str, tuple[str, ...]] = field(default_factory=dict)   # enum_id → ordered labels
    enum_sizes: Mapping[str, int] = field(default_factory=dict)               # enum_id → 1 | 2 | 4
    layouts: Mapping[str, str] = field(default_factory=dict)                  # typedef label → raw layout text (§3.2.1)
    encoding: str = "cp1252"
    init_data_fallback: bool = True      # Camera Init Data: also try the other field order (04 §7.1)

@dataclass(frozen=True)
class LoggingOptions:                    # [logging]
    append_errors: bool = True           # 09 D10 fix; FALSE reproduces the offset-0 overwrite
    fix_info_labels: bool = True         # 09 D11 fix
    decimal_separator: str = "."         # LabVIEW %3u separator (system locale)
    create_log_dirs: bool = True         # 09 D13 fix
    encoding: str = "cp1252"
    diag_level: str = "INFO"
    diag_file_stem: str = "camera_server_diag"

@dataclass(frozen=True)
class ImageOptions:                      # [image]
    codec: str = "provisional"           # "provisional" | "nivision"
    nivision_library: str = ""           # ctypes library name/path for the Phase-3 codec

@dataclass(frozen=True)
class CameraOptions:                     # [camera] (Python-only values the printouts do not give)
    hm_max_framerate: float = 0.0
    bulb_status_inverter: bool = False

@dataclass(frozen=True)
class StageOptions:                      # [stage]
    always_reference: bool = False       # hidden "Check FRF?" option (08 §4.3): FALSE = reference only if FRF? is FALSE
    reference_move: str = "FNL"          # "FNL" | "FPL"
    ready_timeout_s: float = 300.0
    wait_timeout_s: float = 300.0        # General wait for movement to stop (08 §8, Z19)
    do_reference_waits_ready: bool = True  # 08 Z13 fix: #7 then #5 after FNL/FRF
    startup_macro_check: bool = False    # 01 §3.4.3 / 08 §2.4 (A17 default: skip)
    port_map: Mapping[str, str] = field(default_factory=dict)   # "COM2" → "/dev/ttyUSB0" (Phase 2)

@dataclass(frozen=True)
class SimulationOptions:                 # [simulation]
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
    stage_identity: str = "(c) aloxcam SimStage, C-863.11-SIM, 0000000001, 0.0.0"
    stage_dh_macro_s: float = 0.0

@dataclass(frozen=True)
class ServerConfig:                      # the whole alox_camera_server.ini
    server: ServerOptions = field(default_factory=ServerOptions)
    compat: CompatSwitches = field(default_factory=CompatSwitches)
    wire: WireOptions = field(default_factory=WireOptions)
    logging: LoggingOptions = field(default_factory=LoggingOptions)
    image: ImageOptions = field(default_factory=ImageOptions)
    camera: CameraOptions = field(default_factory=CameraOptions)
    stage: StageOptions = field(default_factory=StageOptions)
    simulation: SimulationOptions = field(default_factory=SimulationOptions)
```

#### 3.1.4 `aloxcam/state.py` (01 §8, §10.1)

```python
@dataclass
class ServerState:
    """Front-panel objects used as shared memory, with one RLock (01 §8, A16). Readers get values or copies."""
    high_mag: CameraSettings
    low_mag: CameraSettings
    light: LightSettings
    z: ZStageSettings
    options: ServerOptions = field(default_factory=ServerOptions)    # history sizes
    # status (01 §8.3) — written through update()/helpers only
    connected: bool = False
    server_state: str = "Listen"
    heartbeat: int = 0                   # TCP-loop liveness tick (not the Get Heartbeat reply)
    last_command: str = ""
    error_out: ErrorCluster = NO_ERROR
    inited: dict[str, bool] = field(default_factory=lambda: dict.fromkeys(CAMERA_TYPES, False))
    acq_state: dict[str, str] = field(default_factory=lambda: dict.fromkeys(CAMERA_TYPES, "Start"))
    temperature: dict[str, float] = field(default_factory=lambda: dict.fromkeys(CAMERA_TYPES, 0.0))
    bus_type: dict[str, str] = field(default_factory=lambda: dict.fromkeys(CAMERA_TYPES, ""))
    serial_high: int = 0; serial_low: int = 0
    max_framerate_display: float = 0.0; max_fr_real: int = 0
    led_cmd: bool = False; led_status: bool = False; light_trace: str = ""
    show_ui: bool = False
    cam_log_data: object | None = None   # wire.messages.CamLogData (stored, never read: 03 C5)
    z_inited: bool = False; z_pos: float = 0.0
    running_macros: str = ""; macro_running: bool = False
    # histories: created in __post_init__ with maxlen from `options` (field(init=False))
    commands_history: deque[str]         # newest first, maxlen options.commands_history
    error_history: deque[tuple[float, ErrorCluster]]               # maxlen options.error_history
    temperature_history: deque[tuple[float, float, float]]         # (time.time(), HM, LM), maxlen options.temperature_history
    lock: threading.RLock = field(default_factory=threading.RLock, repr=False, compare=False)

    @classmethod
    def from_settings(cls, alox: AloxSettings, options: ServerOptions = ServerOptions()) -> "ServerState": ...
    def camera_settings(self, camera_type: str) -> CameraSettings: ...            # KeyError for unknown types
    def replace_camera_settings(self, camera_type: str, **changes: Any) -> CameraSettings: ...
    def replace_z_settings(self, **changes: Any) -> ZStageSettings: ...
    def record_command(self, name: str) -> None: ...      # appendleft; also sets last_command (03 §3.3)
    def set_error_out(self, err: ErrorCluster) -> None: ...
    def record_camera_iteration(self) -> None: ...        # bounded (time, error_out) + (time, HM, LM) samples (01 §5.3, A11/A12)
    def update(self, **fields: Any) -> None: ...          # atomic multi-field write; AttributeError on unknown names
    def update_camera(self, camera_type: str, *, inited: bool | None = None, acq_state: str | None = None,
                      temperature: float | None = None, bus_type: str | None = None) -> None: ...
    def snapshot(self) -> dict[str, Any]: ...             # copies of all fields (diagnostics, tests)
```

Thread-safety: every method takes `lock`; settings objects are immutable, so a reader holding a returned object never sees a torn value.

#### 3.1.5 `aloxcam/dispatch.py` — the handler contract (03 §1.1, §4)

```python
CAMERA_WORKER = "camera"; Z_WORKER = "z"
SERVER_CAMERA = "Camera"; SERVER_Z = "Z Stage"          # Log Server Error "Server Type" (09 §7.1)

class ReplyKind(Enum):                                   # 03 §1.1, 02 §8.3
    NONE = "none"; ACK = "ACK"; STR = "STR"; BOOL = "BOOL"; DBL = "DBL"; U32 = "U32"; DBL2D = "DBL2D"; IMAGE = "IMAGE"

class DataMode(Enum):                                    # 04 §5.1
    NOT_READ = "not_read"        # LabVIEW never reads Command Data: any bytes accepted, data = None
    UNUSED = "unused"            # LabVIEW decodes {Extra Param} but never uses it: any bytes accepted, data = None
    STRICT = "strict"            # decoded with the payload layout; failure → Request.decode_error

class FinishOrder(Enum):                                 # 03 §4.2
    A = "A"                      # decorate → log → reply (variant A, A')
    RAW_FIRST = "raw_first"      # reply raw error → decorate reply-out → log (variants B, C, D)
    Z = "z"                      # reply → log, no decoration (03 §4.3)
    CUSTOM = "custom"            # handler does it itself (variant E)

@dataclass(frozen=True)
class CommandSpec:
    name: str                    # exact enum item label
    worker: str                  # CAMERA_WORKER | Z_WORKER (loop that has the case)
    payload: type | None         # wire.messages dataclass; None = no payload type
    data_mode: DataMode
    reply: ReplyKind
    order: FinishOrder
    extra: str                   # Log Server Error "Extra String" (03 §5–§7)
    suffix: str | None           # camera source suffix; None = computed by the handler
    client_routable: bool = True # False: Cam Autofocus Run / Cam Refocus Run (03 C16)
    spec_ref: str = ""           # e.g. "05 §4.2"

@dataclass
class Request:                   # internal form of Camera Queue Data (04 §4.1) after decoding
    command: str                 # item label; "#<code>" when the code has no item
    code: int                    # wire code; -1 for internal requests
    spec: CommandSpec | None
    notifier: bytes = b""        # opaque, echoed byte for byte (02 §10 rule 1)
    data: Any = None             # decoded payload dataclass, or None
    raw_data: bytes = b""        # variant data bytes as received
    decode_error: ErrorCluster | None = None
    request_ts: int = 0
    sending_ts: int = 0
    receiving_ts: int = 0        # set by the router on the camera route (02 §7.1)
    conn_id: int = 0             # ClientLink generation; 0 = internal (never replied)
    internal: bool = False
    requeue_count: int = 0
    @classmethod
    def make_internal(cls, command: str, spec: CommandSpec | None) -> "Request": ...

@dataclass(frozen=True)
class Outcome:
    stop: bool = False           # Exit
    requeue: bool = False        # put the same Request back at the tail of the worker queue
CONTINUE = Outcome(); STOP = Outcome(stop=True); REQUEUE = Outcome(requeue=True)

class ReplySink(Protocol):       # implemented by server.reply.Replier and testing.RecordingReplies; thread-safe
    def ack(self, req: Request, err: ErrorCluster) -> ErrorCluster: ...            # returns err (ACK VI error out)
    def string(self, req: Request, value: str, err: ErrorCluster) -> ErrorCluster: ...   # typed: return NO_ERROR
    def boolean(self, req: Request, value: bool, err: ErrorCluster) -> ErrorCluster: ...
    def dbl(self, req: Request, value: float, err: ErrorCluster) -> ErrorCluster: ...
    def u32(self, req: Request, value: int, err: ErrorCluster) -> ErrorCluster: ...
    def dbl_2d(self, req: Request, value: np.ndarray, err: ErrorCluster) -> ErrorCluster: ...
    def image(self, req: Request, image: ImageData | None, err: ErrorCluster) -> ErrorCluster: ...
    def error_reply(self, req: Request, kind: ReplyKind, err: ErrorCluster) -> ErrorCluster: ...  # kind default value + err

class EventSink(Protocol):       # implemented by eventlog.EventLogs and testing.RecordingEvents; thread-safe
    def error(self, err: ErrorCluster, server_type: str, extra: str, *, command: str | None = "Get Heartbeat",
              camera_type: str = HIGH_MAG) -> ErrorCluster: ...   # Log Server Error: writes only if err.status
    def info(self, err: ErrorCluster, server_type: str, extra: str, *,
             camera_type: str = HIGH_MAG) -> ErrorCluster: ...    # Log Server Infos: always writes

Handler = Callable[[Any, Request], Outcome | None]       # (context, request); None = CONTINUE
HandlerTable = Mapping[str, Handler]                      # command label → handler

class BackgroundStepper(Protocol):                        # Phase 4 focus stepping (06 §9.5); None in Phase 1
    def has_work(self) -> bool: ...
    def step(self) -> None: ...
    def abort(self) -> None: ...

@dataclass
class WorkerContext:             # base context; camera/Z contexts subclass it (camera/handlers.py, zstage/handlers.py)
    state: ServerState
    replies: ReplySink
    events: EventSink
    compat: CompatSwitches
    tick: TickFn

# Finish helpers (03 §4.5). Defaults come from req.spec. All set ctx.state.error_out to the logged error.
def send_reply(ctx: WorkerContext, req: Request, value: Any, err: ErrorCluster) -> ErrorCluster: ...
    # dispatch on req.spec.reply; NONE → no reply, returns err
def finish_camera(ctx: WorkerContext, req: Request, value: Any, err: ErrorCluster, *, camera_type: str | None,
                  suffix: str | None = None, extra: str | None = None,
                  order: FinishOrder | None = None) -> ErrorCluster: ...
    # A: e = decorate(err, suffix); events.error(e, "Camera", extra, command=req.command, camera_type=…); send_reply(e)
    # RAW_FIRST: out = send_reply(err); e = decorate(out, suffix); events.error(e, …)
    # camera_type None → HIGH_MAG (LabVIEW unwired input); pass the real type whenever known (09 D15 fix)
def finish_z(ctx: WorkerContext, req: Request, value: Any, err: ErrorCluster, *,
             extra: str | None = None) -> ErrorCluster: ...
    # send_reply(err); events.error(err, "Z Stage", extra, command=None)
def reject_undecodable(ctx: WorkerContext, req: Request) -> None: ...
    # used by Worker for req.decode_error (value = kind default): camera → finish_camera with req.spec.order (CUSTOM → A),
    # suffix = spec.suffix or "" and the camera type HIGH_MAG (unknown); Z → finish_z. Reply kind NONE → log only.
def reply_default_value(kind: ReplyKind) -> Any: ...      # "", False, 0.0, 0, np.empty((0, 0)), None (image)
```

#### 3.1.6 `aloxcam/imagedata.py` (02 §8.5, 05 §7.5)

```python
NI_IMAGE_U8 = 0; NI_IMAGE_I16 = 1; NI_IMAGE_SGL = 2; NI_IMAGE_RGB = 4; NI_IMAGE_U16 = 7

@dataclass(frozen=True)
class ImageData:
    pixels: np.ndarray                    # 2-D (rows, cols) uint8 | uint16 | int16 | float32, or (rows, cols, 4) uint8 RGBA
    custom_data: Mapping[str, str] = field(default_factory=dict)   # IMAQ custom-data keys, insertion order kept
    @property
    def width(self) -> int: ...
    @property
    def height(self) -> int: ...
    @property
    def ni_image_type(self) -> int: ...   # NI_IMAGE_*; ValueError for other dtypes

class ImageCodec(Protocol):
    name: str
    def flatten(self, image: ImageData) -> bytes: ...   # the "IMAQ Flatten Image to String" replacement; raises CodecError
```

#### 3.1.7 `aloxcam/testing.py` (test doubles shared by all packages)

```python
class FixedClock:                         # TickFn
    def __init__(self, start: int = 0) -> None: ...
    def __call__(self) -> int: ...
    def set(self, value: int) -> None: ...
    def advance(self, ms: int) -> None: ...

@dataclass
class SentReply:
    kind: ReplyKind; req: Request; value: Any; err: ErrorCluster; sent: bool   # sent False = suppressed by send_error_replies

class RecordingReplies:                   # ReplySink with the exact return/suppression semantics of server.reply.Replier
    def __init__(self, send_error_replies: bool = False) -> None: ...
    replies: list[SentReply]

@dataclass
class LoggedEvent:
    level: str; err: ErrorCluster; server_type: str; extra: str; command: str | None; camera_type: str

class RecordingEvents:                    # EventSink; error() records only status TRUE (as Log Server Error)
    events: list[LoggedEvent]

def make_request(command: str, data: Any = None, *, notifier: bytes = b"N1", conn_id: int = 1,
                 ts: tuple[int, int, int] = (0, 0, 0), decode_error: ErrorCluster | None = None) -> Request: ...
    # spec looked up in wire.messages.COMMANDS (lazy import); code from the packaged enum table
def make_state(**overrides: Any) -> ServerState: ...      # from AloxSettings() defaults
def make_worker_context(*, compat: CompatSwitches | None = None, clock: FixedClock | None = None,
                        send_error_replies: bool | None = None) -> WorkerContext: ...
```

---

### 3.2 WP2 — LabVIEW codec and wire layer

#### 3.2.1 `aloxcam/lv/flatten.py` (02 §6, 04 §3)

```python
DEFAULT_ENCODING = "cp1252"
Kind = str          # see the kinds table below
Layout = tuple[tuple[str, Kind], ...]     # ((attribute, kind), …) in CLUSTER order

class EnumCodec(Protocol):                # implemented by wire.enums.WireEnums
    def size(self, enum_id: str) -> int: ...
    def label(self, enum_id: str, code: int) -> str: ...   # ProtocolError if out of range (non-lenient enums)
    def code(self, enum_id: str, label: str) -> int: ...   # ValueError for unknown labels; "#<n>" → n

class Reader:
    def __init__(self, data: bytes, pos: int = 0, *, encoding: str = DEFAULT_ENCODING) -> None: ...
    pos: int
    @property
    def remaining(self) -> int: ...
    def take(self, n: int) -> bytes: ...  # ProtocolError(116, "need n bytes at offset p, have k")
    def bool(self) -> bool: ...           # any non-zero byte is TRUE
    def u8(self) -> int: ...; def i8(self) -> int: ...; def u16(self) -> int: ...; def i16(self) -> int: ...
    def u32(self) -> int: ...; def i32(self) -> int: ...; def u64(self) -> int: ...; def i64(self) -> int: ...
    def sgl(self) -> float: ...; def dbl(self) -> float: ...
    def uint(self, size: int) -> int: ... # 1 | 2 | 4
    def string_bytes(self) -> bytes: ...  # I32 length (negative → ProtocolError) + bytes
    def string(self) -> str: ...          # decoded with errors="replace"
    def error(self) -> ErrorCluster: ...
    def i32_array(self) -> list[int]: ...
    def dbl_2d(self) -> np.ndarray: ...   # shape (rows, cols) float64, preserves 0×0 vs 0×2 (02 §8.3)
    def path(self) -> LVPath: ...

class Writer:
    def __init__(self, *, encoding: str = DEFAULT_ENCODING) -> None: ...
    # same kinds as Reader; integer writers raise ValueError when out of range; string() accepts str (encoded,
    # errors="strict" → ValueError) or bytes (written as is); error(ErrorCluster); dbl_2d(array-like 2-D, shape kept);
    # path(LVPath); uint(value, size); raw(bytes). Every method returns self.
    def bytes(self) -> bytes: ...

def unflatten(cls: type[T], r: Reader, layout: Layout, enums: EnumCodec) -> T: ...
def flatten(obj: Any, layout: Layout, enums: EnumCodec, w: Writer | None = None) -> bytes: ...
def parse_layout(text: str) -> Layout: ...          # "attr:kind, attr:kind" (kind may contain ':' for enums)
def validate_layout(cls: type, layout: Layout) -> None: ...   # ConfigError unless a permutation of the dataclass fields
```

| Kind | Bytes (big-endian) | Python value |
|---|---|---|
| `bool` | 1 | `bool` |
| `u8` `i8` `u16` `i16` `u32` `i32` `u64` `i64` | 1/1/2/2/4/4/8/8 | `int` |
| `sgl` / `dbl` | 4 / 8 IEEE-754 | `float` |
| `str` | I32 n + n bytes | `str` |
| `path` | `PTH0` form (04 §3.2) | `LVPath` |
| `i32[]` | I32 n + n × I32 | `list[int]` |
| `dbl[][]` | I32 rows, I32 cols, row-major DBL | `np.ndarray` |
| `err` | U8 status, I32 code, I32 n + source | `ErrorCluster` |
| `enum:<enum_id>` | `enums.size(enum_id)` | `str` label |

#### 3.2.2 `aloxcam/lv/path.py` (04 §3.2, 09 §6.3)

```python
PATH_ABSOLUTE = 0; PATH_RELATIVE = 1; PATH_NOT_A_PATH = 2; PATH_UNC = 3

@dataclass(frozen=True)
class LVPath:                                   # lossless wire form
    kind: int = PATH_ABSOLUTE
    components: tuple[str, ...] = ()
    @property
    def is_empty(self) -> bool: ...             # absolute with no component (LabVIEW <empty path>)
    def to_windows(self) -> str | None: ...     # 04 §9.2; None for Not-A-Path
    @classmethod
    def from_windows(cls, text: str | None) -> "LVPath": ...
    def to_local(self, *, platform: str = sys.platform, env: Mapping[str, str] = os.environ) -> Path | None: ...
    @classmethod
    def from_local(cls, path: Path | str) -> "LVPath": ...      # POSIX "/a/b" → absolute ("a", "b")
    @classmethod
    def parse_user(cls, text: str) -> "LVPath": ...             # Windows form if it has a drive/backslash/UNC, else from_local

def read_path(r: Reader) -> LVPath: ...
def write_path(w: Writer, p: LVPath) -> Writer: ...
def common_to_windows(text: str) -> str: ...    # NI Common Path to Specific Path, Windows branch (09 §6.3)
def localize_path_text(text: str, *, platform: str = sys.platform,
                       env: Mapping[str, str] = os.environ) -> Path | None: ...
```

Localization rules (Linux development and tests; Windows behaves like LabVIEW): `""`, Not-A-Path → `None`. Windows: text
→ `common_to_windows` → `Path`. POSIX: a drive-letter path (`D:\x`, `/D/x` with a one-letter first component) maps through the
environment variable `ALOX_DRIVE_<LETTER>` (e.g. `ALOX_DRIVE_D=/srv/alox`); `D:\…` without a mapping → `None`; `/D/x` without a
mapping → native `/D/x`; UNC → `None`; anything else is a native path. `to_local` applies the same rules to absolute components.

#### 3.2.3 `aloxcam/lv/variant.py` (02 §7.3–§7.4)

```python
LV2018_VERSION = 0x18008000
class TypeCode(IntEnum):
    VOID = 0x00; I8 = 0x01; I16 = 0x02; I32 = 0x03; I64 = 0x04; U8 = 0x05; U16 = 0x06; U32 = 0x07; U64 = 0x08
    SGL = 0x09; DBL = 0x0A; ENUM_U8 = 0x15; ENUM_U16 = 0x16; ENUM_U32 = 0x17; BOOL = 0x21; STRING = 0x30
    PATH = 0x32; ARRAY = 0x40; CLUSTER = 0x50; VARIANT = 0x53; TYPEDEF = 0xF1

@dataclass(frozen=True)
class VariantHeader:
    version: int
    type_descriptors: tuple[bytes, ...]   # each TD including its size word
    top_indices: tuple[int, ...]
    raw: bytes                            # version … top-level list, exactly as received
    @property
    def top_type_code(self) -> int | None: ...

def read_variant_header(r: Reader) -> VariantHeader: ...
    # 02 §11.1 algorithm: TD size < 4 or bit 15 set → ProtocolError; top index ≥ TD count → ProtocolError
def synth_variant_header(layout: Layout | None, enums: EnumCodec, *, bare: bool = False,
                         version: int = LV2018_VERSION) -> bytes: ...
def write_variant(header: bytes, data: bytes) -> bytes: ...   # header + data + U32 0 (attribute count)
EMPTY_VARIANT_HEADER: bytes               # synth_variant_header(None, …): version, 1 TD (00 04 00 00), top list [0]
```

Synthetic type descriptors are only for **our** test client and simulator (a captured template is preferred, 02 §7.4). They follow
the inferred layout of 02 §7.3 and only need to be well formed for `read_variant_header`; they are **not** claimed to equal
LabVIEW's bytes, except where the spec shows them. Rules: element TDs first, container last, top index = container; no labels.
Scalar `[00 04 00 <code>]`; string/path `[00 08 00 30|32 FF FF FF FF]`; enum `[size 00 15|16|17] U16 n_items` + Pascal strings,
padded to even; 1-D/2-D array `[size 00 40] U16 ndims, I32 −1 per dim, U16 element index`; cluster `[size 00 50] U16 n, U16 index × n`.
`bare=True` (Show UI, Z Do Reference, Z Set Min/Max Limit) emits the single field TD as the top type. Golden check: the bare-DBL
header equals `18 00 80 00 00 00 00 01 00 04 00 0a 00 01 00 00` (02 §7.5 (b)).

#### 3.2.4 `aloxcam/data/wire_enums.toml` and `aloxcam/wire/enums.py` (03 §3, 04 §6)

File format (schema 1). Every table: `label`, `ctl`, `size` (1|2|4), `provisional` (bool), `lenient` (bool, default false: an
unknown code decodes as `"#<code>"` instead of failing), `items` (labels in **value** order: code = index).

```toml
# aloxcam/data/wire_enums.toml — PROVISIONAL. Real orders/representations are UNKNOWN (Q2, 04 §6, 10 R1).
# Override in <settings>/alox_camera_server.ini: [wire] enums_file, [wire.items], [wire.size].
schema = 1

[enums.tcp_datatype]
label = "TCP DataType"
ctl = "HW TCP Datatype.ctl"
size = 2                      # must be 2 (02 §5.2)
provisional = true
items = ["Command", "Response", "Image", "Heartbeat"]

[enums.camera_command]
label = "Function"
ctl = "typedef_ext_camera_camera_command.ctl"
size = 2
provisional = true
items = [
  # camera range "Get Heartbeat".."Cam Get Led": 24 items, order of 04 §6.2
  "Get Heartbeat", "Show UI", "Exit", "Cam Setup", "Cam Start", "Cam Get Image", "Cam Stream Next Image",
  "Cam Get Acquisition Status", "Cam Software Trigger", "Cam Set Exposure", "Cam Set FlatField", "Cam Set Gain",
  "Cam Get Exposure", "Cam Get Gain", "Cam Get Images Array", "Cam Stop", "Cam Cleanup", "Cam Flip", "Cam Autofocus",
  "Cam Refocus", "Cam Get Autofocus Data", "Cam Get Autofocus Progress", "Cam Set Led", "Cam Get Led",
  # explicit camera route
  "Log",
  # Z range "Z Setup".."Z Do Reference": 19 items, Z Do Reference last (03 §3.2 point 3)
  "Z Setup", "Z Get Serial", "Z Get Status", "Z Get Error", "Z Set Speed", "Z Get Speed", "Z Get Reference",
  "Z Get Position", "Z Move Absolute", "Z Move Relative", "Z Wait End Move", "Z Stop", "Z Enable Joystick",
  "Z Disable Joystick", "Z Joystick Enabled ?", "Z Set Min Limit", "Z Set Max Limit", "Z Cleanup", "Z Do Reference",
  # internal, outside both ranges (never routed, 03 C16)
  "Cam Autofocus Run", "Cam Refocus Run",
]

[enums.camera_type]
label = "Camera Type"
ctl = "alox_typedef_camera_type.ctl"
size = 2
provisional = true            # codes inferred strong (04 §3.3); size unknown
items = ["High Mag.", "Low Mag."]

[enums.autofocus_speed]
label = "Autofocus Speed"
ctl = "alox_typedef_autofocus_speed.ctl"
size = 2
provisional = true
items = ["Slow", "Fast"]

[enums.autofocus_method]
label = "Autofocus Method"
ctl = "alox_typedef_autofocus_method.ctl"
size = 2
provisional = true
items = ["FFT", "Sobel", "Prewitt", "Diff", "Gradient", "Sigma", "FULL"]

[enums.refocus_method]
label = "Refocus Method"
ctl = "alox_typedef_refocus_method.ctl"
size = 2
provisional = true
items = ["Fin", "Oxyd"]

[enums.z_reference_method]
label = "Z Reference Method"
ctl = "(bare enum in the main VI)"
size = 2
provisional = true
lenient = true                # any value other than FRF runs FNL (08 §5.3)
items = ["FNL", "FRF"]
```

```python
ENUM_IDS = ("tcp_datatype", "camera_command", "camera_type", "autofocus_speed", "autofocus_method",
            "refocus_method", "z_reference_method")
CAMERA_RANGE = ("Get Heartbeat", "Cam Get Led"); CAMERA_EXTRA = ("Log",); Z_RANGE = ("Z Setup", "Z Do Reference")

@dataclass(frozen=True)
class EnumTable:
    enum_id: str; label: str; ctl: str; size: int; items: tuple[str, ...]; provisional: bool; lenient: bool; source: str
    def code(self, label: str) -> int: ...
    def label_of(self, code: int) -> str: ...

class WireEnums:                          # implements lv.flatten.EnumCodec; immutable after load
    tables: Mapping[str, EnumTable]
    @property
    def provisional_ids(self) -> tuple[str, ...]: ...
    def table(self, enum_id: str) -> EnumTable: ...
    def size(self, enum_id: str) -> int: ...
    def label(self, enum_id: str, code: int) -> str: ...
    def code(self, enum_id: str, label: str) -> int: ...
    @classmethod
    def load(cls, *, override_file: Path | None = None, items: Mapping[str, Sequence[str]] = {},
             sizes: Mapping[str, int] = {}) -> "WireEnums": ...
    @classmethod
    def from_options(cls, wire: WireOptions, settings_dir: Path) -> "WireEnums": ...   # relative enums_file → settings_dir
    def to_toml(self) -> str: ...

@dataclass(frozen=True)
class RoutingTable:                       # evaluated once (03 §3.5); behaves like the LabVIEW selector for every code
    names: Mapping[int, str]              # code → item label (all items)
    routes: Mapping[int, str]             # code → CAMERA_WORKER | Z_WORKER; absent = router "Default"
    def route(self, code: int) -> str | None: ...
    def name(self, code: int) -> str: ... # "#<code>" for codes beyond the item list

def build_routing(enums: WireEnums, *, non_routable: Collection[str] = ("Cam Autofocus Run", "Cam Refocus Run")) -> RoutingTable: ...
def routing_self_check(routing: RoutingTable, cases: Mapping[str, str]) -> list[str]: ...
    # cases: label → worker that has a case; returns "<name>: case unreachable (routes to …)" problems (03 C1)
```

The packaged file is read with `importlib.resources.files("aloxcam") / "data" / "wire_enums.toml"` and `tomllib`.
Merge precedence: packaged file < `override_file` (each table present replaces the packaged table; its `provisional` defaults to
`false`) < `items`/`sizes` (replace the items/size of one table; provisional becomes `false`). Validation (`ConfigError`): unknown
`enum_id`; size ∉ {1, 2, 4}; `tcp_datatype.size ≠ 2`; duplicate labels; `tcp_datatype` lacks one of its 4 items; `camera_type` lacks
`High Mag.`/`Low Mag.`; `camera_command` lacks one of `Get Heartbeat`, `Cam Get Led`, `Log`, `Z Setup`, `Z Do Reference`; an
endpoint pair out of order; overlapping ranges. Known command names missing from the table are not an error: they are reported by
`routing_self_check` (logged at ERROR at start-up).

#### 3.2.5 `aloxcam/wire/messages.py` (04 §4–§5, 03 §2)

Payload dataclasses carry `LV_TYPEDEF: ClassVar[str]` (typedef label = `[wire.layout]` key), `LV_LAYOUT: ClassVar[Layout]`
(working cluster order), `LV_BARE: ClassVar[bool] = False`. Enum fields hold labels with the typedef default label as default.
`CT` below = `camera_type:enum:camera_type` (default `"High Mag."`).

| Class | `LV_TYPEDEF` | `LV_LAYOUT` (cluster order; defaults ≠ zero in brackets) |
|---|---|---|
| `CameraGetImageData`, `CameraStatusData`, `CameraTriggerSoftData`, `CameraGetExposureData`, `CameraGetGainData`, `CameraStopData`, `CameraReleaseData`, `CameraGetAutofocusData`, `CameraGetLedData` (subclasses of `CameraOnlyData`) | `Camera Get Image Data`, `Camera Status Data`, `Camera Trigger Soft Data`, `Camera Get Exposure Data`, `Camera Get Gain Data`, `Camera Stop Data`, `Camera Release Data`, `Camera Get Autofocus Data`, `Camera Get LED Data` | `CT` |
| `CameraInitData` | `Camera Init Data` | `CT, continuous_mode:bool, number_of_buffers:i32, camera_name:str` (`LAYOUT_UNBUNDLE`, working assumption 04 §7.1); `LAYOUT_PANEL = CT, camera_name:str, number_of_buffers:i32, continuous_mode:bool` |
| `CameraStartData` | `Camera Start Data` | `CT, triggered:bool` |
| `CameraSetExposureData` | `Camera Set Exposure Data` | `CT, exposure:dbl` |
| `CameraGainData` | `Camera Gain Data` | `CT, gain:dbl` |
| `CameraSetFlatFieldData` | `Camera Ext Set FlatField` | `CT, enable:bool, ffc_path:path, dfc_path:path, correction_factor:dbl` |
| `CameraFlipData` | `Camera Flip Data` | `CT, horizontal_flip:bool, vertical_flip:bool` |
| `CameraSetLedData` | `Camera Set LED Data` | `CT, led:bool` |
| `CameraStreamData` | `Camera Stream Data` | `CT, simulation:bool, simulation_path:path, simulation_timer_ms:u32, acquisition_timeout_ms:i32 [5000]` |
| `CamLogData` | `Cam Log Data` | `log:bool, path:path, max_lines:i32, max_count:i32` |
| `ShowUIData` (bare) | `Show UI` | `show:bool` |
| `CameraAutofocusData` | `Camera Autofocus Data` | `CT, autofocus_speed:enum:autofocus_speed ["Slow"], autofocus_method:enum:autofocus_method ["FFT"], duration_s:i32, z_min:dbl, z_max:dbl, step:dbl, subsampling:dbl, roi:i32[], acquisition_timeout_ms:i32 [5000]` (panel order, 04 §5.5) |
| `CameraRefocusData` | `Camera Refocus Data` | `CT, autofocus_method:enum:autofocus_method ["FFT"], refocus_method:enum:refocus_method ["Fin"], steps:i32, step_mm:dbl, fin_roi_width:i32, fin_roi_height:i32, oxyd_x_pos:i32, oxyd_y_pos:i32, oxyd_diam:i32, acquisition_timeout_ms:i32 [5000], wait_ms:i32, save_images:bool, folder:path` |
| `ZGetSerialData` | `Z Stage Get Serial Data` | `type:bool` |
| `ZSetSpeedData` | `Z Stage Set Speed Data` | `speed_mm_s:dbl` |
| `ZExtraParamData` | `Z Stage Get Speed Data` | `extra_param:str` |
| `ZMoveAbsData` | `Z Move Abs Data` | `pos_mm:dbl` |
| `ZMoveRelData` | `Z Stage Move Rel Data` | `move_mm:dbl` |
| `ZWaitData` | `Z Stage Wait Data` | `add_wait_ms:i32, polling_cycle_ms:i32 [10], add_wait_only:bool` |
| `ZJoystickData` | `Z Stage Joystick Data` | `joystick_id:i32 [1], joystick_axis:i32 [1]` |
| `ZReferenceData` (bare) | `Z Reference Method` | `method:enum:z_reference_method ["FNL"]` |
| `ZLimitData` (bare) | `Z Limit` | `value_mm:dbl` |

```python
class PayloadCodec:                       # thread-safe after construction (immutable)
    def __init__(self, enums: WireEnums, layout_overrides: Mapping[str, str] = {}, *, init_data_fallback: bool = True,
                 encoding: str = "cp1252", templates: Mapping[str, bytes] = {}) -> None: ...  # ConfigError on bad layouts
    def layout_of(self, cls: type) -> Layout: ...
    def decode(self, spec: CommandSpec, data: bytes) -> Any: ...
        # STRICT: unflatten with layout_of(payload); must consume exactly len(data) (else ProtocolError);
        #   Camera Init Data: try layout_of first, then the other built-in order (if init_data_fallback), log WARNING when
        #   the other one matched (04 §7.1). NOT_READ / UNUSED: return None without looking at the bytes.
    def encode(self, obj: Any) -> bytes: ...                 # data part only (client/simulator)
    def variant_for(self, spec: CommandSpec, obj: Any | None) -> bytes: ...
        # header (templates[spec.name] if present, else synthetic) + data + U32 0; NOT_READ → EMPTY_VARIANT_HEADER + U32 0;
        # UNUSED → ZExtraParamData() when obj is None

def payload_from_mapping(cls: type, mapping: Mapping[str, Any]) -> Any: ...
    # replay scripts: path fields accept str (LVPath.parse_user) or {kind, components}; enums accept labels or ints ("#n")
def payload_to_mapping(obj: Any) -> dict[str, Any]: ...

COMMANDS: Mapping[str, CommandSpec]       # the 46 entries below, insertion order = the table order
CASES: Mapping[str, str]                  # label → worker, for every command that has a handler (excludes the 2 "… Run")
```

**The registry** (normative; "dyn" = suffix computed by the handler; extra/suffix texts from 03 §5–§7, 05 §10.2, 06 §2.1):

| Command | Worker | Payload | Mode | Reply | Order | Extra | Suffix | Spec |
|---|---|---|---|---|---|---|---|---|
| `Get Heartbeat` | camera | — | NOT_READ | U32 | RAW_FIRST | `Heartbeat` | `Get Heartbeat` | 03 §5.1 |
| `Show UI` | camera | `ShowUIData` | STRICT | ACK | A | `Show UI` | `Cam Show UI` | 03 §5.2 |
| `Log` | camera | `CamLogData` | STRICT | ACK | A | `Log` | `Cam Log` | 03 §5.3 |
| `Exit` | camera | — | NOT_READ | NONE | CUSTOM | — | — | 03 §5.4 |
| `Cam Setup` | camera | `CameraInitData` | STRICT | ACK | A | `Setup` | dyn | 05 §4.2 |
| `Cam Start` | camera | `CameraStartData` | STRICT | ACK | A | `Start` | dyn | 05 §4.4 |
| `Cam Stop` | camera | `CameraStopData` | STRICT | ACK | A | `Stop` | `Cam stop` | 05 §4.5 |
| `Cam Cleanup` | camera | `CameraReleaseData` | STRICT | ACK | A | `Cleanup` | `Cam Cleanup` | 05 §4.6 |
| `Cam Get Acquisition Status` | camera | `CameraStatusData` | STRICT | BOOL | A | `Get Status` | `Cam Acquisition status` | 05 §4.7 |
| `Cam Software Trigger` | camera | `CameraTriggerSoftData` | STRICT | ACK | A | `Soft Trigger` | `` | 05 §4.8 |
| `Cam Set Exposure` | camera | `CameraSetExposureData` | STRICT | ACK | A | `Set Exposure` | `` (source replaced) | 05 §5.2 |
| `Cam Get Exposure` | camera | `CameraGetExposureData` | STRICT | DBL | A | `Get Exposure` | dyn | 05 §5.3 |
| `Cam Set Gain` | camera | `CameraGainData` | STRICT | ACK | A | `Set Gain` | dyn | 05 §5.4 |
| `Cam Get Gain` | camera | `CameraGetGainData` | STRICT | DBL | A | `Get Gain` | dyn | 05 §5.4 |
| `Cam Stream Next Image` | camera | `CameraStreamData` | STRICT | IMAGE | CUSTOM | (per outcome) | dyn | 05 §6.2 |
| `Cam Get Image` | camera | `CameraGetImageData` | STRICT | NONE | CUSTOM | — | `Cam Get Image` | 05 §6.5 |
| `Cam Get Images Array` | camera | — | NOT_READ | NONE | CUSTOM | — | — | 03 §5.17 |
| `Cam Flip` | camera | `CameraFlipData` | STRICT | ACK | A | `Flip` | `Cam Flip` | 05 §7.2 |
| `Cam Set FlatField` | camera | `CameraSetFlatFieldData` | STRICT | ACK | A | `Set FlatField` | `Cam Set Field` | 05 §7.3 |
| `Cam Set Led` | camera | `CameraSetLedData` | STRICT | ACK | RAW_FIRST | `Set LED` | `Cam Set Led` | 05 §9.2 |
| `Cam Get Led` | camera | `CameraGetLedData` | STRICT | BOOL | RAW_FIRST | `Get LED` | `Cam Get Led` | 05 §9.3 |
| `Cam Autofocus` | camera | `CameraAutofocusData` | STRICT | ACK | A | `Autofocus` | `Cam Autofocus` | 06 §3.1 |
| `Cam Refocus` | camera | `CameraRefocusData` | STRICT | ACK | A | `Refocus` | `Cam Refocus` | 06 §3.1 |
| `Cam Autofocus Run` | camera | `CameraAutofocusData` | STRICT | NONE | CUSTOM | `Autofocus Run` | `Autofocus` | 06 §3.2 (not routable) |
| `Cam Refocus Run` | camera | `CameraRefocusData` | STRICT | NONE | CUSTOM | `Refocus Run` | `Cam Refocus Run` | 06 §3.2 (not routable) |
| `Cam Get Autofocus Data` | camera | `CameraGetAutofocusData` | STRICT | DBL2D | RAW_FIRST | `Autofocus Data` | `Cam Get Autofocus Data` | 06 §3.3 |
| `Cam Get Autofocus Progress` | camera | `CameraGetAutofocusData` | STRICT | DBL | RAW_FIRST | `Autofocus Progress` | `Cam Get Autofocus Progress` | 06 §3.3 |
| `Z Setup` | z | — | NOT_READ | ACK | Z | `Setup` | — | 08 §4.1 |
| `Z Get Serial` | z | `ZGetSerialData` | STRICT | STR | Z | `Get Serial` | — | 08 §5.3 |
| `Z Get Status` | z | — | NOT_READ | BOOL | Z | `Get Status` | — | 08 §5.3 |
| `Z Get Error` | z | — | NOT_READ | STR | Z | `Get Error` | — | 08 §5.3 |
| `Z Set Speed` | z | `ZSetSpeedData` | STRICT | ACK | Z | `Set Speed` | — | 08 §5.3 |
| `Z Get Speed` | z | `ZExtraParamData` | UNUSED | DBL | Z | `Get Speed` | — | 08 §5.3 |
| `Z Get Reference` | z | `ZExtraParamData` | UNUSED | BOOL | Z | `Get Z Referenced` | — | 08 §5.3 |
| `Z Do Reference` | z | `ZReferenceData` | STRICT | ACK | Z | `Get Z Referenced` | — | 08 §5.3 |
| `Z Get Position` | z | `ZExtraParamData` | UNUSED | DBL | Z | `Get Position` | — | 08 §5.3 |
| `Z Move Absolute` | z | `ZMoveAbsData` | STRICT | ACK | Z | `Move Absolute` | — | 08 §5.3, §6 |
| `Z Move Relative` | z | `ZMoveRelData` | STRICT | ACK | Z | `Move Relative` | — | 08 §5.3, §6 |
| `Z Wait End Move` | z | `ZWaitData` | STRICT | ACK | Z | `Wait End Move` | — | 08 §8 |
| `Z Stop` | z | `ZExtraParamData` | UNUSED | ACK | Z | `Stop` | — | 08 §5.3 |
| `Z Enable Joystick` | z | `ZJoystickData` | STRICT | ACK | Z | `Enable Joystick` | — | 08 §9 |
| `Z Disable Joystick` | z | `ZJoystickData` | STRICT | ACK | Z | `Disable Joystick` | — | 08 §9 |
| `Z Joystick Enabled ?` | z | `ZJoystickData` | STRICT | BOOL | Z | `Is Joystick Enabled ?` | — | 08 §9 |
| `Z Set Min Limit` | z | `ZLimitData` | STRICT | ACK | Z | `Z Set Min Limit` | — | 08 §5.3 |
| `Z Set Max Limit` | z | `ZLimitData` | STRICT | ACK | Z | `Z Set Max Limit` | — | 08 §5.3 |
| `Z Cleanup` | z | — | NOT_READ | ACK | Z | `Cleanup` | — | 08 §5.3 |

#### 3.2.6 `aloxcam/wire/protocol.py` (02 §5, §7, §8)

```python
FRAME_HEADER_SIZE = 6

@dataclass(frozen=True)
class Datatypes:                          # HW TCP Datatype codes from the enum table (02 §5.2)
    command: int; response: int; image: int; heartbeat: int
    @classmethod
    def from_enums(cls, enums: WireEnums) -> "Datatypes": ...
    def name_of(self, code: int) -> str | None: ...

@dataclass(frozen=True)
class FrameHeader:
    payload_len: int
    datatype: int

def encode_frame(datatype: int, payload: bytes = b"") -> bytes: ...          # >iH + payload (02 §5.1)
def parse_header(header6: bytes, max_payload: int) -> FrameHeader: ...      # ProtocolError if N < 2 or N-2 > max_payload
def heartbeat_frame(datatypes: Datatypes, payload: bytes = b"") -> bytes: ...

@dataclass(frozen=True)
class CommandEnvelope:                    # Camera Queue Data as flattened (02 §7.1)
    code: int
    notifier: bytes
    variant: VariantHeader
    data: bytes                           # variant data part (between the header and the U32 attribute count)
    request_ts: int
    sending_ts: int
    receiving_ts: int

def decode_envelope(payload: bytes, enums: WireEnums) -> CommandEnvelope: ...
    # 02 §7.4: code (camera_command size), notifier, variant header walk, data = up to len-16,
    # attribute count must be 0, then 3 × U32. Every truncation raises ProtocolError.
def encode_envelope(code: int, notifier: bytes | str, variant: bytes, enums: WireEnums,
                    ts: tuple[int, int, int] = (0, 0, 0)) -> bytes: ...

@dataclass(frozen=True)
class ResponseTimestamps:                 # HW TCP Response Data fields 2–7, order of 02 §8.1 (inferred, A9)
    request: int = 0; sending: int = 0; receiving: int = 0
    pre_flatten: int = 0; post_flatten: int = 0; tcp_sending: int = 0
    def as_tuple(self) -> tuple[int, int, int, int, int, int]: ...

def flatten_reply(kind: ReplyKind, value: Any, err: ErrorCluster, *, encoding: str = "cp1252") -> bytes: ...
    # Response (Flattened) per 02 §8.3; ACK ignores value; IMAGE value = flattened image bytes (b"" on error)
def encode_response_data(notifier: bytes, flattened: bytes, ts: ResponseTimestamps) -> bytes: ...
def response_frame(datatypes: Datatypes, notifier: bytes, flattened: bytes, ts: ResponseTimestamps) -> bytes: ...

@dataclass(frozen=True)
class ResponseData:
    notifier: bytes; flattened: bytes; ts: ResponseTimestamps
def decode_response_data(payload: bytes) -> ResponseData: ...
def parse_reply_value(kind: ReplyKind, flattened: bytes, *, encoding: str = "cp1252") -> tuple[Any, ErrorCluster]: ...
    # client side; must consume exactly (ProtocolError otherwise); IMAGE value = bytes
```

---

### 3.3 WP3 — configuration and event logs

#### 3.3.1 `aloxcam/config/niconfig.py` (09 §6)

```python
class ConfigTypeInvalid(ValueError): ...   # NI "Config type invalid"
class NIConfig:                           # read-only; never writes or creates files (09 D2)
    @classmethod
    def load(cls, path: Path, *, encoding: str = "cp1252") -> "NIConfig": ...   # FileNotFoundError if missing
    @classmethod
    def parse(cls, text: str) -> "NIConfig": ...
    def sections(self) -> list[str]: ...
    def keys(self, section: str) -> list[str]: ...
    def raw(self, section: str, key: str) -> str | None: ...
    def read_str(self, section: str, key: str, default: str) -> str: ...
    def read_int(self, section: str, key: str, default: int, *, unsigned: bool = False) -> int: ...
    def read_float(self, section: str, key: str, default: float) -> float: ...
    def read_bool(self, section: str, key: str, default: bool) -> bool: ...
    def read_path(self, section: str, key: str, default: str) -> str: ...      # quotes removed, NOT unescaped, NOT localized
    def read_list(self, section: str, key: str, default: Sequence[str] = ()) -> tuple[str, ...]: ...  # comma-separated, stripped
```

Semantics exactly as 09 §6.1–§6.2 and the 09 §6.6 sketch (case-insensitive, first occurrence wins, `;` comments, quoted values,
`\\`/`\XX` unescape for strings, decimal comma, `12abc` → 12, `0`/`1`/`TRUE`/`FALSE`), except: BOM tolerated; path conversion moved
to `lv.path.localize_path_text`.

#### 3.3.2 `aloxcam/config/folders.py` (09 §3–§4)

```python
FOLDER_DEFAULTS: Mapping[str, str] = {"settings": r"D:\alox\settings", "vision": r"D:\alox\02-vision",
                                       "data": r"D:\alox\02-Vision\data"}
def executable_folder(env: Mapping[str, str] = os.environ) -> Path: ...
    # $ALOX_EXE_DIR if set; frozen build → folder of sys.executable; otherwise the current working directory
def resolve_folders(*, setup_ini: Path | None = None, exe_dir: Path | None = None, settings_override: Path | None = None,
                    warn: Callable[[str], None]) -> Folders: ...
    # resolved ONCE (09 D9); missing alox_setup.ini → defaults + warning, file not created (D2); empty value → default +
    # warning (D3); unmappable vision/data → None + warning; settings_override (--settings-dir) wins over [folders] settings;
    # an unmappable settings folder raises ConfigError (nothing can be loaded)
```

#### 3.3.3 `aloxcam/config/alox_ini.py` (09 §5, 01 §3.4)

```python
@dataclass(frozen=True)
class IniKey:
    section: str; key: str; kind: str                  # kind = NI suffix: I32 U32 DBL BOOL STR PATH
    default: Any; target: str                          # dotted target, e.g. "high_mag.flat_field_factor"
ALOX_INI_KEYS: tuple[IniKey, ...]                     # exactly the 53 Read Key calls of 09 §5.2 (HM frameratemax_DBL absent)
BAUD_BY_INDEX: Mapping[int, int] = {0: 9600, 1: 19200, 2: 38400, 3: 115200}
def load_alox_ini(path: Path, camera_options: CameraOptions, *, warn: Callable[[str], None]) -> AloxSettings: ...
```

Rules: missing file → all defaults + warning, not created (D2); each key read on its own, invalid value → its default + warning (D4
fix, no chains); `baudrate_I32` index (other values → 9600 + warning, D7); `cameraconnectortype_STR` case-insensitive `usb` → `USB`,
anything else `GiGe` (D6 fix); paths localized with `localize_path_text` (unmappable → `None` + warning); HM `max_framerate` =
`camera_options.hm_max_framerate` (D5); `LightSettings` derived per 09 §5.3 (`camera_sort_led` from the selected camera's
`plug_type`), `bulb_status_inverter` from `camera_options`.

#### 3.3.4 `aloxcam/config/server_ini.py` (10 §6, 02 §11.3)

```python
SERVER_INI_NAME = "alox_camera_server.ini"
SERVER_INI_TEMPLATE: str                  # Appendix A, verbatim; parses to ServerConfig() defaults
def load_server_config(path: Path, *, warn: Callable[[str], None]) -> ServerConfig: ...
    # missing file → ServerConfig() (INFO); invalid value → default + warning; unknown section or key → warning;
    # [wire.items] values split on commas (NIConfig.read_list), [wire.size] ints, [wire.layout] kept raw (validated by
    # wire.enums / wire.messages at start-up); [server] max_payload_mb → ServerOptions.max_payload_bytes (× 2**20);
    # empty [server] port → None; [stage] port_map "COM2=/dev/ttyUSB0, COM3=/dev/ttyS2" (commas: ';' starts a comment)
def write_template(path: Path) -> None: ...   # refuses to overwrite (FileExistsError)
```

#### 3.3.5 `aloxcam/eventlog.py` (09 §7)

```python
STEM: Mapping[str, str] = {"Camera": "camera", "Monochromator": "wavelength", "XY Stages": "xy", "Z Stage": "z"}

class EventLogs:                          # implements dispatch.EventSink; thread-safe (one lock around formatting + I/O)
    def __init__(self, folders: Folders, options: LoggingOptions, *, now: Callable[[], datetime] = datetime.now,
                 recent_lines: int = 1000) -> None: ...
    def ensure_dirs(self) -> list[str]: ...           # create <vision>/logs and <data>/logs (D13); returns warnings
    def error(self, err, server_type, extra, *, command="Get Heartbeat", camera_type="High Mag.") -> ErrorCluster: ...
    def info(self, err, server_type, extra, *, camera_type="High Mag.") -> ErrorCluster: ...
    def recent(self) -> list[str]: ...                # ring buffer of the last lines (09 §7.5, D19)
    def error_log_path(self, server_type: str, when: datetime) -> Path | None: ...   # <vision>/logs/<yymmdd>_<stem>_events.log
    def info_log_path(self, server_type: str, when: datetime) -> Path | None: ...    # <data>/logs/…
    @staticmethod
    def format_error_line(when: datetime, err: ErrorCluster, server_type: str, extra: str, command: str | None,
                          camera_type: str, decimal_separator: str = ".") -> str: ...   # 09 §7.4
    @staticmethod
    def format_info_line(when: datetime, err: ErrorCluster, server_type: str, extra: str, camera_type: str,
                         decimal_separator: str = ".", fix_info_labels: bool = True) -> str: ...

def setup_diagnostic_logging(folders: Folders | None, options: LoggingOptions, *, level: str | None = None,
                             stderr: bool = True) -> list[logging.Handler]: ...
    # root logger "aloxcam"; DailyFileHandler at <vision>/logs/<yymmdd>_<diag_file_stem>.log (UTF-8) when vision is known
class DailyFileHandler(logging.Handler): ...  # (directory, stem); switches file when the local date changes
```

Line format, tags, stems and folders exactly 09 §7.2–§7.4 (CRLF, `options.encoding`, no space after the tag). Write failures are
swallowed (as LabVIEW `Clear Errors`) and reported once per file in the diagnostic log. `append_errors=False` reproduces the offset-0
write (for byte comparisons), `fix_info_labels=False` reproduces D11.

---

### 3.4 WP4 — server runtime and entry point

#### 3.4.1 `aloxcam/server/link.py`

```python
class ClientLink:
    """The current client connection (01 §10.3). Thread-safe: send() may be called from any thread."""
    def __init__(self) -> None: ...
    def attach(self, sock: socket.socket) -> int: ...   # new conn_id (1, 2, …); the previous connection must be detached
    def detach(self, conn_id: int) -> None: ...         # no-op if not current
    @property
    def current_id(self) -> int | None: ...
    def send(self, frame: bytes, conn_id: int) -> bool: ...
        # whole frame under one lock; False (frame dropped, DEBUG log) when conn_id != current (02 D10), the link is closed
        # or conn_id == 0; on OSError/timeout: shutdown(SHUT_RDWR) the socket (reader sees EOF) and return False (02 D12)
    def close(self) -> None: ...                        # shutdown: every later send() returns False
```

#### 3.4.2 `aloxcam/server/reply.py` (02 §8, 03 §1.1)

```python
class Replier:                            # implements dispatch.ReplySink
    def __init__(self, link: ClientLink, datatypes: Datatypes, codec: ImageCodec, compat: CompatSwitches,
                 tick: TickFn, *, encoding: str = "cp1252") -> None: ...
```

Semantics: build `Response (Flattened)` with `wire.protocol.flatten_reply`; timestamps all 0 except `tcp_sending = tick()` taken
just before `link.send` (02 §8.2); `ack()` with `err.status` and `compat.send_error_replies == False` sends nothing (DEBUG log
"error ACK suppressed") and returns `err` (02 §8.4); warnings are sent. Typed replies are always sent and return `NO_ERROR`.
`image()`: if `err.status` the codec is not called and the value is `b""`; otherwise `pre_flatten = tick()`, `codec.flatten(image)`,
`post_flatten = tick()`; a `CodecError` becomes the reply's error with `b""`; `request/sending/receiving` copied from the request.
`error_reply(req, kind, err)` sends the kind's default value with `err` (ACK path honours `send_error_replies`). Requests with
`conn_id == 0` are never sent.

#### 3.4.3 `aloxcam/server/router.py` (02 §7, 03 §3)

```python
class Router:                             # called only from the TCP thread
    def __init__(self, *, enums: WireEnums, routing: RoutingTable, commands: Mapping[str, CommandSpec],
                 payloads: PayloadCodec, state: ServerState, replies: ReplySink, compat: CompatSwitches,
                 tick: TickFn, camera_put: Callable[[Request], None], z_put: Callable[[Request], None]) -> None: ...
    def route_frame(self, payload: bytes, conn_id: int) -> Request | None: ...
```

`route_frame`: `decode_envelope` (ProtocolError → ERROR log with hex excerpt, drop, `None`); `name = routing.name(code)`;
`state.record_command(name)`; `route = routing.route(code)`; no route → WARNING "unknown command", and with
`reply_unknown_commands` an error reply `{TRUE, 9506, "Unknown command <name>"}` of kind `spec.reply` when the name is known and
its kind is not NONE, else ACK; else `spec = commands.get(name)`, `data = payloads.decode(spec, env.data)` (ProtocolError → `decode_error =
{TRUE, 91, "Variant To Data in alox_camera_server.vi"}`, WARNING log); camera route sets `receiving_ts = tick()`; `camera_put` /
`z_put` (never block). Returns the routed `Request`.

#### 3.4.4 `aloxcam/server/worker.py` (01 §5.3–5.4, §10.2–10.3, 06 §9.5)

```python
class Worker:
    def __init__(self, name: str, ctx: WorkerContext, handlers: HandlerTable, *,
                 background: BackgroundStepper | None = None, poll_interval_s: float = 0.001,
                 after_each: Callable[[], None] | None = None) -> None: ...
    def put(self, req: Request) -> None: ...      # unbounded queue.Queue, never blocks (D11/H2–H5 fix)
    def requeue(self, req: Request) -> None: ...  # tail of the same queue, requeue_count += 1 (internal path)
    def start(self) -> None: ...                  # thread "aloxcam-<name>"
    def join(self, timeout: float | None = None) -> bool: ...
    def stop_now(self) -> None: ...               # puts a private sentinel (used only when the worker must end without Exit)
    @property
    def stopped(self) -> threading.Event: ...
    def qsize(self) -> int: ...
```

Loop: `get` blocks unless `background.has_work()` (then `get_nowait` and one `background.step()` per pass, 06 §9.5). If the dequeued
request was re-queued and the queue is otherwise empty, wait `poll_interval_s` first (LabVIEW's 1 ms `Wait`, 05 §6.3). Dispatch:
`decode_error` → `reject_undecodable`; no handler → DEBUG log, and with `reply_unknown_commands` `error_reply(kind, 9506)`; handler
exception → ERROR log with traceback, `err = cluster_of(exc)` (9500) finished like a decode error; `Outcome.requeue` → `requeue`;
`Outcome.stop` → `after_each()` once, set `stopped`, return. `after_each` runs after every dequeued request (camera: 
`state.record_camera_iteration`). Commands queued after `Exit` are discarded with the queue (03 §3.4).

#### 3.4.5 `aloxcam/server/tcp.py` (02 §3–§5, §9)

```python
class TcpServer:
    def __init__(self, *, host: str, port: int, link: ClientLink, router: Router, state: ServerState,
                 datatypes: Datatypes, compat: CompatSwitches, options: ServerOptions, tick: TickFn) -> None: ...
    def bind(self) -> tuple[str, int]: ...   # create the listener now; OSError propagates (02 D17: fail at start-up)
    @property
    def address(self) -> tuple[str, int]: ...
    def start(self) -> None: ...             # thread "aloxcam-tcp"
    def stop(self) -> None: ...              # close listener and current connection, join (≤ 2 s)
```

Behaviour: one client at a time, the listener stays open (backlog = waiting clients, 02 §3); `state.connected` TRUE on accept,
FALSE on disconnect (A6 fix); `state.heartbeat = tick()` each loop pass. Reading: wait for the first header byte with no timeout
(select with 0.2 s slices to notice `stop()`), then read the rest of the 6 header bytes and the payload with `io_timeout_s`;
mid-frame timeout, EOF, `N < 2` or `N − 2 > max_payload_bytes` → close the connection (02 §5.5, D5, D6). Datatype dispatch:
`Command` → `router.route_frame(payload, conn_id)`; `Heartbeat` with payload → close if `close_on_extra_heartbeat_data` else ignore
(02 §9, D13: no 25 s stall); `Heartbeat` without payload, `Response`, `Image` → ignore; unknown code → WARNING, ignore. After each
command frame sleep `inbound_pause_ms`. The socket timeout is set once at accept (shared with `ClientLink.send`, 01 §10.3).

#### 3.4.6 `aloxcam/server/runtime.py` (01 §7, §10.3)

```python
@dataclass
class RuntimeParts:                       # everything app.build_runtime assembles (tests may build it with fakes)
    config: ServerConfig
    host: str
    port: int
    enums: WireEnums
    routing: RoutingTable
    commands: Mapping[str, CommandSpec]
    payloads: PayloadCodec
    state: ServerState
    events: EventSink
    codec: ImageCodec
    tick: TickFn
    camera_handlers: HandlerTable
    z_handlers: HandlerTable
    make_camera_context: Callable[[WorkerContext, Callable[[Request], None]], WorkerContext]   # (base, forward_to_z)
    make_z_context: Callable[[WorkerContext], WorkerContext]
    camera_background: BackgroundStepper | None = None
    close_hardware: Callable[[], None] = lambda: None

class ServerRuntime:
    def __init__(self, parts: RuntimeParts) -> None: ...
    def start(self) -> tuple[str, int]: ...     # bind, start Z worker, camera worker, TCP thread; returns bound address
    def request_shutdown(self, reason: str = "signal") -> None: ...  # thread-safe, idempotent: posts an internal Exit to
                                                # the camera worker (runs after the queued camera commands); the chain below
                                                # then runs exactly as for a client Exit
    def wait(self, timeout: float | None = None) -> bool: ...        # True once fully stopped
    @property
    def address(self) -> tuple[str, int]: ...
    @property
    def state(self) -> ServerState: ...
    @property
    def link(self) -> ClientLink: ...
```

Shutdown chain (thread `aloxcam-shutdown`, started by `start()`, waits for the camera worker's `stopped`): `link.close()` (replies of
Z commands still queued are dropped, 01 §6.3) → `tcp.stop()` → join the Z worker (≤ `shutdown_timeout_s`; it ends on the forwarded
`Exit`) → `camera_background.abort()` → `close_hardware()` (exceptions logged) → stopped. Z-join timeout → ERROR log; `wait()`
still returns True after `close_hardware`. If the camera worker itself never reaches `Exit` (blocked in a hardware call), `wait()`
keeps returning False; `app.main` then gives up after `shutdown_timeout_s` with exit code 3 (all threads are daemons).

#### 3.4.7 `aloxcam/app.py` and `aloxcam/__main__.py` — CLI

```text
python -m aloxcam [--setup-ini PATH] [--settings-dir PATH] [--simulate] [--port N] [--listen ADDR]
                  [--log-level {DEBUG,INFO,WARNING,ERROR}] [--no-stderr-log] [--ready-file PATH]
                  [--check-config] [--write-settings-template PATH] [--version]
```

| Option | Effect |
|---|---|
| `--setup-ini PATH` | `alox_setup.ini` to use (default `<executable_folder()>/alox_setup.ini`) |
| `--settings-dir PATH` | overrides `[folders] settings` (folder of `alox.ini` and `alox_camera_server.ini`) |
| `--simulate` | use `SimCameraBackend` + `SimStage` (same as `[simulation] enabled = TRUE`). Phase 1 has no hardware backend: without it the program exits with code 2 and a message. |
| `--port N` | overrides `[server] port` and `port_I32` (`0` = ephemeral) |
| `--listen ADDR` | overrides `[compat] listen_address` |
| `--log-level`, `--no-stderr-log` | diagnostic log level / no stderr handler |
| `--ready-file PATH` | after bind, write `"<host> <port>\n"` atomically (tests, supervisors) |
| `--check-config` | load everything, print folders, effective port/address, enum tables (provisional flags), routing table, self-check problems, compat values; exit 0, or 1 on fatal problems |
| `--write-settings-template PATH` | write Appendix A (refuse to overwrite); exit 0 |

```python
@dataclass(frozen=True)
class LoadedConfig:
    folders: Folders; server: ServerConfig; alox: AloxSettings; enums: WireEnums; warnings: tuple[str, ...]
def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace: ...
def load_configuration(args: argparse.Namespace) -> LoadedConfig: ...
    # order: resolve_folders → load_server_config(<settings>/alox_camera_server.ini) → load_alox_ini(<settings>/alox.ini,
    # server.camera) → WireEnums.from_options; warnings collected and replayed into the diagnostic log once it exists
def build_runtime(loaded: LoadedConfig, *, simulate: bool, port: int | None = None, host: str | None = None,
                  tick: TickFn | None = None) -> ServerRuntime: ...
    # creates EventLogs (+ ensure_dirs), codec (imaging.codec.make_codec), backends, ZController, sessions,
    # PayloadCodec, routing (+ self-check logged), ServerState, handler tables, contexts, close_hardware
def main(argv: Sequence[str] | None = None) -> int: ...
```

Exit codes: 0 normal stop (`Exit` or signal); 1 start-up failure (bind error, `ConfigError`); 2 usage error / no backend; 3 shutdown
timeout. Signals: SIGINT/SIGTERM (and SIGBREAK on Windows) → `request_shutdown`; a second signal → `os._exit(130)`. Start-up logs (INFO):
version, folders, port, provisional enum tables (WARNING), compat values differing from defaults.

---

### 3.5 WP5 — Z stage

#### 3.5.1 `aloxcam/zstage/backend.py` — `StageBackend` (GCS level, 08 §3.4, §13.2)

One method per LabVIEW PI driver VI used by the server. Implementations format numbers with 4 decimals (`%.4f`, `.` separator),
never send `ERR?` implicitly (pipython `errcheck = False`), and raise `StageError` with the codes/texts of 08 §3.5. Axis arguments are
GCS axis ids (`"1"`); the "No axis specified" pre-checks are done by `ZController`. Not thread-safe by itself: `ZController` serialises.

```python
class StageBackend(ABC):
    def connect_rs232(self, port: str, baudrate: int, timeout_ms: int) -> None: ...  # PI Open Interface incl. purge
                                                                                      # XXX/*IDN?/ERR? (08 §3.2); -6 on failure
    def close(self) -> bool: ...                       # "Close connection if open"; returns was_connected
    @property
    def is_connected(self) -> bool: ...
    def qerr(self) -> int: ...                         # ERR? → code (0 none); empty answer → StageError(100, "No answer on ERR?")
    def qidn(self) -> str: ...                         # final LF stripped
    def qsai(self) -> list[str]: ...
    def qcst(self, axis: str) -> str: ...
    def svo(self, axis: str, on: bool) -> None: ...
    def qsvo(self, axis: str) -> bool: ...
    def qron(self, axis: str) -> bool: ...
    def qtrs(self, axis: str) -> bool: ...
    def qlim(self, axis: str) -> bool: ...
    def qtmn(self, axis: str) -> float: ...
    def qtmx(self, axis: str) -> float: ...
    def mov(self, axis: str, pos: float) -> None: ...  # "MOV 1 12.5000", no ERR?
    def mvr(self, axis: str, dist: float) -> None: ...
    def vel(self, axis: str, speed: float) -> None: ...   # "VEL 1 10.0000" only (ZController adds the ERR?)
    def qvel(self, axis: str) -> float: ...
    def qpos(self, axis: str) -> float: ...
    def qfrf(self, axis: str) -> bool: ...
    def frf(self, axis: str) -> None: ...
    def fnl(self, axis: str) -> None: ...
    def fpl(self, axis: str) -> None: ...
    def stp(self, axis: str | None) -> None: ...      # "STP 1" as wired (08 §5.3, Z7); None → "STP"
    def qrmc(self) -> str: ...                         # final LF stripped
    def mac_start(self, name: str) -> None: ...
    def qmac(self, name: str) -> str: ...
    def jax(self, joystick_id: int, joystick_axis: int, axis: str) -> None: ...
    def jon(self, joystick_id: int, enable: bool) -> None: ...     # no ERR? (ZController adds it)
    def qjon(self, joystick_id: int) -> int: ...
    def is_moving(self) -> bool: ...                   # #5, any connected axis
    def is_controller_ready(self) -> bool: ...         # #7; unknown reply → StageError(100, "#7 Unknown reply")
    def stop_all(self) -> None: ...                    # #24
    def translate_error(self, code: int) -> str: ...   # GCSTranslateError description, "" when unknown (08 §11)
```

#### 3.5.2 `aloxcam/zstage/gcs_errors.py`

`GCS_ERROR_TEXT: Mapping[int, str]` (codes 0, 1, 2, 5, 7, 10, 15, 17, 51, 1008, −7, 100 with PI's texts) and
`gcs_error_text(code) -> str`. Used by `SimStage.translate_error`; the Phase-2 pipython backend uses `GCSError.translate_error`.

#### 3.5.3 `aloxcam/zstage/sim.py` — `SimStage(StageBackend)`

```python
class SimStage(StageBackend):
    def __init__(self, options: SimulationOptions, *, clock: Callable[[], float] = time.monotonic) -> None: ...
    # test hooks (not part of StageBackend):
    def inject_failure(self, method: str, error: StageError, *, times: int = 1) -> None: ...
    @property
    def error_register(self) -> int: ...
    @property
    def position(self) -> float: ...
```

Behaviour (single axis `"1"`, deterministic given `clock`): `connect_rs232` accepts any port, clears the error register (purge,
08 Z22); not connected → `StageError(-4, "<CMD>: not connected")`. Linear motion at the current velocity from the position at command
time; `is_moving` true until arrival. Controller errors are **latched**, not raised (08 §3.5): `MOV`/`MVR` on an unreferenced axis →
5, target outside `[stage_min_mm, stage_max_mm]` → 7, joystick enabled → 51; `STP` stops and latches 10; `VEL ≤ 0` latches 17;
`MAC START` while a macro runs latches 1008. `qerr` returns and clears the register. `FNL` moves to `stage_min_mm` in
`stage_reference_s`, then referenced; `FRF` → mid-travel, referenced; `is_controller_ready` false while referencing. `qron`/`qlim`
TRUE, `qtrs` FALSE (so set-up uses `FNL`, 08 §4.2 step 12), `qcst` `"SIM-STAGE"`. `MAC START DH` → `qrmc` returns `"DH"` for
`stage_dh_macro_s`, then `""`. `qidn` = `options.stage_identity`.

#### 3.5.4 `aloxcam/zstage/controller.py` — `ZController` (08 §4, §6, §8, §12, §13.3)

```python
@dataclass(frozen=True)
class ZSession:                           # alox_typedef_ext_zstage_session.ctl (08 §2.2); one live copy (08 Z18)
    system: int = 0
    axes: tuple[str, ...] = ()
    identity: str = ""
    serial_number: str = ""

def serial_from_idn(identity: str) -> str: ...   # 3rd comma field (08 §13.3, Z21)

class ZController:
    """The single link to the controller. Thread-safe: every GCS request/answer (and VEL+ERR?, JON+ERR? pairs) runs under
    `lock`; set-up holds it for the whole sequence. Phase 4 focus code uses the same instance."""
    lock: threading.RLock
    def __init__(self, backend: StageBackend, options: StageOptions, *,
                 sleep: Callable[[float], None] = time.sleep, monotonic: Callable[[], float] = time.monotonic) -> None: ...
    @property
    def session(self) -> ZSession: ...
    def setup(self, s: ZStageSettings) -> None: ...   # 08 §4.1–4.2 / §13.3 setup() + VEL normal speed; session written even
                                                      # on failure; error translated (step 19)
    def close(self) -> bool: ...
    def reset_session(self) -> None: ...              # Z Cleanup
    def check_error(self) -> None: ...                # ERR? → StageError(code, "Controller error")
    def translate(self, err: StageError) -> StageError: ...   # appends "\nGCS Error: <text>" when known (08 §11)
    def gcs_error_text(self, code: int) -> str: ...
    def mov(self, pos: float) -> None: ...; def mvr(self, dist: float) -> None: ...
    def vel(self, speed: float) -> None: ...          # VEL + ERR?
    def qvel(self) -> float: ...; def qpos(self) -> float: ...; def qfrf(self) -> bool: ...
    def fnl(self) -> None: ...; def frf(self) -> None: ...; def stp(self) -> None: ...
    def running_macros(self) -> str: ...; def mac_start(self, name: str) -> None: ...
    def jax(self, joystick_id: int, joystick_axis: int) -> None: ...
    def jon(self, joystick_id: int, enable: bool) -> None: ...   # JON + ERR?
    def qjon(self, joystick_id: int) -> bool: ...                # value ≠ 0
    def is_moving(self) -> bool: ...
    def wait_ready(self, timeout_s: float | None = None) -> None: ...                 # #7 polling, 50 ms; -7 on timeout
    def general_wait(self, add_wait_ms: int = 0, polling_ms: int = 10, add_wait_only: bool = False,
                     timeout_s: float | None = None) -> None: ...                    # 08 §8 (additional wait also after failure)
    def check_axis_reference(self, s: ZStageSettings) -> tuple[bool, bool, StageError | None]: ...  # 08 §6
    def startup_macro_check(self) -> None: ...        # 08 §2.4, only if options.startup_macro_check and connected
```

Set-up order (08 §4.2 table / §13.3 `setup()`): `connect_rs232(s.port_name, s.baudrate, 5000)` (its purge replaces the sketch's
explicit `ERR?` read) → `qidn` → `qsai` → session → `qcst` (≠ `"NOSTAGE"`) → `svo` → `qsvo` → `qron` → (`always_reference` or not
`qfrf`) → `qtrs` ? `frf` : `qlim` ? `reference_move` → `wait_ready` → `qfrf` → `qtmn`/`qtmx`/`qpos` → `check_error` (translated) →
`vel(s.speed)`. The port name passes through `options.port_map` when it has an entry.

Axis pre-check: methods needing an axis raise `StageError(100, text)` when `session.axes` is empty, with the texts of 08 §3.3
(`"MOV No axis specified"`, `"MVR no axis specified"`, `"VEL No axis or velocity specified"`, `"STP no axis specified"`, else
`"<CMD> No axis specified"`), without calling the backend.

#### 3.5.5 `aloxcam/zstage/handlers.py` (03 §7, 08 §5, §13.4)

```python
@dataclass
class ZContext(WorkerContext):
    stage: ZController
    stage_options: StageOptions
Z_HANDLERS: HandlerTable      # the 19 Z commands + "Exit"; every handler finishes with dispatch.finish_z (order Z)
```

| Command | Contract (spec) | Phase-1 specifics |
|---|---|---|
| `Z Setup` | 08 §4.1 | `stage.setup(state.z)`; `state.z_inited = ok` |
| `Z Get Serial` | 08 §5.3 | no I/O; `Type` TRUE → identity, FALSE → serial |
| `Z Get Status` | 08 §5.3 | `restart_dh_macro`; updates `running_macros`, `macro_running` |
| `Z Get Error` | 08 §5.3, §11 | value = `gcs_error_text(code)`, error translated |
| `Z Set Speed` | 08 §5.3 | settings unchanged (03 C14) |
| `Z Get Speed` / `Z Get Reference` / `Z Get Position` | 08 §5.3 | defaults 0.0 / FALSE / 0.0 with the error; Get Position writes `state.z_pos` even on error |
| `Z Do Reference` | 08 §5.3 | label `"FRF"` → FRF, anything else FNL (WARNING if not a known item); `do_reference_waits_ready` (Z13 fix) then `general_wait(polling_ms=10)` then `check_error`; error translated |
| `Z Move Absolute` / `Z Move Relative` | 08 §5.3, §6, §7 | `implicit_z_reinit` (FALSE → `{TRUE, 5, "Z axis not referenced"}`); re-init warning → `events.info(…, "Move Absolute: Z Reference resetted")`; inclusive sorted range, NaN out of range → 9405 replacing earlier errors; `state.z_pos` on success; reinit error → target 0.0 for the range test (Absolute, inferred); no wait, no `ERR?` (C13) |
| `Z Wait End Move` | 08 §8 | negative ms → 0; timeout `stage_options.wait_timeout_s` |
| `Z Stop` | 08 §5.3 | `"STP 1"`; `restart_dh_macro` |
| `Z Enable/Disable Joystick`, `Z Joystick Enabled ?` | 08 §9 | `JAX` then `JON`(+`ERR?`); BOOL = `JON?` ≠ 0 |
| `Z Set Min/Max Limit` | 08 §5.3, 03 C15 | `state.replace_z_settings`; non-finite → `{TRUE, 1, "<Command>: value must be finite"}`, limit unchanged |
| `Z Cleanup` | 08 §5.3, Z9 | close; with `restart_dh_macro` `RMC?` fails (−4) → error ACK; session reset; `z_inited` not reset |
| `Exit` | 03 §7.19 | `STOP`, no reply, no log |

---

### 3.6 WP6 — cameras and imaging

#### 3.6.1 `aloxcam/camera/backend.py` (05 §2, §12.3)

```python
class ValueType(Enum):                    # IMAQdx property accessors (05 §5.1)
    DBL = "DBL"; I64 = "I64"; U32 = "U32"; BOOL = "BOOL"; STRING = "STRING"

@dataclass(frozen=True)
class Frame:
    number: int                           # cumulative buffer number (U32), 0 = first frame after configure
    pixels: np.ndarray                    # owned 2-D copy
    t_monotonic: float

class CameraBackend(ABC):                 # ~ the NI-IMAQdx driver
    def open_camera(self, name: str) -> "CameraDevice": ...   # IMAQdx Open Camera ("Controller"); CameraError if unknown
    def close(self) -> None: ...                               # driver release at shutdown; idempotent

class CameraDevice(ABC):                  # ~ one IMAQdx session; all failures raise CameraError
    name: str
    def bus_type(self) -> str: ...
    def serial_number(self) -> tuple[int, int]: ...
    def set_packet_size(self, size: int) -> None: ...
    def configure(self, continuous: bool, num_buffers: int) -> None: ...   # Grab / Sequence (05 §4.3)
    def unconfigure(self) -> None: ...
    def close(self) -> None: ...          # stops a running acquisition first; idempotent
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def acquisition_in_progress(self) -> bool: ...
    def last_buffer_number(self) -> int | None: ...   # None before the first frame
    def buffer_count(self) -> int: ...
    def get_buffer(self, number: int | None, timeout_ms: int) -> Frame: ...  # "Buffer Number" mode; None or overwritten → newest
    def get_next(self, timeout_ms: int) -> Frame: ...                         # "Next" (Phase 4 focus)
    def get_last_new(self, timeout_ms: int) -> Frame: ...                     # "Last New" (Cam Get Image)
    def read_attribute(self, name: str, vtype: ValueType) -> float | int | bool | str: ...
    def write_attribute(self, name: str, vtype: ValueType, value: float | int | bool | str) -> None: ...
    def read_attribute_max(self, name: str) -> float: ...
```

Attribute names are IMAQdx paths (`CameraAttributes::…`); a real backend maps them to GenICam names (05 §12.4). A command
attribute (`TriggerSoftware`) is executed by writing `BOOL True`. Threading: called from the camera worker only (plus `close()` from
the shutdown path after the worker stopped); implementations guard their own grab thread.

#### 3.6.2 `aloxcam/camera/sim.py`

```python
class SimCameraBackend(CameraBackend):
    def __init__(self, options: SimulationOptions, *, clock: Callable[[], float] = time.monotonic,
                 manual: bool = False) -> None: ...      # manual: no grab thread, frames only via emit_frame()
    devices: Mapping[str, "SimCameraDevice"]            # opened devices by name (tests)
class SimCameraDevice(CameraDevice):
    def emit_frame(self) -> Frame: ...                  # test hook: produce one frame now
    def fail_attribute(self, name: str, error: CameraError | None) -> None: ...   # inject/clear a failure
```

Behaviour: unknown name (when `camera_names` is set) → `CameraError(9502, "IMAQdx Open Camera: camera '<n>' not found (simulation)")`;
`bus_type` `"Simulator"`; frames `image_height × image_width` of `pixel_type`: horizontal gradient scaled by exposure, plus the frame
number written big-endian into the first 4 pixels of row 0 (lets tests identify buffers). Continuous: grab thread at the
`AcquisitionFrameRate` attribute (≤ `max_frame_rate`); one-shot: `num_buffers` frames then the counter stops
(`acquisition_in_progress` stays TRUE). `TriggerMode = "On"` → one frame per `TriggerSoftware`. Ring of `num_buffers`; numbering
from 0 after `configure`. Node map: exposure (`CameraAttributes::Exposure::Value` and `…AcquisitionControl::ExposureTime`, DBL,
default 10000), gain (`CameraAttributes::Gain::Value`, `…AnalogControl::Gain`), `AcquisitionFrameRate` (DBL, max =
`max_frame_rate`), `TriggerMode` (STRING `On`/`Off`), `TriggerSoftware` (command), `DeviceTemperature` (DBL =
`temperature_c`), `DigitalIOControl` `LineSelector`/`LineMode`/`LineSource`/`LineInverter` per line 0–3, `UserOutputSelector`,
`UserOutputValue`. Numeric nodes accept DBL/I64/U32; wrong value type or unknown name → `CameraError(9502, …)`. `get_last_new`
when not acquiring → `CameraError(9502, "IMAQdx Get Image2: acquisition not in progress")`; timeout → `CameraError(9502, "…timeout")`.

#### 3.6.3 `aloxcam/camera/attributes.py` (05 §2.1, §5.1)

```python
TRIGGER_MODE = "CameraAttributes::AcquisitionControl::TriggerMode"
TRIGGER_SOFTWARE = "CameraAttributes::AcquisitionControl::TriggerSoftware"
DEVICE_TEMPERATURE = "CameraAttributes::DeviceControl::DeviceTemperature"
DIO = "CameraAttributes::DigitalIOControl::"   # + LineSelector, LineMode, LineSource, LineInverter, UserOutputSelector, UserOutputValue
GIGE_PACKET_SIZE = 8996; NUM_BUFFERS = 5; GET_IMAGE_TIMEOUT_MS = 5000; EXPOSURE_UPPER_LIMIT = 2_000_000.0
def value_kind(type_str: str) -> ValueType: ...           # "I64"/"U32" else DBL (case-sensitive)
def lv_to_int(x: float, vtype: ValueType) -> int: ...     # round half to even + saturate; NaN → CameraError(1, …)
def write_typed(dev: CameraDevice, attr: str, type_str: str, value: float) -> None: ...
def read_typed(dev: CameraDevice, attr: str, type_str: str) -> float: ...
def in_range_and_coerce(x: float, lo: float, hi: float) -> float: ...   # min(max(x, lo), hi)
def format_lv_double(x: float, decimal_separator: str = ".") -> str: ... # LabVIEW default "%f" (6 decimals) (inferred)
```

#### 3.6.4 `aloxcam/camera/pipeline.py` (05 §7, §8)

```python
CUSTOM_DATA_KEYS = ("buffer", "timestamp", "name", "type", "simulation", "flatfield")
def flip_image(img: np.ndarray, horizontal: bool, vertical: bool) -> np.ndarray: ...      # 05 §7.2 (H = top↔bottom)
def flat_field_correct(img: np.ndarray, flat: np.ndarray | None, dark: np.ndarray | None, factor: float) -> np.ndarray: ...
    # inferred formula of 05 §7.3; factor coerced to float32; missing image / shape mismatch → CameraError(9509, …)
def lv_timestamp_text(now: float, decimal_separator: str = ".") -> str: ...            # "%H:%M:%S:%3u" → "HH:MM:SS:.mmm"
def build_custom_data(buffer_number: int, settings: CameraSettings, camera_type: str, *, now: float,
                      decimal_separator: str = ".") -> dict[str, str]: ...             # 05 §7.5 (6 keys)
def to_u8_like_simulation(img: np.ndarray) -> np.ndarray: ...                         # 05 §8.2; -9011 "Invalid Type"
def list_simulation_files(folder: Path) -> list[str]: ...                             # *.png, case-insensitive name order
def simulate_next_image(folder: Path, timer_ms: int, index_in: int, time_in: int,
                        tick: TickFn) -> tuple[np.ndarray, int, int]: ...             # 05 §8.2 (re-listed every call)
def load_correction_image(path: Path) -> np.ndarray: ...                              # PNG via imaging.png; else 9501
```

#### 3.6.5 `aloxcam/camera/acquisition.py` (05 §6.3–6.4, §12.7)

```python
class AcqState(str, Enum):
    START = "Start"; ACQUIRE = "Acquire"; FINISHED = "Finished"; ERROR = "Error"; STOPPED = "Stopped"

class GetImageStateMachine:               # one instance PER CAMERA (05 #3 fix)
    def __init__(self, *, request_buffer_zero: bool, strict_new_frame: bool, tick: TickFn,
                 decimal_separator: str = ".") -> None: ...
    t0: int
    t0_buf: int                           # -1 when no frame existed at Start (buffer 0 counts as new)
    def step(self, state: AcqState, stream: CameraStreamData, session: CameraSession, settings: CameraSettings,
             error_in: ErrorCluster) -> tuple[AcqState, ErrorCluster]: ...
    def reset(self) -> None: ...
```

`step` implements 05 §6.3 (outer error case, Start, Acquire camera/simulation) writing `session.src_image`, `session.custom_data`
and the simulation fields. "New buffer available" = `last > t0_buf` (`last ≥ t0_buf + 2` with `strict_new_frame`); requested
buffer = 0 with `request_buffer_zero`, else newest. 9009 source = `"get image error (timeout)" + f"lastbuffer:{last}/ buffercount:{count}/ buffer t-1:{t0_buf & 0xFFFFFFFF}"`
(`last` = 0 when None). `LastBufferNumber`/`BufferCount` read errors in Acquire are ignored (05 #27) but logged at DEBUG. Simulation
path: no flip, no custom data (custom data cleared), flat-field applied. The waiting diagnostic `{FALSE, 0, "lastbuffer:…"}` is not
reproduced (05 #21).

#### 3.6.6 `aloxcam/camera/session.py` (05 §3.2, 06 §2.5)

```python
@dataclass
class FocusFields:
    af_state: str = "Start"; rf_state: str = "Start"
    last_focus_data: np.ndarray = field(default_factory=lambda: np.empty((0, 0)))
    focus_progress: float = 0.0
    last_error: ErrorCluster | None = None

@dataclass
class CameraSession:                      # touched only by the camera worker thread
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
    def require_device(self, operation: str) -> CameraDevice: ...
        # CameraError(Code.IMAQDX_INVALID_SESSION, f"{operation}: invalid camera session") when device is None
    def reset(self) -> None: ...          # Cam Cleanup: typedef defaults (05 §4.6), state machine reset
```

#### 3.6.7 `aloxcam/camera/handlers.py` (03 §5–§6, 05, 06 §3)

```python
@dataclass
class CameraContext(WorkerContext):
    backend: CameraBackend
    sessions: Mapping[str, CameraSession]          # "High Mag." / "Low Mag."
    forward_to_z: Callable[[Request], None]
    camera_options: CameraOptions
    decimal_separator: str = "."
    stage: ZController | None = None               # Phase 4 focus jobs
    sleep: Callable[[float], None] = time.sleep    # Wait LED, LM USB 100 ms (tests patch it)
CAMERA_HANDLERS: HandlerTable                      # all camera-worker commands of COMMANDS except the two "… Run"
def close_cameras(sessions: Mapping[str, CameraSession]) -> None: ...   # shutdown: stop + close devices, errors logged
```

| Command | Spec | Phase-1 specifics (beyond the spec pseudo-code) |
|---|---|---|
| `Get Heartbeat` | 03 §5.1 | `u32(tick())`, `NO_ERROR` |
| `Show UI` | 03 §5.2 | `state.show_ui = show`; ACK `NO_ERROR` (no GUI) |
| `Log` | 03 §5.3 | `state.cam_log_data = data`; ACK `NO_ERROR` |
| `Exit` | 03 §5.4 | `forward_to_z(Request.make_internal("Exit", …))`; `STOP`; no reply/log |
| `Cam Setup` | 05 §4.2 | closes the previous device first (05 #9); 5 buffers always, INFO log if `number_of_buffers ∉ {0, 5}` (05 #8); GigE `set_packet_size(8996)`, LM USB `sleep(0.1)`; LM suffix = `"Cam Setup"` + sources of Configure, PacketSize, Open errors; `state.update_camera(inited, acq_state="Stopped", bus_type)`, HM serials |
| `Cam Start` | 05 §4.4 | GigE/USB path from the camera's **own** `plug_type` for both cameras (05 #12 fix) |
| `Cam Stop` | 05 §4.5 | also sets `session.acquisition_state = Stopped` (05 #11) |
| `Cam Cleanup` | 05 §4.6 | `inited` FALSE first; `session.reset()` |
| `Cam Get Acquisition Status` | 05 §4.7 | clears −1074360316 |
| `Cam Software Trigger` | 05 §4.8 | always `NO_ERROR` |
| `Cam Set Exposure` | 05 §5.2 | `exposure_sets_max_framerate` (FALSE, 10 §6 fixed behaviour: HM writes `fr_calc` with its `framerate_type`, DBL included; its upper bound is `hm_max_framerate` when > 0, else the attribute's `MaxDBL`); LM unchanged; source replaced; `fr_calc` text via `format_lv_double`; non-finite → error 1 |
| `Cam Get Exposure`, `Cam Set Gain`, `Cam Get Gain` | 05 §5.3–5.4 | suffix = attribute + type string |
| `Cam Stream Next Image` | 05 §6.2 | `temperature_error_fails_image` (FALSE: log the temperature error at WARNING, pass `NO_ERROR` to the state machine); returns `REQUEUE` while `Acquire`; replies `image(ImageData(src_image, custom_data), err)` on `Finished`; logs with the real camera type (05 #26) |
| `Cam Get Image` | 05 §6.5 | no reply, no event log; result only in `session.display_image` / `state.error_out` |
| `Cam Get Images Array` | 03 §5.17 | nothing |
| `Cam Flip` | 05 §7.2 | `state.replace_camera_settings` |
| `Cam Set FlatField` | 05 §7.3 | `flatfield_check_on_set` (TRUE: missing file → `{TRUE, 9501, "<path>: flat-field file not found"}`); WARNING log when enabled and a file is missing; `src_image = None` |
| `Cam Set Led` | 05 §9.2, §12.8 | `state.update(led_cmd, light_trace)`; `ctx.sleep(wait_led_ms/1000)` |
| `Cam Get Led` | 05 §9.3 | `state.update(led_status)` |
| `Cam Autofocus`, `Cam Refocus` | 06 §3.1 | **Phase-1 stub**: `err = {TRUE, 9507, "not implemented in this phase (phase 4)"}` finished in order A (suffix appended, logged, ACK → silent while `send_error_replies` = FALSE); focus fields as a failed start (`af/rf_state = "Error"`, data 0×0, progress 0) |
| `Cam Get Autofocus Data`, `… Progress` | 06 §3.3 | fully implemented (read `session.focus`) |

General rules: every handler selects the session/settings by the decoded `camera_type` label (already validated); every error path
builds `ErrorCluster`s from `CameraError` with `cluster()`; non-finite numeric inputs → `{TRUE, 1, "<attribute>: invalid value"}`
without touching the camera.

#### 3.6.8 `aloxcam/imaging/png.py`

```python
class PngError(Exception): ...            # malformed file
class PngUnsupported(PngError):           # valid PNG, unsupported format; .color_type, .bit_depth
    ...
def read_png(path: Path | str) -> np.ndarray: ...
    # non-interlaced, all 5 filters; gray 8/16 → (h, w) uint8/uint16; RGB/RGBA 8 → (h, w, 3|4) uint8; RGB/RGBA 16 →
    # (h, w, 3|4) uint16; palette, gray+alpha, bit depths < 8, interlaced → PngUnsupported
def write_png(path: Path | str, array: np.ndarray) -> None: ...   # (h, w) uint8/uint16 gray, filter 0
```

Mapping to the simulation (05 §8.2): gray U8 → Mono8; gray 16 → Mono16 (`>> 8`); RGB(A) 8 → luminance `0.299 R + 0.587 G + 0.114 B`
rounded (inferred NI weights); everything else (RGB 16 = NI RGB U64, `PngUnsupported`) → −9011 `"Invalid Type"`; `PngError`/`OSError`
→ 9501.

#### 3.6.9 `aloxcam/imaging/codec.py` (02 §8.5)

```python
PROVISIONAL_MAGIC = b"ALXIMG01"
class ProvisionalCodec:                   # ImageCodec; name = "provisional"
    def flatten(self, image: ImageData) -> bytes: ...
def decode_provisional(data: bytes) -> ImageData: ...    # ProtocolError on malformed data (client side)
class NiVisionCodec:                      # ImageCodec stub; name = "nivision"
    def __init__(self, library: str = "") -> None: ...  # never raises
    def is_available(self) -> bool: ...                 # ctypes can load the library (Windows only)
    def flatten(self, image: ImageData) -> bytes: ...   # Phase 1: CodecError(9508, "NI Vision image flattening is not
                                                        # implemented in this phase (phase 3)")
def make_codec(options: ImageOptions) -> ImageCodec: ...  # ConfigError for unknown names; WARNING when nivision unavailable
```

**Provisional format — NOT the NI format** (the real client cannot decode it; it exists so the server is usable and testable until
Phase 3). All big-endian: `"ALXIMG01"` · U16 NI image type (`ImageData.ni_image_type`) · U16 0 · I32 width · I32 height · I32
custom-data count *n* · *n* × (LabVIEW string key, LabVIEW string value) · I32 pixel byte count · pixels row-major big-endian.

---

### 3.7 WP7 — client library and replay tool

#### 3.7.1 `aloxcam/client.py` (02 §10)

```python
@dataclass(frozen=True)
class ReceivedReply:
    notifier: bytes; flattened: bytes; ts: ResponseTimestamps; frame: bytes; received_at: float
    def value(self, kind: ReplyKind, *, encoding: str = "cp1252") -> tuple[Any, ErrorCluster]: ...

class AloxClient:
    """Protocol client: a reader thread demultiplexes replies by notifier name. Thread-safe."""
    def __init__(self, host: str, port: int, *, enums: WireEnums, payloads: PayloadCodec, timeout_s: float = 5.0) -> None: ...
    def connect(self) -> None: ...
    def close(self) -> None: ...
    def __enter__(self) -> "AloxClient": ...; def __exit__(self, *exc: object) -> None: ...
    @property
    def connected(self) -> bool: ...      # False once the server closed the connection
    def send(self, command: str, data: Any | None = None, *, notifier: bytes | None = None,
             ts: tuple[int, int, int] = (0, 0, 0), data_bytes: bytes | None = None) -> bytes: ...   # returns the notifier
    def send_raw(self, frame: bytes) -> None: ...
    def send_heartbeat(self, payload: bytes = b"") -> None: ...
    def wait_reply(self, notifier: bytes, timeout_s: float | None = None) -> ReceivedReply | None: ...  # None = timeout
    def call(self, command: str, data: Any | None = None, *, timeout_s: float | None = None) -> ReceivedReply | None: ...
    def wait_closed(self, timeout_s: float) -> bool: ...
```

Default notifiers `b"py-<pid>-<seq>"`; `data_bytes` overrides the encoded data part (malformed-payload tests).

#### 3.7.2 `tools/replay_client.py` — CLI and script format

```text
python tools/replay_client.py SCRIPT.toml [--host H] [--port P] [--timeout S] [--var NAME=VALUE ...]
       [--settings-ini PATH] [--enums PATH] [--templates PATH] [--record OUT.jsonl] [--compare REF.jsonl]
       [--mask tick|image ...] [--stop-on-fail] [--quiet]
exit: 0 = all steps PASS (and no compare difference), 1 = a FAIL or difference, 2 = usage/connection error
```

`--settings-ini` reads `[wire]` (enum overrides/layouts) from an `alox_camera_server.ini`; `--templates` is a TOML of captured
variant headers `{ "<command>" = "<hex>" }`.

Script (TOML, `tomllib`):

```toml
[session]                                # all optional; CLI options override
host = "127.0.0.1"
port = 503
timeout_s = 5.0                          # reply timeout
no_reply_wait_s = 0.5                    # how long `reply = "none"` waits
mask = ["tick"]                          # default compare masks

[vars]                                   # ${name} substitution in every string; --var overrides
cam_hm = "cam0"

[[step]]
name = "setup HM"                        # optional label
command = "Cam Setup"                    # Camera Command label
data = { camera_type = "High Mag.", camera_name = "${cam_hm}", number_of_buffers = 5, continuous_mode = true }
# data_hex = "0001"                      # replaces the encoded data part (malformed payload tests)
# notifier = "fixed-name"                # default "replay-<index>"
# timestamps = [1000, 1001, 0]
# wait = false                           # send and continue; collect later with `await`
expect = { reply = "ACK", error = { status = false, code = 0 } }

[[step]]
await = "fixed-name"                     # reply of an earlier wait=false step
expect = { reply = "DBL", value = 1500.0, abs_tol = 1e-9 }

[[step]]
pause_s = 0.2

[[step]]
heartbeat_frame = true                   # Heartbeat datatype frame; payload_hex = "…" optional
expect = { reply = "none" }              # or connection = "closed"

[[step]]
raw_hex = "00 00 00 02 00 03"            # raw bytes (whitespace ignored)
expect = { reply = "none" }

[[step]]
code = 32767                             # a command frame with this numeric code and an empty variant (unknown commands)
expect = { reply = "none" }

[[step]]
command = "Cam Get Autofocus Progress"
data = { camera_type = "High Mag." }
poll = { max_tries = 50, interval_s = 0.1, until_ge = 100.0 }   # repeat until the value condition holds
expect = { reply = "DBL" }
```

A step has exactly one of `command`, `code`, `await`, `pause_s`, `heartbeat_frame`, `raw_hex`. Inline tables must stay on one line
(TOML 1.0); `[step.data]` / `[step.expect]` sub-tables after `[[step]]` are equivalent. `expect` keys: `reply` ∈ `ACK STR BOOL DBL U32
DBL2D IMAGE none any` (for `code` steps the kind is ACK); `error = {status, code, source, source_contains}`; `value`
(scalar, list or list of lists); `abs_tol`, `rel_tol`; `shape = [rows, cols]`; `image = {width, height, ni_image_type,
custom_data_keys = […], custom_data = {…}}` (decoded with `decode_provisional`); `connection = "closed"`; `max_latency_ms`. The reply
kind used to decode a reply is the command's `CommandSpec.reply`. Output: one line per step,
`[  7] PASS Cam Get Exposure  DBL 1500.0  err={F,0,""}  2.1 ms`.

Record (`--record`, JSON Lines): `{"step", "name", "command", "notifier", "request_hex", "reply_hex" | null, "reply_kind",
"latency_ms"}`. Compare (`--compare`) pairs steps by index and compares `reply_hex` after masking: `tick` = `TCP Sending timestamp`
and the U32 value of `Get Heartbeat`; `image` = image bytes and the request/receiving/pre/post timestamps of image replies.

```python
def load_script(path: Path, *, extra_vars: Mapping[str, str] = {}) -> Script: ...
def run_script(script: Script, *, host: str | None = None, port: int | None = None, enums: WireEnums | None = None,
               payloads: PayloadCodec | None = None, record: Path | None = None,
               on_result: Callable[[StepResult], None] | None = None) -> RunReport: ...
def compare_records(actual: Path, reference: Path, masks: Sequence[str]) -> list[str]: ...
def main(argv: Sequence[str] | None = None) -> int: ...
# Script, Step, Expect, StepResult (ok, message, reply_kind, value, error, latency_ms, request_hex, reply_hex),
# RunReport (results, ok, summary()) are dataclasses.
```

---

## 4. Work packages

### 4.1 Overview

| WP | Title | Owns | Needs first | May start against (before the dependency lands) |
|---|---|---|---|---|
| WP1 | Foundation: shared types | `pyproject.toml`, `__init__.py`, `errors.py`, `clock.py`, `settings.py`, `state.py`, `dispatch.py`, `imagedata.py`, `testing.py`, `tests/conftest.py`, `tests/foundation/*` | — | — (small; lands first, ~1–2 days) |
| WP2 | LabVIEW codec and wire layer | `lv/*`, `wire/*`, `data/wire_enums.toml`, `tests/golden/*`, `tests/wire/*` | WP1 | — . **Milestone M2a** (first commit, before the codec internals): `lv/path.py`, `wire/enums.py`, the TOML, `wire/messages.py` dataclasses + `COMMANDS` (with `PayloadCodec` signatures; bodies may follow) |
| WP3 | Configuration and event logs | `config/*`, `eventlog.py`, `tests/config/*` | WP1, WP2-M2a (`lv.path.localize_path_text`) | `NIConfig`, folders and `EventLogs` need nothing from WP2; only the path localization in `alox_ini`/`folders` does |
| WP4 | Server runtime and entry point | `server/*`, `app.py`, `__main__.py`, `tests/server/*` | WP1 | §3.2 APIs (router tests need M2a); fake codec/handlers from `aloxcam.testing`; `app.build_runtime` integrates WP3/5/6 last |
| WP5 | Z stage | `zstage/*`, `tests/zstage/*` | WP1, WP2-M2a | `aloxcam.testing` doubles; no dependency on WP3/WP4 |
| WP6 | Cameras and imaging | `camera/*`, `imaging/*`, `tests/camera/*`, `tests/imaging/*` | WP1, WP2-M2a | `aloxcam.testing` doubles; `ZController` only as a type (Phase 4) |
| WP7 | Client, replay tool, integration | `client.py`, `tools/*`, `tests/tools/*`, `tests/integration/*`, `README.md` | WP2 (client); all (integration) | client + replay tool against WP2 only (unit tests with a socketpair fake server); integration last |

Critical path: WP1 → (WP2 M2a) → WP4/WP5/WP6 → WP7 integration. WP3 needs only WP1 plus the M2a path helpers. WP6 refers to
`ZController` only under `typing.TYPE_CHECKING` (no import-time dependency on WP5); WP4 imports WP3/WP5/WP6 only inside
`app.build_runtime`.

### 4.2 Acceptance tests per package

Every package ships its tests; a package is done when its tests pass and `python -m pytest tests/<pkg>` needs nothing from a later
package.

**WP1** — `tests/foundation/`
- `test_errors.py`: `decorate` only when status TRUE (warnings untouched); `replace_source`; `merge_errors` order (error > warning >
  none); `clear_errors` all/specific; `error_cluster_from_error_code` (code 0, existing error wins, `<ERR>\n` text);
  `cluster_of(ValueError)` → 9500; `Code` values equal §3.1.1.
- `test_clock.py`: wrap-around of `tick_add`/`tick_diff`; `make_tick_source` names.
- `test_settings.py`: defaults equal 09 §5.6 (HM vs LM: name, invert_led, max_framerate); `port_name` (`com ≤ 0` → COM1); every
  `CompatSwitches` default equals 10 §6.
- `test_state.py`: `record_command` newest first, bounded 500; histories bounded; `replace_camera_settings` returns new object;
  concurrent writers (threads) never lose updates; `snapshot` copies.
- `test_dispatch.py`: finish helpers with `RecordingReplies`/`RecordingEvents`: order A (decorated sent + logged), RAW_FIRST (raw
  sent, reply-out decorated + logged: nothing logged for typed replies), Z (no decoration, `command=None`); `reject_undecodable`
  per reply kind; ACK suppression vs warnings in `RecordingReplies`.
- `test_imagedata.py`: `ni_image_type` per dtype; width/height.

**WP2** — `tests/golden/spec_vectors.py` + `tests/wire/`
- `spec_vectors.py` (data only, imported read-only by WP4/WP7 tests): every worked hex example — 02 §7.5 (b) frame, 02 §8.6
  Get Heartbeat reply (57 bytes), the 9 rows of the 02 §8.6 `Response (Flattened)` table, 02 §5.4 heartbeat frame, 04 §3.2 path
  vectors (empty, `C:\f.png`, Not-A-Path), the 18 rows of 04 §5.7. Templates keep `{tt}`/`{cc}` placeholders; `fill(template, **codes)`
  substitutes codes taken from the loaded enum table (tests must not hard-code provisional codes).
- `test_lv_flatten.py`: every kind round-trips; cp1252 notifier bytes kept opaque (incl. 0x81); any non-zero bool byte reads TRUE;
  `dbl_2d` keeps 0×0 vs 0×2; every truncation of every vector raises `ProtocolError`; out-of-range integer writes raise.
- `test_lv_path.py`: 04 §3.2 vectors; Windows/UNC/relative round trips; `common_to_windows("/D/alox/settings")` →
  `D:\alox\settings`; `localize_path_text` POSIX rules with `ALOX_DRIVE_D`.
- `test_lv_variant.py`: bare-DBL synthetic header = 02 §7.5 (b) bytes; parse of synthetic cluster/enum/array headers; TD size < 4 and
  bit-15 sizes rejected; bad top index rejected.
- `test_wire_enums.py`: packaged TOML loads, all tables provisional, 46 command labels; routing: 24 camera-range names + `Log` →
  camera, 19 → Z, `… Run` unrouted; override-file and inline precedence; every validation error of §3.2.4; the 03 §3.5 synthetic order
  (Z Do Reference 8th) makes `routing_self_check` report exactly the 11 commands `Z Get Position` … `Z Cleanup`.
- `test_messages.py`: `COMMANDS` has 46 entries matching 03 §2 (worker, reply kind) and 04 §5.1 (data mode); `CASES` excludes the two
  Run commands; all 18 vectors of 04 §5.7 decode/encode byte-exactly; Camera Init Data both orders + the residual ambiguity cases of
  04 §7.1; a `[wire.layout]` override changes the decode; out-of-range `Camera Type` → `ProtocolError`; lenient `z_reference_method`
  code 7 → `"#7"`; `ENUM size = 1` override shifts payloads as expected; `payload_from_mapping` conversions.
- `test_protocol.py`: decode of the 02 §7.5 (b) frame (command `Z Set Min Limit`, 0.5, `"Z#17"`, ts 1000/1001/0) and byte-exact
  re-encode; the 57-byte Get Heartbeat reply via `response_frame`; the 9 `Response (Flattened)` rows via `flatten_reply`;
  `parse_header` (N < 2, too large); attribute count ≠ 0 rejected; `parse_reply_value` inverse of `flatten_reply` for every kind.

**WP3** — `tests/config/`
- `test_niconfig.py`: 09 §6.1–6.2 rules (case-insensitive sections/keys, first occurrence wins, `;` comment, quoted value with `;`,
  `#` line with `=` is a key, CR/LF/CRLF, decimal comma, `12abc` → 12, `0x10` → 0, `3.7` → 3 for I32, bool `0/1/true/FALSE`, invalid
  → `ConfigTypeInvalid`, string `\\` and `\0A` unescape, path not unescaped, BOM tolerated, `read_list`).
- `test_folders.py`: missing `alox_setup.ini` → defaults + warning and file **not** created; empty value → default + warning;
  `--settings-dir` override; `$ALOX_EXE_DIR`.
- `test_alox_ini.py`: `len(ALOX_INI_KEYS) == 53`; the 09 §5.6 reference file → `AloxSettings()`; per-key fallback with warning (no
  chain effect, D4); baud index table and `115200` → 9600 + warning; `usb` variants → USB (D6); `setLEDcam_I32 ≠ 0` → LM plug type
  and `"LM"`; HM `frameratemax_DBL` ignored; missing file not created.
- `test_server_ini.py`: `SERVER_INI_TEMPLATE` parses to `ServerConfig()`; each of the 15 compat keys overrides its field;
  `[wire.items]`, `[wire.size]`, `[wire.layout]`, `[stage] port_map` parsing; unknown keys warned.
- `test_eventlog.py`: golden lines of 09 §7.4 with a fixed datetime (`29/10/2026 - 16:31:05.123 - ERR (Z STAGE)-(9405) : Z move out of
  range (Move Absolute)` and the `INFO -(0) :  (Move Absolute: Z Reference resetted)` line); file names/folders per 09 §7.3; only
  status TRUE logged; CRLF + cp1252; append mode vs offset-0 mode (reproduces the 09 §7.3 example); info label fix/reproduce; decimal
  separator option; dirs created; unwritable folder swallowed + one diagnostic warning; concurrent writers keep whole lines.

**WP4** — `tests/server/`
- `test_link.py`: N threads sending large frames concurrently → a reader sees only whole frames; stale `conn_id` dropped; failed
  send shuts the socket; `close()` drops later sends.
- `test_reply.py`: ACK suppression by `send_error_replies`; warnings sent; typed replies always sent with the error; return values
  (ACK → err, typed → NO_ERROR); with `FixedClock` the Get Heartbeat reply equals the 57-byte golden vector (value 123456, sending
  12345678); image reply copies the 3 request timestamps + pre/post; codec error → IMAGE `b""` + error.
- `test_router.py`: camera route sets `receiving_ts`, Z route keeps the client value; unknown code → no reply, recorded in history;
  `reply_unknown_commands` → ACK 9506; bad envelope dropped; payload decode error → `decode_error` 91; Run commands never routed.
- `test_worker.py`: FIFO; `REQUEUE` goes to the tail and later commands run in between (05 §6.2); `Exit` stops and discards later
  items; handler exception → 9500 finished + loop continues; `decode_error` → `reject_undecodable`; background stepper called when
  idle.
- `test_tcp.py`: header split across segments; idle client kept; mid-frame timeout closes; N < 2 closes; heartbeat frame ignored;
  heartbeat with payload closes (switch TRUE) / ignored (FALSE); Response/Image frames ignored; second client served after the first
  disconnects; `connected` flag; `bind()` on a busy port raises.
- `test_runtime.py`: with fake handler tables: start/`Exit` chain (camera stops → Z receives `Exit` → link closed → hardware closed →
  `wait()` True); `request_shutdown` path; replies after shutdown dropped.
- `test_app.py`: argument parsing; `--check-config` on a temp tree exits 0 and prints the routing table; no `--simulate` → exit 2;
  `--write-settings-template` refuses to overwrite; `--ready-file` content.

**WP5** — `tests/zstage/`
- `test_sim_stage.py` (fake clock): motion timing and `is_moving`; latched errors 5/7/10/17/51/1008 read once by `qerr`; FNL/FRF
  referencing and `#7`; macros; not connected → −4; `inject_failure`.
- `test_z_controller.py`: set-up sequence on SimStage (session filled, FNL when unreferenced, VEL at INI speed) and on failure (session
  still written, error translated); no-axis texts (100); `vel`/`jon` read `ERR?`; `general_wait` timeout −7 then additional wait;
  `check_axis_reference` both branches; `translate` appends `"\nGCS Error: <text>"`.
- `test_z_handlers.py` (one test per command, `RecordingReplies`/`RecordingEvents`): reply kind/value/error and log label for all 19;
  `restart_dh_macro` and `implicit_z_reinit` both values; 9405 inclusive bounds, reversed limits, NaN; info line on re-init; `Z Stop`
  then `Z Get Status` → BOOL FALSE + 10 "Controller error"; `Z Cleanup` error ACK (suppressed by default) vs success with the switch
  off; `Z Set Min Limit` NaN → error 1; `Exit` → `STOP`.

**WP6** — `tests/camera/`, `tests/imaging/`
- `test_png.py`: write/read round trip gray 8/16; hand-made RGB/RGBA files (zlib) → arrays; each filter type 0–4; unsupported formats.
- `test_codec.py`: provisional header bytes golden for a 2×3 U8 image with 2 custom keys; round trip U8/U16; `NiVisionCodec.flatten`
  → 9508; `make_codec` names.
- `test_sim_camera.py` (`manual=True`): numbering from 0; one-shot stops after 5; trigger mode; ring overwrite + `get_buffer(0)` →
  newest after overwrite; attributes, max, wrong types, injected failures.
- `test_pipeline.py`: flip semantics (H = top↔bottom); flat-field formula on a 2×2 example; `to_u8_like_simulation` (16-bit `>> 8`,
  RGB luminance, −9011); `simulate_next_image` timer/index logic (timer 0 cycles, timer > 0 repeats, empty folder → error);
  `lv_timestamp_text` → `"HH:MM:SS:.mmm"`; custom-data keys/values (`type` `High-Mag`/`Low-Mag`).
- `test_acquisition.py`: the 05 §6.2 transition table; 9009 exact text; `request_buffer_zero` TRUE returns buffer 0 during the first 5
  frames, FALSE the newest; `strict_new_frame`; `error_in` → `Error`; independent `t0` per camera.
- `test_camera_handlers.py`: every camera command (reply kind, suffix, extra, state updates) incl.: Set Exposure clamping (177,
  2 000 000), HM MaxDBL vs LM `fr_calc` and `exposure_sets_max_framerate`, replaced source on error; Set Gain I64 rounding half-even
  (2.5 → 2); Get Acquisition Status / Cam Stop before set-up → FALSE/OK; Software Trigger always OK; Set Led truth table (05 §9.2) and
  wait only when required; Get Led XOR; Stream Next Image: `REQUEUE` then IMAGE with custom data; temperature failure with both switch
  values; Set FlatField with both `flatfield_check_on_set` values; AF/RF stubs → 9507 order A + focus fields; Get AF Data 0×0 / progress
  0; `Exit` forwards an internal `Exit`.

**WP7** — `tests/tools/`, `tests/integration/`
- `test_replay_client.py`: script parsing (vars, inline tables, `await`, `poll`, `raw_hex`, `data_hex`); expectation evaluation;
  record/compare masking (tick, image); against a fake server on a socketpair.
- Integration tests: §5.

### 4.3 Golden vectors summary

| Source | Vector | Tested by |
|---|---|---|
| 02 §7.5 (b) | full `Z Set Min Limit` command frame (52-byte N) | WP2 `test_protocol`, `test_lv_variant` |
| 02 §8.6 | 57-byte `Get Heartbeat` reply frame | WP2 `test_protocol`, WP4 `test_reply` |
| 02 §8.6 table | 9 `Response (Flattened)` encodings | WP2 `test_protocol` |
| 02 §5.4 | heartbeat frame `00 00 00 02 tt tt` | WP2 `test_protocol` |
| 04 §3.2 | 3 path encodings | WP2 `test_lv_path` |
| 04 §5.7 | 18 payload data encodings | WP2 `test_messages` |
| 05 §6.3 | 9009 source text | WP6 `test_acquisition` |
| 09 §5.6 | reference `alox.ini` → defaults | WP3 `test_alox_ini` |
| 09 §7.3–§7.4 | log lines and the offset-0 overwrite example | WP3 `test_eventlog` |
| 03 §3.5 | synthetic order → 11 unreachable Z commands | WP2 `test_wire_enums` |

---

## 5. Integration test plan (WP7)

### 5.1 Fixture (`tests/integration/conftest.py`)

1. `tmp_path` tree: `exe/alox_setup.ini` (`[folders] settings/vision/data` = POSIX paths inside `tmp_path`), `settings/alox.ini`
   (09 §5.6 defaults; `zmin_DBL = 0`, `zmax_DBL = 20`), `settings/alox_camera_server.ini` (`[simulation] enabled = TRUE,
   frame_rate = 50, stage_reference_s = 0.05`; compat defaults or the variant's overrides).
2. `sim/` with 3 PNG files of different sizes (written with `imaging.png.write_png`), `ffc.png` / `dfc.png` (640 × 480 U8).
3. In-process server: `loaded = app.load_configuration(app.parse_args(["--setup-ini", …, "--simulate", "--port", "0"]))`,
   `runtime = app.build_runtime(loaded, simulate=True, port=0)`, `host, port = runtime.start()`; teardown sends nothing and calls
   `runtime.request_shutdown()` + `runtime.wait(10)` if still running.
4. Scripts run through `replay_client.run_script(load_script(path, extra_vars={"sim_dir": …, "ffc": …, "dfc": …}), host=…, port=…)`.

### 5.2 `test_end_to_end.py::test_all_commands_default_compat` — `tools/scripts/phase1_all_commands.toml`

A coverage assertion first: the set of `command` values in the script equals `set(COMMANDS)` (46 names). Steps (expectations in
brackets; "none" = no reply within `no_reply_wait_s`):

1. Heartbeat frame without payload [none, connection kept]; `Get Heartbeat` [U32, no error].
2. `Show UI {show=true}`, `Log {…}` [ACK OK].
3. HM before set-up: `Cam Get Acquisition Status` [BOOL FALSE, OK]; `Cam Stop` [ACK OK].
4. `Cam Setup` HM `cam0` and LM `cam1`, continuous [ACK OK].
5. `Cam Set Exposure` HM 1500 → `Cam Get Exposure` [1500.0]; LM 100 → [177.0]; `Cam Set Gain` HM 2.5 → `Cam Get Gain` [2.0];
   `Cam Flip` LM {F, T} [ACK].
6. `Cam Start` HM and LM {triggered=false} [ACK]; `Cam Get Acquisition Status` HM [TRUE].
7. `Cam Stream Next Image` HM camera mode [IMAGE 640 × 480 U8, keys = `CUSTOM_DATA_KEYS`, `type = "High-Mag"`, `flatfield = "FALSE"`].
8. `Cam Stream Next Image` LM simulation, path `${sim_dir}`, timer 0 [IMAGE with the first PNG's size, no custom data].
9. `Cam Get Image` HM [none]; `Cam Get Images Array` [none]; `Cam Software Trigger` HM [ACK OK].
10. `Cam Set FlatField` HM {enable, `${ffc}`, `${dfc}`, 1.0} [ACK OK] → `Cam Stream Next Image` HM [`flatfield = "TRUE"`].
11. `Cam Set Led` HM TRUE [ACK OK] → `Cam Get Led` HM [TRUE].
12. `Cam Autofocus` HM, `Cam Refocus` HM [none: error ACK 9507 suppressed]; `Cam Get Autofocus Data` [DBL2D shape 0×0];
    `Cam Get Autofocus Progress` [0.0]; `Cam Autofocus Run`, `Cam Refocus Run` [none: not routed].
13. `Z Get Serial {type=false}` before set-up [STR "", OK]; `Z Setup` [ACK OK]; `Z Get Serial {type=true}` [STR = sim identity];
    `Z Get Status` [TRUE]; `Z Get Error` [STR ""].
14. `Z Set Speed 50` [ACK] → `Z Get Speed` [50.0] (fast moves keep the test short); `Z Get Reference` [TRUE].
15. `Z Move Absolute 12.5` [ACK] → `Z Wait End Move {0, 10, F}` [ACK] → `Z Get Position` [12.5 ± 1e-6]; `Z Move Relative -2.5` →
    wait → [10.0]; `Z Move Absolute 30` [none: 9405 suppressed]; `Z Set Max Limit 24` → `Z Move Absolute 22` → wait → [22.0] →
    `Z Set Max Limit 20` [ACK].
16. `Z Enable Joystick {1,1}` [ACK] → `Z Joystick Enabled ?` [TRUE] → `Z Disable Joystick` [ACK] → [FALSE].
17. `Z Stop` [ACK] → `Z Get Status` [BOOL FALSE, error 10 `"Controller error"`] (08 Z6); `Z Do Reference {method="FNL"}` [ACK OK];
    `Z Set Min Limit 0.5` [ACK].
18. `Z Cleanup` [none: RMC? on the closed link fails, error ACK suppressed, 08 Z9].
19. `Cam Stop` HM [ACK]; `Cam Cleanup` HM, LM [ACK].
20. `code = 32767` (unknown command) [none].
21. `Exit` [none; connection closed by the server within `shutdown_timeout_s`].

Post-conditions checked by the test: `runtime.wait(10)` is True; `<vision>/logs/<yymmdd>_z_events.log` contains the 9405 line of
09 §7.4 and the `Cleanup` error; `<vision>/logs/<yymmdd>_camera_events.log` contains `ERR (HI MAG)-Cam Autofocus(9507)`; the sim
devices and SimStage are closed; `state.commands_history[0] == "Exit"` and the unknown code appears as `"#32767"`.

### 5.3 `test_end_to_end.py::test_error_replies` — `tools/scripts/phase1_error_replies.toml`

`[compat] send_error_replies = TRUE`, `reply_unknown_commands = TRUE`: `Z Move Absolute 30` → ACK `{TRUE, 9405, "Z move out of
range"}`; `Cam Autofocus` → ACK `{TRUE, 9507, source_contains "Cam Autofocus"}`; unknown code → ACK 9506; `Cam Set Exposure` with
`data_hex = "0001"` → ACK `{TRUE, 91, source_contains "Variant To Data"}`; `Cam Stream Next Image` on a cleaned-up camera → IMAGE
`b""` with error −1074360316 (typed replies are always sent); `Z Cleanup` → ACK error −4; `Exit`.

### 5.4 Other integration tests

- `test_end_to_end.py::test_reconnect`: disconnect mid-session, reconnect, `Get Heartbeat` works; a reply produced for the first
  connection is not delivered to the second (02 D10, using a slow `Z Wait End Move`).
- `test_end_to_end.py::test_pipelined`: 20 commands sent with `wait = false` (camera + Z interleaved, plus a stream request) → every
  reply arrives exactly once, matched by notifier; no deadlock (01 §9.1).
- `test_cli_smoke.py`: subprocess `python -m aloxcam --setup-ini … --simulate --port 0 --ready-file …`, then
  `tools/scripts/smoke.toml` via `python tools/replay_client.py --port <p>` (exit 0), then the process exits with code 0 within 10 s
  of `Exit`. A second run sends SIGINT instead of `Exit` and expects exit code 0.

---

## 6. Spec ambiguities exposed as configuration

| # | Ambiguity (spec) | Setting | Default chosen |
|---|---|---|---|
| 1 | Order, codes and representation of every wire enum (Q2, 04 §6, 10 R1) | `data/wire_enums.toml`; `[wire] enums_file`, `[wire.items]`, `[wire.size]` | provisional tables of §3.2.4 (Get Heartbeat = 0, camera range first, `Log`, Z range ending with `Z Do Reference`, Run items last; all U16) |
| 2 | Cluster order / integer widths of payloads: `Camera Init Data` (04 §7.1), AF/RF (A4/A5), Z wait/joystick (A6), `Subsampling` (A7) | `[wire.layout] <typedef label> = attr:kind, …`; `[wire] init_data_fallback` | `LV_LAYOUT` of §3.2.5 (Unbundle order for Init Data with fallback ON; panel order elsewhere; integers I32) |
| 3 | String encoding (02 C12) | `[wire] encoding`, `[logging] encoding` | `cp1252` |
| 4 | Tick Count epoch (01-Q5, Q32 e) | `[server] tick_source` | `monotonic` |
| 5 | Image serialisation (02 §8.5, Q6) | `[image] codec`, `nivision_library` | `provisional` |
| 6 | HM `Max Framerate` constant never loaded (05 §3.1, 09 D5) | `[camera] hm_max_framerate` | 0.0 |
| 7 | Saved default of `BulbStatusInverter` (05 §9.1, Q41.3) | `[camera] bulb_status_inverter` | FALSE |
| 8 | Hidden PI set-up options `Check FRF?` / reference move (08 §4.3, Q11) | `[stage] always_reference`, `reference_move` | FALSE (reference only when not referenced), `FNL` |
| 9 | Start-up macro check purpose (01 §3.4.3, A17, Q41.2) | `[stage] startup_macro_check` | FALSE (skip) |
| 10 | 300 s motion/ready timeouts (08 §8, Z19) | `[stage] wait_timeout_s`, `ready_timeout_s` | 300 |
| 11 | End-of-reference detection `#5` vs `#7` (08 Z13) | `[stage] do_reference_waits_ready` | TRUE (`#7`, then `#5`) |
| 12 | `%3u` decimal separator of the production locale (09 §7.4, 05 §7.5) | `[logging] decimal_separator` | `.` |
| 13 | Event-log defects possibly relied on by other tools (09 D10/D11, Q18/Q38) | `[logging] append_errors`, `fix_info_labels` | TRUE, TRUE (fixed) |
| 14 | Frame size limit / socket timeouts (02 D6, D12) | `[server] max_payload_mb`, `io_timeout_s` | 256 MiB, 5 s |
| 15 | Mapping of `D:\…` defaults off Windows (09 open question 7) | environment `ALOX_DRIVE_<LETTER>`; `--settings-dir` | unmapped → folder unavailable (warning) |
| 16 | Client-observable LabVIEW quirks (10 §6) | `[compat]` (15 switches) | LabVIEW behaviour, as listed in 10 §6 |

---

## 7. Decisions fixed directly in Phase 1 (no switch)

Internal defects and out-of-contract input (10 §6 rules), as recommended by the cited items: no TCP queue, unbounded worker queues and
re-enqueue on the internal path (01 A1–A3, 02 D11); reads back to back, no 50 ms gap or 2 s listen (A4, D8, D9); always back to
listening, `connected` reset (A5, A6, D3, D4); undecodable envelopes dropped, undecodable payloads answered with error 91 and never
executed (A20, D7, 03 C4, 04 T1/T6/T8); mid-frame timeout and bad `N` close the connection (D5, D6); write failure closes the
connection, stale replies dropped (A21, D10, D12); internal Run commands never routed (03 C16); bounded, time-stamped histories, internal
steps not recorded (A11–A13); one locked `ServerState` and one `ZController` (A16, 08 Z18); hardware closed on `Exit`/signal (A10, 05
#10); folders resolved once, INI files never created, per-key INI fallback with warnings (09 D2–D4, D9); event-log fixes D10, D11, D13,
D15 (switchable for byte comparison, §6 row 13); per-camera acquisition state (05 #3); camera re-open closes the old session (05 #9);
`Cam Start` uses the camera's own plug type (05 #12); `Cam Stop` sets the acquisition state (05 #11); `Z Do Reference` waits with
`#7` (08 Z13, switchable §6 row 11). Kept as LabVIEW (client-visible): log label `"Get Z Referenced"` for `Z Do Reference` (08 Z15,
logs may be parsed, Q38); `Z Move …` neither waits nor reads `ERR?` (03 C13); `Z Set Speed` leaves the settings unchanged (03 C14).

---

## Appendix A — `<settings>/alox_camera_server.ini` template (`config.server_ini.SERVER_INI_TEMPLATE`)

```ini
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
send_error_replies = FALSE
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
init_data_fallback = TRUE

[wire.items]
; camera_command = Get Heartbeat, Show UI, Exit, ...

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
always_reference = FALSE
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
stage_identity = "(c) aloxcam SimStage, C-863.11-SIM, 0000000001, 0.0.0"
stage_dh_macro_s = 0
```

`[server] port` empty = `alox.ini [hardware.camera] port_I32`. `[stage] port_map` format: `COM2=/dev/ttyUSB0, COM3=/dev/ttyS2`.

## Appendix B — Phase 1 exit criteria mapping (10 §5)

| Exit criterion | Covered by |
|---|---|
| Every request in the capture decodes | `decode_envelope` + `PayloadCodec` on the capture (WP2 tests extend `spec_vectors.py` when the capture arrives; enum table filled via `[wire.items]` or a new packaged TOML) |
| Deterministic replies byte-identical except tick fields | `replay_client --record` against LabVIEW, `--compare … --mask tick` against Python |
| The client connects and runs with simulated devices | `python -m aloxcam --simulate` with the real enum table; `--check-config` shows a clean routing self-check |
