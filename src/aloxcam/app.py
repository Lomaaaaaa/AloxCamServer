"""Command-line entry point: configuration loading order, assembly of the runtime, signals and exit codes.

Spec: 01 §3 (start-up stages 0–4), §7 (shutdown), §10.1 (mapping: ``main()``, cleanup, ``sys.exit``), 01 A10;
09 §3–§5 (folders, ``alox_setup.ini``, ``alox.ini``), §7.7 (diagnostic log); 10 §6 (``[compat]`` registry);
02 D17 (fail at start-up when the port is taken); docs/phase1-design.md §3.4.7, §4.1, Appendix A;
docs/phase2-design.md §3.4 (per-device simulation, the real Z stage, ``--simulate-cameras``).

Device choice (phase 2): ``--simulate`` simulates the cameras and the Z stage; ``--simulate-cameras`` simulates the
cameras and drives the real PI stage (``PiSerialStage``, pyserial); without either, ``[simulation] enabled`` /
``cameras`` / ``stage`` decide. Real cameras arrive in phase 3: without simulated cameras the server exits with code 2.

WP3 (configuration, event logs), WP5 (Z stage) and WP6 (cameras, imaging) are imported only inside the functions
that need them (design §4.1), so this module imports with WP1/WP2 alone.

Exit codes: 0 normal stop (``Exit`` or signal); 1 start-up failure (bind error, ``ConfigError``); 2 usage error / no
backend (real cameras, or the real stage without pyserial); 3 shutdown timeout. ``print`` is used only for
``--check-config`` and ``--version``.
"""

from __future__ import annotations

import argparse
import codecs
import dataclasses
import logging
import os
import signal
import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path
from types import FrameType
from typing import TYPE_CHECKING

from aloxcam import PHASE, __version__
from aloxcam.clock import TickFn, make_tick_source
from aloxcam.dispatch import Request, WorkerContext
from aloxcam.errors import Code, ConfigError
from aloxcam.settings import AloxSettings, CompatSwitches, Folders, ServerConfig, SimulationOptions
from aloxcam.state import ServerState
from aloxcam.wire.enums import WireEnums, build_routing, routing_self_check
from aloxcam.wire.messages import CASES, COMMANDS, PayloadCodec

if TYPE_CHECKING:
    from aloxcam.server.runtime import ServerRuntime

__all__ = [
    "ALOX_INI_NAME",
    "EXIT_NO_BACKEND",
    "EXIT_OK",
    "EXIT_SHUTDOWN_TIMEOUT",
    "EXIT_STARTUP_FAILURE",
    "EXIT_USAGE",
    "LOG_LEVELS",
    "NO_CAMERA_BACKEND",
    "LoadedConfig",
    "backend_problem",
    "build_runtime",
    "check_config",
    "load_configuration",
    "main",
    "parse_args",
    "simulation_modes",
]

log = logging.getLogger(__name__)

EXIT_OK = 0
EXIT_STARTUP_FAILURE = 1
EXIT_USAGE = 2
EXIT_NO_BACKEND = 2
EXIT_SHUTDOWN_TIMEOUT = 3

LOG_LEVELS = ("DEBUG", "INFO", "WARNING", "ERROR")
ALOX_INI_NAME = "alox.ini"
SETUP_INI_NAME = "alox_setup.ini"
NO_CAMERA_BACKEND = ("no hardware backend for the cameras before phase 3: start with --simulate-cameras (real Z stage) "
                     "or --simulate, or set [simulation] cameras = TRUE (or enabled = TRUE)")
_SHUTDOWN_GRACE_S = 3.0          # main gives up shutdown_timeout_s + 3 s after shutdown_started (design §3.4.7)
_WAIT_SLICE_S = 0.2


@dataclass(frozen=True)
class LoadedConfig:
    """Everything read at start-up, in the order of design §3.4.7; ``warnings`` are replayed into the diagnostic log
    once it exists."""

    folders: Folders
    server: ServerConfig
    alox: AloxSettings
    enums: WireEnums
    warnings: tuple[str, ...]


# ---------------------------------------------------------------------------------------------- arguments
def _port(text: str) -> int:
    try:
        value = int(text, 10)
    except ValueError:
        raise argparse.ArgumentTypeError(f"invalid port {text!r}") from None
    if not 0 <= value <= 0xFFFF:
        raise argparse.ArgumentTypeError(f"port {value} outside 0..65535")
    return value


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="aloxcam", description="ALOX Camera Server: Python re-implementation (phase 2: real PI Z stage, "
                                    "simulated cameras).")
    p.add_argument("--setup-ini", type=Path, metavar="PATH",
                   help=f"{SETUP_INI_NAME} to use (default: <executable folder>/{SETUP_INI_NAME})")
    p.add_argument("--settings-dir", type=Path, metavar="PATH",
                   help="settings folder (alox.ini, alox_camera_server.ini); overrides [folders] settings")
    devices = p.add_mutually_exclusive_group()
    devices.add_argument("--simulate", action="store_true",
                         help="use the simulated cameras and Z stage (same as [simulation] enabled = TRUE)")
    devices.add_argument("--simulate-cameras", action="store_true",
                         help="use the simulated cameras and the real PI Z stage (COM port from alox.ini "
                              "[hardware.z], [stage] port_map; needs pyserial)")
    p.add_argument("--port", type=_port, metavar="N",
                   help="TCP port; overrides [server] port and alox.ini port_I32 (0 = ephemeral)")
    p.add_argument("--listen", metavar="ADDR", help="listen address; overrides [compat] listen_address")
    p.add_argument("--log-level", type=str.upper, choices=LOG_LEVELS, help="diagnostic log level")
    p.add_argument("--no-stderr-log", action="store_true", help="no diagnostic log on stderr")
    p.add_argument("--ready-file", type=Path, metavar="PATH",
                   help='after binding, write "<host> <port>" to PATH (atomically)')
    p.add_argument("--check-config", action="store_true",
                   help="load and check the configuration, print it, and exit (0 = OK, 1 = fatal problems)")
    p.add_argument("--write-settings-template", type=Path, metavar="PATH",
                   help="write the alox_camera_server.ini template to PATH (never overwrites) and exit")
    p.add_argument("--version", action="store_true", help="print the version and exit")
    return p


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the command line (design §3.4.7); a usage error raises ``SystemExit(2)`` (argparse)."""
    return _build_parser().parse_args(argv)


# ---------------------------------------------------------------------------------------------- configuration
def load_configuration(args: argparse.Namespace) -> LoadedConfig:
    """Load the configuration in the order of design §3.4.7: ``resolve_folders`` → ``alox_camera_server.ini`` →
    ``alox.ini`` (with ``[camera]``) → wire enums. Tolerated problems are collected as warnings; fatal ones raise
    :class:`ConfigError`. Nothing is created or written (09 D2)."""
    from aloxcam.config.alox_ini import load_alox_ini
    from aloxcam.config.folders import executable_folder, resolve_folders
    from aloxcam.config.server_ini import SERVER_INI_NAME, load_server_config

    warnings: list[str] = []
    setup_ini = args.setup_ini if args.setup_ini is not None else executable_folder() / SETUP_INI_NAME
    folders = resolve_folders(setup_ini=setup_ini, settings_override=args.settings_dir, warn=warnings.append)
    server = load_server_config(folders.settings / SERVER_INI_NAME, warn=warnings.append)
    alox = load_alox_ini(folders.settings / ALOX_INI_NAME, server.camera, warn=warnings.append)
    enums = WireEnums.from_options(server.wire, folders.settings)
    return LoadedConfig(folders=folders, server=server, alox=alox, enums=enums, warnings=tuple(warnings))


def _effective_port(loaded: LoadedConfig, port: int | None) -> int:
    """``--port`` > ``[server] port`` > ``alox.ini [hardware.camera] port_I32`` (02 §3)."""
    if port is not None:
        return port
    if loaded.server.server.port is not None:
        return loaded.server.server.port
    return loaded.alox.port


def _payload_codec(loaded: LoadedConfig) -> PayloadCodec:
    wire = loaded.server.wire
    return PayloadCodec(loaded.enums, wire.layouts, init_data_fallback=wire.init_data_fallback,
                        encoding=wire.encoding)


# ---------------------------------------------------------------------------------------------- assembly
def simulation_modes(args: argparse.Namespace, simulation: SimulationOptions) -> tuple[bool, bool]:
    """``(simulate_cameras, simulate_stage)`` (docs/phase2-design.md §3.4): ``--simulate`` → both;
    ``--simulate-cameras`` → cameras only (the real stage, whatever ``[simulation]`` says); otherwise ``[simulation]``
    decides (``enabled or cameras``, ``enabled or stage``)."""
    if getattr(args, "simulate", False):
        return True, True
    if getattr(args, "simulate_cameras", False):
        return True, False
    return simulation.simulate_cameras, simulation.simulate_stage


def backend_problem(simulate_cameras: bool, simulate_stage: bool) -> str | None:
    """Why the server cannot start with this device choice, or None: real cameras (phase 3), or the real stage without
    pyserial."""
    if not simulate_cameras:
        return NO_CAMERA_BACKEND
    if not simulate_stage:
        from aloxcam.zstage.gcs_serial import PYSERIAL_MISSING, pyserial_version

        if pyserial_version() is None:
            return f"the real Z stage needs pyserial: {PYSERIAL_MISSING} (or start with --simulate)"
    return None


def build_runtime(loaded: LoadedConfig, *, simulate: bool = False, simulate_cameras: bool | None = None,
                  simulate_stage: bool | None = None, port: int | None = None, host: str | None = None,
                  tick: TickFn | None = None) -> ServerRuntime:
    """Assemble the server (design §3.4.7): event logs, image codec, backends, ``ZController``, camera sessions,
    payload codec, routing (+ self-check, ERROR log), ``ServerState``, handler tables, contexts, ``abort_waits`` and
    ``close_hardware``. Nothing is opened here (cameras and stage are opened by ``Cam Setup`` / ``Z Setup``).

    Devices (docs/phase2-design.md §3.4): ``simulate`` simulates both, as in phase 1; ``simulate_cameras`` /
    ``simulate_stage``, when given, override it per device. The real stage is ``ZController(PiSerialStage(),
    [stage])``: ``Z Setup`` opens ``alox.ini`` ``[hardware.z]`` ``COM<n>`` (through ``[stage] port_map``). Real cameras
    do not exist before phase 3, and the real stage needs pyserial: both raise :class:`ConfigError`."""
    simulate_cameras = simulate if simulate_cameras is None else simulate_cameras
    simulate_stage = simulate if simulate_stage is None else simulate_stage
    problem = backend_problem(simulate_cameras, simulate_stage)
    if problem is not None:
        raise ConfigError(Code.LV_INVALID_INPUT, problem)
    from aloxcam.camera.handlers import CAMERA_HANDLERS, CameraContext, close_cameras
    from aloxcam.camera.session import make_sessions
    from aloxcam.camera.sim import SimCameraBackend
    from aloxcam.eventlog import EventLogs
    from aloxcam.imaging.codec import make_codec
    from aloxcam.server.runtime import RuntimeParts, ServerRuntime
    from aloxcam.zstage.backend import StageBackend
    from aloxcam.zstage.controller import ZController
    from aloxcam.zstage.handlers import Z_HANDLERS, ZContext

    cfg = loaded.server
    tick = tick or make_tick_source(cfg.server.tick_source)
    host = host if host is not None else cfg.compat.listen_address
    port = _effective_port(loaded, port)

    events = EventLogs(loaded.folders, cfg.logging, recent_lines=loaded.alox.max_lines)
    for warning in events.ensure_dirs():    # honours [logging] create_log_dirs itself (09 D13)
        log.warning("event logs: %s", warning)
    codec = make_codec(cfg.image)
    payloads = _payload_codec(loaded)
    routing = build_routing(loaded.enums)
    for problem in routing_self_check(routing, CASES):
        log.error("routing self-check: %s", problem)
    state = ServerState.from_settings(loaded.alox, cfg.server)

    camera_backend = SimCameraBackend(cfg.simulation)
    stage_backend: StageBackend
    if simulate_stage:
        from aloxcam.zstage.sim import SimStage

        stage_backend = SimStage(cfg.simulation)
    else:
        from aloxcam.zstage.pi_serial import PiSerialStage

        stage_backend = PiSerialStage()
    log.info("devices: %s cameras, %s", "simulated" if simulate_cameras else "real",
             "simulated Z stage" if simulate_stage else f"real Z stage ({_stage_port_text(loaded)})")
    stage = ZController(stage_backend, cfg.stage)
    sessions = make_sessions(compat=cfg.compat, tick=tick, decimal_separator=cfg.logging.decimal_separator)
    try:
        stage.startup_macro_check()         # 01 §3.4.3: only if [stage] startup_macro_check and connected
    except Exception:
        log.exception("start-up PI macro check failed")

    def make_camera_context(base: WorkerContext, forward_to_z: Callable[[Request], None]) -> CameraContext:
        return CameraContext(**_base_fields(base), backend=camera_backend, sessions=sessions, forward_to_z=forward_to_z,
                             camera_options=cfg.camera, decimal_separator=cfg.logging.decimal_separator, stage=stage)

    def make_z_context(base: WorkerContext) -> ZContext:
        return ZContext(**_base_fields(base), stage=stage, stage_options=cfg.stage)

    def close_hardware() -> None:           # 01 A10 fix: close_cameras → CameraBackend.close → ZController.close
        for what, fn in (("close the cameras", lambda: close_cameras(sessions)),
                         ("release the camera driver", camera_backend.close), ("close the Z stage", stage.close)):
            try:
                fn()
            except Exception:
                log.exception("shutdown: cannot %s", what)

    parts = RuntimeParts(config=cfg, host=host, port=port, enums=loaded.enums, routing=routing, commands=COMMANDS,
                         payloads=payloads, state=state, events=events, codec=codec, tick=tick,
                         camera_handlers=CAMERA_HANDLERS, z_handlers=Z_HANDLERS,
                         make_camera_context=make_camera_context, make_z_context=make_z_context,
                         camera_background=None, abort_waits=stage.cancel_waits, close_hardware=close_hardware)
    return ServerRuntime(parts)


def _stage_port_text(loaded: LoadedConfig) -> str:
    """``"COM4 at 115200 Bd, …"`` or ``"COM4 -> socket://127.0.0.1:5555 ([stage] port_map) at 115200 Bd, …"``."""
    z = loaded.alox.z
    target = loaded.server.stage.port_map.get(z.port_name)
    port = z.port_name if target is None else f"{z.port_name} -> {target} ([stage] port_map)"
    return f"{port} at {z.baudrate} Bd, opened by Z Setup"


def _base_fields(base: WorkerContext) -> dict[str, object]:
    """The ``WorkerContext`` fields of ``base`` as keywords (to build a subclass context)."""
    return {name: getattr(base, name) for name in ("state", "replies", "events", "compat", "tick")}


# ---------------------------------------------------------------------------------------------- --check-config
def _bool_text(value: object) -> str:
    return ("TRUE" if value else "FALSE") if isinstance(value, bool) else str(value)


def check_config(loaded: LoadedConfig, args: argparse.Namespace) -> int:
    """``--check-config`` (design §3.4.7): print folders, effective address, warnings, enum tables, routing table,
    routing self-check and compat values; return 0, or 1 on fatal problems (bad payload layouts, unknown tick source
    or image codec, unknown encodings, port out of range). Routing self-check problems are reported, not fatal (the
    server starts with them and logs them at ERROR)."""
    from aloxcam.config.server_ini import SERVER_INI_NAME

    cfg = loaded.server
    fatal: list[str] = []
    out: list[str] = [f"aloxcam {__version__} (phase {PHASE}): configuration check"]

    f = loaded.folders
    out += ["", "folders"] + [f"  {name:<9} {'(unavailable)' if path is None else path}"
                              for name, path in (("exe", f.exe), ("settings", f.settings), ("vision", f.vision),
                                                 ("data", f.data))]
    server_ini = f.settings / SERVER_INI_NAME
    out += ["", "files",
            f"  {ALOX_INI_NAME:<24} {f.settings / ALOX_INI_NAME}",
            f"  {SERVER_INI_NAME:<24} {server_ini}{'' if server_ini.exists() else '  (missing: defaults)'}"]

    port = _effective_port(loaded, args.port)
    port_source = ("--port" if args.port is not None else
                   "[server] port" if cfg.server.port is not None else "alox.ini port_I32")
    host = args.listen if args.listen is not None else cfg.compat.listen_address
    host_source = "--listen" if args.listen is not None else "[compat] listen_address"
    simulate_cameras, simulate_stage = simulation_modes(args, cfg.simulation)
    if simulate_stage:
        stage_text = "simulated"
    else:
        from aloxcam.zstage.gcs_serial import pyserial_version

        version = pyserial_version()
        stage_text = (f"real PI C-663: {_stage_port_text(loaded)}; "
                      + (f"pyserial {version} importable" if version is not None
                         else "pyserial NOT importable (pip install pyserial): the server would exit with code 2"))
    out += ["", "server",
            f"  listen      {host}:{port}  (address from {host_source}, port from {port_source})",
            "  cameras     " + ("simulated" if simulate_cameras
                                else "real: no camera backend before phase 3, the server would exit with code 2"),
            f"  Z stage     {stage_text}",
            f"  tick source {cfg.server.tick_source}",
            f"  image codec {cfg.image.codec}"]
    if not 0 <= port <= 0xFFFF:
        fatal.append(f"port {port} outside 0..65535")

    out += ["", f"warnings ({len(loaded.warnings)})"] + [f"  - {w}" for w in loaded.warnings]

    out += ["", "enum tables"]
    for t in loaded.enums.tables.values():
        flags = "  PROVISIONAL" if t.provisional else ""
        flags += "  lenient" if t.lenient else ""
        out.append(f"  {t.enum_id:<20} {t.label!r:<22} U{8 * t.size:<3} {len(t.items):>3} items  "
                   f"source {t.source}{flags}")

    routing = build_routing(loaded.enums)
    out += ["", "routing table (camera_command)"]
    for code, name in sorted(routing.names.items()):
        route = routing.route(code)
        note = "" if route is not None else "  (not routed)"
        out.append(f"  {code:>5}  {name:<28} {route or '-'}{note}")
    problems = routing_self_check(routing, CASES)
    out += ["", f"routing self-check: {'OK' if not problems else f'{len(problems)} problem(s)'}"]
    out += [f"  - {p}" for p in problems]

    out += ["", "compat"]
    defaults = CompatSwitches()
    for field in dataclasses.fields(CompatSwitches):
        value = getattr(cfg.compat, field.name)
        note = "" if value == getattr(defaults, field.name) else "  (changed)"
        out.append(f"  {field.name:<30} = {_bool_text(value)}{note}")

    for what, check in (("payload layouts", lambda: _payload_codec(loaded)),
                        ("tick source", lambda: make_tick_source(cfg.server.tick_source)),
                        ("image codec", lambda: _make_codec(cfg)),
                        ("[wire] encoding", lambda: codecs.lookup(cfg.wire.encoding)),
                        ("[logging] encoding", lambda: codecs.lookup(cfg.logging.encoding))):
        try:
            check()
        except ConfigError as exc:
            fatal.append(f"{what}: {exc.source}")
        except LookupError as exc:
            fatal.append(f"{what}: {exc}")

    out += ["", f"fatal problems ({len(fatal)})"] + [f"  - {p}" for p in fatal]
    out += ["", f"result: {'OK' if not fatal else 'FAILED'}"]
    print("\n".join(out))
    return EXIT_OK if not fatal else EXIT_STARTUP_FAILURE


def _make_codec(cfg: ServerConfig) -> object:
    from aloxcam.imaging.codec import make_codec

    return make_codec(cfg.image)


# ---------------------------------------------------------------------------------------------- logging
class _LogScope:
    """The ``aloxcam`` logger set-up of one ``main()`` run: a stderr handler until the diagnostic log exists, then
    the handlers of ``eventlog.setup_diagnostic_logging``; everything is removed (and the level restored) at exit,
    so ``main()`` can run several times in one process (tests)."""

    def __init__(self, level: str | None, stderr: bool) -> None:
        self.logger = logging.getLogger("aloxcam")
        self._saved_level = self.logger.level
        self._handlers: list[logging.Handler] = []
        self.logger.setLevel(level or "INFO")
        if stderr:
            handler = logging.StreamHandler()
            handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s"))
            self.logger.addHandler(handler)
            self._handlers.append(handler)

    def use_diagnostic_log(self, loaded: LoadedConfig, args: argparse.Namespace) -> None:
        """Replace the start-up handler by the diagnostic log (09 §7.7)."""
        from aloxcam.eventlog import setup_diagnostic_logging

        self._remove_handlers()
        self._handlers = list(setup_diagnostic_logging(loaded.folders, loaded.server.logging, level=args.log_level,
                                                       stderr=not args.no_stderr_log))

    def _remove_handlers(self) -> None:
        for handler in self._handlers:
            self.logger.removeHandler(handler)
            try:
                handler.close()
            except Exception:
                pass
        self._handlers = []

    def close(self) -> None:
        self._remove_handlers()
        self.logger.setLevel(self._saved_level)


def _log_startup(loaded: LoadedConfig, args: argparse.Namespace) -> None:
    """Start-up log (design §3.4.7): version, folders, port, provisional enum tables, changed compat values."""
    cfg = loaded.server
    f = loaded.folders
    log.info("aloxcam %s (phase %d) starting, pid %d", __version__, PHASE, os.getpid())
    log.info("folders: exe %s, settings %s, vision %s, data %s", f.exe, f.settings, f.vision, f.data)
    log.info("configured address %s:%d", args.listen if args.listen is not None else cfg.compat.listen_address,
             _effective_port(loaded, args.port))
    for enum_id in loaded.enums.provisional_ids:
        log.warning("wire enum %s is provisional (order not confirmed)", enum_id)
    defaults = CompatSwitches()
    for field in dataclasses.fields(CompatSwitches):
        value = getattr(cfg.compat, field.name)
        if value != getattr(defaults, field.name):
            log.info("[compat] %s = %s (default %s)", field.name, _bool_text(value),
                     _bool_text(getattr(defaults, field.name)))


# ---------------------------------------------------------------------------------------------- run
def _write_ready_file(path: Path, host: str, port: int) -> None:
    """``"<host> <port>\\n"``, written atomically (temporary file + ``os.replace``)."""
    path = Path(path)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    tmp.write_bytes(f"{host} {port}\n".encode("ascii", errors="replace"))
    os.replace(tmp, path)


def _install_signal_handlers(runtime: ServerRuntime) -> Callable[[], None]:
    """SIGINT / SIGTERM (and SIGBREAK on Windows) → ``request_shutdown``; a second signal → ``os._exit(130)``.
    Returns a function restoring the previous handlers. Only possible in the main thread (else nothing is done)."""
    if threading.current_thread() is not threading.main_thread():
        log.debug("not in the main thread: no signal handlers installed")
        return lambda: None
    received = 0

    def handler(signum: int, frame: FrameType | None) -> None:
        nonlocal received
        received += 1
        if received > 1:
            os._exit(130)
        runtime.request_shutdown(f"signal {signal.Signals(signum).name}")

    previous: dict[int, object] = {}
    for name in ("SIGINT", "SIGTERM", "SIGBREAK"):
        signum = getattr(signal, name, None)
        if signum is None:
            continue
        try:
            previous[signum] = signal.signal(signum, handler)
        except (ValueError, OSError):
            log.debug("cannot install a handler for %s", name)

    def restore() -> None:
        for signum, old in previous.items():
            try:
                signal.signal(signum, old if old is not None else signal.SIG_DFL)  # type: ignore[arg-type]
            except (ValueError, OSError, TypeError):
                pass

    return restore


def _serve(runtime: ServerRuntime, args: argparse.Namespace, shutdown_timeout_s: float) -> int:
    """Start, write the ready file, then loop on ``runtime.wait(0.2)`` until stopped (design §3.4.7)."""
    restore_signals = _install_signal_handlers(runtime)
    try:
        try:
            host, port = runtime.start()
        except OSError as exc:
            log.error("cannot listen on %s:%d: %s", runtime.parts.host, runtime.parts.port, exc)
            return EXIT_STARTUP_FAILURE
        log.info("ALOX camera server listening on %s:%d", host, port)
        exit_code = EXIT_OK
        if args.ready_file is not None:
            try:
                _write_ready_file(args.ready_file, host, port)
            except OSError as exc:
                log.error("cannot write the ready file %s: %s; stopping", args.ready_file, exc)
                runtime.request_shutdown("ready file")
                exit_code = EXIT_STARTUP_FAILURE
        deadline: float | None = None
        while not runtime.wait(_WAIT_SLICE_S):
            if runtime.shutdown_started:
                now = time.monotonic()
                if deadline is None:
                    deadline = now + shutdown_timeout_s + _SHUTDOWN_GRACE_S
                elif now >= deadline:
                    log.error("shutdown did not complete within %.1f s: exiting", shutdown_timeout_s
                              + _SHUTDOWN_GRACE_S)
                    return EXIT_SHUTDOWN_TIMEOUT
        return exit_code
    finally:
        restore_signals()


def _write_settings_template(path: Path) -> int:
    from aloxcam.config.server_ini import write_template

    try:
        write_template(path)
    except FileExistsError:
        log.error("%s already exists: not overwritten", path)
        return EXIT_STARTUP_FAILURE
    except OSError as exc:
        log.error("cannot write %s: %s", path, exc)
        return EXIT_STARTUP_FAILURE
    log.info("settings template written to %s", path)
    return EXIT_OK


def _exit_code(exc: SystemExit) -> int:
    code = exc.code
    if code is None:
        return EXIT_OK
    return code if isinstance(code, int) else EXIT_USAGE


def main(argv: Sequence[str] | None = None) -> int:
    """``python -m aloxcam`` (design §3.4.7). Returns the exit code."""
    try:
        args = parse_args(argv)
    except SystemExit as exc:                    # usage error (2) or --help (0)
        return _exit_code(exc)
    if args.version:
        print(f"aloxcam {__version__} (phase {PHASE})")
        return EXIT_OK
    scope = _LogScope(args.log_level, stderr=not args.no_stderr_log)
    try:
        if args.write_settings_template is not None:
            return _write_settings_template(args.write_settings_template)
        try:
            loaded = load_configuration(args)
        except ConfigError as exc:
            log.error("configuration error: %s", exc.source)
            return EXIT_STARTUP_FAILURE
        except Exception:
            log.exception("cannot load the configuration")
            return EXIT_STARTUP_FAILURE
        if args.check_config:
            return check_config(loaded, args)
        scope.use_diagnostic_log(loaded, args)
        for warning in loaded.warnings:
            log.warning("configuration: %s", warning)
        _log_startup(loaded, args)
        simulate_cameras, simulate_stage = simulation_modes(args, loaded.server.simulation)
        problem = backend_problem(simulate_cameras, simulate_stage)
        if problem is not None:
            log.error("%s", problem)
            return EXIT_NO_BACKEND
        try:
            runtime = build_runtime(loaded, simulate=simulate_cameras and simulate_stage,
                                    simulate_cameras=simulate_cameras, simulate_stage=simulate_stage,
                                    port=args.port, host=args.listen)
        except ConfigError as exc:
            log.error("start-up failed: %s", exc.source)
            return EXIT_STARTUP_FAILURE
        except Exception:
            log.exception("start-up failed")
            return EXIT_STARTUP_FAILURE
        return _serve(runtime, args, loaded.server.server.shutdown_timeout_s)
    finally:
        scope.close()
