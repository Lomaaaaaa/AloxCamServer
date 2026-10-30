"""aloxcam.app: arguments, configuration loading order, --check-config, exit codes, ready file, signals
(design §3.4.7).

WP3 (config, eventlog) and WP6 (imaging.codec) are replaced by fake modules installed in ``sys.modules`` that follow
the design's interfaces, so these tests do not depend on their implementations; the tests marked ``real_wp3`` run
against the real modules once they exist (skipped otherwise).
"""

from __future__ import annotations

import logging
import os
import signal
import socket
import subprocess
import sys
import threading
import types
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import pytest

from aloxcam import __version__, app
from aloxcam.dispatch import ReplyKind
from aloxcam.errors import ConfigError
from aloxcam.server.runtime import ServerRuntime
from aloxcam.settings import (
    AloxSettings,
    CameraOptions,
    CompatSwitches,
    Folders,
    ImageOptions,
    LoggingOptions,
    ServerConfig,
    ServerOptions,
    SimulationOptions,
    WireOptions,
)
from aloxcam.testing import AloxTree
from server.wp4_helpers import ENUMS, TIMEOUT, Fakes, command_frame, make_parts, read_reply, wait_until

REPO = Path(__file__).resolve().parents[2]


# ---------------------------------------------------------------------------------------------- fake WP3 / WP6
class ListHandler(logging.Handler):
    def __init__(self) -> None:
        super().__init__(logging.DEBUG)
        self.messages: list[str] = []

    def emit(self, record: logging.LogRecord) -> None:
        self.messages.append(f"{record.levelname} {record.getMessage()}")


@dataclass
class FakeWP:
    """Records the calls made to the fake WP3 / WP6 modules."""

    root: Path
    server_config: ServerConfig = field(default_factory=ServerConfig)
    calls: list[tuple[str, Any]] = field(default_factory=list)
    diag: ListHandler = field(default_factory=ListHandler)

    @property
    def folders(self) -> Folders:
        return Folders(exe=self.root / "exe", settings=self.root / "settings", vision=self.root / "vision",
                       data=None)

    # aloxcam.config.folders
    def executable_folder(self, env: Any = None) -> Path:
        return self.root / "exe"

    def resolve_folders(self, *, setup_ini: Path | None = None, exe_dir: Path | None = None,
                        settings_override: Path | None = None, warn: Callable[[str], None]) -> Folders:
        self.calls.append(("resolve_folders", (setup_ini, settings_override)))
        warn("folders warning")
        folders = self.folders
        return replace(folders, settings=settings_override) if settings_override else folders

    # aloxcam.config.server_ini
    def load_server_config(self, path: Path, *, warn: Callable[[str], None]) -> ServerConfig:
        self.calls.append(("load_server_config", path))
        warn("server ini warning")
        return self.server_config

    @staticmethod
    def write_template(path: Path) -> None:
        if Path(path).exists():
            raise FileExistsError(path)
        Path(path).write_text("; template\n")

    # aloxcam.config.alox_ini
    def load_alox_ini(self, path: Path, camera_options: CameraOptions, *, warn: Callable[[str], None]) -> AloxSettings:
        self.calls.append(("load_alox_ini", (path, camera_options)))
        warn("alox ini warning")
        return AloxSettings(port=4242)

    # aloxcam.eventlog
    def setup_diagnostic_logging(self, folders: Folders | None, options: LoggingOptions, *, level: str | None = None,
                                 stderr: bool = True) -> list[logging.Handler]:
        self.calls.append(("setup_diagnostic_logging", (level, stderr)))
        logging.getLogger("aloxcam").addHandler(self.diag)
        return [self.diag]

    # aloxcam.imaging.codec
    def make_codec(self, options: ImageOptions) -> object:
        if options.codec not in ("provisional", "nivision"):
            raise ConfigError(1, f"[image] codec: unknown codec {options.codec!r}")
        return object()


@pytest.fixture
def wp(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> FakeWP:
    fake = FakeWP(tmp_path)

    def install(name: str, **attrs: Any) -> None:
        module = types.ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)

    install("aloxcam.config.folders", executable_folder=fake.executable_folder, resolve_folders=fake.resolve_folders)
    install("aloxcam.config.server_ini", SERVER_INI_NAME="alox_camera_server.ini",
            load_server_config=fake.load_server_config, write_template=fake.write_template)
    install("aloxcam.config.alox_ini", load_alox_ini=fake.load_alox_ini)
    install("aloxcam.eventlog", setup_diagnostic_logging=fake.setup_diagnostic_logging)
    install("aloxcam.imaging.codec", make_codec=fake.make_codec)
    return fake


def loaded_config(wp: FakeWP, **server_changes: Any) -> app.LoadedConfig:
    return app.LoadedConfig(folders=wp.folders, server=replace(ServerConfig(), **server_changes),
                            alox=AloxSettings(), enums=ENUMS, warnings=("w1", "w2"))


# ---------------------------------------------------------------------------------------------- arguments
def test_parse_args_defaults() -> None:
    args = app.parse_args([])
    assert (args.setup_ini, args.settings_dir, args.simulate, args.port, args.listen, args.log_level) == \
        (None, None, False, None, None, None)
    assert (args.no_stderr_log, args.ready_file, args.check_config, args.write_settings_template, args.version) == \
        (False, None, False, None, False)


def test_parse_args_all_options(tmp_path: Path) -> None:
    args = app.parse_args(["--setup-ini", "s.ini", "--settings-dir", "conf", "--simulate", "--port", "0",
                           "--listen", "0.0.0.0", "--log-level", "debug", "--no-stderr-log", "--ready-file", "r",
                           "--check-config", "--write-settings-template", "t.ini", "--version"])
    assert args.setup_ini == Path("s.ini") and args.settings_dir == Path("conf") and args.simulate
    assert args.port == 0 and args.listen == "0.0.0.0" and args.log_level == "DEBUG"
    assert args.no_stderr_log and args.ready_file == Path("r") and args.check_config
    assert args.write_settings_template == Path("t.ini") and args.version


@pytest.mark.parametrize("argv", [["--port", "70000"], ["--port", "x"], ["--log-level", "TRACE"], ["--bogus"]])
def test_usage_errors_exit_2(argv: list[str], capsys: pytest.CaptureFixture[str]) -> None:
    assert app.main(argv) == 2
    assert "usage" in capsys.readouterr().err


def test_version(capsys: pytest.CaptureFixture[str]) -> None:
    assert app.main(["--version"]) == 0
    assert capsys.readouterr().out.strip() == f"aloxcam {__version__} (phase 1)"


def test_python_dash_m_aloxcam_version() -> None:
    env = dict(os.environ, PYTHONPATH=str(REPO / "src"))
    result = subprocess.run([sys.executable, "-m", "aloxcam", "--version"], capture_output=True, text=True, env=env,
                            timeout=60, check=False)
    assert result.returncode == 0 and result.stdout.strip() == f"aloxcam {__version__} (phase 1)"


# ---------------------------------------------------------------------------------------------- configuration
def test_load_configuration_order_and_warnings(wp: FakeWP, tmp_path: Path) -> None:
    wp.server_config = replace(ServerConfig(), camera=CameraOptions(hm_max_framerate=12.0))
    loaded = app.load_configuration(app.parse_args(["--setup-ini", str(tmp_path / "setup.ini")]))
    settings = tmp_path / "settings"
    assert wp.calls == [
        ("resolve_folders", (tmp_path / "setup.ini", None)),
        ("load_server_config", settings / "alox_camera_server.ini"),
        ("load_alox_ini", (settings / "alox.ini", CameraOptions(hm_max_framerate=12.0))),
    ]
    assert loaded.warnings == ("folders warning", "server ini warning", "alox ini warning")
    assert loaded.alox.port == 4242 and loaded.server is wp.server_config and loaded.folders == wp.folders
    assert loaded.enums.table("camera_command").items == ENUMS.table("camera_command").items


def test_load_configuration_defaults_and_settings_dir(wp: FakeWP, tmp_path: Path) -> None:
    loaded = app.load_configuration(app.parse_args(["--settings-dir", str(tmp_path / "conf")]))
    assert wp.calls[0] == ("resolve_folders", (tmp_path / "exe" / "alox_setup.ini", tmp_path / "conf"))
    assert wp.calls[1] == ("load_server_config", tmp_path / "conf" / "alox_camera_server.ini")
    assert loaded.folders.settings == tmp_path / "conf"


def test_load_configuration_applies_wire_overrides(wp: FakeWP, tmp_path: Path) -> None:
    items = list(ENUMS.table("camera_type").items)
    wp.server_config = replace(ServerConfig(), wire=WireOptions(enum_items={"camera_type": tuple(reversed(items))}))
    loaded = app.load_configuration(app.parse_args([]))
    assert loaded.enums.table("camera_type").items == tuple(reversed(items))


def test_check_config_prints_the_routing_table(wp: FakeWP, monkeypatch: pytest.MonkeyPatch,
                                               capsys: pytest.CaptureFixture[str]) -> None:
    monkeypatch.setattr(app, "load_configuration",
                        lambda args: loaded_config(wp, compat=CompatSwitches(send_error_replies=True)))
    assert app.main(["--check-config", "--port", "1234", "--listen", "127.0.0.2"]) == 0
    out = capsys.readouterr().out
    assert "routing table (camera_command)" in out
    assert "      0  Get Heartbeat                camera" in out
    assert "     22  Cam Autofocus Run            -  (not routed)" in out
    assert "     44  Z Do Reference               z" in out
    assert "     45  Log                          camera" in out
    assert "routing self-check: OK" in out
    assert "127.0.0.2:1234  (address from --listen, port from --port)" in out
    assert "z_reference_method" in out and "PROVISIONAL" in out
    assert "send_error_replies             = TRUE  (changed)" in out
    assert "  - w1" in out and "result: OK" in out
    assert not any(name == "setup_diagnostic_logging" for name, _ in wp.calls)


def test_check_config_port_from_alox_ini_and_routing_problems(wp: FakeWP, monkeypatch: pytest.MonkeyPatch,
                                                               capsys: pytest.CaptureFixture[str]) -> None:
    items = list(ENUMS.table("camera_command").items)
    items.remove("Z Do Reference")
    items.insert(items.index("Z Setup") + 7, "Z Do Reference")             # the 03 §3.5 synthetic order
    enums = type(ENUMS).load(items={"camera_command": items})
    monkeypatch.setattr(app, "load_configuration", lambda args: replace(loaded_config(wp), enums=enums))
    assert app.main(["--check-config"]) == 0                               # reported, not fatal
    out = capsys.readouterr().out
    assert "127.0.0.1:503  (address from [compat] listen_address, port from alox.ini port_I32)" in out
    assert "routing self-check: 11 problem(s)" in out and "Z Cleanup: case unreachable" in out


@pytest.mark.parametrize("changes", [
    {"wire": WireOptions(layouts={"Camera Flip Data": "camera_type:enum:camera_type"})},
    {"image": ImageOptions(codec="jpeg")},
    {"server": ServerOptions(tick_source="sundial")},
    {"wire": WireOptions(encoding="no-such-encoding")},
])
def test_check_config_fatal_problems_exit_1(wp: FakeWP, monkeypatch: pytest.MonkeyPatch,
                                            capsys: pytest.CaptureFixture[str], changes: dict[str, Any]) -> None:
    monkeypatch.setattr(app, "load_configuration", lambda args: loaded_config(wp, **changes))
    assert app.main(["--check-config"]) == 1
    out = capsys.readouterr().out
    assert "fatal problems (1)" in out and "result: FAILED" in out


def test_configuration_error_exits_1(monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture) -> None:
    def broken(args: Any) -> app.LoadedConfig:
        raise ConfigError(1, "settings folder unmappable")

    monkeypatch.setattr(app, "load_configuration", broken)
    with caplog.at_level(logging.ERROR, logger="aloxcam.app"):
        assert app.main(["--check-config"]) == 1
        assert app.main(["--simulate"]) == 1
    assert "settings folder unmappable" in caplog.text


def test_write_settings_template_refuses_to_overwrite(wp: FakeWP, tmp_path: Path) -> None:
    target = tmp_path / "alox_camera_server.ini"
    assert app.main(["--write-settings-template", str(target)]) == 0
    assert target.read_text() == "; template\n"
    target.write_text("mine")
    assert app.main(["--write-settings-template", str(target)]) == 1
    assert target.read_text() == "mine"


def test_without_simulate_exits_2(wp: FakeWP, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(app, "load_configuration", lambda args: loaded_config(wp))

    def no_build(*args: Any, **kwargs: Any) -> ServerRuntime:
        raise AssertionError("build_runtime must not be called")

    monkeypatch.setattr(app, "build_runtime", no_build)
    assert app.main(["--log-level", "WARNING", "--no-stderr-log"]) == 2
    assert ("setup_diagnostic_logging", ("WARNING", False)) in wp.calls
    assert "WARNING configuration: w1" in wp.diag.messages                 # warnings replayed into the diag log
    assert any("no hardware backend" in m for m in wp.diag.messages)
    assert wp.diag not in logging.getLogger("aloxcam").handlers            # handlers removed at exit


def test_build_runtime_without_simulate_raises(wp: FakeWP) -> None:
    with pytest.raises(ConfigError):
        app.build_runtime(loaded_config(wp), simulate=False)


# ---------------------------------------------------------------------------------------------- running
@dataclass
class Served:
    """``main()`` run in the main thread (signal handlers), with ``client`` running in a helper thread."""

    exit_code: int
    error: BaseException | None


def serve(argv: list[str], client: Callable[[], None]) -> Served:
    errors: list[BaseException] = []

    def run() -> None:
        try:
            client()
        except BaseException as exc:      # reported to the test
            errors.append(exc)

    helper = threading.Thread(target=run, name="test-client", daemon=True)
    helper.start()
    code = app.main(argv)
    helper.join(TIMEOUT)
    return Served(code, errors[0] if errors else None)


@pytest.fixture
def fake_runtime(wp: FakeWP, monkeypatch: pytest.MonkeyPatch) -> Iterator[tuple[Fakes, list[ServerRuntime]]]:
    """``load_configuration`` → simulation enabled; ``build_runtime`` → a runtime with fake handler tables."""
    fakes = Fakes()
    built: list[ServerRuntime] = []
    monkeypatch.setattr(app, "load_configuration",
                        lambda args: loaded_config(wp, simulation=SimulationOptions(enabled=True),
                                                   server=ServerOptions(shutdown_timeout_s=0.3)))

    def build(loaded: app.LoadedConfig, *, simulate: bool, port: int | None = None, host: str | None = None,
              tick: Any = None) -> ServerRuntime:
        assert simulate
        parts = replace(make_parts(fakes), port=port if port is not None else 0, host=host or "127.0.0.1")
        runtime = ServerRuntime(parts)
        built.append(runtime)
        return runtime

    monkeypatch.setattr(app, "build_runtime", build)
    yield fakes, built
    fakes.release_wait.set()
    for runtime in built:
        if runtime.started and not runtime.wait(0):
            runtime.request_shutdown("test teardown")
            runtime.wait(TIMEOUT)


def _wait_ready(path: Path) -> tuple[str, int]:
    assert wait_until(lambda: path.exists(), TIMEOUT), "no ready file"
    host, port = path.read_text().split()
    return host, int(port)


def test_ready_file_and_client_exit(fake_runtime: tuple[Fakes, list[ServerRuntime]], tmp_path: Path) -> None:
    fakes, built = fake_runtime
    ready = tmp_path / "ready.txt"
    content: list[str] = []

    def client() -> None:
        host, port = _wait_ready(ready)
        content.append(ready.read_text())
        with socket.create_connection((host, port), timeout=TIMEOUT) as sock:
            sock.sendall(command_frame("Get Heartbeat", notifier=b"HB"))
            assert read_reply(sock, ReplyKind.U32)[0] == b"HB"
            sock.sendall(command_frame("Exit"))

    result = serve(["--simulate", "--port", "0", "--ready-file", str(ready)], client)
    assert result.error is None and result.exit_code == 0
    (runtime,) = built
    assert content == [f"127.0.0.1 {runtime.address[1]}\n"]
    assert not list(tmp_path.glob(".ready.txt.*"))              # the temporary file was renamed
    assert fakes.order == ["camera Exit", "Z Exit", "close_hardware"]


@pytest.mark.skipif(sys.platform == "win32", reason="POSIX signal delivery")
def test_sigint_stops_with_exit_code_0(fake_runtime: tuple[Fakes, list[ServerRuntime]], tmp_path: Path) -> None:
    fakes, _ = fake_runtime
    ready = tmp_path / "ready"
    previous = signal.getsignal(signal.SIGINT)

    def client() -> None:
        _wait_ready(ready)                                      # the handlers are installed before the bind
        os.kill(os.getpid(), signal.SIGINT)

    result = serve(["--ready-file", str(ready)], client)
    assert result.error is None and result.exit_code == 0
    assert fakes.order == ["camera Exit", "Z Exit", "close_hardware"]
    assert signal.getsignal(signal.SIGINT) is previous          # restored


def test_bind_failure_exits_1(fake_runtime: tuple[Fakes, list[ServerRuntime]]) -> None:
    with socket.create_server(("127.0.0.1", 0)) as busy:
        port = busy.getsockname()[1]
        assert app.main(["--port", str(port)]) == 1


def test_unwritable_ready_file_exits_1(fake_runtime: tuple[Fakes, list[ServerRuntime]], tmp_path: Path) -> None:
    _, built = fake_runtime
    assert app.main(["--ready-file", str(tmp_path / "missing" / "ready")]) == 1
    assert built[0].wait(0)                                     # stopped cleanly


def test_shutdown_timeout_exits_3(fake_runtime: tuple[Fakes, list[ServerRuntime]], tmp_path: Path,
                                  monkeypatch: pytest.MonkeyPatch) -> None:
    fakes, built = fake_runtime
    monkeypatch.setattr(app, "_SHUTDOWN_GRACE_S", 0.0)
    fakes.heartbeat_gate = threading.Event()                    # Get Heartbeat blocks in a "hardware call"
    ready = tmp_path / "ready"

    def client() -> None:
        host, port = _wait_ready(ready)
        with socket.create_connection((host, port), timeout=TIMEOUT) as sock:
            sock.sendall(command_frame("Get Heartbeat"))
            assert fakes.heartbeat_entered.wait(TIMEOUT)
            built[0].request_shutdown("test")                   # the camera worker never reaches the Exit

    try:
        result = serve(["--ready-file", str(ready)], client)    # shutdown_timeout_s + grace 0 s
    finally:
        fakes.heartbeat_gate.set()
    assert result.error is None and result.exit_code == 3


# ---------------------------------------------------------------------------------------------- real WP3
def _real_wp3() -> None:
    for name in ("aloxcam.config.folders", "aloxcam.config.server_ini", "aloxcam.config.alox_ini",
                 "aloxcam.imaging.codec"):
        pytest.importorskip(name, reason="WP3 / WP6 not merged yet")


def test_real_check_config_on_a_temp_tree(tmp_alox_tree: AloxTree, capsys: pytest.CaptureFixture[str]) -> None:
    _real_wp3()
    assert app.main(["--setup-ini", str(tmp_alox_tree.setup_ini), "--check-config", "--no-stderr-log"]) == 0
    out = capsys.readouterr().out
    assert "routing table (camera_command)" in out and "routing self-check: OK" in out
    assert f"settings  {tmp_alox_tree.settings}" in out
    assert not (tmp_alox_tree.settings / "alox_camera_server.ini").exists()   # nothing created (09 D2)


def test_real_write_settings_template(tmp_path: Path) -> None:
    _real_wp3()
    target = tmp_path / "alox_camera_server.ini"
    assert app.main(["--write-settings-template", str(target), "--no-stderr-log"]) == 0
    original = target.read_bytes()
    assert b"[compat]" in original
    assert app.main(["--write-settings-template", str(target), "--no-stderr-log"]) == 1
    assert target.read_bytes() == original
