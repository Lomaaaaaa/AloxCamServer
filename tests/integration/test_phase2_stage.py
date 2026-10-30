"""Phase 2 end to end: the server with simulated cameras and the real Z stage backend (``PiSerialStage``) talking to a
``FakeC663Server`` through ``[stage] port_map``.

Spec: docs/phase2-design.md §3.4 (settings, runtime, CLI), §4.2 (``phase2_z_stage.toml``), §5 (runtime and CLI tests);
08 §3.2–§3.4 (frames), Z6 (error 10 after ``STP``), Z9 (``Z Cleanup``).
"""

from __future__ import annotations

import logging
import subprocess
import sys
import time
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

import pytest

from aloxcam import app
from aloxcam.errors import ConfigError
from aloxcam.server.runtime import ServerRuntime
from aloxcam.settings import SimulationOptions
from aloxcam.zstage.fake_c663 import FakeC663Server
from integration.conftest import REPO, SCRIPTS, Installation, make_installation
from integration.test_cli_smoke import EXIT_TIMEOUT_S, _env, _wait_ready
from replay_client import RunReport, Script, load_script, run_script

pytestmark = pytest.mark.integration

PHASE2_SCRIPT = SCRIPTS / "phase2_z_stage.toml"
#: Inside the reference alox.ini range [0, 20] and the fake's travel [0, 25]; fast so that the test is short.
SCRIPT_VARS = {"z_abs": "12.5", "z_rel": "-2.5", "z_rel_end": "10.0", "z_min": "0.0", "z_max": "20.0",
               "slow_speed": "50.0"}
STOP_TIMEOUT_S = 10.0

SERVER_INI = """\
; alox_camera_server.ini of the phase 2 tests: real Z stage on a FakeC663Server
[server]
shutdown_timeout_s = 10
[stage]
port_map = COM2={url}
[simulation]
{simulation}
frame_rate = 50
"""


def failures(report: RunReport) -> str:
    return "\n".join(r.line() for r in report.failures)


def wait_until(condition: object, timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():  # type: ignore[operator]
            return True
        time.sleep(0.01)
    return False


@dataclass
class Rig:
    """A FakeC663Server and an installation whose ``COM2`` (reference alox.ini ``comport_I32 = 2``) maps to it."""

    fake: FakeC663Server
    install: Installation

    def loaded(self, *argv: str) -> app.LoadedConfig:
        return app.load_configuration(app.parse_args(["--setup-ini", str(self.install.tree.setup_ini), *argv]))

    def script(self, **overrides: str) -> Script:
        return load_script(PHASE2_SCRIPT, extra_vars={**SCRIPT_VARS, **overrides})


@pytest.fixture
def rig(tmp_path: Path) -> Iterator[Rig]:
    fake = FakeC663Server(SimulationOptions(stage_reference_s=0.05)).start()
    install = make_installation(tmp_path / "alox")
    write_server_ini(install, fake.url, "cameras = TRUE")
    try:
        yield Rig(fake, install)
    finally:
        fake.close()


def write_server_ini(install: Installation, url: str, simulation: str) -> None:
    text = SERVER_INI.format(url=url, simulation=simulation)
    (install.tree.settings / "alox_camera_server.ini").write_bytes(text.replace("\n", "\r\n").encode("ascii"))


@pytest.fixture
def runtimes() -> Iterator[list[ServerRuntime]]:
    started: list[ServerRuntime] = []
    yield started
    for runtime in started:
        if runtime.started and not runtime.wait(0):
            runtime.request_shutdown("test teardown")
            runtime.wait(STOP_TIMEOUT_S)


def start(runtimes: list[ServerRuntime], loaded: app.LoadedConfig, **kwargs: bool) -> tuple[str, int, ServerRuntime]:
    runtime = app.build_runtime(loaded, port=0, **kwargs)
    host, port = runtime.start()
    runtimes.append(runtime)
    return host, port, runtime


# ---------------------------------------------------------------------------------------------- runtime + script
def test_phase2_script_against_the_fake_controller(rig: Rig, runtimes: list[ServerRuntime]) -> None:
    loaded = rig.loaded("--port", "0")
    assert loaded.server.simulation.simulate_cameras and not loaded.server.simulation.simulate_stage
    host, port, runtime = start(runtimes, loaded, simulate_cameras=True, simulate_stage=False)
    script = rig.script()
    assert {"Z Setup", "Z Get Serial", "Z Get Status", "Z Get Error", "Z Set Speed", "Z Get Speed",
            "Z Get Reference", "Z Get Position", "Z Move Absolute", "Z Move Relative", "Z Wait End Move", "Z Stop",
            "Z Enable Joystick", "Z Disable Joystick", "Z Joystick Enabled ?", "Z Set Min Limit", "Z Set Max Limit",
            "Z Cleanup", "Exit"} == script.commands

    report = run_script(script, host=host, port=port, enums=loaded.enums)

    assert report.ok, failures(report)
    assert runtime.wait(STOP_TIMEOUT_S), "the server did not stop after Exit"
    frames = rig.fake.controller.frames
    assert frames[:5] == ["XXX", "*idn?", "err?", "*IDN?", "SAI?"]           # the purge of 08 §3.2, then set-up
    for frame in ("FNL 1", "VEL 1 10.0000", "VEL 1 50.0000", "MOV 1 12.5000", "MVR 1 -2.5000", "STP 1",
                  "MAC START DH", "JAX 1 1 1", "JON 1 1", "JON 1 0", "JON? 1", "#5", "#7"):
        assert frame in frames, frame
    assert "MOV 1 1000.0000" not in frames                                    # 9405: refused by the server
    assert wait_until(lambda: not rig.fake.connected)                         # Z Cleanup closed the port
    z_log = rig.install.event_log("z")
    assert "- ERR (Z STAGE)-(9405) : Z move out of range (Move Absolute)\r\n" in z_log
    assert "- ERR (Z STAGE)-(10) : Controller error (Get Status)\r\n" in z_log
    assert "- ERR (Z STAGE)-(-4) : RMC?: not connected (Cleanup)\r\n" in z_log


def test_exit_closes_the_open_link(rig: Rig, runtimes: list[ServerRuntime], tmp_path: Path) -> None:
    loaded = rig.loaded("--port", "0")
    host, port, runtime = start(runtimes, loaded, simulate_cameras=True, simulate_stage=False)
    script = tmp_path / "setup_exit.toml"
    script.write_text("""
[session]
timeout_s = 10.0
[[step]]
command = "Z Setup"
expect = { reply = "ACK", error = { status = false, code = 0 } }
[[step]]
command = "Z Get Position"
expect = { reply = "DBL", value = 0.0, error = { status = false, code = 0 } }
[[step]]
command = "Exit"
expect = { reply = "none", connection = "closed" }
""")
    report = run_script(load_script(script), host=host, port=port, enums=loaded.enums)
    assert report.ok, failures(report)
    assert rig.fake.connections == 1
    assert runtime.wait(STOP_TIMEOUT_S)
    assert wait_until(lambda: not rig.fake.connected)                         # close_hardware → ZController.close


def test_simulate_still_uses_the_simulated_stage(rig: Rig, runtimes: list[ServerRuntime], tmp_path: Path) -> None:
    loaded = rig.loaded("--simulate", "--port", "0")
    host, port, runtime = start(runtimes, loaded, simulate=True)              # phase 1 call, port_map ignored
    script = tmp_path / "setup_exit.toml"
    script.write_text("""
[[step]]
command = "Z Setup"
expect = { reply = "ACK", error = { status = false, code = 0 } }
[[step]]
command = "Exit"
expect = { reply = "none", connection = "closed" }
""")
    report = run_script(load_script(script), host=host, port=port, enums=loaded.enums)
    assert report.ok, failures(report)
    assert runtime.wait(STOP_TIMEOUT_S)
    assert rig.fake.connections == 0


@pytest.mark.slow
def test_controller_off_gives_a_timeout_ack(rig: Rig, runtimes: list[ServerRuntime], tmp_path: Path) -> None:
    rig.fake.controller.answer_delay_s = 30.0                                 # a controller that never answers
    loaded = rig.loaded("--port", "0")
    stage_options = loaded.server.stage
    host, port, runtime = start(runtimes, loaded, simulate_cameras=True, simulate_stage=False)
    script = tmp_path / "setup.toml"
    script.write_text("""
[session]
timeout_s = 15.0
[[step]]
command = "Z Setup"
expect.reply = "ACK"
expect.error = { status = true, code = -7, source_contains = "Data Receive Timeout (RS-232, socket://" }
""")
    t0 = time.monotonic()
    report = run_script(load_script(script), host=host, port=port, enums=loaded.enums)
    assert report.ok, failures(report)
    assert 4.5 < time.monotonic() - t0 < 10.0                                 # the 5 s VISA-like timeout of Z Setup
    assert stage_options.port_map["COM2"] == rig.fake.url
    assert rig.fake.controller.frames == ["XXX", "*idn?"]                     # stuck in the purge's *idn? read


@pytest.mark.parametrize(("kwargs", "message"), [
    ({"simulate_cameras": False, "simulate_stage": True}, "no hardware backend for the cameras"),
    ({"simulate": False}, "no hardware backend for the cameras"),
])
def test_build_runtime_refuses_real_cameras(rig: Rig, kwargs: dict[str, bool], message: str) -> None:
    with pytest.raises(ConfigError) as info:
        app.build_runtime(rig.loaded(), **kwargs)
    assert message in info.value.source


def test_build_runtime_without_pyserial(rig: Rig, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setitem(sys.modules, "serial", None)
    with pytest.raises(ConfigError) as info:
        app.build_runtime(rig.loaded(), simulate_cameras=True, simulate_stage=False)
    assert "pip install pyserial" in info.value.source and "aloxcam[stage]" in info.value.source
    app.build_runtime(rig.loaded(), simulate=True)                            # the simulated stage needs no pyserial


# ---------------------------------------------------------------------------------------------- CLI
def test_check_config_with_simulate_cameras(rig: Rig, capsys: pytest.CaptureFixture[str]) -> None:
    setup_ini = str(rig.install.tree.setup_ini)
    assert app.main(["--setup-ini", setup_ini, "--simulate-cameras", "--check-config", "--no-stderr-log"]) == 0
    out = capsys.readouterr().out
    assert "  cameras     simulated\n" in out
    assert (f"  Z stage     real PI C-663: COM2 -> {rig.fake.url} ([stage] port_map) at 115200 Bd, opened by Z Setup; "
            "pyserial ") in out and " importable\n" in out
    assert "result: OK" in out
    assert rig.fake.connections == 0                                          # --check-config opens nothing

    assert app.main(["--setup-ini", setup_ini, "--simulate", "--check-config", "--no-stderr-log"]) == 0
    assert "  Z stage     simulated\n" in capsys.readouterr().out


def test_check_config_without_pyserial(rig: Rig, capsys: pytest.CaptureFixture[str],
                                       monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setitem(sys.modules, "serial", None)
    assert app.main(["--setup-ini", str(rig.install.tree.setup_ini), "--check-config", "--no-stderr-log"]) == 0
    out = capsys.readouterr().out                                             # [simulation] cameras = TRUE: real stage
    assert "pyserial NOT importable (pip install pyserial): the server would exit with code 2" in out


def test_start_without_pyserial_exits_2(rig: Rig, monkeypatch: pytest.MonkeyPatch,
                                        caplog: pytest.LogCaptureFixture) -> None:
    monkeypatch.setitem(sys.modules, "serial", None)
    with caplog.at_level(logging.ERROR, logger="aloxcam.app"):
        code = app.main(["--setup-ini", str(rig.install.tree.setup_ini), "--simulate-cameras", "--port", "0",
                         "--no-stderr-log"])
    assert code == app.EXIT_NO_BACKEND == 2
    assert "the real Z stage needs pyserial: pyserial is not installed: pip install pyserial" in caplog.text


def test_real_cameras_exit_2(rig: Rig, caplog: pytest.LogCaptureFixture) -> None:
    write_server_ini(rig.install, rig.fake.url, "stage = TRUE")              # simulated stage, "real" cameras
    with caplog.at_level(logging.ERROR, logger="aloxcam.app"):
        code = app.main(["--setup-ini", str(rig.install.tree.setup_ini), "--port", "0", "--no-stderr-log"])
    assert code == 2
    assert "no hardware backend for the cameras before phase 3" in caplog.text


def test_simulate_and_simulate_cameras_exclude_each_other(capsys: pytest.CaptureFixture[str]) -> None:
    assert app.main(["--simulate", "--simulate-cameras"]) == 2
    assert "not allowed with argument" in capsys.readouterr().err


@pytest.mark.slow
def test_rehearsal_cli_with_the_replay_tool(rig: Rig, tmp_path: Path) -> None:
    """The rehearsal of README "Phase 2": ``python -m aloxcam --simulate-cameras`` in a subprocess, the fake on a TCP
    port through ``[stage] port_map``, then ``tools/replay_client.py phase2_z_stage.toml``."""
    write_server_ini(rig.install, rig.fake.url, "enabled = FALSE")
    ready = tmp_path / "ready.txt"
    log = tmp_path / "server.log"
    with open(log, "wb") as out:
        process = subprocess.Popen(
            [sys.executable, "-m", "aloxcam", "--setup-ini", str(rig.install.tree.setup_ini), "--simulate-cameras",
             "--port", "0", "--ready-file", str(ready)], stdout=out, stderr=subprocess.STDOUT, env=_env(),
            cwd=tmp_path)
    try:
        host, port = _wait_ready(process, ready, log)
        argv = [sys.executable, str(REPO / "tools" / "replay_client.py"), str(PHASE2_SCRIPT), "--host", host,
                "--port", str(port), "--record", str(tmp_path / "z.jsonl")]
        for name, value in SCRIPT_VARS.items():
            argv += ["--var", f"{name}={value}"]
        result = subprocess.run(argv, capture_output=True, text=True, env=_env(), timeout=120, check=False)
        assert result.returncode == 0, result.stdout + result.stderr
        assert "-- OK" in result.stdout
        assert process.wait(EXIT_TIMEOUT_S) == 0, log.read_text(errors="replace")
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(EXIT_TIMEOUT_S)
    assert "real Z stage (COM2 -> socket://" in log.read_text(errors="replace")
    assert (tmp_path / "z.jsonl").read_text().count("\n") >= 29
