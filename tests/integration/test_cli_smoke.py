"""CLI smoke tests: ``python -m aloxcam`` in a subprocess, driven by ``python tools/replay_client.py``.

Spec: docs/phase1-design.md §5.4 (CLI smoke test with ``Exit`` and with SIGINT), §3.4.7 (``--ready-file``, exit codes,
``--check-config``), §3.7.2 (replay client CLI and exit codes), Appendix B (the client connects and runs with
simulated devices; a clean routing self-check).
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

import pytest

from integration.conftest import REAL_ALOX_INI, REPO, SCRIPTS, Installation, make_installation

pytestmark = pytest.mark.integration

START_TIMEOUT_S = 30.0          # interpreter + numpy start-up on a loaded machine
EXIT_TIMEOUT_S = 10.0           # design §5.4: the process exits within 10 s of Exit


def _env() -> dict[str, str]:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(REPO / "src"), env.get("PYTHONPATH", "")]).rstrip(os.pathsep)
    return env


@dataclass
class ServerProcess:
    process: subprocess.Popen[bytes]
    install: Installation
    host: str
    port: int
    log: Path

    def output(self) -> str:
        return self.log.read_text(encoding="utf-8", errors="replace")


def _wait_ready(process: subprocess.Popen[bytes], ready: Path, log: Path) -> tuple[str, int]:
    """The ready file is the only signal across processes: poll it (the server writes it atomically)."""
    deadline = time.monotonic() + START_TIMEOUT_S
    while not ready.exists():
        if process.poll() is not None:
            raise AssertionError(f"server exited with {process.returncode}:\n{log.read_text(errors='replace')}")
        if time.monotonic() > deadline:
            raise AssertionError(f"no ready file after {START_TIMEOUT_S} s:\n{log.read_text(errors='replace')}")
        time.sleep(0.02)
    host, port = ready.read_text(encoding="ascii").split()
    return host, int(port)


@pytest.fixture
def server_process(tmp_path: Path) -> Iterator[ServerProcess]:
    """``python -m aloxcam --setup-ini … --simulate --port 0 --ready-file …`` on the owner's production alox.ini."""
    install = make_installation(tmp_path / "alox", alox_ini=REAL_ALOX_INI.read_bytes())
    ready = tmp_path / "ready.txt"
    log = tmp_path / "server.log"
    with open(log, "wb") as out:
        process = subprocess.Popen(
            [sys.executable, "-m", "aloxcam", "--setup-ini", str(install.tree.setup_ini), "--simulate", "--port", "0",
             "--ready-file", str(ready)], stdout=out, stderr=subprocess.STDOUT, env=_env(), cwd=tmp_path)
    try:
        host, port = _wait_ready(process, ready, log)
        yield ServerProcess(process=process, install=install, host=host, port=port, log=log)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(EXIT_TIMEOUT_S)


def test_smoke_script_then_exit(server_process: ServerProcess) -> None:
    result = subprocess.run(
        [sys.executable, str(REPO / "tools" / "replay_client.py"), str(SCRIPTS / "smoke.toml"),
         "--host", server_process.host, "--port", str(server_process.port)],
        capture_output=True, text=True, env=_env(), timeout=60, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "3 step(s) run: 3 passed, 0 failed" in result.stdout
    assert server_process.process.wait(EXIT_TIMEOUT_S) == 0, server_process.output()
    assert "server stopped" in server_process.output()


@pytest.mark.skipif(sys.platform == "win32", reason="POSIX signal delivery")
def test_sigint_stops_the_server(server_process: ServerProcess) -> None:
    server_process.process.send_signal(signal.SIGINT)
    assert server_process.process.wait(EXIT_TIMEOUT_S) == 0, server_process.output()
    assert "shutdown requested (signal SIGINT)" in server_process.output()


def test_check_config_production_alox_ini(tmp_path: Path) -> None:
    install = make_installation(tmp_path / "alox", alox_ini=REAL_ALOX_INI.read_bytes())
    result = subprocess.run(
        [sys.executable, "-m", "aloxcam", "--setup-ini", str(install.tree.setup_ini), "--check-config",
         "--no-stderr-log"], capture_output=True, text=True, env=_env(), timeout=60, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    out = result.stdout
    assert "routing self-check: OK" in out and "result: OK" in out
    assert "127.0.0.1:503  (address from [compat] listen_address, port from alox.ini port_I32)" in out
    assert "  cameras     simulated" in out and "  Z stage     simulated" in out

