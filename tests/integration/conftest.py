"""Fixtures of the integration tests: a temporary installation and an in-process server with simulated devices.

Spec: docs/phase1-design.md §5.1. The tree is ``exe/alox_setup.ini`` (POSIX folders inside ``tmp_path``),
``settings/alox.ini`` (the 09 §5.6 reference file, or the owner's production file) and
``settings/alox_camera_server.ini`` (``[simulation] enabled = TRUE, frame_rate = 50, stage_reference_s = 0.05`` and the
test's ``[compat]`` values); ``sim/`` holds 3 PNG files of different sizes, ``ffc.png`` / ``dfc.png`` are 640 × 480 U8
with F ≠ D. The server is built with ``app.load_configuration`` / ``app.build_runtime`` and listens on an ephemeral
loopback port; teardown sends nothing and calls ``request_shutdown()`` + ``wait(10)`` when it still runs.
"""

from __future__ import annotations

import dataclasses
from collections.abc import Callable, Iterator, Mapping
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pytest

import aloxcam.zstage.sim as zsim
from aloxcam import app
from aloxcam.client import AloxClient
from aloxcam.imaging.png import write_png
from aloxcam.server.runtime import ServerRuntime
from aloxcam.testing import REFERENCE_ALOX_INI, AloxTree, write_alox_tree
from aloxcam.wire.messages import PayloadCodec
from replay_client import RunReport, Script, load_script, run_script

REPO = Path(__file__).resolve().parents[2]
SCRIPTS = REPO / "tools" / "scripts"
#: The owner's production alox.ini (docs/answers/ini/alox.ini, copied by WP3).
REAL_ALOX_INI = REPO / "tests" / "config" / "data" / "real" / "alox.ini"
#: Simulation PNG files: name → (rows, cols); the first in name order is served first.
SIM_PNGS: Mapping[str, tuple[int, int]] = {"a_first.png": (240, 320), "b_second.png": (120, 160),
                                           "c_third.png": (48, 64)}
FRAME_SHAPE = (480, 640)
STOP_TIMEOUT_S = 10.0

SERVER_INI = """\
; alox_camera_server.ini of the integration tests
[server]
shutdown_timeout_s = 10
[compat]
{compat}
[simulation]
enabled = TRUE
frame_rate = 50
stage_reference_s = 0.05
"""


@dataclass(frozen=True)
class Installation:
    """The temporary installation and the PNG fixtures."""

    tree: AloxTree
    sim_dir: Path
    ffc: Path
    dfc: Path

    @property
    def vars(self) -> dict[str, str]:
        """``sim_dir``, ``ffc``, ``dfc`` for the replay scripts (design §5.1 step 4)."""
        return {"sim_dir": str(self.sim_dir), "ffc": str(self.ffc), "dfc": str(self.dfc)}

    def event_log(self, stem: str) -> str:
        """Every error event-log line of ``<vision>/logs/*_<stem>_events.log`` (``camera`` or ``z``), cp1252."""
        files = sorted((self.tree.vision / "logs").glob(f"*_{stem}_events.log"))
        return "".join(f.read_bytes().decode("cp1252") for f in files)


def make_installation(root: Path, *, compat: Mapping[str, str] = {}, alox_ini: str | bytes = REFERENCE_ALOX_INI
                      ) -> Installation:
    """Write the tree of design §5.1 under ``root``; ``alox_ini`` bytes are copied verbatim (a real file)."""
    tree = write_alox_tree(root, alox_ini=alox_ini if isinstance(alox_ini, str) else REFERENCE_ALOX_INI)
    if isinstance(alox_ini, bytes):
        tree.alox_ini.write_bytes(alox_ini)
    compat_text = "\n".join(f"{key} = {value}" for key, value in compat.items())
    (tree.settings / "alox_camera_server.ini").write_bytes(
        SERVER_INI.format(compat=compat_text).replace("\n", "\r\n").encode("ascii"))
    sim_dir = root / "sim"
    sim_dir.mkdir()
    for i, (name, shape) in enumerate(SIM_PNGS.items()):
        rows, cols = shape
        write_png(sim_dir / name, ((np.arange(rows * cols).reshape(shape) + 40 * i) % 256).astype(np.uint8))
    ffc, dfc = root / "ffc.png", root / "dfc.png"
    write_png(ffc, np.full(FRAME_SHAPE, 200, dtype=np.uint8))
    write_png(dfc, np.full(FRAME_SHAPE, 10, dtype=np.uint8))
    return Installation(tree=tree, sim_dir=sim_dir, ffc=ffc, dfc=dfc)


@dataclass
class LiveServer:
    """A running in-process server (design §5.1 step 3)."""

    runtime: ServerRuntime
    host: str
    port: int
    install: Installation
    loaded: app.LoadedConfig
    stages: list[zsim.SimStage] = field(default_factory=list)   # the SimStage built by build_runtime

    def client(self, timeout_s: float = 10.0) -> AloxClient:
        """A connected protocol client (close it, or use it as a context manager)."""
        loaded = self.loaded
        wire = loaded.server.wire
        payloads = PayloadCodec(loaded.enums, wire.layouts, init_data_fallback=wire.init_data_fallback,
                                encoding=wire.encoding)
        client = AloxClient(self.host, self.port, enums=loaded.enums, payloads=payloads, timeout_s=timeout_s)
        client.connect()
        return client

    def run(self, script: Path | Script, **extra_vars: str) -> RunReport:
        """Run a replay script against this server (design §5.1 step 4); the installation's variables are set."""
        if not isinstance(script, Script):
            script = load_script(script, extra_vars={**self.install.vars, **extra_vars})
        return run_script(script, host=self.host, port=self.port, enums=self.loaded.enums)

    def stop(self) -> bool:
        """``request_shutdown()`` then ``wait()``; True when stopped."""
        self.runtime.request_shutdown("test")
        return self.runtime.wait(STOP_TIMEOUT_S)


StartServer = Callable[..., LiveServer]


@pytest.fixture
def start_server(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[StartServer]:
    """``start_server(compat={...}, alox_ini=...)`` → a :class:`LiveServer` on 127.0.0.1, ephemeral port."""
    started: list[ServerRuntime] = []
    stages: list[zsim.SimStage] = []

    class RecordingSimStage(zsim.SimStage):
        """``SimStage`` that remembers its instances (``build_runtime`` creates it internally)."""

        def __init__(self, *args: object, **kwargs: object) -> None:
            super().__init__(*args, **kwargs)  # type: ignore[arg-type]
            stages.append(self)

    monkeypatch.setattr(zsim, "SimStage", RecordingSimStage)

    def start(*, compat: Mapping[str, str] = {}, alox_ini: str | bytes = REFERENCE_ALOX_INI,
              name: str = "alox") -> LiveServer:
        install = make_installation(tmp_path / name, compat=compat, alox_ini=alox_ini)
        loaded = app.load_configuration(app.parse_args(["--setup-ini", str(install.tree.setup_ini), "--simulate",
                                                        "--port", "0"]))
        runtime = app.build_runtime(loaded, simulate=True, port=0)
        host, port = runtime.start()
        started.append(runtime)
        return LiveServer(runtime=runtime, host=host, port=port, install=install, loaded=loaded,
                          stages=list(stages[-1:]))

    yield start
    for runtime in started:
        if runtime.started and not runtime.wait(0):
            runtime.request_shutdown("test teardown")
            runtime.wait(STOP_TIMEOUT_S)


@pytest.fixture
def server(start_server: StartServer) -> LiveServer:
    """A server with the 09 §5.6 reference alox.ini and the default ``[compat]`` values."""
    return start_server()


def replace_session(script: Script, **changes: object) -> Script:
    """``script`` with changed ``[session]`` options."""
    return dataclasses.replace(script, session=dataclasses.replace(script.session, **changes))
