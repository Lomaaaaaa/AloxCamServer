"""Fixtures of the Z stage tests: a fake clock shared by the stage and ``ZController``, the stage backend under test and
a ready-made Z context.

With :class:`FakeTime` every simulated wait (``#7`` / ``#5`` polling, additional waits, reference moves) runs instantly
and deterministically: ``sleep`` only advances the clock that the controller model reads.

Phase 2 (docs/phase2-design.md §5): ``make_sim`` — and so ``make_env`` — is parametrized by ``stage_kind``:

- ``"sim"``: a :class:`SimStage` (phase 1);
- ``"fake"``: the real :class:`aloxcam.zstage.pi_serial.PiSerialStage` talking GCS over a loopback socket to a
  :class:`aloxcam.zstage.fake_c663.FakeC663Server` whose model runs on the same fake clock
  (:class:`fake_probe.FakeStageProbe`, which has the ``SimStage`` test hooks).

So the ``ZController`` and Z handler suites run through both paths. A test that only applies to ``SimStage`` carries
``@pytest.mark.sim_only("reason")``; a module that tests ``SimStage`` itself overrides ``stage_kind``.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from dataclasses import dataclass, replace
from typing import Any

import pytest

from aloxcam.settings import CompatSwitches, SimulationOptions, StageOptions
from aloxcam.testing import FixedClock, RecordingEvents, RecordingReplies, make_state
from aloxcam.zstage.backend import StageBackend
from aloxcam.zstage.controller import ZController
from aloxcam.zstage.fake_c663 import FakeC663Server
from aloxcam.zstage.handlers import ZContext
from aloxcam.zstage.sim import SimStage
from zstage.fake_probe import FakeStageProbe

STAGE_KINDS = ("sim", "fake")


class FakeTime:
    """``monotonic`` / ``sleep`` pair: ``sleep`` advances the clock and records the duration."""

    def __init__(self, start: float = 1000.0) -> None:
        self.now = start
        self.sleeps: list[float] = []
        self.on_sleep: Callable[[float], None] | None = None

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.sleeps.append(seconds)
        self.now += seconds
        if self.on_sleep is not None:
            self.on_sleep(seconds)

    def advance(self, seconds: float) -> None:
        self.now += seconds


@dataclass
class ZEnv:
    """A Z worker context on the stage under test (``SimStage`` or ``FakeStageProbe``) driven by a :class:`FakeTime`."""

    ctx: ZContext
    sim: Any                                 # SimStage | FakeStageProbe: the same test hooks
    time: FakeTime

    @property
    def stage(self) -> ZController:
        return self.ctx.stage

    @property
    def replies(self) -> RecordingReplies:
        assert isinstance(self.ctx.replies, RecordingReplies)
        return self.ctx.replies

    @property
    def events(self) -> RecordingEvents:
        assert isinstance(self.ctx.events, RecordingEvents)
        return self.ctx.events


@pytest.fixture
def fake_time() -> FakeTime:
    return FakeTime()


@pytest.fixture(params=STAGE_KINDS)
def stage_kind(request: pytest.FixtureRequest) -> str:
    """The backend ``make_sim`` builds: ``"sim"`` (SimStage) or ``"fake"`` (PiSerialStage + FakeC663Server)."""
    marker = request.node.get_closest_marker("sim_only")
    if request.param == "fake" and marker is not None:
        pytest.skip(f"SimStage only: {marker.args[0] if marker.args else 'see the test'}")
    return str(request.param)


@pytest.fixture
def make_sim(fake_time: FakeTime, stage_kind: str) -> Iterator[Callable[..., Any]]:
    """``make_sim(dh_running=True, **SimulationOptions overrides)`` → the stage backend under test."""
    probes: list[FakeStageProbe] = []

    def factory(*, dh_running: bool = True, **overrides: Any) -> StageBackend:
        options = SimulationOptions(**overrides)
        if stage_kind == "sim":
            return SimStage(options, clock=fake_time.monotonic, dh_running=dh_running)
        server = FakeC663Server(options, clock=fake_time.monotonic, dh_running=dh_running).start()
        probe = FakeStageProbe(server)
        probes.append(probe)
        return probe

    yield factory
    for probe in probes:
        probe.shutdown()


@pytest.fixture
def make_controller(fake_time: FakeTime) -> Callable[..., ZController]:
    """``make_controller(backend, **StageOptions overrides)``."""
    def factory(backend: StageBackend, **overrides: Any) -> ZController:
        return ZController(backend, StageOptions(**overrides), sleep=fake_time.sleep, monotonic=fake_time.monotonic)
    return factory


@pytest.fixture
def make_env(fake_time: FakeTime, make_sim: Callable[..., Any]) -> Callable[..., ZEnv]:
    """``make_env(compat=…, stage=StageOptions, sim=dict, send_error_replies=…)`` → :class:`ZEnv`."""
    def factory(*, compat: dict[str, Any] | None = None, stage: StageOptions | None = None,
                sim: dict[str, Any] | None = None, send_error_replies: bool = False) -> ZEnv:
        switches = replace(CompatSwitches(), send_error_replies=send_error_replies, **(compat or {}))
        options = stage or StageOptions()
        backend = make_sim(**(sim or {}))
        controller = ZController(backend, options, sleep=fake_time.sleep, monotonic=fake_time.monotonic)
        ctx = ZContext(state=make_state(), replies=RecordingReplies(send_error_replies=send_error_replies),
                       events=RecordingEvents(), compat=switches, tick=FixedClock(), stage=controller,
                       stage_options=options)
        return ZEnv(ctx, backend, fake_time)
    return factory
