"""Fixtures of the Z stage tests: a fake clock shared by ``SimStage`` and ``ZController``, and a ready-made Z context.

With :class:`FakeTime` every simulated wait (``#7`` / ``#5`` polling, additional waits, reference moves) runs instantly
and deterministically: ``sleep`` only advances the clock that ``SimStage`` reads.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import Any

import pytest

from aloxcam.settings import CompatSwitches, SimulationOptions, StageOptions
from aloxcam.testing import FixedClock, RecordingEvents, RecordingReplies, make_state
from aloxcam.zstage.controller import ZController
from aloxcam.zstage.handlers import ZContext
from aloxcam.zstage.sim import SimStage


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
    """A Z worker context on a ``SimStage`` driven by a :class:`FakeTime`."""

    ctx: ZContext
    sim: SimStage
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


@pytest.fixture
def make_sim(fake_time: FakeTime) -> Callable[..., SimStage]:
    """``make_sim(dh_running=True, **SimulationOptions overrides)``."""
    def factory(*, dh_running: bool = True, **overrides: Any) -> SimStage:
        return SimStage(SimulationOptions(**overrides), clock=fake_time.monotonic, dh_running=dh_running)
    return factory


@pytest.fixture
def make_controller(fake_time: FakeTime) -> Callable[..., ZController]:
    """``make_controller(backend, **StageOptions overrides)``."""
    def factory(backend: SimStage, **overrides: Any) -> ZController:
        return ZController(backend, StageOptions(**overrides), sleep=fake_time.sleep, monotonic=fake_time.monotonic)
    return factory


@pytest.fixture
def make_env(fake_time: FakeTime, make_sim: Callable[..., SimStage]) -> Callable[..., ZEnv]:
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
