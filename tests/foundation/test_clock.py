"""aloxcam.clock: U32 tick arithmetic and tick sources (01 §10.1, design §3.1.2)."""

from __future__ import annotations

import pytest

from aloxcam.clock import U32_MASK, make_tick_source, monotonic_tick, tick_add, tick_diff, windows_tick
from aloxcam.errors import ConfigError
from aloxcam.testing import FixedClock


def test_tick_add_wraps() -> None:
    assert tick_add(10, 5) == 15
    assert tick_add(U32_MASK, 1) == 0
    assert tick_add(0xFFFFFFF0, 0x20) == 0x10
    assert tick_add(5, -10) == U32_MASK - 4


def test_tick_diff_wraps() -> None:
    assert tick_diff(15, 10) == 5
    assert tick_diff(0x10, 0xFFFFFFF0) == 0x20      # across the wrap
    assert tick_diff(10, 10) == 0


def test_tick_sources() -> None:
    assert make_tick_source("monotonic") is monotonic_tick
    assert make_tick_source("windows") is windows_tick
    assert make_tick_source(" Monotonic ") is monotonic_tick
    for source in (monotonic_tick, windows_tick):
        value = source()
        assert isinstance(value, int) and 0 <= value <= U32_MASK
    with pytest.raises(ConfigError):
        make_tick_source("lab")


def test_monotonic_tick_increases() -> None:
    a = monotonic_tick()
    b = monotonic_tick()
    assert tick_diff(b, a) < 1000


def test_fixed_clock(fixed_clock: FixedClock) -> None:
    assert fixed_clock() == 0
    fixed_clock.advance(25)
    assert fixed_clock() == 25
    fixed_clock.set(U32_MASK)
    fixed_clock.advance(2)
    assert fixed_clock() == 1
    assert FixedClock(start=-1)() == U32_MASK
