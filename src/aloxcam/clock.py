"""U32 millisecond tick sources (LabVIEW ``Tick Count (ms)``) and wrap-around arithmetic.

Spec: 01 §1.2, §10.1; 02 §1.2; questionnaire Q32 (e).

Protocol timestamps always come from an injected :data:`TickFn`; code never calls ``time.monotonic()`` for them.
"""

from __future__ import annotations

import sys
import time
from collections.abc import Callable

from aloxcam.errors import Code, ConfigError

__all__ = ["U32_MASK", "TickFn", "make_tick_source", "monotonic_tick", "tick_add", "tick_diff", "windows_tick"]

U32_MASK = 0xFFFFFFFF

#: Returns a U32 millisecond tick (LabVIEW ``Tick Count``).
TickFn = Callable[[], int]

_get_tick_count: Callable[[], int] | None = None


def monotonic_tick() -> int:
    """``int(time.monotonic() * 1000) & U32_MASK`` (01 §10.1)."""
    return int(time.monotonic() * 1000) & U32_MASK


def windows_tick() -> int:
    """``GetTickCount`` (LabVIEW's epoch: milliseconds since boot) on Windows; :func:`monotonic_tick` elsewhere."""
    global _get_tick_count
    if sys.platform != "win32":
        return monotonic_tick()
    if _get_tick_count is None:
        import ctypes

        fn = ctypes.windll.kernel32.GetTickCount
        fn.restype = ctypes.c_uint32
        _get_tick_count = fn
    return int(_get_tick_count()) & U32_MASK


def make_tick_source(name: str) -> TickFn:
    """``"monotonic"`` or ``"windows"`` (``[server] tick_source``); :class:`ConfigError` otherwise."""
    sources: dict[str, TickFn] = {"monotonic": monotonic_tick, "windows": windows_tick}
    try:
        return sources[name.strip().lower()]
    except KeyError:
        raise ConfigError(Code.LV_INVALID_INPUT,
                          f"[server] tick_source: unknown tick source {name!r} (monotonic | windows)") from None


def tick_add(t: int, ms: int) -> int:
    """U32 wrap-around addition."""
    return (t + ms) & U32_MASK


def tick_diff(later: int, earlier: int) -> int:
    """``(later - earlier) & U32_MASK``: elapsed ms across one wrap-around."""
    return (later - earlier) & U32_MASK
