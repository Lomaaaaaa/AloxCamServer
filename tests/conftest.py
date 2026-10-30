"""Shared pytest fixtures for every package (design §1: WP1).

- ``fixed_clock``: an :class:`aloxcam.testing.FixedClock` starting at 0.
- ``tmp_alox_tree``: a temporary installation (``exe/alox_setup.ini`` → ``settings/alox.ini`` with the 09 §5.6
  defaults, ``vision/``, ``data/``) as an :class:`aloxcam.testing.AloxTree`.
- ``_fast_socket_close`` (autouse, phase 2): pyserial's ``socket://`` transport sleeps 0.3 s in ``close()`` "in case of
  quick reconnects"; the tests close and re-open links to the fake C-663 hundreds of times, so that pause is removed.
  Real serial ports never had it.
"""

from __future__ import annotations

import types
from pathlib import Path

import pytest

from aloxcam.testing import AloxTree, FixedClock, write_alox_tree


@pytest.fixture
def fixed_clock() -> FixedClock:
    return FixedClock()


@pytest.fixture
def tmp_alox_tree(tmp_path: Path) -> AloxTree:
    return write_alox_tree(tmp_path / "alox")


@pytest.fixture(autouse=True)
def _fast_socket_close(monkeypatch: pytest.MonkeyPatch) -> None:
    try:
        from serial.urlhandler import protocol_socket
    except ImportError:                     # pyserial missing: nothing to speed up
        return
    monkeypatch.setattr(protocol_socket, "time", types.SimpleNamespace(sleep=lambda _seconds: None))
