"""Shared pytest fixtures for every package (design §1: WP1).

- ``fixed_clock``: an :class:`aloxcam.testing.FixedClock` starting at 0.
- ``tmp_alox_tree``: a temporary installation (``exe/alox_setup.ini`` → ``settings/alox.ini`` with the 09 §5.6
  defaults, ``vision/``, ``data/``) as an :class:`aloxcam.testing.AloxTree`.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from aloxcam.testing import AloxTree, FixedClock, write_alox_tree


@pytest.fixture
def fixed_clock() -> FixedClock:
    return FixedClock()


@pytest.fixture
def tmp_alox_tree(tmp_path: Path) -> AloxTree:
    return write_alox_tree(tmp_path / "alox")
