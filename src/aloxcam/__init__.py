"""ALOX Camera Server: Python re-implementation of ``alox_camera_server.vi`` (LabVIEW 2018).

Spec: docs/10-migration-plan.md §5 (Phase 1, protocol skeleton); docs/phase1-design.md.
"""

from __future__ import annotations

__all__ = ["PHASE", "__version__"]

__version__ = "0.1.0"
PHASE = 1
