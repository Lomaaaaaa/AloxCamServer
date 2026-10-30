"""ALOX Camera Server: Python re-implementation of ``alox_camera_server.vi`` (LabVIEW 2018).

Spec: docs/10-migration-plan.md §5 (Phase 1, protocol skeleton; Phase 2, real PI Z stage); docs/phase1-design.md,
docs/phase2-design.md.
"""

from __future__ import annotations

__all__ = ["PHASE", "__version__"]

__version__ = "0.2.0"
PHASE = 2
