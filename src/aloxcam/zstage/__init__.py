"""The Z stage: GCS-level backend interface (``backend``), simulated PI C-663 (``sim``), error texts (``gcs_errors``),
the locked controller link (``controller``) and the 19 Z command handlers (``handlers``). Phase 2: the raw GCS serial
transport (``gcs_serial``), the real C-663 backend (``pi_serial``) and an emulated C-663 on a TCP port for tests and
rehearsal (``fake_c663``).

Spec: 08 (Z stage), 03 §7 (Z commands); docs/answers/owner-answers.md §4–§5 (C-663, ``DH`` / ``HOME`` macros);
docs/phase2-design.md.
"""

from __future__ import annotations
