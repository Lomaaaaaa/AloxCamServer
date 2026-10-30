"""The Z stage: GCS-level backend interface (``backend``), simulated PI C-663 (``sim``), error texts (``gcs_errors``),
the locked controller link (``controller``) and the 19 Z command handlers (``handlers``).

Spec: 08 (Z stage), 03 §7 (Z commands); docs/answers/owner-answers.md §4–§5 (C-663, ``DH`` / ``HOME`` macros).
"""

from __future__ import annotations
