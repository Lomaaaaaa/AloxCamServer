"""``python -m aloxcam`` → :func:`aloxcam.app.main`.

Spec: 01 §3 (start-up), §7 (shutdown); docs/phase1-design.md §3.4.7.
"""

from __future__ import annotations

from aloxcam.app import main

if __name__ == "__main__":
    raise SystemExit(main())
