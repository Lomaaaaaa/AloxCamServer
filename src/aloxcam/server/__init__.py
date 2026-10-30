"""The server runtime: client link (``link``), replies (``reply``), routing (``router``), workers (``worker``), TCP
front end (``tcp``) and the assembled runtime with its shutdown chain (``runtime``).

Spec: 01 §5, §7, §10; 02 §3–§5, §8, §9; 03 §3; docs/phase1-design.md §2, §3.4.
"""

from __future__ import annotations
