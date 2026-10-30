"""Cameras: the ``CameraBackend`` driver interface, the simulated camera, the image pipeline, the acquisition state
machine, the per-camera sessions and the camera-worker command handlers.

Spec: 05 (camera acquisition), 03 §5–§6 (camera commands), 06 §2.5 / §3 (focus fields and command shells).
"""

from __future__ import annotations
