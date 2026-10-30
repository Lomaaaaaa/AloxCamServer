"""Minimal PI GCS error text table (the codes the simulated controller and the server can produce).

Spec: 08 §3.5 (driver errors), §11 (``GCSTranslateError`` and the codes the Z subsystem relays); design §3.5.2.

The texts are PI's (``pipython.pidevice.gcserror``). LabVIEW's ``GCSTranslateError.vi`` holds a 404-line table and
pipython ships the same one; this module keeps only the codes the Z subsystem can produce or relay (08 §11) and is not
grown into a copy of PI's table. Used by :class:`aloxcam.zstage.sim.SimStage` and by the phase 2 backend
:class:`aloxcam.zstage.pi_serial.PiSerialStage`, which does not use ``pipython`` (docs/phase2-design.md §2): a code
missing here is reported without the ``"GCS Error: …"`` text.
"""

from __future__ import annotations

from collections.abc import Mapping
from types import MappingProxyType

__all__ = ["GCS_ERROR_TEXT", "gcs_error_text"]

#: PI error descriptions by code: controller codes (> 0), interface codes (< 0) and 100 (LabVIEW driver).
GCS_ERROR_TEXT: Mapping[int, str] = MappingProxyType({
    0: "No error",
    1: "Parameter syntax error",
    2: "Unknown command",
    5: "Unallowable move attempted on unreferenced axis, or move attempted with servo off",
    7: "Position out of limits",
    10: "Controller was stopped by command",
    15: "Invalid axis identifier",
    17: "Parameter out of range",
    20: "Macro not found",
    51: "Selected axis is controlled by joystick",
    1008: "Controller is (already) running a macro",
    -2: "Error while sending data",
    -3: "Error while receiving data",
    -4: "Not connected (no port with given ID open)",
    -6: "Error while opening port",
    -7: "Timeout error",
    100: "PI LabVIEW driver reports error. See source control for details.",
})


def gcs_error_text(code: int) -> str:
    """The description ``GCSTranslateError`` would append for ``code``; ``""`` when the code is unknown (08 §11)."""
    return GCS_ERROR_TEXT.get(int(code), "")
