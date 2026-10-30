"""Typed camera-attribute access, LabVIEW numeric coercion and the fixed attribute names and constants.

Spec: 05 §2.1 (attributes used by the server), §5.1 (value-type selection by a case-sensitive string), §5.2 / §5.5
(exposure limits, ``In Range and Coerce``), §1.2 (DBL → I64/U32 coercion), §9.1 (digital I/O enumeration values);
design §3.6.3.
"""

from __future__ import annotations

import math

from aloxcam.camera.backend import CameraDevice, ValueType
from aloxcam.errors import CameraError, Code

__all__ = [
    "BUFFER_COUNT_ATTRIBUTES",
    "DEVICE_TEMPERATURE",
    "DIO",
    "EXPOSURE_UPPER_LIMIT",
    "FRAMERATE_LOWER_LIMIT",
    "GET_IMAGE_TIMEOUT_MS",
    "GIGE_PACKET_SIZE",
    "LINE_INVERTER",
    "LINE_MODE",
    "LINE_SELECTOR",
    "LINE_SOURCE",
    "NUM_BUFFERS",
    "PROPERTY_NODE",
    "TRIGGER_MODE",
    "TRIGGER_SOFTWARE",
    "USER_OUTPUT_SELECTOR",
    "USER_OUTPUT_VALUE",
    "format_lv_double",
    "in_range_and_coerce",
    "lv_to_int",
    "read_typed",
    "value_kind",
    "write_typed",
]

TRIGGER_MODE = "CameraAttributes::AcquisitionControl::TriggerMode"
TRIGGER_SOFTWARE = "CameraAttributes::AcquisitionControl::TriggerSoftware"
DEVICE_TEMPERATURE = "CameraAttributes::DeviceControl::DeviceTemperature"
DIO = "CameraAttributes::DigitalIOControl::"   # + LineSelector, LineMode, LineSource, LineInverter, UserOutput…
LINE_SELECTOR = DIO + "LineSelector"
LINE_MODE = DIO + "LineMode"
LINE_SOURCE = DIO + "LineSource"
LINE_INVERTER = DIO + "LineInverter"
USER_OUTPUT_SELECTOR = DIO + "UserOutputSelector"
USER_OUTPUT_VALUE = DIO + "UserOutputValue"
#: ``operation`` text of IMAQdx property-node accesses in "invalid camera session" errors.
PROPERTY_NODE = "IMAQdx Property Node"
#: Session properties read by the state machine (05 §6.3), named in diagnostics only.
BUFFER_COUNT_ATTRIBUTES = ("LastBufferNumber", "BufferCount")

GIGE_PACKET_SIZE = 8996                  # Cam Setup, GigE plug type (05 §4.2)
NUM_BUFFERS = 5                          # ring size, constant (05 #8)
GET_IMAGE_TIMEOUT_MS = 5000              # Get Image2 timeout constant / default (05 §6.3, §6.5)
EXPOSURE_UPPER_LIMIT = 2_000_000.0       # Cam Set Exposure clamp (05 §5.2)
FRAMERATE_LOWER_LIMIT = 0.5              # fr_calc clamp (05 §5.2)

_I64_MIN, _I64_MAX = -(2**63), 2**63 - 1
_U32_MAX = 2**32 - 1


def value_kind(type_str: str) -> ValueType:
    """``"I64"`` / ``"U32"`` select ``ValueI64`` / ``ValueU32``; any other string (``"DBL"``, ``"dbl"``, ``""``)
    is the case structure's ``Default``: ``ValueDBL`` (05 §5.1, case-sensitive)."""
    if type_str == "I64":
        return ValueType.I64
    if type_str == "U32":
        return ValueType.U32
    return ValueType.DBL


def lv_to_int(x: float, vtype: ValueType) -> int:
    """LabVIEW DBL → I64 / U32 coercion: round half to even, then saturate (05 §1.2, inferred).

    ±Inf saturate; NaN raises ``CameraError(1, …)`` (the LabVIEW result is unknown; handlers reject non-finite
    inputs before getting here). Other value types return ``int(x)`` rounded the same way, unsaturated.
    """
    if math.isnan(x):
        raise CameraError(Code.LV_INVALID_INPUT, f"NaN cannot be coerced to {vtype.value}")
    lo, hi = (0, _U32_MAX) if vtype is ValueType.U32 else (_I64_MIN, _I64_MAX)
    if math.isinf(x):
        return hi if x > 0 else lo
    n = round(x)                         # Python round(): ties to even
    if vtype in (ValueType.I64, ValueType.U32):
        return min(max(n, lo), hi)
    return n


def write_typed(dev: CameraDevice, attr: str, type_str: str, value: float) -> None:
    """Write ``value`` with the accessor chosen by ``type_str`` (05 §5.1): DBL as is, I64/U32 coerced."""
    kind = value_kind(type_str)
    if kind is ValueType.DBL:
        dev.write_attribute(attr, kind, float(value))
    else:
        dev.write_attribute(attr, kind, lv_to_int(float(value), kind))


def read_typed(dev: CameraDevice, attr: str, type_str: str) -> float:
    """Read with the accessor chosen by ``type_str``, converted to DBL (05 §5.1: "reads always convert to DBL")."""
    return float(dev.read_attribute(attr, value_kind(type_str)))


def in_range_and_coerce(x: float, lo: float, hi: float) -> float:
    """LabVIEW ``In Range and Coerce`` as used by the server: ``min(max(x, lo), hi)`` (05 §1.2).

    With ``hi < lo`` (HM ``Max Framerate`` = 0) the result is ``hi``; LabVIEW's result there is unknown and only
    reaches a display and an error text (05 §5.2).
    """
    return min(max(x, lo), hi)


def format_lv_double(x: float, decimal_separator: str = ".") -> str:
    """``Format Into String`` of a DBL without a format string: ``%f`` with 6 decimals (inferred, 05 §5.2);
    ``Inf`` / ``-Inf`` / ``NaN`` as LabVIEW prints them."""
    if math.isnan(x):
        return "NaN"
    if math.isinf(x):
        return "Inf" if x > 0 else "-Inf"
    return f"{x:.6f}".replace(".", decimal_separator)
