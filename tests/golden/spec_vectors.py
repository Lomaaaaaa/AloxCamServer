"""Every worked hex example of the specification, as data (design §4.2 WP2, §4.3). Imported read-only by tests.

Sources: 02 §5.4 (heartbeat frame), 02 §7.5 (b) (``Z Set Min Limit`` command frame), 02 §8.6 (57-byte
``Get Heartbeat`` reply and the 9 ``Response (Flattened)`` rows), 04 §3.2 (paths), 04 §5.7 (18 payload data
encodings + the 66-byte autofocus size note), 03 §3.5 (synthetic enum order), 09 §7.3–§7.4 (event-log lines).

Hex templates keep placeholders where the spec wrote a code it did not know: ``{tt}`` (``HW TCP Datatype``),
``{cc}`` (``Camera Command``) and, in the payload rows, one ``{<field>}`` per enum field. :func:`fill` substitutes
codes taken from the **loaded** enum table, so tests never hard-code enum codes (``PayloadVector.codes`` says which
enum item each placeholder stands for).
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field, replace
from datetime import datetime
from typing import Any

from aloxcam.dispatch import ReplyKind
from aloxcam.errors import NO_ERROR, ErrorCluster
from aloxcam.lv.path import PATH_ABSOLUTE, PATH_NOT_A_PATH, PATH_RELATIVE, PATH_UNC, LVPath

_PLACEHOLDER = re.compile(r"\{(\w+)\}")


def h(text: str) -> bytes:
    """Hex text with any whitespace → bytes."""
    return bytes.fromhex("".join(text.split()))


def fill(template: str, **codes: int | bytes) -> bytes:
    """Substitute each ``{name}`` with ``codes[name]`` (an int is written as a big-endian U16, bytes as they are)."""
    def sub(m: re.Match[str]) -> str:
        value = codes[m.group(1)]
        return value.hex() if isinstance(value, bytes) else int(value).to_bytes(2, "big").hex()
    return h(_PLACEHOLDER.sub(sub, template))


def placeholders(template: str) -> set[str]:
    """The placeholder names used by ``template``."""
    return set(_PLACEHOLDER.findall(template))


# ---------------------------------------------------------------------------------------------- frames (02 §5.4, §7.5)
#: 02 §5.4: a heartbeat frame. ``tt`` = "Heartbeat".
HEARTBEAT_FRAME = "00 00 00 02 {tt}"


@dataclass(frozen=True)
class CommandFrameVector:
    """A whole command frame (02 §7.5 (b)); ``tt`` = ``datatype`` item, ``cc`` = ``command`` item."""

    template: str
    datatype: str
    command: str
    notifier: bytes
    payload_class: str
    payload_fields: dict[str, Any]
    ts: tuple[int, int, int]
    variant_version: int
    top_type_code: int
    data_hex: str


#: 02 §7.5 (b): ``Z Set Min Limit`` = 0.5, notifier ``"Z#17"``, timestamps 1000 / 1001 / 0 (N = 52).
Z_SET_MIN_LIMIT_FRAME = CommandFrameVector(
    template="""
        00 00 00 34  {tt}  {cc}
        00 00 00 04 5a 23 31 37
        18 00 80 00  00 00 00 01  00 04 00 0a
        00 01 00 00
        3f e0 00 00 00 00 00 00
        00 00 00 00
        00 00 03 e8  00 00 03 e9  00 00 00 00
    """,
    datatype="Command",
    command="Z Set Min Limit",
    notifier=b"Z#17",
    payload_class="ZLimitData",
    payload_fields={"value_mm": 0.5},
    ts=(1000, 1001, 0),
    variant_version=0x18008000,
    top_type_code=0x0A,
    data_hex="3f e0 00 00 00 00 00 00",
)

#: 02 §7.5 (b): the bare-DBL variant header alone (version 18.0, 1 TD ``00 04 00 0a``, top-level list [0]).
BARE_DBL_VARIANT_HEADER = "18 00 80 00 00 00 00 01 00 04 00 0a 00 01 00 00"


# ---------------------------------------------------------------------------------------------- replies (02 §8.6)
@dataclass(frozen=True)
class ReplyFrameVector:
    """A whole reply frame (02 §8.6); ``tt`` = ``datatype`` item."""

    template: str
    datatype: str
    notifier: bytes
    kind: ReplyKind
    value: Any
    err: ErrorCluster
    ts: tuple[int, int, int, int, int, int]
    size: int


#: 02 §8.6: complete reply to ``Get Heartbeat`` (U32 123456, notifier ``"NotifA"``, TCP Sending 12345678; 57 bytes).
GET_HEARTBEAT_REPLY = ReplyFrameVector(
    template="""
        00 00 00 35
        {tt}
        00 00 00 06 4e 6f 74 69 66 41
        00 00 00 0d
           00 01 e2 40
           00 00 00 00 00 00 00 00 00
        00 00 00 00
        00 00 00 00
        00 00 00 00
        00 00 00 00
        00 00 00 00
        00 bc 61 4e
    """,
    datatype="Response",
    notifier=b"NotifA",
    kind=ReplyKind.U32,
    value=123456,
    err=NO_ERROR,
    ts=(0, 0, 0, 0, 0, 12345678),
    size=57,
)


@dataclass(frozen=True)
class FlattenedReplyVector:
    """One row of the 02 §8.6 ``Response (Flattened)`` table."""

    name: str
    kind: ReplyKind
    value: Any
    err: ErrorCluster
    hex: str


#: 02 §8.6: the 9 ``Response (Flattened)`` encodings (sources marked "illustrative" in the spec kept verbatim).
RESPONSE_FLATTENED: tuple[FlattenedReplyVector, ...] = (
    FlattenedReplyVector("error only, OK", ReplyKind.ACK, None, NO_ERROR, "00 00000000 00000000"),
    FlattenedReplyVector("error only, 9405", ReplyKind.ACK, None, ErrorCluster(True, 9405, "Z Move Absolute"),
                         "01 000024bd 0000000f 5a204d6f7665204162736f6c757465"),
    FlattenedReplyVector("STR hello, OK", ReplyKind.STR, "hello", NO_ERROR,
                         "00000005 68656c6c6f 00 00000000 00000000"),
    FlattenedReplyVector("Boolean TRUE, OK", ReplyKind.BOOL, True, NO_ERROR, "01 00 00000000 00000000"),
    FlattenedReplyVector("Boolean FALSE with IMAQdx error", ReplyKind.BOOL, False,
                         ErrorCluster(True, -1074360316, "Cam Acquisition status"),
                         "00 01 bff69004 00000016 43616d204163717569736974696f6e20737461747573"),
    FlattenedReplyVector("DBL 1500.0, OK", ReplyKind.DBL, 1500.0, NO_ERROR,
                         "4097700000000000 00 00000000 00000000"),
    FlattenedReplyVector("U32 123456, OK", ReplyKind.U32, 123456, NO_ERROR, "0001e240 00 00000000 00000000"),
    FlattenedReplyVector("DBL 2-D 2x2, OK", ReplyKind.DBL2D, [[1.0, 0.5], [2.0, 0.75]], NO_ERROR,
                         "00000002 00000002 3ff0000000000000 3fe0000000000000 4000000000000000 3fe8000000000000 "
                         "00 00000000 00000000"),
    FlattenedReplyVector("Image on error", ReplyKind.IMAGE, b"", ErrorCluster(True, 1, "e"),
                         "00000000 01 00000001 00000001 65"),
)


# ---------------------------------------------------------------------------------------------- paths (04 §3.2)
@dataclass(frozen=True)
class PathVector:
    """A path, its Windows text and (when the spec gives it) its flattened bytes."""

    name: str
    path: LVPath
    windows: str | None
    hex: str | None


PATH_VECTORS: tuple[PathVector, ...] = (
    PathVector("empty path", LVPath(PATH_ABSOLUTE, ()), "", "50 54 48 30 00 00 00 04 00 00 00 00"),
    PathVector("C:\\f.png", LVPath(PATH_ABSOLUTE, ("C", "f.png")), "C:\\f.png",
               "50 54 48 30 00 00 00 0c 00 00 00 02 01 43 05 66 2e 70 6e 67"),
    PathVector("Not A Path", LVPath(PATH_NOT_A_PATH, ()), None, "50 54 48 30 00 00 00 04 00 02 00 00"),
)

#: 04 §3.2: text forms given without bytes (UNC and relative with "..").
PATH_TEXT_VECTORS: tuple[PathVector, ...] = (
    PathVector("UNC", LVPath(PATH_UNC, ("srv", "share", "a.png")), "\\\\srv\\share\\a.png", None),
    PathVector("relative", LVPath(PATH_RELATIVE, ("", "x", "y")), "..\\x\\y", None),
)


# ---------------------------------------------------------------------------------------------- payloads (04 §5.7)
@dataclass(frozen=True)
class PayloadVector:
    """One row of 04 §5.7 (data part of the variant). ``layout`` names a class attribute when the row uses an
    order other than ``LV_LAYOUT`` (e.g. ``"LAYOUT_UNBUNDLE"``); ``codes`` maps placeholders to ``(enum_id, label)``."""

    name: str
    payload_class: str
    fields: dict[str, Any]
    size: int
    hex: str
    codes: dict[str, tuple[str, str]] = field(default_factory=dict)
    layout: str | None = None

    def code_values(self, enums: Any) -> dict[str, bytes]:
        """Placeholder values from an enum table (anything with ``code(enum_id, label)`` and ``size(enum_id)``),
        each written big-endian at the enum's width (Camera Type is U32 on the wire: capture-analysis.md)."""
        return {name: enums.code(enum_id, label).to_bytes(enums.size(enum_id), "big")
                for name, (enum_id, label) in self.codes.items()}


def _ct(label: str) -> dict[str, tuple[str, str]]:
    return {"camera_type": ("camera_type", label)}


_ZEROS = " ".join(["00"] * 40)
_AF_DEFAULT_CODES = {"camera_type": ("camera_type", "High Mag."),
                     "autofocus_speed": ("autofocus_speed", "Slow"),
                     "autofocus_method": ("autofocus_method", "FFT")}
_RF_DEFAULT_CODES = {"camera_type": ("camera_type", "High Mag."),
                     "autofocus_method": ("autofocus_method", "FFT"),
                     "refocus_method": ("refocus_method", "Fin")}

PAYLOAD_VECTORS: tuple[PayloadVector, ...] = (
    PayloadVector("Camera Stop Data{Low Mag.}", "CameraStopData", {"camera_type": "Low Mag."}, 2,
                  "{camera_type}", _ct("Low Mag.")),
    PayloadVector("Camera Start Data{High Mag., TRUE}", "CameraStartData",
                  {"camera_type": "High Mag.", "triggered": True}, 3, "{camera_type} 01", _ct("High Mag.")),
    PayloadVector("Camera Flip Data{Low Mag., FALSE, TRUE}", "CameraFlipData",
                  {"camera_type": "Low Mag.", "horizontal_flip": False, "vertical_flip": True}, 4,
                  "{camera_type} 00 01", _ct("Low Mag.")),
    PayloadVector("Camera Set Exposure Data{Low Mag., 1500.0}", "CameraSetExposureData",
                  {"camera_type": "Low Mag.", "exposure": 1500.0}, 10, "{camera_type} 40 97 70 00 00 00 00 00",
                  _ct("Low Mag.")),
    PayloadVector("Camera Gain Data{High Mag., 2.5}", "CameraGainData", {"camera_type": "High Mag.", "gain": 2.5},
                  10, "{camera_type} 40 04 00 00 00 00 00 00", _ct("High Mag.")),
    PayloadVector("Camera Init Data{High Mag., cam0, 5, TRUE}, Unbundle order", "CameraInitData",
                  {"camera_type": "High Mag.", "camera_name": "cam0", "number_of_buffers": 5,
                   "continuous_mode": True}, 15,
                  "{camera_type} 01 00 00 00 05 00 00 00 04 63 61 6d 30", _ct("High Mag."), layout="LAYOUT_UNBUNDLE"),
    PayloadVector("Camera Init Data{High Mag., cam0, 5, TRUE}, panel order", "CameraInitData",
                  {"camera_type": "High Mag.", "camera_name": "cam0", "number_of_buffers": 5,
                   "continuous_mode": True}, 15,
                  "{camera_type} 00 00 00 04 63 61 6d 30 00 00 00 05 01", _ct("High Mag.")),
    PayloadVector("Camera Stream Data (defaults)", "CameraStreamData", {}, 23,
                  "{camera_type} 00 50 54 48 30 00 00 00 04 00 00 00 00 00 00 00 00 00 00 13 88", _ct("High Mag.")),
    PayloadVector("Camera Stream Data{High Mag., TRUE, D:\\sim, 200, 5000}", "CameraStreamData",
                  {"camera_type": "High Mag.", "simulation": True,
                   "simulation_path": LVPath(PATH_ABSOLUTE, ("D", "sim")), "simulation_timer_ms": 200,
                   "acquisition_timeout_ms": 5000}, 29,
                  "{camera_type} 01 50 54 48 30 00 00 00 0a 00 00 00 02 01 44 03 73 69 6d 00 00 00 c8 00 00 13 88",
                  _ct("High Mag.")),
    PayloadVector("Camera Ext Set FlatField{High Mag., TRUE, C:\\f.png, <empty>, 1.0}", "CameraSetFlatFieldData",
                  {"camera_type": "High Mag.", "enable": True, "ffc_path": LVPath(PATH_ABSOLUTE, ("C", "f.png")),
                   "dfc_path": LVPath(), "correction_factor": 1.0}, 43,
                  "{camera_type} 01 50 54 48 30 00 00 00 0c 00 00 00 02 01 43 05 66 2e 70 6e 67 "
                  "50 54 48 30 00 00 00 04 00 00 00 00 3f f0 00 00 00 00 00 00", _ct("High Mag.")),
    PayloadVector("Cam Log Data (defaults)", "CamLogData", {}, 21,
                  "00 50 54 48 30 00 00 00 04 00 00 00 00 00 00 00 00 00 00 00 00"),
    PayloadVector("Show UI TRUE", "ShowUIData", {"show": True}, 1, "01"),
    PayloadVector("Camera Autofocus Data (defaults)", "CameraAutofocusData", {}, 50,
                  "{camera_type} {autofocus_speed} {autofocus_method} " + _ZEROS + " 00 00 13 88", _AF_DEFAULT_CODES),
    PayloadVector("Camera Refocus Data (defaults)", "CameraRefocusData", {}, 59,
                  "{camera_type} {autofocus_method} {refocus_method} " + " ".join(["00"] * 32)
                  + " 00 00 13 88 " + " ".join(["00"] * 5) + " 50 54 48 30 00 00 00 04 00 00 00 00", _RF_DEFAULT_CODES),
    PayloadVector("Z Stage Wait Data (defaults)", "ZWaitData", {}, 9, "00 00 00 00 00 00 00 0a 00"),
    PayloadVector("Z Stage Joystick Data (defaults)", "ZJoystickData", {}, 8, "00 00 00 01 00 00 00 01"),
    PayloadVector("Z Move Abs Data{12.5}", "ZMoveAbsData", {"pos_mm": 12.5}, 8, "40 29 00 00 00 00 00 00"),
    PayloadVector("Z Set Min Limit 0.5", "ZLimitData", {"value_mm": 0.5}, 8, "3f e0 00 00 00 00 00 00"),
)

#: 04 §5.7 size note: ``Camera Autofocus Data`` with a 4-element ROI is 66 bytes (50 + 4 × 4).
AF_WITH_ROI4 = PayloadVector("Camera Autofocus Data with a 4-element ROI", "CameraAutofocusData",
                             {"roi": [10, 20, 300, 400]}, 66, "", _AF_DEFAULT_CODES)


# ---------------------------------------------------------------------------------------------- routing (03 §3.5)
#: The camera range of the real enum (owner-answers §1, codes 0..25).
_CAMERA_ITEMS = (
    "Get Heartbeat", "Cam Setup", "Cam Start", "Cam Get Image", "Cam Stream Next Image", "Cam Get Acquisition Status",
    "Cam Software Trigger", "Cam Set Exposure", "Cam Get Exposure", "Cam Set Gain", "Cam Get Gain",
    "Cam Get Images Array", "Cam Stop", "Cam Cleanup", "Exit", "Show UI", "Cam Autofocus", "Cam Set FlatField",
    "Cam Refocus", "Cam Flip", "Cam Get Autofocus Data", "Cam Get Autofocus Progress", "Cam Autofocus Run",
    "Cam Refocus Run", "Cam Set Led", "Cam Get Led",
)
#: 03 §3.2 point 3: the Z case-frame order, with ``Z Do Reference`` 8th of 19.
Z_CASE_FRAME_ORDER = (
    "Z Setup", "Z Get Serial", "Z Get Status", "Z Get Error", "Z Set Speed", "Z Get Speed", "Z Get Reference",
    "Z Do Reference", "Z Get Position", "Z Move Absolute", "Z Move Relative", "Z Wait End Move", "Z Stop",
    "Z Enable Joystick", "Z Disable Joystick", "Z Joystick Enabled ?", "Z Set Min Limit", "Z Set Max Limit",
    "Z Cleanup",
)
#: 03 §3.5: a synthetic item order in which ``Z Do Reference`` sits 8th of the 19 Z items.
SYNTHETIC_ORDER_03_3_5 = _CAMERA_ITEMS + Z_CASE_FRAME_ORDER + ("Log",)
#: The 11 Z commands ``routing_self_check`` must report as unreachable with that order.
SYNTHETIC_UNREACHABLE = Z_CASE_FRAME_ORDER[8:]


# ---------------------------------------------------------------------------------------------- event logs (09 §7)
@dataclass(frozen=True)
class LogLineVector:
    """A 09 §7.4 line. Where the spec wrote ``<code>`` / ``<driver source>``, ``line`` and ``err.source`` hold
    ``{code}`` / ``{source}`` placeholders: use :meth:`filled` to get a concrete line."""

    level: str                            # "ERROR" (Log Server Error) | "INFO" (Log Server Infos)
    when: datetime
    err: ErrorCluster
    server_type: str
    extra: str
    command: str | None
    camera_type: str
    line: str

    @property
    def is_template(self) -> bool:
        return "{code}" in self.line or "{source}" in self.line

    def filled(self, code: int, source: str) -> LogLineVector:
        """A concrete instance of a template line."""
        return replace(self, err=ErrorCluster(self.err.status, code, self.err.source.format(source=source)),
                       line=self.line.format(code=code, source=source))


LOG_LINES: tuple[LogLineVector, ...] = (
    LogLineVector("ERROR", datetime(2026, 10, 29, 16, 31, 5, 123000), ErrorCluster(True, 9405, "Z move out of range"),
                  "Z Stage", "Move Absolute", None, "High Mag.",
                  "29/10/2026 - 16:31:05.123 - ERR (Z STAGE)-(9405) : Z move out of range (Move Absolute)"),
    LogLineVector("ERROR", datetime(2026, 10, 29, 16, 31, 7, 456000), ErrorCluster(True, 0, "{source}Cam Setup"),
                  "Camera", "Setup", "Cam Setup", "Low Mag.",
                  "29/10/2026 - 16:31:07.456 - ERR (LOW MAG)-Cam Setup({code}) : {source}Cam Setup (Setup)"),
    LogLineVector("INFO", datetime(2026, 10, 29, 16, 32, 0, 789000), NO_ERROR, "Z Stage",
                  "Move Absolute: Z Reference resetted", None, "High Mag.",
                  "29/10/2026 - 16:32:00.789 - INFO -(0) :  (Move Absolute: Z Reference resetted)"),
)


@dataclass(frozen=True)
class OffsetZeroExample:
    """09 §7.3: two error lines written at offset 0 (``append_errors = FALSE``); the second is shorter."""

    first: LogLineVector
    second: LogLineVector
    expected_tail: bytes                  # what remains of the first line after the second one


OFFSET_ZERO_EXAMPLE = OffsetZeroExample(
    first=LOG_LINES[0],
    second=LogLineVector("ERROR", datetime(2026, 10, 29, 16, 31, 6, 0), ErrorCluster(True, 1, "short"), "Z Stage",
                         "x", None, "High Mag.", "29/10/2026 - 16:31:06.000 - ERR (Z STAGE)-(1) : short (x)"),
    expected_tail=b"ut of range (Move Absolute)\r\n",
)
