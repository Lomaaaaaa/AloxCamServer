"""LabVIEW error clusters, the server's error codes and the Python exception hierarchy.

Spec: 09 §8 (LabVIEW error semantics), §9 (code register), §10 (Python error strategy); 04 §4.3 (error cluster).

Inside the server, failures are exceptions from this module; at the TCP / event-log boundary they become an
:class:`ErrorCluster`. Codes are never invented outside :class:`Code`.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, replace
from enum import IntEnum
from types import MappingProxyType

__all__ = [
    "NO_ERROR",
    "TEXT",
    "AloxError",
    "CameraError",
    "Code",
    "CodecError",
    "ConfigError",
    "ErrorCluster",
    "FocusError",
    "ProtocolError",
    "StageError",
    "clear_errors",
    "cluster_of",
    "decorate",
    "error_cluster_from_error_code",
    "merge_errors",
    "replace_source",
]


class Code(IntEnum):
    """Every code the server produces or tests (09 §9, §10.5). Values are final."""

    NO_ERROR = 0
    LV_INVALID_INPUT = 1                # LabVIEW "An input parameter is invalid" (non-finite numeric inputs)
    EXTRA_DATA_FOUND = -1               # HW TCP Extract TCP Data (02 §5.3) — diagnostic only
    PI_NOT_CONNECTED = -4               # Python choice (08 §13.3): GCS call with no open link
    PI_OPEN_PORT = -6                   # Python choice (08 §13.3): serial port open failure
    PI_TIMEOUT = -7                     # "Wait for axes to stop.vi: Timeout." / serial read timeout
    GCS_UNREFERENCED = 5                # proposal, implicit_z_reinit = FALSE (09 §10.5)
    TCP_TIMEOUT = 56
    VARIANT_TYPE_MISMATCH = 91          # undecodable Command Data (04 T1)
    PI_DRIVER = 100                     # PI LabVIEW driver checks ("<CMD> No axis specified", "No answer on ERR?")
    UNFLATTEN_FAILED = 116              # malformed/truncated flattened data (ProtocolError default)
    GET_IMAGE_TIMEOUT = 9009
    SIM_INVALID_TYPE = -9011            # negative, as printed (09 D16)
    REFOCUS_JOYSTICK = 9031
    Z_POSITION_TIMEOUT = 9401
    Z_OUT_OF_RANGE = 9405
    INTERNAL = 9500                     # unexpected exception (bug)            — 95xx block: 09 §10.5 proposal
    IMAGE_FILE = 9501                   # image file read/write failed
    CAMERA_SDK = 9502                   # camera driver error without IMAQdx equivalent (incl. SimCamera)
    FOCUS_METRIC = 9503                 # Phase 4
    FOCUS_INVALID_PARAMS = 9504         # Phase 4, focus_validate_params
    FOCUS_JOB_RUNNING = 9505            # Phase 4, focus_reject_concurrent_start
    UNKNOWN_COMMAND = 9506              # reply_unknown_commands = TRUE
    NOT_IMPLEMENTED_IN_PHASE = 9507     # Phase-1 stubs (Cam Autofocus, Cam Refocus)
    IMAGE_CODEC_UNAVAILABLE = 9508      # ImageCodec cannot flatten (NI Vision stub)
    VISION_OPERATION = 9509             # NI Vision operation failed, NI code unknown (flat-field size mismatch…)
    ABORTED_BY_SHUTDOWN = 9510          # Z set-up/reference/wait cancelled by the shutdown chain (reply dropped anyway)
    IMAQDX_INVALID_SESSION = -1074360316


#: Fixed source texts of the ALOX codes (09 §9.1), reproduced byte for byte.
TEXT: Mapping[int, str] = MappingProxyType({
    Code.Z_OUT_OF_RANGE: "Z move out of range",
    Code.GET_IMAGE_TIMEOUT: "get image error (timeout)",
    Code.SIM_INVALID_TYPE: "Invalid Type",
    Code.REFOCUS_JOYSTICK: "'refocus' warning (joystick enabled)",
    Code.Z_POSITION_TIMEOUT: "z stage get position timeout",
})


@dataclass(frozen=True)
class ErrorCluster:
    """LabVIEW error cluster ``{status, code, source}`` (04 §4.3); the only error form on the wire or in event logs.

    ``source`` is kept verbatim whatever ``status`` is (09 D18).
    """

    status: bool = False
    code: int = 0
    source: str = ""

    @property
    def is_warning(self) -> bool:
        """``status`` FALSE with a non-zero code (09 §8.1)."""
        return not self.status and self.code != 0

    @property
    def ok(self) -> bool:
        """``status`` FALSE (no error; warnings are ok)."""
        return not self.status


NO_ERROR = ErrorCluster()


class AloxError(Exception):
    """Base exception: carries a LabVIEW-compatible code and source text (09 §10.2)."""

    def __init__(self, code: int, source: str) -> None:
        super().__init__(f"{int(code)}: {source}")
        self.code = int(code)
        self.source = source

    def cluster(self) -> ErrorCluster:
        """The error as a cluster with ``status`` TRUE."""
        return ErrorCluster(True, self.code, self.source)


class ConfigError(AloxError):
    """Fatal configuration problem (tolerated INI problems are warnings, never exceptions)."""


class ProtocolError(AloxError):
    """Framing / flattening failure (02 §5–§8); code normally :attr:`Code.UNFLATTEN_FAILED`."""


class CameraError(AloxError):
    """Camera driver / acquisition failure (05)."""


class CodecError(CameraError):
    """:class:`aloxcam.imagedata.ImageCodec` failure (02 §8.5)."""


class StageError(AloxError):
    """PI GCS controller or driver failure (08)."""


class FocusError(AloxError):
    """Autofocus / refocus / focus-metric failure (06, 07; Phase 4)."""


def decorate(err: ErrorCluster, suffix: str) -> ErrorCluster:
    """Append ``suffix`` to ``source`` only when ``status`` is TRUE, no separator (03 §1, 09 §10.2)."""
    return replace(err, source=err.source + suffix) if err.status else err


def replace_source(err: ErrorCluster, source: str) -> ErrorCluster:
    """Replace ``source`` only when ``status`` is TRUE (``Cam Set Exposure``, 05 §5.2)."""
    return replace(err, source=source) if err.status else err


def merge_errors(*errs: ErrorCluster) -> ErrorCluster:
    """``Merge Errors``: the first error in input order, else the first warning, else no error (09 §8.1)."""
    for e in errs:
        if e.status:
            return e
    for e in errs:
        if e.code != 0:
            return e
    return NO_ERROR


def clear_errors(err: ErrorCluster, specific_code: int = 0) -> tuple[ErrorCluster, bool]:
    """``Clear Errors.vi``: returns ``(error out, specific error cleared?)`` (09 §8.1)."""
    if specific_code == 0:
        return NO_ERROR, False
    if err.code == specific_code:
        return NO_ERROR, True
    return err, False


def error_cluster_from_error_code(code: int, message: str = "", *, is_warning: bool = False,
                                  error_in: ErrorCluster = NO_ERROR, caller: str = "") -> ErrorCluster:
    """``Error Cluster From Error Code.vi`` with ``show call chain?`` FALSE (09 §8.1).

    Code 0 or an incoming error returns ``error_in`` unchanged; a warning on top of a warning keeps the existing one.
    Otherwise ``source = caller + "<ERR>\\n" + message`` (trailing whitespace trimmed; no marker without a message).
    """
    if code == 0 or error_in.status or (is_warning and error_in.code != 0):
        return error_in
    extra = f"<ERR>\n{message}".rstrip(" \t\r\n") if message else ""
    return ErrorCluster(not is_warning, int(code), caller + extra)


def cluster_of(exc: BaseException) -> ErrorCluster:
    """:class:`AloxError` → its cluster; anything else → ``{TRUE, 9500, "<type>: <message>"}`` (09 §10.3)."""
    if isinstance(exc, AloxError):
        return exc.cluster()
    return ErrorCluster(True, Code.INTERNAL, f"{type(exc).__name__}: {exc}")
