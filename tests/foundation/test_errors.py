"""aloxcam.errors: error clusters, LabVIEW helpers and the code register (09 §8–§10, design §3.1.1)."""

from __future__ import annotations

import pytest

from aloxcam.errors import (
    NO_ERROR,
    TEXT,
    AloxError,
    CameraError,
    Code,
    CodecError,
    ConfigError,
    ErrorCluster,
    FocusError,
    ProtocolError,
    StageError,
    clear_errors,
    cluster_of,
    decorate,
    error_cluster_from_error_code,
    merge_errors,
    replace_source,
)

ERR = ErrorCluster(True, 9405, "Z move out of range")
WARN = ErrorCluster(False, 42, "a warning")


def test_code_values_are_final() -> None:
    expected = {
        "NO_ERROR": 0, "LV_INVALID_INPUT": 1, "EXTRA_DATA_FOUND": -1, "PI_SEND_ERROR": -2, "PI_RECEIVE_ERROR": -3,
        "PI_NOT_CONNECTED": -4, "PI_OPEN_PORT": -6,
        "PI_TIMEOUT": -7, "GCS_UNREFERENCED": 5, "TCP_TIMEOUT": 56, "VARIANT_TYPE_MISMATCH": 91, "PI_DRIVER": 100,
        "UNFLATTEN_FAILED": 116, "GET_IMAGE_TIMEOUT": 9009, "SIM_INVALID_TYPE": -9011, "REFOCUS_JOYSTICK": 9031,
        "Z_POSITION_TIMEOUT": 9401, "Z_OUT_OF_RANGE": 9405, "INTERNAL": 9500, "IMAGE_FILE": 9501,
        "CAMERA_SDK": 9502, "FOCUS_METRIC": 9503, "FOCUS_INVALID_PARAMS": 9504, "FOCUS_JOB_RUNNING": 9505,
        "UNKNOWN_COMMAND": 9506, "NOT_IMPLEMENTED_IN_PHASE": 9507, "IMAGE_CODEC_UNAVAILABLE": 9508,
        "VISION_OPERATION": 9509, "ABORTED_BY_SHUTDOWN": 9510, "IMAQDX_INVALID_SESSION": -1074360316,
    }
    assert {c.name: c.value for c in Code} == expected


def test_fixed_source_texts() -> None:
    assert TEXT[9405] == "Z move out of range"
    assert TEXT[9009] == "get image error (timeout)"
    assert TEXT[-9011] == "Invalid Type"
    assert TEXT[9031] == "'refocus' warning (joystick enabled)"
    assert TEXT[9401] == "z stage get position timeout"


def test_cluster_properties() -> None:
    assert NO_ERROR == ErrorCluster(False, 0, "")
    assert NO_ERROR.ok and not NO_ERROR.is_warning
    assert WARN.ok and WARN.is_warning
    assert not ERR.ok and not ERR.is_warning
    with pytest.raises(AttributeError):
        ERR.code = 1  # type: ignore[misc]


def test_decorate_only_when_status() -> None:
    assert decorate(ERR, "Cam Setup") == ErrorCluster(True, 9405, "Z move out of rangeCam Setup")
    assert decorate(WARN, "Cam Setup") is WARN
    assert decorate(NO_ERROR, "x") is NO_ERROR


def test_replace_source_only_when_status() -> None:
    assert replace_source(ERR, "new") == ErrorCluster(True, 9405, "new")
    assert replace_source(WARN, "new") is WARN


def test_merge_errors_order() -> None:
    e2 = ErrorCluster(True, 1, "second")
    w2 = ErrorCluster(False, 7, "second warning")
    assert merge_errors(WARN, ERR, e2) is ERR          # first error wins over an earlier warning
    assert merge_errors(NO_ERROR, WARN, w2) is WARN    # else the first warning
    assert merge_errors(NO_ERROR, NO_ERROR) is NO_ERROR
    assert merge_errors() is NO_ERROR


def test_clear_errors() -> None:
    assert clear_errors(ERR) == (NO_ERROR, False)
    assert clear_errors(ERR, 9405) == (NO_ERROR, True)
    assert clear_errors(ERR, -1074360316) == (ERR, False)
    session = ErrorCluster(True, Code.IMAQDX_INVALID_SESSION, "x")
    assert clear_errors(session, Code.IMAQDX_INVALID_SESSION) == (NO_ERROR, True)


def test_error_cluster_from_error_code() -> None:
    assert error_cluster_from_error_code(0, "ignored") is NO_ERROR
    assert error_cluster_from_error_code(0, error_in=WARN) is WARN
    assert error_cluster_from_error_code(5, "m", error_in=ERR) is ERR                  # an error in trumps the code
    assert error_cluster_from_error_code(5, is_warning=True, error_in=WARN) is WARN   # existing warning kept
    got = error_cluster_from_error_code(-1, "Extra Data Found", caller="HW TCP Extract TCP Data.vi")
    assert got == ErrorCluster(True, -1, "HW TCP Extract TCP Data.vi<ERR>\nExtra Data Found")
    assert error_cluster_from_error_code(-1, "msg  \n") == ErrorCluster(True, -1, "<ERR>\nmsg")
    assert error_cluster_from_error_code(7, caller="x.vi") == ErrorCluster(True, 7, "x.vi")
    assert error_cluster_from_error_code(7, "w", is_warning=True) == ErrorCluster(False, 7, "<ERR>\nw")


def test_exceptions_carry_code_and_source() -> None:
    exc = StageError(Code.Z_OUT_OF_RANGE, "Z move out of range")
    assert exc.code == 9405 and type(exc.code) is int and exc.source == "Z move out of range"
    assert exc.cluster() == ErrorCluster(True, 9405, "Z move out of range")
    assert "9405" in str(exc)
    for cls in (ConfigError, ProtocolError, CameraError, CodecError, StageError, FocusError):
        assert issubclass(cls, AloxError)
    assert issubclass(CodecError, CameraError)


def test_cluster_of() -> None:
    assert cluster_of(CameraError(-1074360316, "IMAQdx")) == ErrorCluster(True, -1074360316, "IMAQdx")
    got = cluster_of(ValueError("boom"))
    assert got == ErrorCluster(True, 9500, "ValueError: boom")
    assert got.code == Code.INTERNAL
