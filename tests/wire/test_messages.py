"""aloxcam.wire.messages: registry, payload codec and the 04 §5.7 vectors (03 §2, 04 §5, §7.1, design §3.2.5)."""

from __future__ import annotations

import dataclasses
import logging
import string

import numpy as np
import pytest

from aloxcam.dispatch import CAMERA_WORKER, Z_WORKER, DataMode, FinishOrder, ReplyKind
from aloxcam.errors import ConfigError, ErrorCluster, ProtocolError
from aloxcam.lv.flatten import Reader
from aloxcam.lv.path import PATH_ABSOLUTE, PATH_UNC, LVPath
from aloxcam.lv.variant import EMPTY_VARIANT_HEADER, read_variant_header
from aloxcam.testing import make_request, packaged_enums
from aloxcam.wire import messages
from aloxcam.wire.enums import WireEnums
from aloxcam.wire.messages import (
    CASES,
    COMMANDS,
    PAYLOAD_CLASSES,
    CameraAutofocusData,
    CameraFlipData,
    CameraInitData,
    CameraSetExposureData,
    CameraSetFlatFieldData,
    CameraStopData,
    CamLogData,
    PayloadCodec,
    ZExtraParamData,
    ZLimitData,
    ZReferenceData,
    payload_from_mapping,
    payload_to_mapping,
)
from golden import spec_vectors as sv

# The spec examples (02 §7.5, 04 §5.7) were written with a U16 Camera Type; the real wire uses U32 (capture),
# which tests/wire/test_capture_vectors.py covers. These tests check the codec mechanics against the spec bytes.
ENUMS = WireEnums.load(sizes={"camera_type": 2})
C, Z = CAMERA_WORKER, Z_WORKER
A, S, D, U, B, IMG, D2, N = (ReplyKind.ACK, ReplyKind.STR, ReplyKind.DBL, ReplyKind.U32, ReplyKind.BOOL,
                             ReplyKind.IMAGE, ReplyKind.DBL2D, ReplyKind.NONE)

#: 03 §2 (worker, reply kind) and 04 §5.1 (Command Data typedef label; None = not read; "unused" = {Extra Param}).
EXPECTED = {
    "Get Heartbeat": (C, U, None), "Show UI": (C, A, "Show UI"), "Log": (C, A, "Cam Log Data"), "Exit": (C, N, None),
    "Cam Setup": (C, A, "Camera Init Data"), "Cam Start": (C, A, "Camera Start Data"),
    "Cam Stop": (C, A, "Camera Stop Data"), "Cam Cleanup": (C, A, "Camera Release Data"),
    "Cam Get Acquisition Status": (C, B, "Camera Status Data"),
    "Cam Software Trigger": (C, A, "Camera Trigger Soft Data"),
    "Cam Set Exposure": (C, A, "Camera Set Exposure Data"), "Cam Get Exposure": (C, D, "Camera Get Exposure Data"),
    "Cam Set Gain": (C, A, "Camera Gain Data"), "Cam Get Gain": (C, D, "Camera Get Gain Data"),
    "Cam Stream Next Image": (C, IMG, "Camera Stream Data"), "Cam Get Image": (C, N, "Camera Get Image Data"),
    "Cam Get Images Array": (C, N, None), "Cam Flip": (C, A, "Camera Flip Data"),
    "Cam Set FlatField": (C, A, "Camera Ext Set FlatField"), "Cam Set Led": (C, A, "Camera Set LED Data"),
    "Cam Get Led": (C, B, "Camera Get LED Data"), "Cam Autofocus": (C, A, "Camera Autofocus Data"),
    "Cam Refocus": (C, A, "Camera Refocus Data"), "Cam Autofocus Run": (C, N, "Camera Autofocus Data"),
    "Cam Refocus Run": (C, N, "Camera Refocus Data"), "Cam Get Autofocus Data": (C, D2, "Camera Get Autofocus Data"),
    "Cam Get Autofocus Progress": (C, D, "Camera Get Autofocus Data"),
    "Z Setup": (Z, A, None), "Z Get Serial": (Z, S, "Z Stage Get Serial Data"), "Z Get Status": (Z, B, None),
    "Z Get Error": (Z, S, None), "Z Set Speed": (Z, A, "Z Stage Set Speed Data"), "Z Get Speed": (Z, D, "unused"),
    "Z Get Reference": (Z, B, "unused"), "Z Do Reference": (Z, A, "Z Reference Method"),
    "Z Get Position": (Z, D, "unused"), "Z Move Absolute": (Z, A, "Z Move Abs Data"),
    "Z Move Relative": (Z, A, "Z Stage Move Rel Data"), "Z Wait End Move": (Z, A, "Z Stage Wait Data"),
    "Z Stop": (Z, A, "unused"), "Z Enable Joystick": (Z, A, "Z Stage Joystick Data"),
    "Z Disable Joystick": (Z, A, "Z Stage Joystick Data"), "Z Joystick Enabled ?": (Z, B, "Z Stage Joystick Data"),
    "Z Set Min Limit": (Z, A, "Z Limit"), "Z Set Max Limit": (Z, A, "Z Limit"), "Z Cleanup": (Z, A, None),
}


# ---------------------------------------------------------------------------------------------- registry
def test_commands_registry_matches_03_2_and_04_5_1() -> None:
    assert len(COMMANDS) == 46 and set(COMMANDS) == set(EXPECTED)
    assert set(COMMANDS) == set(ENUMS.table("camera_command").items)
    for name, (worker, reply, typedef) in EXPECTED.items():
        spec = COMMANDS[name]
        assert spec.name == name and spec.worker == worker and spec.reply is reply, name
        if typedef is None:
            assert spec.data_mode is DataMode.NOT_READ and spec.payload is None, name
        elif typedef == "unused":
            assert spec.data_mode is DataMode.UNUSED and spec.payload is ZExtraParamData, name
        else:
            assert spec.data_mode is DataMode.STRICT and spec.payload.LV_TYPEDEF == typedef, name
        assert spec.spec_ref, name
    assert all(spec.order is FinishOrder.Z for spec in COMMANDS.values() if spec.worker == Z_WORKER)
    assert {n for n, s in COMMANDS.items() if s.order is FinishOrder.RAW_FIRST} == {
        "Get Heartbeat", "Cam Set Led", "Cam Get Led", "Cam Get Autofocus Data", "Cam Get Autofocus Progress"}


def test_registry_texts() -> None:
    assert (COMMANDS["Cam Stop"].extra, COMMANDS["Cam Stop"].suffix) == ("Stop", "Cam stop")
    assert (COMMANDS["Cam Set FlatField"].extra, COMMANDS["Cam Set FlatField"].suffix) == ("Set FlatField",
                                                                                        "Cam Set Field")
    assert COMMANDS["Cam Setup"].suffix is None and COMMANDS["Cam Set Exposure"].suffix == ""
    assert COMMANDS["Z Do Reference"].extra == "Get Z Referenced"
    assert COMMANDS["Z Joystick Enabled ?"].extra == "Is Joystick Enabled ?"
    assert COMMANDS["Cam Autofocus Run"].suffix == "Autofocus"


def test_cases_exclude_the_run_commands() -> None:
    assert len(CASES) == 44
    assert "Cam Autofocus Run" not in CASES and "Cam Refocus Run" not in CASES
    assert not COMMANDS["Cam Autofocus Run"].client_routable and not COMMANDS["Cam Refocus Run"].client_routable
    assert CASES["Exit"] == CAMERA_WORKER and CASES["Log"] == CAMERA_WORKER and CASES["Z Cleanup"] == Z_WORKER


def test_make_request_looks_up_spec_and_code() -> None:
    r = make_request("Cam Setup", CameraInitData(camera_name="cam0"))
    assert r.spec is COMMANDS["Cam Setup"] and r.code == 1 and r.data.camera_name == "cam0"
    unknown = make_request("#99")
    assert unknown.spec is None and unknown.code == 99
    with pytest.raises(ValueError):
        make_request("No Such Command")


def test_payload_classes() -> None:
    typedefs = [cls.LV_TYPEDEF for cls in PAYLOAD_CLASSES]
    assert len(set(typedefs)) == len(typedefs) == 30
    assert {cls for cls in PAYLOAD_CLASSES if cls.LV_BARE} == {messages.ShowUIData, ZReferenceData, ZLimitData}
    for cls in PAYLOAD_CLASSES:
        names = [f.name for f in dataclasses.fields(cls)]
        assert sorted(a for a, _ in cls.LV_LAYOUT) == sorted(names), cls
    with pytest.raises(dataclasses.FrozenInstanceError):
        CameraStopData().camera_type = "Low Mag."  # type: ignore[misc]
    assert CameraInitData.LV_LAYOUT == CameraInitData.LAYOUT_PANEL
    assert CameraAutofocusData().autofocus_speed == "Slow" and messages.CameraRefocusData().refocus_method == "Fin"
    assert ZReferenceData().method == "FNL" and messages.CameraStreamData().acquisition_timeout_ms == 5000


# ---------------------------------------------------------------------------------------------- 04 §5.7 vectors
def _spec_for(cls: type):  # type: ignore[no-untyped-def]
    return next(s for s in COMMANDS.values() if s.payload is cls and s.data_mode is DataMode.STRICT)


@pytest.mark.parametrize("vector", sv.PAYLOAD_VECTORS, ids=lambda v: v.name)
def test_04_5_7_vectors_decode_and_encode_byte_exactly(vector: sv.PayloadVector) -> None:
    assert len(sv.PAYLOAD_VECTORS) == 18
    cls = getattr(messages, vector.payload_class)
    obj = cls(**vector.fields)
    data = sv.fill(vector.hex, **vector.code_values(ENUMS))
    assert len(data) == vector.size
    assert sv.placeholders(vector.hex) == set(vector.codes)
    if vector.layout:
        layout = getattr(cls, vector.layout)
        codec = PayloadCodec(ENUMS, {cls.LV_TYPEDEF: ", ".join(f"{a}:{k}" for a, k in layout)})
        assert codec.layout_of(cls) == layout
    else:
        codec = PayloadCodec(ENUMS)
    assert codec.encode(obj) == data
    assert codec.decode(_spec_for(cls), data) == obj


def test_af_size_note_66_bytes() -> None:
    v = sv.AF_WITH_ROI4
    data = PayloadCodec(ENUMS).encode(CameraAutofocusData(**v.fields))
    assert len(data) == v.size == 66
    assert PayloadCodec(ENUMS).decode(COMMANDS["Cam Autofocus"], data).roi == [10, 20, 300, 400]


# ---------------------------------------------------------------------------------------------- Camera Init Data
def _unbundle_bytes(name: str, buffers: int, continuous: bool) -> bytes:
    nb = name.encode("cp1252")
    return (b"\x00\x00" + bytes([continuous]) + buffers.to_bytes(4, "big") + len(nb).to_bytes(4, "big") + nb)


def _panel_bytes(name: str, buffers: int, continuous: bool) -> bytes:
    nb = name.encode("cp1252")
    return b"\x00\x00" + len(nb).to_bytes(4, "big") + nb + buffers.to_bytes(4, "big") + bytes([continuous])


def test_init_data_panel_order_is_primary() -> None:
    codec = PayloadCodec(ENUMS)
    spec = COMMANDS["Cam Setup"]
    for name in ("", "a", "cam0", "Basler acA2440"):
        for buffers in (0, 1, 5, 256, 70000):
            for continuous in (False, True):
                expected = CameraInitData("High Mag.", name, buffers, continuous)
                assert codec.decode(spec, _panel_bytes(name, buffers, continuous)) == expected
    with pytest.raises(ProtocolError):                    # the old order is rejected without the fallback
        codec.decode(spec, _unbundle_bytes("cam0", 5, True))


def test_init_data_fallback_accepts_the_unbundle_order(caplog: pytest.LogCaptureFixture) -> None:
    codec = PayloadCodec(ENUMS, init_data_fallback=True)
    with caplog.at_level(logging.WARNING, logger="aloxcam.wire.messages"):
        got = codec.decode(COMMANDS["Cam Setup"], _unbundle_bytes("cam0", 5, True))
    assert got == CameraInitData("High Mag.", "cam0", 5, True)
    assert "fallback" in caplog.text
    caplog.clear()
    with caplog.at_level(logging.WARNING, logger="aloxcam.wire.messages"):
        codec.decode(COMMANDS["Cam Setup"], _panel_bytes("cam0", 5, True))
    assert caplog.text == ""


def test_init_data_residual_ambiguity_04_7_1() -> None:
    """With the panel order first, an Unbundle-order frame is misread exactly when Continuous Mode is FALSE and
    len(name) == Number of buffers >> 8 (empty name with < 256 buffers, 1-char name with 256..511 …)."""
    codec = PayloadCodec(ENUMS, init_data_fallback=True)
    spec = COMMANDS["Cam Setup"]
    misread = codec.decode(spec, _unbundle_bytes("", 5, False))
    assert misread != CameraInitData("High Mag.", "", 5, False)
    assert (misread.camera_name, misread.continuous_mode, misread.number_of_buffers) == ("", False, 0x05000000)
    assert codec.decode(spec, _unbundle_bytes("a", 300, False)) != CameraInitData("High Mag.", "a", 300, False)
    panel_only = PayloadCodec(ENUMS)
    names = ["", "a", "ab", "cam0", string.ascii_letters[:12]]
    for name in names:
        for buffers in list(range(0, 300, 7)) + [255, 256, 511, 512, 1024, 70000]:
            for continuous in (False, True):
                data = _unbundle_bytes(name, buffers, continuous)
                got = codec.decode(spec, data)
                if not continuous and len(name) == buffers >> 8:          # also fits the panel order: it wins
                    assert got == panel_only.decode(spec, data), (name, buffers, continuous)
                else:                                                      # only the fallback fits: correct
                    assert got == CameraInitData("High Mag.", name, buffers, continuous), (name, buffers, continuous)
                    with pytest.raises(ProtocolError):
                        panel_only.decode(spec, data)


def test_init_data_unbundle_override_misreads_a_one_char_panel_frame() -> None:
    """04 §7.1 first case: with the Unbundle order configured first, a panel-order frame with a 1-character name
    and 1 buffer also fits the Unbundle order."""
    layout = ", ".join(f"{a}:{k}" for a, k in CameraInitData.LAYOUT_UNBUNDLE)
    codec = PayloadCodec(ENUMS, {"camera init data": layout}, init_data_fallback=True)
    spec = COMMANDS["Cam Setup"]
    assert codec.decode(spec, _panel_bytes("x", 1, True)) != CameraInitData("High Mag.", "x", 1, True)
    assert codec.decode(spec, _panel_bytes("cam0", 5, True)) == CameraInitData("High Mag.", "cam0", 5, True)
    assert codec.decode(spec, _unbundle_bytes("cam0", 5, True)) == CameraInitData("High Mag.", "cam0", 5, True)


# ---------------------------------------------------------------------------------------------- decode rules
def test_layout_override_changes_the_decode() -> None:
    data = b"\x00\x01\x00\x01"
    default = PayloadCodec(ENUMS).decode(COMMANDS["Cam Flip"], data)
    assert default == CameraFlipData("Low Mag.", horizontal_flip=False, vertical_flip=True)
    swapped = PayloadCodec(ENUMS, {"  Camera Flip DATA ": "camera_type:enum:camera_type, vertical_flip:bool, "
                                                          "horizontal_flip:bool"})
    assert swapped.decode(COMMANDS["Cam Flip"], data) == CameraFlipData("Low Mag.", horizontal_flip=True,
                                                                        vertical_flip=False)
    assert swapped.encode(CameraFlipData("Low Mag.", True, False)) == data


def test_layout_override_errors() -> None:
    with pytest.raises(ConfigError):
        PayloadCodec(ENUMS, {"No Such Typedef": "a:i32"})
    with pytest.raises(ConfigError):
        PayloadCodec(ENUMS, {"Camera Flip Data": "camera_type:enum:camera_type, vertical_flip:bool"})
    with pytest.raises(ConfigError):
        PayloadCodec(ENUMS, {"Camera Flip Data": "camera_type:enum:nope, vertical_flip:bool, horizontal_flip:bool"})
    with pytest.raises(ConfigError):
        PayloadCodec(ENUMS, templates={"Z Set Min Limit": b"\x18\x00"})


def test_strict_decode_requires_exact_consumption() -> None:
    codec = PayloadCodec(ENUMS)
    with pytest.raises(ProtocolError):
        codec.decode(COMMANDS["Cam Stop"], b"\x00\x01\x00")
    with pytest.raises(ProtocolError):
        codec.decode(COMMANDS["Cam Stop"], b"")


def test_out_of_range_camera_type_is_rejected() -> None:
    codec = PayloadCodec(ENUMS)
    for data in (b"\x00\x02", b"\xff\xff"):
        with pytest.raises(ProtocolError):
            codec.decode(COMMANDS["Cam Stop"], data)
    with pytest.raises(ProtocolError):
        codec.decode(COMMANDS["Cam Set Exposure"], b"\x00\x02" + bytes(8))


def test_lenient_z_reference_method() -> None:
    codec = PayloadCodec(ENUMS)
    assert codec.decode(COMMANDS["Z Do Reference"], b"\x00\x07") == ZReferenceData("#7")
    assert codec.decode(COMMANDS["Z Do Reference"], b"\x00\x01") == ZReferenceData("FRF")
    assert codec.encode(ZReferenceData("#7")) == b"\x00\x07"


def test_not_read_and_unused_accept_any_bytes() -> None:
    codec = PayloadCodec(ENUMS)
    for name in ("Get Heartbeat", "Exit", "Z Setup", "Z Get Speed", "Z Stop", "Cam Get Images Array"):
        assert codec.decode(COMMANDS[name], b"\x01\x02\x03") is None
        assert codec.decode(COMMANDS[name], b"") is None


def test_enum_size_1_override_shifts_payloads() -> None:
    small = PayloadCodec(WireEnums.load(sizes={"camera_type": 1}))
    assert small.encode(CameraStopData("Low Mag.")) == b"\x01"
    assert small.encode(CameraSetExposureData("Low Mag.", 1500.0)) == b"\x01" + sv.h("40 97 70 00 00 00 00 00")
    assert small.decode(COMMANDS["Cam Stop"], b"\x01") == CameraStopData("Low Mag.")
    with pytest.raises(ProtocolError):                    # a U16 payload no longer decodes exactly
        small.decode(COMMANDS["Cam Stop"], b"\x00\x01")
    with pytest.raises(ProtocolError):
        PayloadCodec(ENUMS).decode(COMMANDS["Cam Stop"], b"\x01")
    assert read_variant_header(Reader(small.variant_for(COMMANDS["Cam Stop"], None))).type_descriptors[0][3] == 0x15


# ---------------------------------------------------------------------------------------------- variants
def test_variant_for() -> None:
    codec = PayloadCodec(ENUMS)
    assert codec.variant_for(COMMANDS["Get Heartbeat"], None) == EMPTY_VARIANT_HEADER + bytes(4)
    assert codec.variant_for(COMMANDS["Exit"], CameraStopData()) == EMPTY_VARIANT_HEADER + bytes(4)
    unused = codec.variant_for(COMMANDS["Z Get Speed"], None)
    r = Reader(unused)
    vh = read_variant_header(r)
    assert vh.top_type_code == 0x50 and r.data[r.pos:] == bytes(4) + bytes(4)   # "" + attribute count
    limit = codec.variant_for(COMMANDS["Z Set Min Limit"], ZLimitData(0.5))
    assert limit == sv.h(sv.BARE_DBL_VARIANT_HEADER) + sv.h("3f e0 00 00 00 00 00 00") + bytes(4)
    defaults = codec.variant_for(COMMANDS["Cam Stop"], None)
    assert defaults.endswith(b"\x00\x00" + bytes(4))


def test_variant_templates_are_used() -> None:
    template = "18 00 80 00 00 00 00 01 00 04 00 0a 00 01 00 00"
    codec = PayloadCodec(ENUMS, templates={"Z Set Max Limit": template})
    assert codec.variant_for(COMMANDS["Z Set Max Limit"], ZLimitData(1.0)).startswith(sv.h(template))
    codec2 = PayloadCodec(ENUMS, templates={"Z Setup": sv.h(template)})
    assert codec2.variant_for(COMMANDS["Z Setup"], None) == sv.h(template) + bytes(4)


# ---------------------------------------------------------------------------------------------- mappings
def test_payload_from_mapping_conversions() -> None:
    ff = payload_from_mapping(CameraSetFlatFieldData, {"camera_type": 1, "enable": True, "ffc_path": "C:\\f.png",
                                                       "dfc_path": {"kind": 3, "components": ["srv", "s"]},
                                                       "correction_factor": 2})
    assert ff == CameraSetFlatFieldData("#1", True, LVPath(PATH_ABSOLUTE, ("C", "f.png")),
                                        LVPath(PATH_UNC, ("srv", "s")), 2.0)
    assert isinstance(ff.correction_factor, float)
    assert PayloadCodec(ENUMS).encode(ff)[:2] == b"\x00\x01"
    log = payload_from_mapping(CamLogData, {"path": "/tmp/x.log", "max_lines": 10})
    assert log.path == LVPath(PATH_ABSOLUTE, ("tmp", "x.log")) and log.max_lines == 10 and not log.log
    af = payload_from_mapping(CameraAutofocusData, {"autofocus_method": "Sobel", "roi": [1, 2, 3, 4], "z_min": 1})
    assert af.autofocus_method == "Sobel" and af.roi == [1, 2, 3, 4] and af.z_min == 1.0
    assert payload_from_mapping(ZLimitData, {}) == ZLimitData()
    for bad in ({"nope": 1}, {"camera_type": True}, {"camera_type": 1.5}):
        with pytest.raises(ValueError):
            payload_from_mapping(CameraStopData, bad)
    with pytest.raises(ValueError):
        payload_from_mapping(messages.ZWaitData, {"add_wait_ms": "10"})
    with pytest.raises(ValueError):
        payload_from_mapping(messages.ZGetSerialData, {"type": "yes"})


def test_payload_to_mapping_round_trips() -> None:
    for obj in (CameraSetFlatFieldData("Low Mag.", True, LVPath(PATH_ABSOLUTE, ("C", "f.png")), LVPath(), 1.0),
                CameraAutofocusData(roi=[1, 2, 3, 4]), messages.CameraStreamData(), ZReferenceData("FRF")):
        plain = payload_to_mapping(obj)
        assert payload_from_mapping(type(obj), plain) == obj
    assert payload_to_mapping(CamLogData()) == {"log": False, "path": {"kind": 0, "components": []},
                                                "max_lines": 0, "max_count": 0}


def test_mapping_helpers_for_generic_kinds() -> None:
    @dataclasses.dataclass(frozen=True)
    class Generic:
        a: object = None
        e: object = None
        LV_LAYOUT = (("a", "dbl[][]"), ("e", "err"))

    g = payload_from_mapping(Generic, {"a": [[1, 2]], "e": {"status": True, "code": 5, "source": "s"}})
    assert isinstance(g.a, np.ndarray) and g.a.shape == (1, 2)
    assert g.e == ErrorCluster(True, 5, "s")
    assert payload_to_mapping(g) == {"a": [[1.0, 2.0]], "e": {"status": True, "code": 5, "source": "s"}}
