"""aloxcam.wire.enums: packaged tables, overrides, validation and routing (03 §3, 04 §6, design §3.2.4)."""

from __future__ import annotations

from pathlib import Path

import pytest

from aloxcam.dispatch import CAMERA_WORKER, Z_WORKER
from aloxcam.errors import ConfigError, ProtocolError
from aloxcam.settings import WireOptions
from aloxcam.wire.enums import (
    CAMERA_EXTRA,
    CAMERA_RANGE,
    ENUM_IDS,
    Z_RANGE,
    RoutingTable,
    WireEnums,
    build_routing,
    routing_self_check,
)
from aloxcam.wire.messages import CASES
from golden import spec_vectors as sv

#: owner-answers §1, Camera Command (U16, 46 items), code → item.
OWNER_CAMERA_COMMAND = {
    0: "Get Heartbeat", 1: "Cam Setup", 2: "Cam Start", 3: "Cam Get Image", 4: "Cam Stream Next Image",
    5: "Cam Get Acquisition Status", 6: "Cam Software Trigger", 7: "Cam Set Exposure", 8: "Cam Get Exposure",
    9: "Cam Set Gain", 10: "Cam Get Gain", 11: "Cam Get Images Array", 12: "Cam Stop", 13: "Cam Cleanup", 14: "Exit",
    15: "Show UI", 16: "Cam Autofocus", 17: "Cam Set FlatField", 18: "Cam Refocus", 19: "Cam Flip",
    20: "Cam Get Autofocus Data", 21: "Cam Get Autofocus Progress", 22: "Cam Autofocus Run", 23: "Cam Refocus Run",
    24: "Cam Set Led", 25: "Cam Get Led", 26: "Z Setup", 27: "Z Get Serial", 28: "Z Get Status", 29: "Z Get Error",
    30: "Z Set Speed", 31: "Z Get Speed", 32: "Z Get Position", 33: "Z Move Absolute", 34: "Z Move Relative",
    35: "Z Wait End Move", 36: "Z Stop", 37: "Z Enable Joystick", 38: "Z Disable Joystick",
    39: "Z Joystick Enabled ?", 40: "Z Set Min Limit", 41: "Z Set Max Limit", 42: "Z Cleanup", 43: "Z Get Reference",
    44: "Z Do Reference", 45: "Log",
}
CAMERA_COMMANDS = tuple(OWNER_CAMERA_COMMAND[i] for i in range(46))


@pytest.fixture(scope="module")
def enums() -> WireEnums:
    return WireEnums.load()


def test_packaged_table_loads(enums: WireEnums) -> None:
    assert tuple(enums.tables) == ENUM_IDS
    assert enums.provisional_ids == ("z_reference_method",)
    for t in enums.tables.values():
        assert t.size == 2 and t.source == "packaged"
    assert enums.table("z_reference_method").lenient and not enums.table("camera_type").lenient


def test_camera_command_codes_equal_owner_answers(enums: WireEnums) -> None:
    table = enums.table("camera_command")
    assert dict(enumerate(table.items)) == OWNER_CAMERA_COMMAND
    for code, label in OWNER_CAMERA_COMMAND.items():
        assert enums.code("camera_command", label) == code
        assert enums.label("camera_command", code) == label
    key = ("Get Heartbeat", "Exit", "Cam Autofocus Run", "Z Setup", "Z Do Reference", "Log")
    assert [enums.code("camera_command", label) for label in key] == [0, 14, 22, 26, 44, 45]


def test_other_tables(enums: WireEnums) -> None:
    assert enums.table("tcp_datatype").items == ("Command", "Response", "Image", "Heartbeat", "Statut")
    assert enums.code("tcp_datatype", "Statut") == 4
    assert enums.table("camera_type").items == ("High Mag.", "Low Mag.")
    assert enums.table("autofocus_speed").items == ("Slow", "Fast")
    assert enums.table("autofocus_method").items == ("FFT", "Sobel", "Prewitt", "Diff", "Gradient", "Sigma", "FULL")
    assert enums.table("refocus_method").items == ("Fin", "Oxyd")
    assert enums.table("z_reference_method").items == ("FNL", "FRF")


def test_label_and_code_edge_cases(enums: WireEnums) -> None:
    assert enums.code("camera_type", "#5") == 5
    with pytest.raises(ValueError):
        enums.code("camera_type", "Mid Mag.")
    with pytest.raises(ValueError):
        enums.code("camera_type", "#x")
    with pytest.raises(ProtocolError):
        enums.label("camera_type", 2)
    assert enums.label("z_reference_method", 7) == "#7"
    with pytest.raises(KeyError):
        enums.table("nope")


def test_routing_of_the_real_table(enums: WireEnums) -> None:
    routing = build_routing(enums)
    camera = [routing.name(c) for c, w in routing.routes.items() if w == CAMERA_WORKER]
    z = [routing.name(c) for c, w in routing.routes.items() if w == Z_WORKER]
    assert len(camera) == 25 and "Log" in camera and len(z) == 19
    assert len([n for n in camera if n != "Log"]) == 24
    assert routing.route(22) is None and routing.route(23) is None     # the two "… Run": inside the range, unrouted
    assert routing.route(14) == CAMERA_WORKER and routing.route(44) == Z_WORKER and routing.route(45) == CAMERA_WORKER
    assert routing.route(46) is None and routing.name(46) == "#46" and routing.name(32767) == "#32767"
    assert routing.names[22] == "Cam Autofocus Run"
    assert routing_self_check(routing, CASES) == []
    unrouted = build_routing(enums, non_routable=())
    assert unrouted.route(22) == CAMERA_WORKER


def test_synthetic_order_03_3_5_reports_11_unreachable() -> None:
    enums = WireEnums.load(items={"camera_command": sv.SYNTHETIC_ORDER_03_3_5})
    problems = routing_self_check(build_routing(enums), CASES)
    assert problems == [f"{name}: case unreachable (routes to nowhere)" for name in CASES
                        if name in sv.SYNTHETIC_UNREACHABLE]
    assert len(problems) == 11
    assert {p.split(":")[0] for p in problems} == set(sv.SYNTHETIC_UNREACHABLE)


def test_self_check_names_missing_and_misrouted() -> None:
    routing = RoutingTable(names={0: "A", 1: "B"}, routes={0: CAMERA_WORKER})
    assert routing_self_check(routing, {"A": Z_WORKER, "B": CAMERA_WORKER, "C": Z_WORKER}) == [
        "A: case unreachable (routes to camera)",
        "B: case unreachable (routes to nowhere)",
        "C: case unreachable (not in the camera_command enum table)",
    ]


def test_override_file_and_inline_precedence(tmp_path: Path) -> None:
    override = tmp_path / "enums.toml"
    override.write_text(
        'schema = 1\n[enums.camera_type]\nitems = ["Low Mag.", "High Mag."]\n'
        '[enums.z_reference_method]\nitems = ["FNL", "FPL", "FRF"]\nlenient = true\n', encoding="utf-8")
    e = WireEnums.load(override_file=override)
    assert e.code("camera_type", "High Mag.") == 1
    t = e.table("camera_type")
    assert t.source == str(override) and t.label == "Camera Type" and t.size == 2 and not t.provisional
    assert e.provisional_ids == ()                                # override tables default to provisional = false
    assert e.code("z_reference_method", "FRF") == 2
    inline = WireEnums.load(override_file=override, items={"Camera_Type": ["High Mag.", "Low Mag."]},
                            sizes={"camera_type": 1})
    assert inline.code("camera_type", "High Mag.") == 0 and inline.size("camera_type") == 1
    assert inline.table("camera_type").source == "[wire.size]"
    sized = WireEnums.load(sizes={"z_reference_method": 4})
    assert sized.size("z_reference_method") == 4 and sized.provisional_ids == ()


def test_from_options_resolves_relative_file(tmp_path: Path) -> None:
    (tmp_path / "my_enums.toml").write_text('[enums.refocus_method]\nitems = ["Oxyd", "Fin"]\n', encoding="utf-8")
    e = WireEnums.from_options(WireOptions(enums_file=Path("my_enums.toml"), enum_sizes={"autofocus_speed": 4}),
                               tmp_path)
    assert e.code("refocus_method", "Fin") == 1 and e.size("autofocus_speed") == 4
    assert WireEnums.from_options(WireOptions(), tmp_path).table("refocus_method").source == "packaged"


def test_to_toml_round_trips(tmp_path: Path, enums: WireEnums) -> None:
    path = tmp_path / "out.toml"
    path.write_text(enums.to_toml(), encoding="utf-8")
    again = WireEnums.load(override_file=path)
    for enum_id in ENUM_IDS:
        a, b = enums.table(enum_id), again.table(enum_id)
        assert (a.label, a.ctl, a.size, a.items, a.provisional, a.lenient) == \
               (b.label, b.ctl, b.size, b.items, b.provisional, b.lenient)


def _without(items: tuple[str, ...], label: str) -> list[str]:
    return [i for i in items if i != label]


@pytest.mark.parametrize(
    ("items", "sizes"),
    [
        ({"bogus": ["a"]}, {}),                                               # unknown enum id
        ({}, {"bogus": 2}),
        ({}, {"camera_type": 3}),                                             # size not in 1, 2, 4
        ({}, {"tcp_datatype": 4}),                                            # tcp_datatype must be 2
        ({"camera_type": ["High Mag.", "Low Mag.", "High Mag."]}, {}),        # duplicate labels
        ({"tcp_datatype": ["Command", "Response", "Image"]}, {}),             # lacks Heartbeat
        ({"camera_type": ["High Mag."]}, {}),                                 # lacks Low Mag.
        ({"camera_command": _without(CAMERA_COMMANDS, "Log")}, {}),           # lacks Log
        ({"camera_command": _without(CAMERA_COMMANDS, "Z Do Reference")}, {}),
        ({"camera_command": ["Cam Get Led", "Get Heartbeat", "Z Setup", "Z Do Reference", "Log"]}, {}),  # order
        ({"camera_command": ["Get Heartbeat", "Cam Get Led", "Z Do Reference", "Z Setup", "Log"]}, {}),
        ({"camera_command": ["Get Heartbeat", "Z Setup", "Cam Get Led", "Z Do Reference", "Log"]}, {}),  # overlap
        ({"camera_command": ["Get Heartbeat", "Cam Get Led", "Z Setup", "Log", "Z Do Reference"]}, {}),  # Log in Z
        ({"refocus_method": []}, {}),                                         # no items
        ({"refocus_method": ["#1", "Fin"]}, {}),                              # reserved '#'
        ({"refocus_method": [f"m{i}" for i in range(300)]}, {"refocus_method": 1}),   # does not fit
        ({"refocus_method": "Fin, Oxyd"}, {}),                                # a string, not a sequence
    ],
)
def test_validation_errors(items: dict, sizes: dict) -> None:  # type: ignore[type-arg]
    with pytest.raises(ConfigError):
        WireEnums.load(items=items, sizes=sizes)


@pytest.mark.parametrize(
    "text",
    [
        "[enums.bogus]\nitems = ['a']\n",
        "schema = 2\n",
        "[enums.camera_type]\nlabel = 'x'\n",                        # no items
        "[enums.camera_type]\nitems = ['High Mag.', 'Low Mag.']\nsize = 'two'\n",
        "[enums.camera_type]\nitems = ['High Mag.', 'Low Mag.']\nprovisional = 'yes'\n",
        "[enums.camera_type]\nitems = ['High Mag.', 'Low Mag.']\nitmes = []\n",
        "[enums.camera_type]\nitems = ['High Mag.', 'Low Mag.']\nsize = 3\n",
        "[enums\n",                                                    # TOML syntax error
        "extra = 1\n",
    ],
)
def test_override_file_errors(tmp_path: Path, text: str) -> None:
    path = tmp_path / "bad.toml"
    path.write_text(text, encoding="utf-8")
    with pytest.raises(ConfigError):
        WireEnums.load(override_file=path)


def test_missing_override_file(tmp_path: Path) -> None:
    with pytest.raises(ConfigError):
        WireEnums.load(override_file=tmp_path / "missing.toml")


def test_constants() -> None:
    assert CAMERA_RANGE == ("Get Heartbeat", "Cam Get Led") and CAMERA_EXTRA == ("Log",)
    assert Z_RANGE == ("Z Setup", "Z Do Reference")
