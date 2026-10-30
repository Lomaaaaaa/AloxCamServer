"""aloxcam.config.server_ini: alox_camera_server.ini, the [compat] registry and the template
(10 §6, design §3.3.4, §6, Appendix A)."""

from __future__ import annotations

import dataclasses
import logging
import re
import sys
from pathlib import Path

import pytest

from aloxcam.config.niconfig import NIConfig
from aloxcam.config.server_ini import SERVER_INI_NAME, SERVER_INI_TEMPLATE, load_server_config, write_template
from aloxcam.settings import (
    CompatSwitches,
    ServerConfig,
    ServerOptions,
    SimulationOptions,
    StageOptions,
)

DESIGN = Path(__file__).resolve().parents[2] / "docs" / "phase1-design.md"

#: Field → (INI text, expected value) giving a non-default value for each of the 15 [compat] switches (10 §6).
COMPAT_OVERRIDES = {
    "send_error_replies": ("FALSE", False),
    "reply_unknown_commands": ("1", True),
    "listen_address": ("0.0.0.0", "0.0.0.0"),
    "inbound_pause_ms": ("50", 50),
    "close_on_extra_heartbeat_data": ("FALSE", False),
    "exposure_sets_max_framerate": ("false", False),
    "strict_new_frame": ("TRUE", True),
    "request_buffer_zero": ("0", False),
    "temperature_error_fails_image": ("FALSE", False),
    "flatfield_check_on_set": ("TRUE", True),
    "focus_reject_concurrent_start": ("TRUE", True),
    "focus_validate_params": ("TRUE", True),
    "af_check_z_limits": ("TRUE", True),
    "restart_dh_macro": ("FALSE", False),
    "implicit_z_reinit": ("FALSE", False),
}


def load_text(tmp_path: Path, text: str) -> tuple[ServerConfig, list[str]]:
    p = tmp_path / SERVER_INI_NAME
    p.write_bytes(text.replace("\n", "\r\n").encode("cp1252"))
    warnings: list[str] = []
    return load_server_config(p, warn=warnings.append), warnings


def test_template_is_appendix_a_verbatim() -> None:
    text = DESIGN.read_text(encoding="utf-8")
    m = re.search(r"## Appendix A.*?```ini\n(.*?)```", text, re.S)
    assert m is not None
    assert SERVER_INI_TEMPLATE == m.group(1)


def test_template_parses_to_defaults(tmp_path: Path) -> None:
    config, warnings = load_text(tmp_path, SERVER_INI_TEMPLATE)
    assert config == ServerConfig()
    assert warnings == []


def test_template_covers_every_field() -> None:
    cfg = NIConfig.parse(SERVER_INI_TEMPLATE)
    renamed = {("server", "max_payload_bytes"): "max_payload_mb"}
    free = {("wire", "enum_items"), ("wire", "enum_sizes"), ("wire", "layouts")}
    for section in ("server", "compat", "wire", "logging", "image", "camera", "stage", "simulation"):
        for f in dataclasses.fields(getattr(ServerConfig(), section)):
            if (section, f.name) in free:
                continue
            key = renamed.get((section, f.name), f.name)
            assert cfg.raw(section, key) is not None, f"[{section}] {key}"
    assert {"wire.items", "wire.size", "wire.layout"} <= set(cfg.sections())


def test_missing_file_gives_defaults(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    warnings: list[str] = []
    with caplog.at_level(logging.INFO, logger="aloxcam"):
        config = load_server_config(tmp_path / SERVER_INI_NAME, warn=warnings.append)
    assert config == ServerConfig() and warnings == []
    assert any(r.levelno == logging.INFO and "not found" in r.getMessage() for r in caplog.records)
    assert not (tmp_path / SERVER_INI_NAME).exists()


def test_empty_file_gives_defaults(tmp_path: Path) -> None:
    assert load_text(tmp_path, "") == (ServerConfig(), [])


# ---------------------------------------------------------------------------------------------- [compat]
def test_compat_registry_has_15_switches() -> None:
    assert [f.name for f in dataclasses.fields(CompatSwitches)] == list(COMPAT_OVERRIDES)


@pytest.mark.parametrize("field", list(COMPAT_OVERRIDES))
def test_each_compat_key_overrides_its_field(tmp_path: Path, field: str) -> None:
    text, value = COMPAT_OVERRIDES[field]
    assert getattr(CompatSwitches(), field) != value
    config, warnings = load_text(tmp_path, f"[compat]\n{field} = {text}\n")
    assert config.compat == dataclasses.replace(CompatSwitches(), **{field: value})
    assert warnings == []
    assert dataclasses.replace(config, compat=CompatSwitches()) == ServerConfig()


def test_compat_case_insensitive_names(tmp_path: Path) -> None:
    config, warnings = load_text(tmp_path, "[COMPAT]\nSend_Error_Replies = false\n")
    assert config.compat.send_error_replies is False and warnings == []


# ---------------------------------------------------------------------------------------------- [server] and others
def test_server_section(tmp_path: Path) -> None:
    config, warnings = load_text(tmp_path, "[server]\nport = 5003\nio_timeout_s = 2,5\nmax_payload_mb = 1\n"
                                           "stream_poll_ms = 0\ntick_source = Windows\nshutdown_timeout_s = 3\n"
                                           "commands_history = 10\nerror_history = 20\ntemperature_history = 30\n")
    assert config.server == ServerOptions(port=5003, io_timeout_s=2.5, max_payload_bytes=2**20, stream_poll_ms=0.0,
                                          tick_source="windows", shutdown_timeout_s=3.0, commands_history=10,
                                          error_history=20, temperature_history=30)
    assert warnings == []


@pytest.mark.parametrize(("text", "port"), [("", None), ("0", 0), ("65535", 65535), ('""', None)])
def test_server_port(tmp_path: Path, text: str, port: int | None) -> None:
    config, warnings = load_text(tmp_path, f"[server]\nport = {text}\n")
    assert config.server.port == port and warnings == []


def test_max_payload_fraction(tmp_path: Path) -> None:
    config, _ = load_text(tmp_path, "[server]\nmax_payload_mb = 0.5\n")
    assert config.server.max_payload_bytes == 2**19


def test_logging_image_camera_stage_simulation(tmp_path: Path) -> None:
    config, warnings = load_text(tmp_path, """\
[logging]
append_errors = FALSE
fix_info_labels = FALSE
decimal_separator = ","
create_log_dirs = FALSE
encoding = utf-8
diag_level = debug
diag_file_stem = diag
[image]
codec = NIVision
nivision_library = C:\\Program Files\\National Instruments\\nivision.dll
[camera]
hm_max_framerate = 24,6
bulb_status_inverter = TRUE
[stage]
always_reference = TRUE
reference_move = fpl
ready_timeout_s = 10
wait_timeout_s = 20
do_reference_waits_ready = FALSE
startup_macro_check = TRUE
[simulation]
enabled = TRUE
camera_names = cam1, cam2
image_width = 2448
image_height = 2048
pixel_type = u16
frame_rate = 50
max_frame_rate = 60
temperature_c = -5
stage_min_mm = -1
stage_max_mm = 90
stage_initially_referenced = TRUE
stage_reference_s = 0.05
stage_identity = PI C-663 ; real one
stage_dh_macro_s = 1,5
""")
    assert warnings == []
    log_ = config.logging
    assert (log_.append_errors, log_.fix_info_labels, log_.decimal_separator, log_.create_log_dirs) == \
        (False, False, ",", False)
    assert (log_.encoding, log_.diag_level, log_.diag_file_stem) == ("utf-8", "DEBUG", "diag")
    assert config.image.codec == "nivision"
    assert config.image.nivision_library == r"C:\Program Files\National Instruments\nivision.dll"
    assert config.camera.hm_max_framerate == 24.6 and config.camera.bulb_status_inverter is True
    assert config.stage == StageOptions(always_reference=True, reference_move="FPL", ready_timeout_s=10.0,
                                        wait_timeout_s=20.0, do_reference_waits_ready=False, startup_macro_check=True)
    assert config.simulation == SimulationOptions(
        enabled=True, camera_names=("cam1", "cam2"), image_width=2448, image_height=2048, pixel_type="U16",
        frame_rate=50.0, max_frame_rate=60.0, temperature_c=-5.0, stage_min_mm=-1.0, stage_max_mm=90.0,
        stage_initially_referenced=True, stage_reference_s=0.05, stage_identity="PI C-663", stage_dh_macro_s=1.5)


def test_stage_port_map(tmp_path: Path) -> None:
    config, warnings = load_text(tmp_path, "[stage]\nport_map = COM2=/dev/ttyUSB0, com3 = /dev/ttyS2,"
                                           " COM10=\\\\.\\COM10, broken, COM2=/dev/other\n")
    assert dict(config.stage.port_map) == {"COM2": "/dev/ttyUSB0", "COM3": "/dev/ttyS2", "COM10": r"\\.\COM10"}
    assert len(warnings) == 1 and "broken" in warnings[0]


# ---------------------------------------------------------------------------------------------- [wire]
def test_wire_tables(tmp_path: Path) -> None:
    config, warnings = load_text(tmp_path, """\
[wire]
encoding = cp1252
init_data_fallback = TRUE
[wire.items]
 Camera_Command  = Get Heartbeat, Cam Setup ,Cam Start
z_reference_method = FNL, FRF,
empty =
[wire.size]
CAMERA_COMMAND = 2
tcp_datatype = 1 ; byte
bad = x
[wire.layout]
Camera Init Data = camera_type:enum:camera_type, camera_name:str, number_of_buffers:i32, continuous_mode:bool
""")
    wire = config.wire
    assert wire.init_data_fallback is True and wire.encoding == "cp1252" and wire.enums_file is None
    assert dict(wire.enum_items) == {"camera_command": ("Get Heartbeat", "Cam Setup", "Cam Start"),
                                     "z_reference_method": ("FNL", "FRF")}
    assert dict(wire.enum_sizes) == {"camera_command": 2, "tcp_datatype": 1}
    assert dict(wire.layouts) == {"camera init data": "camera_type:enum:camera_type, camera_name:str, "
                                                      "number_of_buffers:i32, continuous_mode:bool"}
    assert len(warnings) == 2
    assert any("empty" in w for w in warnings) and any("bad" in w for w in warnings)


@pytest.mark.skipif(sys.platform == "win32", reason="POSIX path rules")
def test_wire_enums_file(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("ALOX_DRIVE_D", raising=False)
    config, warnings = load_text(tmp_path, "[wire]\nenums_file = my_enums.toml\n")
    assert config.wire.enums_file == Path("my_enums.toml") and warnings == []   # resolved later against <settings>
    config, warnings = load_text(tmp_path, f'[wire]\nenums_file = "{tmp_path / "e.toml"}"\n')
    assert config.wire.enums_file == tmp_path / "e.toml" and warnings == []
    config, warnings = load_text(tmp_path, "[wire]\nenums_file = D:\\alox\\e.toml\n")
    assert config.wire.enums_file is None and len(warnings) == 1


# ---------------------------------------------------------------------------------------------- invalid / unknown
@pytest.mark.parametrize(("section", "key", "text", "attr"), [
    ("server", "port", "70000", "port"),
    ("server", "port", "abc", "port"),
    ("server", "io_timeout_s", "0", "io_timeout_s"),
    ("server", "io_timeout_s", "nan", "io_timeout_s"),
    ("server", "max_payload_mb", "-1", "max_payload_bytes"),
    ("server", "tick_source", "wallclock", "tick_source"),
    ("server", "commands_history", "-5", "commands_history"),
    ("compat", "send_error_replies", "yes", "send_error_replies"),
    ("compat", "listen_address", '""', "listen_address"),
    ("compat", "inbound_pause_ms", "-1", "inbound_pause_ms"),
    ("logging", "decimal_separator", "..", "decimal_separator"),
    ("logging", "encoding", "klingon", "encoding"),
    ("logging", "diag_level", "LOUD", "diag_level"),
    ("image", "codec", "jpeg", "codec"),
    ("stage", "reference_move", "FRF", "reference_move"),
    ("stage", "wait_timeout_s", "0", "wait_timeout_s"),
    ("simulation", "pixel_type", "RGB", "pixel_type"),
    ("simulation", "image_width", "0", "image_width"),
    ("simulation", "frame_rate", "inf", "frame_rate"),
])
def test_invalid_value_gives_default_with_warning(tmp_path: Path, section: str, key: str, text: str,
                                                  attr: str) -> None:
    config, warnings = load_text(tmp_path, f"[{section}]\n{key} = {text}\n")
    assert config == ServerConfig()
    assert len(warnings) == 1 and f"[{section}] {key}" in warnings[0] and "default" in warnings[0]


def test_invalid_value_does_not_affect_other_keys(tmp_path: Path) -> None:
    config, warnings = load_text(tmp_path, "[compat]\nsend_error_replies = maybe\nstrict_new_frame = TRUE\n")
    assert config.compat.send_error_replies is True and config.compat.strict_new_frame is True
    assert len(warnings) == 1


def test_unknown_sections_and_keys_warned(tmp_path: Path) -> None:
    config, warnings = load_text(tmp_path, "[compat]\nsend_error_replys = TRUE\n[serveur]\nport = 1\n"
                                           "[server]\nport = 9\n# note = x\n")
    assert config.server.port == 9 and config.compat == CompatSwitches()
    assert len(warnings) == 3
    assert any("send_error_replys" in w for w in warnings)
    assert any("[serveur]" in w for w in warnings)
    assert any("# note" in w for w in warnings)


def test_unreadable_file_warns(tmp_path: Path) -> None:
    (tmp_path / SERVER_INI_NAME).mkdir()
    warnings: list[str] = []
    assert load_server_config(tmp_path / SERVER_INI_NAME, warn=warnings.append) == ServerConfig()
    assert len(warnings) == 1


# ---------------------------------------------------------------------------------------------- write_template
def test_write_template(tmp_path: Path) -> None:
    p = tmp_path / SERVER_INI_NAME
    write_template(p)
    data = p.read_bytes()
    assert data == SERVER_INI_TEMPLATE.replace("\n", "\r\n").encode("ascii")
    warnings: list[str] = []
    assert load_server_config(p, warn=warnings.append) == ServerConfig() and warnings == []
    with pytest.raises(FileExistsError):
        write_template(p)
    assert p.read_bytes() == data
