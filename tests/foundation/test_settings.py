"""aloxcam.settings: defaults equal 09 §5.6 and the 10 §6 compat registry (design §3.1.3)."""

from __future__ import annotations

import dataclasses
from pathlib import Path

import pytest

from aloxcam.settings import (
    CAMERA_TYPES,
    HIGH_MAG,
    LOW_MAG,
    PLUG_GIGE,
    AloxSettings,
    CameraSettings,
    CompatSwitches,
    Folders,
    LightSettings,
    ServerConfig,
    ServerOptions,
    WireOptions,
    ZStageSettings,
)


def test_camera_types() -> None:
    assert CAMERA_TYPES == (HIGH_MAG, LOW_MAG) == ("High Mag.", "Low Mag.")


def test_camera_defaults_09_5_6() -> None:
    hm, lm = AloxSettings().high_mag, AloxSettings().low_mag
    assert (hm.camera_name, hm.invert_led, hm.max_framerate) == ("HighMag", False, 0.0)
    assert (lm.camera_name, lm.invert_led, lm.max_framerate) == ("LowMag", True, 24.6)
    common = {
        "buffers": 5, "timeout": 5000, "flat_field": False, "flat_field_path": None, "dark_current_path": None,
        "flat_field_factor": 1.0, "exposure_attribute": "CameraAttributes::Exposure::Value",
        "gain_attribute": "CameraAttributes::Gain::Value", "exposure_type": "DBL", "gain_type": "I64",
        "horizontal_flip": False, "vertical_flip": False,
        "framerate_attribute": "CameraAttributes::AcquisitionControl::AcquisitionFrameRate",
        "framerate_type": "DBL", "wait_led_ms": 0, "plug_type": PLUG_GIGE, "exposure_lower_limit": 177.0,
    }
    for cam in (hm, lm):
        assert {k: getattr(cam, k) for k in common} == common
    assert CameraSettings.default_high_mag() == hm
    assert CameraSettings.default_low_mag() == lm


def test_general_light_and_z_defaults() -> None:
    alox = AloxSettings()
    assert (alox.port, alox.core, alox.max_lines) == (503, 1, 1000)
    assert alox.light == LightSettings(set_led_cam=0, camera_sort_led="GiGe", connector="HM", hm_led_line=0,
                                       hm_bulb_line=0, led_status_inverter=True, bulb_status_inverter=False)
    assert alox.z == ZStageSettings(com=2, baudrate=115200, axis=1, speed=10.0, slow_speed=1.0, min_pos=0.0,
                                    max_pos=20.0)


@pytest.mark.parametrize(("com", "port"), [(2, "COM2"), (4, "COM4"), (1, "COM1"), (0, "COM1"), (-3, "COM1")])
def test_port_name(com: int, port: str) -> None:
    assert ZStageSettings(com=com).port_name == port


def test_compat_defaults_10_6() -> None:
    assert dataclasses.asdict(CompatSwitches()) == {
        "send_error_replies": False,
        "reply_unknown_commands": False,
        "listen_address": "127.0.0.1",
        "inbound_pause_ms": 0,
        "close_on_extra_heartbeat_data": True,
        "exposure_sets_max_framerate": True,
        "strict_new_frame": False,
        "request_buffer_zero": True,
        "temperature_error_fails_image": True,
        "flatfield_check_on_set": False,
        "focus_reject_concurrent_start": False,
        "focus_validate_params": False,
        "af_check_z_limits": False,
        "restart_dh_macro": True,
        "implicit_z_reinit": True,
    }


def test_server_config_defaults() -> None:
    cfg = ServerConfig()
    assert cfg.server == ServerOptions()
    assert cfg.server.max_payload_bytes == 256 * 2**20 and cfg.server.port is None
    assert cfg.wire == WireOptions() and cfg.wire.encoding == "cp1252" and not cfg.wire.init_data_fallback
    assert cfg.image.codec == "provisional"
    assert cfg.stage.reference_move == "FNL" and cfg.stage.wait_timeout_s == 300.0
    assert cfg.simulation.stage_identity == "(c) aloxcam SimStage, C-663-SIM, 0000000001, 0.0.0"


def test_settings_are_frozen() -> None:
    with pytest.raises(dataclasses.FrozenInstanceError):
        AloxSettings().high_mag.flat_field = True  # type: ignore[misc]
    folders = Folders(exe=Path("/e"), settings=Path("/s"), vision=None, data=None)
    with pytest.raises(dataclasses.FrozenInstanceError):
        folders.vision = Path("/v")  # type: ignore[misc]
