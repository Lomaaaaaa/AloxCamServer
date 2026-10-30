"""aloxcam.config.alox_ini: the 53 alox.ini keys, defaults, derived values and the owner's real files
(09 §5, D2/D4–D7, design §3.3.3 and §4.2)."""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import pytest

from aloxcam.config.alox_ini import ALOX_INI_KEYS, BAUD_BY_INDEX, IniKey, load_alox_ini
from aloxcam.settings import PLUG_GIGE, PLUG_USB, AloxSettings, CameraOptions, CameraSettings, LightSettings
from aloxcam.testing import REFERENCE_ALOX_INI, AloxTree

REAL = Path(__file__).parent / "data" / "real"
REAL_FILES = ["alox.ini", "alox_GiGeIDS.ini", "alox_mx.ini", "alox_usbAllied.ini", "alox_usbIDS.ini"]
HM, LM = "hardware.camera.highmag", "hardware.camera.lowmag"
posix_only = pytest.mark.skipif(sys.platform == "win32", reason="drive-letter mapping off Windows")


@pytest.fixture(autouse=True)
def _no_drive_mapping(monkeypatch: pytest.MonkeyPatch) -> None:
    for letter in "CD":
        monkeypatch.delenv(f"ALOX_DRIVE_{letter}", raising=False)


def camera_rows(section: str, cam: str, low: bool) -> list[tuple[str, str, str, object, str]]:
    rows = [
        ("name_STR", "STR", "LowMag" if low else "HighMag", "camera_name"),
        ("buffers_I32", "I32", 5, "buffers"),
        ("timeout_I32", "I32", 5000, "timeout"),
        ("flatfieldcorrection_BOOL", "BOOL", False, "flat_field"),
        ("flatfieldimage_PATH", "PATH", "", "flat_field_path"),
        ("darkfieldimage_PATH", "PATH", "", "dark_current_path"),
        ("flatfieldfactor_DBL", "DBL", 1, "flat_field_factor"),
        ("exposure_STR", "STR", "CameraAttributes::Exposure::Value", "exposure_attribute"),
        ("gain_STR", "STR", "CameraAttributes::Gain::Value", "gain_attribute"),
        ("exposuretype_STR", "STR", "DBL", "exposure_type"),
        ("gaintype_STR", "STR", "I64", "gain_type"),
        ("hflip_BOOL", "BOOL", False, "horizontal_flip"),
        ("vflip_BOOL", "BOOL", False, "vertical_flip"),
        ("framerate_STR", "STR", "CameraAttributes::AcquisitionControl::AcquisitionFrameRate", "framerate_attribute"),
        ("frameratetype_STR", "STR", "DBL", "framerate_type"),
        *([("frameratemax_DBL", "DBL", 24.6, "max_framerate")] if low else []),
        ("LimitMinExposure_DBL", "DBL", 177, "exposure_lower_limit"),
        ("waitled_U32", "U32", 0, "wait_led_ms"),
        ("invertled_BOOL", "BOOL", low, "invert_led"),
        ("cameraconnectortype_STR", "STR", "gige", "plug_type"),
    ]
    return [(section, key, kind, default, f"{cam}.{field}") for key, kind, default, field in rows]


#: The design §3.3.3 table, written out independently of the implementation.
EXPECTED_TABLE = [
    ("hardware.camera", "port_I32", "I32", 503, "port"),
    ("general", "core_I32", "I32", 1, "core"),
    ("log", "maxlines_I32", "I32", 1000, "max_lines"),
    *camera_rows(HM, "high_mag", False),
    *camera_rows(LM, "low_mag", True),
    ("hardware.z", "comport_I32", "I32", 2, "z.com"),
    ("hardware.z", "baudrate_I32", "I32", 3, "z.baudrate"),
    ("hardware.z", "zaxis_I32", "I32", 1, "z.axis"),
    ("hardware.z", "speed_DBL", "DBL", 10, "z.speed"),
    ("hardware.z", "slowspeed_DBL", "DBL", 1, "z.slow_speed"),
    ("hardware.z", "zmin_DBL", "DBL", 0, "z.min_pos"),
    ("hardware.z", "zmax_DBL", "DBL", 20, "z.max_pos"),
    (HM, "setLEDline_I32", "I32", 0, "light.hm_led_line"),
    (HM, "setBULBline_I32", "I32", 0, "light.hm_bulb_line"),
    ("general", "setLEDcam_I32", "I32", 0, "light.set_led_cam"),
    ("hardware.camera", "lightStatusInverter_BOOL", "BOOL", True, "light.led_status_inverter"),
]


def load(path: Path, options: CameraOptions | None = None) -> tuple[AloxSettings, list[str]]:
    warnings: list[str] = []
    return load_alox_ini(path, options or CameraOptions(), warn=warnings.append), warnings


def write_ini(tmp_path: Path, text: str, name: str = "alox.ini") -> Path:
    p = tmp_path / name
    p.write_bytes(text.replace("\n", "\r\n").encode("cp1252"))
    return p


def with_values(**edits: str) -> str:
    """The reference file with ``"<section>|<key>"`` lines replaced (the key must exist in that section)."""
    lines = REFERENCE_ALOX_INI.splitlines()
    for spec, value in edits.items():
        section, key = spec.split("|")
        current = None
        for i, line in enumerate(lines):
            if line.startswith("["):
                current = line.strip("[]")
            elif current == section and line.split("=")[0].strip() == key:
                lines[i] = f"{key} = {value}"
                break
        else:
            raise KeyError(spec)
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------------------------- schema
def test_alox_ini_keys_equal_the_design_table() -> None:
    assert len(ALOX_INI_KEYS) == 53
    assert [(k.section, k.key, k.kind, k.default, k.target) for k in ALOX_INI_KEYS] == EXPECTED_TABLE
    assert all(isinstance(k, IniKey) for k in ALOX_INI_KEYS)
    for k in ALOX_INI_KEYS:
        assert k.key.rsplit("_", 1)[1] == k.kind                # the suffix is the NI instance
    assert not any(k.section == HM and k.key == "frameratemax_DBL" for k in ALOX_INI_KEYS)   # D5
    assert len({(k.section, k.key.lower()) for k in ALOX_INI_KEYS}) == 53


def test_every_target_names_a_settings_field() -> None:
    alox = AloxSettings()
    for k in ALOX_INI_KEYS:
        obj = alox
        for part in k.target.split("."):
            assert hasattr(obj, part), k.target
            obj = getattr(obj, part)


def test_baud_table() -> None:
    assert dict(BAUD_BY_INDEX) == {0: 9600, 1: 19200, 2: 38400, 3: 115200}


# ---------------------------------------------------------------------------------------------- reference file
def test_reference_file_gives_defaults(tmp_alox_tree: AloxTree) -> None:
    alox, warnings = load(tmp_alox_tree.alox_ini)
    assert alox == AloxSettings()
    assert warnings == []


def test_missing_file_defaults_warning_not_created(tmp_path: Path) -> None:
    p = tmp_path / "settings" / "alox.ini"
    p.parent.mkdir()
    alox, warnings = load(p)
    assert alox == AloxSettings()
    assert len(warnings) == 1 and "not found" in warnings[0]
    assert not p.exists()                                       # 09 D2 fix


def test_empty_file_gives_defaults_silently(tmp_path: Path) -> None:
    alox, warnings = load(write_ini(tmp_path, ""))
    assert alox == AloxSettings() and warnings == []


# ---------------------------------------------------------------------------------------------- per-key rules
@pytest.mark.parametrize(("value", "port", "warned"), [
    ("70000", 503, True), ("-1", 503, True), ("65535", 65535, False), ("0", 0, False), ("1234", 1234, False),
])
def test_port_range(tmp_path: Path, value: str, port: int, warned: bool) -> None:
    alox, warnings = load(write_ini(tmp_path, with_values(**{"hardware.camera|port_I32": value})))
    assert alox.port == port
    assert bool(warnings) is warned
    if warned:
        assert "port_I32" in warnings[0] and value in warnings[0]


def test_invalid_value_falls_back_per_key_without_chain(tmp_path: Path) -> None:
    """D4 fix: an invalid chained key (port_I32, comport_I32, name chain…) does not reset the next keys."""
    text = with_values(**{
        "hardware.camera|port_I32": "abc",
        "general|core_I32": "4",
        "hardware.z|comport_I32": "COM4",
        "hardware.z|baudrate_I32": "2",
        "hardware.z|zaxis_I32": "3",
        f"{HM}|buffers_I32": "many",
        f"{HM}|timeout_I32": "2500",
        f"{HM}|flatfieldcorrection_BOOL": "yes",
        f"{HM}|hflip_BOOL": "1",
        f"{LM}|flatfieldfactor_DBL": "x",
        f"{LM}|LimitMinExposure_DBL": "176,5",
        f"{LM}|waitled_U32": "oops",
    })
    alox, warnings = load(write_ini(tmp_path, text))
    assert alox.port == 503 and alox.core == 4
    assert alox.z.com == 2 and alox.z.baudrate == 38400 and alox.z.axis == 3
    assert alox.high_mag.buffers == 5 and alox.high_mag.timeout == 2500
    assert alox.high_mag.flat_field is False and alox.high_mag.horizontal_flip is True
    assert alox.low_mag.flat_field_factor == 1.0 and alox.low_mag.exposure_lower_limit == 176.5
    assert alox.low_mag.wait_led_ms == 0
    assert len(warnings) == 6
    for key in ("port_I32", "comport_I32", "buffers_I32", "flatfieldcorrection_BOOL", "flatfieldfactor_DBL",
                "waitled_U32"):
        assert sum(f"] {key} = " in w for w in warnings) == 1, key


@pytest.mark.parametrize(("value", "rate", "warned"), [
    ("0", 9600, False), ("1", 19200, False), ("2", 38400, False), ("3", 115200, False),
    ("115200", 9600, True), ("4", 9600, True), ("-1", 9600, True),
])
def test_baud_index(tmp_path: Path, value: str, rate: int, warned: bool) -> None:
    alox, warnings = load(write_ini(tmp_path, with_values(**{"hardware.z|baudrate_I32": value})))
    assert alox.z.baudrate == rate
    assert bool(warnings) is warned                             # D7: reproduce + warn


@pytest.mark.parametrize(("value", "plug"), [
    ('"USB"', PLUG_USB), ("usb", PLUG_USB), ("uSb", PLUG_USB), ("usB", PLUG_USB), ('"Usb"', PLUG_USB),
    ('"gige"', PLUG_GIGE), ('"GiGe"', PLUG_GIGE), ('"GigE"', PLUG_GIGE), ('""', PLUG_GIGE), ('"usb3"', PLUG_GIGE),
])
def test_connector_type_case_insensitive(tmp_path: Path, value: str, plug: str) -> None:
    text = with_values(**{f"{HM}|cameraconnectortype_STR": value, f"{LM}|cameraconnectortype_STR": value})
    alox, warnings = load(write_ini(tmp_path, text))
    assert alox.high_mag.plug_type == alox.low_mag.plug_type == plug   # D6 fix
    assert warnings == []


def test_set_led_cam_selects_low_mag(tmp_path: Path) -> None:
    text = with_values(**{"general|setLEDcam_I32": "2", f"{LM}|cameraconnectortype_STR": "USB",
                          f"{HM}|setLEDline_I32": "4", f"{HM}|setBULBline_I32": "5",
                          "hardware.camera|lightStatusInverter_BOOL": "FALSE"})
    alox, _ = load(write_ini(tmp_path, text), CameraOptions(bulb_status_inverter=True))
    assert alox.light == LightSettings(set_led_cam=2, camera_sort_led=PLUG_USB, connector="LM", hm_led_line=4,
                                       hm_bulb_line=5, led_status_inverter=False, bulb_status_inverter=True)
    alox, _ = load(write_ini(tmp_path, with_values(**{f"{LM}|cameraconnectortype_STR": "USB"})))
    assert alox.light.connector == "HM" and alox.light.camera_sort_led == PLUG_GIGE   # HM's plug type


def test_hm_frameratemax_is_ignored(tmp_path: Path) -> None:
    text = REFERENCE_ALOX_INI.replace("; frameratemax_DBL is not read for High Mag", "frameratemax_DBL = 50")
    text = text.replace("frameratemax_DBL = 24.6", "frameratemax_DBL = 30,5")
    alox, warnings = load(write_ini(tmp_path, text))
    assert alox.high_mag.max_framerate == 0.0 and alox.low_mag.max_framerate == 30.5
    alox, _ = load(write_ini(tmp_path, text), CameraOptions(hm_max_framerate=60.0))
    assert alox.high_mag.max_framerate == 60.0                   # D5: [camera] hm_max_framerate
    assert warnings == []


@posix_only
def test_paths_localized(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    text = with_values(**{f"{HM}|flatfieldimage_PATH": r'"D:\alox\ffc.png"',
                          f"{HM}|darkfieldimage_PATH": "/D/alox/dfc.png",
                          f"{LM}|flatfieldimage_PATH": str(tmp_path / "lm.png")})
    p = write_ini(tmp_path, text)
    alox, warnings = load(p)
    assert alox.high_mag.flat_field_path is None                 # D:\… unmapped: None + warning
    assert alox.high_mag.dark_current_path == Path("/D/alox/dfc.png")
    assert alox.low_mag.flat_field_path == tmp_path / "lm.png"
    assert alox.low_mag.dark_current_path is None
    assert len(warnings) == 1 and "flatfieldimage_PATH" in warnings[0]
    monkeypatch.setenv("ALOX_DRIVE_D", str(tmp_path / "d"))
    alox, warnings = load(p)
    assert alox.high_mag.flat_field_path == tmp_path / "d" / "alox" / "ffc.png"
    assert alox.high_mag.dark_current_path == tmp_path / "d" / "alox" / "dfc.png"
    assert warnings == []


def test_strings_kept_as_written(tmp_path: Path) -> None:
    text = with_values(**{f"{HM}|exposuretype_STR": "i64", f"{HM}|name_STR": '"Cam\\\\1"',
                          f"{LM}|gain_STR": "CameraAttributes::AnalogControl::Gain ; IDS"})
    alox, _ = load(write_ini(tmp_path, text))
    assert alox.high_mag.exposure_type == "i64"                  # case-sensitive, interpreted by the camera code
    assert alox.high_mag.camera_name == "Cam\\1"
    assert alox.low_mag.gain_attribute == "CameraAttributes::AnalogControl::Gain"


# ---------------------------------------------------------------------------------------------- owner files
@pytest.mark.parametrize("name", REAL_FILES)
def test_real_files_load_without_warning(name: str) -> None:
    alox, warnings = load(REAL / name)
    assert warnings == []
    assert (alox.port, alox.core, alox.max_lines) == (503, 1, 1000)
    assert alox.z.baudrate == 115200 and alox.z.axis == 1
    assert alox.z.speed == 5.0 and alox.z.slow_speed == 5.0 and alox.z.min_pos == 0.0
    for cam in (alox.high_mag, alox.low_mag):
        assert cam.exposure_attribute == "CameraAttributes::AcquisitionControl::ExposureTime"
        assert cam.gain_attribute == "CameraAttributes::AnalogControl::Gain"
        assert cam.timeout == 2500 and cam.buffers == 5
        assert cam.exposure_lower_limit == 176.0 and cam.flat_field_factor == 1.0
        assert cam.dark_current_path is None
    assert alox.low_mag.max_framerate == pytest.approx(24.6)    # decimal comma
    assert alox.high_mag.max_framerate == 0.0                   # D5: HM frameratemax_DBL not read


def test_real_alox_ini_values() -> None:
    alox, _ = load(REAL / "alox.ini")
    assert alox.light == LightSettings(set_led_cam=0, camera_sort_led=PLUG_USB, connector="HM", hm_led_line=2,
                                       hm_bulb_line=3, led_status_inverter=False, bulb_status_inverter=False)
    assert alox.z.com == 4 and alox.z.port_name == "COM4" and alox.z.max_pos == 90.0
    assert alox.high_mag.camera_name == "LowMagg" and alox.low_mag.camera_name == "HighMag"   # swapped in the file
    assert alox.high_mag.plug_type == alox.low_mag.plug_type == PLUG_USB
    assert (alox.high_mag.invert_led, alox.low_mag.invert_led) == (False, True)
    assert not alox.high_mag.horizontal_flip and not alox.high_mag.vertical_flip


@posix_only
def test_real_alox_ini_paths(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    alox, _ = load(REAL / "alox.ini")
    assert alox.high_mag.flat_field_path == Path("/C/Users/Admin/Documents/alox/settings/HM200727.png")
    monkeypatch.setenv("ALOX_DRIVE_C", str(tmp_path))
    alox, _ = load(REAL / "alox.ini")
    assert alox.low_mag.flat_field_path == tmp_path / "Users" / "Admin" / "Documents" / "alox" / "settings" / \
        "low_mag_bright.png"


def test_real_gige_ids_values() -> None:
    alox, _ = load(REAL / "alox_GiGeIDS.ini")
    assert alox.high_mag.plug_type == alox.low_mag.plug_type == PLUG_GIGE
    assert alox.light.led_status_inverter is True               # key missing → LabVIEW default TRUE
    assert alox.light.camera_sort_led == PLUG_GIGE and alox.light.hm_led_line == 1
    assert alox.high_mag.camera_name == "cam2" and alox.z.com == 2 and alox.z.max_pos == 80.0


def test_real_mx_selects_low_mag_lights() -> None:
    alox, _ = load(REAL / "alox_mx.ini")
    assert alox.light.set_led_cam == 1 and alox.light.connector == "LM"
    assert alox.high_mag.plug_type == PLUG_USB and alox.low_mag.plug_type == PLUG_GIGE
    assert alox.light.camera_sort_led == PLUG_GIGE              # the Low Mag camera's plug type
    assert alox.high_mag.horizontal_flip and alox.high_mag.vertical_flip


def test_real_files_differ_only_where_expected() -> None:
    usb_ids, _ = load(REAL / "alox_usbIDS.ini")
    usb_allied, _ = load(REAL / "alox_usbAllied.ini")
    assert usb_ids == usb_allied
    expected_hm = replace(CameraSettings.default_high_mag(), camera_name="HighMag", timeout=2500,
                          flat_field_path=usb_ids.high_mag.flat_field_path,
                          exposure_attribute="CameraAttributes::AcquisitionControl::ExposureTime",
                          gain_attribute="CameraAttributes::AnalogControl::Gain", horizontal_flip=True,
                          vertical_flip=True, plug_type=PLUG_USB, exposure_lower_limit=176.0)
    assert usb_ids.high_mag == expected_hm
