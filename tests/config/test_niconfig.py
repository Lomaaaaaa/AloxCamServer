"""aloxcam.config.niconfig: NI config.llb parsing and Read Key conversions (09 §6.1–§6.2, design §3.3.1)."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from aloxcam.config.niconfig import ConfigTypeInvalid, NIConfig

REAL = Path(__file__).parent / "data" / "real"


def cfg_of(body: str, section: str = "s") -> NIConfig:
    return NIConfig.parse(f"[{section}]\n{body}\n")


# ---------------------------------------------------------------------------------------------- parsing (09 §6.1)
def test_case_insensitive_sections_and_keys() -> None:
    cfg = NIConfig.parse("[Hardware.Camera]\nPort_I32 = 503\n")
    assert cfg.raw("hardware.camera", "port_i32") == "503"
    assert cfg.raw("HARDWARE.CAMERA", "PORT_I32") == "503"
    assert cfg.read_int("hardware.camera", "  port_I32\t", 0) == 503      # the requested key is trimmed


def test_first_occurrence_wins_for_keys_and_sections() -> None:
    cfg = NIConfig.parse("[a]\nk = 1\nK = 2\n[b]\nx = 1\n[A]\nk = 3\nother = 4\n")
    assert cfg.read_int("a", "k", 0) == 1
    assert cfg.raw("a", "other") is None                # a duplicated section hides the second copy completely
    assert cfg.sections() == ["a", "b"]
    assert cfg.keys("a") == ["k"]


def test_lines_before_the_first_section_are_not_searchable() -> None:
    cfg = NIConfig.parse("k = 1\n[s]\n")
    assert cfg.raw("", "k") is None and cfg.raw("s", "k") is None
    assert cfg.sections() == ["s"]


def test_missing_section_or_key_gives_default_without_error() -> None:
    cfg = cfg_of("k = 1")
    assert cfg.raw("nope", "k") is None and cfg.raw("s", "nope") is None
    assert cfg.read_int("nope", "k", 7) == 7
    assert cfg.read_float("s", "nope", 2.5) == 2.5
    assert cfg.read_bool("s", "nope", True) is True
    assert cfg.read_str("s", "nope", "d") == "d"
    assert cfg.read_path("s", "nope", r"D:\x") == r"D:\x"
    assert cfg.read_list("s", "nope", ["a"]) == ("a",)
    assert cfg.keys("nope") == []


def test_semicolon_comment_lines_and_inline_comments() -> None:
    cfg = cfg_of("; k = 1\n  ;x=2\nk = 5 ; five\nname = \"HighMag\" ; note")
    assert cfg.keys("s") == ["k", "name"]
    assert cfg.raw("s", "k") == "5"
    assert cfg.raw("s", "name") == '"HighMag"'          # not wholly quoted: cut at ';'
    assert cfg.read_str("s", "name", "") == "HighMag"


def test_quoted_value_keeps_semicolon() -> None:
    cfg = cfg_of("a = \"x;y\"   \nb = 'p;q'\nc = \"x;y\" z")
    assert cfg.raw("s", "a") == '"x;y"'
    assert cfg.read_str("s", "a", "") == "x;y"
    assert cfg.read_str("s", "b", "") == "p;q"
    assert cfg.raw("s", "c") == '"x'                    # does not end with the quote: comment cut


def test_hash_line_with_equals_is_a_key() -> None:
    cfg = cfg_of("# note = 1\n# plain comment\n#k=2")
    assert cfg.keys("s") == ["# note", "#k"]
    assert cfg.read_int("s", "# note", 0) == 1


def test_key_value_split_at_first_equals_and_trimmed() -> None:
    cfg = cfg_of("\t key \t=  a = b  \n=novalue\n   \nlonely")
    assert cfg.raw("s", "key") == "a = b"
    assert cfg.keys("s") == ["key"]


@pytest.mark.parametrize("eol", ["\r\n", "\n", "\r"])
def test_line_endings(eol: str) -> None:
    cfg = NIConfig.parse(eol.join(["[a]", "x = 1", "", "[b]", "y = 2"]))
    assert cfg.read_int("a", "x", 0) == 1 and cfg.read_int("b", "y", 0) == 2


def test_section_header_with_trailing_text() -> None:
    cfg = NIConfig.parse("  [sec] ; comment\nk = 1\n")
    assert cfg.sections() == ["sec"] and cfg.read_int("sec", "k", 0) == 1


def test_bom_tolerated(tmp_path: Path) -> None:
    p = tmp_path / "bom.ini"
    p.write_bytes(b"\xef\xbb\xbf[folders]\r\nsettings = D:\\alox\\x\r\n")
    assert NIConfig.load(p).read_path("folders", "settings", "") == r"D:\alox\x"
    assert NIConfig.parse("\ufeff[a]\nk = 1").read_int("a", "k", 0) == 1


def test_load_cp1252_and_missing_file(tmp_path: Path) -> None:
    p = tmp_path / "x.ini"
    p.write_bytes("[s]\nname = \"Caméra €\"\n".encode("cp1252"))
    assert NIConfig.load(p).read_str("s", "name", "") == "Caméra €"
    with pytest.raises(FileNotFoundError):
        NIConfig.load(tmp_path / "missing.ini")
    assert not (tmp_path / "missing.ini").exists()      # never created (09 D2)


# ---------------------------------------------------------------------------------------------- Read Key (09 §6.2)
@pytest.mark.parametrize(("text", "value"), [
    ("503", 503), ("  -12", -12), ("+7", 7), ("12abc", 12), ("0x10", 0), ("3.7", 3), ("3,7", 3), ("\"42\"", 42),
    ("'5'", 5), ("1    ", 1), ("99999999999", 2**31 - 1), ("-99999999999", -(2**31)),
])
def test_read_int(text: str, value: int) -> None:
    assert cfg_of(f"k = {text}").read_int("s", "k", -1) == value


def test_read_int_unsigned_saturates() -> None:
    cfg = cfg_of("a = -5\nb = 4294967296\nc = 4000000000")
    assert cfg.read_int("s", "a", 9, unsigned=True) == 0
    assert cfg.read_int("s", "b", 9, unsigned=True) == 2**32 - 1
    assert cfg.read_int("s", "c", 9, unsigned=True) == 4_000_000_000


@pytest.mark.parametrize("text", ["abc", "", "x12", "\"\"", "- 1"])
def test_read_int_invalid(text: str) -> None:
    with pytest.raises(ConfigTypeInvalid):
        cfg_of(f"k = {text}").read_int("s", "k", 0)


@pytest.mark.parametrize(("text", "value"), [
    ("24.6", 24.6), ("24,6", 24.6), ("76,200000    ", 76.2), ("-42,753100", -42.7531), ("1", 1.0), (".5", 0.5),
    (",5", 0.5), ("1e3", 1000.0), ("2,5E-1", 0.25), ("3630398355,263576", 3630398355.263576), ("10abc", 10.0),
    ("1.000,5", 1.0), ("1,000.5", 1.0), ("\"3,5\"", 3.5), ("inf", math.inf), ("-Inf", -math.inf),
])
def test_read_float_decimal_point_and_comma(text: str, value: float) -> None:
    assert cfg_of(f"k = {text}").read_float("s", "k", -1.0) == pytest.approx(value)


def test_read_float_nan_and_invalid() -> None:
    assert math.isnan(cfg_of("k = NaN").read_float("s", "k", 0.0))
    for text in ("abc", "", ",", "."):
        with pytest.raises(ConfigTypeInvalid):
            cfg_of(f"k = {text}").read_float("s", "k", 0.0)


@pytest.mark.parametrize(("text", "value"), [
    ("0", False), ("1", True), ("true", True), ("TRUE    ", True), ("FALSE", False), ("False", False),
    ("\"TRUE\"", True), ("true ; x", True),
])
def test_read_bool(text: str, value: bool) -> None:
    assert cfg_of(f"k = {text}").read_bool("s", "k", not value) is value


@pytest.mark.parametrize("text", ["yes", "on", "2", "01", "", "T"])
def test_read_bool_invalid(text: str) -> None:
    with pytest.raises(ConfigTypeInvalid):
        cfg_of(f"k = {text}").read_bool("s", "k", False)


def test_read_str_quotes_and_unescape() -> None:
    cfg = cfg_of(r'a = "C:\\dir\\x"' "\n" r"b = line\0Aend" "\n" r"c = keep\x\n\0a" "\n" r"d = \1F\20" "\n"
                 "e = HighMag    \nf = \"\"\ng = \"unterminated\nh = '  spaced  '")
    assert cfg.read_str("s", "a", "") == "C:\\dir\\x"
    assert cfg.read_str("s", "b", "") == "line\nend"
    assert cfg.read_str("s", "c", "") == r"keep\x\n\0a"        # lower-case hex and other escapes kept
    assert cfg.read_str("s", "d", "") == "\x1f\\20"            # only [01][0-9A-F]
    assert cfg.read_str("s", "e", "") == "HighMag"
    assert cfg.read_str("s", "f", "x") == ""
    assert cfg.read_str("s", "g", "") == '"unterminated'
    assert cfg.read_str("s", "h", "") == "  spaced  "


def test_read_path_not_unescaped_and_not_localized() -> None:
    cfg = cfg_of(r'a = "D:\\alox\10\settings"' "\n" r"b = D:\alox\settings" "\n" 'c = "/D/ALOX/profiles"\n'
                 'd = ""\ne =\n' r"f = \\server\share\x")
    assert cfg.read_path("s", "a", "") == r"D:\\alox\10\settings"
    assert cfg.read_path("s", "b", "") == r"D:\alox\settings"
    assert cfg.read_path("s", "c", "") == "/D/ALOX/profiles"
    assert cfg.read_path("s", "d", "default") == ""            # empty path, not the default
    assert cfg.read_path("s", "e", "default") == ""
    assert cfg.read_path("s", "f", "") == r"\\server\share\x"


def test_read_list() -> None:
    cfg = cfg_of("a = Get Heartbeat, Cam Setup ,Cam Start,\nb =\nc = \"x, y\"\nd = one")
    assert cfg.read_list("s", "a") == ("Get Heartbeat", "Cam Setup", "Cam Start")
    assert cfg.read_list("s", "b", ["default"]) == ()
    assert cfg.read_list("s", "c") == ("x", "y")
    assert cfg.read_list("s", "d") == ("one",)


# ---------------------------------------------------------------------------------------------- owner files
@pytest.mark.parametrize("name", ["alox.ini", "alox_GiGeIDS.ini", "alox_mx.ini", "alox_usbAllied.ini",
                                  "alox_usbIDS.ini"])
def test_real_files_parse(name: str) -> None:
    cfg = NIConfig.load(REAL / name)
    assert "hardware.camera.highmag" in cfg.sections() and "hardware.z" in cfg.sections()
    assert cfg.read_int("hardware.xy", "xaxis_I32", 0) == 1             # duplicate key: 1, then 10
    assert cfg.read_int("hardware.bandit", "nbpoints9_U32", 9, unsigned=True) == 0
    assert cfg.read_float("general", "wafersize_DBL", 0.0) == pytest.approx(76.2)
    assert cfg.read_str("general", "outputfolder_STR", "") == "D:/alox/data"
    assert cfg.read_path("general", "profilefolder_PATH", "") == "/D/ALOX/profiles"
    assert cfg.read_str("hardware.bandit", "ipadress_STR", "") == " 192.168.1.2"
    assert cfg.read_path("hardware.bandit", "ScriptPythonPath_PATH", "") == \
        'py "D:/ALOX/software/Device Software/Python/readdata_arg.py"  --path '
    assert cfg.read_str("version", "release_STR", "") == "15/01/2019 - 11:59"
