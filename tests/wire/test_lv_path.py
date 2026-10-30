"""aloxcam.lv.path: PTH0 vectors, Windows text round trips and localization (04 §3.2, 09 §6.3, design §3.2.2)."""

from __future__ import annotations

from pathlib import Path

import pytest

from aloxcam.errors import ProtocolError
from aloxcam.lv.flatten import Reader, Writer
from aloxcam.lv.path import (
    PATH_ABSOLUTE,
    PATH_NOT_A_PATH,
    PATH_RELATIVE,
    PATH_UNC,
    LVPath,
    common_to_windows,
    localize_path_text,
    read_path,
    write_path,
)
from golden import spec_vectors as sv

POSIX = "linux"
ENV = {"ALOX_DRIVE_D": "/srv/d", "ALOX_DRIVE_C": "/mnt/c"}


@pytest.mark.parametrize("vector", sv.PATH_VECTORS, ids=lambda v: v.name)
def test_spec_vectors_04_3_2(vector: sv.PathVector) -> None:
    assert vector.hex is not None
    data = sv.h(vector.hex)
    assert write_path(Writer(), vector.path).bytes() == data
    assert Writer().path(vector.path).bytes() == data
    r = Reader(data)
    assert read_path(r) == vector.path and r.remaining == 0
    assert vector.path.to_windows() == vector.windows
    assert LVPath.from_windows(vector.windows) == vector.path


@pytest.mark.parametrize("vector", sv.PATH_TEXT_VECTORS, ids=lambda v: v.name)
def test_text_vectors_04_3_2(vector: sv.PathVector) -> None:
    assert LVPath.from_windows(vector.windows) == vector.path
    assert vector.path.to_windows() == vector.windows
    assert Reader(Writer().path(vector.path).bytes()).path() == vector.path


@pytest.mark.parametrize("text", ["C:\\f.png", "D:\\alox\\02-vision\\data", "C:\\", "\\\\srv\\share\\a.png",
                                  "..\\x\\y", "a\\b", "..\\..\\up", ""])
def test_windows_round_trips(text: str) -> None:
    p = LVPath.from_windows(text)
    assert p.to_windows() == text
    assert Reader(Writer().path(p).bytes()).path() == p


def test_from_windows_details() -> None:
    assert LVPath.from_windows("D:/sim") == LVPath(PATH_ABSOLUTE, ("D", "sim"))
    assert LVPath.from_windows(".\\a") == LVPath(PATH_RELATIVE, ("a",))
    assert LVPath.from_windows(None) == LVPath(PATH_NOT_A_PATH)
    assert LVPath().is_empty and not LVPath(PATH_RELATIVE).is_empty and not LVPath(PATH_NOT_A_PATH).is_empty


def test_local_and_user_forms() -> None:
    assert LVPath.from_local("/a/b") == LVPath(PATH_ABSOLUTE, ("a", "b"))
    assert LVPath.from_local(Path("/a/b")) == LVPath(PATH_ABSOLUTE, ("a", "b"))
    assert LVPath.from_local("rel/../x") == LVPath(PATH_RELATIVE, ("rel", "", "x"))
    assert LVPath.from_local("") == LVPath()
    assert LVPath.parse_user("C:\\f.png") == LVPath(PATH_ABSOLUTE, ("C", "f.png"))
    assert LVPath.parse_user("\\\\srv\\s") == LVPath(PATH_UNC, ("srv", "s"))
    assert LVPath.parse_user("/tmp/sim") == LVPath(PATH_ABSOLUTE, ("tmp", "sim"))
    assert LVPath.parse_user("") == LVPath()


def test_to_local_posix() -> None:
    assert LVPath.from_local("/tmp/x.png").to_local(platform=POSIX, env={}) == Path("/tmp/x.png")
    assert LVPath(PATH_ABSOLUTE, ("D", "sim")).to_local(platform=POSIX, env=ENV) == Path("/srv/d/sim")
    assert LVPath(PATH_ABSOLUTE, ("D", "sim")).to_local(platform=POSIX, env={}) == Path("/D/sim")
    assert LVPath(PATH_UNC, ("srv", "s")).to_local(platform=POSIX, env=ENV) is None
    assert LVPath().to_local(platform=POSIX, env=ENV) is None
    assert LVPath(PATH_NOT_A_PATH).to_local(platform=POSIX, env=ENV) is None
    assert LVPath(PATH_RELATIVE, ("", "x")).to_local(platform=POSIX, env=ENV) == Path("../x")


def test_to_local_windows() -> None:
    assert str(LVPath(PATH_ABSOLUTE, ("D", "sim")).to_local(platform="win32", env={})) == "D:\\sim"
    assert LVPath().to_local(platform="win32") is None


def test_common_to_windows() -> None:
    assert common_to_windows("/D/alox/settings") == "D:\\alox\\settings"
    assert common_to_windows("D:\\alox\\settings") == "D:\\alox\\settings"
    assert common_to_windows("\\\\server\\share\\x") == "\\\\server\\share\\x"
    assert common_to_windows("relative\\dir") == "relative\\dir"
    assert common_to_windows("/C/Users/x") == "C:\\Users\\x"
    assert common_to_windows("") == ""


def test_localize_path_text_posix_rules() -> None:
    def loc(text: str, env: dict[str, str] = ENV) -> Path | None:
        return localize_path_text(text, platform=POSIX, env=env)

    assert loc("") is None
    assert loc("D:\\alox\\settings") == Path("/srv/d/alox/settings")
    assert loc("d:/alox/settings") == Path("/srv/d/alox/settings")
    assert loc("D:\\") == Path("/srv/d")
    assert loc("D:\\alox\\settings", {}) is None                  # drive path without a mapping
    assert loc("/D/alox/settings") == Path("/srv/d/alox/settings")
    assert loc("/D/alox/settings", {}) == Path("/D/alox/settings")  # common form without a mapping: native
    assert loc("\\\\server\\share\\x") is None
    assert loc("//server/share/x") is None
    assert loc("/tmp/alox/settings") == Path("/tmp/alox/settings")
    assert loc("relative/dir") == Path("relative/dir")
    assert loc("E:\\x") is None                                    # no ALOX_DRIVE_E


def test_localize_path_text_windows() -> None:
    assert str(localize_path_text("/D/alox/settings", platform="win32", env={})) == "D:\\alox\\settings"
    assert localize_path_text("", platform="win32") is None


def test_malformed_paths() -> None:
    for bad in (b"PTH1\x00\x00\x00\x04\x00\x00\x00\x00",                 # wrong signature
                b"PTH0\x00\x00\x00\x02\x00\x00",                         # body shorter than type + count
                b"PTH0\x00\x00\x00\x05\x00\x00\x00\x00\x00",             # a byte after the components
                b"PTH0\x00\x00\x00\x06\x00\x00\x00\x01\x05a",            # component runs past the body
                b"PTH0\x00\x00\x00\x04\x00\x09\x00\x00"):                # unknown path type
        with pytest.raises(ProtocolError):
            Reader(bad).path()
    assert Reader(b"PTH0\x00\x00\x00\x00").path() == LVPath()           # zero-length body: empty path
    with pytest.raises(ValueError):
        Writer().path(LVPath(PATH_ABSOLUTE, ("C", "x" * 256)))
