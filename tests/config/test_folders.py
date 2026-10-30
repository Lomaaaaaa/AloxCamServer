"""aloxcam.config.folders: executable folder and alox_setup.ini resolution (09 §3–§4, D2/D3/D9, design §3.3.2)."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

from aloxcam.config.folders import FOLDER_DEFAULTS, SETUP_INI_NAME, executable_folder, resolve_folders
from aloxcam.errors import ConfigError
from aloxcam.settings import Folders
from aloxcam.testing import AloxTree

posix_only = pytest.mark.skipif(sys.platform == "win32", reason="drive-letter mapping off Windows")


@pytest.fixture(autouse=True)
def _no_drive_mapping(monkeypatch: pytest.MonkeyPatch) -> None:
    for letter in "CD":
        monkeypatch.delenv(f"ALOX_DRIVE_{letter}", raising=False)
    monkeypatch.delenv("ALOX_EXE_DIR", raising=False)


def write_setup(folder: Path, text: str) -> Path:
    folder.mkdir(parents=True, exist_ok=True)
    p = folder / SETUP_INI_NAME
    p.write_bytes(text.replace("\n", "\r\n").encode("cp1252"))
    return p


def test_defaults_09_3_2() -> None:
    assert dict(FOLDER_DEFAULTS) == {"settings": r"D:\alox\settings", "vision": r"D:\alox\02-vision",
                                     "data": r"D:\alox\02-Vision\data"}


# ---------------------------------------------------------------------------------------------- executable folder
def test_executable_folder_env_override(tmp_path: Path) -> None:
    assert executable_folder({"ALOX_EXE_DIR": str(tmp_path)}) == tmp_path


def test_executable_folder_development_is_cwd(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.chdir(tmp_path)
    assert executable_folder({}) == Path.cwd()
    assert executable_folder({"ALOX_EXE_DIR": ""}) == Path.cwd()


def test_executable_folder_frozen(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    exe = tmp_path / "bin" / "aloxcam.exe"
    exe.parent.mkdir()
    exe.write_bytes(b"")
    monkeypatch.setattr(sys, "frozen", True, raising=False)
    monkeypatch.setattr(sys, "executable", str(exe))
    assert executable_folder({}) == exe.parent.resolve()
    assert executable_folder({"ALOX_EXE_DIR": str(tmp_path)}) == tmp_path   # the variable still wins


def test_resolve_uses_alox_exe_dir(monkeypatch: pytest.MonkeyPatch, tmp_alox_tree: AloxTree) -> None:
    monkeypatch.setenv("ALOX_EXE_DIR", str(tmp_alox_tree.exe))
    warnings: list[str] = []
    folders = resolve_folders(warn=warnings.append)
    assert folders == Folders(tmp_alox_tree.exe, tmp_alox_tree.settings, tmp_alox_tree.vision, tmp_alox_tree.data)
    assert warnings == []


# ---------------------------------------------------------------------------------------------- alox_setup.ini
def test_tree_fixture(tmp_alox_tree: AloxTree) -> None:
    warnings: list[str] = []
    folders = resolve_folders(exe_dir=tmp_alox_tree.exe, warn=warnings.append)
    assert folders == Folders(tmp_alox_tree.exe, tmp_alox_tree.settings, tmp_alox_tree.vision, tmp_alox_tree.data)
    assert warnings == []
    # an explicit --setup-ini; exe stays the executable folder
    other = tmp_alox_tree.root / "elsewhere"
    folders = resolve_folders(setup_ini=tmp_alox_tree.setup_ini, exe_dir=other, warn=warnings.append)
    assert folders.exe == other and folders.settings == tmp_alox_tree.settings and warnings == []


@posix_only
def test_missing_setup_ini_defaults_warning_not_created(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setenv("ALOX_DRIVE_D", str(tmp_path / "d"))
    exe = tmp_path / "exe"
    exe.mkdir()
    warnings: list[str] = []
    folders = resolve_folders(exe_dir=exe, warn=warnings.append)
    assert folders == Folders(exe, tmp_path / "d" / "alox" / "settings", tmp_path / "d" / "alox" / "02-vision",
                              tmp_path / "d" / "alox" / "02-Vision" / "data")
    assert len(warnings) == 1 and "not found" in warnings[0]
    assert not (exe / SETUP_INI_NAME).exists()          # 09 D2 fix: never created
    assert list(exe.iterdir()) == []


@posix_only
def test_unmappable_defaults(tmp_path: Path) -> None:
    exe = tmp_path / "exe"
    exe.mkdir()
    with pytest.raises(ConfigError, match="settings"):
        resolve_folders(exe_dir=exe, warn=lambda _: None)
    warnings: list[str] = []
    folders = resolve_folders(exe_dir=exe, settings_override=tmp_path / "s", warn=warnings.append)
    assert folders.settings == tmp_path / "s" and folders.vision is None and folders.data is None
    assert len(warnings) == 3                           # file missing + vision + data unmappable
    assert any("vision" in w and "disabled" in w for w in warnings)
    assert any("data" in w and "disabled" in w for w in warnings)


@posix_only
def test_empty_value_gives_default_with_warning(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setenv("ALOX_DRIVE_D", str(tmp_path / "d"))
    setup = write_setup(tmp_path / "exe", f'[folders]\nsettings = {tmp_path / "s"}\nvision =\ndata = ""\n')
    warnings: list[str] = []
    folders = resolve_folders(setup_ini=setup, exe_dir=tmp_path / "exe", warn=warnings.append)
    assert folders.settings == tmp_path / "s"
    assert folders.vision == tmp_path / "d" / "alox" / "02-vision"           # 09 D3 fix
    assert folders.data == tmp_path / "d" / "alox" / "02-Vision" / "data"
    assert len(warnings) == 2 and all("empty" in w for w in warnings)


@posix_only
def test_missing_keys_take_defaults_silently(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setenv("ALOX_DRIVE_D", str(tmp_path / "d"))
    setup = write_setup(tmp_path / "exe", "[Folders]\nSETTINGS = /D/alox/custom ; NI common form\n")
    warnings: list[str] = []
    folders = resolve_folders(setup_ini=setup, warn=warnings.append)
    assert folders.settings == tmp_path / "d" / "alox" / "custom"
    assert folders.vision == tmp_path / "d" / "alox" / "02-vision"
    assert warnings == []


@posix_only
def test_quoted_and_windows_values(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setenv("ALOX_DRIVE_D", str(tmp_path / "d"))
    setup = write_setup(tmp_path / "exe", '[folders]\nsettings = "D:\\alox\\settings"\nvision = D:\\v\\10\ndata = '
                                          '\\\\server\\share\\data\n')
    warnings: list[str] = []
    folders = resolve_folders(setup_ini=setup, warn=warnings.append)
    assert folders.settings == tmp_path / "d" / "alox" / "settings"
    assert folders.vision == tmp_path / "d" / "v" / "10"                     # backslashes literal (not unescaped)
    assert folders.data is None                                             # UNC: unavailable off Windows
    assert len(warnings) == 1 and "data" in warnings[0]


def test_settings_override_wins(tmp_alox_tree: AloxTree, tmp_path: Path) -> None:
    override = tmp_path / "override"
    folders = resolve_folders(setup_ini=tmp_alox_tree.setup_ini, settings_override=override, warn=lambda _: None)
    assert folders.settings == override
    assert folders.vision == tmp_alox_tree.vision and folders.data == tmp_alox_tree.data


def test_unreadable_setup_ini_is_treated_as_missing(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setenv("ALOX_DRIVE_D", str(tmp_path / "d"))
    setup = tmp_path / "exe" / SETUP_INI_NAME
    setup.mkdir(parents=True)                           # a directory: cannot be read
    warnings: list[str] = []
    folders = resolve_folders(setup_ini=setup, settings_override=tmp_path / "s", warn=warnings.append)
    assert folders.settings == tmp_path / "s"
    assert any("cannot be read" in w for w in warnings)
