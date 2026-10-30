"""aloxcam.testing helpers and the shared fixtures of tests/conftest.py (design §3.1.7)."""

from __future__ import annotations

from aloxcam.settings import HIGH_MAG, ServerOptions
from aloxcam.testing import REFERENCE_ALOX_INI, AloxTree, make_state


def test_tmp_alox_tree(tmp_alox_tree: AloxTree) -> None:
    tree = tmp_alox_tree
    for folder in (tree.exe, tree.settings, tree.vision, tree.data):
        assert folder.is_dir() and folder.parent == tree.root
    setup = tree.setup_ini.read_bytes().decode("cp1252")
    assert setup.startswith("[folders]\r\n")
    assert f"settings = {tree.settings}\r\n" in setup and f"vision = {tree.vision}\r\n" in setup
    assert f"data = {tree.data}\r\n" in setup
    ini = tree.alox_ini.read_bytes()
    assert ini == REFERENCE_ALOX_INI.replace("\n", "\r\n").encode("cp1252")
    assert b"zmax_DBL = 20\r\n" in ini and b"frameratemax_DBL = 24.6" in ini
    assert not (tree.vision / "logs").exists() and not (tree.settings / "alox_camera_server.ini").exists()


def test_make_state_overrides() -> None:
    state = make_state(options=ServerOptions(commands_history=2), connected=True, z_pos=3.5)
    assert state.connected and state.z_pos == 3.5 and state.commands_history.maxlen == 2
    assert state.high_mag.camera_name == "HighMag" and state.inited[HIGH_MAG] is False
