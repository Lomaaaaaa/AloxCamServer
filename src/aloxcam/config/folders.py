"""Executable folder and ``alox_setup.ini`` → :class:`aloxcam.settings.Folders`, resolved once at start-up.

Spec: 09 §3 (``tools_application_executable_folder.vi`` and the three folder resolvers), §4 (``alox_setup.ini``), §3.4
(reference sketch), D2 (never create the file), D3 (empty value = default), D9 (resolve once); design §3.3.2.

Path values are read raw (quotes removed, not unescaped, 09 §4) and localized with
:func:`aloxcam.lv.path.localize_path_text`: off Windows, ``D:\\…`` maps through ``$ALOX_DRIVE_D`` or is unavailable.
"""

from __future__ import annotations

import logging
import os
import sys
from collections.abc import Callable, Mapping
from pathlib import Path
from types import MappingProxyType

from aloxcam.config.niconfig import NIConfig
from aloxcam.errors import Code, ConfigError
from aloxcam.lv.path import localize_path_text
from aloxcam.settings import Folders

__all__ = ["FOLDER_DEFAULTS", "SETUP_INI_NAME", "executable_folder", "resolve_folders"]

log = logging.getLogger(__name__)

SETUP_INI_NAME = "alox_setup.ini"
_SECTION = "folders"

#: ``[folders]`` keys and their hard-coded defaults (09 §3.2; note ``02-vision`` vs ``02-Vision``).
FOLDER_DEFAULTS: Mapping[str, str] = MappingProxyType({
    "settings": r"D:\alox\settings",
    "vision": r"D:\alox\02-vision",
    "data": r"D:\alox\02-Vision\data",
})

_LOG_USE = {"vision": "error event log and diagnostic log", "data": "information event log"}


def executable_folder(env: Mapping[str, str] = os.environ) -> Path:
    """``tools_application_executable_folder.vi`` (09 §3.1): ``$ALOX_EXE_DIR`` if set; a frozen build → the folder of
    ``sys.executable`` (``App.Kind == Run Time System``); otherwise the current working directory."""
    override = env.get("ALOX_EXE_DIR")
    if override:
        return Path(override)
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path.cwd()


def resolve_folders(*, setup_ini: Path | None = None, exe_dir: Path | None = None,
                    settings_override: Path | None = None, warn: Callable[[str], None]) -> Folders:
    """Read ``[folders] settings / vision / data`` once (09 §3.2, D9).

    ``exe`` is ``exe_dir`` or :func:`executable_folder`; the file is ``setup_ini`` or ``<exe>/alox_setup.ini``.
    A missing or unreadable file → every default + warning; the file is not created (D2). An empty value → its
    default + warning (D3). An unmappable ``vision`` / ``data`` → None + warning (that log is disabled).
    ``settings_override`` (``--settings-dir``) replaces ``[folders] settings``. Raises :class:`ConfigError` when the
    settings folder cannot be mapped (nothing could be loaded).
    """
    exe = Path(exe_dir) if exe_dir is not None else executable_folder()
    setup = Path(setup_ini) if setup_ini is not None else exe / SETUP_INI_NAME
    try:
        cfg = NIConfig.load(setup)
    except FileNotFoundError:
        warn(f"{setup}: not found; built-in folder defaults used (the file is not created)")
        cfg = NIConfig.parse("")
    except OSError as exc:
        warn(f"{setup}: cannot be read ({exc}); built-in folder defaults used")
        cfg = NIConfig.parse("")

    found: dict[str, Path | None] = {}
    for key, default in FOLDER_DEFAULTS.items():
        if key == "settings" and settings_override is not None:
            found[key] = Path(settings_override)
            continue
        text = cfg.read_path(_SECTION, key, default)
        if text.strip() == "":
            warn(f"{setup}: [{_SECTION}] {key} is empty; default {default} used")
            text = default
        local = localize_path_text(text)
        if local is None:
            if key == "settings":
                raise ConfigError(Code.LV_INVALID_INPUT,
                                  f"[{_SECTION}] settings = {text!r} cannot be mapped to a local folder "
                                  "(set ALOX_DRIVE_<LETTER> or use --settings-dir)")
            warn(f"[{_SECTION}] {key} = {text!r} cannot be mapped to a local folder "
                 f"(set ALOX_DRIVE_<LETTER>); {_LOG_USE[key]} disabled")
        found[key] = local
    settings = found["settings"]
    assert settings is not None
    log.debug("folders resolved from %s: %s", setup, found)
    return Folders(exe=exe, settings=settings, vision=found["vision"], data=found["data"])
