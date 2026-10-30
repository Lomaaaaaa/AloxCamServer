"""LabVIEW paths: the lossless ``PTH0`` wire form, Windows text and local (POSIX or Windows) paths.

Spec: 04 §3.2 (flattened ``PTH0`` layout, inferred), 04 §9.2 (``LVPath`` reference sketch), 09 §6.3 (NI ``Common Path
to Specific Path``, Windows branch); design §3.2.2 (localization rules off Windows).

Localization off Windows (development and tests): a drive-letter path (``D:\\x``, or ``/D/x`` with a one-letter first
component) maps through the environment variable ``ALOX_DRIVE_<LETTER>``; ``D:\\…`` without a mapping → None; ``/D/x``
without a mapping → native ``/D/x``; UNC → None; anything else is a native path.
"""

from __future__ import annotations

import os
import re
import struct
import sys
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import TYPE_CHECKING

from aloxcam.errors import Code, ProtocolError

if TYPE_CHECKING:
    from aloxcam.lv.flatten import Reader, Writer

__all__ = [
    "PATH_ABSOLUTE",
    "PATH_NOT_A_PATH",
    "PATH_RELATIVE",
    "PATH_UNC",
    "LVPath",
    "common_to_windows",
    "localize_path_text",
    "read_path",
    "write_path",
]

PATH_ABSOLUTE = 0
PATH_RELATIVE = 1
PATH_NOT_A_PATH = 2
PATH_UNC = 3

PTH0 = b"PTH0"
_DRIVE = re.compile(r"^([A-Za-z]):(?:[\\/]|$)")
_SEPARATORS = re.compile(r"[\\/]+")


def _is_windows(platform: str) -> bool:
    return platform == "win32"


def _looks_windows(text: str) -> bool:
    """A drive letter, a backslash or a UNC prefix: Windows text rather than a POSIX path."""
    return bool(_DRIVE.match(text)) or "\\" in text


@dataclass(frozen=True)
class LVPath:
    """A LabVIEW path as it travels: path type + components (04 §3.2). Lossless; convert at the edges."""

    kind: int = PATH_ABSOLUTE
    components: tuple[str, ...] = ()

    @property
    def is_empty(self) -> bool:
        """Absolute with no component: LabVIEW ``<empty path>``."""
        return self.kind == PATH_ABSOLUTE and not self.components

    def to_windows(self) -> str | None:
        """Windows text form (04 §9.2): None for Not-A-Path, ``""`` for the empty path, ``..`` for empty components."""
        if self.kind == PATH_NOT_A_PATH:
            return None
        if self.kind == PATH_UNC:
            return "\\\\" + "\\".join(self.components)
        if self.kind == PATH_ABSOLUTE:
            if not self.components:
                return ""
            drive, *rest = self.components
            return drive + ":\\" + "\\".join(rest)
        return "\\".join(".." if c == "" else c for c in self.components)

    @classmethod
    def from_windows(cls, text: str | None) -> LVPath:
        """Parse Windows text (04 §9.2): None → Not-A-Path, ``""`` → empty path, ``\\\\srv\\s`` → UNC,
        ``C:\\x`` → absolute ``("C", "x")``, anything else relative (``..`` → empty component)."""
        if text is None:
            return cls(PATH_NOT_A_PATH)
        s = text.replace("/", "\\")
        if s == "":
            return cls()
        if s.startswith("\\\\"):
            return cls(PATH_UNC, tuple(c for c in s[2:].split("\\") if c))
        if len(s) >= 2 and s[1] == ":":
            return cls(PATH_ABSOLUTE, (s[0],) + tuple(c for c in s[2:].split("\\") if c))
        return cls(PATH_RELATIVE, tuple("" if c == ".." else c for c in s.split("\\") if c not in ("", ".")))

    def to_local(self, *, platform: str = sys.platform, env: Mapping[str, str] = os.environ) -> Path | None:
        """A local path, or None when there is none (empty path, Not-A-Path, unmappable; design §3.2.2)."""
        if self.kind == PATH_NOT_A_PATH or self.is_empty:
            return None
        if _is_windows(platform):
            text = self.to_windows()
            return Path(text) if text else None
        if self.kind == PATH_UNC:
            return None
        if self.kind == PATH_ABSOLUTE:
            return localize_path_text("/" + "/".join(self.components), platform=platform, env=env)
        if not self.components:
            return None
        return Path(*(".." if c == "" else c for c in self.components))

    @classmethod
    def from_local(cls, path: Path | str) -> LVPath:
        """POSIX ``/a/b`` → absolute ``("a", "b")``; relative → relative; Windows-looking text →
        :meth:`from_windows`."""
        s = os.fspath(path)
        if s == "":
            return cls()
        if _looks_windows(s):
            return cls.from_windows(s)
        p = PurePosixPath(s)
        if p.is_absolute():
            return cls(PATH_ABSOLUTE, tuple(p.parts[1:]))
        return cls(PATH_RELATIVE, tuple("" if part == ".." else part for part in p.parts if part != "."))

    @classmethod
    def parse_user(cls, text: str) -> LVPath:
        """User text (replay scripts): the Windows form if it has a drive / backslash / UNC, else :meth:`from_local`."""
        if text == "":
            return cls()
        return cls.from_windows(text) if _looks_windows(text) else cls.from_local(text)


def read_path(r: Reader) -> LVPath:
    """Read ``"PTH0"`` · U32 n · U16 type · U16 count · count × (U8 length + bytes) (04 §3.2).

    A zero-length body reads as the empty path (defensive); a body with bytes left over, a component running past
    the body or an unknown path type raises :class:`ProtocolError`.
    """
    start = r.pos
    if r.take(4) != PTH0:
        raise ProtocolError(Code.UNFLATTEN_FAILED, f"path: 'PTH0' expected at offset {start}")
    n = r.u32()
    body = r.take(n)
    if n == 0:
        return LVPath()
    if n < 4:
        raise ProtocolError(Code.UNFLATTEN_FAILED, f"path at offset {start}: body of {n} bytes is too short")
    kind, count = struct.unpack(">HH", body[:4])
    if kind > PATH_UNC:
        raise ProtocolError(Code.UNFLATTEN_FAILED, f"path at offset {start}: unknown path type {kind}")
    encoding = getattr(r, "encoding", "cp1252")
    pos = 4
    components: list[str] = []
    for _ in range(count):
        if pos >= n:
            raise ProtocolError(Code.UNFLATTEN_FAILED, f"path at offset {start}: component list truncated")
        length = body[pos]
        pos += 1
        if pos + length > n:
            raise ProtocolError(Code.UNFLATTEN_FAILED, f"path at offset {start}: component runs past the body")
        components.append(body[pos:pos + length].decode(encoding, "replace"))
        pos += length
    if pos != n:
        raise ProtocolError(Code.UNFLATTEN_FAILED, f"path at offset {start}: {n - pos} unexpected bytes in the body")
    return LVPath(kind, tuple(components))


def write_path(w: Writer, p: LVPath) -> Writer:
    """Write ``p`` in the ``PTH0`` form; :class:`ValueError` for a component longer than 255 bytes."""
    encoding = getattr(w, "encoding", "cp1252")
    errors = getattr(w, "errors", "strict")
    body = bytearray(struct.pack(">HH", p.kind, len(p.components)))
    for c in p.components:
        b = c.encode(encoding, errors)
        if len(b) > 255:
            raise ValueError(f"path component longer than 255 bytes: {c[:40]!r}…")
        body.append(len(b))
        body += b
    return w.raw(PTH0 + struct.pack(">I", len(body)) + bytes(body))


def common_to_windows(text: str) -> str:
    """NI ``Common Path to Specific Path``, Windows branch (09 §6.3): ``/D/alox/x`` → ``D:\\alox\\x``.

    Every ``/`` becomes ``\\``; a single leading ``\\`` becomes ``<letter>:``; native and UNC text are unchanged.
    """
    s = text.replace("/", "\\")
    if s.startswith("\\") and not s.startswith("\\\\") and len(s) >= 2:
        s = s[1] + ":" + s[2:]
    return s


def _mapped(letter: str, rest: str, env: Mapping[str, str]) -> Path | None:
    base = env.get(f"ALOX_DRIVE_{letter.upper()}")
    if not base:
        return None
    return Path(base, *(part for part in _SEPARATORS.split(rest) if part))


def localize_path_text(text: str, *, platform: str = sys.platform,
                       env: Mapping[str, str] = os.environ) -> Path | None:
    """INI / user path text → local path, or None when it cannot be used here (design §3.2.2, 09 §6.3).

    ``""`` → None. Windows: :func:`common_to_windows`. Elsewhere: ``D:\\…`` → ``$ALOX_DRIVE_D/…`` or None;
    ``/D/…`` → ``$ALOX_DRIVE_D/…`` or native ``/D/…``; UNC → None; anything else native.
    """
    if not text:
        return None
    if _is_windows(platform):
        return Path(common_to_windows(text))
    if text.startswith(("\\\\", "//")):
        return None
    m = _DRIVE.match(text)
    if m:
        return _mapped(m.group(1), text[2:], env)
    if text.startswith("/"):
        parts = [part for part in text.split("/") if part]
        if parts and len(parts[0]) == 1 and parts[0].isalpha():
            mapped = _mapped(parts[0], "/".join(parts[1:]), env)
            if mapped is not None:
                return mapped
    return Path(text)
