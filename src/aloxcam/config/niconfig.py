"""Read-only INI reader with the semantics of NI ``config.llb`` (LabVIEW 2018).

Spec: 09 §6.1 (parsing: ``String to Config``, ``Parse Key Value Pair``, lookup), §6.2 (``Read Key`` conversions),
§6.6 (reference sketch); docs/phase1-design.md §3.3.1.

Differences from ``configparser``: section and key names are case-insensitive and the **first** occurrence wins (a
duplicated section hides the second copy completely); ``;`` starts a comment unless the whole value is quoted; ``#``
lines containing ``=`` are keys; quotes and ``\\XX`` escapes follow NI; numbers accept a decimal comma. Deviations from
NI: a UTF-8 BOM is tolerated; path values are returned as text (quotes removed, not unescaped) and converted by
:func:`aloxcam.lv.path.localize_path_text`. Nothing is ever written or created (09 D2).
"""

from __future__ import annotations

import re
from collections.abc import Sequence
from pathlib import Path

__all__ = ["ConfigTypeInvalid", "NIConfig"]

_LINES = re.compile(r"[\n\r]+")                          # String to Config: CRLF, LF and CR; empty lines vanish
_SECTION = re.compile(r"[ \t]*\[([^\]]+)\]")             # ^[\t\s]*\[[^\]]+\]
_KEY_VALUE = re.compile(r"[ \t]*[^ \t;=\n\r][^=\n\r]*=")  # ^[\t\s]*[^\t\s;=\n\r][^=\n\r]*[\s\t]*=.*
_ESCAPE = re.compile(r"\\(\\|[01][0-9A-F])")             # Unescape String: \\ and \XX (upper-case hex, 00-1F)
_INT = re.compile(r"[ \t]*([+-]?[0-9]+)")                # Decimal String To Number: stops at the first non-digit
_BOOL = re.compile(r"[ \t]*(true|false)", re.IGNORECASE)
_BLANKS = " \t"
_BOM = b"\xef\xbb\xbf"
_I32 = (-(2**31), 2**31 - 1)
_U32 = (0, 2**32 - 1)


def _float_pattern(sep: str) -> re.Pattern[str]:
    e = re.escape(sep)
    return re.compile(rf"[ \t]*([+-]?(?:[0-9]+(?:{e}[0-9]*)?|{e}[0-9]+)(?:[eE][+-]?[0-9]+)?"
                      rf"|[+-]?(?:inf|nan))", re.IGNORECASE)


_FLOATS = (_float_pattern("."), _float_pattern(","))     # Scan '%.;%f' then '%,;%f': the longer match wins


class ConfigTypeInvalid(ValueError):
    """NI "Config type invalid": the value cannot be converted to the requested type (09 §6.2; code not printed)."""


def _unquote(value: str) -> tuple[bool, str]:
    """``Remove Quotes`` (09 §6.2): ``(in quotes?, text)``; the text between matching quotes, else ``value``."""
    s = value.lstrip(_BLANKS)
    if s[:1] in ('"', "'"):
        m = re.search(re.escape(s[0]) + r"[ \t]*$", s[1:])
        if m:
            return True, s[1:][:m.start()]
    return False, value


def _unescape(text: str) -> str:
    """``Unescape String`` (09 §6.2): ``\\\\`` → ``\\``, ``\\XX`` (``[01][0-9A-F]``) → chr(XX); other ``\\`` kept."""
    return _ESCAPE.sub(lambda m: "\\" if m.group(1) == "\\" else chr(int(m.group(1), 16)), text)


class NIConfig:
    """A parsed INI file (09 §6.1). Immutable after parsing; never writes or creates files (09 D2).

    Lines before the first section form an unnamed section that is never searched (NI: "the first section is for ini
    file's comment"). Missing sections or keys give the caller's default, without an error (09 §6.1).
    """

    def __init__(self, sections: Sequence[tuple[str, Sequence[tuple[str, str]]]]) -> None:
        self._sections: tuple[tuple[str, tuple[tuple[str, str], ...]], ...] = tuple(
            (name, tuple(pairs)) for name, pairs in sections)

    # -- construction
    @classmethod
    def load(cls, path: Path, *, encoding: str = "cp1252") -> NIConfig:
        """Read and parse ``path`` (bytes decoded with ``encoding``, undecodable bytes replaced).

        Raises :class:`FileNotFoundError` when the file is missing (it is never created, 09 D2) and :class:`OSError`
        when it cannot be read.
        """
        data = Path(path).read_bytes()
        if data.startswith(_BOM):                       # deviation: tolerate a UTF-8 BOM whatever the encoding
            data = data[len(_BOM):]
        return cls.parse(data.decode(encoding, errors="replace"))

    @classmethod
    def parse(cls, text: str) -> NIConfig:
        """``String to Config`` + ``Parse Key Value Pair`` (09 §6.1)."""
        text = text.removeprefix("﻿")
        sections: list[tuple[str, list[tuple[str, str]]]] = [("", [])]
        for line in _LINES.split(text):
            m = _SECTION.match(line)
            if m:
                sections.append((m.group(1), []))
                continue
            if not _KEY_VALUE.match(line):
                continue                                # empty line or comment (';', '#' without '=')
            key, _, rest = line.partition("=")
            if _unquote(rest)[0]:
                value = rest.strip(_BLANKS)             # quoted: kept whole, ';' included
            else:
                value = rest.partition(";")[0].strip(_BLANKS)
            sections[-1][1].append((key.strip(_BLANKS), value))
        return cls(sections)

    # -- introspection
    def sections(self) -> list[str]:
        """Section names in file order, as written; a duplicated section (case-insensitive) is listed once."""
        seen: set[str] = set()
        names: list[str] = []
        for name, _ in self._sections[1:]:
            if name.lower() not in seen:
                seen.add(name.lower())
                names.append(name)
        return names

    def keys(self, section: str) -> list[str]:
        """Keys of the (first) matching section in file order, as written, each listed once; ``[]`` if absent."""
        pairs = self._find(section)
        if pairs is None:
            return []
        seen: set[str] = set()
        keys: list[str] = []
        for key, _ in pairs:
            if key.lower() not in seen:
                seen.add(key.lower())
                keys.append(key)
        return keys

    def _find(self, section: str) -> tuple[tuple[str, str], ...] | None:
        s = section.lower()
        for name, pairs in self._sections[1:]:
            if name.lower() == s:
                return pairs
        return None

    def raw(self, section: str, key: str) -> str | None:
        """``Get Section`` / ``Get Key``: the stored value of the first matching section and key (case-insensitive,
        the requested key trimmed), or None when absent (09 §6.1)."""
        pairs = self._find(section)
        if pairs is None:
            return None
        k = key.strip(_BLANKS).lower()
        for kk, value in pairs:
            if kk.lower() == k:
                return value
        return None

    def _without_comment(self, section: str, key: str) -> str | None:
        """``Read Value Without Comment``: quotes removed, then cut at the first ``;`` (09 §6.2)."""
        v = self.raw(section, key)
        return None if v is None else _unquote(v)[1].partition(";")[0]

    @staticmethod
    def _invalid(section: str, key: str, value: str, kind: str) -> ConfigTypeInvalid:
        return ConfigTypeInvalid(f"[{section}] {key} = {value!r} is not a valid {kind}")

    # -- typed reads (Read Key instances, 09 §6.2)
    def read_str(self, section: str, key: str, default: str) -> str:
        """``Read Key (String)``: quotes removed, then ``\\\\`` and ``\\XX`` unescaped."""
        v = self.raw(section, key)
        return default if v is None else _unescape(_unquote(v)[1])

    def read_int(self, section: str, key: str, default: int, *, unsigned: bool = False) -> int:
        """``Read Key (I32/U32)``: leading decimal integer (``12abc`` → 12, ``0x10`` → 0, ``3.7`` → 3), saturated to
        the I32 / U32 range. :class:`ConfigTypeInvalid` when no digit is found."""
        v = self._without_comment(section, key)
        if v is None:
            return default
        m = _INT.match(v)
        if not m:
            raise self._invalid(section, key, v, "U32" if unsigned else "I32")
        lo, hi = _U32 if unsigned else _I32
        return min(max(int(m.group(1)), lo), hi)

    def read_float(self, section: str, key: str, default: float) -> float:
        """``Read Key (Double)``: point and comma scans, the longer match wins (``24,6`` = ``24.6``).
        :class:`ConfigTypeInvalid` when neither scan matches."""
        v = self._without_comment(section, key)
        if v is None:
            return default
        best: tuple[int, float] | None = None
        for pattern in _FLOATS:
            m = pattern.match(v)
            if m and (best is None or m.end() > best[0]):
                best = (m.end(), float(m.group(1).replace(",", ".")))
        if best is None:
            raise self._invalid(section, key, v, "DBL")
        return best[1]

    def read_bool(self, section: str, key: str, default: bool) -> bool:
        """``Read Key (Boolean)``: exactly ``0`` / ``1``, else a case-insensitive ``TRUE`` / ``FALSE`` scan.
        :class:`ConfigTypeInvalid` otherwise (``yes``, ``on``…)."""
        v = self._without_comment(section, key)
        if v is None:
            return default
        if v in ("0", "1"):
            return v == "1"
        m = _BOOL.match(v)
        if not m:
            raise self._invalid(section, key, v, "BOOL")
        return m.group(1).lower() == "true"

    def read_path(self, section: str, key: str, default: str) -> str:
        """``Read Key (Path)`` text: quotes removed, **not** unescaped and **not** localized (use
        :func:`aloxcam.lv.path.localize_path_text`). An empty value gives ``""`` (empty path), not the default."""
        v = self.raw(section, key)
        return default if v is None else _unquote(v)[1]

    def read_list(self, section: str, key: str, default: Sequence[str] = ()) -> tuple[str, ...]:
        """Python-only: :meth:`read_str`, split on commas, items stripped, empty items dropped (an empty value gives
        ``()``)."""
        v = self.raw(section, key)
        if v is None:
            return tuple(default)
        items = (item.strip(_BLANKS) for item in _unescape(_unquote(v)[1]).split(","))
        return tuple(item for item in items if item)
