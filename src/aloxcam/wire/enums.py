"""Wire enum tables (codes = item positions), their overrides, and the routing table derived from them.

Spec: 04 §6 (enumerations), §6.2 (``Camera Command`` contract), 04 T2 (route by explicit name sets), 03 §3.1–§3.5
(routing rule, item order, routing sketch), 03 C1 / C16 (self-check, internal ``… Run`` commands never routed),
02 §5.2 (``HW TCP Datatype`` is U16); docs/answers/owner-answers.md §1 (confirmed orders); design §3.2.4.

Wire enums are carried as their item **label** everywhere above ``aloxcam.lv`` / ``aloxcam.wire``; codes exist only
here. Merge precedence: packaged file < ``override_file`` (each table present replaces the packaged one; its
``provisional`` defaults to false) < ``items`` / ``sizes`` (replace the items / size of one table; provisional
becomes false).
"""

from __future__ import annotations

import importlib.resources
import json
import tomllib
from collections.abc import Collection, Mapping, Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from types import MappingProxyType
from typing import Any

from aloxcam.dispatch import CAMERA_WORKER, Z_WORKER
from aloxcam.errors import Code, ConfigError, ProtocolError
from aloxcam.settings import WireOptions

__all__ = [
    "CAMERA_EXTRA",
    "CAMERA_RANGE",
    "ENUM_IDS",
    "Z_RANGE",
    "EnumTable",
    "RoutingTable",
    "WireEnums",
    "build_routing",
    "routing_self_check",
]

ENUM_IDS = ("tcp_datatype", "camera_command", "camera_type", "autofocus_speed", "autofocus_method",
            "refocus_method", "z_reference_method")
CAMERA_RANGE = ("Get Heartbeat", "Cam Get Led")
CAMERA_EXTRA = ("Log",)
Z_RANGE = ("Z Setup", "Z Do Reference")

SCHEMA = 1
PACKAGED_SOURCE = "packaged"
_TABLE_KEYS = frozenset({"label", "ctl", "size", "provisional", "lenient", "items"})
_REQUIRED_ITEMS = {
    "tcp_datatype": ("Command", "Response", "Image", "Heartbeat"),
    "camera_type": ("High Mag.", "Low Mag."),
    "camera_command": CAMERA_RANGE + CAMERA_EXTRA + Z_RANGE,
}


def _bad(message: str) -> ConfigError:
    return ConfigError(Code.LV_INVALID_INPUT, f"wire enums: {message}")


@dataclass(frozen=True)
class EnumTable:
    """One enum: item labels in **value** order (code = index), representation size and provenance."""

    enum_id: str
    label: str
    ctl: str
    size: int
    items: tuple[str, ...]
    provisional: bool
    lenient: bool
    source: str

    def code(self, label: str) -> int:
        """Code of ``label``; ``"#<n>"`` → n; :class:`ValueError` for an unknown label."""
        try:
            return self.items.index(label)
        except ValueError:
            pass
        if label.startswith("#"):
            try:
                return int(label[1:])
            except ValueError:
                pass
        raise ValueError(f"{self.enum_id}: unknown item {label!r}")

    def label_of(self, code: int) -> str:
        """Label of ``code``; ``"#<code>"`` for out-of-range codes of a lenient enum, else :class:`ProtocolError`."""
        if 0 <= code < len(self.items):
            return self.items[code]
        if self.lenient:
            return f"#{code}"
        raise ProtocolError(Code.UNFLATTEN_FAILED,
                            f"{self.label}: value {code} out of range (0..{len(self.items) - 1})")


def _norm_id(enum_id: str) -> str:
    return str(enum_id).strip().lower()


def _parse_table(enum_id: str, data: Any, source: str, base: EnumTable | None) -> EnumTable:
    if not isinstance(data, Mapping):
        raise _bad(f"[enums.{enum_id}] in {source} is not a table")
    unknown = set(data) - _TABLE_KEYS
    if unknown:
        raise _bad(f"[enums.{enum_id}] in {source}: unknown key(s) {', '.join(sorted(unknown))}")
    items = data.get("items")
    if not isinstance(items, list) or not all(isinstance(i, str) for i in items):
        raise _bad(f"[enums.{enum_id}] in {source}: 'items' must be a list of strings")
    size = data.get("size", base.size if base else 2)
    if not isinstance(size, int) or isinstance(size, bool):
        raise _bad(f"[enums.{enum_id}] in {source}: 'size' must be an integer")
    flags = {}
    for key in ("provisional", "lenient"):
        value = data.get(key, False)
        if not isinstance(value, bool):
            raise _bad(f"[enums.{enum_id}] in {source}: '{key}' must be a boolean")
        flags[key] = value
    return EnumTable(enum_id=enum_id, label=str(data.get("label", base.label if base else enum_id)),
                     ctl=str(data.get("ctl", base.ctl if base else "")), size=size, items=tuple(items),
                     provisional=flags["provisional"], lenient=flags["lenient"], source=source)


def _tables_from_toml(text: str, source: str, base: Mapping[str, EnumTable]) -> dict[str, EnumTable]:
    try:
        doc = tomllib.loads(text)
    except tomllib.TOMLDecodeError as exc:
        raise _bad(f"{source}: {exc}") from None
    schema = doc.get("schema", SCHEMA)
    if schema != SCHEMA:
        raise _bad(f"{source}: unsupported schema {schema!r} (expected {SCHEMA})")
    unknown = set(doc) - {"schema", "enums"}
    if unknown:
        raise _bad(f"{source}: unknown top-level key(s) {', '.join(sorted(unknown))}")
    enums = doc.get("enums", {})
    if not isinstance(enums, Mapping):
        raise _bad(f"{source}: [enums] is not a table")
    out: dict[str, EnumTable] = {}
    for raw_id, data in enums.items():
        enum_id = _norm_id(raw_id)
        if enum_id not in ENUM_IDS:
            raise _bad(f"{source}: unknown enum id {raw_id!r}")
        out[enum_id] = _parse_table(enum_id, data, source, base.get(enum_id))
    return out


def _validate(tables: Mapping[str, EnumTable]) -> None:
    for enum_id in tables:
        if enum_id not in ENUM_IDS:
            raise _bad(f"unknown enum id {enum_id!r}")
    for enum_id in ENUM_IDS:
        if enum_id not in tables:
            raise _bad(f"table {enum_id!r} missing")
    for t in tables.values():
        if t.size not in (1, 2, 4):
            raise _bad(f"{t.enum_id}: size {t.size} not in 1, 2, 4")
        if not t.items:
            raise _bad(f"{t.enum_id}: no items")
        if len(t.items) > 2 ** (8 * t.size):
            raise _bad(f"{t.enum_id}: {len(t.items)} items do not fit in {t.size} byte(s)")
        seen: set[str] = set()
        for item in t.items:
            if item in seen:
                raise _bad(f"{t.enum_id}: duplicate item {item!r}")
            if item.startswith("#"):
                raise _bad(f"{t.enum_id}: item {item!r} may not start with '#'")
            seen.add(item)
    if tables["tcp_datatype"].size != 2:
        raise _bad("tcp_datatype: size must be 2 (02 §5.2)")
    for enum_id, required in _REQUIRED_ITEMS.items():
        missing = [item for item in required if item not in tables[enum_id].items]
        if missing:
            raise _bad(f"{enum_id}: missing item(s) {', '.join(map(repr, missing))}")
    code = {label: i for i, label in enumerate(tables["camera_command"].items)}
    for first, last in (CAMERA_RANGE, Z_RANGE):
        if code[first] > code[last]:
            raise _bad(f"camera_command: range {first!r}..{last!r} is out of order")
    camera = set(range(code[CAMERA_RANGE[0]], code[CAMERA_RANGE[1]] + 1)) | {code[x] for x in CAMERA_EXTRA}
    z = set(range(code[Z_RANGE[0]], code[Z_RANGE[1]] + 1))
    if camera & z:
        raise _bad("camera_command: the camera and Z routing ranges overlap (LabVIEW refuses this case structure)")


class WireEnums:
    """The wire enum tables (implements :class:`aloxcam.lv.flatten.EnumCodec`); immutable after load."""

    def __init__(self, tables: Mapping[str, EnumTable]) -> None:
        _validate(tables)
        self.tables: Mapping[str, EnumTable] = MappingProxyType({eid: tables[eid] for eid in ENUM_IDS})

    def __repr__(self) -> str:
        provisional = ", ".join(self.provisional_ids) or "none"
        return f"WireEnums({len(self.tables)} tables, provisional: {provisional})"

    @property
    def provisional_ids(self) -> tuple[str, ...]:
        """Ids of the tables whose order is still provisional (logged at WARNING at start-up)."""
        return tuple(eid for eid, t in self.tables.items() if t.provisional)

    def table(self, enum_id: str) -> EnumTable:
        """The table ``enum_id``; :class:`KeyError` when unknown."""
        return self.tables[enum_id]

    def size(self, enum_id: str) -> int:
        return self.tables[enum_id].size

    def label(self, enum_id: str, code: int) -> str:
        return self.tables[enum_id].label_of(code)

    def code(self, enum_id: str, label: str) -> int:
        return self.tables[enum_id].code(label)

    @classmethod
    def load(cls, *, override_file: Path | None = None, items: Mapping[str, Sequence[str]] = {},
             sizes: Mapping[str, int] = {}) -> WireEnums:
        """Packaged table < ``override_file`` < ``items`` / ``sizes``; :class:`ConfigError` on any problem."""
        packaged = importlib.resources.files("aloxcam") / "data" / "wire_enums.toml"
        tables = _tables_from_toml(packaged.read_text(encoding="utf-8"), PACKAGED_SOURCE, {})
        if override_file is not None:
            path = Path(override_file)
            try:
                text = path.read_text(encoding="utf-8")
            except OSError as exc:
                raise _bad(f"cannot read enums_file {path}: {exc}") from None
            tables.update(_tables_from_toml(text, str(path), tables))
        for raw_id, labels in items.items():
            enum_id = _norm_id(raw_id)
            if enum_id not in tables:
                raise _bad(f"[wire.items]: unknown enum id {raw_id!r}")
            if isinstance(labels, str):
                raise _bad(f"[wire.items] {raw_id}: a sequence of labels is expected")
            tables[enum_id] = replace(tables[enum_id], items=tuple(str(x) for x in labels), provisional=False,
                                      source="[wire.items]")
        for raw_id, size in sizes.items():
            enum_id = _norm_id(raw_id)
            if enum_id not in tables:
                raise _bad(f"[wire.size]: unknown enum id {raw_id!r}")
            tables[enum_id] = replace(tables[enum_id], size=int(size), provisional=False, source="[wire.size]")
        return cls(tables)

    @classmethod
    def from_options(cls, wire: WireOptions, settings_dir: Path) -> WireEnums:
        """Load with the ``[wire]`` options; a relative ``enums_file`` is resolved against ``settings_dir``."""
        override = wire.enums_file
        if override is not None and not Path(override).is_absolute():
            override = Path(settings_dir) / override
        return cls.load(override_file=override, items=wire.enum_items, sizes=wire.enum_sizes)

    def to_toml(self) -> str:
        """The tables in the ``wire_enums.toml`` format (schema 1); loadable as an ``enums_file``."""
        lines = ["# Wire enum tables written by aloxcam (format: aloxcam/data/wire_enums.toml).", f"schema = {SCHEMA}"]
        for t in self.tables.values():
            lines += ["", f"[enums.{t.enum_id}]", f"label = {json.dumps(t.label)}", f"ctl = {json.dumps(t.ctl)}",
                      f"size = {t.size}", f"provisional = {str(t.provisional).lower()}",
                      f"lenient = {str(t.lenient).lower()}", "items = ["]
            lines += [f"  {json.dumps(item)}," for item in t.items]
            lines.append("]")
        return "\n".join(lines) + "\n"


@dataclass(frozen=True)
class RoutingTable:
    """Code → worker, evaluated once from the item list (03 §3.5); behaves like the LabVIEW selector for every code."""

    names: Mapping[int, str]              # code → item label (all items)
    routes: Mapping[int, str]             # code → CAMERA_WORKER | Z_WORKER; absent = router "Default"

    def route(self, code: int) -> str | None:
        """The worker of ``code``, or None (router ``Default``: dropped, still recorded in the history)."""
        return self.routes.get(code)

    def name(self, code: int) -> str:
        """Item label of ``code``; ``"#<code>"`` for codes beyond the item list."""
        return self.names.get(code, f"#{code}")


def build_routing(enums: WireEnums, *,
                  non_routable: Collection[str] = ("Cam Autofocus Run", "Cam Refocus Run")) -> RoutingTable:
    """``"Z Setup".."Z Do Reference"`` → Z, ``"Get Heartbeat".."Cam Get Led", "Log"`` → camera (inclusive ranges,
    03 §3.1), minus the ``non_routable`` items (03 C16)."""
    items = enums.table("camera_command").items
    code = {label: i for i, label in enumerate(items)}
    routes: dict[int, str] = {c: Z_WORKER for c in range(code[Z_RANGE[0]], code[Z_RANGE[1]] + 1)}
    for c in range(code[CAMERA_RANGE[0]], code[CAMERA_RANGE[1]] + 1):
        routes[c] = CAMERA_WORKER
    for label in CAMERA_EXTRA:
        routes[code[label]] = CAMERA_WORKER
    for label in non_routable:
        if label in code:
            routes.pop(code[label], None)
    return RoutingTable(names=MappingProxyType(dict(enumerate(items))), routes=MappingProxyType(routes))


def routing_self_check(routing: RoutingTable, cases: Mapping[str, str]) -> list[str]:
    """Every command that has a case must reach the worker holding it (03 C1); returns the problems found.

    ``cases``: label → worker that has a case (``wire.messages.CASES``). Each problem reads
    ``"<name>: case unreachable (routes to <worker | nowhere>)"`` or ``"<name>: case unreachable (not in the
    camera_command enum table)"``.
    """
    codes = {name: c for c, name in routing.names.items()}
    problems: list[str] = []
    for name, worker in cases.items():
        if name not in codes:
            problems.append(f"{name}: case unreachable (not in the camera_command enum table)")
            continue
        route = routing.route(codes[name])
        if route != worker:
            problems.append(f"{name}: case unreachable (routes to {route or 'nowhere'})")
    return problems
