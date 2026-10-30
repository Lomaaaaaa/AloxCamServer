#!/usr/bin/env python3
"""Scripted client of the ALOX camera server: send commands, check the replies, record them and compare recordings.

Spec: 10 §7 (validation strategy: the same scripted session against the LabVIEW and the Python server, replies
diffed), 10 §5 (Phase 1 exit criteria), 02 §10 (notifier correlation), 02 §5 / §8 / §9 (frames, reply payloads,
heartbeat frames); docs/phase1-design.md §3.7.2 (CLI, script format, record / compare), §5.

Usage::

    python tools/replay_client.py SCRIPT.toml [--host H] [--port P] [--timeout S] [--var NAME=VALUE ...]
           [--settings-ini PATH] [--enums PATH] [--templates PATH] [--record OUT.jsonl] [--compare REF.jsonl]
           [--mask tick|image ...] [--stop-on-fail] [--quiet]

Exit code: 0 = every step PASS (and no compare difference), 1 = a FAIL or a difference, 2 = usage or connection
error. The script format is described in docs/phase1-design.md §3.7.2; the additions of this implementation are listed
in :func:`load_script`.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import re
import sys
import tempfile
import time
import tomllib
from collections.abc import Callable, Iterator, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, TextIO

try:                                    # a source checkout without ``pip install -e .``: use <repo>/src
    import aloxcam  # noqa: F401
except ImportError:                     # pragma: no cover - depends on how the tool is started
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np

from aloxcam.client import AloxClient, ReceivedReply
from aloxcam.dispatch import ReplyKind
from aloxcam.errors import ConfigError, ErrorCluster, ProtocolError
from aloxcam.imaging.codec import decode_provisional
from aloxcam.lv.flatten import Reader
from aloxcam.settings import WireOptions
from aloxcam.wire.enums import WireEnums
from aloxcam.wire.messages import COMMANDS, PayloadCodec, payload_from_mapping
from aloxcam.wire.protocol import (
    FRAME_HEADER_SIZE,
    Datatypes,
    EnvelopeError,
    ResponseTimestamps,
    decode_envelope,
    decode_response_data,
    encode_frame,
    encode_response_data,
    heartbeat_frame,
)

__all__ = [
    "EXPECT_REPLIES",
    "MASKS",
    "REPLY_KINDS",
    "ErrorExpect",
    "Expect",
    "ImageExpect",
    "Poll",
    "RunReport",
    "Script",
    "ScriptError",
    "SessionOptions",
    "Step",
    "StepResult",
    "compare_records",
    "load_script",
    "main",
    "mask_reply",
    "run_script",
]

REPLY_KINDS = ("ACK", "STR", "BOOL", "DBL", "U32", "DBL2D", "IMAGE")
EXPECT_REPLIES = REPLY_KINDS + ("none", "any")
MASKS = ("tick", "image")
CONNECTION_STATES = ("closed", "open")
POLL_CONDITIONS: Mapping[str, Callable[[Any, Any], bool]] = {
    "until_eq": lambda v, t: v == t,
    "until_ne": lambda v, t: v != t,
    "until_ge": lambda v, t: v >= t,
    "until_gt": lambda v, t: v > t,
    "until_le": lambda v, t: v <= t,
    "until_lt": lambda v, t: v < t,
}
EXIT_OK, EXIT_FAIL, EXIT_USAGE = 0, 1, 2
NOTIFIER_ENCODING = "cp1252"

_ACTIONS = {"command": "command", "code": "code", "await": "await", "pause_s": "pause",
            "heartbeat_frame": "heartbeat", "raw_hex": "raw"}
_ALLOWED_KEYS = {
    "command": {"name", "command", "data", "data_hex", "notifier", "timestamps", "wait", "poll", "expect", "timeout_s"},
    "code": {"name", "code", "data_hex", "notifier", "timestamps", "wait", "expect", "timeout_s"},
    "await": {"name", "await", "expect", "timeout_s"},
    "pause": {"name", "pause_s"},
    "heartbeat": {"name", "heartbeat_frame", "payload_hex", "expect", "timeout_s"},
    "raw": {"name", "raw_hex", "expect", "timeout_s"},
}
_SESSION_KEYS = {"host", "port", "timeout_s", "no_reply_wait_s", "mask", "stop_on_fail"}
_EXPECT_KEYS = {"reply", "error", "value", "abs_tol", "rel_tol", "shape", "image", "connection", "max_latency_ms"}
_ERROR_KEYS = {"status", "code", "source", "source_contains"}
_IMAGE_KEYS = {"width", "height", "ni_image_type", "custom_data_keys", "custom_data"}
_VAR = re.compile(r"\$\$|\$\{([A-Za-z_][A-Za-z0-9_]*)\}")
_U32 = 0xFFFFFFFF


class ScriptError(ValueError):
    """An invalid script (or invalid ``--var`` / codec option): reported with exit code 2."""


# ---------------------------------------------------------------------------------------------- script model
@dataclass(frozen=True)
class SessionOptions:
    """``[session]``; the CLI options override it."""

    host: str = "127.0.0.1"
    port: int = 503
    timeout_s: float = 5.0               # reply timeout
    no_reply_wait_s: float = 0.5         # how long ``reply = "none"`` waits
    mask: tuple[str, ...] = ()           # default compare masks
    stop_on_fail: bool = False


@dataclass(frozen=True)
class ErrorExpect:
    """``expect.error``: each field that is not None is checked."""

    status: bool | None = None
    code: int | None = None
    source: str | None = None
    source_contains: str | None = None


@dataclass(frozen=True)
class ImageExpect:
    """``expect.image``: the reply's image decoded with :func:`aloxcam.imaging.codec.decode_provisional`."""

    width: int | None = None
    height: int | None = None
    ni_image_type: int | None = None
    custom_data_keys: tuple[str, ...] | None = None      # exact keys, in order
    custom_data: Mapping[str, str] | None = None         # these keys with these values (others ignored)


@dataclass(frozen=True)
class Poll:
    """``poll``: resend the command until ``condition(value, target)`` holds, at most ``max_tries`` times."""

    max_tries: int
    interval_s: float
    condition: str                       # one of POLL_CONDITIONS
    target: Any


@dataclass(frozen=True)
class Expect:
    """``expect``; ``reply`` None = the step's default (the command's reply kind, see :func:`load_script`)."""

    reply: str | None = None
    error: ErrorExpect | None = None
    value: Any = None
    has_value: bool = False
    abs_tol: float = 0.0
    rel_tol: float = 0.0
    shape: tuple[int, int] | None = None
    image: ImageExpect | None = None
    connection: str | None = None        # "closed" | "open"
    max_latency_ms: float | None = None


@dataclass(frozen=True)
class Step:
    """One ``[[step]]``; ``action`` is ``command``, ``code``, ``await``, ``pause``, ``heartbeat`` or ``raw``."""

    index: int                           # 1-based position in the script
    action: str
    name: str = ""
    command: str | None = None           # label; "#<code>" for code steps; the awaited command for await steps
    data: Any = None                     # payload dataclass (converted at load time)
    data_bytes: bytes | None = None      # data_hex
    notifier: bytes | None = None        # None = "replay-<index>"
    timestamps: tuple[int, int, int] = (0, 0, 0)
    wait: bool = True
    await_notifier: bytes | None = None
    pause_s: float = 0.0
    payload: bytes = b""                 # heartbeat payload_hex / raw_hex bytes
    poll: Poll | None = None
    timeout_s: float | None = None       # overrides [session] timeout_s for this step
    expect: Expect = field(default_factory=Expect)

    @property
    def default_notifier(self) -> bytes:
        return self.notifier if self.notifier is not None else f"replay-{self.index}".encode("ascii")

    @property
    def label(self) -> str:
        """The name shown in the output: ``name`` or a description of the step."""
        if self.name:
            return self.name
        if self.action == "command":
            return str(self.command)
        if self.action == "code":
            return f"code {str(self.command)[1:]}"
        if self.action == "await":
            return f"await {_text(self.await_notifier or b'')}"
        if self.action == "pause":
            return f"pause {self.pause_s:g} s"
        if self.action == "heartbeat":
            return "heartbeat frame" + (f" (+{len(self.payload)} bytes)" if self.payload else "")
        return f"raw {len(self.payload)} bytes"


@dataclass(frozen=True)
class Script:
    """A loaded script: session options, the resolved variables and the steps."""

    session: SessionOptions
    steps: tuple[Step, ...]
    vars: Mapping[str, str] = field(default_factory=dict)
    path: Path | None = None

    @property
    def commands(self) -> frozenset[str]:
        """The ``command`` labels used by the script's command steps (coverage checks)."""
        return frozenset(s.command for s in self.steps if s.action == "command" and s.command is not None)


@dataclass
class StepResult:
    """Outcome of one step. ``reply_kind`` is the kind used to decode the reply (``"none"``: no reply)."""

    step: Step
    ok: bool
    message: str = ""
    reply_kind: str | None = None
    value: Any = None
    error: ErrorCluster | None = None
    latency_ms: float | None = None
    request_hex: str | None = None
    reply_hex: str | None = None
    notifier: bytes | None = None

    def line(self) -> str:
        """``[  7] PASS Cam Get Exposure  DBL 1500.0  err={F,0,""}  2.1 ms``."""
        parts = [f"[{self.step.index:3d}] {'PASS' if self.ok else 'FAIL'} {self.step.label}"]
        if self.reply_kind is not None:
            parts.append(_describe_value(self.reply_kind, self.value))
        if self.error is not None:
            parts.append(_describe_error(self.error))
        if self.latency_ms is not None:
            parts.append(f"{self.latency_ms:.1f} ms")
        text = "  ".join(parts)
        if self.message:
            text += f"  -- {self.message}" if not self.ok else f"  ({self.message})"
        return text

    def record(self) -> dict[str, Any]:
        """The JSON Lines record (design §3.7.2)."""
        return {"step": self.step.index, "name": self.step.label, "command": self.step.command,
                "notifier": None if self.notifier is None else _text(self.notifier),
                "request_hex": self.request_hex, "reply_hex": self.reply_hex, "reply_kind": self.reply_kind,
                "latency_ms": None if self.latency_ms is None else round(self.latency_ms, 3)}


@dataclass
class RunReport:
    """Every step result, in order."""

    results: list[StepResult] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return all(r.ok for r in self.results)

    @property
    def failures(self) -> list[StepResult]:
        return [r for r in self.results if not r.ok]

    def result(self, index: int) -> StepResult:
        """The result of step ``index`` (1-based); :class:`KeyError` when it did not run."""
        for r in self.results:
            if r.step.index == index:
                return r
        raise KeyError(index)

    def summary(self) -> str:
        failed = len(self.failures)
        return (f"{len(self.results)} step(s) run: {len(self.results) - failed} passed, {failed} failed"
                + ("" if failed else " -- OK"))


# ---------------------------------------------------------------------------------------------- loading
def _text(b: bytes) -> str:
    return b.decode("latin-1")


def _substitute(value: Any, variables: Mapping[str, str], where: str) -> Any:
    """``${name}`` → the variable's value in every string (``$$`` → ``$``); recursive in lists and tables."""
    if isinstance(value, str):
        def repl(m: re.Match[str]) -> str:
            name = m.group(1)
            if name is None:
                return "$"
            if name not in variables:
                raise ScriptError(f"{where}: undefined variable ${{{name}}} (define it in [vars] or with --var)")
            return variables[name]
        return _VAR.sub(repl, value)
    if isinstance(value, list):
        return [_substitute(v, variables, where) for v in value]
    if isinstance(value, dict):
        return {k: _substitute(v, variables, where) for k, v in value.items()}
    return value


def _check_keys(table: Mapping[str, Any], allowed: set[str], where: str) -> None:
    unknown = set(table) - allowed
    if unknown:
        raise ScriptError(f"{where}: unknown key(s) {', '.join(sorted(unknown))} "
                          f"(allowed: {', '.join(sorted(allowed))})")


def _table(value: Any, where: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ScriptError(f"{where}: a table is expected, got {value!r}")
    return value


def _number(value: Any, where: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise ScriptError(f"{where}: a number is expected, got {value!r}")
    try:
        number = float(value)
    except ValueError:
        raise ScriptError(f"{where}: a number is expected, got {value!r}") from None
    if not math.isfinite(number) or (minimum is not None and number < minimum):
        raise ScriptError(f"{where}: invalid value {value!r}")
    return number


def _integer(value: Any, where: str, *, minimum: int | None = None, maximum: int | None = None) -> int:
    if isinstance(value, str):
        try:
            value = int(value, 0)
        except ValueError:
            raise ScriptError(f"{where}: an integer is expected, got {value!r}") from None
    if isinstance(value, bool) or not isinstance(value, int):
        raise ScriptError(f"{where}: an integer is expected, got {value!r}")
    if (minimum is not None and value < minimum) or (maximum is not None and value > maximum):
        raise ScriptError(f"{where}: {value} out of range")
    return value


def _boolean(value: Any, where: str) -> bool:
    if not isinstance(value, bool):
        raise ScriptError(f"{where}: true or false expected, got {value!r}")
    return value


def _hex(value: Any, where: str) -> bytes:
    if not isinstance(value, str):
        raise ScriptError(f"{where}: hex text expected, got {value!r}")
    try:
        return bytes.fromhex("".join(value.split()))
    except ValueError as exc:
        raise ScriptError(f"{where}: invalid hex text ({exc})") from None


def _notifier(value: Any, where: str) -> bytes:
    if not isinstance(value, str) or value == "":
        raise ScriptError(f"{where}: a non-empty notifier name is expected, got {value!r}")
    try:
        return value.encode(NOTIFIER_ENCODING)
    except UnicodeEncodeError:
        raise ScriptError(f"{where}: notifier {value!r} is not {NOTIFIER_ENCODING} text") from None


def _parse_session(raw: Any, where: str) -> SessionOptions:
    table = _table(raw, where)
    _check_keys(table, _SESSION_KEYS, where)
    defaults = SessionOptions()
    mask = table.get("mask", [])
    if isinstance(mask, str):
        mask = [mask]
    if not isinstance(mask, list) or any(m not in MASKS for m in mask):
        raise ScriptError(f"{where}.mask: a list of {', '.join(MASKS)} is expected, got {mask!r}")
    host = table.get("host", defaults.host)
    if not isinstance(host, str) or not host:
        raise ScriptError(f"{where}.host: a host name is expected, got {host!r}")
    return SessionOptions(
        host=host,
        port=_integer(table.get("port", defaults.port), f"{where}.port", minimum=0, maximum=0xFFFF),
        timeout_s=_number(table.get("timeout_s", defaults.timeout_s), f"{where}.timeout_s", minimum=0.0),
        no_reply_wait_s=_number(table.get("no_reply_wait_s", defaults.no_reply_wait_s), f"{where}.no_reply_wait_s",
                                minimum=0.0),
        mask=tuple(mask),
        stop_on_fail=_boolean(table.get("stop_on_fail", defaults.stop_on_fail), f"{where}.stop_on_fail"))


def _parse_error_expect(raw: Any, where: str) -> ErrorExpect:
    table = _table(raw, where)
    _check_keys(table, _ERROR_KEYS, where)
    for key in ("source", "source_contains"):
        if key in table and not isinstance(table[key], str):
            raise ScriptError(f"{where}.{key}: text expected, got {table[key]!r}")
    return ErrorExpect(
        status=_boolean(table["status"], f"{where}.status") if "status" in table else None,
        code=_integer(table["code"], f"{where}.code") if "code" in table else None,
        source=table.get("source"), source_contains=table.get("source_contains"))


def _parse_image_expect(raw: Any, where: str) -> ImageExpect:
    table = _table(raw, where)
    _check_keys(table, _IMAGE_KEYS, where)
    keys = table.get("custom_data_keys")
    if keys is not None and (not isinstance(keys, list) or not all(isinstance(k, str) for k in keys)):
        raise ScriptError(f"{where}.custom_data_keys: a list of key names is expected, got {keys!r}")
    data = table.get("custom_data")
    if data is not None:
        data = {str(k): str(v) if not isinstance(v, bool) else ("TRUE" if v else "FALSE")
                for k, v in _table(data, f"{where}.custom_data").items()}
    ints = {k: _integer(table[k], f"{where}.{k}", minimum=0)
            for k in ("width", "height", "ni_image_type") if k in table}
    return ImageExpect(custom_data_keys=None if keys is None else tuple(keys), custom_data=data, **ints)


def _normalise_reply(value: Any, where: str) -> str:
    if not isinstance(value, str):
        raise ScriptError(f"{where}: one of {', '.join(EXPECT_REPLIES)} expected, got {value!r}")
    if value.upper() in REPLY_KINDS:
        return value.upper()
    if value.lower() in ("none", "any"):
        return value.lower()
    raise ScriptError(f"{where}: one of {', '.join(EXPECT_REPLIES)} expected, got {value!r}")


def _parse_expect(raw: Any, where: str) -> Expect:
    table = _table(raw, where)
    _check_keys(table, _EXPECT_KEYS, where)
    reply = _normalise_reply(table["reply"], f"{where}.reply") if "reply" in table else None
    connection = table.get("connection")
    if connection is not None and connection not in CONNECTION_STATES:
        raise ScriptError(f"{where}.connection: one of {', '.join(CONNECTION_STATES)} expected, got {connection!r}")
    shape = table.get("shape")
    if shape is not None:
        if not isinstance(shape, list) or len(shape) != 2:
            raise ScriptError(f"{where}.shape: [rows, cols] expected, got {shape!r}")
        shape = (_integer(shape[0], f"{where}.shape", minimum=0), _integer(shape[1], f"{where}.shape", minimum=0))
    return Expect(
        reply=reply,
        error=_parse_error_expect(table["error"], f"{where}.error") if "error" in table else None,
        value=table.get("value"), has_value="value" in table,
        abs_tol=_number(table.get("abs_tol", 0.0), f"{where}.abs_tol", minimum=0.0),
        rel_tol=_number(table.get("rel_tol", 0.0), f"{where}.rel_tol", minimum=0.0),
        shape=shape,
        image=_parse_image_expect(table["image"], f"{where}.image") if "image" in table else None,
        connection=connection,
        max_latency_ms=(_number(table["max_latency_ms"], f"{where}.max_latency_ms", minimum=0.0)
                        if "max_latency_ms" in table else None))


def _parse_poll(raw: Any, where: str) -> Poll:
    table = _table(raw, where)
    _check_keys(table, {"max_tries", "interval_s", *POLL_CONDITIONS}, where)
    conditions = [k for k in POLL_CONDITIONS if k in table]
    if len(conditions) != 1:
        raise ScriptError(f"{where}: exactly one of {', '.join(POLL_CONDITIONS)} is expected")
    return Poll(max_tries=_integer(table.get("max_tries", 10), f"{where}.max_tries", minimum=1),
                interval_s=_number(table.get("interval_s", 0.1), f"{where}.interval_s", minimum=0.0),
                condition=conditions[0], target=table[conditions[0]])


def _payload_data(command: str, raw: Any, where: str) -> Any:
    spec = COMMANDS[command]
    cls = spec.payload
    if cls is None:
        raise ScriptError(f"{where}: {command} takes no Command Data")
    if not isinstance(raw, dict):
        if len(cls.LV_LAYOUT) != 1:
            raise ScriptError(f"{where}: a table of {cls.__name__} fields is expected, got {raw!r}")
        raw = {cls.LV_LAYOUT[0][0]: raw}        # a bare value for a single-field payload
    try:
        return payload_from_mapping(cls, raw)
    except (ValueError, TypeError) as exc:
        raise ScriptError(f"{where}: {exc}") from None


def _reply_kind_of(step: Step) -> ReplyKind | None:
    """The kind used to decode the step's reply: the command's ``CommandSpec.reply``; ACK for code steps; None when
    unknown before run time (raw steps) or when the step has no reply (pause)."""
    if step.action in ("command", "await") and step.command in COMMANDS:
        return COMMANDS[step.command].reply
    if step.action == "code":
        return ReplyKind.ACK
    return None


def _decode_kind(kind: ReplyKind) -> ReplyKind:
    """The kind a reply is decoded with: the command's own; ACK for a command without reply (the server answers
    such a command only with the 9506 ACK of ``reply_unknown_commands``)."""
    return ReplyKind.ACK if kind is ReplyKind.NONE else kind


def _check_expectation(step: Step, where: str) -> None:
    exp = step.expect
    kind = _reply_kind_of(step)
    if exp.reply in REPLY_KINDS:
        if step.action in ("heartbeat", "pause"):
            raise ScriptError(f"{where}.expect.reply: a {step.action} step gets no reply")
        if kind is ReplyKind.NONE and exp.reply != "ACK":
            raise ScriptError(f"{where}.expect.reply: {step.command} has no reply (only the 9506 ACK of "
                              f"reply_unknown_commands), not {exp.reply}")
        if kind is not None and kind is not ReplyKind.NONE and exp.reply != kind.value:
            raise ScriptError(f"{where}.expect.reply: {step.command} has {kind.value} replies, not {exp.reply}")
    if exp.reply == "none" and (exp.has_value or exp.error or exp.shape or exp.image):
        raise ScriptError(f"{where}.expect: reply = \"none\" cannot check a value, an error, a shape or an image")
    decode_kind = exp.reply if exp.reply in REPLY_KINDS else (_decode_kind(kind).value if kind is not None else None)
    if exp.has_value and decode_kind in ("ACK", ReplyKind.NONE.value):
        raise ScriptError(f"{where}.expect.value: a {decode_kind} reply has no value")
    if exp.shape is not None and decode_kind not in (None, "DBL2D"):
        raise ScriptError(f"{where}.expect.shape: only for DBL2D replies")
    if exp.image is not None and decode_kind not in (None, "IMAGE"):
        raise ScriptError(f"{where}.expect.image: only for IMAGE replies")


def _parse_step(index: int, raw: Any, outstanding: dict[bytes, Step]) -> Step:
    where = f"step {index}"
    table = _table(raw, where)
    actions = [key for key in _ACTIONS if key in table]
    if len(actions) != 1:
        raise ScriptError(f"{where}: exactly one of {', '.join(_ACTIONS)} is expected, found "
                          f"{', '.join(actions) or 'none'}")
    action = _ACTIONS[actions[0]]
    _check_keys(table, _ALLOWED_KEYS[action], where)
    name = table.get("name", "")
    if not isinstance(name, str):
        raise ScriptError(f"{where}.name: text expected, got {name!r}")
    kw: dict[str, Any] = {"index": index, "action": action, "name": name}
    if "expect" in table:
        kw["expect"] = _parse_expect(table["expect"], f"{where}.expect")
    if "timeout_s" in table:
        kw["timeout_s"] = _number(table["timeout_s"], f"{where}.timeout_s", minimum=0.0)
    if action in ("command", "code"):
        if action == "command":
            command = table["command"]
            if not isinstance(command, str) or command not in COMMANDS:
                raise ScriptError(f"{where}.command: unknown Camera Command {command!r}")
            if "data" in table:
                kw["data"] = _payload_data(command, table["data"], f"{where}.data")
            if "poll" in table:
                kw["poll"] = _parse_poll(table["poll"], f"{where}.poll")
        else:
            command = f"#{_integer(table['code'], f'{where}.code', minimum=0)}"
        kw["command"] = command
        if "data_hex" in table:
            kw["data_bytes"] = _hex(table["data_hex"], f"{where}.data_hex")
        if "notifier" in table:
            kw["notifier"] = _notifier(table["notifier"], f"{where}.notifier")
        if "timestamps" in table:
            ts = table["timestamps"]
            if not isinstance(ts, list) or len(ts) != 3:
                raise ScriptError(f"{where}.timestamps: [request, sending, receiving] expected, got {ts!r}")
            kw["timestamps"] = tuple(_integer(t, f"{where}.timestamps", minimum=0, maximum=_U32) for t in ts)
        kw["wait"] = _boolean(table.get("wait", True), f"{where}.wait")
        if not kw["wait"] and ("poll" in kw or "expect" in kw):
            raise ScriptError(f"{where}: a wait = false step cannot poll or expect (use an await step)")
    elif action == "await":
        notifier = _notifier(table["await"], f"{where}.await")
        sender = outstanding.pop(notifier, None)
        if sender is None:
            raise ScriptError(f"{where}.await: no earlier wait = false step sent notifier {table['await']!r}")
        kw["await_notifier"] = notifier
        kw["command"] = sender.command
    elif action == "pause":
        kw["pause_s"] = _number(table["pause_s"], f"{where}.pause_s", minimum=0.0)
    elif action == "heartbeat":
        if table["heartbeat_frame"] is not True:
            raise ScriptError(f"{where}.heartbeat_frame: must be true")
        if "payload_hex" in table:
            kw["payload"] = _hex(table["payload_hex"], f"{where}.payload_hex")
    else:
        kw["payload"] = _hex(table["raw_hex"], f"{where}.raw_hex")
        if not kw["payload"]:
            raise ScriptError(f"{where}.raw_hex: no bytes")
    step = Step(**kw)
    _check_expectation(step, where)
    if action in ("command", "code") and not step.wait:
        outstanding[step.default_notifier] = step
    return step


def load_script(path: Path, *, extra_vars: Mapping[str, str] = {}) -> Script:
    """Read and validate a script (design §3.7.2); :class:`ScriptError` on any problem.

    ``${name}`` is replaced in every string by ``extra_vars`` (``--var``), else ``[vars]``; ``$$`` gives ``$``.
    ``data`` tables are converted with :func:`aloxcam.wire.messages.payload_from_mapping` (a single-field payload
    also accepts a bare value). Additions to the design's format: ``[session] stop_on_fail``; a per-step
    ``timeout_s``; ``expect.connection = "open"`` (the connection must still be open after the step); an IMAGE
    ``value`` is the hex text of the image bytes (``""`` = empty image); ``poll`` conditions ``until_eq``,
    ``until_ne``, ``until_ge``, ``until_gt``, ``until_le``, ``until_lt``. Default ``reply``: the command's reply
    kind (``none`` for commands without reply), ``any`` for ``code`` and ``raw_hex`` steps, ``none`` for heartbeat
    frames.
    """
    path = Path(path)
    try:
        doc = tomllib.loads(path.read_bytes().decode("utf-8"))
    except OSError as exc:
        raise ScriptError(f"{path}: {exc.strerror or exc}") from None
    except (UnicodeDecodeError, tomllib.TOMLDecodeError) as exc:
        raise ScriptError(f"{path}: {exc}") from None
    try:
        _check_keys(doc, {"session", "vars", "step"}, "script")
        raw_vars = _table(doc.get("vars", {}), "[vars]")
        variables: dict[str, str] = {}
        for key, value in raw_vars.items():
            if isinstance(value, (dict, list)):
                raise ScriptError(f"[vars] {key}: a scalar value is expected")
            variables[key] = ("true" if value else "false") if isinstance(value, bool) else str(value)
        variables.update({str(k): str(v) for k, v in extra_vars.items()})
        session = _parse_session(_substitute(doc.get("session", {}), variables, "[session]"), "[session]")
        raw_steps = doc.get("step", [])
        if not isinstance(raw_steps, list):
            raise ScriptError("[[step]] must be an array of tables")
        outstanding: dict[bytes, Step] = {}
        steps = tuple(_parse_step(i, _substitute(raw, variables, f"step {i}"), outstanding)
                      for i, raw in enumerate(raw_steps, start=1))
    except ScriptError as exc:
        raise ScriptError(f"{path}: {exc}") from None
    return Script(session=session, steps=steps, vars=variables, path=path)


# ---------------------------------------------------------------------------------------------- running
def _describe_error(err: ErrorCluster) -> str:
    return f"err={{{'T' if err.status else 'F'},{err.code},{json.dumps(err.source, ensure_ascii=False)}}}"


def _describe_value(kind: str, value: Any) -> str:
    if kind in ("none", "ACK") or value is None:
        return kind
    if kind == "STR":
        return f"STR {json.dumps(value, ensure_ascii=False)}"
    if kind == "BOOL":
        return f"BOOL {'TRUE' if value else 'FALSE'}"
    if kind == "DBL2D":
        rows, cols = np.shape(value)
        return f"DBL2D {rows}x{cols}"
    if kind == "IMAGE":
        data = bytes(value)
        if not data:
            return "IMAGE empty"
        try:
            image = decode_provisional(data)
        except ProtocolError:
            return f"IMAGE {len(data)} bytes"
        return f"IMAGE {image.width}x{image.height} type {image.ni_image_type} ({len(image.custom_data)} keys)"
    return f"{kind} {value!r}"


def _close(actual: Any, expected: Any, abs_tol: float, rel_tol: float) -> bool:
    if isinstance(expected, bool) or isinstance(actual, (bool, np.bool_)):
        return bool(actual) == expected if isinstance(expected, bool) else False
    if isinstance(expected, (int, float)):
        try:
            a = float(actual)
        except (TypeError, ValueError):
            return False
        if math.isnan(a) or math.isnan(expected):
            return math.isnan(a) and math.isnan(expected)
        return a == expected or math.isclose(a, expected, rel_tol=rel_tol, abs_tol=abs_tol)
    return bool(actual == expected)


def _value_problem(kind: str, actual: Any, exp: Expect) -> str | None:
    expected = exp.value
    if kind == "IMAGE":
        if not isinstance(expected, str):
            return f"an IMAGE value is the hex text of the image bytes, got {expected!r}"
        try:
            want = bytes.fromhex("".join(expected.split()))
        except ValueError:
            return f"invalid hex text {expected!r}"
        data = bytes(actual)
        return None if data == want else f"image bytes differ ({len(data)} bytes, {len(want)} expected)"
    if kind == "DBL2D":
        arr = np.asarray(actual, dtype=np.float64)
        want_arr = np.asarray(expected, dtype=np.float64)
        if want_arr.size == 0 and want_arr.ndim == 1:
            want_arr = want_arr.reshape(0, 0)
        if arr.shape != want_arr.shape:
            return f"value shape {arr.shape}, expected {want_arr.shape}"
        if not np.allclose(arr, want_arr, rtol=exp.rel_tol, atol=exp.abs_tol, equal_nan=True):
            return f"value {arr.tolist()!r}, expected {want_arr.tolist()!r}"
        return None
    if kind == "STR":
        return None if actual == expected else f"value {actual!r}, expected {expected!r}"
    if kind == "BOOL" and not isinstance(expected, bool):
        return f"a BOOL value is true or false, got {expected!r}"
    if not _close(actual, expected, exp.abs_tol, exp.rel_tol):
        tol = f" (abs_tol {exp.abs_tol:g}, rel_tol {exp.rel_tol:g})" if exp.abs_tol or exp.rel_tol else ""
        return f"value {actual!r}, expected {expected!r}{tol}"
    return None


def _error_problems(err: ErrorCluster, exp: ErrorExpect) -> list[str]:
    problems: list[str] = []
    if exp.status is not None and err.status != exp.status:
        problems.append(f"error status {err.status}, expected {exp.status}")
    if exp.code is not None and err.code != exp.code:
        problems.append(f"error code {err.code}, expected {exp.code}")
    if exp.source is not None and err.source != exp.source:
        problems.append(f"error source {err.source!r}, expected {exp.source!r}")
    if exp.source_contains is not None and exp.source_contains not in err.source:
        problems.append(f"error source {err.source!r} does not contain {exp.source_contains!r}")
    return problems


def _image_problems(data: bytes, exp: ImageExpect) -> list[str]:
    try:
        image = decode_provisional(data)
    except ProtocolError as exc:
        return [f"image does not decode ({len(data)} bytes): {exc.source}"]
    problems: list[str] = []
    for attr in ("width", "height", "ni_image_type"):
        want = getattr(exp, attr)
        have = getattr(image, attr)
        if want is not None and have != want:
            problems.append(f"image {attr} {have}, expected {want}")
    keys = tuple(image.custom_data)
    if exp.custom_data_keys is not None and keys != exp.custom_data_keys:
        problems.append(f"image custom-data keys {list(keys)}, expected {list(exp.custom_data_keys)}")
    for key, want in (exp.custom_data or {}).items():
        have = image.custom_data.get(key)
        if have != want:
            problems.append(f"image custom data {key!r} = {have!r}, expected {want!r}")
    return problems


@dataclass
class _Sent:
    step: Step
    kind: ReplyKind
    sent_at: float


class _Runner:
    """Runs the steps of one script on one connection."""

    def __init__(self, script: Script, client: AloxClient, enums: WireEnums) -> None:
        self.session = script.session
        self.client = client
        self.enums = enums
        self.datatypes = Datatypes.from_enums(enums)
        self.outstanding: dict[bytes, _Sent] = {}

    def run(self, step: Step) -> StepResult:
        try:
            if step.action in ("command", "code"):
                return self._command(step)
            if step.action == "await":
                return self._await(step)
            if step.action == "pause":
                time.sleep(step.pause_s)
                return StepResult(step, True)
            if step.action == "heartbeat":
                return self._frame(step, heartbeat_frame(self.datatypes, step.payload))
            return self._frame(step, step.payload)
        except ConnectionError as exc:
            return StepResult(step, False, f"{exc}")

    # -- steps
    def _command(self, step: Step) -> StepResult:
        kind = _reply_kind_of(step) or ReplyKind.ACK
        tries = step.poll.max_tries if step.poll is not None else 1
        for attempt in range(1, tries + 1):
            notifier = step.default_notifier + (f"-{attempt}".encode("ascii") if attempt > 1 else b"")
            try:
                frame, notifier = self.client.build_command_frame(step.command or "", step.data, notifier=notifier,
                                                                  ts=step.timestamps, data_bytes=step.data_bytes)
            except ValueError as exc:
                return StepResult(step, False, f"cannot encode the command: {exc}", notifier=notifier)
            sent_at = time.monotonic()
            self.client.send_raw(frame)
            if not step.wait:
                self.outstanding[notifier] = _Sent(step, kind, sent_at)
                return StepResult(step, True, "sent; reply awaited later", request_hex=frame.hex(),
                                  notifier=notifier)
            reply, problems, waited = self._receive(step, kind, notifier)
            if step.poll is not None and attempt < tries and reply is not None and not problems:
                try:
                    value, _ = reply.value(_decode_kind(kind), encoding=self.client.payloads.encoding)
                except ProtocolError:
                    pass                                     # judged below: "does not decode"
                else:
                    if not self._poll_done(step.poll, value):
                        time.sleep(step.poll.interval_s)
                        continue
            result = self._judge(step, kind, notifier, sent_at, frame.hex(), reply, problems, waited)
            if step.poll is not None and result.ok and not self._poll_done(step.poll, result.value):
                result.ok = False
                result.message = (f"poll: {step.poll.condition} {step.poll.target!r} not reached after {attempt} "
                                  f"tries (last value {result.value!r})")
            return result
        raise AssertionError("unreachable")                  # pragma: no cover

    @staticmethod
    def _poll_done(poll: Poll, value: Any) -> bool:
        try:
            return bool(POLL_CONDITIONS[poll.condition](value, poll.target))
        except TypeError:
            return False

    def _await(self, step: Step) -> StepResult:
        notifier = step.await_notifier or b""
        sent = self.outstanding.pop(notifier, None)
        if sent is None:
            return StepResult(step, False, f"nothing was sent with notifier {_text(notifier)!r}", notifier=notifier)
        return self._check(step, sent.kind, notifier, sent.sent_at, None)

    def _frame(self, step: Step, frame: bytes) -> StepResult:
        """Heartbeat / raw frame: the reply is matched by the notifier of a decodable command envelope."""
        notifier, kind = self._envelope_of(frame) if step.action == "raw" else (None, None)
        count = self.client.reply_count
        sent_at = time.monotonic()
        self.client.send_raw(frame)
        if notifier is not None and kind is not None:
            return self._check(step, kind, notifier, sent_at, frame.hex())
        return self._check_unmatched(step, count, frame.hex())

    def _envelope_of(self, frame: bytes) -> tuple[bytes | None, ReplyKind | None]:
        """Notifier and reply kind of a raw ``Command`` frame (also of an undecodable envelope whose code and
        notifier decoded, which the server answers with error 116)."""
        if len(frame) < FRAME_HEADER_SIZE:
            return None, None
        n, datatype = int.from_bytes(frame[:4], "big", signed=True), int.from_bytes(frame[4:6], "big")
        if datatype != self.datatypes.command or n < 2 or len(frame) < FRAME_HEADER_SIZE + n - 2:
            return None, None
        try:
            env = decode_envelope(frame[FRAME_HEADER_SIZE:FRAME_HEADER_SIZE + n - 2], self.enums)
            code, notifier = env.code, env.notifier
        except EnvelopeError as exc:
            code, notifier = exc.code_value, exc.notifier
        if code is None or notifier is None:
            return None, None
        items = self.enums.table("camera_command").items
        spec = COMMANDS.get(items[code]) if 0 <= code < len(items) else None
        kind = spec.reply if spec is not None and spec.reply is not ReplyKind.NONE else ReplyKind.ACK
        return notifier, kind

    # -- checks
    def _default_reply(self, step: Step, kind: ReplyKind) -> str:
        if step.action in ("command", "await"):
            return "none" if kind is ReplyKind.NONE else kind.value
        return "any"

    def _check(self, step: Step, kind: ReplyKind, notifier: bytes, sent_at: float,
               request_hex: str | None) -> StepResult:
        reply, problems, waited = self._receive(step, kind, notifier)
        return self._judge(step, kind, notifier, sent_at, request_hex, reply, problems, waited)

    def _receive(self, step: Step, kind: ReplyKind, notifier: bytes) -> tuple[ReceivedReply | None, list[str], float]:
        """Wait for the closure (``connection = "closed"``) and for the reply: ``timeout_s`` when a reply is
        expected, ``no_reply_wait_s`` for ``none``. Returns the reply, the problems found and the wait used."""
        exp = step.expect
        expected = exp.reply or self._default_reply(step, kind)
        timeout = step.timeout_s if step.timeout_s is not None else self.session.timeout_s
        problems: list[str] = []
        closed_checked = False
        if exp.connection == "closed":
            closed_checked = True
            if not self.client.wait_closed(timeout):
                problems.append(f"connection still open after {timeout:g} s")
        if expected == "none" or (expected == "any" and kind is ReplyKind.NONE):
            wait = 0.0 if closed_checked else self.session.no_reply_wait_s
        else:
            wait = timeout
        return self.client.wait_reply(notifier, wait), problems, wait

    def _judge(self, step: Step, kind: ReplyKind, notifier: bytes, sent_at: float, request_hex: str | None,
               reply: ReceivedReply | None, problems: list[str], wait: float) -> StepResult:
        """Check a received reply (or its absence) against ``step.expect``."""
        exp = step.expect
        expected = exp.reply or self._default_reply(step, kind)
        result = StepResult(step, True, request_hex=request_hex, notifier=notifier)
        if reply is None:
            result.reply_kind = "none"
            if expected in REPLY_KINDS:
                why = f" ({self.client.end_reason})" if not self.client.connected else ""
                problems.append(f"no reply within {wait:g} s{why}")
        else:
            problems += self._check_reply(result, reply, kind, expected, exp, sent_at)
        if exp.connection == "open" and not self.client.connected:
            problems.append(f"connection {self.client.end_reason or 'closed'}")
        result.ok = not problems
        result.message = "; ".join(problems)
        return result

    def _check_reply(self, result: StepResult, reply: ReceivedReply, kind: ReplyKind, expected: str, exp: Expect,
                     sent_at: float) -> list[str]:
        result.reply_hex = reply.frame.hex()
        result.latency_ms = (reply.received_at - sent_at) * 1000.0
        kind = _decode_kind(kind)
        result.reply_kind = kind.value
        try:
            value, err = reply.value(kind, encoding=self.client.payloads.encoding)
        except ProtocolError as exc:
            return [f"reply does not decode as {kind.value}: {exc.source}"]
        result.value, result.error = value, err
        if expected == "none":
            return [f"unexpected {kind.value} reply"]
        problems: list[str] = []
        if exp.error is not None:
            problems += _error_problems(err, exp.error)
        if exp.has_value:
            problem = _value_problem(kind.value, value, exp)
            if problem:
                problems.append(problem)
        if exp.shape is not None and tuple(np.shape(value)) != exp.shape:
            problems.append(f"shape {list(np.shape(value))}, expected {list(exp.shape)}")
        if exp.image is not None:
            problems += _image_problems(bytes(value), exp.image)
        if exp.max_latency_ms is not None and result.latency_ms > exp.max_latency_ms:
            problems.append(f"latency {result.latency_ms:.1f} ms above {exp.max_latency_ms:g} ms")
        return problems

    def _check_unmatched(self, step: Step, count_before: int, request_hex: str) -> StepResult:
        """A step without a notifier (heartbeat, raw bytes): ``none`` means no reply frame arrived at all during
        ``no_reply_wait_s`` (except replies to outstanding ``wait = false`` steps)."""
        exp = step.expect
        expected = exp.reply or ("none" if step.action == "heartbeat" else "any")
        timeout = step.timeout_s if step.timeout_s is not None else self.session.timeout_s
        problems: list[str] = []
        if exp.connection == "closed":
            if not self.client.wait_closed(timeout):
                problems.append(f"connection still open after {timeout:g} s")
        else:
            self.client.wait_closed(self.session.no_reply_wait_s)       # returns early if the connection ends
        if expected in REPLY_KINDS:
            problems.append("the frame has no decodable notifier: a reply cannot be matched")
        new = self.client.reply_count - count_before
        unexpected = [n for n in (self.client.received_notifiers()[-new:] if new > 0 else [])
                      if n not in self.outstanding]
        if expected == "none" and unexpected:
            problems.append(f"unexpected reply for notifier(s) {', '.join(repr(_text(n)) for n in unexpected)}")
        if exp.connection == "open" and not self.client.connected:
            problems.append(f"connection {self.client.end_reason or 'closed'}")
        return StepResult(step, not problems, "; ".join(problems), reply_kind="none", request_hex=request_hex)


def run_script(script: Script, *, host: str | None = None, port: int | None = None, enums: WireEnums | None = None,
               payloads: PayloadCodec | None = None, record: Path | None = None,
               on_result: Callable[[StepResult], None] | None = None) -> RunReport:
    """Connect, run every step in order and return the results (design §3.7.2).

    ``host`` / ``port`` override ``[session]``; ``enums`` defaults to the packaged table and ``payloads`` to
    ``PayloadCodec(enums)``. ``record`` writes one JSON Lines record per step. A connection failure raises
    ``OSError`` (nothing is run); ``[session] stop_on_fail`` stops after the first failed step.
    """
    enums = enums if enums is not None else WireEnums.load()
    payloads = payloads if payloads is not None else PayloadCodec(enums)
    session = script.session
    client = AloxClient(host if host is not None else session.host, port if port is not None else session.port,
                        enums=enums, payloads=payloads, timeout_s=session.timeout_s)
    client.connect()
    runner = _Runner(script, client, enums)
    report = RunReport()
    out: TextIO | None = None
    try:
        if record is not None:
            out = open(record, "w", encoding="utf-8", newline="\n")
        for step in script.steps:
            result = runner.run(step)
            report.results.append(result)
            if out is not None:
                out.write(json.dumps(result.record(), ensure_ascii=False) + "\n")
                out.flush()
            if on_result is not None:
                on_result(result)
            if not result.ok and session.stop_on_fail:
                break
    finally:
        client.close()
        if out is not None:
            out.close()
    return report


# ---------------------------------------------------------------------------------------------- record / compare
def _records(path: Path) -> Iterator[dict[str, Any]]:
    with open(path, encoding="utf-8") as f:
        for number, line in enumerate(f, start=1):
            if line.strip():
                try:
                    record = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise ScriptError(f"{path}:{number}: not a JSON record ({exc})") from None
                if not isinstance(record, dict):
                    raise ScriptError(f"{path}:{number}: not a JSON object")
                yield record


def mask_reply(frame: bytes, masks: Sequence[str], *, command: str | None = None,
               reply_kind: str | None = None) -> bytes:
    """``frame`` with the masked fields zeroed (design §3.7.2): ``tick`` → ``TCP Sending timestamp`` and the U32
    value of ``Get Heartbeat``; ``image`` → the image bytes (replaced by an empty string) and the request,
    receiving, pre- and post-flattening timestamps of IMAGE replies. A frame that does not decode is unchanged."""
    if len(frame) < FRAME_HEADER_SIZE:
        return frame
    datatype = int.from_bytes(frame[4:6], "big")
    try:
        data = decode_response_data(frame[FRAME_HEADER_SIZE:])
    except ProtocolError:
        return frame
    flattened = data.flattened
    ts = list(data.ts.as_tuple())
    if "tick" in masks:
        ts[5] = 0
        if command == "Get Heartbeat" and len(flattened) >= 4:
            flattened = bytes(4) + flattened[4:]
    if "image" in masks and reply_kind == ReplyKind.IMAGE.value:
        r = Reader(flattened)
        try:
            r.string_bytes()
            flattened = bytes(4) + flattened[r.pos:]
        except ProtocolError:
            pass
        ts[0] = ts[2] = ts[3] = ts[4] = 0
    return encode_frame(datatype, encode_response_data(data.notifier, flattened, ResponseTimestamps(*ts)))


def _first_difference(a: bytes, b: bytes) -> str:
    offset = next((i for i, (x, y) in enumerate(zip(a, b)) if x != y), min(len(a), len(b)))
    lo = max(0, offset - 4)
    return (f"first difference at byte {offset} ({len(a)} vs {len(b)} bytes): "
            f"{a[lo:offset + 12].hex(' ')} | reference {b[lo:offset + 12].hex(' ')}")


def compare_records(actual: Path, reference: Path, masks: Sequence[str]) -> list[str]:
    """Pair the records of two ``--record`` files by step number and compare ``reply_hex`` after
    :func:`mask_reply`; returns one message per difference (empty = identical)."""
    unknown = set(masks) - set(MASKS)
    if unknown:
        raise ScriptError(f"unknown mask(s) {', '.join(sorted(unknown))} (known: {', '.join(MASKS)})")
    got = {r.get("step"): r for r in _records(Path(actual))}
    ref = {r.get("step"): r for r in _records(Path(reference))}
    differences: list[str] = []
    for step in sorted(set(got) | set(ref), key=lambda s: (s is None, s if isinstance(s, int) else 0)):
        a, b = got.get(step), ref.get(step)
        if a is None or b is None:
            differences.append(f"step {step}: only in the {'reference' if a is None else 'actual'} record")
            continue
        label = f"step {step} ({a.get('name') or a.get('command')})"
        if a.get("command") != b.get("command"):
            differences.append(f"{label}: command {a.get('command')!r}, reference {b.get('command')!r}")
            continue
        ha, hb = a.get("reply_hex"), b.get("reply_hex")
        if ha is None and hb is None:
            continue
        if ha is None or hb is None:
            differences.append(f"{label}: {'no reply' if ha is None else 'a reply'}, reference "
                               f"{'no reply' if hb is None else 'a reply'}")
            continue
        try:
            fa, fb = bytes.fromhex(ha), bytes.fromhex(hb)
        except ValueError:
            differences.append(f"{label}: reply_hex is not hex text")
            continue
        ma = mask_reply(fa, masks, command=a.get("command"), reply_kind=a.get("reply_kind"))
        mb = mask_reply(fb, masks, command=b.get("command"), reply_kind=b.get("reply_kind"))
        if ma != mb:
            differences.append(f"{label}: reply differs; {_first_difference(ma, mb)}")
    return differences


# ---------------------------------------------------------------------------------------------- CLI
def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="replay_client.py",
                                description="Run a replay script against an ALOX camera server (Python or LabVIEW).")
    p.add_argument("script", type=Path, metavar="SCRIPT.toml")
    p.add_argument("--host", help="server address (default: [session] host, else 127.0.0.1)")
    p.add_argument("--port", type=int, help="server port (default: [session] port, else 503)")
    p.add_argument("--timeout", type=float, metavar="S", help="reply timeout in seconds ([session] timeout_s)")
    p.add_argument("--var", action="append", default=[], metavar="NAME=VALUE", help="set a script variable")
    p.add_argument("--settings-ini", type=Path, metavar="PATH",
                   help="read [wire] (enum overrides, layouts, encoding) from an alox_camera_server.ini")
    p.add_argument("--enums", type=Path, metavar="PATH", help="wire enum override file (wire_enums.toml format)")
    p.add_argument("--templates", type=Path, metavar="PATH",
                   help='TOML of captured variant headers: "<command>" = "<hex>"')
    p.add_argument("--record", type=Path, metavar="OUT.jsonl", help="record every request and reply (JSON Lines)")
    p.add_argument("--compare", type=Path, metavar="REF.jsonl", help="compare the replies with a recording")
    p.add_argument("--mask", action="extend", nargs="+", choices=MASKS, default=None,
                   help="fields ignored by --compare (default: [session] mask)")
    p.add_argument("--stop-on-fail", action="store_true", help="stop at the first failed step")
    p.add_argument("--quiet", action="store_true", help="print only failed steps and the summary")
    return p


def _parse_vars(items: Sequence[str]) -> dict[str, str]:
    out: dict[str, str] = {}
    for item in items:
        name, sep, value = item.partition("=")
        if not sep or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name):
            raise ScriptError(f"--var {item!r}: NAME=VALUE expected")
        out[name] = value
    return out


def _load_templates(path: Path) -> dict[str, str]:
    try:
        doc = tomllib.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, tomllib.TOMLDecodeError) as exc:
        raise ScriptError(f"--templates {path}: {exc}") from None
    templates: dict[str, str] = {}
    for name, value in doc.items():
        if name not in COMMANDS:
            raise ScriptError(f"--templates {path}: unknown command {name!r}")
        if not isinstance(value, str):
            raise ScriptError(f"--templates {path}: {name}: hex text expected")
        templates[name] = value
    return templates


def _codecs(args: argparse.Namespace) -> tuple[WireEnums, PayloadCodec]:
    """Enum table and payload codec from ``--settings-ini`` ``[wire]``, ``--enums`` and ``--templates``."""
    wire = WireOptions()
    base: Path | None = None
    if args.settings_ini is not None:
        if not args.settings_ini.is_file():
            raise ScriptError(f"--settings-ini {args.settings_ini}: no such file")
        from aloxcam.config.server_ini import load_server_config

        def warn(message: str) -> None:
            print(f"replay_client: warning: {message}", file=sys.stderr)

        wire = load_server_config(args.settings_ini, warn=warn).wire
        base = args.settings_ini.parent
    override = args.enums
    if override is None and wire.enums_file is not None:
        override = wire.enums_file if wire.enums_file.is_absolute() or base is None else base / wire.enums_file
    enums = WireEnums.load(override_file=override, items=wire.enum_items, sizes=wire.enum_sizes)
    templates = _load_templates(args.templates) if args.templates is not None else {}
    payloads = PayloadCodec(enums, wire.layouts, init_data_fallback=wire.init_data_fallback, encoding=wire.encoding,
                            templates=templates)
    return enums, payloads


def main(argv: Sequence[str] | None = None) -> int:
    """The command line (design §3.7.2); returns the exit code (0 pass, 1 fail / difference, 2 usage or
    connection error)."""
    try:
        args = _build_parser().parse_args(argv)
    except SystemExit as exc:
        return exc.code if isinstance(exc.code, int) else EXIT_USAGE
    try:
        script = load_script(args.script, extra_vars=_parse_vars(args.var))
        enums, payloads = _codecs(args)
        if args.port is not None and not 0 <= args.port <= 0xFFFF:
            raise ScriptError(f"--port {args.port}: outside 0..65535")
    except (ScriptError, ConfigError) as exc:
        message = exc.source if isinstance(exc, ConfigError) else str(exc)
        print(f"replay_client: {message}", file=sys.stderr)
        return EXIT_USAGE
    session = script.session
    session = dataclasses.replace(
        session, host=args.host if args.host is not None else session.host,
        port=args.port if args.port is not None else session.port,
        timeout_s=args.timeout if args.timeout is not None else session.timeout_s,
        stop_on_fail=args.stop_on_fail or session.stop_on_fail)
    script = dataclasses.replace(script, session=session)

    def show(result: StepResult) -> None:
        if not args.quiet or not result.ok:
            print(result.line(), flush=True)

    with tempfile.TemporaryDirectory(prefix="replay-") as tmp:
        record = args.record
        if record is None and args.compare is not None:
            record = Path(tmp) / "actual.jsonl"
        try:
            report = run_script(script, enums=enums, payloads=payloads, record=record, on_result=show)
        except OSError as exc:
            print(f"replay_client: cannot connect to {session.host}:{session.port}: {exc}", file=sys.stderr)
            return EXIT_USAGE
        print(report.summary())
        code = EXIT_OK if report.ok else EXIT_FAIL
        if args.compare is not None:
            masks = tuple(args.mask) if args.mask is not None else session.mask
            try:
                differences = compare_records(record, args.compare, masks)
            except (OSError, ScriptError) as exc:
                print(f"replay_client: compare: {exc}", file=sys.stderr)
                return EXIT_USAGE
            for difference in differences:
                print(f"DIFF {difference}")
            print(f"compare with {args.compare} (masks: {', '.join(masks) or 'none'}): "
                  f"{len(differences)} difference(s)")
            if differences:
                code = EXIT_FAIL
    return code


if __name__ == "__main__":
    sys.exit(main())
