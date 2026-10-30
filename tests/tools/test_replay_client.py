"""``aloxcam.client.AloxClient`` and ``tools/replay_client.py``: script parsing, expectation evaluation, record /
compare masking and the command line, against a fake server on a loopback socket.

Spec: docs/phase1-design.md §3.7 (client API, CLI, script format, record / compare), §4.2 (WP7 acceptance tests);
02 §10 (notifier correlation), 02 §5.4 (heartbeat frame), 02 §8 (reply payloads).

The fake server decodes every ``Command`` frame with the wire layer and lets each test choose the replies; it listens
on an ephemeral loopback port because ``run_script`` connects by host and port.
"""

from __future__ import annotations

import json
import os
import socket
import struct
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from aloxcam.client import AloxClient
from aloxcam.dispatch import ReplyKind
from aloxcam.errors import NO_ERROR, ErrorCluster, ProtocolError
from aloxcam.imagedata import ImageData
from aloxcam.imaging.codec import ProvisionalCodec
from aloxcam.lv.flatten import Reader
from aloxcam.lv.path import LVPath
from aloxcam.lv.variant import EMPTY_VARIANT_HEADER, read_variant_header, synth_variant_header
from aloxcam.testing import packaged_enums
from aloxcam.wire.enums import build_routing
from aloxcam.wire.messages import COMMANDS, CameraInitData, CameraSetExposureData, PayloadCodec, ZLimitData
from aloxcam.wire.protocol import (
    CommandEnvelope,
    Datatypes,
    EnvelopeError,
    ResponseTimestamps,
    decode_envelope,
    encode_frame,
    flatten_reply,
    response_frame,
)
from replay_client import (
    Script,
    ScriptError,
    StepResult,
    compare_records,
    load_script,
    main,
    mask_reply,
    run_script,
)

ENUMS = packaged_enums()
DT = Datatypes.from_enums(ENUMS)
PAYLOADS = PayloadCodec(ENUMS)
ROUTING = build_routing(ENUMS)
CLOSE = object()
TIMEOUT = 5.0
HEADER = '[session]\ntimeout_s = 2.0\nno_reply_wait_s = 0.05\n'
#: For steps that must see an unexpected reply or a closure inside the no-reply window, even on a loaded machine.
WIDE_HEADER = '[session]\ntimeout_s = 2.0\nno_reply_wait_s = 0.5\n'


# ---------------------------------------------------------------------------------------------- fake server
@dataclass
class Got:
    """One frame received by the fake server."""

    datatype: int
    payload: bytes
    env: CommandEnvelope | None = None
    name: str | None = None
    data: Any = None


Handler = Callable[[Got, "FakeServer"], Any]


class FakeServer:
    """A loopback stand-in for the ALOX server: one client at a time; ``handler(got, fake)`` returns the frames to
    send back, :data:`CLOSE` to close the connection, or None."""

    def __init__(self, handler: Handler | None = None) -> None:
        self.handler: Handler = handler or (lambda got, fake: None)
        self.listener = socket.create_server(("127.0.0.1", 0))
        self.listener.settimeout(0.05)
        self.port = self.listener.getsockname()[1]
        self.received: list[Got] = []
        self.connections = 0
        self.cond = threading.Condition()
        self._conn: socket.socket | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, name="fake-server", daemon=True)
        self._thread.start()

    def _serve(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _ = self.listener.accept()
            except TimeoutError:
                continue
            except OSError:
                return
            conn.settimeout(None)
            with self.cond:
                self.connections += 1
                self._conn = conn
                self.cond.notify_all()
            try:
                self._read(conn)
            except OSError:
                pass
            finally:
                with self.cond:
                    self._conn = None
                conn.close()

    @staticmethod
    def _recv(conn: socket.socket, n: int) -> bytes | None:
        buf = b""
        while len(buf) < n:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def _read(self, conn: socket.socket) -> None:
        while True:
            header = self._recv(conn, 6)
            if header is None:
                return
            n, datatype = struct.unpack(">iH", header)
            payload = self._recv(conn, n - 2) if n > 2 else b""
            if payload is None:
                return
            got = Got(datatype, payload)
            if datatype == DT.command:
                try:
                    got.env = decode_envelope(payload, ENUMS)
                    got.name = ROUTING.name(got.env.code)
                    spec = COMMANDS.get(got.name)
                    if spec is not None:
                        try:
                            got.data = PAYLOADS.decode(spec, got.env.data)
                        except ProtocolError:
                            got.data = "undecodable"
                except EnvelopeError:
                    pass
            with self.cond:
                self.received.append(got)
                self.cond.notify_all()
            action = self.handler(got, self)
            if action is CLOSE:
                conn.shutdown(socket.SHUT_RDWR)
                return
            for frame in action or ():
                conn.sendall(frame)

    def send(self, frame: bytes) -> None:
        """Send a frame on the current connection (from the test thread)."""
        with self.cond:
            conn = self._conn
        assert conn is not None
        conn.sendall(frame)

    def wait_received(self, count: int, timeout: float = TIMEOUT) -> list[Got]:
        with self.cond:
            assert self.cond.wait_for(lambda: len(self.received) >= count, timeout), self.received
            return list(self.received)

    def commands(self) -> list[Got]:
        with self.cond:
            return [g for g in self.received if g.env is not None]

    def close(self) -> None:
        self._stop.set()
        self.listener.close()
        with self.cond:
            conn = self._conn
        if conn is not None:
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self._thread.join(TIMEOUT)


def reply(got: Got, kind: ReplyKind, value: Any = None, err: ErrorCluster = NO_ERROR,
          ts: ResponseTimestamps = ResponseTimestamps(tcp_sending=7)) -> bytes:
    assert got.env is not None
    return response_frame(DT, got.env.notifier, flatten_reply(kind, value, err), ts)


def table(entries: dict[str, Any]) -> Handler:
    """Handler from ``{command: (kind, value[, err]) | callable | CLOSE}``; other commands get no reply."""
    def handler(got: Got, fake: FakeServer) -> Any:
        entry = entries.get(got.name or "")
        if entry is None:
            return None
        if entry is CLOSE:
            return CLOSE
        if callable(entry):
            return entry(got, fake)
        kind, value, *err = entry
        return [reply(got, kind, value, *err)]
    return handler


@pytest.fixture
def fake() -> Any:
    servers: list[FakeServer] = []

    def make(handler: Handler | None = None) -> FakeServer:
        server = FakeServer(handler)
        servers.append(server)
        return server

    yield make
    for server in servers:
        server.close()


def write(tmp_path: Path, text: str, name: str = "script.toml") -> Path:
    path = tmp_path / name
    path.write_text(text, encoding="utf-8")
    return path


def run(server: FakeServer, tmp_path: Path, text: str, *, header: str = HEADER, **kw: Any) -> Any:
    return run_script(load_script(write(tmp_path, header + text)), host="127.0.0.1", port=server.port, **kw)


# ---------------------------------------------------------------------------------------------- script parsing
def test_load_script_vars_inline_tables_and_sub_tables(tmp_path: Path) -> None:
    path = write(tmp_path, """
[session]
host = "${host}"
port = 5030
mask = ["tick"]

[vars]
cam = "cam0"
host = "localhost"
folder = "/data/x"

[[step]]
name = "setup ${cam} $$1"
command = "Cam Setup"
data = { camera_type = "Low Mag.", camera_name = "${cam}", number_of_buffers = 5, continuous_mode = true }
notifier = "n-${cam}"
timestamps = [1000, 1001, 0]
expect = { reply = "ACK", error = { status = false, code = 0 } }

[[step]]
command = "Cam Setup"
[step.data]
camera_type = "Low Mag."
camera_name = "${cam}"
number_of_buffers = 5
continuous_mode = true
[step.expect]
reply = "ACK"
error = { status = false, code = 0 }

[[step]]
command = "Cam Set FlatField"
data = { ffc_path = "${folder}/f.png", dfc_path = 'C:\\x\\d.png' }

[[step]]
command = "Z Set Min Limit"
data = 0.5
""")
    script = load_script(path, extra_vars={"cam": "cam9"})
    assert script.session.host == "localhost" and script.session.port == 5030 and script.session.mask == ("tick",)
    first, second, flat, limit = script.steps
    assert first.name == "setup cam9 $1" and first.notifier == b"n-cam9" and first.timestamps == (1000, 1001, 0)
    assert first.data == CameraInitData("Low Mag.", "cam9", 5, True)
    assert (second.data, second.expect) == (first.data, first.expect)       # sub-tables = inline tables
    assert first.expect.error is not None and first.expect.error.code == 0
    assert flat.data.ffc_path == LVPath(0, ("data", "x", "f.png"))
    assert flat.data.dfc_path == LVPath(0, ("C", "x", "d.png"))
    assert limit.data == ZLimitData(0.5)                                      # bare value of a 1-field payload
    assert script.commands == {"Cam Setup", "Cam Set FlatField", "Z Set Min Limit"}
    assert first.default_notifier == b"n-cam9" and second.default_notifier == b"replay-2"
    assert isinstance(script, Script) and script.vars["cam"] == "cam9"


def test_load_script_actions_await_poll_raw_and_data_hex(tmp_path: Path) -> None:
    script = load_script(write(tmp_path, """
[[step]]
command = "Cam Get Autofocus Progress"
data = { camera_type = "High Mag." }
poll = { max_tries = 50, interval_s = 0.1, until_ge = 100.0 }
expect = { reply = "DBL" }

[[step]]
command = "Z Get Position"
notifier = "fixed-name"
wait = false

[[step]]
code = 32767
wait = false

[[step]]
await = "fixed-name"
expect = { reply = "DBL", value = 1500.0, abs_tol = 1e-9 }

[[step]]
await = "replay-3"

[[step]]
pause_s = 0.2

[[step]]
heartbeat_frame = true
payload_hex = "ab cd"
expect = { connection = "closed" }

[[step]]
raw_hex = "00 00 00 02\\n 00 03"
expect = { reply = "none" }

[[step]]
command = "Cam Set Exposure"
data_hex = "00 01"
timeout_s = 1.5
"""))
    poll, sent, code, awaited, awaited_code, pause, heartbeat, raw, malformed = script.steps
    assert poll.poll is not None and (poll.poll.max_tries, poll.poll.interval_s, poll.poll.condition,
                                      poll.poll.target) == (50, 0.1, "until_ge", 100.0)
    assert not sent.wait and sent.default_notifier == b"fixed-name"
    assert code.action == "code" and code.command == "#32767" and code.label == "code 32767"
    assert awaited.action == "await" and awaited.command == "Z Get Position" and awaited.expect.value == 1500.0
    assert awaited_code.command == "#32767" and awaited_code.await_notifier == b"replay-3"
    assert pause.pause_s == 0.2 and pause.label == "pause 0.2 s"
    assert heartbeat.payload == b"\xab\xcd" and heartbeat.expect.connection == "closed"
    assert raw.payload == bytes.fromhex("000000020003") and raw.label == "raw 6 bytes"
    assert malformed.data_bytes == b"\x00\x01" and malformed.timeout_s == 1.5
    assert script.commands == {"Cam Get Autofocus Progress", "Z Get Position", "Cam Set Exposure"}


@pytest.mark.parametrize("text, message", [
    ('[[step]]\ncommand = "Get Heartbeat"\npause_s = 1', "exactly one of"),
    ('[[step]]\nname = "x"', "exactly one of"),
    ('[[step]]\ncommand = "Get Heartbeat"\nbogus = 1', "unknown key(s) bogus"),
    ('[[step]]\ncommand = "Cam Explode"', "unknown Camera Command"),
    ('[[step]]\ncommand = "Cam Setup"\nexpect = { reply = "DBL" }', "has ACK replies, not DBL"),
    ('[[step]]\ncommand = "Exit"\nexpect = { reply = "U32" }', "has no reply"),
    ('[[step]]\ncommand = "Cam Setup"\nexpect = { value = 1 }', "a ACK reply has no value"),
    ('[[step]]\ncommand = "Cam Get Exposure"\nexpect = { shape = [1, 2] }', "only for DBL2D"),
    ('[[step]]\ncommand = "Cam Get Exposure"\nexpect = { image = { width = 1 } }', "only for IMAGE"),
    ('[[step]]\ncommand = "Get Heartbeat"\nexpect = { reply = "none", value = 3 }', 'reply = "none" cannot'),
    ('[[step]]\ncommand = "Get Heartbeat"\nexpect = { reply = "maybe" }', "one of ACK"),
    ('[[step]]\ncommand = "Get Heartbeat"\nexpect = { connection = "half" }', "closed, open"),
    ('[[step]]\nawait = "nobody"', "no earlier wait = false step"),
    ('[[step]]\ncommand = "Get Heartbeat"\nnotifier = "a"\n[[step]]\nawait = "a"', "no earlier wait = false"),
    ('[[step]]\ncommand = "Get Heartbeat"\nwait = false\nexpect = { reply = "U32" }', "cannot poll or expect"),
    ('[[step]]\ncommand = "Get Heartbeat"\npoll = { max_tries = 2 }', "exactly one of until_eq"),
    ('[[step]]\nraw_hex = "zz"', "invalid hex"),
    ('[[step]]\ncommand = "Cam Setup"\ndata = { colour = 1 }', "unknown field(s) colour"),
    ('[[step]]\ncommand = "Get Heartbeat"\ndata = { x = 1 }', "takes no Command Data"),
    ('[[step]]\nheartbeat_frame = false', "must be true"),
    ('[[step]]\nheartbeat_frame = true\nexpect = { reply = "ACK" }', "gets no reply"),
    ('[[step]]\ncommand = "Get Heartbeat"\ntimestamps = [1, 2]', "[request, sending, receiving]"),
    ('[[step]]\ncommand = "Get Heartbeat"\nnotifier = "${missing}"', "undefined variable ${missing}"),
    ('[session]\nmask = ["pixels"]', "mask"),
    ('[session]\nport = 70000', "out of range"),
    ('[sessions]\nport = 1', "unknown key(s) sessions"),
    ('[[step]\n', "line"),
])
def test_load_script_errors(tmp_path: Path, text: str, message: str) -> None:
    with pytest.raises(ScriptError) as excinfo:
        load_script(write(tmp_path, text))
    assert message in str(excinfo.value)


def test_load_script_missing_file(tmp_path: Path) -> None:
    with pytest.raises(ScriptError, match="missing.toml"):
        load_script(tmp_path / "missing.toml")


def test_step_result_line_format() -> None:
    script = load_script_from_text('[[step]]\ncommand = "Get Heartbeat"\n')
    step = script.steps[0]
    result = StepResult(step, True, reply_kind="DBL", value=1500.0, error=NO_ERROR, latency_ms=2.08)
    assert result.line() == '[  1] PASS Get Heartbeat  DBL 1500.0  err={F,0,""}  2.1 ms'
    failed = StepResult(step, False, "no reply within 2 s", reply_kind="none")
    assert failed.line() == "[  1] FAIL Get Heartbeat  none  -- no reply within 2 s"
    err = StepResult(step, True, reply_kind="ACK", error=ErrorCluster(True, 10, 'Controller "x"\nerror'))
    assert err.line() == '[  1] PASS Get Heartbeat  ACK  err={T,10,"Controller \\"x\\"\\nerror"}'


def load_script_from_text(text: str) -> Script:
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        return load_script(write(Path(tmp), text))


# ---------------------------------------------------------------------------------------------- running
def test_values_errors_and_tolerances(fake: Any, tmp_path: Path) -> None:
    server = fake(table({
        "Get Heartbeat": (ReplyKind.U32, 123),
        "Cam Get Exposure": (ReplyKind.DBL, 1500.0000001),
        "Cam Setup": (ReplyKind.ACK, None),
        "Z Get Serial": (ReplyKind.STR, "abc"),
        "Cam Get Acquisition Status": (ReplyKind.BOOL, True),
        "Cam Get Autofocus Data": (ReplyKind.DBL2D, np.array([[1.0, 2.0], [3.0, 4.0]])),
        "Z Move Absolute": (ReplyKind.ACK, None, ErrorCluster(True, 9405, "Z move out of range")),
    }))
    report = run(server, tmp_path, """
[[step]]
command = "Get Heartbeat"
expect = { reply = "U32", value = 123, error = { status = false, code = 0, source = "" } }
[[step]]
command = "Cam Get Exposure"
expect = { value = 1500.0, abs_tol = 1e-6 }
[[step]]
command = "Cam Get Exposure"
expect = { value = 1500.0 }
[[step]]
command = "Cam Get Exposure"
expect = { value = 1500.0, rel_tol = 1e-9 }
[[step]]
command = "Cam Setup"
expect = { error = { status = false, code = 0 } }
[[step]]
command = "Z Get Serial"
expect = { value = "abd" }
[[step]]
command = "Cam Get Acquisition Status"
expect = { value = true }
[[step]]
command = "Cam Get Autofocus Data"
expect = { shape = [2, 2], value = [[1.0, 2.0], [3.0, 4.0]] }
[[step]]
command = "Cam Get Autofocus Data"
expect = { shape = [0, 0] }
[[step]]
command = "Z Move Absolute"
expect = { error = { status = true, code = 9405, source_contains = "out of range" } }
[[step]]
command = "Z Move Absolute"
expect = { error = { status = false, code = 1, source = "x", source_contains = "zz" } }
""")
    assert [r.ok for r in report.results] == [True, True, False, True, True, False, True, True, False, True, False]
    assert "value 1500.0000001, expected 1500.0" in report.result(3).message
    assert report.result(6).message == "value 'abc', expected 'abd'"
    assert "shape [2, 2], expected [0, 0]" in report.result(9).message
    message = report.result(11).message
    for part in ("error status True, expected False", "error code 9405, expected 1", "expected 'x'",
                 "does not contain 'zz'"):
        assert part in message
    assert report.result(1).value == 123 and report.result(1).reply_kind == "U32"
    assert report.result(1).latency_ms is not None and report.result(1).latency_ms >= 0
    assert not report.ok and len(report.failures) == 4
    assert report.summary() == "11 step(s) run: 7 passed, 4 failed"


def test_numeric_values_from_variables(fake: Any, tmp_path: Path) -> None:
    server = fake(table({"Get Heartbeat": (ReplyKind.U32, 123), "Z Get Position": (ReplyKind.DBL, 12.5)}))
    report = run(server, tmp_path, """
[vars]
pos = "12.5"
[[step]]
command = "Z Get Position"
expect = { value = "${pos}", abs_tol = 1e-9 }
[[step]]
command = "Get Heartbeat"
expect = { value = "123" }
[[step]]
command = "Z Get Position"
expect = { value = "12.6" }
[[step]]
command = "Z Get Position"
expect = { value = "twelve" }
""")
    assert [r.ok for r in report.results] == [True, True, False, False]
    assert report.result(3).message == "value 12.5, expected 12.6"
    assert report.result(4).message == "a DBL value is a number, got 'twelve'"


def test_none_any_and_missing_replies(fake: Any, tmp_path: Path) -> None:
    server = fake(table({"Get Heartbeat": (ReplyKind.U32, 1), "Cam Refocus Run": (ReplyKind.ACK, None)}))
    report = run(server, tmp_path, header=WIDE_HEADER, text="""
[[step]]
command = "Z Move Absolute"
expect = { reply = "none" }
[[step]]
command = "Z Move Absolute"
timeout_s = 0.1
[[step]]
command = "Z Move Absolute"
expect = { reply = "any" }
timeout_s = 0.1
[[step]]
command = "Get Heartbeat"
expect = { reply = "none" }
[[step]]
command = "Get Heartbeat"
expect = { reply = "any" }
[[step]]
command = "Cam Get Image"
[[step]]
name = "9506 ACK of reply_unknown_commands"
command = "Cam Refocus Run"
expect = { reply = "ACK" }
[[step]]
command = "Cam Refocus Run"
""")
    assert [r.ok for r in report.results] == [True, False, True, False, True, True, True, False]
    assert report.result(2).message == "no reply within 0.1 s" and report.result(2).reply_kind == "none"
    assert report.result(4).message == "unexpected U32 reply"
    assert report.result(8).message == "unexpected ACK reply"


def test_await_matches_out_of_order_replies(fake: Any, tmp_path: Path) -> None:
    held: list[Got] = []

    def hold(got: Got, fake: FakeServer) -> Any:
        held.append(got)
        return None

    def release(got: Got, fake: FakeServer) -> Any:                  # replies of the held commands, reversed
        frames = [reply(g, ReplyKind.DBL, float(i)) for i, g in enumerate(held)]
        return list(reversed(frames)) + [reply(got, ReplyKind.U32, 9)]

    server = fake(table({"Z Get Position": hold, "Cam Get Exposure": hold, "Get Heartbeat": release}))
    report = run(server, tmp_path, """
[[step]]
command = "Z Get Position"
notifier = "first"
wait = false
[[step]]
command = "Cam Get Exposure"
wait = false
[[step]]
command = "Get Heartbeat"
expect = { value = 9 }
[[step]]
await = "replay-2"
expect = { value = 1.0 }
[[step]]
await = "first"
expect = { reply = "DBL", value = 0.0 }
""")
    assert report.ok, [r.line() for r in report.failures]
    assert report.result(1).message == "sent; reply awaited later" and report.result(1).reply_kind is None
    assert report.result(4).step.command == "Cam Get Exposure"
    notifiers = [g.env.notifier for g in server.commands()]  # type: ignore[union-attr]
    assert notifiers == [b"first", b"replay-2", b"replay-3"]


def test_poll_until_the_condition_holds(fake: Any, tmp_path: Path) -> None:
    progress = iter([0.0, 50.0, 100.0, 100.0, 0.0, 10.0])
    server = fake(table({"Cam Get Autofocus Progress": lambda got, f: [reply(got, ReplyKind.DBL, next(progress))]}))
    report = run(server, tmp_path, """
[[step]]
command = "Cam Get Autofocus Progress"
poll = { max_tries = 5, interval_s = 0.0, until_ge = 100.0 }
expect = { reply = "DBL", value = 100.0 }
[[step]]
command = "Cam Get Autofocus Progress"
poll = { max_tries = 3, interval_s = 0.0, until_eq = 100.0 }
[[step]]
command = "Cam Get Autofocus Progress"
poll = { max_tries = 2, interval_s = 0.0, until_gt = 50.0 }
""")
    assert [r.ok for r in report.results] == [True, True, False]
    assert report.result(3).message == "poll: until_gt 50.0 not reached after 2 tries (last value 10.0)"
    notifiers = [g.env.notifier for g in server.commands()]  # type: ignore[union-attr]
    assert notifiers == [b"replay-1", b"replay-1-2", b"replay-1-3", b"replay-2", b"replay-3", b"replay-3-2"]


def test_frames_reach_the_server_as_scripted(fake: Any, tmp_path: Path) -> None:
    server = fake()
    report = run(server, tmp_path, """
[[step]]
heartbeat_frame = true
expect = { connection = "open" }
[[step]]
raw_hex = "00 00 00 03 7f ff 42"
[[step]]
command = "Cam Set Exposure"
data_hex = "0001"
timestamps = [1000, 1001, 7]
expect = { reply = "none" }
[[step]]
command = "Cam Set Exposure"
data = { camera_type = "Low Mag.", exposure = 1500.0 }
notifier = "exp"
expect = { reply = "any" }
timeout_s = 0.05
[[step]]
code = 32767
expect = { reply = "none" }
""")
    assert report.ok, [r.line() for r in report.failures]
    heartbeat, raw, malformed, exposure, unknown = server.wait_received(5)
    assert (heartbeat.datatype, heartbeat.payload) == (DT.heartbeat, b"")
    assert (raw.datatype, raw.payload) == (0x7FFF, b"\x42")
    assert malformed.env is not None and malformed.data == "undecodable" and malformed.env.data == b"\x00\x01"
    header = synth_variant_header(PAYLOADS.layout_of(CameraSetExposureData), ENUMS, bare=False)
    assert malformed.env.variant.raw == header                            # the command's own variant header
    assert (malformed.env.request_ts, malformed.env.sending_ts, malformed.env.receiving_ts) == (1000, 1001, 7)
    assert malformed.env.notifier == b"replay-3"
    assert exposure.env is not None and exposure.env.notifier == b"exp"
    assert exposure.data == CameraSetExposureData("Low Mag.", 1500.0)
    assert unknown.env is not None and unknown.env.code == 32767 and unknown.env.data == b""
    assert unknown.env.variant.raw == EMPTY_VARIANT_HEADER
    assert report.result(2).request_hex == "000000037fff42"


def test_raw_command_frame_is_matched_by_its_notifier(fake: Any, tmp_path: Path) -> None:
    server = fake(table({"Get Heartbeat": (ReplyKind.U32, 5), "Cam Setup": (ReplyKind.ACK, None)}))
    client = AloxClient("127.0.0.1", 1, enums=ENUMS, payloads=PAYLOADS)
    frame, _ = client.build_command_frame("Get Heartbeat", notifier=b"raw-1")
    broken = bytearray(client.build_command_frame("Cam Setup", notifier=b"raw-2")[0])
    broken[-17] ^= 0xFF                                                   # attribute count != 0: envelope error
    report = run(server, tmp_path, f"""
[[step]]
raw_hex = "{frame.hex()}"
expect = {{ reply = "U32", value = 5 }}
[[step]]
raw_hex = "{bytes(broken).hex()}"
expect = {{ reply = "ACK" }}
""")
    assert report.ok, [r.line() for r in report.failures]
    assert report.result(1).notifier == b"raw-1" and report.result(2).notifier == b"raw-2"


def test_connection_expectations(fake: Any, tmp_path: Path) -> None:
    def heartbeat(got: Got, fake: FakeServer) -> Any:
        return CLOSE if got.payload else None

    server = fake(lambda got, f: heartbeat(got, f) if got.datatype == DT.heartbeat else table({"Exit": CLOSE})(got, f))
    report = run(server, tmp_path, header=WIDE_HEADER, text="""
[[step]]
heartbeat_frame = true
expect = { reply = "none", connection = "open" }
[[step]]
heartbeat_frame = true
payload_hex = "01"
expect = { connection = "open" }
""")
    assert [r.ok for r in report.results] == [True, False]
    assert report.result(2).message == "connection closed by the server"
    report = run(server, tmp_path, """
[[step]]
command = "Exit"
expect = { reply = "none", connection = "closed" }
[[step]]
command = "Get Heartbeat"
""")
    assert report.result(1).ok and report.result(1).reply_kind == "none"
    assert not report.result(2).ok and report.result(2).message.startswith("not connected")
    report = run(server, tmp_path, """
[[step]]
command = "Get Heartbeat"
expect = { reply = "none", connection = "closed" }
timeout_s = 0.1
""")
    assert report.result(1).message == "connection still open after 0.1 s"


def test_image_expectations(fake: Any, tmp_path: Path) -> None:
    pixels = np.arange(12, dtype=np.uint8).reshape(3, 4)
    image = ProvisionalCodec().flatten(ImageData(pixels, {"buffer": "0", "type": "High-Mag"}))
    server = fake(table({
        "Cam Stream Next Image": (ReplyKind.IMAGE, image),
        "Cam Get Image": (ReplyKind.ACK, None),
    }))
    camera_error = ErrorCluster(True, -1074360316, "IMAQdx Property Node: invalid camera session")
    empty = fake(table({"Cam Stream Next Image": (ReplyKind.IMAGE, b"", camera_error)}))
    report = run(server, tmp_path, """
[[step]]
command = "Cam Stream Next Image"
[step.expect.image]
width = 4
height = 3
ni_image_type = 0
custom_data_keys = ["buffer", "type"]
custom_data = { type = "High-Mag" }
[[step]]
command = "Cam Stream Next Image"
expect = { image = { width = 3, custom_data_keys = ["type"], custom_data = { buffer = "1", name = "x" } } }
""")
    assert report.result(1).ok, report.result(1).line()
    assert report.result(1).line().startswith("[  1] PASS Cam Stream Next Image  IMAGE 4x3 type 0 (2 keys)")
    message = report.result(2).message
    for part in ("image width 4, expected 3", "custom-data keys ['buffer', 'type'], expected ['type']",
                 "'buffer' = '0', expected '1'", "'name' = None, expected 'x'"):
        assert part in message
    report = run(empty, tmp_path, """
[[step]]
command = "Cam Stream Next Image"
expect = { value = "", error = { status = true, code = -1074360316 } }
[[step]]
command = "Cam Stream Next Image"
expect = { image = { width = 1 } }
[[step]]
command = "Cam Stream Next Image"
expect = { value = "00" }
""")
    assert [r.ok for r in report.results] == [True, False, False]
    assert report.result(1).line().startswith("[  1] PASS Cam Stream Next Image  IMAGE empty")
    assert "image does not decode (0 bytes)" in report.result(2).message


def test_latency_and_stop_on_fail(fake: Any, tmp_path: Path) -> None:
    def slow(got: Got, fake: FakeServer) -> Any:
        time.sleep(0.05)
        return [reply(got, ReplyKind.U32, 1)]

    server = fake(table({"Get Heartbeat": slow}))
    report = run(server, tmp_path, """
[[step]]
command = "Get Heartbeat"
expect = { max_latency_ms = 1 }
[[step]]
command = "Get Heartbeat"
expect = { max_latency_ms = 5000 }
""")
    assert [r.ok for r in report.results] == [False, True]
    assert "above 1 ms" in report.result(1).message
    script = load_script(write(tmp_path, HEADER + "stop_on_fail = true\n" + """
[[step]]
command = "Z Get Position"
timeout_s = 0.05
[[step]]
command = "Get Heartbeat"
""", "stop.toml"))
    seen: list[StepResult] = []
    report = run_script(script, host="127.0.0.1", port=server.port, on_result=seen.append)
    assert len(report.results) == 1 and seen == report.results and not report.ok


def test_decode_failure_is_reported(fake: Any, tmp_path: Path) -> None:
    server = fake(table({"Get Heartbeat": (ReplyKind.STR, "not a U32")}))
    report = run(server, tmp_path, '[[step]]\ncommand = "Get Heartbeat"\n')
    assert report.result(1).message.startswith("reply does not decode as U32")


def test_connection_refused_raises(tmp_path: Path) -> None:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    with pytest.raises(OSError):
        run_script(load_script(write(tmp_path, HEADER)), host="127.0.0.1", port=port)


# ---------------------------------------------------------------------------------------------- record / compare
def frame_hex(notifier: bytes, kind: ReplyKind, value: Any, err: ErrorCluster = NO_ERROR, **ts: int) -> str:
    return response_frame(DT, notifier, flatten_reply(kind, value, err), ResponseTimestamps(**ts)).hex()


def write_records(path: Path, records: list[dict[str, Any]]) -> Path:
    path.write_text("".join(json.dumps(r) + "\n" for r in records), encoding="utf-8")
    return path


def record(step: int, command: str, reply_hex: str | None, kind: str) -> dict[str, Any]:
    return {"step": step, "name": command, "command": command, "notifier": f"replay-{step}", "request_hex": "00",
            "reply_hex": reply_hex, "reply_kind": kind, "latency_ms": 1.0}


def test_record_file_contents(fake: Any, tmp_path: Path) -> None:
    server = fake(table({"Get Heartbeat": (ReplyKind.U32, 77)}))
    out = tmp_path / "out.jsonl"
    report = run(server, tmp_path, """
[[step]]
command = "Get Heartbeat"
[[step]]
pause_s = 0.0
[[step]]
command = "Z Get Position"
expect = { reply = "none" }
""", record=out)
    assert report.ok
    lines = [json.loads(line) for line in out.read_text(encoding="utf-8").splitlines()]
    assert [list(r) for r in lines] == [["step", "name", "command", "notifier", "request_hex", "reply_hex",
                                         "reply_kind", "latency_ms"]] * 3
    first, pause, position = lines
    assert (first["step"], first["command"], first["notifier"], first["reply_kind"]) == (1, "Get Heartbeat",
                                                                                         "replay-1", "U32")
    assert bytes.fromhex(first["request_hex"])[4:6] == DT.command.to_bytes(2, "big")
    assert first["reply_hex"] == report.result(1).reply_hex and isinstance(first["latency_ms"], float)
    assert pause["request_hex"] is None and pause["reply_hex"] is None and pause["command"] is None
    assert position["reply_hex"] is None and position["reply_kind"] == "none"


def test_compare_with_the_tick_mask(tmp_path: Path) -> None:
    a = write_records(tmp_path / "a.jsonl", [
        record(1, "Get Heartbeat", frame_hex(b"replay-1", ReplyKind.U32, 1000, tcp_sending=5), "U32"),
        record(2, "Cam Get Exposure", frame_hex(b"replay-2", ReplyKind.DBL, 1500.0, tcp_sending=6), "DBL"),
        record(3, "Z Move Absolute", None, "none"),
    ])
    b = write_records(tmp_path / "b.jsonl", [
        record(1, "Get Heartbeat", frame_hex(b"replay-1", ReplyKind.U32, 2000, tcp_sending=9), "U32"),
        record(2, "Cam Get Exposure", frame_hex(b"replay-2", ReplyKind.DBL, 1500.0, tcp_sending=10), "DBL"),
        record(3, "Z Move Absolute", None, "none"),
    ])
    assert compare_records(a, b, ["tick"]) == []
    differences = compare_records(a, b, [])
    assert len(differences) == 2 and differences[0].startswith("step 1 (Get Heartbeat): reply differs")
    c = write_records(tmp_path / "c.jsonl", [
        record(1, "Get Heartbeat", frame_hex(b"replay-1", ReplyKind.U32, 1, tcp_sending=1), "U32"),
        record(2, "Cam Get Exposure", frame_hex(b"replay-2", ReplyKind.DBL, 1501.0, tcp_sending=1), "DBL"),
        record(3, "Z Move Absolute", frame_hex(b"replay-3", ReplyKind.ACK, None, ErrorCluster(True, 9405, "x")),
               "ACK"),
        record(4, "Exit", None, "none"),
    ])
    differences = compare_records(a, c, ["tick", "image"])
    assert differences[0].startswith("step 2 (Cam Get Exposure): reply differs; first difference at byte")
    assert differences[1] == "step 3 (Z Move Absolute): no reply, reference a reply"
    assert differences[2] == "step 4: only in the reference record"
    assert len(differences) == 3


def test_compare_with_the_image_mask(tmp_path: Path) -> None:
    codec = ProvisionalCodec()
    image1 = codec.flatten(ImageData(np.zeros((2, 2), np.uint8), {"timestamp": "10:00:00:.001"}))
    image2 = codec.flatten(ImageData(np.ones((3, 2), np.uint8), {"timestamp": "10:00:01:.002"}))
    a = write_records(tmp_path / "a.jsonl", [record(1, "Cam Stream Next Image", frame_hex(
        b"replay-1", ReplyKind.IMAGE, image1, request=1, sending=2, receiving=3, pre_flatten=4, post_flatten=5,
        tcp_sending=6), "IMAGE")])
    b = write_records(tmp_path / "b.jsonl", [record(1, "Cam Stream Next Image", frame_hex(
        b"replay-1", ReplyKind.IMAGE, image2, request=11, sending=2, receiving=13, pre_flatten=14, post_flatten=15,
        tcp_sending=16), "IMAGE")])
    assert compare_records(a, b, ["tick", "image"]) == []
    assert len(compare_records(a, b, ["tick"])) == 1
    assert len(compare_records(a, b, ["image"])) == 1                    # TCP Sending still differs
    c = write_records(tmp_path / "c.jsonl", [record(1, "Cam Stream Next Image", frame_hex(
        b"replay-1", ReplyKind.IMAGE, image2, ErrorCluster(True, 9508, "codec"), request=11, sending=2, receiving=13,
        tcp_sending=16), "IMAGE")])
    assert len(compare_records(a, c, ["tick", "image"])) == 1            # the error is never masked
    d = write_records(tmp_path / "d.jsonl", [record(1, "Cam Stream Next Image", frame_hex(
        b"replay-1", ReplyKind.IMAGE, image1, request=1, sending=99, receiving=3, pre_flatten=4, post_flatten=5,
        tcp_sending=6), "IMAGE")])
    assert len(compare_records(a, d, ["tick", "image"])) == 1            # the client's sending timestamp is kept


def test_mask_reply_details() -> None:
    frame = bytes.fromhex(frame_hex(b"n", ReplyKind.U32, 123, tcp_sending=0xDEADBEEF))
    masked = mask_reply(frame, ["tick"], command="Get Heartbeat", reply_kind="U32")
    assert masked == bytes.fromhex(frame_hex(b"n", ReplyKind.U32, 0, tcp_sending=0))
    assert mask_reply(frame, ["tick"], command="Cam Get Exposure", reply_kind="U32") == bytes.fromhex(
        frame_hex(b"n", ReplyKind.U32, 123))
    assert mask_reply(frame, []) == frame
    assert mask_reply(b"\x00\x01", ["tick"]) == b"\x00\x01"                  # not a frame: unchanged
    garbage = encode_frame(DT.response, b"\x00\x00\x00\x09ab")
    assert mask_reply(garbage, ["tick"]) == garbage


def test_compare_rejects_bad_input(tmp_path: Path) -> None:
    a = write_records(tmp_path / "a.jsonl", [record(1, "Exit", None, "none")])
    with pytest.raises(ScriptError, match="unknown mask"):
        compare_records(a, a, ["pixels"])
    bad = tmp_path / "bad.jsonl"
    bad.write_text("{not json\n", encoding="utf-8")
    with pytest.raises(ScriptError, match="not a JSON record"):
        compare_records(a, bad, [])
    other = write_records(tmp_path / "o.jsonl", [record(1, "Get Heartbeat", None, "none")])
    assert compare_records(a, other, []) == ["step 1 (Exit): command 'Exit', reference 'Get Heartbeat'"]


# ---------------------------------------------------------------------------------------------- command line
def test_main_exit_codes_and_output(fake: Any, tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    server = fake(table({"Get Heartbeat": (ReplyKind.U32, 1), "Cam Get Exposure": (ReplyKind.DBL, 1.0)}))
    good = write(tmp_path, HEADER + '[[step]]\ncommand = "Get Heartbeat"\n[[step]]\ncommand = "Cam Get Exposure"\n'
                 'expect = { value = 1.0 }\n', "good.toml")
    bad = write(tmp_path, HEADER + '[[step]]\ncommand = "Cam Get Exposure"\nexpect = { value = 2.0 }\n'
                '[[step]]\ncommand = "Get Heartbeat"\n', "bad.toml")
    port = ["--port", str(server.port)]
    assert main([str(good), *port]) == 0
    out = capsys.readouterr().out
    assert "[  1] PASS Get Heartbeat  U32 1  err={F,0,\"\"}" in out and "2 step(s) run: 2 passed, 0 failed -- OK" in out
    assert main([str(bad), *port, "--quiet"]) == 1
    out = capsys.readouterr().out
    assert "[  1] FAIL Cam Get Exposure" in out and "PASS" not in out and "1 failed" in out
    assert main([str(bad), *port, "--stop-on-fail"]) == 1
    assert "1 step(s) run" in capsys.readouterr().out


def test_main_usage_errors(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    script = write(tmp_path, HEADER + '[[step]]\ncommand = "Get Heartbeat"\nnotifier = "${who}"\n')
    assert main([]) == 2
    assert main([str(tmp_path / "missing.toml")]) == 2
    assert main([str(script)]) == 2                                           # undefined variable
    assert "undefined variable ${who}" in capsys.readouterr().err
    assert main([str(script), "--var", "who"]) == 2
    assert "NAME=VALUE expected" in capsys.readouterr().err
    assert main([str(script), "--var", "who=me", "--port", "70000"]) == 2
    assert main([str(script), "--var", "who=me", "--mask", "pixels"]) == 2
    assert main([str(script), "--var", "who=me", "--settings-ini", str(tmp_path / "none.ini")]) == 2
    bad_templates = write(tmp_path, '"Cam Explode" = "00"\n', "templates.toml")
    assert main([str(script), "--var", "who=me", "--templates", str(bad_templates)]) == 2
    assert "unknown command 'Cam Explode'" in capsys.readouterr().err
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        free_port = probe.getsockname()[1]
    assert main([str(script), "--var", "who=me", "--port", str(free_port), "--timeout", "1"]) == 2
    assert "cannot connect to 127.0.0.1:" in capsys.readouterr().err


def test_main_record_and_compare(fake: Any, tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    ticks = iter(range(100, 200))
    server = fake(table({"Get Heartbeat": lambda got, f: [reply(got, ReplyKind.U32, next(ticks),
                                                                ts=ResponseTimestamps(tcp_sending=next(ticks)))]}))
    script = write(tmp_path, HEADER + '[[step]]\ncommand = "Get Heartbeat"\n')
    reference = tmp_path / "reference.jsonl"
    assert main([str(script), "--port", str(server.port), "--record", str(reference)]) == 0
    capsys.readouterr()
    assert main([str(script), "--port", str(server.port), "--compare", str(reference), "--mask", "tick"]) == 0
    assert "0 difference(s)" in capsys.readouterr().out
    assert main([str(script), "--port", str(server.port), "--compare", str(reference)]) == 1   # [session] mask: none
    out = capsys.readouterr().out
    assert "DIFF step 1 (Get Heartbeat): reply differs" in out and "1 difference(s)" in out


def test_main_wire_options(fake: Any, tmp_path: Path) -> None:
    """``--settings-ini`` [wire] and ``--enums`` change the encoding of the command code; ``--templates`` the
    variant header."""
    seen: list[Got] = []
    server = fake(lambda got, f: seen.append(got))
    script = write(tmp_path, HEADER + '[[step]]\ncommand = "Z Setup"\nexpect = { reply = "none" }\n')
    ini = tmp_path / "alox_camera_server.ini"
    ini.write_text("[wire.size]\ncamera_command = 4\n", encoding="ascii")    # the code as a U32
    assert main([str(script), "--port", str(server.port), "--settings-ini", str(ini)]) == 0
    enums_file = tmp_path / "enums.toml"
    enums_file.write_text(ENUMS.to_toml().replace('  "Get Heartbeat",\n', '  "Get Heartbeat",\n  "Spare",\n', 1),
                          encoding="utf-8")
    assert main([str(script), "--port", str(server.port), "--enums", str(enums_file)]) == 0
    template = PAYLOADS.variant_for(COMMANDS["Z Get Speed"], None)
    header = read_variant_header(Reader(template)).raw
    templates = write(tmp_path, f'"Z Setup" = "{header.hex()}"\n', "templates.toml")
    assert main([str(script), "--port", str(server.port), "--templates", str(templates)]) == 0
    assert seen[0].payload[:4] == (26).to_bytes(4, "big")                 # Z Setup = 26, 4 bytes
    assert [struct.unpack(">H", g.payload[:2])[0] for g in seen[1:]] == [27, 26]   # "Spare" shifts Z Setup
    assert seen[2].env is not None and seen[2].env.variant.raw == header


# ---------------------------------------------------------------------------------------------- AloxClient
def test_client_notifiers_and_demultiplexing(fake: Any) -> None:
    held: list[Got] = []

    def handler(got: Got, fake: FakeServer) -> Any:
        held.append(got)
        if len(held) < 3:
            return None
        return [reply(g, ReplyKind.U32, i) for i, g in reversed(list(enumerate(held)))]

    server = fake(handler)
    with AloxClient("127.0.0.1", server.port, enums=ENUMS, payloads=PAYLOADS, timeout_s=2) as client:
        assert client.connected
        names = [client.send("Get Heartbeat") for _ in range(3)]
        pid = os.getpid()
        assert names == [f"py-{pid}-{i}".encode() for i in (1, 2, 3)]
        replies = [client.wait_reply(name) for name in names]
        assert [r.value(ReplyKind.U32)[0] for r in replies if r is not None] == [0, 1, 2]
        assert client.received_notifiers() == list(reversed(names)) and client.reply_count == 3
        assert client.wait_reply(names[0], 0.05) is None                  # claimed once only
        assert client.unclaimed() == {}
        server.send(response_frame(DT, b"stray", flatten_reply(ReplyKind.ACK, None, NO_ERROR), ResponseTimestamps()))
        stray = client.wait_reply(b"stray", 2)
        assert stray is not None and stray.ts == ResponseTimestamps() and stray.frame[4:6] == b"\x00\x01"
        server.send(encode_frame(DT.heartbeat))                           # ignored by the client
        server.send(response_frame(DT, b"late", flatten_reply(ReplyKind.ACK, None, NO_ERROR), ResponseTimestamps()))
        assert client.wait_reply(b"late", 2) is not None
        server.send(response_frame(DT, b"keep", flatten_reply(ReplyKind.ACK, None, NO_ERROR), ResponseTimestamps()))
        deadline = time.monotonic() + TIMEOUT
        while client.unclaimed() != {b"keep": 1} and time.monotonic() < deadline:
            time.sleep(0.005)
        assert client.unclaimed() == {b"keep": 1}
    assert not client.connected and client.wait_closed(0)
    assert client.end_reason == "closed by the client"


def test_client_sees_the_server_close(fake: Any) -> None:
    server = fake(table({"Exit": CLOSE}))
    client = AloxClient("127.0.0.1", server.port, enums=ENUMS, payloads=PAYLOADS, timeout_s=2)
    assert not client.connected and client.wait_closed(0)                   # never connected
    client.connect()
    with pytest.raises(RuntimeError, match="already connected"):
        client.connect()
    name = client.send("Exit")
    assert client.wait_closed(TIMEOUT) and not client.connected
    assert client.end_reason == "closed by the server"
    started = time.monotonic()
    assert client.wait_reply(name, 5) is None and time.monotonic() - started < 1   # no waiting once closed
    with pytest.raises(ConnectionError, match="not connected"):
        client.send("Get Heartbeat")
    client.close()
    client.close()                                                             # idempotent


def test_client_call_and_bad_frames(fake: Any) -> None:
    server = fake(table({"Get Heartbeat": lambda got, f: [encode_frame(DT.response, b"\x00\x00"),
                                                          reply(got, ReplyKind.U32, 42)]}))
    with AloxClient("127.0.0.1", server.port, enums=ENUMS, payloads=PAYLOADS, timeout_s=2) as client:
        result = client.call("Get Heartbeat")                               # an undecodable frame is skipped
        assert result is not None and result.value(ReplyKind.U32) == (42, NO_ERROR)
        assert client.call("Z Get Position", timeout_s=0.05) is None
        server.send(b"\x00\x00\x00\x01\x00\x01")                            # N < 2: desynchronised
        assert client.wait_closed(TIMEOUT)
        assert client.end_reason is not None and client.end_reason.startswith("bad frame header")


def test_client_build_command_frame() -> None:
    client = AloxClient("127.0.0.1", 1, enums=ENUMS, payloads=PAYLOADS)
    frame, name = client.build_command_frame("Cam Setup", {"camera_type": "Low Mag.", "camera_name": "c"},
                                             notifier="n1", ts=(1, 2, 3))
    assert name == b"n1" and frame[4:6] == DT.command.to_bytes(2, "big")
    env = decode_envelope(frame[6:], ENUMS)
    assert env.code == ENUMS.code("camera_command", "Cam Setup") and (env.request_ts, env.sending_ts,
                                                                      env.receiving_ts) == (1, 2, 3)
    assert PAYLOADS.decode(COMMANDS["Cam Setup"], env.data) == CameraInitData("Low Mag.", "c")
    same, _ = client.build_command_frame("Cam Setup", CameraInitData("Low Mag.", "c"), notifier=b"n1", ts=(1, 2, 3))
    assert same == frame
    malformed, _ = client.build_command_frame("Cam Setup", notifier=b"n1", data_bytes=b"\x01")
    env = decode_envelope(malformed[6:], ENUMS)
    assert env.data == b"\x01" and env.variant.raw == decode_envelope(frame[6:], ENUMS).variant.raw
    unknown, _ = client.build_command_frame("#40000", notifier=b"u")
    env = decode_envelope(unknown[6:], ENUMS)
    assert env.code == 40000 and env.variant.raw == EMPTY_VARIANT_HEADER and env.data == b""
    heartbeat, _ = client.build_command_frame("Get Heartbeat", notifier=b"h", data_bytes=b"xyz")
    assert decode_envelope(heartbeat[6:], ENUMS).data == b"xyz"
    for command, data in (("Cam Explode", None), ("Get Heartbeat", {"x": 1}), ("#40000", {"x": 1}),
                          ("Cam Setup", {"colour": 1}), ("Cam Setup", ZLimitData(1.0)), ("#70000", None)):
        with pytest.raises(ValueError):
            client.build_command_frame(command, data)
    with pytest.raises(ConnectionError):
        client.send_heartbeat()


def test_client_heartbeat_frames(fake: Any) -> None:
    server = fake()
    with AloxClient("127.0.0.1", server.port, enums=ENUMS, payloads=PAYLOADS) as client:
        client.send_heartbeat()
        client.send_heartbeat(b"\x01\x02")
        client.send_raw(b"\x00\x00\x00\x02\x00\x07")
    first, second, third = server.wait_received(3)
    assert (first.datatype, first.payload) == (DT.heartbeat, b"")
    assert (second.datatype, second.payload) == (DT.heartbeat, b"\x01\x02")
    assert (third.datatype, third.payload) == (7, b"")
