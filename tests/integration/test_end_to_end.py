"""End-to-end tests: the replay client and the protocol client against the in-process server with simulated devices.

Spec: docs/phase1-design.md §5.2 (every command, default ``[compat]``), §5.3 (error replies switched on), §5.4
(reconnect, pipelined commands), Appendix B (record / compare); 02 §10 (notifier correlation), 02 D10 (replies of an
earlier connection dropped), 01 §7 (``Exit`` shutdown chain), 01 §9.1 (no deadlock).
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import pytest

from aloxcam.dispatch import ReplyKind
from aloxcam.errors import NO_ERROR, ErrorCluster
from aloxcam.imaging.codec import decode_provisional
from aloxcam.settings import HIGH_MAG, LOW_MAG, PLUG_USB
from aloxcam.wire.messages import COMMANDS
from integration.conftest import REAL_ALOX_INI, SCRIPTS, LiveServer, StartServer
from replay_client import RunReport, compare_records, load_script, run_script

pytestmark = pytest.mark.integration

ALL_COMMANDS = SCRIPTS / "phase1_all_commands.toml"
ERROR_REPLIES = SCRIPTS / "phase1_error_replies.toml"


def failures(report: RunReport) -> str:
    return "\n".join(r.line() for r in report.failures)


# ---------------------------------------------------------------------------------------------- §5.2
@pytest.mark.slow
def test_all_commands_default_compat(server: LiveServer) -> None:
    script = load_script(ALL_COMMANDS, extra_vars=server.install.vars)
    assert script.commands == set(COMMANDS) and len(script.commands) == 46       # coverage first

    report = server.run(script)

    assert report.ok, failures(report)
    assert len(report.results) == len(script.steps)
    runtime = server.runtime
    assert runtime.wait(10), "the server did not stop after Exit"
    z_log = server.install.event_log("z")
    assert "- ERR (Z STAGE)-(9405) : Z move out of range (Move Absolute)\r\n" in z_log       # 09 §7.4
    assert "- ERR (Z STAGE)-(-4) : RMC?: not connected (Cleanup)\r\n" in z_log                # 08 Z9
    assert "- ERR (Z STAGE)-(10) : Controller error (Get Status)\r\n" in z_log
    camera_log = server.install.event_log("camera")
    assert "ERR (HI MAG)-Cam Autofocus(9507)" in camera_log
    assert "ERR (HI MAG)-Cam Refocus(9507)" in camera_log
    backend = runtime.camera_worker.ctx.backend                                  # type: ignore[attr-defined]
    assert backend.closed and set(backend.devices) == {"cam0", "cam1"}
    assert all(device.closed for device in backend.devices.values())
    (stage,) = server.stages
    assert not stage.is_connected
    history = list(runtime.state.commands_history)
    assert history[0] == "Exit"
    assert "#32767" in history and "Cam Autofocus Run" in history and "Cam Refocus Run" in history


# ---------------------------------------------------------------------------------------------- §5.3
def test_error_replies(start_server: StartServer) -> None:
    server = start_server(compat={"send_error_replies": "TRUE", "reply_unknown_commands": "TRUE"})
    report = server.run(ERROR_REPLIES)
    assert report.ok, failures(report)
    assert server.runtime.wait(10)
    stream = next(r for r in report.results if r.step.command == "Cam Stream Next Image")
    assert stream.reply_kind == "IMAGE" and stream.value == b""


def test_unknown_commands_only(start_server: StartServer, tmp_path: Path) -> None:
    server = start_server(compat={"reply_unknown_commands": "TRUE"})
    script = tmp_path / "unknown_only.toml"
    script.write_text("""
[session]
timeout_s = 10.0
no_reply_wait_s = 0.25

[[step]]
code = 32767
expect = { reply = "ACK", error = { status = true, code = 9506, source = "Unknown command #32767" } }

[[step]]
name = "9405 still suppressed"
command = "Z Move Absolute"
data = { pos_mm = 30.0 }
expect = { reply = "none" }

[[step]]
name = "91 ACK suppressed"
command = "Cam Set Exposure"
data_hex = "0001"
expect = { reply = "none" }

[[step]]
name = "internal command, not routed: 9506 ACK"
command = "Cam Refocus Run"
expect = { reply = "ACK", error = { status = true, code = 9506, source = "Unknown command Cam Refocus Run" } }

[[step]]
command = "Get Heartbeat"
expect = { reply = "U32", error = { status = false, code = 0 } }
""", encoding="utf-8")
    report = server.run(script)
    assert report.ok, failures(report)


# ---------------------------------------------------------------------------------------------- Appendix B
def test_record_and_compare_two_servers(start_server: StartServer, tmp_path: Path) -> None:
    """Two runs of the same script on two servers give byte-identical replies once the tick fields (and the image
    fields) are masked: the comparison used against the LabVIEW server (10 §7)."""
    compat = {"send_error_replies": "TRUE", "reply_unknown_commands": "TRUE"}
    records = []
    for name in ("first", "second"):
        server = start_server(compat=compat, name=name)
        record = tmp_path / f"{name}.jsonl"
        report = run_script(load_script(ERROR_REPLIES, extra_vars=server.install.vars), host=server.host,
                            port=server.port, enums=server.loaded.enums, record=record)
        assert report.ok, failures(report)
        records.append(record)
    assert compare_records(records[0], records[1], ["tick", "image"]) == []
    unmasked = compare_records(records[0], records[1], [])
    assert any("(Get Heartbeat)" in d for d in unmasked)                     # the tick value and TCP Sending differ


# ---------------------------------------------------------------------------------------------- §5.4
def test_reconnect(server: LiveServer) -> None:
    """02 D10: a reply produced for the first connection is never delivered to the second one."""
    with server.client() as first:
        reply = first.call("Z Setup")
        assert reply is not None and reply.value(ReplyKind.ACK)[1] == NO_ERROR
        started = time.monotonic()
        first.send("Z Wait End Move", {"add_wait_ms": 1500, "polling_cycle_ms": 10, "add_wait_only": True},
                   notifier=b"slow-wait")
    assert not first.connected

    with server.client() as second:
        heartbeat = second.call("Get Heartbeat")
        assert heartbeat is not None and heartbeat.value(ReplyKind.U32)[1] == NO_ERROR
        assert time.monotonic() - started < 1.5, "reconnected after the slow wait ended: the test proves nothing"
        position = second.call("Z Get Position", timeout_s=10)         # queued behind the slow wait (Z worker FIFO)
        assert position is not None and position.value(ReplyKind.DBL)[1] == NO_ERROR
        assert time.monotonic() - started >= 1.5
        assert b"slow-wait" not in second.received_notifiers()
        assert second.wait_reply(b"slow-wait", 0.2) is None
        assert second.reply_count == 2


def test_pipelined(start_server: StartServer) -> None:
    """20 commands sent without waiting, camera and Z interleaved, with a stream request: every reply arrives
    exactly once and is matched by its notifier (02 §10 rule 4); no deadlock (01 §9.1). The server runs with the
    owner's production alox.ini (port 503 overridden by --port 0)."""
    server = start_server(alox_ini=REAL_ALOX_INI.read_bytes())
    assert server.port != 503 and server.loaded.alox.port == 503
    hm = {"camera_type": HIGH_MAG}
    commands: list[tuple[str, dict[str, object] | None]] = [
        ("Cam Setup", {**hm, "camera_name": "cam0", "number_of_buffers": 5, "continuous_mode": True}),
        ("Z Setup", None),
        ("Cam Start", {**hm, "triggered": False}),
        ("Z Get Serial", {"type": True}),
        ("Cam Stream Next Image", {**hm, "simulation": False}),
        ("Z Get Status", None),
        ("Get Heartbeat", None),
        ("Z Set Speed", {"speed_mm_s": 50.0}),
        ("Cam Set Exposure", {**hm, "exposure": 2000.0}),
        ("Z Move Absolute", {"pos_mm": 5.0}),
        ("Cam Get Exposure", hm),
        ("Z Wait End Move", {"add_wait_ms": 0, "polling_cycle_ms": 10, "add_wait_only": False}),
        ("Cam Get Acquisition Status", hm),
        ("Z Get Position", None),
        ("Cam Set Gain", {**hm, "gain": 1.0}),
        ("Z Get Reference", None),
        ("Cam Get Gain", hm),
        ("Z Get Speed", None),
        ("Get Heartbeat", None),
        ("Z Get Error", None),
    ]
    with server.client(timeout_s=10) as client:
        notifiers = [client.send(name, data, notifier=f"pipe-{i}".encode()) for i, (name, data) in
                     enumerate(commands)]
        replies = {}
        for (name, _), notifier in zip(commands, notifiers):
            reply = client.wait_reply(notifier, 10)
            assert reply is not None, f"no reply to {name} ({notifier!r})"
            value, err = reply.value(COMMANDS[name].reply)
            assert err == NO_ERROR, f"{name}: {err}"
            replies[name] = value
        assert replies["Cam Get Exposure"] == 2000.0 and replies["Cam Get Gain"] == 1.0
        assert replies["Cam Get Acquisition Status"] is True and replies["Z Get Reference"] is True
        assert replies["Z Get Position"] == pytest.approx(5.0, abs=1e-6) and replies["Z Get Speed"] == 50.0
        image = decode_provisional(replies["Cam Stream Next Image"])
        assert (image.width, image.height) == (640, 480)
        assert image.custom_data["name"] == "LowMagg"            # HM name_STR of the owner's file (sic)
        for name in ("Get Heartbeat", "Z Get Error"):            # sentinels: both workers have nothing left
            assert client.call(name) is not None
        assert client.reply_count == len(commands) + 2           # every reply exactly once
        assert client.unclaimed() == {}
        assert sorted(client.received_notifiers()[:len(commands)]) == sorted(notifiers)


def test_production_alox_ini(start_server: StartServer) -> None:
    """The owner's alox.ini drives the server: USB cameras, swapped camera names, 176 µs minimum exposure, Z speed
    5 mm/s and Z range 0..90 mm."""
    server = start_server(alox_ini=REAL_ALOX_INI.read_bytes())
    state = server.runtime.state
    hm, lm = state.camera_settings(HIGH_MAG), state.camera_settings(LOW_MAG)
    assert (hm.plug_type, lm.plug_type) == (PLUG_USB, PLUG_USB)
    assert (hm.camera_name, lm.camera_name) == ("LowMagg", "HighMag")
    assert (state.z.speed, state.z.min_pos, state.z.max_pos) == (5.0, 0.0, 90.0)
    with server.client(timeout_s=10) as client:
        def call(name: str, data: dict[str, object] | None = None) -> tuple[object, ErrorCluster]:
            reply = client.call(name, data)
            assert reply is not None, f"no reply to {name}"
            return reply.value(COMMANDS[name].reply)

        for camera_type, camera_name in ((HIGH_MAG, "cam0"), (LOW_MAG, "cam1")):
            assert call("Cam Setup", {"camera_type": camera_type, "camera_name": camera_name,
                                      "continuous_mode": True})[1] == NO_ERROR
        assert call("Cam Set Exposure", {"camera_type": LOW_MAG, "exposure": 100.0})[1] == NO_ERROR
        assert call("Cam Get Exposure", {"camera_type": LOW_MAG}) == (176.0, NO_ERROR)
        assert call("Cam Start", {"camera_type": LOW_MAG})[1] == NO_ERROR
        value, err = call("Cam Stream Next Image", {"camera_type": LOW_MAG})
        assert err == NO_ERROR
        image = decode_provisional(value)  # type: ignore[arg-type]
        assert (image.custom_data["name"], image.custom_data["type"]) == ("HighMag", "Low-Mag")
        assert np.asarray(image.pixels).shape == (480, 640)
        assert call("Cam Set Led", {"camera_type": HIGH_MAG, "led": True})[1] == NO_ERROR    # USB line path
        # lightStatusInverter_BOOL = FALSE: the read-back is the inverse of the command (05 §9.3, LabVIEW behaviour)
        assert call("Cam Get Led", {"camera_type": HIGH_MAG}) == (False, NO_ERROR)
        assert call("Z Setup")[1] == NO_ERROR
        assert call("Z Get Speed") == (5.0, NO_ERROR)                     # [hardware.z] speed_DBL = 5,000000
        assert call("Z Set Max Limit", {"value_mm": 90.0})[1] == NO_ERROR
        client.send("Exit")
        assert client.wait_closed(10)
    assert server.runtime.wait(10)
