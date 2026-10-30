"""aloxcam.server.router: envelope decoding, history, routing, receiving timestamp (02 §7, 03 §3, design §3.4.3)."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field

import pytest

from aloxcam.dispatch import CAMERA_WORKER, Z_WORKER, CommandSpec, ReplyKind, Request
from aloxcam.errors import ErrorCluster
from aloxcam.server.router import ENVELOPE_ERROR, PAYLOAD_ERROR, Router
from aloxcam.settings import CompatSwitches
from aloxcam.state import ServerState
from aloxcam.testing import FixedClock, RecordingReplies, make_state
from aloxcam.wire.enums import build_routing
from aloxcam.wire.messages import (
    COMMANDS,
    CameraSetExposureData,
    CameraStreamData,
    ZLimitData,
    ZMoveAbsData,
)
from aloxcam.wire.protocol import encode_envelope
from golden import spec_vectors as sv
from server.wp4_helpers import ENUMS, PAYLOADS, code_of, envelope

ROUTING = build_routing(ENUMS)


@dataclass
class Rig:
    router: Router
    state: ServerState
    replies: RecordingReplies
    clock: FixedClock
    camera: list[Request] = field(default_factory=list)
    z: list[Request] = field(default_factory=list)


def make_rig(*, compat: CompatSwitches | None = None, commands: Mapping[str, CommandSpec] = COMMANDS) -> Rig:
    compat = compat or CompatSwitches()
    state = make_state()
    replies = RecordingReplies(send_error_replies=compat.send_error_replies)
    clock = FixedClock(5000)
    camera: list[Request] = []
    z: list[Request] = []
    router = Router(enums=ENUMS, routing=ROUTING, commands=commands, payloads=PAYLOADS, state=state, replies=replies,
                    compat=compat, tick=clock, camera_put=camera.append, z_put=z.append)
    return Rig(router, state, replies, clock, camera, z)


@pytest.fixture
def rig() -> Rig:
    return make_rig()


def test_camera_route_sets_receiving_timestamp(rig: Rig) -> None:
    data = CameraSetExposureData(camera_type="Low Mag.", exposure=1500.0)
    payload = envelope("Cam Set Exposure", data, notifier=b"N\x81", ts=(1000, 1001, 7))
    req = rig.router.route_frame(payload, conn_id=3)
    assert req is not None and rig.camera == [req] and rig.z == []
    assert (req.command, req.code, req.spec) == ("Cam Set Exposure", 7, COMMANDS["Cam Set Exposure"])
    assert req.notifier == b"N\x81" and req.conn_id == 3 and req.data == data and req.decode_error is None
    assert (req.request_ts, req.sending_ts, req.receiving_ts) == (1000, 1001, 5000)
    assert req.raw_data == PAYLOADS.encode(data)
    assert rig.state.commands_history[0] == "Cam Set Exposure" and rig.state.last_command == "Cam Set Exposure"
    assert rig.replies.replies == []


def test_z_route_keeps_the_client_receiving_timestamp(rig: Rig) -> None:
    req = rig.router.route_frame(envelope("Z Move Absolute", ZMoveAbsData(12.5), ts=(1, 2, 3)), conn_id=1)
    assert req is not None and rig.z == [req] and rig.camera == []
    assert (req.request_ts, req.sending_ts, req.receiving_ts) == (1, 2, 3)
    assert req.data == ZMoveAbsData(12.5)


def test_the_02_7_5_b_frame_is_routed_to_z(rig: Rig) -> None:
    v = sv.Z_SET_MIN_LIMIT_FRAME
    frame = sv.fill(v.template, tt=ENUMS.code("tcp_datatype", v.datatype), cc=ENUMS.code("camera_command", v.command))
    req = rig.router.route_frame(frame[6:], conn_id=1)
    assert req is not None and rig.z == [req]
    assert (req.command, req.notifier, req.data) == ("Z Set Min Limit", v.notifier, ZLimitData(0.5))
    assert (req.request_ts, req.sending_ts, req.receiving_ts) == v.ts


def test_log_goes_to_the_camera_worker(rig: Rig) -> None:
    req = rig.router.route_frame(envelope("Log"), conn_id=1)
    assert req is not None and rig.camera == [req] and req.code == 45


def test_unknown_code_dropped_without_reply_but_recorded(rig: Rig) -> None:
    assert rig.router.route_frame(envelope(32767), conn_id=1) is None
    assert rig.camera == rig.z == [] and rig.replies.replies == []
    assert rig.state.commands_history[0] == "#32767"


def test_reply_unknown_commands_sends_a_forced_9506_ack() -> None:
    rig = make_rig(compat=CompatSwitches(reply_unknown_commands=True))
    assert rig.router.route_frame(envelope(32767, notifier=b"U1"), conn_id=4) is None
    (reply,) = rig.replies.replies
    assert reply.kind is ReplyKind.ACK and reply.sent          # sent although send_error_replies is FALSE
    assert reply.err == ErrorCluster(True, 9506, "Unknown command #32767")
    assert reply.req.notifier == b"U1" and reply.req.conn_id == 4


@pytest.mark.parametrize("command", ["Cam Autofocus Run", "Cam Refocus Run"])
def test_run_commands_are_never_routed(command: str) -> None:
    rig = make_rig(compat=CompatSwitches(reply_unknown_commands=True))
    assert rig.router.route_frame(envelope(command), conn_id=1) is None
    assert rig.camera == rig.z == []
    assert rig.state.commands_history[0] == command
    (reply,) = rig.replies.replies                              # reply kind NONE → ACK 9506
    assert reply.kind is ReplyKind.ACK and reply.err.code == 9506


def test_unknown_command_with_a_typed_reply_kind_uses_that_kind() -> None:
    commands = dict(COMMANDS)
    rig = make_rig(compat=CompatSwitches(reply_unknown_commands=True), commands=commands)
    # a known name that the routing table does not route: the stub routing of a custom enum order
    unrouted = Router(enums=ENUMS, routing=build_routing(ENUMS, non_routable=("Z Get Position",)), commands=commands,
                      payloads=PAYLOADS, state=rig.state, replies=rig.replies, compat=rig.router.compat,
                      tick=rig.clock, camera_put=rig.camera.append, z_put=rig.z.append)
    assert unrouted.route_frame(envelope("Z Get Position"), conn_id=1) is None
    (reply,) = rig.replies.replies
    assert reply.kind is ReplyKind.DBL and reply.value == 0.0 and reply.err.code == 9506


def test_payload_decode_error_gives_91(rig: Rig) -> None:
    req = rig.router.route_frame(envelope("Cam Set Exposure", data_bytes=b"\x00\x01"), conn_id=1)
    assert req is not None and rig.camera == [req]
    assert req.decode_error == PAYLOAD_ERROR == ErrorCluster(True, 91, "Variant To Data in alox_camera_server.vi")
    assert req.data is None and req.raw_data == b"\x00\x01"
    assert rig.replies.replies == []                            # the worker rejects it


def test_not_read_and_unused_payloads_accept_any_bytes(rig: Rig) -> None:
    hb = rig.router.route_frame(envelope("Get Heartbeat", data_bytes=b"junk"), conn_id=1)
    pos = rig.router.route_frame(envelope("Z Get Position", data_bytes=b"\xff"), conn_id=1)
    assert hb is not None and hb.decode_error is None and hb.data is None
    assert pos is not None and pos.decode_error is None and pos.data is None


def test_routed_code_without_command_spec_has_no_spec_and_no_data() -> None:
    commands = {name: spec for name, spec in COMMANDS.items() if name != "Cam Stop"}
    rig = make_rig(commands=commands)
    req = rig.router.route_frame(envelope(code_of("Cam Stop"), data_bytes=b"\x00\x00"), conn_id=1)
    assert req is not None and rig.camera == [req]
    assert (req.command, req.spec, req.data, req.decode_error) == ("Cam Stop", None, None, None)


def _broken_variant_envelope(command: str | int, notifier: bytes = b"BAD") -> bytes:
    """Code and notifier decode; the variant header does not (TD size < 4)."""
    code = command if isinstance(command, int) else code_of(command)
    variant = bytes.fromhex("18008000 00000001 0002")
    return encode_envelope(code, notifier, variant, ENUMS, (9, 9, 9))


def test_envelope_with_code_and_notifier_is_rejected_with_116(rig: Rig) -> None:
    req = rig.router.route_frame(_broken_variant_envelope("Z Move Absolute"), conn_id=2)
    assert req is not None and rig.z == [req]
    assert req.decode_error == ENVELOPE_ERROR
    assert ENVELOPE_ERROR == ErrorCluster(True, 116, "Unflatten From String in alox_camera_server.vi")
    assert (req.command, req.notifier, req.data, req.conn_id) == ("Z Move Absolute", b"BAD", None, 2)
    assert (req.request_ts, req.sending_ts, req.receiving_ts) == (0, 0, 0)
    assert rig.state.commands_history[0] == "Z Move Absolute"


def test_envelope_error_on_the_camera_route(rig: Rig) -> None:
    req = rig.router.route_frame(_broken_variant_envelope("Cam Stream Next Image"), conn_id=2)
    assert req is not None and rig.camera == [req] and req.decode_error == ENVELOPE_ERROR
    assert (req.request_ts, req.sending_ts) == (0, 0) and req.receiving_ts == 5000


@pytest.mark.parametrize("payload", [
    b"",                                                    # nothing at all
    b"\x00",                                                # truncated command code
    b"\x00\x07\x00\x00\x00\x09ab",                          # truncated notifier
])
def test_envelopes_without_code_or_notifier_are_dropped(rig: Rig, payload: bytes) -> None:
    assert rig.router.route_frame(payload, conn_id=1) is None
    assert rig.camera == rig.z == [] and list(rig.state.commands_history) == []


def test_envelope_error_of_an_unrouted_or_unknown_code_is_dropped(rig: Rig) -> None:
    assert rig.router.route_frame(_broken_variant_envelope(32767), conn_id=1) is None
    assert rig.router.route_frame(_broken_variant_envelope("Cam Autofocus Run"), conn_id=1) is None
    assert rig.camera == rig.z == [] and list(rig.state.commands_history) == []


def test_routes_match_the_routing_table(rig: Rig) -> None:
    for name, spec in COMMANDS.items():
        data = CameraStreamData() if name == "Cam Stream Next Image" else None
        req = rig.router.route_frame(envelope(name, data), conn_id=1)
        if not spec.client_routable:
            assert req is None
            continue
        assert req is not None and req.decode_error is None, name
        assert req in (rig.camera if spec.worker == CAMERA_WORKER else rig.z), name
        assert spec.worker in (CAMERA_WORKER, Z_WORKER)
    assert len(rig.camera) + len(rig.z) == 44 and len(rig.state.commands_history) == 46
