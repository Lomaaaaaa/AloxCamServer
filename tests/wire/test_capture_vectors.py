"""Real LabVIEW frames from the network capture (docs/answers/capture-analysis.md).

``tests/golden/capture_vectors.json`` holds, for each of the 21 command types seen on the wire, the first request sent by
the real ALOX Vision Core client and the LabVIEW server's reply. The codec must decode every request and re-encode its
payload and every reply byte for byte.
"""

import json
from pathlib import Path

import pytest

from aloxcam.dispatch import ReplyKind
from aloxcam.wire.enums import WireEnums
from aloxcam.wire.messages import COMMANDS, PayloadCodec
from aloxcam.wire.protocol import decode_envelope, decode_response_data, flatten_reply, parse_reply_value

DATA = json.loads((Path(__file__).parent.parent / "golden" / "capture_vectors.json").read_text(encoding="utf-8"))
VECTORS = DATA["vectors"]
ENUMS = WireEnums.load()
CODEC = PayloadCodec(ENUMS)


def _id(vector: dict) -> str:
    return str(vector["command"])


def test_the_capture_covers_21_command_types() -> None:
    assert len(VECTORS) == 21
    assert {v["command"] for v in VECTORS} <= set(COMMANDS)


@pytest.mark.parametrize("vector", VECTORS, ids=_id)
def test_request_decodes_and_its_payload_re_encodes_byte_exactly(vector: dict) -> None:
    payload = bytes.fromhex(vector["request"])
    env = decode_envelope(payload, ENUMS)
    assert ENUMS.label("camera_command", env.code) == vector["command"]
    spec = COMMANDS[vector["command"]]
    obj = CODEC.decode(spec, env.data)
    if obj is not None and spec.payload is not None:
        assert CODEC.encode(obj) == env.data
    assert env.receiving_ts == 0 and env.sending_ts > 0                          # set by the client: "sending" …
    assert (env.request_ts > 0) == (vector["command"] == "Cam Stream Next Image")  # … and "request" for images


@pytest.mark.parametrize("vector", [v for v in VECTORS if "reply" in v], ids=_id)
def test_reply_re_encodes_byte_exactly(vector: dict) -> None:
    rd = decode_response_data(bytes.fromhex(vector["reply"]))
    assert rd.notifier == decode_envelope(bytes.fromhex(vector["request"]), ENUMS).notifier
    kind = COMMANDS[vector["command"]].reply
    value, err = parse_reply_value(kind, rd.flattened)
    assert flatten_reply(kind, value, err) == rd.flattened
    assert rd.ts.as_tuple()[:5] == (0, 0, 0, 0, 0) and rd.ts.tcp_sending > 0     # non-image replies: TCP sending only


def test_camera_type_is_a_u32_enum_on_the_wire() -> None:
    (vector,) = [v for v in VECTORS if v["command"] == "Cam Setup"]
    env = decode_envelope(bytes.fromhex(vector["request"]), ENUMS)
    assert env.data[:4] == b"\x00\x00\x00\x00"                                     # High Mag. as U32
    obj = CODEC.decode(COMMANDS["Cam Setup"], env.data)
    assert (obj.camera_type, obj.camera_name, obj.number_of_buffers, obj.continuous_mode) == \
        ("High Mag.", "HighMag", 5, True)


def test_labview_sends_error_acks() -> None:
    case = DATA["error_ack"]
    assert case["command"] == "Cam Set Led"
    rd = decode_response_data(bytes.fromhex(case["reply"]))
    _, err = parse_reply_value(ReplyKind.ACK, rd.flattened)
    assert err.status and err.code == -1074360304
    assert err.source == "Property Node (arg 2) in alox_camera_server.vi"
