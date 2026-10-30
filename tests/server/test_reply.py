"""aloxcam.server.reply: reply frames per kind, suppression, timestamps, image path (02 §8, design §3.4.2)."""

from __future__ import annotations

import socket
from collections.abc import Iterator
from dataclasses import dataclass

import numpy as np
import pytest

from aloxcam.dispatch import ReplyKind, Request
from aloxcam.errors import NO_ERROR, ErrorCluster
from aloxcam.imagedata import ImageData
from aloxcam.server.link import ClientLink
from aloxcam.server.reply import Replier
from aloxcam.settings import CompatSwitches
from aloxcam.testing import FixedClock, make_request
from golden import spec_vectors as sv
from server.wp4_helpers import CODEC_ERROR, DT, ENUMS, FakeCodec, read_frame, read_reply

ERR = ErrorCluster(True, 9405, "Z move out of range")
WARN = ErrorCluster(False, 5, "a warning")


class CountingClock:
    """A tick that increases by 10 at every call (to tell the timestamps apart)."""

    def __init__(self, start: int = 1000) -> None:
        self.value = start

    def __call__(self) -> int:
        self.value += 10
        return self.value


@dataclass
class Rig:
    replier: Replier
    link: ClientLink
    client: socket.socket
    conn_id: int
    codec: FakeCodec

    def req(self, command: str = "Cam Stop", **kw: object) -> Request:
        kw.setdefault("conn_id", self.conn_id)
        return make_request(command, **kw)  # type: ignore[arg-type]

    def sentinel(self) -> None:
        """Send a marker reply: the next frame read must be this one when nothing was sent before it."""
        self.replier.string(self.req("Z Get Error", notifier=b"SENTINEL"), "end", NO_ERROR)

    def expect_sentinel(self) -> None:
        notifier, value, _, _ = read_reply(self.client, ReplyKind.STR)
        assert (notifier, value) == (b"SENTINEL", "end")


def _rig(compat: CompatSwitches, tick: object, codec: FakeCodec | None = None) -> Iterator[Rig]:
    server_side, client = socket.socketpair()
    server_side.settimeout(5.0)
    link = ClientLink()
    conn_id = link.attach(server_side)
    codec = codec or FakeCodec()
    replier = Replier(link, DT, codec, compat, tick)  # type: ignore[arg-type]
    try:
        yield Rig(replier, link, client, conn_id, codec)
    finally:
        server_side.close()
        client.close()


@pytest.fixture
def rig() -> Iterator[Rig]:
    yield from _rig(CompatSwitches(), CountingClock())


@pytest.fixture
def rig_send_errors() -> Iterator[Rig]:
    yield from _rig(CompatSwitches(send_error_replies=True), CountingClock())


@pytest.fixture
def rig_suppress_errors() -> Iterator[Rig]:
    yield from _rig(CompatSwitches(send_error_replies=False), CountingClock())


def test_get_heartbeat_reply_equals_the_57_byte_golden_vector() -> None:
    v = sv.GET_HEARTBEAT_REPLY
    for rig in _rig(CompatSwitches(), FixedClock(12345678)):
        out = rig.replier.u32(rig.req("Get Heartbeat", notifier=v.notifier), v.value, v.err)
        assert out == NO_ERROR
        frame = read_frame(rig.client)
        assert frame is not None
        expected = sv.fill(v.template, tt=ENUMS.code("tcp_datatype", v.datatype))
        assert len(expected) == v.size == 57
        assert (6 + len(frame[1])) == 57 and frame[0] == DT.response
        raw = (len(frame[1]) + 2).to_bytes(4, "big") + frame[0].to_bytes(2, "big") + frame[1]
        assert raw == expected


def test_error_ack_sent_by_default(rig: Rig) -> None:
    rig.replier.ack(rig.req(notifier=b"E"), ERR)        # LabVIEW sends error ACKs (network capture)
    notifier, _, err, _ = read_reply(rig.client, ReplyKind.ACK)
    assert (notifier, err) == (b"E", ERR)


def test_error_ack_suppressed_when_switched_off(rig_suppress_errors: Rig) -> None:
    rig = rig_suppress_errors
    req = rig.req()
    assert rig.replier.ack(req, ERR) is ERR           # the ACK VI's error out
    assert req.replied
    rig.sentinel()
    rig.expect_sentinel()                             # nothing was sent before the sentinel


def test_warning_and_success_acks_are_sent(rig: Rig) -> None:
    for err in (WARN, NO_ERROR):
        assert rig.replier.ack(rig.req(notifier=b"A"), err) == err
        notifier, value, got, ts = read_reply(rig.client, ReplyKind.ACK)
        assert (notifier, value, got) == (b"A", None, err)
        assert ts.as_tuple()[:5] == (0, 0, 0, 0, 0) and ts.tcp_sending > 0


def test_error_ack_sent_with_send_error_replies(rig_send_errors: Rig) -> None:
    rig = rig_send_errors
    assert rig.replier.ack(rig.req(), ERR) == ERR
    assert read_reply(rig.client, ReplyKind.ACK)[2] == ERR


def test_forced_error_ack_sent_even_without_send_error_replies(rig: Rig) -> None:
    unknown = ErrorCluster(True, 9506, "Unknown command #99")
    assert rig.replier.ack(rig.req(), unknown, force=True) == unknown
    assert read_reply(rig.client, ReplyKind.ACK)[2] == unknown


@pytest.mark.parametrize(("method", "kind", "value"), [
    ("string", ReplyKind.STR, "hello"),
    ("boolean", ReplyKind.BOOL, True),
    ("dbl", ReplyKind.DBL, 1500.0),
    ("u32", ReplyKind.U32, 123456),
])
def test_typed_replies_always_sent_with_the_error(rig: Rig, method: str, kind: ReplyKind, value: object) -> None:
    req = rig.req(notifier=b"T")
    assert getattr(rig.replier, method)(req, value, ERR) == NO_ERROR
    assert req.replied
    notifier, got, err, ts = read_reply(rig.client, kind)
    assert (notifier, got, err) == (b"T", value, ERR)
    assert ts.as_tuple()[:5] == (0, 0, 0, 0, 0)


def test_dbl_2d_reply(rig: Rig) -> None:
    table = np.array([[1.0, 0.5], [2.0, 0.75]])
    assert rig.replier.dbl_2d(rig.req(), table, NO_ERROR) == NO_ERROR
    _, got, err, _ = read_reply(rig.client, ReplyKind.DBL2D)
    assert np.array_equal(got, table) and err == NO_ERROR


def test_image_reply_copies_request_timestamps_and_flatten_ticks(rig: Rig) -> None:
    image = ImageData(np.arange(6, dtype=np.uint8).reshape(2, 3), {"name": "HighMag"})
    req = rig.req("Cam Stream Next Image", notifier=b"I", ts=(11, 22, 33))
    assert rig.replier.image(req, image, NO_ERROR) == NO_ERROR
    assert req.replied and rig.codec.calls == [image]
    _, value, err, ts = read_reply(rig.client, ReplyKind.IMAGE)
    assert value == b"IMG" + image.pixels.tobytes() and err == NO_ERROR
    assert (ts.request, ts.sending, ts.receiving) == (11, 22, 33)
    assert 0 < ts.pre_flatten < ts.post_flatten < ts.tcp_sending


def test_image_reply_on_error_skips_the_codec_but_keeps_the_ticks(rig: Rig) -> None:
    image = ImageData(np.zeros((2, 2), dtype=np.uint8))
    err = ErrorCluster(True, -1074360316, "Cam Get Image")
    req = rig.req("Cam Stream Next Image", ts=(1, 2, 3))
    assert rig.replier.image(req, image, err) == NO_ERROR
    assert rig.codec.calls == []
    _, value, got, ts = read_reply(rig.client, ReplyKind.IMAGE)
    assert (value, got) == (b"", err)
    assert (ts.request, ts.sending, ts.receiving) == (1, 2, 3) and 0 < ts.pre_flatten < ts.post_flatten


def test_codec_error_becomes_the_image_reply_error() -> None:
    for rig in _rig(CompatSwitches(), CountingClock(), FakeCodec(CODEC_ERROR)):
        image = ImageData(np.zeros((2, 2), dtype=np.uint8))
        assert rig.replier.image(rig.req("Cam Stream Next Image"), image, NO_ERROR) == NO_ERROR
        _, value, err, ts = read_reply(rig.client, ReplyKind.IMAGE)
        assert (value, err) == (b"", CODEC_ERROR.cluster()) and ts.pre_flatten < ts.post_flatten


def test_unexpected_codec_exception_becomes_9500() -> None:
    for rig in _rig(CompatSwitches(), CountingClock(), FakeCodec(ValueError("boom"))):
        rig.replier.image(rig.req("Cam Stream Next Image"), ImageData(np.zeros((1, 1), np.uint8)), NO_ERROR)
        _, value, err, _ = read_reply(rig.client, ReplyKind.IMAGE)
        assert value == b"" and (err.status, err.code, err.source) == (True, 9500, "ValueError: boom")


@pytest.mark.parametrize(("kind", "default"), [
    (ReplyKind.STR, ""), (ReplyKind.BOOL, False), (ReplyKind.DBL, 0.0), (ReplyKind.U32, 0), (ReplyKind.IMAGE, b""),
])
def test_error_reply_sends_the_default_value(rig: Rig, kind: ReplyKind, default: object) -> None:
    req = rig.req()
    assert rig.replier.error_reply(req, kind, ERR) == NO_ERROR
    assert req.replied
    _, value, err, _ = read_reply(rig.client, kind)
    assert (value, err) == (default, ERR)


def test_error_reply_dbl2d_and_ack_and_none(rig_suppress_errors: Rig) -> None:
    rig = rig_suppress_errors
    rig.replier.error_reply(rig.req(), ReplyKind.DBL2D, ERR)
    _, value, err, _ = read_reply(rig.client, ReplyKind.DBL2D)
    assert value.shape == (0, 0) and err == ERR
    none_req, ack_req = rig.req(), rig.req()
    assert rig.replier.error_reply(none_req, ReplyKind.NONE, ERR) == ERR and none_req.replied
    assert rig.replier.error_reply(ack_req, ReplyKind.ACK, ERR) == ERR and ack_req.replied   # suppressed
    forced = rig.req(notifier=b"F")
    rig.replier.error_reply(forced, ReplyKind.ACK, ERR, force=True)
    notifier, _, err, _ = read_reply(rig.client, ReplyKind.ACK)
    assert (notifier, err) == (b"F", ERR)


def test_every_method_sets_replied_and_internal_requests_are_never_sent(rig: Rig) -> None:
    image = ImageData(np.zeros((1, 1), np.uint8))
    calls = [
        lambda r: rig.replier.ack(r, NO_ERROR),
        lambda r: rig.replier.ack(r, ERR),
        lambda r: rig.replier.string(r, "s", NO_ERROR),
        lambda r: rig.replier.boolean(r, True, NO_ERROR),
        lambda r: rig.replier.dbl(r, 1.0, NO_ERROR),
        lambda r: rig.replier.u32(r, 1, NO_ERROR),
        lambda r: rig.replier.dbl_2d(r, np.empty((0, 2)), NO_ERROR),
        lambda r: rig.replier.image(r, image, NO_ERROR),
        lambda r: rig.replier.error_reply(r, ReplyKind.NONE, ERR),
        lambda r: rig.replier.error_reply(r, ReplyKind.BOOL, ERR),
    ]
    for call in calls:
        req = rig.req(conn_id=0)
        call(req)
        assert req.replied
    assert rig.codec.calls == [image]                 # the image path still runs; only the write is skipped
    rig.sentinel()
    rig.expect_sentinel()


def test_replies_for_an_old_connection_are_dropped(rig: Rig) -> None:
    rig.replier.boolean(rig.req(conn_id=rig.conn_id + 5), True, NO_ERROR)
    rig.sentinel()
    rig.expect_sentinel()


def test_unencodable_value_sends_the_default_with_9500(rig: Rig) -> None:
    rig.replier.u32(rig.req(notifier=b"U"), -1, NO_ERROR)         # out of U32 range: handler bug
    notifier, value, err, _ = read_reply(rig.client, ReplyKind.U32)
    assert (notifier, value, err.status, err.code) == (b"U", 0, True, 9500)


def test_unencodable_text_is_replaced_not_fatal(rig: Rig) -> None:
    rig.replier.string(rig.req(), "€中", ErrorCluster(True, 1, "café ☃"))
    _, value, err, _ = read_reply(rig.client, ReplyKind.STR)
    assert value == "€?" and err.source == "café ?"
