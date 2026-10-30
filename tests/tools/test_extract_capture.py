"""tools/extract_capture.py: pcap → LabVIEW frames of the camera-server port (synthetic captures)."""

import gzip
import json
import struct
from pathlib import Path

import pytest

import extract_capture as xc

SERVER = (bytes([127, 0, 0, 1]), 503)
CLIENT = (bytes([127, 0, 0, 1]), 50001)
OTHER = (bytes([127, 0, 0, 1]), 504)


def lv_string(text: str) -> bytes:
    raw = text.encode("latin-1")
    return struct.pack(">i", len(raw)) + raw


def frame(datatype: int, payload: bytes) -> bytes:
    return struct.pack(">iH", len(payload) + 2, datatype) + payload


def command(code: int, notifier: str, rest: bytes = b"\x00" * 12) -> bytes:
    return frame(0, struct.pack(">H", code) + lv_string(notifier) + rest)


def response(notifier: str, body: bytes) -> bytes:
    return frame(1, lv_string(notifier) + body)


def ipv4_tcp(src, dst, seq: int, flags: int, payload: bytes) -> bytes:
    total = 40 + len(payload)
    ip = struct.pack(">BBHHHBBH4s4s", 0x45, 0, total if total <= 0xFFFF else 0, 0, 0x4000, 128, 6, 0, src[0], dst[0])
    tcp = struct.pack(">HHIIBBHHH", src[1], dst[1], seq & 0xFFFFFFFF, 0, 5 << 4, flags, 65535, 0, 0)
    return ip + tcp + payload


def write_pcap(path: Path, packets, linktype: int = 101) -> None:
    with open(path, "wb") as f:
        f.write(struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 262144, linktype))
        for i, packet in enumerate(packets):
            f.write(struct.pack("<IIII", 1000 + i // 1000, (i % 1000) * 1000, len(packet), len(packet)))
            f.write(packet)


class Conversation:
    """Builds the packets of one TCP connection; ``drop`` skips a segment (lost by the capture)."""

    def __init__(self, client=CLIENT, server=SERVER, syn=True):
        self.client, self.server = client, server
        self.seq = {"c2s": 1000, "s2c": 5000}
        self.packets = []
        if syn:
            self.packets.append(ipv4_tcp(client, server, self.seq["c2s"] - 1, 0x02, b""))
            self.packets.append(ipv4_tcp(server, client, self.seq["s2c"] - 1, 0x12, b""))

    def send(self, direction: str, data: bytes, segment: int = 65000, drop=()):
        src, dst = (self.client, self.server) if direction == "c2s" else (self.server, self.client)
        for index, start in enumerate(range(0, len(data), segment)):
            chunk = data[start:start + segment]
            if index not in drop:
                self.packets.append(ipv4_tcp(src, dst, self.seq[direction], 0x18, chunk))
            self.seq[direction] += len(chunk)

    def retransmit_last(self):
        self.packets.append(self.packets[-1])


def records(path: Path):
    with gzip.open(path, "rt", encoding="ascii") as f:
        return [json.loads(line) for line in f]


def run(tmp_path: Path, packets, **options):
    pcap, out = tmp_path / "capture.pcap", tmp_path / "out.jsonl.gz"
    write_pcap(pcap, packets, options.pop("linktype", 101))
    settings = dict(port=503, max_payload=1000, keep_large=1, head_bytes=16, progress=False)
    settings.update(options)
    output, streams, counters = xc.extract(str(pcap), str(out), **settings)
    return records(out), output, streams, counters


def test_frames_commands_notifiers_and_large_frame_truncation(tmp_path):
    conv = Conversation()
    conv.send("c2s", command(7, "n1"))                                   # Cam Set Exposure
    conv.send("s2c", response("n1", b"\x00" * 9))
    big = b"\xab" * 20000
    conv.send("s2c", response("img1", big), segment=6000)                 # kept whole (first large frame)
    conv.send("s2c", response("img2", big), segment=6000)                 # truncated to the head
    conv.send("c2s", frame(3, b"\x00\x01"))                              # heartbeat
    recs, output, streams, counters = run(tmp_path, conv.packets)

    frames = [r for r in recs if "type" in r]
    assert [(r["dir"], r["type"], r.get("cmd"), r.get("notifier")) for r in frames] == [
        ("c2s", "Command", "Cam Set Exposure", "n1"),
        ("s2c", "Response", None, "n1"),
        ("s2c", "Response", None, "img1"),
        ("s2c", "Response", None, "img2"),
        ("c2s", "Heartbeat", None, None),
    ]
    assert bytes.fromhex(frames[2]["hex"]) == lv_string("img1") + big
    assert frames[3]["truncated"] and len(bytes.fromhex(frames[3]["head"])) == 16
    assert frames[0]["hex"] == command(7, "n1")[6:].hex()
    assert output.commands == {"Cam Set Exposure": 1}
    assert counters["on_port"] == len(conv.packets)
    assert not any("gap" in r for r in recs)


def test_lost_segment_gives_a_gap_and_parsing_resumes_at_the_next_frame(tmp_path):
    conv = Conversation()
    conv.send("s2c", response("img1", b"\x01" * 20000), segment=6000, drop=(1,))   # a middle segment is lost
    conv.send("s2c", response("n2", b"\x00" * 9))                                   # next frame starts a segment
    recs, _, streams, _ = run(tmp_path, conv.packets)

    gaps = [r for r in recs if "gap" in r]
    assert len(gaps) == 1 and gaps[0]["gap"] == 6000 and gaps[0]["dir"] == "s2c"
    frames = [r for r in recs if "type" in r]
    assert [r.get("notifier") for r in frames] == ["n2"]                 # the damaged frame is skipped
    stream = streams[(1, "s2c")]
    assert stream.gaps == 1 and stream.gap_bytes == 6000 and stream.resyncs == 1


def test_capture_started_mid_stream_resyncs_on_a_frame_start(tmp_path):
    conv = Conversation(syn=False)
    conv.send("s2c", b"\x07" * 3000)                                     # tail of a frame sent before the capture
    conv.send("s2c", response("n3", b"\x00" * 9))
    conv.send("c2s", command(33, "n4"))                                  # Z Move Absolute
    recs, output, streams, _ = run(tmp_path, conv.packets)

    frames = [r for r in recs if "type" in r]
    assert [r.get("notifier") for r in frames] == ["n3", "n4"]
    assert frames[1]["cmd"] == "Z Move Absolute"
    assert streams[(1, "s2c")].started_mid_stream


def test_retransmission_and_other_ports_are_ignored(tmp_path):
    conv = Conversation()
    conv.send("c2s", command(0, "hb"))                                   # Get Heartbeat
    conv.retransmit_last()
    other = Conversation(client=(CLIENT[0], 50002), server=OTHER)
    other.send("c2s", command(1, "elsewhere"))
    recs, output, streams, counters = run(tmp_path, conv.packets + other.packets)

    assert [r.get("cmd") for r in recs if "type" in r] == ["Get Heartbeat"]
    assert streams[(1, "c2s")].retransmissions == 1
    assert counters["on_port"] == len(conv.packets)


def test_a_new_syn_from_the_same_client_port_is_a_new_connection(tmp_path):
    first, second = Conversation(), Conversation()
    first.send("c2s", command(0, "a"))
    second.send("c2s", command(0, "b"))
    recs, _, streams, _ = run(tmp_path, first.packets + second.packets)
    assert [(r["conn"], r["notifier"]) for r in recs if "type" in r] == [(1, "a"), (2, "b")]


def test_unknown_command_code_and_statut_datatype(tmp_path):
    conv = Conversation()
    conv.send("c2s", command(77, "x"))
    conv.send("s2c", frame(4, b"\x00\x00"))
    recs, output, _, _ = run(tmp_path, conv.packets)
    frames = [r for r in recs if "type" in r]
    assert frames[0]["cmd"] == "#77" and frames[1]["type"] == "Statut"


@pytest.mark.parametrize("linktype", [0, 1, 101, 228])
def test_link_layers(tmp_path, linktype):
    conv = Conversation()
    conv.send("c2s", command(2, "ls"))
    if linktype == 0:
        packets = [struct.pack("<I", 2) + p for p in conv.packets]
    elif linktype == 1:
        packets = [b"\x00" * 12 + b"\x08\x00" + p for p in conv.packets]
    else:
        packets = conv.packets
    recs, _, _, _ = run(tmp_path, packets, linktype=linktype)
    assert [r.get("cmd") for r in recs if "type" in r] == ["Cam Start"]


def test_pcapng_is_refused_with_a_clear_message(tmp_path):
    path = tmp_path / "capture.pcapng"
    path.write_bytes(b"\x0a\x0d\x0d\x0a" + b"\x00" * 40)
    with pytest.raises(xc.PcapError, match="pcapng"):
        list(xc.read_pcap(str(path)))


def test_main_writes_default_output_and_summary(tmp_path, capsys):
    conv = Conversation()
    conv.send("c2s", command(26, "z"))                                   # Z Setup
    write_pcap(tmp_path / "capture.pcap", conv.packets)
    assert xc.main([str(tmp_path / "capture.pcap")]) == 0
    out = capsys.readouterr().out
    assert (tmp_path / "capture_503.jsonl.gz").exists()
    assert "Z Setup" in out and "port 503" in out


def test_command_table_matches_the_packaged_enum_table():
    from aloxcam.wire.enums import WireEnums

    assert xc.CAMERA_COMMANDS == list(WireEnums.load().table("camera_command").items)
