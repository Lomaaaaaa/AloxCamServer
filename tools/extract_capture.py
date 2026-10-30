#!/usr/bin/env python3
"""Extract the ALOX camera-server traffic from a large loopback capture.

Reads a pcap file (RawCap, or Wireshark/tcpdump saved as libpcap "pcap"), keeps only the TCP traffic of one port (503 by
default), reassembles each TCP stream, splits it into LabVIEW frames ``[I32 length][U16 datatype][payload]``
(docs/02-tcp-protocol.md §5) and writes them to a small gzip-compressed JSON-lines file.

Standalone: it needs only the Python standard library (Python 3.6 or later), so it runs on the lab PC without installing
anything::

    py extract_capture.py capture.pcap

Output: ``capture_503.jsonl.gz`` next to the capture, and a summary on the screen. Large frames (images) are kept whole
for the first few of each direction; the others keep only their first bytes. Lost packets ("Buffer is full" in RawCap)
are reported as gaps, and parsing resumes at the next frame that starts a TCP segment.
"""

import argparse
import gzip
import json
import os
import struct
import sys
import time

DATATYPES = {0: "Command", 1: "Response", 2: "Image", 3: "Heartbeat", 4: "Statut"}

# typedef_ext_camera_camera_command.ctl, item order confirmed by the owner (docs/answers/owner-answers.md §1).
CAMERA_COMMANDS = [
    "Get Heartbeat", "Cam Setup", "Cam Start", "Cam Get Image", "Cam Stream Next Image", "Cam Get Acquisition Status",
    "Cam Software Trigger", "Cam Set Exposure", "Cam Get Exposure", "Cam Set Gain", "Cam Get Gain",
    "Cam Get Images Array", "Cam Stop", "Cam Cleanup", "Exit", "Show UI", "Cam Autofocus", "Cam Set FlatField",
    "Cam Refocus", "Cam Flip", "Cam Get Autofocus Data", "Cam Get Autofocus Progress", "Cam Autofocus Run",
    "Cam Refocus Run", "Cam Set Led", "Cam Get Led", "Z Setup", "Z Get Serial", "Z Get Status", "Z Get Error",
    "Z Set Speed", "Z Get Speed", "Z Get Position", "Z Move Absolute", "Z Move Relative", "Z Wait End Move", "Z Stop",
    "Z Enable Joystick", "Z Disable Joystick", "Z Joystick Enabled ?", "Z Set Min Limit", "Z Set Max Limit",
    "Z Cleanup", "Z Get Reference", "Z Do Reference", "Log",
]

MAX_FRAME = 256 * 1024 * 1024        # larger I32 lengths are treated as a lost frame boundary
SEQ_MASK = 0xFFFFFFFF
TCP_SYN = 0x02
C2S, S2C = "c2s", "s2c"              # client → server, server → client


class PcapError(Exception):
    pass


def read_pcap(path):
    """Yield ``(timestamp, linktype, packet_bytes, truncated)`` for every record of a libpcap file."""
    with open(path, "rb") as f:
        header = f.read(24)
        if len(header) < 24:
            raise PcapError("not a pcap file (too short)")
        magic = header[:4]
        if magic == b"\xd4\xc3\xb2\xa1":
            endian, scale = "<", 1e-6
        elif magic == b"\xa1\xb2\xc3\xd4":
            endian, scale = ">", 1e-6
        elif magic == b"\x4d\x3c\xb2\xa1":
            endian, scale = "<", 1e-9
        elif magic == b"\xa1\xb2\x3c\x4d":
            endian, scale = ">", 1e-9
        elif magic == b"\x0a\x0d\x0d\x0a":
            raise PcapError("this is a pcapng file: save it as \"Wireshark/tcpdump - pcap\" (RawCap writes pcap)")
        else:
            raise PcapError("not a pcap file (unknown magic number {})".format(magic.hex()))
        linktype = struct.unpack(endian + "I", header[20:24])[0] & 0xFFFF
        record = struct.Struct(endian + "IIII")
        while True:
            head = f.read(16)
            if len(head) < 16:
                return
            sec, frac, incl_len, orig_len = record.unpack(head)
            data = f.read(incl_len)
            if len(data) < incl_len:
                return                                      # capture cut off in the middle of a record
            yield sec + frac * scale, linktype, data, incl_len < orig_len


def ip_packet(linktype, data):
    """Strip the link layer; return the IP packet bytes, or None for anything that is not IPv4."""
    if linktype in (101, 12, 14, 228):                      # raw IP (RawCap) / IPv4
        ip = data
    elif linktype in (0, 108):                              # BSD loopback / OpenBSD loopback: 4-byte family
        ip = data[4:]
    elif linktype == 1:                                     # Ethernet, with optional VLAN tags
        if len(data) < 14:
            return None
        ethertype, offset = struct.unpack(">H", data[12:14])[0], 14
        while ethertype in (0x8100, 0x88A8) and len(data) >= offset + 4:
            ethertype, offset = struct.unpack(">H", data[offset + 2:offset + 4])[0], offset + 4
        if ethertype != 0x0800:
            return None
        ip = data[offset:]
    elif linktype == 113:                                   # Linux cooked capture
        ip = data[16:] if data[14:16] == b"\x08\x00" else None
    elif linktype == 276:                                   # Linux cooked capture v2
        ip = data[20:] if data[0:2] == b"\x08\x00" else None
    else:
        return None
    if ip is None or len(ip) < 20 or ip[0] >> 4 != 4:
        return None
    return ip


def tcp_segment(ip):
    """Return ``(src, sport, dst, dport, seq, flags, payload)`` for an unfragmented IPv4/TCP packet, else None."""
    if ip[9] != 6:
        return None
    if struct.unpack(">H", ip[6:8])[0] & 0x3FFF:            # IP fragment: not expected on loopback
        return None
    ihl = (ip[0] & 0x0F) * 4
    total = struct.unpack(">H", ip[2:4])[0]
    if total >= ihl:                                        # 0 = segmentation offload: keep the whole packet
        ip = ip[:total] if total else ip
    tcp = ip[ihl:]
    if len(tcp) < 20:
        return None
    sport, dport, seq = struct.unpack(">HHI", tcp[:8])
    data_offset = (tcp[12] >> 4) * 4
    return ip[12:16], sport, ip[16:20], dport, seq, tcp[13], tcp[data_offset:]


def looks_like_frame_start(data):
    if len(data) < 6:
        return False
    length = struct.unpack(">i", data[0:4])[0]
    datatype = struct.unpack(">H", data[4:6])[0]
    return 2 <= length <= MAX_FRAME and datatype in DATATYPES


def lv_string(data, offset):
    """Read a LabVIEW flattened string (I32 length + bytes); return ``(text, next_offset)`` or ``(None, offset)``."""
    if len(data) < offset + 4:
        return None, offset
    length = struct.unpack(">i", data[offset:offset + 4])[0]
    if length < 0 or length > 4096 or len(data) < offset + 4 + length:
        return None, offset
    return data[offset + 4:offset + 4 + length].decode("latin-1"), offset + 4 + length


class Output(object):
    """Writes the JSON-lines records and keeps the statistics for the summary."""

    def __init__(self, path, max_payload, keep_large, head_bytes):
        self.file = gzip.open(path, "wt", encoding="ascii", compresslevel=6)
        self.max_payload, self.keep_large, self.head_bytes = max_payload, keep_large, head_bytes
        self.records = 0
        self.t0 = None
        self.large_kept = {C2S: 0, S2C: 0}
        self.frames = {}                                    # (direction, datatype name) → count
        self.commands = {}                                  # command name → count
        self.truncated = 0

    def write(self, record):
        self.file.write(json.dumps(record, separators=(",", ":")) + "\n")
        self.records += 1

    def frame(self, t, conn, direction, datatype, payload):
        if self.t0 is None:
            self.t0 = t
        name = DATATYPES[datatype]
        self.frames[(direction, name)] = self.frames.get((direction, name), 0) + 1
        record = {"t": round(t - self.t0, 6), "conn": conn, "dir": direction, "type": name, "len": len(payload)}
        if direction == C2S and datatype == 0 and len(payload) >= 2:
            code = struct.unpack(">H", payload[0:2])[0]
            command = CAMERA_COMMANDS[code] if code < len(CAMERA_COMMANDS) else "#{}".format(code)
            record["cmd"] = command
            self.commands[command] = self.commands.get(command, 0) + 1
            notifier, _ = lv_string(payload, 2)
            if notifier is not None:
                record["notifier"] = notifier
        elif direction == S2C and datatype in (1, 2):
            notifier, _ = lv_string(payload, 0)
            if notifier is not None:
                record["notifier"] = notifier
        if len(payload) <= self.max_payload or self.large_kept[direction] < self.keep_large:
            if len(payload) > self.max_payload:
                self.large_kept[direction] += 1
            record["hex"] = payload.hex()
        else:
            record["head"] = payload[:self.head_bytes].hex()
            record["truncated"] = True
            self.truncated += 1
        self.write(record)

    def gap(self, t, conn, direction, lost):
        if self.t0 is None:
            self.t0 = t
        self.write({"t": round(t - self.t0, 6), "conn": conn, "dir": direction, "gap": lost})

    def close(self):
        self.file.close()


class Stream(object):
    """One direction of one TCP connection: reassembly and frame splitting."""

    def __init__(self, conn, direction, output):
        self.conn, self.direction, self.output = conn, direction, output
        self.next_seq = None
        self.buffer = bytearray()
        self.synced = True
        self.packets = self.bytes = self.frames = 0
        self.gaps = self.gap_bytes = self.retransmissions = self.resyncs = 0
        self.started_mid_stream = False

    def segment(self, t, seq, flags, payload, truncated):
        self.packets += 1
        if flags & TCP_SYN:
            self.next_seq = (seq + 1) & SEQ_MASK
            self.buffer = bytearray()
            self.synced = True
            return
        if not payload:
            return
        segment_start = True
        if self.next_seq is None:                           # capture started after the connection
            self.next_seq = seq
            self.started_mid_stream = True
            self.synced = False
        ahead = (seq - self.next_seq) & SEQ_MASK
        if ahead and ahead < 0x80000000:                    # bytes missing before this segment
            self.lost(t, ahead)
            self.next_seq = seq
        elif ahead:                                         # retransmission or overlap
            behind = (self.next_seq - seq) & SEQ_MASK
            if behind >= len(payload):
                self.retransmissions += 1
                return
            payload = payload[behind:]
            segment_start = False
        self.next_seq = (self.next_seq + len(payload)) & SEQ_MASK
        self.bytes += len(payload)
        self.feed(t, payload, segment_start)
        if truncated:                                       # snap length cut the packet: its tail is lost
            self.lost(t, 0)

    def lost(self, t, count):
        self.gaps += 1
        self.gap_bytes += count
        self.output.gap(t, self.conn, self.direction, count)
        self.buffer = bytearray()
        self.synced = False

    def feed(self, t, data, segment_start):
        if not self.synced:
            if not (segment_start and looks_like_frame_start(data)):
                return
            self.synced = True
            self.resyncs += 1
        self.buffer += data
        while len(self.buffer) >= 6:
            length = struct.unpack(">i", bytes(self.buffer[0:4]))[0]
            datatype = struct.unpack(">H", bytes(self.buffer[4:6]))[0]
            if not (2 <= length <= MAX_FRAME and datatype in DATATYPES):
                self.lost(t, 0)                             # not a frame boundary: wait for the next one
                return
            if len(self.buffer) < 4 + length:
                return
            payload = bytes(self.buffer[6:4 + length])
            del self.buffer[:4 + length]
            self.frames += 1
            self.output.frame(t, self.conn, self.direction, datatype, payload)


def extract(pcap_path, out_path, port, max_payload, keep_large, head_bytes, progress=True):
    """Run the extraction; return ``(output, streams, counters)``."""
    output = Output(out_path, max_payload, keep_large, head_bytes)
    streams, connections = {}, {}
    counters = {"records": 0, "ipv4_tcp": 0, "on_port": 0, "other_link": 0}
    total = os.path.getsize(pcap_path)
    processed, last_report = 24, time.time()
    try:
        for t, linktype, data, truncated in read_pcap(pcap_path):
            counters["records"] += 1
            processed += 16 + len(data)
            if progress and time.time() - last_report > 5:
                last_report = time.time()
                sys.stderr.write("  {:.0f} %  ({} MB of {} MB)\n".format(
                    100.0 * processed / total, processed // 2**20, total // 2**20))
            ip = ip_packet(linktype, data)
            if ip is None:
                counters["other_link"] += 1
                continue
            segment = tcp_segment(ip)
            if segment is None:
                continue
            counters["ipv4_tcp"] += 1
            src, sport, dst, dport, seq, flags, payload = segment
            if dport == port:
                direction, client = C2S, (src, sport)
            elif sport == port:
                direction, client = S2C, (dst, dport)
            else:
                continue
            counters["on_port"] += 1
            if client not in connections or (direction == C2S and flags & TCP_SYN and not flags & 0x10):
                connections[client] = len(connections) + 1  # a new SYN from the same client port = new connection
            conn = connections[client]
            key = (conn, direction)
            if key not in streams:
                streams[key] = Stream(conn, direction, output)
            streams[key].segment(t, seq, flags, payload, truncated)
    finally:
        output.close()
    return output, streams, counters


def print_summary(pcap_path, out_path, port, output, streams, counters):
    print("ALOX camera-server capture extraction")
    print("  capture   {}  ({} MB, {} packets)".format(pcap_path, os.path.getsize(pcap_path) // 2**20,
                                                         counters["records"]))
    print("  port {}   {} packets of {} IPv4/TCP packets".format(port, counters["on_port"], counters["ipv4_tcp"]))
    print("  output    {}  ({:.1f} MB, {} records)".format(out_path, os.path.getsize(out_path) / 2**20, output.records))
    if not streams:
        print("\nNo traffic on port {} was found. Was the camera server running, and is its port {}?".format(port, port))
        return
    print("\nconnections")
    for (conn, direction), s in sorted(streams.items()):
        notes = []
        if s.started_mid_stream:
            notes.append("started before the capture")
        if s.gaps:
            notes.append("{} gaps ({} bytes lost)".format(s.gaps, s.gap_bytes))
        if s.retransmissions:
            notes.append("{} retransmissions".format(s.retransmissions))
        print("  #{:<3} {}  {:>9} packets {:>12} bytes {:>8} frames  {}".format(
            conn, direction, s.packets, s.bytes, s.frames, ", ".join(notes)))
    print("\nframes")
    for (direction, name), count in sorted(output.frames.items()):
        print("  {}  {:<10} {}".format(direction, name, count))
    if output.commands:
        print("\ncommands sent by the client")
        for name, count in sorted(output.commands.items(), key=lambda item: -item[1]):
            print("  {:>7}  {}".format(count, name))
    print("\nlarge frames kept whole: {} client→server, {} server→client; {} others truncated to their first {} bytes"
          .format(output.large_kept[C2S], output.large_kept[S2C], output.truncated, output.head_bytes))


def main(argv=None):
    parser = argparse.ArgumentParser(description="Extract the ALOX camera-server frames from a pcap capture.")
    parser.add_argument("pcap", help="capture file (pcap format, e.g. from RawCap)")
    parser.add_argument("--port", type=int, default=503, help="camera server TCP port (default 503)")
    parser.add_argument("--out", help="output file (default: <capture folder>/capture_<port>.jsonl.gz)")
    parser.add_argument("--max-payload", type=int, default=65536,
                        help="frames up to this size are always kept whole (default 65536 bytes)")
    parser.add_argument("--keep-large", type=int, default=3,
                        help="number of larger frames kept whole in each direction (default 3)")
    parser.add_argument("--head", type=int, default=256,
                        help="bytes kept from the other large frames (default 256)")
    args = parser.parse_args(argv)
    out_path = args.out or os.path.join(os.path.dirname(os.path.abspath(args.pcap)),
                                        "capture_{}.jsonl.gz".format(args.port))
    try:
        result = extract(args.pcap, out_path, args.port, args.max_payload, args.keep_large, args.head)
    except (PcapError, OSError) as exc:
        sys.stderr.write("extract_capture: {}\n".format(exc))
        return 1
    print_summary(args.pcap, out_path, args.port, *result)
    return 0


if __name__ == "__main__":
    sys.exit(main())
