"""Protocol client of the ALOX camera server: commands out, replies demultiplexed by notifier name.

Spec: 02 §10 (notifier-name correlation, the client's synchronous-call pattern, rules 1–5), 02 §5 (frames), §7.1 /
§7.4 (``Camera Queue Data`` envelope, ``Command Data`` variant), §8.1–§8.3 (``HW TCP Response Data``,
``Response (Flattened)``), §9 (``Heartbeat`` frames); docs/phase1-design.md §3.7.1.

Used by the integration tests and by ``tools/replay_client.py``. A reader thread (``aloxcam-client-reader``) reads
whole frames and files every ``Response`` under its notifier; :meth:`AloxClient.wait_reply` takes the oldest reply
of a notifier. Replies may arrive in any order (02 §10 rule 4): correlation is by name only, the notifier being opaque
bytes (rule 1). Replies are kept until claimed, so a reply that nobody waits for stays available.
"""

from __future__ import annotations

import itertools
import logging
import os
import socket
import threading
import time
from collections import deque
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from aloxcam.dispatch import CommandSpec, ReplyKind
from aloxcam.errors import ErrorCluster, ProtocolError
from aloxcam.lv.flatten import Reader
from aloxcam.lv.variant import EMPTY_VARIANT_HEADER, read_variant_header, write_variant
from aloxcam.wire.enums import WireEnums
from aloxcam.wire.messages import COMMANDS, PayloadCodec, payload_from_mapping
from aloxcam.wire.protocol import (
    FRAME_HEADER_SIZE,
    Datatypes,
    ResponseTimestamps,
    decode_response_data,
    encode_envelope,
    encode_frame,
    heartbeat_frame,
    parse_header,
    parse_reply_value,
)

__all__ = ["MAX_REPLY_BYTES", "AloxClient", "ReceivedReply"]

log = logging.getLogger(__name__)

#: Largest reply payload the client accepts (a full 2448 × 2048 U16 image is ~10 MB).
MAX_REPLY_BYTES = 512 * 2**20
_RECENT_NOTIFIERS = 10_000
_JOIN_S = 2.0


@dataclass(frozen=True)
class ReceivedReply:
    """One ``Response`` frame as received (02 §8.1). ``received_at`` is ``time.monotonic()`` at receipt."""

    notifier: bytes
    flattened: bytes
    ts: ResponseTimestamps
    frame: bytes
    received_at: float

    def value(self, kind: ReplyKind, *, encoding: str = "cp1252") -> tuple[Any, ErrorCluster]:
        """``(value, error)`` of ``Response (Flattened)`` decoded as ``kind`` (02 §8.3): ACK → None, IMAGE → bytes.
        :class:`aloxcam.errors.ProtocolError` when the bytes do not decode exactly as ``kind``."""
        return parse_reply_value(kind, self.flattened, encoding=encoding)


class AloxClient:
    """Protocol client: a reader thread demultiplexes replies by notifier name. Thread-safe."""

    def __init__(self, host: str, port: int, *, enums: WireEnums, payloads: PayloadCodec,
                 timeout_s: float = 5.0) -> None:
        self.host = host
        self.port = int(port)
        self.enums = enums
        self.payloads = payloads
        self.timeout_s = float(timeout_s)
        self.datatypes = Datatypes.from_enums(enums)
        self.commands: Mapping[str, CommandSpec] = COMMANDS
        self._sock: socket.socket | None = None
        self._send_lock = threading.Lock()
        self._cond = threading.Condition()
        self._pending: dict[bytes, deque[ReceivedReply]] = {}
        self._recent: deque[bytes] = deque(maxlen=_RECENT_NOTIFIERS)
        self._reply_count = 0
        self._ended = threading.Event()          # the reader stopped (EOF, error or close())
        self._closing = False
        self._reader: threading.Thread | None = None
        self._seq = itertools.count(1)
        self.end_reason: str | None = None       # why the connection ended (None while connected)

    # ------------------------------------------------------------------ connection
    def connect(self) -> None:
        """Connect (``OSError`` propagates) and start the reader thread. A client connects once."""
        if self._sock is not None:
            raise RuntimeError("AloxClient.connect: already connected")
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        sock.settimeout(None)                    # the reader blocks; close() wakes it with a shutdown
        self._sock = sock
        self._reader = threading.Thread(target=self._read_loop, args=(sock,), name="aloxcam-client-reader",
                                        daemon=True)
        self._reader.start()
        log.debug("connected to %s:%d", self.host, self.port)

    def close(self) -> None:
        """Close the connection and stop the reader thread; idempotent. Unclaimed replies stay readable."""
        sock = self._sock
        if sock is None:                         # never connected
            return
        self._closing = True
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass
        reader = self._reader
        if reader is not None and reader is not threading.current_thread():
            reader.join(_JOIN_S)

    def __enter__(self) -> AloxClient:
        """Connects unless :meth:`connect` was already called."""
        if self._sock is None:
            self.connect()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    @property
    def connected(self) -> bool:
        """False before :meth:`connect`, and once the server closed the connection (or :meth:`close` ran)."""
        return self._sock is not None and not self._ended.is_set()

    def wait_closed(self, timeout_s: float) -> bool:
        """Wait until the connection has ended (the server closed it, or :meth:`close`); True when it has."""
        if self._sock is None:
            return True
        return self._ended.wait(max(0.0, timeout_s))

    # ------------------------------------------------------------------ sending
    def new_notifier(self) -> bytes:
        """A fresh default notifier name ``b"py-<pid>-<seq>"`` (02 §10: unique per request)."""
        return f"py-{os.getpid()}-{next(self._seq)}".encode("ascii")

    def build_command_frame(self, command: str, data: Any | None = None, *, notifier: bytes | str | None = None,
                            ts: tuple[int, int, int] = (0, 0, 0),
                            data_bytes: bytes | None = None) -> tuple[bytes, bytes]:
        """A whole ``Command`` frame and its notifier (02 §7.1), without sending it.

        ``command`` is a ``Camera Command`` label, or ``"#<code>"`` for a numeric code (a code without a known command
        gets an empty variant). ``data`` is a payload dataclass or a mapping of its fields
        (:func:`~aloxcam.wire.messages.payload_from_mapping`); commands whose ``Command Data`` LabVIEW never reads
        accept none. ``data_bytes`` replaces the encoded data part, keeping the command's variant header
        (malformed-payload tests). :class:`ValueError` for an unknown label or unusable data.
        """
        code = self.enums.code("camera_command", command)
        spec = self.commands.get(command)
        if notifier is None:
            notifier = self.new_notifier()
        elif isinstance(notifier, str):
            notifier = notifier.encode(self.payloads.encoding)
        variant = self._variant(command, spec, data, data_bytes)
        payload = encode_envelope(code, notifier, variant, self.enums, ts)
        return encode_frame(self.datatypes.command, payload), bytes(notifier)

    def _variant(self, command: str, spec: CommandSpec | None, data: Any | None, data_bytes: bytes | None) -> bytes:
        if spec is None:
            if data is not None:
                raise ValueError(f"{command}: no command with this code, so no Command Data can be encoded")
            return write_variant(EMPTY_VARIANT_HEADER, b"" if data_bytes is None else bytes(data_bytes))
        if data is not None:
            if spec.payload is None:
                raise ValueError(f"{command}: this command takes no Command Data")
            if isinstance(data, Mapping):
                data = payload_from_mapping(spec.payload, data)
            elif not isinstance(data, spec.payload):
                raise ValueError(f"{command}: {spec.payload.__name__} or a mapping expected, got "
                                 f"{type(data).__name__}")
        variant = self.payloads.variant_for(spec, data)
        if data_bytes is None:
            return variant
        header = read_variant_header(Reader(variant)).raw
        return write_variant(header, bytes(data_bytes))

    def send(self, command: str, data: Any | None = None, *, notifier: bytes | None = None,
             ts: tuple[int, int, int] = (0, 0, 0), data_bytes: bytes | None = None) -> bytes:
        """Send one command (see :meth:`build_command_frame`); returns its notifier."""
        frame, name = self.build_command_frame(command, data, notifier=notifier, ts=ts, data_bytes=data_bytes)
        self.send_raw(frame)
        return name

    def send_raw(self, frame: bytes) -> None:
        """Write raw bytes (one or several frames, or a malformed one); :class:`ConnectionError` when not connected
        or when the write fails."""
        sock = self._sock
        if sock is None or self._ended.is_set():
            raise ConnectionError(f"not connected ({self.end_reason or 'never connected'})")
        with self._send_lock:
            try:
                sock.sendall(frame)
            except OSError as exc:
                raise ConnectionError(f"send failed: {exc}") from exc

    def send_heartbeat(self, payload: bytes = b"") -> None:
        """A ``Heartbeat`` frame ``00 00 00 02 tt tt`` (02 §5.4); a payload makes the server close (02 §9)."""
        self.send_raw(heartbeat_frame(self.datatypes, payload))

    # ------------------------------------------------------------------ replies
    def wait_reply(self, notifier: bytes, timeout_s: float | None = None) -> ReceivedReply | None:
        """The oldest unclaimed reply for ``notifier``, waiting up to ``timeout_s`` (default: the client timeout);
        None on timeout, or as soon as the connection has ended with no such reply."""
        timeout = self.timeout_s if timeout_s is None else max(0.0, timeout_s)
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                queue = self._pending.get(notifier)
                if queue:
                    reply = queue.popleft()
                    if not queue:
                        del self._pending[notifier]
                    return reply
                if self._ended.is_set():
                    return None
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cond.wait(remaining)

    def call(self, command: str, data: Any | None = None, *, timeout_s: float | None = None) -> ReceivedReply | None:
        """Synchronous call (02 §10): send with a fresh notifier and wait for its reply (None = timeout)."""
        return self.wait_reply(self.send(command, data), timeout_s)

    @property
    def reply_count(self) -> int:
        """Number of ``Response`` frames received so far (claimed or not)."""
        with self._cond:
            return self._reply_count

    def received_notifiers(self) -> list[bytes]:
        """Notifiers of the replies received so far, in arrival order (the last 10 000)."""
        with self._cond:
            return list(self._recent)

    def unclaimed(self) -> dict[bytes, int]:
        """Notifier → number of replies received and not yet taken by :meth:`wait_reply`."""
        with self._cond:
            return {name: len(queue) for name, queue in self._pending.items() if queue}

    # ------------------------------------------------------------------ reader thread
    @staticmethod
    def _recv_exact(sock: socket.socket, n: int) -> bytes | None:
        chunks: list[bytes] = []
        have = 0
        while have < n:
            chunk = sock.recv(min(n - have, 1 << 20))
            if not chunk:
                return None
            chunks.append(chunk)
            have += len(chunk)
        return b"".join(chunks)

    def _read_loop(self, sock: socket.socket) -> None:
        reason = "closed by the server"
        try:
            while True:
                header = self._recv_exact(sock, FRAME_HEADER_SIZE)
                if header is None:
                    break
                try:
                    fh = parse_header(header, MAX_REPLY_BYTES)
                except ProtocolError as exc:
                    reason = f"bad frame header: {exc.source}"
                    log.warning("reply stream desynchronised (%s): closing", exc.source)
                    break
                payload = self._recv_exact(sock, fh.payload_len) if fh.payload_len else b""
                if payload is None:
                    reason = "closed by the server inside a frame"
                    break
                self._on_frame(fh.datatype, header + payload, payload)
        except OSError as exc:
            reason = "closed by the client" if self._closing else f"connection error: {exc}"
        finally:
            if self._closing and reason == "closed by the server":
                reason = "closed by the client"
            self.end_reason = reason
            with self._cond:
                self._ended.set()
                self._cond.notify_all()
            log.debug("connection to %s:%d ended: %s", self.host, self.port, reason)

    def _on_frame(self, datatype: int, frame: bytes, payload: bytes) -> None:
        if datatype != self.datatypes.response:
            log.warning("ignored a frame of datatype %d (%d bytes) from the server", datatype, len(payload))
            return
        try:
            data = decode_response_data(payload)
        except ProtocolError as exc:
            log.warning("ignored an undecodable Response frame (%s)", exc.source)
            return
        reply = ReceivedReply(notifier=data.notifier, flattened=data.flattened, ts=data.ts, frame=frame,
                              received_at=time.monotonic())
        with self._cond:
            self._pending.setdefault(reply.notifier, deque()).append(reply)
            self._recent.append(reply.notifier)
            self._reply_count += 1
            self._cond.notify_all()
