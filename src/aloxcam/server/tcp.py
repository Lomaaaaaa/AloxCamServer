"""``TcpServer``: the listener and the reader thread (one client at a time, whole frames).

Spec: 02 §3 (connection model), §4 (state machine, outcomes of ``Read``), §5.1–§5.5 (frame layout, reading,
desynchronisation), §9 (heartbeat frames), §11.2–§11.3 (front end, ``listen_address``, ``inbound_pause_ms``,
``close_on_extra_heartbeat_data``), 02 D3–D6, D8, D9, D13, D16, D17, 01 §5.2, §10.3, 01 A4–A6;
docs/phase1-design.md §3.4.5, §7.

Behaviour: one client at a time; the listener stays open, so later clients wait in the backlog (02 §3). The reader
waits for the first header byte with no timeout (``select`` in 0.2 s slices to notice :meth:`TcpServer.stop`), then
reads the rest of the header and the payload within ``io_timeout_s``; a mid-frame timeout, EOF, ``N < 2`` or
``N - 2 > max_payload_bytes`` closes the connection (02 §5.5, D5, D6). ``Command`` frames go to the router; a
``Heartbeat`` frame with a payload closes the connection (``close_on_extra_heartbeat_data``) or is ignored, without
LabVIEW's 25 s stall (D13); other frames are ignored. The socket timeout is set once at accept and shared with
``ClientLink.send`` (01 §10.3), so it is never changed while reading (deadlines use ``select``).

Thread-safety: ``Router`` and the reader run on the TCP thread only; :meth:`stop` may be called from any thread.
"""

from __future__ import annotations

import logging
import select
import socket
import threading
import time

from aloxcam.clock import TickFn
from aloxcam.errors import ProtocolError
from aloxcam.server.link import ClientLink
from aloxcam.server.router import Router
from aloxcam.settings import CompatSwitches, ServerOptions
from aloxcam.state import ServerState
from aloxcam.wire.protocol import FRAME_HEADER_SIZE, Datatypes, parse_header

__all__ = ["TcpServer"]

log = logging.getLogger(__name__)

_SLICE_S = 0.2                    # select slice while idle (notice stop())
_JOIN_S = 2.0                     # stop(): join timeout
_ACCEPT_RETRY_S = 0.2             # pause after an unexpected accept error (02 D17: no busy loop)


class _ConnectionEnded(Exception):
    """The connection must be closed (EOF, mid-frame timeout, bad header, extra heartbeat data)."""


class TcpServer:
    """Listener + reader thread ``aloxcam-tcp`` (02 §3–§5, §9)."""

    def __init__(self, *, host: str, port: int, link: ClientLink, router: Router, state: ServerState,
                 datatypes: Datatypes, compat: CompatSwitches, options: ServerOptions, tick: TickFn) -> None:
        self.host = host
        self.port = port
        self.link = link
        self.router = router
        self.state = state
        self.datatypes = datatypes
        self.compat = compat
        self.options = options
        self.tick = tick
        self._listener: socket.socket | None = None
        self._address: tuple[str, int] | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._conn_lock = threading.Lock()          # guards _conn (stop() shuts it down from another thread)
        self._conn: socket.socket | None = None

    # ------------------------------------------------------------------ public API
    def bind(self) -> tuple[str, int]:
        """Create the listener now and return the bound ``(host, port)``; ``OSError`` propagates (02 D17: fail at
        start-up instead of retrying forever). Idempotent."""
        if self._listener is None:
            family = socket.AF_INET6 if ":" in self.host else socket.AF_INET
            listener = socket.create_server((self.host, self.port), family=family)
            listener.settimeout(_SLICE_S)
            self._listener = listener
            sockname = listener.getsockname()
            self._address = (str(sockname[0]), int(sockname[1]))
            log.debug("listening on %s:%d", *self._address)
        assert self._address is not None
        return self._address

    @property
    def address(self) -> tuple[str, int]:
        """The bound ``(host, port)``; :class:`RuntimeError` before :meth:`bind`."""
        if self._address is None:
            raise RuntimeError("TcpServer is not bound")
        return self._address

    def start(self) -> None:
        """Bind if needed and start the thread ``aloxcam-tcp`` (daemon)."""
        if self._thread is not None:
            raise RuntimeError("TcpServer already started")
        self.bind()
        self._thread = threading.Thread(target=self._run, name="aloxcam-tcp", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Close the listener and the current connection, then join the thread (≤ 2 s). Idempotent."""
        self._stop.set()
        with self._conn_lock:
            conn = self._conn
        if conn is not None:
            _shutdown(conn)                          # the reader sees EOF at once
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(_JOIN_S)
            if thread.is_alive():
                log.warning("TCP thread did not stop within %.1f s", _JOIN_S)
        if thread is None or not thread.is_alive():
            self._close_listener()

    # ------------------------------------------------------------------ accept loop
    def _run(self) -> None:
        listener = self._listener
        assert listener is not None
        try:
            while not self._stop.is_set():
                self._beat("Listen")
                try:
                    conn, peer = listener.accept()
                except TimeoutError:
                    continue
                except OSError as exc:
                    if self._stop.is_set():
                        break
                    log.error("accept failed: %r; retrying", exc)
                    self._stop.wait(_ACCEPT_RETRY_S)
                    continue
                self._serve(conn, peer)
        except Exception:
            log.critical("TCP thread crashed", exc_info=True)
        finally:
            self._close_listener()

    def _serve(self, conn: socket.socket, peer: object) -> None:
        """Serve one client until it disconnects or the connection must be closed (always back to listening)."""
        try:
            conn.settimeout(self.options.io_timeout_s)   # once, shared with ClientLink.send (01 §10.3)
            conn_id = self.link.attach(conn)
        except Exception:
            log.exception("cannot attach connection from %s", peer)
            _shutdown(conn)
            conn.close()
            return
        with self._conn_lock:
            self._conn = conn
        self.state.update(connected=True, server_state="Read")
        log.info("connection %d from %s", conn_id, _peer_text(peer))
        reason = "server stopping"
        try:
            while not self._stop.is_set():
                self._beat("Read")
                if not _readable(conn, _SLICE_S):
                    continue
                if self._stop.is_set():
                    break
                datatype, payload = self._read_frame(conn)
                self._dispatch(datatype, payload, conn_id)
        except _ConnectionEnded as exc:
            reason = str(exc)
        except OSError as exc:
            reason = f"socket error {exc!r}"
        except Exception:
            log.exception("connection %d: unexpected error in the TCP thread", conn_id)
            reason = "internal error"
        finally:
            _shutdown(conn)                              # a writer blocked in sendall fails at once
            self.link.detach(conn_id)                    # waits for that writer; then nobody uses the socket
            with self._conn_lock:
                self._conn = None
            conn.close()
            self.state.update(connected=False, server_state="Listen")   # 01 A6 fix
            log.info("connection %d closed (%s)", conn_id, reason)

    # ------------------------------------------------------------------ frames
    def _read_frame(self, conn: socket.socket) -> tuple[int, bytes]:
        """One whole frame after the first header byte became readable (02 §5.3, §5.5)."""
        first = conn.recv(FRAME_HEADER_SIZE)            # readable: returns at once, possibly fewer bytes
        if not first:
            raise _ConnectionEnded("closed by the client")
        deadline = time.monotonic() + self.options.io_timeout_s
        header = first + self._recv_exact(conn, FRAME_HEADER_SIZE - len(first), deadline, "frame header")
        try:
            fh = parse_header(header, self.options.max_payload_bytes)
        except ProtocolError as exc:
            raise _ConnectionEnded(f"bad frame header {header.hex(' ')}: {exc.source}") from None
        payload = self._recv_exact(conn, fh.payload_len, deadline, "payload") if fh.payload_len else b""
        return fh.datatype, payload

    def _recv_exact(self, conn: socket.socket, n: int, deadline: float, what: str) -> bytes:
        """``n`` bytes before ``deadline`` (a mid-frame timeout closes the connection: 02 §5.5, D5)."""
        buf = bytearray(n)
        view = memoryview(buf)
        got = 0
        while got < n:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not _readable(conn, remaining):
                raise _ConnectionEnded(f"timeout after {got} of {n} {what} bytes "
                                       f"(io_timeout_s = {self.options.io_timeout_s:g})")
            k = conn.recv_into(view[got:], n - got)
            if k == 0:
                raise _ConnectionEnded(f"closed by the client after {got} of {n} {what} bytes")
            got += k
        return bytes(buf)

    def _dispatch(self, datatype: int, payload: bytes, conn_id: int) -> None:
        """Datatype dispatch of a whole frame (02 §4.4, §5.2, §9)."""
        dt = self.datatypes
        if datatype == dt.command:
            try:
                self.router.route_frame(payload, conn_id)
            except Exception:
                log.exception("connection %d: routing failed; frame dropped", conn_id)
            if self.compat.inbound_pause_ms > 0:                 # LabVIEW's ≈ 50 ms gap (02 §4.6), off by default
                self._stop.wait(self.compat.inbound_pause_ms / 1000.0)
        elif datatype == dt.heartbeat:
            if payload and self.compat.close_on_extra_heartbeat_data:
                raise _ConnectionEnded(f"heartbeat frame with {len(payload)} payload bytes "
                                       "(LabVIEW: -1 Extra Data Found)")
            log.debug("connection %d: heartbeat frame (%d payload bytes) ignored", conn_id, len(payload))
        elif datatype in (dt.response, dt.image):
            log.debug("connection %d: %s frame (%d bytes) ignored", conn_id, dt.name_of(datatype), len(payload))
        else:
            log.warning("connection %d: frame with unknown datatype %d (%d bytes) ignored", conn_id, datatype,
                        len(payload))

    # ------------------------------------------------------------------ helpers
    def _beat(self, server_state: str) -> None:
        """``Heartbeat`` (TCP-loop liveness tick) and ``Server State`` indicators (01 §8.3)."""
        self.state.update(heartbeat=self.tick(), server_state=server_state)

    def _close_listener(self) -> None:
        listener, self._listener = self._listener, None
        if listener is not None:
            try:
                listener.close()
            except OSError:
                pass


def _readable(sock: socket.socket, timeout: float) -> bool:
    readable, _, _ = select.select([sock], [], [], max(0.0, timeout))
    return bool(readable)


def _shutdown(sock: socket.socket) -> None:
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass


def _peer_text(peer: object) -> str:
    if isinstance(peer, tuple) and len(peer) >= 2:
        return f"{peer[0]}:{peer[1]}"
    return str(peer)
