"""``ClientLink``: the current client connection; whole reply frames are written under one lock.

Spec: 01 §10.2–§10.3 (workers write their replies directly, one lock per frame), 02 §4.6 (frames never interleave),
02 D10 (replies of an earlier connection are dropped), 02 D12 (a failed write closes the connection),
01 §6.3 / §7.1 (replies produced after ``Exit`` are lost); docs/phase1-design.md §3.4.1.

Every connection gets a generation number (``conn_id`` 1, 2, …) when it is attached; a request carries the
``conn_id`` of the connection it arrived on, and :meth:`ClientLink.send` drops a frame whose ``conn_id`` is not the
current one. ``conn_id`` 0 marks internal requests, which are never replied.

Thread-safety: :meth:`send` may be called from any thread. The send lock is a leaf lock (design §2): nothing else
is called while it is held, except the socket write itself.
"""

from __future__ import annotations

import logging
import socket
import threading

__all__ = ["ClientLink"]

log = logging.getLogger(__name__)


class ClientLink:
    """The current client connection (01 §10.3). Thread-safe: ``send()`` may be called from any thread."""

    def __init__(self) -> None:
        self._lock = threading.Lock()          # the send lock: one whole frame at a time
        self._sock: socket.socket | None = None
        self._conn_id: int | None = None
        self._last_id = 0
        self._broken = False                   # a write failed on the current connection (02 D12)
        self._closed = threading.Event()       # set by close(): every later send() returns False

    def attach(self, sock: socket.socket) -> int:
        """Make ``sock`` the current connection and return its new ``conn_id`` (1, 2, …).

        The previous connection must have been detached (:class:`RuntimeError` otherwise). The socket's timeout
        must already be set: it bounds every whole-frame write (02 D12).
        """
        with self._lock:
            if self._sock is not None:
                raise RuntimeError(f"ClientLink.attach: connection {self._conn_id} is still attached")
            self._last_id += 1
            self._sock, self._conn_id, self._broken = sock, self._last_id, False
            return self._last_id

    def detach(self, conn_id: int) -> None:
        """Forget connection ``conn_id`` (no-op if it is not the current one).

        Waits for a write in progress, so the caller may close the socket afterwards without racing a writer.
        """
        with self._lock:
            if self._conn_id == conn_id:
                self._sock, self._conn_id, self._broken = None, None, False

    @property
    def current_id(self) -> int | None:
        """``conn_id`` of the attached connection, or None."""
        return self._conn_id

    @property
    def closed(self) -> bool:
        """True once :meth:`close` was called."""
        return self._closed.is_set()

    def send(self, frame: bytes, conn_id: int) -> bool:
        """Write one whole frame to connection ``conn_id``; True when it was written.

        Returns False (frame dropped, DEBUG log) when ``conn_id`` is 0 (internal request), is not the current
        connection (02 D10), the link is closed (shutdown, 01 §6.3) or an earlier write on this connection failed.
        On ``OSError`` / timeout the socket is shut down, so the reader sees EOF and goes back to listening
        (02 D12): a half-written frame would desynchronise the client.
        """
        if conn_id == 0:
            log.debug("reply of an internal request dropped (%d bytes)", len(frame))
            return False
        with self._lock:
            sock = self._sock
            if self._closed.is_set() or sock is None or conn_id != self._conn_id or self._broken:
                log.debug("reply for connection %d dropped (current %s, closed %s, %d bytes)", conn_id,
                          self._conn_id, self._closed.is_set(), len(frame))
                return False
            try:
                sock.sendall(frame)
                return True
            except OSError as exc:              # includes TimeoutError (socket timeout, 02 D12)
                self._broken = True
                log.warning("connection %d: reply write failed (%r); closing the connection", conn_id, exc)
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                return False

    def close(self) -> None:
        """Shutdown chain (01 §7.1): every later ``send()`` returns False. Never blocks; the socket itself is
        closed by the TCP server."""
        self._closed.set()
