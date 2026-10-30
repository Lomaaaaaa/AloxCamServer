"""``ServerRuntime``: assembles link, replier, router, the two workers and the TCP server, and runs the shutdown chain.

Spec: 01 §7 (shutdown chain), §10.1–§10.3 (threads, queues, cleanup on ``Exit``), 01 A10 (hardware closed on
``Exit`` / signal), 01 §6.3 / 03 §3.4 (replies after ``Exit`` lost, later commands discarded), 08 §13.3 (cancelling a
running Z wait); docs/phase1-design.md §2, §3.4.6.

Shutdown chain (thread ``aloxcam-shutdown``, started by :meth:`ServerRuntime.start`; it waits for the camera worker
to stop on ``Exit``, from the client or posted by :meth:`ServerRuntime.request_shutdown`):

``link.close()`` (later replies dropped) → ``tcp.stop()`` → join the Z worker (≤ ``shutdown_timeout_s``; it ends on the
``Exit`` the camera worker forwarded, after the Z commands queued before it) → on timeout ``abort_waits()`` and join
again (≤ 2 s) → ``camera_background.abort()`` → ``close_hardware()`` → stopped (:meth:`ServerRuntime.wait` True).
"""

from __future__ import annotations

import logging
import threading
from collections.abc import Callable, Mapping
from dataclasses import dataclass

from aloxcam.clock import TickFn
from aloxcam.dispatch import BackgroundStepper, CommandSpec, EventSink, HandlerTable, Request, WorkerContext
from aloxcam.imagedata import ImageCodec
from aloxcam.server.link import ClientLink
from aloxcam.server.reply import Replier
from aloxcam.server.router import Router
from aloxcam.server.tcp import TcpServer
from aloxcam.server.worker import Worker
from aloxcam.settings import ServerConfig
from aloxcam.state import ServerState
from aloxcam.wire.enums import RoutingTable, WireEnums
from aloxcam.wire.messages import PayloadCodec
from aloxcam.wire.protocol import Datatypes

__all__ = ["RuntimeParts", "ServerRuntime"]

log = logging.getLogger(__name__)

_SECOND_Z_JOIN_S = 2.0


def _nothing() -> None:
    return None


@dataclass
class RuntimeParts:
    """Everything ``app.build_runtime`` assembles (tests may build it with fakes)."""

    config: ServerConfig
    host: str
    port: int
    enums: WireEnums
    routing: RoutingTable
    commands: Mapping[str, CommandSpec]
    payloads: PayloadCodec
    state: ServerState
    events: EventSink
    codec: ImageCodec
    tick: TickFn
    camera_handlers: HandlerTable
    z_handlers: HandlerTable
    make_camera_context: Callable[[WorkerContext, Callable[[Request], None]], WorkerContext]   # (base, forward_to_z)
    make_z_context: Callable[[WorkerContext], WorkerContext]
    camera_background: BackgroundStepper | None = None
    abort_waits: Callable[[], None] = _nothing      # app: ZController.cancel_waits (design §3.5.4)
    close_hardware: Callable[[], None] = _nothing


class ServerRuntime:
    """The running server: TCP thread, camera and Z workers, shutdown chain (01 §10.2)."""

    def __init__(self, parts: RuntimeParts) -> None:
        self.parts = parts
        cfg = parts.config
        self._link = ClientLink()
        self.datatypes = Datatypes.from_enums(parts.enums)
        self.replier = Replier(self._link, self.datatypes, parts.codec, cfg.compat, parts.tick,
                               encoding=cfg.wire.encoding)
        self.z_worker = Worker("z", parts.make_z_context(self._base_context()), parts.z_handlers)
        camera_ctx = parts.make_camera_context(self._base_context(), self.z_worker.put)
        self.camera_worker = Worker("camera", camera_ctx, parts.camera_handlers, background=parts.camera_background,
                                    poll_interval_s=cfg.server.stream_poll_ms / 1000.0,
                                    after_each=parts.state.record_camera_iteration)
        self.router = Router(enums=parts.enums, routing=parts.routing, commands=parts.commands,
                             payloads=parts.payloads, state=parts.state, replies=self.replier, compat=cfg.compat,
                             tick=parts.tick, camera_put=self.camera_worker.put, z_put=self.z_worker.put)
        self.tcp = TcpServer(host=parts.host, port=parts.port, link=self._link, router=self.router,
                             state=parts.state, datatypes=self.datatypes, compat=cfg.compat, options=cfg.server,
                             tick=parts.tick)
        self._lock = threading.Lock()
        self._started = False
        self._shutdown_requested = False
        self._stopped = threading.Event()
        self._shutdown_thread: threading.Thread | None = None

    def _base_context(self) -> WorkerContext:
        p = self.parts
        return WorkerContext(state=p.state, replies=self.replier, events=p.events, compat=p.config.compat, tick=p.tick)

    # ------------------------------------------------------------------ public API
    def start(self) -> tuple[str, int]:
        """Bind (``OSError`` propagates: nothing is started then), start the Z worker, the camera worker, the TCP
        thread and the shutdown thread; returns the bound address."""
        with self._lock:
            if self._started:
                raise RuntimeError("ServerRuntime already started")
            address = self.tcp.bind()
            self._started = True
        self.z_worker.start()
        self.camera_worker.start()
        self.tcp.start()
        self._shutdown_thread = threading.Thread(target=self._shutdown_chain, name="aloxcam-shutdown", daemon=True)
        self._shutdown_thread.start()
        return address

    def request_shutdown(self, reason: str = "signal") -> None:
        """Thread-safe and idempotent: post an internal ``Exit`` to the camera worker. It runs after the camera
        commands already queued; the shutdown chain then runs exactly as for a client ``Exit``."""
        with self._lock:
            if self._shutdown_requested:
                return
            self._shutdown_requested = True
        log.info("shutdown requested (%s)", reason)
        self.camera_worker.put(Request.make_internal("Exit", self.parts.commands.get("Exit")))

    def wait(self, timeout: float | None = None) -> bool:
        """True once fully stopped (after ``close_hardware``)."""
        return self._stopped.wait(timeout)

    @property
    def started(self) -> bool:
        """True once :meth:`start` has bound the listener."""
        return self._started

    @property
    def shutdown_started(self) -> bool:
        """``request_shutdown()`` was called or the camera worker stopped (client ``Exit``)."""
        return self._shutdown_requested or self.camera_worker.stopped.is_set()

    @property
    def address(self) -> tuple[str, int]:
        """The bound ``(host, port)``; :class:`RuntimeError` before :meth:`start`."""
        return self.tcp.address

    @property
    def state(self) -> ServerState:
        return self.parts.state

    @property
    def link(self) -> ClientLink:
        return self._link

    # ------------------------------------------------------------------ shutdown chain
    def _shutdown_chain(self) -> None:
        """01 §7.1 with the A10 fix (design §3.4.6)."""
        self.camera_worker.stopped.wait()
        timeout = self.parts.config.server.shutdown_timeout_s
        log.info("camera worker stopped: shutting down")
        try:
            self._link.close()                        # replies of Z commands still queued are dropped (01 §6.3)
            self._guard("stop the TCP server", self.tcp.stop)
            if not self.z_worker.join(timeout):
                log.error("Z worker still busy %.1f s after Exit: cancelling its waits", timeout)
                self._guard("cancel the Z waits", self.parts.abort_waits)
                self.z_worker.stop_now()              # ends it even if it never receives the forwarded Exit
                if not self.z_worker.join(_SECOND_Z_JOIN_S):
                    log.error("Z worker did not stop; abandoned (daemon thread)")
            background = self.parts.camera_background
            if background is not None:
                self._guard("abort the background job", background.abort)
            self._guard("close the hardware", self.parts.close_hardware)
        finally:
            self._stopped.set()
            log.info("server stopped")

    @staticmethod
    def _guard(what: str, fn: Callable[[], object]) -> None:
        try:
            fn()
        except Exception:
            log.exception("shutdown: cannot %s", what)
