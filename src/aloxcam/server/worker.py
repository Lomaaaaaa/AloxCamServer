"""``Worker``: a consumer thread (camera or Z) with an unbounded queue, the re-enqueue path and ``Exit``.

Spec: 01 §5.3–§5.4 (consumer loops), §10.2–§10.3 (threads, unbounded queues), 01 A1–A3 / 02 D11 (no blocking
self-enqueue), 01 A8 (``Default`` case), 03 §3.4 (commands queued after ``Exit`` are discarded), 05 §6.2–§6.3 (stream
re-enqueue, 1 ms wait), 06 §9.5 (background stepping), 02 §10 rule 3 (at most one reply); docs/phase1-design.md §2,
§3.4.4, §7.

Loop: ``get`` blocks unless ``background.has_work()`` (then ``get_nowait`` and one ``background.step()`` per pass).
A request that was re-queued is delayed by ``poll_interval_s`` when nothing else is queued (LabVIEW's 1 ms ``Wait``).
Dispatch: ``decode_error`` → :func:`~aloxcam.dispatch.reject_undecodable`; no handler → WARNING (and a forced 9506
reply with ``reply_unknown_commands``); handler exception → ERROR with traceback, finished as a 9500 error only if
the request was not replied yet; ``Outcome.requeue`` → tail of the queue; ``Outcome.stop`` → ``after_each()`` once,
``stopped`` set, the rest of the queue discarded.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from collections.abc import Callable

from aloxcam.dispatch import (
    CONTINUE,
    STOP,
    BackgroundStepper,
    HandlerTable,
    Outcome,
    ReplyKind,
    Request,
    WorkerContext,
    reject_undecodable,
)
from aloxcam.errors import cluster_of
from aloxcam.server.router import unknown_command_error

__all__ = ["Worker"]

log = logging.getLogger(__name__)


class _StopNow:
    """Private sentinel of :meth:`Worker.stop_now`."""

    def __repr__(self) -> str:
        return "<stop now>"


_STOP_NOW = _StopNow()


class Worker:
    """One consumer loop (01 §5.3 / §5.4) on its own daemon thread ``aloxcam-<name>``."""

    def __init__(self, name: str, ctx: WorkerContext, handlers: HandlerTable, *,
                 background: BackgroundStepper | None = None, poll_interval_s: float = 0.001,
                 after_each: Callable[[], None] | None = None) -> None:
        self.name = name
        self.ctx = ctx
        self.handlers = handlers
        self.background = background
        self.poll_interval_s = max(0.0, poll_interval_s)
        self.after_each = after_each
        self._queue: queue.Queue[Request | _StopNow] = queue.Queue()     # unbounded: put never blocks
        self._stopped = threading.Event()
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------ public API
    def put(self, req: Request) -> None:
        """Queue a request (router, ``forward_to_z``, ``request_shutdown``); never blocks. After ``stopped`` the
        request is discarded (03 §3.4)."""
        if self._stopped.is_set():
            log.debug("%s worker stopped: %s discarded", self.name, req.command)
            return
        self._queue.put_nowait(req)

    def requeue(self, req: Request) -> None:
        """Internal re-enqueue path (05 §6.2): the same request at the tail of the queue, ``requeue_count += 1``."""
        req.requeue_count += 1
        self._queue.put_nowait(req)

    def start(self) -> None:
        """Start the thread ``aloxcam-<name>`` (daemon)."""
        if self._thread is not None:
            raise RuntimeError(f"{self.name} worker already started")
        self._thread = threading.Thread(target=self._run, name=f"aloxcam-{self.name}", daemon=True)
        self._thread.start()

    def join(self, timeout: float | None = None) -> bool:
        """Wait for the thread to end; True when it is not running (also when it was never started)."""
        thread = self._thread
        if thread is None:
            return True
        thread.join(timeout)
        return not thread.is_alive()

    def stop_now(self) -> None:
        """End the loop without ``Exit`` (shutdown fallback): a private sentinel queued at the tail."""
        self._queue.put_nowait(_STOP_NOW)

    @property
    def stopped(self) -> threading.Event:
        """Set when the loop has ended (``Exit``, :meth:`stop_now`)."""
        return self._stopped

    def qsize(self) -> int:
        """Approximate number of queued items (``CAM Queue Size`` / ``Z Queue Size``, 01 §8.3)."""
        return self._queue.qsize()

    # ------------------------------------------------------------------ loop
    def _run(self) -> None:
        log.debug("%s worker started", self.name)
        try:
            while True:
                busy = self._has_background_work()
                item: Request | _StopNow | None
                if busy:
                    try:
                        item = self._queue.get_nowait()
                    except queue.Empty:
                        item = None
                else:
                    item = self._queue.get()
                if item is _STOP_NOW:
                    log.debug("%s worker: stop requested without Exit", self.name)
                    break
                if isinstance(item, Request):
                    if item.requeue_count and self.poll_interval_s and self._queue.empty():
                        time.sleep(self.poll_interval_s)     # LabVIEW's 1 ms Wait between stream polls (05 §6.3)
                    outcome = self._dispatch(item)
                    if outcome.requeue and not outcome.stop:
                        self.requeue(item)
                    self._after_each()
                    if outcome.stop:
                        break
                if self._has_background_work():
                    self._step_background()
        except BaseException:                                  # a bug in the loop itself: never silent
            log.critical("%s worker loop crashed", self.name, exc_info=True)
            raise
        finally:
            self._stopped.set()
            self._discard_pending()
            log.debug("%s worker stopped", self.name)

    def _dispatch(self, req: Request) -> Outcome:
        """Run one request; never raises."""
        try:
            if req.decode_error is not None:
                reject_undecodable(self.ctx, req)
                return CONTINUE
            handler = self.handlers.get(req.command)
            if handler is None:
                self._no_handler(req)
                return CONTINUE
            try:
                outcome = handler(self.ctx, req)
            except Exception as exc:
                return self._handler_failed(req, exc)
            return outcome if outcome is not None else CONTINUE
        except Exception:                                      # the error path itself failed
            log.exception("%s worker: error while finishing %s", self.name, req.command)
            return STOP if req.command == "Exit" else CONTINUE

    def _handler_failed(self, req: Request, exc: Exception) -> Outcome:
        """Handler exception: ERROR log with traceback; a 9500 error finished like a decode error, only if nothing
        was replied yet (02 §10 rule 3). ``Exit`` still stops the loop."""
        log.error("%s worker: command %s failed", self.name, req.command, exc_info=exc)
        if req.replied:
            log.error("%s worker: %s had already replied; the error is logged only", self.name, req.command)
        else:
            req.decode_error = cluster_of(exc)
            reject_undecodable(self.ctx, req)
        return STOP if req.command == "Exit" else CONTINUE

    def _no_handler(self, req: Request) -> None:
        """Consumer ``Default`` case (01 A8, 03 §3.4): nothing, unless ``reply_unknown_commands`` (forced 9506)."""
        log.warning("%s worker: no handler for %s (code %d): ignored", self.name, req.command, req.code)
        if self.ctx.compat.reply_unknown_commands and not req.internal:
            spec = req.spec
            kind = spec.reply if spec is not None and spec.reply is not ReplyKind.NONE else ReplyKind.ACK
            self.ctx.replies.error_reply(req, kind, unknown_command_error(req.command), force=True)

    def _after_each(self) -> None:
        if self.after_each is None:
            return
        try:
            self.after_each()
        except Exception:
            log.exception("%s worker: after-iteration hook failed", self.name)

    def _has_background_work(self) -> bool:
        if self.background is None:
            return False
        try:
            return bool(self.background.has_work())
        except Exception:
            log.exception("%s worker: background has_work() failed", self.name)
            return False

    def _step_background(self) -> None:
        try:
            self.background.step()  # type: ignore[union-attr]
        except Exception:
            log.exception("%s worker: background step failed", self.name)

    def _discard_pending(self) -> None:
        """Commands queued after ``Exit`` are discarded with the queue (03 §3.4): never executed, never replied."""
        discarded = 0
        while True:
            try:
                item = self._queue.get_nowait()
            except queue.Empty:
                break
            if isinstance(item, Request):
                discarded += 1
        if discarded:
            log.info("%s worker: %d queued request(s) discarded at stop", self.name, discarded)
