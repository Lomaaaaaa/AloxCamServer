"""aloxcam.server.worker: FIFO, re-enqueue, Exit, error paths, background stepping (design §3.4.4)."""

from __future__ import annotations

import logging
import threading
from collections.abc import Iterator
from typing import Any

import pytest

from aloxcam.dispatch import REQUEUE, STOP, Outcome, ReplyKind, Request, WorkerContext
from aloxcam.errors import NO_ERROR, ErrorCluster
from aloxcam.server.worker import Worker
from aloxcam.settings import CompatSwitches
from aloxcam.testing import RecordingEvents, RecordingReplies, make_request, make_worker_context

TIMEOUT = 5.0


class Journal:
    """Handlers that record the commands they ran; ``Exit`` stops."""

    def __init__(self) -> None:
        self.ran: list[str] = []
        self.lock = threading.Lock()

    def note(self, name: str) -> None:
        with self.lock:
            self.ran.append(name)

    def plain(self, ctx: WorkerContext, req: Request) -> Outcome | None:
        self.note(req.command)
        return None

    def exit(self, ctx: WorkerContext, req: Request) -> Outcome:
        self.note(req.command)
        return STOP


def _replies(ctx: WorkerContext) -> RecordingReplies:
    assert isinstance(ctx.replies, RecordingReplies)
    return ctx.replies


def _events(ctx: WorkerContext) -> RecordingEvents:
    assert isinstance(ctx.events, RecordingEvents)
    return ctx.events


@pytest.fixture
def journal() -> Journal:
    return Journal()


@pytest.fixture
def ctx() -> WorkerContext:
    return make_worker_context()


def run_until_stopped(worker: Worker) -> None:
    worker.start()
    assert worker.join(TIMEOUT), "worker did not stop"
    assert worker.stopped.is_set()


def test_fifo_order_and_exit(ctx: WorkerContext, journal: Journal) -> None:
    handlers = {"Cam Stop": journal.plain, "Cam Start": journal.plain, "Exit": journal.exit}
    iterations: list[int] = []
    worker = Worker("camera", ctx, handlers, after_each=lambda: iterations.append(1))
    for name in ("Cam Start", "Cam Stop", "Cam Start", "Exit"):
        worker.put(make_request(name))
    run_until_stopped(worker)
    assert journal.ran == ["Cam Start", "Cam Stop", "Cam Start", "Exit"]
    assert len(iterations) == 4                       # after_each also runs once for Exit


def test_exit_discards_later_requests_and_put_after_stop(ctx: WorkerContext, journal: Journal,
                                                         caplog: pytest.LogCaptureFixture) -> None:
    handlers = {"Cam Stop": journal.plain, "Exit": journal.exit}
    worker = Worker("camera", ctx, handlers)
    for name in ("Cam Stop", "Exit", "Cam Stop", "Cam Stop"):
        worker.put(make_request(name))
    run_until_stopped(worker)
    assert journal.ran == ["Cam Stop", "Exit"]
    assert worker.qsize() == 0
    worker.put(make_request("Cam Stop"))              # accepted and discarded (03 §3.4)
    assert worker.qsize() == 0 and _replies(ctx).replies == []


def test_requeue_goes_to_the_tail(ctx: WorkerContext, journal: Journal) -> None:
    done = threading.Event()

    def stream(c: WorkerContext, req: Request) -> Outcome | None:
        journal.note(f"S{req.requeue_count}")
        if req.requeue_count < 2:
            return REQUEUE
        c.replies.boolean(req, True, NO_ERROR)
        done.set()
        return None

    handlers = {"Cam Get Acquisition Status": stream, "Cam Stop": journal.plain, "Cam Start": journal.plain,
                "Exit": journal.exit}
    worker = Worker("camera", ctx, handlers, poll_interval_s=0.001)
    stream_req = make_request("Cam Get Acquisition Status")
    for req in (stream_req, make_request("Cam Stop"), make_request("Cam Start")):
        worker.put(req)
    worker.start()
    assert done.wait(TIMEOUT)
    worker.put(make_request("Exit"))
    assert worker.join(TIMEOUT)
    assert journal.ran == ["S0", "Cam Stop", "Cam Start", "S1", "S2", "Exit"]
    assert stream_req.requeue_count == 2
    assert [r.kind for r in _replies(ctx).replies] == [ReplyKind.BOOL]   # exactly one reply (02 §10 rule 3)


def test_requeue_method_increments_the_count(ctx: WorkerContext) -> None:
    worker = Worker("camera", ctx, {})
    req = make_request("Cam Stop")
    worker.requeue(req)
    assert req.requeue_count == 1 and worker.qsize() == 1


def test_handler_exception_finished_as_9500_and_loop_continues(ctx: WorkerContext, journal: Journal,
                                                               caplog: pytest.LogCaptureFixture) -> None:
    def broken(c: WorkerContext, req: Request) -> None:
        raise ValueError("boom")

    worker = Worker("camera", ctx, {"Cam Stop": broken, "Cam Start": journal.plain, "Exit": journal.exit})
    for name in ("Cam Stop", "Cam Start", "Exit"):
        worker.put(make_request(name))
    with caplog.at_level(logging.ERROR, logger="aloxcam.server.worker"):
        run_until_stopped(worker)
    assert journal.ran == ["Cam Start", "Exit"]
    (reply,) = _replies(ctx).replies
    assert reply.kind is ReplyKind.ACK and not reply.sent               # error ACK: send_error_replies FALSE
    assert reply.err == ErrorCluster(True, 9500, "ValueError: boomCam stop")
    (event,) = _events(ctx).events
    assert (event.err.code, event.server_type, event.extra) == (9500, "Camera", "Stop")
    assert any(r.exc_info for r in caplog.records if r.levelno == logging.ERROR)   # traceback logged


def test_handler_exception_on_a_z_command(journal: Journal) -> None:
    ctx = make_worker_context(send_error_replies=True)

    def broken(c: WorkerContext, req: Request) -> None:
        raise RuntimeError("z broke")

    worker = Worker("z", ctx, {"Z Get Position": broken, "Exit": journal.exit})
    worker.put(make_request("Z Get Position"))
    worker.put(make_request("Exit"))
    run_until_stopped(worker)
    (reply,) = _replies(ctx).replies
    assert reply.kind is ReplyKind.DBL and reply.value == 0.0 and reply.sent
    assert (reply.err.code, reply.err.source) == (9500, "RuntimeError: z broke")
    assert _events(ctx).events[0].server_type == "Z Stage"


def test_handler_that_replied_then_raised_gets_no_second_reply(ctx: WorkerContext, journal: Journal) -> None:
    def reply_then_fail(c: WorkerContext, req: Request) -> None:
        c.replies.dbl(req, 1.5, NO_ERROR)
        raise RuntimeError("after the reply")

    worker = Worker("camera", ctx, {"Cam Get Exposure": reply_then_fail, "Exit": journal.exit})
    worker.put(make_request("Cam Get Exposure"))
    worker.put(make_request("Exit"))
    run_until_stopped(worker)
    assert [(r.kind, r.value) for r in _replies(ctx).replies] == [(ReplyKind.DBL, 1.5)]
    assert _events(ctx).events == []                  # logged in the diagnostic log only


def test_exit_handler_exception_still_stops(ctx: WorkerContext) -> None:
    def broken_exit(c: WorkerContext, req: Request) -> None:
        raise RuntimeError("cannot forward")

    worker = Worker("camera", ctx, {"Exit": broken_exit})
    worker.put(make_request("Exit"))
    run_until_stopped(worker)


def test_no_handler_logs_a_warning_without_reply(ctx: WorkerContext, journal: Journal,
                                                 caplog: pytest.LogCaptureFixture) -> None:
    worker = Worker("camera", ctx, {"Exit": journal.exit})
    worker.put(make_request("Cam Stop"))
    worker.put(make_request("Exit"))
    with caplog.at_level(logging.WARNING, logger="aloxcam.server.worker"):
        run_until_stopped(worker)
    assert _replies(ctx).replies == [] and _events(ctx).events == []
    assert any("no handler for Cam Stop" in r.getMessage() for r in caplog.records)


def test_no_handler_with_reply_unknown_commands_sends_a_forced_9506(journal: Journal) -> None:
    ctx = make_worker_context(compat=CompatSwitches(reply_unknown_commands=True))
    worker = Worker("camera", ctx, {"Exit": journal.exit})
    for req in (make_request("Cam Stop"), make_request("Cam Get Exposure"),
                make_request("#99", spec=None, code=99), Request.make_internal("Cam Stop", None),
                make_request("Exit")):
        worker.put(req)
    run_until_stopped(worker)
    kinds = [(r.kind, r.err.code, r.sent) for r in _replies(ctx).replies]
    assert kinds == [(ReplyKind.ACK, 9506, True), (ReplyKind.DBL, 9506, True), (ReplyKind.ACK, 9506, True)]
    assert _replies(ctx).replies[2].err.source == "Unknown command #99"


def test_decode_error_goes_to_reject_undecodable(ctx: WorkerContext, journal: Journal) -> None:
    called: list[Request] = []
    bad = make_request("Cam Get Exposure", decode_error=ErrorCluster(True, 91, "Variant To Data in x"))
    worker = Worker("camera", ctx, {"Cam Get Exposure": lambda c, r: called.append(r), "Exit": journal.exit})
    worker.put(bad)
    worker.put(make_request("Exit"))
    run_until_stopped(worker)
    assert called == []                               # never executed (04 T1)
    (reply,) = _replies(ctx).replies
    assert reply.kind is ReplyKind.DBL and reply.value == 0.0 and reply.err.code == 91
    assert _events(ctx).events[0].err.code == 91


class Stepper:
    """BackgroundStepper double: ``work`` steps to do; optional failing steps."""

    def __init__(self, work: int, fail_first: int = 0) -> None:
        self.work = work
        self.fail_first = fail_first
        self.steps = 0
        self.aborted = False
        self.done = threading.Event()

    def has_work(self) -> bool:
        return self.steps < self.work

    def step(self) -> None:
        self.steps += 1
        if self.steps >= self.work:
            self.done.set()
        if self.steps <= self.fail_first:
            raise RuntimeError("step failed")

    def abort(self) -> None:
        self.aborted = True


def test_background_stepper_runs_when_idle(ctx: WorkerContext, journal: Journal) -> None:
    stepper = Stepper(work=5)
    worker = Worker("camera", ctx, {"Exit": journal.exit}, background=stepper)
    worker.start()
    assert stepper.done.wait(TIMEOUT)                 # stepped with an empty queue (06 §9.5)
    worker.put(make_request("Exit"))
    assert worker.join(TIMEOUT)
    assert stepper.steps == 5 and not stepper.aborted   # abort is the shutdown chain's job


def test_background_step_and_after_each_exceptions_are_logged(ctx: WorkerContext, journal: Journal,
                                                              caplog: pytest.LogCaptureFixture) -> None:
    stepper = Stepper(work=3, fail_first=2)

    def failing_hook() -> None:
        raise RuntimeError("hook failed")

    worker = Worker("camera", ctx, {"Cam Stop": journal.plain, "Exit": journal.exit}, background=stepper,
                    after_each=failing_hook)
    with caplog.at_level(logging.ERROR, logger="aloxcam.server.worker"):
        worker.start()
        assert stepper.done.wait(TIMEOUT)
        worker.put(make_request("Cam Stop"))
        worker.put(make_request("Exit"))
        assert worker.join(TIMEOUT)
    assert journal.ran == ["Cam Stop", "Exit"]
    messages = [r.getMessage() for r in caplog.records]
    assert sum("background step failed" in m for m in messages) == 2
    assert sum("after-iteration hook failed" in m for m in messages) == 2


def test_stop_now_ends_the_loop_without_exit(ctx: WorkerContext, journal: Journal) -> None:
    worker = Worker("z", ctx, {"Z Stop": journal.plain})
    worker.put(make_request("Z Stop"))
    worker.stop_now()
    run_until_stopped(worker)
    assert journal.ran == ["Z Stop"]


def test_thread_name_and_double_start(ctx: WorkerContext) -> None:
    names: list[str] = []

    def exit_handler(c: WorkerContext, req: Request) -> Outcome:
        names.append(threading.current_thread().name)
        return STOP

    worker = Worker("z", ctx, {"Exit": exit_handler})
    assert worker.join(0) is True                     # never started: nothing to wait for
    worker.put(make_request("Exit"))
    run_until_stopped(worker)
    assert names == ["aloxcam-z"]
    with pytest.raises(RuntimeError):
        worker.start()


@pytest.fixture(autouse=True)
def _no_leaked_worker_threads() -> Iterator[None]:
    before = {t.name for t in threading.enumerate()}
    yield
    leaked: list[Any] = [t for t in threading.enumerate()
                         if t.name.startswith("aloxcam-") and t.name not in before and t.is_alive()]
    assert not leaked, f"worker threads still running: {leaked}"
