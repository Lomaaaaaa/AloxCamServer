"""``tools/z_stage_check.py``: the information mode (exactly the documented read-only frames) and the guided motion
tests with their safety rules, against the emulated C-663.

Spec: docs/phase2-design.md §4.1 and §5 ("tools/z_stage_check.py: the info mode, and --motion --fake --yes, against
the fake"); docs/phase2-notes.md ("Lab tool z_stage_check").

Most tests run the tool's ``main()`` in process with virtual time: the tool's ``sleep`` / ``monotonic`` and the fake
controller's clock are one :class:`FakeTime`, and :class:`zstage.fake_probe.SyncedLink` makes every write return once
the fake has processed it. A 1 mm move at 1 mm/s therefore takes no real time and every result is deterministic. Two
subprocess tests run ``python tools/z_stage_check.py`` as the owner will, without PYTHONPATH (the tool finds
``<repo>/src`` by itself).
"""

from __future__ import annotations

import io
import json
import os
import re
import subprocess
import sys
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import pytest

import z_stage_check
from aloxcam.errors import StageError
from aloxcam.settings import SimulationOptions
from aloxcam.zstage import gcs_serial
from aloxcam.zstage.fake_c663 import FAKE_IDENTITY, FakeC663Server
from aloxcam.zstage.gcs_serial import GcsSerialLink
from z_stage_check import (
    INFO_FRAMES,
    INFO_STEPS,
    TEST_SPECS,
    Environment,
    Refusal,
    advice,
    command_text,
    compute_window,
    frame_text,
    main,
    report_paths,
    resolve_target,
)
from zstage.fake_probe import SyncedLink

REPO = Path(__file__).resolve().parents[2]
TOOL = REPO / "tools" / "z_stage_check.py"
START = 50.0                                     # the fake stage starts here (referenced by FRF at mid-travel)
#: Frames that move the stage, change a setting or stop / start a macro: never sent by the information mode.
CHANGING = ("MOV ", "MVR ", "VEL ", "SVO ", "JON ", "JAX ", "STP", "FNL", "FRF ", "FPL", "MAC START", "#24", "RON ")


def changing(frames: list[str]) -> list[str]:
    return [f for f in frames if f.startswith(CHANGING)]


def targets(frames: list[str], command: str) -> list[float]:
    return [float(f.split()[2]) for f in frames if f.startswith(command + " ")]


class FakeTime:
    """Virtual time shared by the tool and the fake controller; ``on_sleep`` runs after every tool sleep (to inject
    a Ctrl+C, a HOME start, a lost answer, a pulled cable while the stage moves)."""

    def __init__(self, start: float = 1000.0) -> None:
        self.now = start
        self.on_sleep: Callable[[], None] | None = None

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(0.0, seconds)
        if self.on_sleep is not None:
            self.on_sleep()


def answers_from(answers: list[Any]) -> Callable[[], str]:
    """``read_line`` for the tool: the next answer; an exception class or instance in the list is raised (Ctrl+C);
    the end of the list is the end of input."""
    it: Iterator[Any] = iter(answers)

    def read_line() -> str:
        try:
            answer = next(it)
        except StopIteration:
            raise EOFError from None
        if isinstance(answer, BaseException) or (isinstance(answer, type) and issubclass(answer, BaseException)):
            raise answer
        return str(answer)

    return read_line


@dataclass
class Rig:
    time: FakeTime
    server: FakeC663Server
    out_dir: Path
    answers: list[Any] = field(default_factory=list)
    out: io.StringIO = field(default_factory=io.StringIO)

    @property
    def model(self) -> Any:
        return self.server.controller.model

    @property
    def frames(self) -> list[str]:
        """What the fake controller received."""
        return self.server.controller.frames

    @property
    def output(self) -> str:
        return self.out.getvalue()

    def env(self, *, synced: bool = True) -> Environment:
        fake = self.server.controller

        def synced_link(port: str, baudrate: int, timeout_s: float) -> GcsSerialLink:
            return SyncedLink(self.server.url, baudrate, timeout_s, fake=fake, name=port)

        return Environment(read_line=answers_from(self.answers), output=self.out, sleep=self.time.sleep,
                           monotonic=self.time.monotonic, link_factory=synced_link if synced else None,
                           fake_server=self.server)

    def run(self, *argv: str, synced: bool = True) -> int:
        return main([*argv, "--out", str(self.out_dir)], env=self.env(synced=synced))

    def report_path(self) -> Path:
        files = sorted(self.out_dir.glob("z_stage_report_*.json"))
        assert len(files) == 1, files
        return files[0]

    def report(self) -> dict[str, Any]:
        return json.loads(self.report_path().read_text(encoding="utf-8"))

    def motion_tests(self) -> dict[str, dict[str, Any]]:
        return {t["key"]: t for t in self.report()["motion"]["tests"]}


@pytest.fixture
def rig(tmp_path: Path) -> Iterator[Rig]:
    clock = FakeTime()
    options = SimulationOptions(stage_min_mm=0.0, stage_max_mm=100.0, stage_reference_s=0.2,
                                stage_initially_referenced=True, stage_identity=FAKE_IDENTITY)
    server = FakeC663Server(options, clock=clock.monotonic).start()
    model = server.controller.model
    model.frf("1")                               # referenced at mid-travel, as start_fake_server() does
    clock.now += 0.2
    assert model.referenced and model.position == pytest.approx(START)
    model.qerr()
    server.controller.clear_frames()
    out_dir = tmp_path / "out"
    out_dir.mkdir()
    yield Rig(clock, server, out_dir)
    server.close()


# ---------------------------------------------------------------------------------------------- information mode
def test_info_sends_exactly_the_documented_read_only_frames(rig: Rig) -> None:
    model = rig.model
    before = (model.position, model.velocity, model.referenced, model.running_macro, model.joystick_enabled)
    assert rig.run("--fake") == 0
    assert rig.frames == list(INFO_FRAMES)
    assert changing(rig.frames) == []
    assert (model.position, model.velocity, model.referenced, model.running_macro, model.joystick_enabled) == before

    report = rig.report()
    queries = report["info"]["queries"]
    assert [q["frame"] for q in queries] == [step.frame for step in INFO_STEPS]
    assert all(q["ok"] for q in queries)
    values = {q["key"]: q["value"] for q in queries}
    assert values["sai"] == ["1"] and values["pos"] == START and values["vel"] == 10.0
    assert values["tmn"] == 0.0 and values["tmx"] == 100.0 and values["rmc"] == "DH" and values["ready"] is True
    assert report["connection"]["purge"] == {"sent": ["XXX", "*idn?", "err?"], "err_code": 2,
                                             "note": "normal: the purge's own XXX sets error 2 (Unknown command)"}
    assert report["motion"] is None and report["result"]["exit_code"] == 0
    refs = {f["ref"] for f in report["findings"]}
    assert {"Q9", "Q10", "Q35 test 7", "design §6"} <= refs
    assert any(f["topic"] == "MAC? DH" and f["answer"].startswith("same as") for f in report["findings"])

    frames = report["frames"]                    # every raw frame, time-stamped, as sent and received
    assert [f["cmd"] for f in frames if f["dir"] == "tx"] == list(INFO_FRAMES)
    assert frames[0]["text"] == "XXX\\n" and frames[0]["hex"] == "58 58 58 0a"
    ready = next(i for i, f in enumerate(frames) if f.get("cmd") == "#7")
    assert frames[ready]["hex"] == "07" and frames[ready + 1] == {**frames[ready + 1], "dir": "rx", "text": "\\xb1"}
    assert next(q for q in queries if q["frame"] == "POS? 1")["raw"] == ["1=50.000000\\n"]

    json_path = rig.report_path()
    text = json_path.with_suffix(".txt").read_text(encoding="utf-8")
    assert text.startswith("Z STAGE CHECK REPORT\nSend this report back: " + str(json_path))
    assert "  POS? 1     50.0000 mm" in text and "tx   XXX\\n" in text
    assert f"PLEASE SEND THIS FILE BACK: {json_path}" in rig.output
    assert rig.output.rstrip().endswith("=" * 78)


def test_info_frames_are_listed_in_the_docstring_and_the_help(capsys: pytest.CaptureFixture[str]) -> None:
    doc = z_stage_check.__doc__ or ""
    purge = doc[doc.index("1. Opening the port"):doc.index("2. The queries")]
    queries = doc[doc.index("2. The queries"):doc.index("3. Only when")]
    tokens = re.findall(r"``([^`]+)``", purge)
    assert tokens[tokens.index("XXX"):tokens.index("XXX") + 3] == ["XXX", "*idn?", "err?"]
    assert re.findall(r"``([^`]+)``", queries) == [step.frame for step in INFO_STEPS]
    assert main(["--help"]) == 0
    help_text = capsys.readouterr().out
    assert "XXX, *idn?, err?" in help_text
    assert ", ".join(step.frame for step in INFO_STEPS) in " ".join(help_text.split())   # wrapped in the help
    assert "ERR? reads AND clears" in help_text and "--yes" in help_text


def test_a_query_without_answer_is_followed_by_err(rig: Rig) -> None:
    rig.server.controller.fail_next("TRS?", 2)   # no answer: −7, then ERR? reads the controller's reason
    assert rig.run("--fake", "--timeout-ms", "200") == 0
    i = rig.frames.index("TRS? 1")
    assert rig.frames[i + 1:i + 3] == ["ERR?", "LIM? 1"]
    assert changing(rig.frames) == []
    report = rig.report()
    entry = next(q for q in report["info"]["queries"] if q["frame"] == "TRS? 1")
    assert not entry["ok"] and entry["code"] == -7 and entry["err_after"] == "2  Unknown command"
    assert report["info"]["failed"] == 1 and "1 information queries failed (see the report)" in report["problems"]


# ---------------------------------------------------------------------------------------------- motion tests
def test_motion_with_fake_and_yes_answers_the_q35_questions(rig: Rig) -> None:
    assert rig.run("--fake", "--yes", "--motion") == 0
    report = rig.report()
    motion = report["motion"]
    assert [t["key"] for t in motion["tests"]] == [spec.key for spec in TEST_SPECS]
    tests = rig.motion_tests()
    assert {t["status"] for t in tests.values()} == {"done"}
    assert tests["pos_during_move"]["answer"].startswith("POS? 1 gives the REAL position")
    stp = tests["stp_during_move"]
    assert stp["answer"].startswith("STP 1 STOPPED the stage") and stp["rmc_after"] == ""
    assert (stp["err_after_stp"], stp["err"]) == (10, 0)
    assert tests["mvr_during_move"]["answer"].startswith("MVR adds to the TARGET")
    joystick = tests["mov_with_joystick"]
    assert joystick["moved"] is False and joystick["err_mov"] == 51 and joystick["jon_after_off"] == 0
    assert tests["vel_zero"]["err"] == 17 and tests["vel_zero"]["vel_after"] == 1.0
    assert {f["ref"] for f in report["findings"]} >= {f"Q35 test {n}" for n in range(1, 6)}

    frames = rig.frames
    assert frames[:len(INFO_FRAMES)] == list(INFO_FRAMES)
    assert "#24" not in frames and not motion["emergency_stops"]
    assert not [f for f in frames if f.split()[0] in ("FNL", "FRF", "FPL")]      # never a reference move
    assert all(START - 1.0 <= x <= START + 1.0 for x in targets(frames, "MOV"))  # inside the window
    samples = [s for t in tests.values() for key in ("samples", "samples_after_stp", "samples_after_mvr")
               for s in t.get(key, [])]
    assert samples and all(START - 1.0 <= pos <= START + 1.0 for _, _, pos in samples)
    assert [f for f in frames if f.startswith("VEL ")] == ["VEL 1 1.0000", "VEL 1 0.0000", "VEL 1 1.0000",
                                                         "VEL 1 10.0000"]      # test speed, VEL 0, original
    assert frames.count("MAC START DH") == 1     # DH ran at the start, STP stopped it: restarted once
    assert motion["move_back"]["final_mm"] == pytest.approx(START)
    model = rig.model
    assert model.position == pytest.approx(START) and model.velocity == 10.0
    assert model.running_macro == "DH" and not model.joystick_enabled and not model.is_moving()
    assert "THE STAGE WILL MOVE" in rig.output and "MOVES THE STAGE TO Z = 0 mm" in rig.output


@pytest.mark.parametrize("window, answer", [
    ("1", "STP 1 did NOT stop the stage: it went on to its target"),        # the move ends within the 1 s grace
    ("3", "STP 1 did NOT stop the stage within 1 s; #24 was sent"),         # the grace ends first: #24 stops it
])
def test_a_refused_stp_is_reported(rig: Rig, window: str, answer: str) -> None:
    rig.server.controller.fail_next("STP", 1)    # the controller refuses STP 1 (error 1) and keeps moving
    assert rig.run("--fake", "--yes", "--motion", "--window", window) == 0
    stp = rig.motion_tests()["stp_during_move"]
    assert stp["status"] == "done" and stp["answer"].startswith(answer) and stp["err_after_stp"] == 1
    assert not rig.report()["motion"]["emergency_stops"]               # a finding, not an emergency
    assert ("#24" in rig.frames) == (window == "3")


@pytest.mark.parametrize("answer", ["y", "", "YES please", "ok"])
def test_motion_needs_a_typed_yes(rig: Rig, answer: str) -> None:
    rig.answers = [answer]
    assert rig.run("--fake", "--motion") == 0
    assert rig.frames == list(INFO_FRAMES)       # nothing beyond the read-only information
    report = rig.report()
    assert report["motion"]["declined"] is True
    assert report["operator_answers"][0]["meaning"] != "yes"


def test_skipping_every_test_changes_nothing(rig: Rig) -> None:
    rig.answers = ["yes", "no", "", "skip", "y", "nope"]
    assert rig.run("--fake", "--motion") == 0
    assert changing(rig.frames) == []            # not even the test speed
    assert {t["status"] for t in rig.motion_tests().values()} == {"skipped"}
    assert rig.report()["motion"]["move_back"]["result"].startswith("not needed")


def test_quit_skips_the_remaining_tests(rig: Rig) -> None:
    rig.answers = ["yes", "quit"]
    assert rig.run("--fake", "--motion") == 0
    statuses = [t["status"] for t in rig.report()["motion"]["tests"]]
    assert statuses == ["skipped"] + ["skipped (quit)"] * (len(TEST_SPECS) - 1)
    assert changing(rig.frames) == []


def test_a_window_too_small_refuses_the_moving_tests(rig: Rig) -> None:
    assert rig.run("--fake", "--yes", "--motion", "--window", "0.1") == 0
    tests = rig.motion_tests()
    for key in ("pos_during_move", "stp_during_move", "mvr_during_move"):
        assert tests[key]["status"] == "refused"
        assert tests[key]["reason"].startswith("the window is too small for this test: at 1.0000 mm/s it needs "
                                               "0.6000 mm")
    assert tests["mov_with_joystick"]["status"] == "done" and tests["vel_zero"]["status"] == "done"
    assert targets(rig.frames, "MOV") == [pytest.approx(START + 0.1)]      # the joystick move only (refused: 51)
    assert not targets(rig.frames, "MVR") and "STP 1" not in rig.frames


def test_alox_ini_limits_the_window(rig: Rig, tmp_path: Path) -> None:
    ini = tmp_path / "alox.ini"
    ini.write_bytes(b"[hardware.z]\r\ncomport_I32 = 4\r\nbaudrate_I32 = 3\r\nzmin_DBL = 0,000000\r\n"
                    b"zmax_DBL = 50,500000\r\nspeed_DBL = 5,000000\r\n")
    assert rig.run("--fake", "--yes", "--motion", "--alox-ini", str(ini)) == 0
    report = rig.report()
    window = report["motion"]["window"]
    assert (window["lo"], window["hi"]) == (START - 1.0, 50.5)
    assert [lim["from"] for lim in window["limits"]][1] == "alox.ini [hardware.z] zmin_DBL .. zmax_DBL"
    assert report["alox_ini"]["comport_I32"] == 4 and report["alox_ini"]["zmax_DBL"] == 50.5
    assert all(START - 1.0 <= x <= 50.5 for x in targets(rig.frames, "MOV"))
    assert report["connection"]["port"] == rig.server.url             # --fake: the INI port is not opened


def test_motion_is_refused_when_the_stage_is_outside_the_alox_ini_range(rig: Rig, tmp_path: Path) -> None:
    ini = tmp_path / "alox.ini"
    ini.write_bytes(b"[hardware.z]\r\nzmin_DBL = 0,000000\r\nzmax_DBL = 40,000000\r\n")
    assert rig.run("--fake", "--yes", "--motion", "--alox-ini", str(ini)) == 0
    assert "outside the alox.ini [hardware.z] zmin_DBL .. zmax_DBL" in rig.report()["motion"]["refused"]
    assert rig.frames == list(INFO_FRAMES)


def test_motion_is_refused_on_an_unreferenced_axis(rig: Rig) -> None:
    rig.model.power_cycle()                      # servo off, not referenced
    assert rig.run("--fake", "--yes", "--motion") == 0
    assert rig.report()["motion"]["refused"].startswith("the servo is off (SVO? 1 = 0)")
    assert rig.frames == list(INFO_FRAMES)


def test_real_mode_never_moves_faster_than_the_controller(rig: Rig) -> None:
    rig.model.vel("1", 0.5)                      # the controller's own speed is below --speed (1 mm/s)
    rig.answers = ["yes", "yes", "quit", "no"]   # start, POS? test, quit, leave the stage where it is
    assert rig.run("--port", "COM7", "--motion") == 0              # real-stage rules: no --fake, typed answers
    report = rig.report()
    assert report["connection"]["port"] == "COM7" and report["motion"]["speed_mm_s"] == 0.5
    assert [f for f in rig.frames if f.startswith("VEL ")] == ["VEL 1 0.5000", "VEL 1 0.5000"]   # test, restore
    assert "(lower than --speed: the tool never moves faster than the controller's speed)" in rig.output
    assert report["motion"]["move_back"]["result"].startswith("declined: the stage stays at 51.0000 mm")


def test_real_mode_refuses_a_very_low_controller_speed(rig: Rig) -> None:
    rig.model.vel("1", 0.01)
    assert rig.run("--port", "COM7", "--motion") == 0
    assert rig.report()["motion"]["refused"].startswith("the controller's speed is very low (VEL? 1 = 0.0100 mm/s)")
    assert rig.frames == list(INFO_FRAMES)


def test_setup_twice_references_twice_then_moves_back(rig: Rig) -> None:
    assert rig.run("--fake", "--yes", "--motion", "--setup-twice", "--speed", "10", "--window", "10") == 0
    setup = rig.motion_tests()["setup_twice"]
    assert setup["status"] == "done" and len(setup["runs"]) == 2
    for run in setup["runs"]:
        assert run["reference_frames"] == ["FNL 1"] and run["frf_after"] is True
        assert run["position_after_mm"] == 0.0 and run["ready_busy_answers"] > 0 and run["duration_s"] > 0.2
    assert rig.frames.count("FNL 1") == 2
    report = rig.report()
    assert "Type yes again to confirm the reference moves:" in [a["question"] for a in report["operator_answers"]]
    assert report["motion"]["move_back"]["final_mm"] == pytest.approx(START)
    assert rig.model.position == pytest.approx(START) and rig.model.velocity == 10.0
    assert rig.model.running_macro == "DH"


def test_setup_twice_needs_a_second_yes(rig: Rig) -> None:
    rig.answers = ["yes", "no", "no", "no", "no", "no", "yes", "no"]
    assert rig.run("--fake", "--motion", "--setup-twice") == 0
    assert rig.motion_tests()["setup_twice"]["status"] == "skipped"
    assert changing(rig.frames) == []


# ---------------------------------------------------------------------------------------------- stopping
def _first_move_then_stop(frames: list[str]) -> tuple[int, int]:
    first_move = next(i for i, f in enumerate(frames) if f.startswith("MOV "))
    stop = frames.index("#24")
    assert stop > first_move
    assert not changing(frames[first_move + 1:stop])                # nothing else was commanded before the stop
    return first_move, stop


def _assert_stopped_and_restored(rig: Rig) -> dict[str, Any]:
    model = rig.model
    assert not model.is_moving() and model.velocity == 10.0 and not model.joystick_enabled
    motion = rig.report()["motion"]
    assert motion["emergency_stops"] and motion["emergency_stops"][0]["sent"] is True
    assert "move_back" not in motion                                # no move back after a stop
    assert "!!! Sending #24 (stop all axes) now." in rig.output
    return motion


def test_ctrl_c_during_a_move_sends_stop_all_and_restores(rig: Rig) -> None:
    fired: list[float] = []

    def ctrl_c() -> None:
        if not fired and rig.model.is_moving():
            fired.append(rig.model.position)
            raise KeyboardInterrupt

    rig.time.on_sleep = ctrl_c
    assert rig.run("--fake", "--yes", "--motion") == 1
    _first_move_then_stop(rig.frames)
    motion = _assert_stopped_and_restored(rig)
    assert motion["aborted"] == "Ctrl+C pressed by the operator"
    assert motion["tests"][0]["status"] == "aborted"
    assert [t.get("status") for t in motion["tests"][1:]] == []    # the other tests never started
    assert rig.model.running_macro == "DH"                          # --yes: DH restarted after the question
    assert rig.report()["result"]["exit_code"] == 1


def test_ctrl_c_with_the_joystick_on_sends_stop_all_then_joystick_off(rig: Rig) -> None:
    fired: list[bool] = []

    def ctrl_c() -> None:
        if not fired and rig.model.joystick_enabled:
            fired.append(True)
            raise KeyboardInterrupt

    rig.time.on_sleep = ctrl_c
    rig.answers = ["yes", "no", "no", "no", "yes", "yes"]          # only the joystick test; then restart DH
    assert rig.run("--fake", "--motion") == 1
    frames = rig.frames
    stop = frames.index("#24")
    assert frames[stop - 1].startswith("POS? 1") and frames[stop + 1] == "JON 1 0"   # off at once, then restore
    assert not rig.model.joystick_enabled and rig.model.velocity == 10.0
    motion = rig.report()["motion"]
    assert motion["emergency_stops"][0]["joystick_off"] == "JON 1 0 sent"
    assert motion["restore"][0] == {"what": "joystick", "result": "switched off (JON? 1 = 0, ERR? = 10)"}  # 10: #24
    assert rig.model.running_macro == "DH"


def test_a_move_timeout_sends_stop_all(rig: Rig, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(z_stage_check, "MOVE_TIMEOUT_MIN_S", 0.0)
    monkeypatch.setattr(z_stage_check, "MOVE_TIMEOUT_FACTOR", 0.2)   # a 1 s move "times out" after 0.2 s
    assert rig.run("--fake", "--yes", "--motion") == 1
    _first_move_then_stop(rig.frames)
    motion = _assert_stopped_and_restored(rig)
    assert motion["aborted"].startswith("timeout: the stage still moved 0.2 s after the command")


def test_leaving_the_window_sends_stop_all(rig: Rig) -> None:
    started: list[bool] = []

    def home_button() -> None:                   # digital input 1: DH starts HOME (a move to Z = 0)
        if not started and rig.model.is_moving():
            started.append(rig.model.trigger_home())

    rig.time.on_sleep = home_button
    assert rig.run("--fake", "--yes", "--motion") == 1
    assert started == [True]
    _first_move_then_stop(rig.frames)
    motion = _assert_stopped_and_restored(rig)
    assert motion["aborted"].startswith(f"the stage left the allowed range {START - 1:.4f} .. {START + 1:.4f} mm")
    assert START - 1.0 - 0.15 < rig.model.position < START - 1.0 - 0.05     # stopped within one poll


def test_a_lost_answer_during_a_move_sends_stop_all(rig: Rig) -> None:
    dropped: list[bool] = []

    def drop() -> None:
        if not dropped and rig.model.is_moving():
            dropped.append(True)
            rig.server.controller.drop_next_answer()

    rig.time.on_sleep = drop
    assert rig.run("--fake", "--yes", "--motion", "--timeout-ms", "200") == 1
    _first_move_then_stop(rig.frames)
    motion = _assert_stopped_and_restored(rig)
    assert motion["aborted"].startswith("unexpected error: error -7: Data Receive Timeout")
    assert [a["reopened"] for a in motion["emergency_stops"][0]["attempts"]] == [False]
    assert "The controller did not answer within 200 ms. Check:" in rig.output


def test_a_lost_link_reopens_the_port_to_send_stop_all(rig: Rig) -> None:
    pulled: list[bool] = []

    def pull_cable() -> None:
        if not pulled and rig.model.is_moving():
            pulled.append(True)
            rig.server.disconnect()

    rig.time.on_sleep = pull_cable
    assert rig.run("--fake", "--yes", "--motion", "--timeout-ms", "3000", synced=False) == 1
    frames = rig.frames
    assert rig.server.connections == 2
    reopened = [i for i, f in enumerate(frames) if f == "XXX"][1]
    assert "#24" in frames[reopened:]
    motion = _assert_stopped_and_restored(rig)
    attempts = motion["emergency_stops"][0]["attempts"]
    assert attempts[-1] == {"attempt": len(attempts), "reopened": True, "result": "#24 sent"}
    assert motion["aborted"].startswith("unexpected error: error -")


def test_ctrl_c_at_the_first_question_sends_nothing(rig: Rig) -> None:
    rig.answers = [KeyboardInterrupt]
    assert rig.run("--fake", "--motion") == 1
    assert rig.frames == list(INFO_FRAMES)       # not armed yet: no #24 either
    assert "interrupted by Ctrl+C" in rig.report()["problems"]


@pytest.mark.parametrize("restart", ["yes", "no"])
def test_ctrl_c_at_a_later_question_stops_and_asks_before_restarting_dh(rig: Rig, restart: str) -> None:
    rig.answers = ["yes", "no", "yes", KeyboardInterrupt, restart]   # STP test runs (DH stops), Ctrl+C at MVR
    assert rig.run("--fake", "--motion") == 1
    frames = rig.frames
    assert frames.index("#24") > frames.index("STP 1")
    assert rig.model.velocity == 10.0
    question = rig.report()["operator_answers"][-1]
    assert question["question"].startswith("Type yes to restart DH now") and question["answer"] == restart
    if restart == "yes":
        dh = frames.index("MAC START DH")
        assert dh > frames.index("#24") and frames[dh + 1:dh + 3] == ["RMC?", "ERR?"]
        assert rig.model.running_macro == "DH"
    else:
        assert "MAC START DH" not in frames and rig.model.running_macro == ""
        assert "LabVIEW restarts DH by itself at its next Z Get Status or Z Stop." in rig.output


# ---------------------------------------------------------------------------------------------- command line
@pytest.mark.parametrize("argv, text", [
    (["--port", "COM9", "--yes"], "--yes is only accepted together with --fake"),
    (["--port", "COM9", "--motion", "--yes"], "--yes is only accepted together with --fake"),
    (["--port", "COM9", "--motion", "--speed", "5.5"], "--speed above 5 mm/s is refused on the real stage"),
    (["--port", "COM9", "--motion", "--window", "10.5"], "--window above 10 mm is refused on the real stage"),
    (["--port", "COM9", "--motion", "--speed", "0"], "--speed must be a positive number"),
    (["--fake", "--motion", "--speed", "0.01"], "--speed below 0.05 mm/s is too slow for the tests"),
    (["--port", "COM9", "--setup-twice"], "--setup-twice is a motion test: add --motion"),
    ([], "say which port to use: --alox-ini"),
    (["--fake", "--port", "COM4"], "--fake and --port cannot be used together"),
])
def test_usage_errors_open_nothing(tmp_path: Path, argv: list[str], text: str) -> None:
    def no_port(*_args: object) -> GcsSerialLink:
        raise AssertionError("no port may be opened on a usage error")

    out = io.StringIO()
    assert main([*argv, "--out", str(tmp_path)], env=Environment(output=out, link_factory=no_port)) == 2
    assert f"error: {text}" in out.getvalue()
    assert list(tmp_path.iterdir()) == []


def test_missing_alox_ini_is_a_usage_error(tmp_path: Path) -> None:
    out = io.StringIO()
    missing = tmp_path / "alox.ini"
    assert main(["--alox-ini", str(missing), "--out", str(tmp_path)], env=Environment(output=out)) == 2
    assert f"error: alox.ini not found: {missing}" in out.getvalue()


def test_target_from_alox_ini_and_port(tmp_path: Path) -> None:
    ini = tmp_path / "alox.ini"
    ini.write_bytes(b"[hardware.z]\r\ncomport_I32 = 4\r\nbaudrate_I32 = 3\r\nzmin_DBL = 0,000000\r\n"
                    b"zmax_DBL = 90,000000\r\n")
    parser = z_stage_check.build_parser()
    target = resolve_target(parser.parse_args(["--alox-ini", str(ini)]))
    assert (target.port, target.baudrate, target.zrange, target.com_number) == ("COM4", 115200, (0.0, 90.0), 4)
    target = resolve_target(parser.parse_args(["--alox-ini", str(ini), "--port", "3", "--baud", "9600"]))
    assert (target.port, target.baudrate, target.zrange) == ("COM3", 9600, (0.0, 90.0))
    assert resolve_target(parser.parse_args(["--port", "com5"])).port == "COM5"
    assert resolve_target(parser.parse_args(["--port", "/dev/ttyUSB0"])).com_number == 1


def test_report_goes_next_to_the_tool_by_default(rig: Rig, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    tools = tmp_path / "tools"
    monkeypatch.setattr(z_stage_check, "TOOL_DIR", tools)
    assert main(["--fake"], env=rig.env()) == 0
    names = sorted(p.suffix for p in tools.glob("z_stage_report_*"))
    assert names == [".json", ".txt"]
    started = datetime(2026, 10, 30, 14, 5, 9)
    assert report_paths(None, started) == (tools / "z_stage_report_20261030-140509.json",
                                           tools / "z_stage_report_20261030-140509.txt")
    assert report_paths(tmp_path / "mine.json", started) == (tmp_path / "mine.json", tmp_path / "mine.txt")


# ---------------------------------------------------------------------------------------------- guidance
def test_missing_pyserial_gives_the_exact_pip_command(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    def missing() -> object:
        raise StageError(-6, gcs_serial.PYSERIAL_MISSING)

    monkeypatch.setattr(gcs_serial, "import_pyserial", missing)
    out = io.StringIO()
    assert main(["--port", "COM9", "--out", str(tmp_path)], env=Environment(output=out)) == 1
    assert f'    "{sys.executable}" -m pip install pyserial' in out.getvalue()
    report = json.loads(next(tmp_path.glob("*.json")).read_text(encoding="utf-8"))
    assert report["problems"][0].startswith("Opening the port failed: error -6: pyserial is not installed")


def test_a_busy_port_says_to_close_the_labview_server(tmp_path: Path) -> None:
    def busy(port: str, baudrate: int, _timeout_s: float) -> GcsSerialLink:
        raise StageError(-6, f"Open RS232 {port} @ {baudrate}: could not open port '{port}': "
                             "PermissionError(13, 'Access is denied.', None, 5)")

    out = io.StringIO()
    assert main(["--port", "COM4", "--out", str(tmp_path)], env=Environment(output=out, link_factory=busy)) == 1
    text = out.getvalue()
    assert "COM4 is busy: another program has it open." in text
    assert "Close the LabVIEW camera server (it keeps the COM port open)" in text
    assert "PLEASE SEND THIS FILE BACK" in text


def test_no_answer_points_at_the_port_the_moxa_and_the_power(rig: Rig) -> None:
    rig.server.controller.drop_next_answer()     # the *idn? of the purge: −7
    out = io.StringIO()
    assert main(["--port", rig.server.url, "--timeout-ms", "200", "--out", str(rig.out_dir)],
                env=Environment(output=out)) == 1
    text = out.getvalue()
    assert "The controller did not answer within 200 ms. Check:" in text
    assert "the Moxa serial server" in text and "is switched on" in text and "the port number" in text


@pytest.mark.parametrize("source, start", [
    ("Open RS232 /dev/ttyS0 @ 115200: Could not exclusively lock port /dev/ttyS0: [Errno 11] Resource temporarily "
     "unavailable", "/dev/ttyS0 is busy"),
    ("Open RS232 COM9 @ 115200: could not open port 'COM9': FileNotFoundError(2, 'The system cannot find the file "
     "specified.', None, 2)", "COM9 does not exist on this PC."),
    ("Open RS232 /dev/ttyUSB9 @ 115200: [Errno 2] could not open port /dev/ttyUSB9: [Errno 2] No such file or "
     "directory: '/dev/ttyUSB9'", "/dev/ttyUSB9 does not exist on this PC."),
])
def test_advice_for_open_errors(source: str, start: str) -> None:
    port = start.split()[0]
    assert advice(StageError(-6, source), port)[0].startswith(start)
    assert advice(StageError(-3, "Data Receive Error (RS-232, COM4, baud 115200): x"), "COM4")[0].startswith(
        "The serial link failed")


# ---------------------------------------------------------------------------------------------- helpers
def test_window_is_the_intersection_of_the_ranges() -> None:
    window, limits = compute_window(60.0, 1.0, 0.0, 102.0, (0.0, 60.5))
    assert (window.lo, window.hi) == (59.0, 60.5) and len(limits) == 3
    window, _ = compute_window(0.3, 1.0, 0.0, 102.0, None)
    assert (window.lo, window.hi) == (0.0, 1.3)
    with pytest.raises(Refusal, match="outside the alox.ini"):
        compute_window(95.0, 1.0, 0.0, 102.0, (0.0, 90.0))
    with pytest.raises(Refusal, match="travel range is empty"):
        compute_window(5.0, 1.0, 10.0, 10.0, None)


def test_frame_texts() -> None:
    assert frame_text(b"POS? 1\n") == "POS? 1\\n"
    assert frame_text(b"\xb1") == "\\xb1" and frame_text(b"a\\b\r") == "a\\\\b\\r"
    assert [command_text(bytes([b])) for b in (5, 7, 0x18)] == ["#5", "#7", "#24"]
    assert command_text(b"MOV 1 12.5000\n") == "MOV 1 12.5000"


# ---------------------------------------------------------------------------------------------- as the owner runs it
def _without_pythonpath() -> dict[str, str]:
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    return env


def test_runs_from_a_checkout_without_installing(tmp_path: Path) -> None:
    result = subprocess.run([sys.executable, str(TOOL), "--fake", "--out", str(tmp_path)], capture_output=True,
                            text=True, env=_without_pythonpath(), cwd=tmp_path, timeout=120, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Information done: 20 queries, 0 failed." in result.stdout
    assert "PLEASE SEND THIS FILE BACK" in result.stdout
    assert len(list(tmp_path.glob("z_stage_report_*.json"))) == 1


@pytest.mark.slow
def test_motion_rehearsal_in_real_time(tmp_path: Path) -> None:
    result = subprocess.run([sys.executable, str(TOOL), "--fake", "--motion", "--yes", "--out", str(tmp_path)],
                            capture_output=True, text=True, env=_without_pythonpath(), cwd=tmp_path, timeout=120,
                            check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    report = json.loads(next(tmp_path.glob("z_stage_report_*.json")).read_text(encoding="utf-8"))
    assert {t["status"] for t in report["motion"]["tests"]} == {"done"}
    assert report["motion"]["move_back"]["final_mm"] == pytest.approx(START)
