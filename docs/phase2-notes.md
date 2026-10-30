# Phase 2 implementation notes

This file records the decisions taken while implementing [phase2-design.md](phase2-design.md) (§3 modules, §4.2 replay
script, §5 tests) where the design left a point open or where the implementation differs. The phase 1 notes
([phase1-notes.md](phase1-notes.md)) still apply. `tools/z_stage_check.py` (design §4.1) came afterwards: the API it
needed is summarised below, and its own decisions are in the last section, "Lab tool `tools/z_stage_check.py`".

## Deviations from the design

1. **Two more error codes.** The design names −4, −6 and −7 only. A failed write gives
   `StageError(-2, "Data Send Error (RS-232, <port>, baudrate <b>): …")` and a failed read that is not a timeout (for
   example the `socket://` peer closed the connection, or a USB adapter was pulled) gives
   `StageError(-3, "Data Receive Error (RS-232, <port>, baud <b>): …")`. These are PI's interface codes ("Error while
   sending data", "Error while receiving data"). `Code` gained `PI_SEND_ERROR` / `PI_RECEIVE_ERROR`, `gcs_errors` their
   texts, and `test_code_values_are_final` lists them.
2. **Stale input is discarded before every command** (`GcsSerialLink`, no change on the wire). An answer that arrives
   after a −7 timeout would otherwise be read as the answer of the next query and shift every later answer. The
   discarded bytes are logged at WARNING and recorded in `frames` as `"drop"`. A lone LF right after a `#7` answer is
   dropped at DEBUG only, in case the real C-663 terminates its ready byte with LF (pipython's `IsControllerReady`
   expects one; the LabVIEW driver and 08 §3.4 do not).
3. **The purge's own error 2 is expected.** A real controller keeps only the last error, so the purge's `XXX` replaces
   any pending error with 2 and `err?` reads 2. `PiSerialStage.connect_rs232` returns that code as read (the
   `StageBackend` contract), and `ZController` now logs code 2 at DEBUG instead of the Z22 WARNING (the option "treat 2
   as expected" of the phase 1 notes). `SimStage` never latches 2, so `--simulate` is unchanged.
4. **The "Commanded axis not connected" pre-check applies once `SAI?` has been read on the current link.** Before that
   (no `Z Setup` yet, or a direct backend call) any axis is sent. In the server `SAI?` is always read right after the
   purge, so this only matters to tools and tests.
5. **A failed purge leaves the link open** (for example −7 when the controller is off), as the VISA session stays open
   in LabVIEW. The next `Z Setup`, or the reference check before a move, closes and re-opens it.
6. **`socket://` links set `TCP_NODELAY`** (and so do the fake's connections). Without it, Nagle's algorithm holds a
   short frame until the previous one is acknowledged, which a serial line never does (about 8 ms per frame on
   loopback). pyserial's socket transport keeps its socket in the private attribute `_socket`; the option is skipped
   when it is missing. Real COM ports are not affected.
7. **`GcsSerialLink.discard_input()` returns the discarded bytes** (design: `None`).
8. **Device choice on the command line.** `--simulate` and `--simulate-cameras` are mutually exclusive (usage error 2).
   `--simulate-cameras` selects the real stage even when `[simulation] enabled` or `stage` is TRUE: the command line
   wins. `build_runtime(simulate=False)` is now the default; `simulate_cameras` / `simulate_stage` default to
   `simulate`, and `build_runtime` does not read `[simulation]` itself: `app.simulation_modes(args, simulation)` does,
   in `main()` and `--check-config`.
9. **Missing pyserial exits with code 2** ("no backend"), like real cameras. `main()` checks both before
   `build_runtime` with `app.backend_problem()`; `build_runtime` raises `ConfigError` for both as well.
10. **`--check-config` prints `cameras` and `Z stage` lines** instead of the phase 1 `simulation TRUE/FALSE` line, for
    example `Z stage     real PI C-663: COM4 -> socket://127.0.0.1:5555 ([stage] port_map) at 115200 Bd, opened by Z
    Setup; pyserial 3.5 importable`. A missing pyserial or real cameras are reported there but are not "fatal problems"
    (exit 0), as the phase 1 simulation note was not.
11. **`PHASE = 2`, version 0.2.0** (`aloxcam --version` prints `aloxcam 0.2.0 (phase 2)`).
12. **Replay client:** a DBL or U32 `expect.value` may be a number in a string, so that it can come from a variable
    (`value = "${z_abs}"`). Variables are substituted only in strings, so this was the only way to check positions
    passed with `--var`.

## Additions to the design's APIs

- `gcs_serial`: `import_pyserial()`, `pyserial_version()`, `PYSERIAL_MISSING`, `SERIAL_LOGGER`, the byte constants
  (`CHAR_MOVING`, `CHAR_READY`, `CHAR_STOP_ALL`, `READY_BYTE`, `BUSY_BYTE`), `GcsSerialLink.is_open`, `timeout_s`,
  `encoding`, context-manager use.
- `pi_serial`: `PiSerialStage(link_factory=GcsSerialLink, *, sleep=time.sleep)`; `link`, `connected_axes`,
  `raw_query(cmd)`, `raw_send(cmd)`; the parsers `parse_axis_values`, `axis_value`, `parse_error_code`; the constants
  `OPEN_PAUSE_S`, `PURGE_PAUSE_S`, `PURGE_FRAMES`, `ASSIGN_VALUES_TEXT`, `NO_ANSWER_ON_ERR`, `UNKNOWN_READY_REPLY`.
- `fake_c663`: `FakeC663(options=None, *, clock=None, dh_running=True, encoding="cp1252")` with `model` (the
  `SimStage`), `frames` / `clear_frames()`, `processed_bytes` / `wait_processed(count, timeout)`,
  `take_disconnect_request()` and `reset_input()` (for the server). `FakeC663Server(options=None, *, clock=None,
  host="127.0.0.1", port=0, dh_running=True)` with `address`, `port`, `url`, `connections`, `connected` and
  `disconnect()`. Command-line options beyond the design: `--host`, `--no-dh`, `--min-mm` / `--max-mm` (default
  0–100 mm, so that the production `zmax_DBL = 90` fits), `--reference-s` (default 2 s), `--identity`,
  `--answer-delay-ms`, `--log-level`. It stops on Ctrl+C or SIGTERM (exit code 0).
- `SimStage.latch_error(code)`: the fake latches 1 and 2 (and injected codes) through it.
- `controller.no_axis_text(command)` (public, shared with `PiSerialStage`) and `controller.PURGE_OWN_ERROR`.
- `app.simulation_modes`, `app.backend_problem`, `app.NO_CAMERA_BACKEND`; `SimulationOptions.cameras`, `.stage`,
  `.simulate_cameras`, `.simulate_stage`.
- `StageBackend` is unchanged.

## Behaviour details

- **Frames.** Exactly those of 08 §3.2–§3.4: the purge `XXX\n`, 10 ms, `*idn?\n`, `err?\n` (lower case, as `PI Open
  Interface` sends them) after a 50 ms pause; numbers with 4 decimals; `STP 1` (or `STP` for `stp(None)`); `#5` /
  `#7` / `#24` as single bytes. Nothing is ever added: no `ERR?` after a command, no `CSV?`, no `SAI? ALL`.
- **Answers.** `axis=value` lines, every space removed, the first value of an axis wins. Numbers accept any Python
  float syntax; booleans are numbers ≠ 0. Errors 100: an empty answer `"<CMD>: Assign values: Input string empty or
  connected axes not defined"` (the `SimStage` text), a missing axis (same text plus the answer), a value that is not a
  number. `ERR?`: empty → 100 `"No answer on ERR?"`; no leading integer → 0 with a WARNING (LabVIEW `Decimal String To
  Number`). `#5`: hexadecimal (`0x` accepted), unparsable → 100 `"#5: unexpected answer …"`. `#7`: 0xB1 / 0xB0, else
  100 `"#7 Unknown reply"`. `SAI?`: whitespace-separated ids. `CST?`: the value with spaces removed. `MAC?`: the
  `" \n"` continuation marks become `"\n"`, so the text equals the macro as stored. `RMC?` and `*IDN?`: the raw answer,
  final LF removed.
- **A line ending with `" \n"` announces another line** (08 §3.3): an answer such as `"   \n"` therefore waits for
  more and times out. The real C-663's formats are a lab question (design §6).
- **Timeouts.** −7 per line, like the VISA timeout: `ZController` passes 5000 ms. `#7` waits for one byte only.
- **Not connected.** Every method raises −4 `"<CMD>: not connected"` with the same `<CMD>` as `SimStage` (`MAC START`,
  `MAC?`, `#5`, …). The link check comes before the axis pre-checks.
- **Opening.** pyserial `serial_for_url(port, 8N1, no handshake, timeout, write_timeout = timeout, exclusive=True)`,
  then the input buffer is cleared. `COM4`, `/dev/ttyUSB0` and `socket://host:port` all work; Windows COM ports are
  exclusive anyway, `exclusive=True` makes POSIX ports behave the same.
- **The fake.** Unknown commands (`XXX`, `HLT`, `RON`, `DIO`, `ONT?`, `CSV?`…) latch 2; malformed ones latch 1
  (`MOV 1 abc`, `SVO 1 2`, `MAC?` without a name). A failed query gets **no answer** (unknown axis: 15, unknown macro:
  20), so the driver times out; `PiSerialStage`'s pre-check normally prevents that. Single bytes are handled as soon as
  they arrive, even in the middle of a partial line. `fail_next(prefix, code)` matches the frame case-insensitively and
  also applies to `"#5"`, `"#7"`, `"#24"`; a failed frame is still recorded in `frames`. `disconnect()` with no client
  connected is ignored. Positions are answered with 6 decimals, so the fake rounds what `SimStage` computes.
- **pyserial's `socket://` transport sleeps 0.3 s in `close()`.** Harmless for rehearsal; the tests remove it with the
  autouse fixture `_fast_socket_close` (`tests/conftest.py`).

## Tests

- **Both backends run the phase 1 suites.** `make_sim` (`tests/zstage/conftest.py`) is parametrized by `stage_kind`:
  `"sim"` builds a `SimStage`, `"fake"` a `FakeStageProbe` (`tests/zstage/fake_probe.py`) — the real `PiSerialStage` on
  a `FakeC663Server` whose model runs on the test's fake clock, with `SimStage`'s test hooks (`frames`,
  `inject_failure`, `position`, `power_cycle()`, …). Its `SyncedLink` returns from a write only once the fake has
  processed the bytes, so commands without answer run at the caller's fake time. Every `ZController` and Z handler test
  therefore runs twice; the only exception is `test_connect_purge_discarded_code_is_logged`
  (`@pytest.mark.sim_only`), replaced on the fake path by `test_connect_purge_own_error_is_expected`.
  `test_sim_stage.py` overrides `stage_kind` to `"sim"`.
- `ScriptedPeer` (same helper module) answers canned bytes per frame, for the formats the fake never produces.
- New modules: `tests/zstage/test_gcs_serial.py`, `test_pi_serial.py`, `test_fake_c663.py`,
  `tests/integration/test_phase2_stage.py` (the replay script through the runtime, `Exit` closing the link, a silent
  controller, the CLI with and without pyserial, and a subprocess rehearsal with `tools/replay_client.py`).
- The suite takes about 35 s (23 s before): the parametrized Z suites add about 2 s, the silent-controller test 5 s
  (the real set-up timeout, marked `slow`).

## For `tools/z_stage_check.py` (design §4.1)

Use the server's own classes, so that the check exercises the same code path:

```python
from aloxcam.zstage.pi_serial import PiSerialStage
from aloxcam.zstage.fake_c663 import FakeC663Server      # --fake
from aloxcam.settings import SimulationOptions

stage = PiSerialStage()                                 # PiSerialStage(sleep=...) in tests
code = stage.connect_rs232("COM4", 115200, 5000)        # purge; returns the err? code (normally 2, see above)
stage.qidn(); stage.qerr(); axes = stage.qsai()         # qsai() also arms the "axis not connected" pre-check
stage.qcst("1"); stage.qsvo("1"); stage.qron("1"); stage.qtrs("1"); stage.qlim("1")
stage.qtmn("1"); stage.qtmx("1"); stage.qfrf("1"); stage.qpos("1"); stage.qvel("1")
stage.qjon(1); stage.qrmc(); stage.qmac("DH"); stage.qmac("HOME")
raw = stage.link.frames                                 # deque of ("tx" | "rx" | "drop", bytes): the exact answers
answer = stage.raw_query("ONT? 1")                      # any other query, raw text (final LF removed)
stage.close()
```

- Every method raises `aloxcam.errors.StageError` (`.code`, `.source`): −4 not connected, −6 open failure or pyserial
  missing, −7 timeout, −2 / −3 write / read failure, 100 driver checks and unparsable answers. Nothing raises for a
  controller-side refusal: read it with `qerr()` (which also clears it).
- For motion tests: `mov`, `mvr`, `vel`, `stp("1")` (`STP 1`) / `stp(None)` (`STP`), `stop_all()` (`#24`), `fnl`,
  `jax(1, 1, "1")` + `jon(1, True)`, `mac_start("DH")`, `is_moving()` (`#5`), `is_controller_ready()` (`#7`). Or wrap
  the stage in `ZController(stage, StageOptions())` for `setup()`, `general_wait()`, `wait_ready()`, `check_error()`
  (these add the `ERR?` of `VEL.vi` / `JON.vi`, the waits and the error translation).
- `--fake`: `with FakeC663Server(SimulationOptions(stage_min_mm=0, stage_max_mm=100, stage_reference_s=0.2)) as
  server:` then connect to `server.url`. The model is `server.controller.model` (`position`, `referenced`,
  `error_register`, `running_macro`, `power_cycle()`, `trigger_home()`). Fault injection on `server.controller`:
  `answer_delay_s`, `drop_next_answer()`, `fail_next(prefix, code)`, `bad_ready_byte()`, `disconnect()`. Received
  frames: `server.controller.frames`.
- `GcsSerialLink(port, baudrate, timeout_s)` is usable alone for raw experiments (`send`, `send_char`, `receive`,
  `query`, `query_char`, `discard_input`, `close`).

## Only the lab can confirm

Design §6 and 08 §13.6 (questionnaire Q35): the C-663's answer formats (decimals, spacing; whether `#7` is followed by
an LF), whether `XXX` latches 2, whether `STP 1` is accepted and latches 10 (also when idle), `MVR` semantics while a
move runs, `MOV` with the joystick on (error 51?), `VEL 0` (error 17?), `#7` while referencing, whether FNL runs at
every `Z Setup`, macro behaviour while commands run, the serial number LabVIEW returns, and how the Moxa virtual COM
port behaves on a network drop (−2 / −3 / −7). `phase2_z_stage.toml` marks the steps that record these as "lab
finding".

## Lab tool `tools/z_stage_check.py` (design §4.1)

The tool runs the server's own `PiSerialStage` / `GcsSerialLink` (and `ZController.setup` for `--setup-twice`), so the
frames are exactly the server's. Its module docstring and `--help` list every frame; README "Check the Z stage" gives
the owner's commands.

### Information mode

1. **Frames.** The purge (`XXX`, 10 ms, `*idn?`, `err?`) as part of opening, then `SAI?` first (it arms the "axis not
   connected" pre-check), `*IDN?`, `ERR?`, `CST? 1`, `SVO? 1`, `RON? 1`, `TRS? 1`, `LIM? 1`, `TMN? 1`, `TMX? 1`,
   `FRF? 1`, `POS? 1`, `VEL? 1`, `JON? 1`, `RMC?`, `MAC? DH`, `MAC? HOME`, `#5`, `#7`, `ERR?`. Three additions to the
   design's list, all read-only: `#5` (the motion tests depend on it, so it is proven first), `#7` (design §6: is the
   ready byte followed by an LF? A following LF shows up as a dropped frame and is reported), and a final `ERR?` (was
   any query refused?). After a query that gets no usable answer, one more `ERR?` reads the controller's reason (only
   the last error is kept, so it must be read at once). Three timeouts in a row stop the queries; a link error (−2 /
   −3 / −4) too. A test compares the fake's received frames with `INFO_FRAMES` exactly.
2. **The purge's code** is reported with its meaning: 2 normal, 0 "XXX latched nothing" (a finding), any other code
   "an error was pending" (08 Z22).
3. **Findings** derived from the answers: Q9 (`*IDN?`, `CST? 1`, `TRS? 1` / `LIM? 1` and so FRF or FNL), Q10 (`MAC?`
   texts compared with owner-answers §5, `RMC?`), Q35 test 7 (what the Python `Z Get Serial` returns: the 3rd `*IDN?`
   field), design §6 (raw answer formats).

### Report

4. `z_stage_report_<yyyymmdd-hhmmss>.json` and `.txt` (the design says `<date>`; the time is added so that the info
   run and the motion run do not overwrite each other), next to the tool by default (`tools\`, ignored by git), or
   `--out <folder>` / `--out <name>.json`; the current folder when the tools folder is not writable. The JSON holds the
   versions (tool, aloxcam, Python, pyserial, platform, git commit), the command line, every query with its raw
   answer, every motion test with its plan, samples (`[t, moving, POS?]`) and answer, every emergency stop and
   restore step, the operator's answers, and **every raw frame** with a time stamp, direction, escaped text and hex.
   The link's own `frames` deque keeps only 512 entries, so the tool replaces it with a deque subclass that copies
   each frame into its own log (this relies on `GcsSerialLink._record` appending to `frames`). The report is also
   written after a failure (busy port, timeout, Ctrl+C); the last console lines name the file to send back.
5. **Exit codes:** 0 the tool ran to the end (findings, failed queries and refused tests are in the report), 1 it
   could not connect, lost the link or the motion tests were stopped (`#24`), 2 usage error (nothing opened, no
   report).

### Motion tests

6. **Order:** POS? during a move (Q35 test 2, the gentlest; it also proves the watch loop), STP 1 during a move
   (test 1), MVR while a move runs (test 3), MOV with the joystick on (test 4), VEL 0 (test 5), then, only with
   `--setup-twice`, Z Setup twice (test 6 / Q11) as the last and most invasive one. Test IDs in the report keep the
   Q35 numbers.
7. **Preconditions** (else the motion phase is refused and nothing beyond the information is sent): `SAI?` has axis 1,
   servo on, referenced (no reference move is ever made to fix that: run LabVIEW's `Z Setup` first), joystick off,
   `#5` = 0, `HOME` not running, a controller speed ≥ 0.05 mm/s, the start position inside `[TMN?, TMX?]` and the
   `alox.ini` range. `JON? 1` is only needed by the joystick test (a missing joystick must not block the others).
   After the operator's `yes`, `POS? 1` and `#5` are read again: a stage that moved meanwhile refuses the tests.
8. **Window** = `[TMN?, TMX?]` ∩ `[min, max]` of `zmin_DBL` / `zmax_DBL` ∩ start ± `--window`. A test move travels
   to the side with more room, at most `--window`, and must last at least 0.6 s (so that `POS?` is read during it):
   it needs `max(0.2 mm, 0.6 s × speed)` of room, else that test is refused with the numbers. STP and MVR are sent
   after 40 % of the move; MVR's test moves ⅔ of the travel then adds ⅓, so both possible answers (target + d,
   position + d) stay inside the window and are about 0.4 mm apart with the defaults. The joystick test moves 0.2 mm.
9. **Speed:** `--speed` (default 1 mm/s) must be ≥ 0.05; on the real stage ≤ 5 mm/s and never above the controller's
   current `VEL?` (the tool uses the lower one and says so). `--window` ≤ 10 mm on the real stage. `--fake` lifts the
   upper caps (rehearsal at a higher speed). The test speed is set (`VEL`, `ERR?`, `VEL? 1` checked) only once the
   first test or the move back is confirmed, so declining everything changes nothing; the original speed is restored
   at the end (with 4 decimals, as every `VEL` frame).
10. **Watch loop:** `#5` and `POS? 1` every 50 ms until the stage stands still (at the target ± 0.01 mm, or 0.3 s
    after the command for a refused move). A position more than 0.05 mm outside the window (this also catches a
    `HOME` started by digital input 1, or a joystick drift) or a timeout of 3 s + 3 × the expected duration sends
    `#24` and stops the tests. STP gets 1 s to stop the stage, then the tool sends `#24` itself and records that STP
    did not stop it; `ERR?` is read right after `STP 1`, because a later `#24` would replace a refusal code.
11. **Every test** starts with `ERR?` (clears an older error, recorded as `pending_error_before`), `POS? 1` and `#5`,
    before its `yes`. Answers: only `yes` (any case) runs a test; `quit`, `q`, `stop` or the end of input skips the
    rest; anything else, a bare Enter included, skips that test. `--setup-twice` needs a second `yes` after a warning.
12. **Stop on anything abnormal.** Once armed (the operator's first `yes`), Ctrl+C (also at a prompt), any exception,
    a timeout or a window exit sends `#24` at once. When the link itself failed (−2 / −3 / −4), or the first `#24`
    could not be written, the port is re-opened (purge included) and `#24` sent again, twice at most: a write into a
    dead socket can "succeed". If the tool had switched the joystick on, `JON 1 0` follows at once (`#24` may not stop
    a joystick drift). Then up to 3 s of `#5` polling confirm the standstill; if the stop cannot be confirmed the
    operator is told to switch the controller off. After a stop the stage is never moved back.
13. **End** (always, also after a stop): joystick off if the tool switched it on (`JAX 1 1 1`, `JON 1 0`, `ERR?`,
    `JON? 1`), original speed, then `DH`: when it ran at the start (`RMC?` = `DH` or `HOME`) and is not running now
    (STP / `#24` stop it), `MAC START DH` + `RMC?` + `ERR?` — automatically after a normal end (and said so), only
    after a typed `yes` after a stop (LabVIEW restarts it anyway at its next `Z Get Status` / `Z Stop`). The operator
    is warned that `DH` starts `HOME` (a move to 0 mm) when digital input 1 is high. A Ctrl+C during these steps
    skips the rest and says what was not restored. The move back to the start position is offered (`yes`) only
    after a normal end, at the test speed, with the watch loop on the path between the two positions.
14. **`--setup-twice`** runs `ZController(stage, StageOptions(ready_timeout_s=180, port_map={COM<n>: port}))
    .setup(ZStageSettings(..., speed = test speed))` twice: the server's defaults (`always_reference = TRUE`, FNL),
    so it measures the Python path — durations, the reference frames sent, how often `#7` answered busy (design §6
    "`#7` while referencing") — not LabVIEW's hidden option (Q35 test 6 still needs the LabVIEW server). It passes a
    plain `sleep` function so that `ZController` does not wait on a `threading.Event`, which Ctrl+C may not interrupt
    on Windows. The window does not apply during it (the stage goes to its negative limit); the move back offered
    afterwards can be long at 1 mm/s.
15. **`--yes` only with `--fake`.** `--fake` runs an in-process `FakeC663Server` (0–100 mm, `DH` running, referenced
    by FRF at 50 mm); `--port` is then refused, `--alox-ini` still limits the window.

### Limits (what the tool cannot do)

- It cannot see or block digital input 1: a `HOME` started by the button is only stopped by the window check (within
  one 50 ms poll plus the 0.05 mm margin). It does not read `DIO?` (the fake does not implement it).
- On Windows a Ctrl+C during a blocking serial read is acted on when the read returns (at most the answer timeout).
- `JAX` is not restored to an earlier value (it is not read; LabVIEW always sends `JAX 1 1 1`).
- The POS? / MVR / STP verdicts use thresholds (0.01 mm on target, ⅓ of the distance between the two MVR answers);
  the samples and frames in the report allow a second look.

### Tests

`tests/tools/test_z_stage_check.py` runs `main()` in process with **virtual time**: the tool's `sleep` /
`monotonic` and the fake's clock are one object, and `zstage.fake_probe.SyncedLink` makes a write return once the fake
has processed it, so a 1 mm move at 1 mm/s takes no real time. Faults are injected from the virtual `sleep` while the
stage moves: Ctrl+C, a `HOME` start (window exit), a dropped answer (−7), a pulled cable (`disconnect()`, plain
`GcsSerialLink`: the port is re-opened and `#24` arrives on the second connection), a move timeout, Ctrl+C with the
joystick on. Two subprocess tests run `python tools/z_stage_check.py` without PYTHONPATH (the tool adds `<repo>/src`
itself): `--fake` (information) and `--fake --motion --yes` in real time (about 4 s, marked `slow`).
