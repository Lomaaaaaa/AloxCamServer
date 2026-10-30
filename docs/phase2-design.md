# Phase 2 design: the real Z stage (PI C-663 over RS-232)

**Status:** implementation contract for phase 2 of [10-migration-plan.md](10-migration-plan.md) §5. Behaviour is specified by
[08-z-stage.md](08-z-stage.md) (cited as "08 §x"). The phase 1 contract ([phase1-design.md](phase1-design.md)) and its notes
([phase1-notes.md](phase1-notes.md)) still apply. Owner facts are in [answers/owner-answers.md](answers/owner-answers.md):
C-663 controller, RS-232 through a Moxa serial device server, 115200 Bd, COM port from `alox.ini`, L-509.40S stage, `DH` / `HOME`
macros.

## 1. Scope

| In scope | Out of scope |
|---|---|
| A real `StageBackend` for the C-663 (`PiSerialStage`) | Real cameras (phase 3) |
| A fake C-663 that speaks the serial protocol, for tests and rehearsal | Focus jobs (phase 4) |
| Per-device simulation, so the server can run with the real stage and simulated cameras at the fab | USB or daisy-chain links |
| Lab tooling: a stage check tool (read-only information plus guided tests for Q35) and a replay script for the 19 Z commands | |

## 2. Decision: raw `pyserial`, not `pipython`

08 §13.1 recommended `pipython`. Phase 2 uses a small `pyserial` transport (08 §13.5) instead:

- **Same frames as LabVIEW.** `pipython` sends extra frames of its own: `CSV?` and `ERR?` when the command object is built, `*IDN?`
  on the first `qSAI()`, `SAI? ALL` for `IsMoving()`, and `ERR?` after every command unless disabled. These change what the
  controller sees, for example when a pending error is cleared (08 §3.5). The `pyserial` transport sends exactly the frames of
  08 §3.2–§3.4, the same ones the LabVIEW PI driver sends.
- **Licence:** `pyserial` is BSD-licensed; `pipython` is distributed under a proprietary licence.
- **Testing:** `serial.serial_for_url` also opens `socket://host:port`. The whole hardware path can therefore be tested, and
  rehearsed at the fab, against an emulated controller on a local TCP port.

`StageBackend` stays the boundary, so a `pipython` backend can still be added later if USB or PI tooling is ever needed.

## 3. Modules

### 3.1 `aloxcam/zstage/gcs_serial.py`: `GcsSerialLink`

The transport. It follows the rules of 08 §3.1 and §3.3.

```python
class GcsSerialLink:
    def __init__(self, port: str, baudrate: int, timeout_s: float, *, encoding: str = "cp1252") -> None: ...
        # serial.serial_for_url(port, baudrate=..., bytesize=8, parity="N", stopbits=1, no handshake, timeout=...).
        # It accepts "COM4", "/dev/ttyUSB0" and "socket://127.0.0.1:5555". The input buffer is cleared on open.
        # StageError(-6, ...) when the port cannot be opened; StageError(-6, "pyserial is not installed ...") without pyserial.
    port: str; baudrate: int
    def send(self, cmd: str) -> None: ...            # text command, LF appended
    def send_char(self, code: int) -> None: ...      # #5 (0x05), #7 (0x07), #24 (0x18): one byte, no LF
    def receive(self) -> str: ...                    # reads until a line that does not end with " \n"; removes only the final LF
                                                     # timeout: StageError(-7, "Data Receive Timeout (RS-232, <port>, baud <b>)")
    def query(self, cmd: str) -> str: ...            # send + receive
    def query_char(self, code: int) -> str: ...      # send_char + receive (#5 answers "<hex>\n"; #7 answers one byte, 0xB0 or 0xB1)
    def discard_input(self) -> None: ...
    def close(self) -> None: ...                     # idempotent
    frames: collections.deque                        # the last 512 (direction, bytes) pairs, for tests and diagnostics
```

- Every frame is also logged at DEBUG level on the `aloxcam.zstage.serial` logger.
- `#7` answers a single byte with no LF. `query_char(7)` must not wait for a line: it reads one byte, and a timeout gives −7.
- The link is used only under `ZController`'s lock (phase 1 thread contract), so it needs no lock of its own.

### 3.2 `aloxcam/zstage/pi_serial.py`: `PiSerialStage(StageBackend)`

- Implements every `StageBackend` method with the exact frames of 08 §3.2–§3.4.
  - **Open:** close any open link, wait 50 ms, open the port, then purge: `XXX\n`, wait 10 ms, `*idn?\n` and read,
    `err?\n` and read. `connect_rs232` returns the code read by that `err?` (the discarded pending error, 08 Z22).
  - **Numbers** use `format_gcs_float` (4 decimals) for `MOV`, `MVR` and `VEL`; modes are integers (`SVO 1 1`, `JON 1 0`).
  - **Answers** are parsed as in 08 §3.3 (`axis=value` lines, spaces removed). An empty answer gives error 100 with the
    `Assign values …` text.
  - **Driver pre-checks** (08 §3.3): an empty axis gives 100 `"<CMD> No axis specified"`. An axis that is not in the `SAI?` list read
    at set-up gives 100 `"Commanded axis not connected: <axis>"`. Nothing is sent in either case.
  - **Error texts** come from 08 §3.5, and `translate_error` uses `gcs_errors`.
- The constructor takes `link_factory: Callable[[str, int, float], GcsSerialLink] = GcsSerialLink` so tests can inject a link.
  `timeout_ms` from `connect_rs232` becomes the link timeout.
- `is_connected`, `close` and every query behave as the `StageBackend` docstrings say. A query without a link raises the same −4
  `"<CMD>: not connected"` as `SimStage`.

### 3.3 `aloxcam/zstage/fake_c663.py`: `FakeC663` and `FakeC663Server`

- **`FakeC663(options: SimulationOptions, *, clock=None)`** interprets the GCS ASCII protocol on top of the `SimStage` model:
  motion, referencing, macros, joystick and the latched error. It reuses `SimStage` and does not duplicate it.
  `feed(data: bytes) -> bytes` takes any chunking. It buffers partial lines and handles 0x05, 0x07 and 0x18 immediately.
  - **Answer formats** follow 08 §3.4: `1=12.500000`, `0`, the single byte 0xB1 or 0xB0, multi-line answers with `" \n"` on every line
    but the last (`MAC? DH` returns the owner's macro text), and an empty line for `RMC?` when no macro runs.
  - **Errors:** an unknown command latches error 2, and `XXX` is answered by latching 2.
- **Fault injection:** `answer_delay_s`, `drop_next_answer()`, `fail_next(command_prefix, code)` (latches `code` instead of
  executing), `bad_ready_byte()` (the next `#7` answers `"?"`), and `disconnect()`.
- **`FakeC663Server(options, *, clock=None)`:** a TCP server on 127.0.0.1 (port 0 by default) in a daemon thread. It serves one
  connection at a time; the controller state survives reconnections. It exposes `.url` (`"socket://127.0.0.1:<port>"`),
  `.controller` (the `FakeC663`), `start()` and `close()`, and works as a context manager.
- **Command line:** `python -m aloxcam.zstage.fake_c663 [--port 5555] [--initially-referenced]` runs one for rehearsal, together with
  `[stage] port_map = COM4=socket://127.0.0.1:5555`.

### 3.4 Settings and runtime

- **`[simulation]`** gets two new booleans, `cameras` and `stage`, both FALSE by default. `enabled = TRUE` still means both. The
  effective values are `simulate_cameras = enabled or cameras` and `simulate_stage = enabled or stage`.
- **CLI:**
  - `--simulate` simulates both, as before;
  - the new `--simulate-cameras` simulates only the cameras and uses the real stage;
  - `build_runtime(loaded, *, simulate=…, simulate_cameras=None, simulate_stage=None, …)` keeps `simulate` for compatibility;
    the two new arguments override it.
- **Real stage:** `ZController(PiSerialStage(), cfg.stage)`. The port and baud rate come from `alox.ini` `[hardware.z]` through
  `ZController.setup` (with `[stage] port_map`). As in LabVIEW, the link is opened by `Z Setup`, not when the server starts.
- **Real cameras** are still unavailable. If the cameras are not simulated, start-up fails with `ConfigError` and exit code 2
  ("real cameras arrive in phase 3").
- **`pyserial`** is imported lazily. If it is missing when the real stage is selected, start-up fails with `ConfigError` and a
  message telling the user to `pip install pyserial` (or `aloxcam[stage]`).
- **`--check-config`** prints the stage mode: simulated, or real with the mapped port and baud rate, and whether `pyserial` is
  importable.
- **`pyproject.toml`:** a `stage = ["pyserial>=3.5"]` extra, and `pyserial` added to `dev` for the tests.

## 4. Lab tooling

### 4.1 `tools/z_stage_check.py`

A standalone check of the controller through `PiSerialStage`, so it tests the same code path as the server. It is run on the lab
PC while the LabVIEW server is stopped, because the COM port is exclusive.

- **`info` (default, read-only, safe):**
  - connect;
  - query `*IDN?`, `ERR?`, `SAI?`, `CST? 1`, `SVO? 1`, `RON? 1`, `TRS? 1`, `LIM? 1`, `TMN? 1`, `TMX? 1`, `FRF? 1`, `POS? 1`,
    `VEL? 1`, `JON? 1`, `RMC?`, `MAC? DH`, `MAC? HOME`;
  - print everything and write a report (`z_stage_report_<date>.json` plus a readable `.txt`).
  - It sends no command that moves the stage, changes a setting or stops a macro.
- **`tests` (opt-in with `--motion`):** the hardware checks of questionnaire Q35 and 08 §13.6.
  - **Tests:** `STP 1` during a move; `POS?` during a move; `MVR` while a move runs; `MOV` with the joystick on; `VEL 1 0.0000`;
    optionally `Z Setup` twice (FNL every time?).
  - **Safety:**
    - every test is explained, then waits for the operator to type `yes`;
    - moves stay inside the intersection of `[TMN?, TMX?]`, `[zmin_DBL, zmax_DBL]` from `alox.ini` (when `--alox-ini` is given)
      and an operator-given window around the start position (default ±1 mm);
    - the test speed is limited (default 1 mm/s);
    - Ctrl+C or any error sends `#24` and then restores what it can;
    - at the end the original speed is restored, the stage moves back to its start position after confirmation, and `DH` is
      restarted if it was running at the start.
- **Options:**
  - `--port COM4 --baud 115200`, or `--alox-ini <path>` to read `[hardware.z]`;
  - `--fake` runs everything against an in-process `FakeC663Server`, for rehearsal and the tests;
  - `--yes` answers every confirmation (only allowed together with `--fake`).

### 4.2 `tools/scripts/phase2_z_stage.toml`

A replay script for the Python server running with the real stage (`--simulate-cameras`). It sends:

- `Z Setup`, `Z Get Serial`, `Z Get Status`, `Z Get Error`;
- `Z Set Speed` and `Z Get Speed` (slow speed);
- `Z Get Reference` and `Z Get Position`;
- `Z Move Absolute` / `Z Move Relative` to positions passed with `--var` (inside a safe window), each followed by
  `Z Wait End Move`;
- one move outside the software range, which expects a 9405 error ACK;
- `Z Stop`;
- the three joystick commands, `Z Set Min Limit` / `Z Set Max Limit` (restoring the INI values), `Z Cleanup` and `Exit`.

The same script runs in the tests against `FakeC663Server`.

## 5. Tests

- **`GcsSerialLink`** against a socket peer: multi-line answers, removal of the final LF only, the −7 timeout, single-byte
  commands, `#7` without LF, cp1252 bytes.
- **`PiSerialStage`:**
  - every method's exact frame bytes against the 08 §3.4 table, and the purge sequence with its returned code;
  - answer parsing and the empty-answer error;
  - the driver pre-checks;
  - −4 when not connected and −6 on an unopenable port.
- **The Z handler and `ZController` suites** run through `PiSerialStage` + `FakeC663Server` as well as through `SimStage`.
  Parametrize the existing fixtures where possible rather than copying tests.
- **Runtime:** `build_runtime(simulate_cameras=True, simulate_stage=False)` with `port_map` pointing `COM<n>` at a
  `FakeC663Server`, then `phase2_z_stage.toml` through the replay client. `Exit` closes the link.
- **CLI:** `--simulate-cameras` with `--check-config`; without `pyserial` the start-up message is clear (monkeypatched import).
- **`tools/z_stage_check.py`:** the `info` mode, and `--motion --fake --yes`, against the fake.

## 6. What only the lab can confirm

These are 08 §13.6 and questionnaire Q35:

- the C-663's real answer formats (number of decimals, spacing) — the parser must accept any `axis=value` spacing and number format;
- whether `STP 1` is accepted;
- `MVR` semantics;
- `MOV` with the joystick on;
- `VEL 0`;
- `#7` while referencing;
- whether FNL runs at every `Z Setup`;
- macro behaviour while commands run.

The check tool records all of them in its report.
