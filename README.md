# AloxCamServer

Re-implementation of the ALOX Vision Core **Alox Camera Server** (`alox_camera_server.vi`, LabVIEW 2018).

The LabVIEW server is a TCP server that drives:

- two cameras (High Mag / Low Mag) through NI-IMAQdx;
- a PI Mercury Z stage through the PI GCS driver over RS-232;
- LED and bulb lighting through the cameras' digital outputs;
- autofocus and refocus routines.

The Python package `aloxcam` replaces it. **Phase 1** of the migration plan (the protocol skeleton) is done: the LabVIEW
wire protocol, the configuration files and event logs, the server runtime, the 46 commands with **simulated** cameras
and Z stage, a protocol client library and a replay tool. **Phase 2** adds the real PI C-663 Z stage over RS-232
(see [Phase 2: real Z stage](#phase-2-real-z-stage)); the cameras stay simulated until phase 3. The autofocus and
refocus jobs come in phase 4; until then `Cam Autofocus` and `Cam Refocus` reply error 9507 ("not implemented in this
phase").

## Documentation

The LabVIEW code has been analysed into a specification for the re-implementation. It is in [`docs/`](docs/README.md).

- [`docs/README.md`](docs/README.md): index and reading order.
- [`docs/10-migration-plan.md`](docs/10-migration-plan.md): recommended language, target design, phased plan, risks.
- [`docs/open-questions.md`](docs/open-questions.md): questions for the system owner, as a fill-in questionnaire.
- [`docs/phase1-design.md`](docs/phase1-design.md): the phase 1 implementation contract (package layout, APIs, tests).
- [`docs/phase1-notes.md`](docs/phase1-notes.md): decisions taken while implementing phase 1.
- [`docs/phase2-design.md`](docs/phase2-design.md): the phase 2 contract (the real Z stage over RS-232).
- [`docs/phase2-notes.md`](docs/phase2-notes.md): decisions taken while implementing phase 2.

## Development set-up

Python 3.11 or later. The only runtime dependency is numpy; the real Z stage needs pyserial (`pip install -e
".[stage]"`); the tests need pytest and pyserial (both in `[dev]`).

```sh
python3 -m venv .venv
.venv/bin/pip install -e ".[dev]"
```

The commands below assume this virtual environment is active (`. .venv/bin/activate`). Without the editable install,
prefix the `python -m aloxcam` commands with `PYTHONPATH=src`.

## Running the tests

```sh
python -m pytest                       # the whole suite (about 20 s)
python -m pytest -m "not slow"         # without the tests that take more than 2 s
python -m pytest tests/integration     # end to end: in-process server, replay scripts, CLI smoke tests
```

The tests need no hardware and no network besides loopback; servers listen on ephemeral ports.

## Running the simulated server

The server finds its folders as the LabVIEW one does: `alox_setup.ini` (`[folders]`) names the settings, vision and
data folders; the settings folder holds `alox.ini` (read-only, shared with the other ALOX programs) and the optional
Python-only `alox_camera_server.ini`. A minimal installation on Linux, using the production `alox.ini`:

```sh
mkdir -p /tmp/alox/settings /tmp/alox/vision /tmp/alox/data
printf '[folders]\nsettings = /tmp/alox/settings\nvision = /tmp/alox/vision\ndata = /tmp/alox/data\n' \
    > /tmp/alox/alox_setup.ini
cp docs/answers/ini/alox.ini /tmp/alox/settings/
python -m aloxcam --setup-ini /tmp/alox/alox_setup.ini --simulate --port 5030
```

- `--simulate` uses the simulated cameras and Z stage (`[simulation] enabled = TRUE` does the same);
  `--simulate-cameras` keeps the cameras simulated and drives the real Z stage (next section). There is no camera
  backend before phase 3: without one of the two options (or `[simulation] enabled` / `cameras = TRUE`) the server
  exits with code 2.
- `--port` overrides the `alox.ini` port (503, which needs administrator rights on Linux); `--port 0` picks a free
  port, and `--ready-file PATH` writes `<host> <port>` once the server listens.
- Stop the server with Ctrl+C or with the `Exit` command. Exit codes: 0 normal stop, 1 start-up failure, 2 usage error,
  3 shutdown timeout.
- The LabVIEW-format event logs are written to `<vision>/logs/<yymmdd>_camera_events.log` and `…_z_events.log`, the
  diagnostic log to `<vision>/logs/<yymmdd>_camera_server_diag.log` and to stderr.
- `python -m aloxcam --write-settings-template /tmp/alox/settings/alox_camera_server.ini` writes the template of the
  Python-only settings: the `[compat]` switches of [docs/10](docs/10-migration-plan.md) §6, `[simulation]`, `[wire]`,
  `[logging]`, `[stage]`, …
- `python -m aloxcam --help` lists every option. After `pip install -e .` the `aloxcam` command is the same program.

Without `--setup-ini`, `alox_setup.ini` is read from the current folder (or `$ALOX_EXE_DIR`); when it is missing, the
LabVIEW default folders `D:\alox\…` apply, which off Windows need `ALOX_DRIVE_D=<folder>` or `--settings-dir`.

### On Windows (the target platform)

In a Command Prompt, from the repository folder. `py` is the Python launcher installed with Python 3.11+.

```bat
py -3 -m venv .venv
.venv\Scripts\pip install -e ".[dev]"
.venv\Scripts\python -m pytest
```

To run the simulated server against the real ALOX folders, use the existing `alox_setup.ini` (or a copy). It only reads
`alox.ini` and never writes to it. Choose another port than 503 while the LabVIEW server is running:

```bat
.venv\Scripts\python -m aloxcam --setup-ini D:\alox\alox_setup.ini --check-config
.venv\Scripts\python -m aloxcam --setup-ini D:\alox\alox_setup.ini --simulate --port 5030
.venv\Scripts\python tools\replay_client.py tools\scripts\smoke.toml --port 5030
```

The simulated server writes its event logs to `<vision>\logs`, next to the LabVIEW ones. Their file names are the same
(`<yymmdd>_camera_events.log`), so point `alox_setup.ini` at a separate test folder if you don't want both servers writing
to the same daily files.

## Phase 2: real Z stage

The server drives the real PI C-663 (RS-232, 115200 Bd, through the Moxa virtual COM port) with the GCS frames of the
LabVIEW PI driver, while the cameras stay simulated. It uses `pyserial`, not PI's `pipython`
([docs/phase2-design.md](docs/phase2-design.md) §2).

```bat
.venv\Scripts\pip install -e ".[stage]"
.venv\Scripts\python -m aloxcam --setup-ini D:\alox\alox_setup.ini --simulate-cameras --check-config
.venv\Scripts\python -m aloxcam --setup-ini D:\alox\alox_setup.ini --simulate-cameras --port 5030
```

- **Stop the LabVIEW server first.** The COM port is exclusive: while LabVIEW holds it, `Z Setup` fails with error −6
  (and the other way round).
- The port and baud rate come from `alox.ini` `[hardware.z]` (`comport_I32` → `COM<n>`, `baudrate_I32 = 3` →
  115200). As in LabVIEW, the port is opened by the client's `Z Setup`, not at start-up, and `Z Setup` references the
  axis (the stage drives to its negative limit switch, `[stage] always_reference = TRUE`); `Z Get Status`, `Z Stop` and
  `Z Cleanup` restart the `DH` macro as before. `--check-config` shows the stage mode, the port and whether pyserial is
  importable.
- `[stage] port_map` in `alox_camera_server.ini` sends a port name elsewhere: `COM4=/dev/ttyUSB0` on Linux, or
  `COM4=socket://127.0.0.1:5555` for the fake controller below. `[simulation] cameras = TRUE` is the INI form of
  `--simulate-cameras`.
- Every frame exchanged with the controller is logged at DEBUG on the `aloxcam.zstage.serial` logger
  (`--log-level DEBUG`), in the diagnostic log.

### Rehearsal with the fake controller

`aloxcam.zstage.fake_c663` emulates a C-663 on a TCP port (the simulated stage behind the real serial protocol). It
lets the whole real-stage path be rehearsed without the machine:

```sh
python -m aloxcam.zstage.fake_c663 --port 5555          # prints the URL; Ctrl+C stops it
```

then in `<settings>/alox_camera_server.ini` (with the production `alox.ini`, `comport_I32 = 4`):

```ini
[stage]
port_map = COM4=socket://127.0.0.1:5555
```

and start the server with `--simulate-cameras` as above. The fake's travel is 0–100 mm by default (`--max-mm`), its
reference move takes 2 s (`--reference-s`), `--initially-referenced` and `--no-dh` change its power-up state, and
`--log-level DEBUG` prints every frame it receives. Remove the `port_map` line before running against the real stage.

### The Z replay script

`tools/scripts/phase2_z_stage.toml` sends the 19 Z commands to a server running with the real stage (or the fake):
set-up, status, a slow speed, absolute and relative moves with `Z Wait End Move` and position checks, a move outside
the software range (9405), `Z Stop` during a move (error 10 left pending), the joystick commands, the limits, `Z
Cleanup` and `Exit`. **The stage moves**: read the script header first. The positions have no default; choose a safe
window and pass the `alox.ini` limits:

```bat
.venv\Scripts\python tools\replay_client.py tools\scripts\phase2_z_stage.toml --port 5030 ^
    --var z_abs=60.0 --var z_rel=0.5 --var z_rel_end=60.5 --var z_min=0.0 --var z_max=90.0 --record z_stage.jsonl
```

Steps marked "lab finding" record what only the real controller can show (08 §13.6: `STP 1`, `MVR`, the serial
number); a FAIL there is a finding to report. Keep the `--record` file. The script ends with `Exit`.

## Checking the configuration

```sh
python -m aloxcam --setup-ini /tmp/alox/alox_setup.ini --check-config
```

prints the folders, the effective listen address, the device modes (simulated cameras, simulated or real Z stage
with its port), the configuration warnings, the wire enum tables, the routing table and its self-check, and the
`[compat]` values. Exit code 0 means usable; 1 means a fatal problem (listed at the end).

## Replay client

`tools/replay_client.py` runs a script of commands against a server (Python or LabVIEW), checks every reply and can
record the session and compare it with an earlier recording. With the server above running:

```sh
python tools/replay_client.py tools/scripts/smoke.toml --port 5030
```

```text
[  1] PASS heartbeat frame (ignored by the server)  none
[  2] PASS Get Heartbeat  U32 8602644  err={F,0,""}  1.5 ms
[  3] PASS Exit: no reply, the server closes the connection  none
3 step(s) run: 3 passed, 0 failed -- OK
```

The smoke script ends with `Exit`, so the server stops. Exit codes: 0 every step passed (and no compare difference), 1 a
step failed or a difference was found, 2 usage or connection error. The other scripts in `tools/scripts/` are used by the
integration tests: `phase1_all_commands.toml` sends every command (it expects the reference `alox.ini` of
[docs/09](docs/09-configuration-logging-errors.md) §5.6 and takes `--var sim_dir=… --var ffc=… --var dfc=…`, see its
header) and `phase1_error_replies.toml` needs `[compat] send_error_replies = TRUE` and `reply_unknown_commands = TRUE`;
`phase2_z_stage.toml` is described above.

A script is a TOML file (format: [docs/phase1-design.md](docs/phase1-design.md) §3.7.2):

```toml
[session]
port = 5030
timeout_s = 5.0

[vars]                                   # ${name} in any string; --var name=value overrides
cam = "cam0"

[[step]]
command = "Cam Setup"
data = { camera_type = "High Mag.", camera_name = "${cam}", number_of_buffers = 5, continuous_mode = true }
expect = { reply = "ACK", error = { status = false, code = 0 } }

[[step]]
command = "Cam Get Exposure"
data = { camera_type = "High Mag." }
expect = { reply = "DBL", value = 10000.0, abs_tol = 1e-9 }

[[step]]
command = "Z Move Absolute"
data = { pos_mm = 95.0 }
expect = { reply = "ACK", error = { status = true, code = 9405 } }   # outside [zmin_DBL, zmax_DBL]

[[step]]
command = "Exit"
expect = { reply = "none", connection = "closed" }
```

To compare the Python server with the LabVIEW one, record the same script against both and diff the replies, ignoring
the tick fields (`TCP Sending timestamp`, the `Get Heartbeat` value) and, if needed, the image bytes:

```sh
python tools/replay_client.py session.toml --host <LabVIEW PC> --port 503 --record labview.jsonl
python tools/replay_client.py session.toml --port 5030 --compare labview.jsonl --mask tick
```

Other options: `--timeout S`, `--settings-ini PATH` (wire enum overrides and layouts from an `alox_camera_server.ini`),
`--enums PATH`, `--templates PATH` (captured variant headers), `--stop-on-fail`, `--quiet`.

The protocol client used by the tool is a library (`aloxcam.client.AloxClient`):

```python
from aloxcam.client import AloxClient
from aloxcam.dispatch import ReplyKind
from aloxcam.wire.enums import WireEnums
from aloxcam.wire.messages import PayloadCodec

enums = WireEnums.load()
with AloxClient("127.0.0.1", 5030, enums=enums, payloads=PayloadCodec(enums)) as client:
    reply = client.call("Get Heartbeat")
    value, error = reply.value(ReplyKind.U32)
```
