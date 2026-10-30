# AloxCamServer

Re-implementation of the ALOX Vision Core **Alox Camera Server** (`alox_camera_server.vi`, LabVIEW 2018).

The LabVIEW server is a TCP server that drives:

- two cameras (High Mag / Low Mag) through NI-IMAQdx;
- a PI Mercury Z stage through the PI GCS driver over RS-232;
- LED and bulb lighting through the cameras' digital outputs;
- autofocus and refocus routines.

The Python package `aloxcam` replaces it. **Phase 1** of the migration plan (the protocol skeleton) is done: the LabVIEW
wire protocol, the configuration files and event logs, the server runtime, the 46 commands with **simulated** cameras
and Z stage, a protocol client library and a replay tool. The real hardware backends come in phase 2 (PI Z stage) and
phase 3 (cameras). The autofocus and refocus jobs come in phase 4; until then `Cam Autofocus` and `Cam Refocus` reply
error 9507 ("not implemented in this phase").

## Documentation

The LabVIEW code has been analysed into a specification for the re-implementation. It is in [`docs/`](docs/README.md).

- [`docs/README.md`](docs/README.md): index and reading order.
- [`docs/10-migration-plan.md`](docs/10-migration-plan.md): recommended language, target design, phased plan, risks.
- [`docs/open-questions.md`](docs/open-questions.md): questions for the system owner, as a fill-in questionnaire.
- [`docs/phase1-design.md`](docs/phase1-design.md): the phase 1 implementation contract (package layout, APIs, tests).
- [`docs/phase1-notes.md`](docs/phase1-notes.md): decisions taken while implementing phase 1.

## Development set-up

Python 3.11 or later. The only runtime dependency is numpy; the tests need pytest.

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

- `--simulate` uses the simulated cameras and Z stage (`[simulation] enabled = TRUE` does the same). Phase 1 has no
  hardware backend: without it the server exits with code 2.
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

## Checking the configuration

```sh
python -m aloxcam --setup-ini /tmp/alox/alox_setup.ini --check-config
```

prints the folders, the effective listen address, the configuration warnings, the wire enum tables, the routing table
and its self-check, and the `[compat]` values. Exit code 0 means usable; 1 means a fatal problem (listed at the end).

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
header) and `phase1_error_replies.toml` needs `[compat] send_error_replies = TRUE` and `reply_unknown_commands = TRUE`.

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
expect = { reply = "none" }              # outside [zmin_DBL, zmax_DBL]: error 9405, the error ACK is not sent

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
