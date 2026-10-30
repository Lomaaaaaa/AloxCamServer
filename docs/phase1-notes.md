# Phase 1 implementation notes

This file records the decisions taken while implementing [phase1-design.md](phase1-design.md) where the design left a point
open. Later work packages must follow them. Each package adds its own section.

## WP1 (foundation) and WP2 (LabVIEW codec and wire layer)

### Decisions where the design was open

1. **`ConfigError` code.** `AloxError` always needs a code, and none is defined for configuration errors. WP1, WP2 and all
   later packages use `Code.LV_INVALID_INPUT` (1).
2. **`EnvelopeError` constructor:** `EnvelopeError(source, *, code_value=None, notifier=None)`. The code is fixed at 116.
3. **Payload dataclasses are frozen**, because they cross threads inside `Request`. Use `dataclasses.replace` to change a
   field. `roi` stays a `list`.
4. **`ImageData`** is `frozen=True, eq=False`, with its own `__eq__` that compares dtype, shape, pixels and the order of the
   custom data. It is unhashable.
5. **`reject_undecodable(ctx, req)` reads `req.decode_error`.** For the handler-exception path (WP4), set
   `req.decode_error = cluster_of(exc)` first, then call it. When `decode_error` is None, a 9500 cluster is used. When `spec` is
   None, the request is only logged and dropped.
6. **`finish_camera` defaults:**
   - order CUSTOM behaves like A; order Z replies first, then logs without decoration;
   - `extra` defaults to `spec.extra or spec.name`, and `suffix` to `spec.suffix or ""`.
7. **`send_reply`:** a value of None is replaced by the reply kind's default (an IMAGE value stays None). It raises `ValueError`
   when `req.spec` is None.
8. **`ServerState.update`** refuses `lock` and the history fields, and checks every name before writing anything.
9. **Paths:**
   - `ALOX_DRIVE_<L>` maps the root of drive L.
   - `LVPath.to_local` applies the text rules to `"/" + components`, so an unmapped `("D", "x")` becomes the native path
     `/D/x` rather than None.
   - `read_path` rejects path types above 3 and reads a zero-length body as the empty path.
10. **Enum overrides and validation:**
    - a table in an override file must give `items`; `label`, `ctl` and `size` default to the packaged table, and `lenient`
      defaults to false;
    - unknown keys and wrong value types raise `ConfigError`;
    - `[wire.items]` and `[wire.size]` keys are stripped and lower-cased;
    - additional checks beyond the design: missing or empty table, a label starting with `#`, too many items for the size, a
      string instead of a list, and `Log` inside the Z range (treated as an overlap).
11. **Small behaviours:**
    - routing self-check messages read `"<name>: case unreachable (routes to camera|z|nowhere)"` or `"… (not in the
      camera_command enum table)"`;
    - `Datatypes.name_of` returns None for `Statut` and for unknown codes;
    - `decode_response_data` and `parse_reply_value` must consume the bytes exactly;
    - `flatten_reply` with a None value writes the reply kind's default.

### Test helpers (`aloxcam.testing`, `tests/conftest.py`)

- **`RecordingReplies`:**
  - a reply to a request with `conn_id == 0` is recorded with `sent=False`;
  - `error_reply(NONE)` records nothing but still sets `replied`;
  - helpers: `sent_replies`, `last`, `clear()`. `RecordingEvents` also has `clear()`.
- **`packaged_enums()`** loads the packaged enum table once.
- **`REFERENCE_ALOX_INI`** is the reference file of 09 §5.6.
- **`AloxTree` / `write_alox_tree(root)`** builds a temporary installation.
- **`tmp_alox_tree` fixture** returns an `AloxTree` (root, exe, settings, vision, data, setup_ini, alox_ini).
  - `exe/alox_setup.ini` holds POSIX paths.
  - Both INI files are CRLF and cp1252.
  - No `logs` folders and no `alox_camera_server.ini` are created.
- **`make_request`:**
  - when `code` is not given, `"#<n>"` gives n, and an unknown label raises `ValueError`;
  - `spec` is None for unknown names.

### Codec details

- **`PayloadCodec` templates:**
  - `templates` accepts bytes or hex text and validates each one; a bad template raises `ConfigError`;
  - a template also applies to NOT_READ commands;
  - a `[wire.layout]` override for `Camera Init Data` still works together with `init_data_fallback`.
- **Synthetic variant headers:**
  - element type descriptors are not deduplicated;
  - enum descriptors list the items from `WireEnums.table()`; any other `EnumCodec` is probed code by code until a label fails.
- **Additional public names:**
  - `lv.flatten`: `KINDS`, `ENUM_PREFIX`, `enum_id_of`, `read_value`, `write_value`, `Writer.i32_array`, and the attributes
    `Reader.data`, `Reader.encoding`, `Writer.encoding`, `Writer.errors`;
  - `wire.messages.PAYLOAD_CLASSES` (all 30 payload classes);
  - `wire.protocol.ENVELOPE_TRAILER_SIZE`.

### Golden vectors (`tests/golden/spec_vectors.py`)

- `fill(template, **codes)` writes integers as big-endian U16 and inserts bytes unchanged.
- Payload rows use `{camera_type}`, `{autofocus_speed}`, `{autofocus_method}` and `{refocus_method}` placeholders. Fill them with
  `fill(row.hex, **row.code_values(enums))`.
- Frames use `{tt}` and `{cc}`.
- For WP3: `LOG_LINES` (the camera line is a template: call `.filled(code, source)`) and `OFFSET_ZERO_EXAMPLE` (09 §7.3, expected
  tail `b"ut of range (Move Absolute)\r\n"`).
- For WP4 (`test_reply`): `GET_HEARTBEAT_REPLY`.

## WP3 (configuration and event logs)

### Additions to the design

- **New public names:** `config.folders.SETUP_INI_NAME`, `config.alox_ini.ALOX_INI_NAME`, `DailyFileHandler.path_for(day)` and
  `DailyFileHandler.current_path`. `DailyFileHandler` also takes the keyword-only arguments `encoding="utf-8"`, `create_dir=True`
  and `level`.
- **Value checks in `load_server_config`.** An invalid value gives the default plus a warning:
  - `tick_source` (monotonic | windows) and `codec` (provisional | nivision) are normalized to lower case;
  - `reference_move` (FNL | FPL), `pixel_type` (U8 | U16) and `diag_level` (a stdlib level name) are normalized to upper case;
  - encodings must exist (`codecs.lookup`), `decimal_separator` must be one character, and `listen_address` and
    `diag_file_stem` must not be empty;
  - ports must be 0–65535 and floats finite; timeouts, frame rates, image size and `max_payload_mb` must be above 0, and the
    other numbers 0 or more.
- **`EventLogs` with `create_log_dirs` = TRUE** also creates `logs/` before each write, so lines are not lost if the folder
  disappears while the server runs.

### Behaviour details

- **`NIConfig`:**
  - section names match case-insensitively but are not trimmed (09 §6.6 sketch);
  - keys written before the first section can never be read;
  - `sections()` and `keys()` list each name once, with its first spelling, in file order; `keys()` only shows the first copy of
    a duplicated section;
  - `read_list` reads like `read_str`, splits on commas, strips each item and drops empty ones; an empty value gives `()`;
  - `load` removes a UTF-8 BOM whatever the encoding and replaces undecodable bytes.
- **Folders:**
  - `Folders.exe` is always `exe_dir` or `executable_folder()`, even when `setup_ini` is given;
  - a value that is empty or only spaces counts as empty (default plus warning, D3);
  - a file that is missing or unreadable gives the defaults plus a warning;
  - the `--settings-dir` value is used as given, without localization, and relative folder paths are kept.
- **`load_alox_ini`:**
  - a missing key gives its default with only a DEBUG log, because four of the real files have no `lightStatusInverter_BOOL`;
  - a High Mag `frameratemax_DBL` is ignored, also logged at DEBUG;
  - `cameraconnectortype_STR` is compared without trimming.
- **`load_server_config`:**
  - in `[wire.items]`, `[wire.size]` and `[wire.layout]`, an empty entry or a non-integer size is warned and skipped; the size
    values (1/2/4) are checked by `wire.enums`;
  - `[wire] enums_file` goes through `localize_path_text`; an unmappable path gives None plus a warning;
  - `[server] max_payload_mb` may be a fraction and becomes `int(round(mb * 2**20))`;
  - `[stage] port_map` keys are upper-cased (`com3` → `COM3`) and backslashes are kept (`\\.\COM10`); a bad item is warned and
    skipped, and the first occurrence wins;
  - `[image] nivision_library` is read without unescaping, so Windows backslashes are kept;
  - duplicate keys are not warned.
- **Server INI template:** `write_template` writes the Appendix A text as CRLF ASCII and does not create parent folders. A test
  checks that `SERVER_INI_TEMPLATE` equals Appendix A of `phase1-design.md`, so the two must be edited together.
- **`EventLogs`:**
  - `recent()` holds both error and info lines (`deque(maxlen=max(0, recent_lines))`), including lines whose file write failed;
  - a missing folder (`vision` or `data` = None) is reported once in the diagnostic log;
  - an unknown `options.encoding` falls back to cp1252 with a warning;
  - offset-0 mode (`append_errors = FALSE`) reproduces the 09 §7.3 example byte for byte; info lines always append;
  - every diagnostic write warning includes the lost line and comes from the `aloxcam.eventlog` logger.
- **`setup_diagnostic_logging`:**
  - it can be called again: handlers from an earlier call are closed and removed;
  - it sets the level of the `aloxcam` logger and leaves `propagate` alone, so `caplog` keeps working; tests that call it should
    restore the logger afterwards (`test_eventlog.py` has a fixture for this);
  - the diagnostic file is `<vision>/logs/<yymmdd>_<diag_file_stem>.log`, UTF-8 with plain newlines; if it cannot be opened, the
    failure is reported once per day and that day's records are dropped.
- **Tests:** path tests clear `ALOX_DRIVE_C`, `ALOX_DRIVE_D` and `ALOX_EXE_DIR` and skip the drive-mapping cases on win32. The
  unwritable-folder test puts a file where `logs/` should be, so it also works when tests run as root. The five owner INI
  files are byte-exact copies in `tests/config/data/real/`.

## WP5 (Z stage)

### Owner answer applied after the merge

- **`[stage] always_reference` now defaults to TRUE** (`settings.StageOptions`, the `alox_camera_server.ini` template, design §6
  row 8 and Appendix A). The owner wants the axis referenced at every `Z Setup` (Q11). The implicit re-initialisation inside
  `Z Move …` uses the same set-up, so it also re-references, as in LabVIEW.

### Deviations from the design

1. **`stage_dh_macro_s` ≤ 0 means `DH` runs forever.** That matches the owner's `DH`, a permanent daemon that restarts itself.
   A value > 0 makes `DH` end that long after it starts.
2. **`DH` runs from power-up** (`SimStage(options, *, clock=…, dh_running=True)`).
3. **The GCS error-text table has three extra codes** with PI's texts: −4, −6 and 20 ("Macro not found").
4. **`setup()` translates an error from any of steps 1–18,** not only the final `ERR?`, because LabVIEW's `GCSTranslateError`
   receives the whole error chain. The error of the "Set Normal Speed" `VEL` comes after that step and is not translated.

### Wiring (for WP4/WP7)

- `ZContext.from_base(base, *, stage, stage_options)` builds the Z context for `RuntimeParts.make_z_context`. `Z_HANDLERS` is
  read-only and holds the 19 Z commands plus `Exit`.
- `app.build_runtime`:
  - create `SimStage(config.simulation)`, then `ZController(sim, config.stage)` with the default `sleep` and `monotonic`;
  - `abort_waits = controller.cancel_waits`;
  - closing the hardware calls `controller.close()`.
- With the default `time.sleep`, `ZController` sleeps on an interruptible wait, so `cancel_waits()` cuts long waits short. An
  injected `sleep` (tests) is called as is.

### Behaviour details

- **Shutdown abort:** after `cancel_waits()`, every wait and every later `setup()` raises
  `StageError(9510, "<what>: aborted by server shutdown")`. `<what>` is "Wait for axes to stop", "Wait for controller ready" or
  "Z Setup". A cancelled wait skips its additional wait (LabVIEW "Stopped").
- **Timeout texts:** −7 `"Wait for axes to stop.vi: Timeout."` and −7 `"Wait for controller ready: Timeout."`.
- **Not connected:** −4 `"<CMD>: not connected"`. For example `Z Cleanup` gives `"RMC?: not connected"` and `Z Wait End Move`
  without a link gives `"#5: not connected"`.
- **Re-initialisation info line:** `events.info(err, "Z Stage", "Move Absolute: Z Reference resetted")`, where `err` is the
  re-initialisation result (code 0 when it worked).
- **`Z Do Reference`:** a label starting with `#` (lenient decoding of an unknown code) runs FNL with a WARNING. Any other label
  except `"FRF"` runs FNL silently.
- **Zero polling:** `Z Wait End Move` with negative times uses 0 ms, and a 0 ms poll loops without sleeping, as LabVIEW does.
  Tests with a fake clock must advance time themselves.
- **With `implicit_z_reinit` = FALSE,** a failed reference check also makes `Z Move Absolute` test 0.0 against the range
  (08 §13.4 sketch). Error 9405 still replaces the error.
- `check_axis_reference` holds the controller lock for the whole check and re-initialisation.
- The Z handlers read `state.z` once per message and use that copy throughout.

### Simulator (`SimStage`)

- Controller state (position, reference, servo, velocity, joystick, macros, pending error) survives `Z Cleanup` and a later
  `Z Setup`.
- Only the last controller error is kept; `ERR?` reads and clears it.
- `MVR` adds to the last commanded target, and numbers sent pass through the 4-decimal format.
- FNL, FPL and FRF take `stage_reference_s`. Meanwhile the axis is unreferenced and `#7` reports busy.
- The starting velocity is 10 mm/s.
- **Test hooks:**
  - `frames` (GCS frames sent, capped at 4096) and `clear_frames()`;
  - `trigger_home()` simulates digital input 1, so a running `DH` starts `HOME` (joystick off, move to 0, then `DH` again);
  - `power_cycle()` loses the reference, useful for testing the implicit re-initialisation;
  - read-only `referenced`, `running_macro`, `velocity`, `joystick_enabled` and `port`.
- **Other public names:**
  - `zstage/sim.py`: `INJECTABLE_METHODS` and the owner's macro texts `DH_MACRO_TEXT` / `HOME_MACRO_TEXT`;
  - `zstage/backend.py`: `format_gcs_float` and `GCS_FLOAT_FORMAT`;
  - `ZController.options`, `controller.KNOWN_MACROS`, `controller.DH_MACRO`;
  - `handlers.NOT_REFERENCED_TEXT` and `REFERENCE_RESET_INFO`.

### For phase 2 (real `pipython` backend)

- **New module `zstage/pi_backend.py`:** a `StageBackend` on `PISerial` + `GCSCommands`, with `errcheck = False` and
  `floatformat = '.4f'`. Each method maps onto a `pipython` call (08 §13.2):
  - `stp(axis)` sends `"STP 1"`; `qerr` reads `"ERR?"`, and an empty answer gives 100 "No answer on ERR?";
  - `is_moving` uses `IsMoving()`; `is_controller_ready` uses `IsControllerReady()`, where `ValueError` gives 100
    "#7 Unknown reply"; `stop_all` uses `StopAll(noraise=True)`;
  - `translate_error` uses `GCSError.translate_error` with the trailing code stripped;
  - `qidn` and `qrmc` strip the final LF;
  - `GCSError`, `OSError` and `serial.SerialException` are wrapped as `StageError` (−6 when opening, −7 on timeout).
- **Dependencies:** add `pyserial`, and `pipython` if used.
- **Purge:** on real hardware the `XXX` of the purge itself latches error 2, so `connect_rs232` would always return 2 and log the
  Z22 warning. Either read `ERR?` before `XXX`, or treat 2 as expected.
- **Ports:** `[stage] port_map` is already applied in `setup()` (for example `COM2` → `/dev/ttyUSB0`).
- **Still to check on the hardware** (08 §13.6, questionnaire Q35):
  - whether the C-663 accepts `STP 1`;
  - `MVR` semantics;
  - where `Z Get Serial` takes its serial number from;
  - `#7` while referencing;
  - whether a macro aborts on error and how `MAC START` chains macros.
