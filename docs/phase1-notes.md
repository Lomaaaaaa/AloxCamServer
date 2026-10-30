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
