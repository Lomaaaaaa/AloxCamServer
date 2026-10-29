# Z stage (PI Mercury, GCS)

**Summary.** The Z axis is a PI stage driven by a PI controller of the "Mercury_GCS" family (GCS 2.0 syntax, most probably a
C-863) over RS-232. The LabVIEW server uses PI's LabVIEW GCS driver: ASCII commands terminated by LF, one axis (normally `"1"`),
positions in mm and speeds in mm/s. A dedicated consumer loop executes the 19 `Z …` client commands one at a time. `Z Setup` opens
the link, runs PI's password-protected initialisation VI (servo on, then possibly a reference move to the negative limit switch,
depending on hidden options, §4.3) and sets the "normal" speed. Moves (`MOV`, `MVR`) are acknowledged as soon as they are sent;
`Z Wait End Move` polls the motion status.
Before every move the server checks that the axis is referenced and silently re-initialises it if not. It also checks the target
against a software range (error 9405). Several commands restart a controller macro called `DH`. The autofocus and refocus jobs drive
the same controller directly from the camera loop. This chapter specifies the link, every GCS frame, every Z command, the error
handling, the LabVIEW defects, and a Python replacement based on `pipython`.

**Sources:** main VI `alox_camera_server.vi`: Z loop cases [main VI frames d220–d250, main VI PDF p.6–7, 9–10, 12–13, 15–16],
`[hardware.z]` reading [main VI frames d48–d58], start-up macro check [main VI frames d59–d60], routing [main VI frames d0, d65–d66];
`alox_z_check_axis_reference.vi` [Doc6 p.139–143]; typedefs `alox_typedef_ext_zstage_session.ctl` [Doc6 p.644–647],
`alox_typedef_ext_zstage_properties.ctl` [Doc6 p.652–655], `typedef_ext_z_set_speed_data.ctl` [Doc6 p.277–280],
`typedef_ext_z_getserial_data.ctl` [Doc6 p.281–284], `typedef_ext_z_get_speed_data.ctl` [Doc6 p.293–296],
`typedef_ext_z_move_relative_data.ctl` [Doc6 p.325–328], `typedef_ext_z_move_wait_data.ctl` [Doc6 p.329–332],
`typedef_ext_z_stop_data.ctl` [Doc6 p.333–336], `typedef_ext_z_joystick_data.ctl` [Doc6 p.337–340],
`typedef_ext_z_move_absolute_data.ctl` [Doc6 p.447–450]; PI GCS LabVIEW driver: `Mercury_GCS_Configuration_Setup.vi` [Doc6 p.111–115],
`PI Open Interface of one system.vi` [Doc6 p.513–523], `PI Send String.vi` [Doc6 p.870–877], `PI Receive String.vi` [Doc6 p.696–705],
`PI VISA Receive Characters.vi` [Doc6 p.691–695], `Build command substring.vi` [Doc6 p.747–750], `Build query command substring.vi`
[Doc6 p.673–677], `Return space.vi` [Doc6 p.743–746], `Commanded axes connected?.vi` [Doc6 p.739–742], `Assign values from string to
axes.vi` [Doc6 p.678–685], `Assign booleans from string to axes.vi` [Doc6 p.127–133], `*IDN?` [Doc6 p.532–535], `SAI?` [Doc6 p.314–321],
`CST?` [Doc6 p.302–305], `SVO` [Doc6 p.306–309], `SVO?` [Doc6 p.240–243], `RON`/`RON?` [Doc6 p.228–235], `TRS?` [Doc6 p.216–219],
`LIM?` [Doc6 p.5–8], `TMN?` [Doc6 p.1–4], `TMX?` [Doc6 p.22–25], `FNL` [Doc6 p.18–21], `FPL` [Doc6 p.26–29], `FRF` [Doc6 p.236–239],
`FRF?` [Doc6 p.224–227], `MOV` [Doc6 p.878–882], `MVR` [Doc6 p.1212–1217], `POS?` [Doc6 p.668–672], `VEL` [Doc6 p.716–721],
`VEL?` [Doc6 p.710–715], `STP` [Doc6 p.1208–1211], `#24` [Doc6 p.467–470], `#5` [Doc6 p.118–121], `#7` [Doc6 p.30–33],
`JAX` [Doc6 p.1200–1203], `JON` [Doc6 p.1204–1207], `JON?` [Doc6 p.664–667], `RMC?` [Doc6 p.405–408], `MAC START` [Doc6 p.413–417],
`ERR?` [Doc6 p.706–709], `GCSTranslateError.vi` [Doc6 p.148–154], `General wait for movement to stop.vi` [Doc6 p.159–203],
`Wait for axes to stop.vi` [Doc6 p.210–215], `Wait for controller ready.vi` [Doc6 p.9–13], `Close connection if open.vi`
[Doc6 p.1191–1195]; ALOX helpers `Enqueue Send … Response.vi` [Doc6 p.435–466], `Log Server Error.vi` [Doc6 p.430–434],
`Log Server Infos.vi` [Doc6 p.1218–1221], `Clear Errors.vi` [Doc6 p.57–60]. The `pipython` statements were checked against the
pipython 2.13.0.2 source.

Related chapters: [01-architecture.md](01-architecture.md) (loops and queues), [02-tcp-protocol.md](02-tcp-protocol.md) (framing,
replies), [03-command-reference.md](03-command-reference.md), [04-data-types.md](04-data-types.md) (payload flattening),
[06-autofocus-refocus.md](06-autofocus-refocus.md) (direct stage use by the focus jobs),
[09-configuration-logging-errors.md](09-configuration-logging-errors.md) (INI reader, logs, error clusters),
[10-migration-plan.md](10-migration-plan.md) (compatibility switches).

---

## 1. Overview

### 1.1 Stack

| Layer | LabVIEW implementation | Evidence |
|---|---|---|
| Client request | 19 `Z …` values of the `Camera Command` enum. The TCP loop routes the range `"Z Setup".."Z Do Reference"` to the Z queue. | [main VI frames d0, d65–d66] |
| Queue | `"Z Stage Mgr"`, 5 elements, element type `Camera Queue Data`; blocking enqueue (timeout −1) | [main VI frame d0] |
| Consumer | "Z STAGE CONSUMER LOOP": `Dequeue` (timeout −1), then a Case on `Camera Command` with `Default`, 19 commands and `Exit` | [main VI frames d220–d250] |
| ALOX sub-VI | `alox_z_check_axis_reference.vi` (reference check and re-initialisation before moves) | [Doc6 p.139–143] |
| Vendor driver | PI GCS LabVIEW driver for Mercury, `C:\Users\Public\PI\Mercury\GCS_LabVIEW\…`, set-up program "V6.14.0" | [Doc6 p.111–114] |
| Transport | RS-232 through NI-VISA. The driver's RS232 branch uses VISA [Doc6 p.521]. That the password-protected set-up VI maps its `RS232` interface to this branch is **(inferred)**; the DLL branch would send the same frames. | [Doc6 p.513–523] |
| Controller | "Mercury_GCS" family, GCS 2.0, LF terminator [Doc6 p.516]. The driver's USB and daisy-chain search dialogs filter this family on the string `C-863` [Doc6 p.474, p.487], so a C-863 is most likely **(inferred)**. The exact model and stage are **(unknown)** (questionnaire Q9). | |

### 1.2 Data flow

```text
client ─TCP─▶ TCP loop ──(cmd in "Z Setup".."Z Do Reference")──▶ queue "Z Stage Mgr" (5)
                                                                        │
                    Z STAGE CONSUMER LOOP ◀─────────────────────────────┘
                    shift register: Z Session      local variable read per message: Z Stage Settings
                    case Camera Command ─▶ PI driver VIs ─▶ VISA serial ─▶ Mercury controller ─▶ stage
                         │
                         ├─▶ Enqueue Send <ACK|STR|BOOL|DBL> Response ─▶ TCP queue "Camera Server" ─▶ client
                         ├─▶ Log Server Error ("Z Stage") ─▶ <vision>\logs\%y%m%d_z_events.log (errors only)
                         └─▶ Log Server Infos ("Z Stage") ─▶ <data>\logs\%y%m%d_z_events.log (re-initialisation notices, §6)

camera loop (autofocus / refocus steps) ──▶ PI driver VIs directly (VEL?, VEL, POS?, MOV, JON?) — no queue, no lock
```

### 1.3 LabVIEW and driver behaviours that matter here

| Behaviour | Meaning for the re-implementation |
|---|---|
| Error cluster `{status, code, source}` | Every VI takes `error in` and returns `error out`. Every PI driver VI does **nothing** when `error in.status` is TRUE. A chain of VIs therefore behaves like a chain of calls that stops at the first exception, and the first error is the one reported. |
| Unwired `error in` | Each Z case starts with an empty error (no `error in` is wired on the first node), so an error never leaks from one command to the next. |
| `Clear Errors.vi` | Discards the error. Used in the reference check and in the start-up macro check. |
| `Variant To Data` | Decodes `Command Data` into the expected typedef. On a type mismatch it returns the typedef **default value** plus an error. Whether that error is wired decides whether the command runs with defaults (§5.3). |
| `Index Array` on an empty array | Returns the type default (0.0, FALSE, ""). A failed query therefore yields 0.0 or FALSE next to the error. |
| Front-panel objects as shared memory | `Z Stage Settings`, `Z Session`, `Z Inited`, `Z Pos (mm)`, `Running macros`, `Macro running ?` are front-panel controls or indicators, read and written through local variables. In Python they become fields of a state object. |
| Shift register `Z Session` | Private copy of the session inside the Z loop, initialised from the `Z Session` control. The control itself (read by the focus jobs) is only updated by `Z Setup` and `Z Cleanup`. |
| PI "System no." | 1-based index into the driver's global tables (interface, VISA resource, connected axes). It is **not** an axis. ALOX feeds it from the INI key `zaxis_I32`. |
| `%.;%.4f` | LabVIEW format string: 4 decimals, the `%.;` prefix forces a `.` decimal separator (the PC uses a French locale). |

---

## 2. Configuration and state

### 2.1 `[hardware.z]` → `Z Stage Settings` (`alox_typedef_ext_zstage_properties.ctl`)

Read once at start-up by a For loop (N = 7) in the initialisation sequence [main VI frames d48–d58]. The cluster constant
`{1, 9600, 1, 10, 1, 0, 0}` that seeds the loop is fully overwritten. A key that is missing or cannot be read takes its default and
nothing is reported. The error wire is chained through `comport_I32` → `baudrate_I32` → `zaxis_I32` → `speed_DBL` → `slowspeed_DBL`
(not `zmin_DBL`, `zmax_DBL`), so a non-convertible value in one of these keys also forces the **later chained keys** to their
defaults. The section is never written back ([09-configuration-logging-errors.md](09-configuration-logging-errors.md) §5.4).

| # | Field | Type | INI key | Default | Meaning and use |
|---|---|---|---|---|---|
| 0 | `Com` | I32 | `comport_I32` | 2 | COM port number; `≤ 0` → COM1 [main VI frame d221] |
| 1 | `Baudrate` | U16 ring | `baudrate_I32` | 3 | **Index**: 0 → 9600, 1 → 19200, 2 → 38400, 3 → 115200, any other value (including a literal `115200`) → 9600 [main VI frames d51–d54] |
| 2 | `Axis` | I32 | `zaxis_I32` | 1 | PI driver **System no.** (not a GCS axis id) [main VI frame d221, Doc6 p.141] |
| 3 | `Speed (mm/s)` | DBL | `speed_DBL` | 10 | "Normal speed": `VEL` after each (re)initialisation; also used by the focus jobs |
| 4 | `Slow Speed (mm/s)` | DBL | `slowspeed_DBL` | 1 | Never used |
| 5 | `Min Position (mm)` | DBL | `zmin_DBL` | 0 | Lower bound of the software range (§7) |
| 6 | `Max Position (mm)` | DBL | `zmax_DBL` | 20 | Upper bound of the software range (§7) |

The field order is that of the positional unbundle (I32, U16, I32, DBL×4) [Doc6 p.141, main VI frame d221]. The `Baudrate` field is a
16-bit ring, which cannot hold 115200 (the reference-check VI front panel shows `<49664>`, i.e. 115200 mod 65536 [Doc6 p.140]). How
the ring encodes the rate is **(unknown)**; the real link evidently works. In Python, store the real baud rate as an int.

### 2.2 `Z Session` (`alox_typedef_ext_zstage_session.ctl`) [Doc6 p.645]

| Field | Type | Default | Filled by |
|---|---|---|---|
| `System` | I32 | 0 | `Z Setup` and the re-initialisation: `= Z Stage Settings.Axis` |
| `Axes` | 1-D string array | empty | `Conn. Axes` of the set-up VI = answer of `SAI?` (e.g. `["1"]`) |
| `Identity` | string | "" | `Identification` of the set-up VI = answer of `*IDN?` |
| `Serial Number` | string | "" | `SN out` of the set-up VI; its source in RS-232 mode is **(unknown)** (the diagram is password-protected) |

All PI calls use `System no. = System`, the axes array `Axes` and element 0 of each returned array.

### 2.3 Other Z state

| Object | Written by | Read by | Notes |
|---|---|---|---|
| `Z Inited` (Boolean) | `Z Setup` (= no error); `Z Move Absolute/Relative` (output of the reference check) | nobody | Not reset at start-up nor by `Z Cleanup` [main VI frames d2, d248] |
| `Z Pos (mm)` (DBL) | `Z Get Position` (read value, 0.0 on error); `Z Move Absolute/Relative` (commanded target, only if the send succeeded) | nobody | Display only; never a measured position after a move |
| `Running macros` (string), `Macro running ?` (Boolean) | start-up check (string only), `Z Get Status` | nobody | Display only |
| `error out` (cluster) | every Z case (after logging) | display | Last error of the Z loop |

### 2.4 Start-up macro check [main VI frames d59–d60]

During start-up (step 2, inner frame 4), before any loop runs:

```text
sys            = Z Session (front-panel control, saved default).System
running, err   = RMC?(sys)                         # error in not wired
Running macros = running
if running == "DH" or running == "HOME":           # exact, case-sensitive string compare
    err = MAC START(sys, "DH", err)
Clear Errors(err)
```

No PI link has been opened at that point (nothing earlier in the start-up sequence calls the PI driver), so in the built executable
`RMC?` most probably fails and nothing happens apart from `Running macros` being set (probably to `""`); in the LabVIEW IDE a link
left open by a previous run could be reused **(inferred)**. The condition is the reverse of the one used by `Z Get Status`, `Z Stop`
and `Z Cleanup` (§10).

---

## 3. Controller link

### 3.1 Serial parameters

| Parameter | Value | Evidence |
|---|---|---|
| Port | `COM<n>`, `n = Com`, or 1 when `Com ≤ 0`. ALOX passes `n − 1` to the driver, whose RS232 branch opens `"COM" + (index + 1)`. | [main VI frame d221], [Doc6 p.521] |
| Baud rate | 9600 / 19200 / 38400 / 115200 from the INI index; default 115200 | [main VI frames d51–d54] |
| Frame | 8 data bits, no parity, 1 stop bit **(inferred)**: these are the defaults of `PI Open Interface of one system.vi`; what the password-protected set-up VI passes is not visible | [Doc6 p.517] |
| Flow control | none ("Mercury_GCS … RS232: Input and output HW handshake must be FALSE") | [Doc6 p.516] |
| Terminator | LF on commands; GCS multi-line answer convention (§3.3) | [Doc6 p.516, p.872, p.698] |
| I/O timeout | 5000 ms (VISA timeout, wired by ALOX) | [main VI frame d221] |
| Text encoding | single-byte; `#7` answers 0xB0/0xB1. Use cp1252 or latin-1 (pipython uses cp1252). | [Doc6 p.32] |

### 3.2 Opening the link (`PI Open Interface of one system.vi`, RS232 branch) [Doc6 p.517–521]

1. Store the interface settings in the driver globals at index `System no. − 1`.
2. `VISA Close` the resource previously stored for this system number, wait 50 ms, `VISA Open COM<n>`, configure the port (§3.1),
   set a 4096-byte I/O buffer (which buffer the mask selects is **(inferred)**), read and discard the bytes already waiting.
3. Purge sequence [Doc6 p.518]: send `XXX\n` (no command; the controller presumably sets error 2 "Unknown command" **(inferred)**),
   wait 10 ms, send `*idn?\n` and read the answer, send `err?\n` and read the answer, which is discarded. PI's own documentation: "The
   interface and error status of the chosen system are cleared by this VI, which sends XXX (no command), *IDN? and ERR?"
   [Doc6 p.517].

Consequences: a controller error left pending by an earlier command (for example error 10 after `Z Stop`) is **silently discarded** by
`Z Setup` and by the automatic re-initialisation (§6) **(inferred)**. Because step 2 closes the resource stored for the same system
number, a second `Z Setup` most probably re-opens the port cleanly even though ALOX never closes it first **(inferred: the set-up VI
is protected, so its use of this RS232 branch is itself inferred)**.

### 3.3 Frame rules

| Rule | Detail | Evidence |
|---|---|---|
| Send | `PI Send String` appends LF to text commands. The single-byte commands `#5` (0x05), `#7` (0x07), `#24` (0x18) are sent **without** LF. | [Doc6 p.872], [Doc6 p.120, p.32, p.469] |
| Receive | `PI Receive String` reads chunks of up to 255 bytes until the last line does **not** end with "space + LF" (GCS: every line of a multi-line answer except the last ends with `" \n"`). It removes the final LF and nothing else (spaces are kept). | [Doc6 p.696–705] |
| Numbers | `Build command substring`: for each axis `" " + axis + " " + format("%.;%.<n>f", value)`. `n` = `No. of digits`: 4 for `MOV`, `MVR`, `VEL`; 0 for `SVO` and `JON` modes. The space between axis and value is added because the syntax is GCS 2.0 (`Return space.vi`). | [Doc6 p.747–750, p.743–746, p.718, p.880, p.1214, p.306–309, p.1206] |
| Axis lists | `Build query command substring`: `" " + axes joined by spaces`, e.g. `POS? 1`. | [Doc6 p.673–677] |
| Answer parsing | `Assign values/booleans from string to axes`: split lines, remove spaces, split `axis=value`, assign by axis id. Empty answer → error 100 "Assign values: Input string empty or connected axes not defined". | [Doc6 p.678–685, p.127–133] |
| Pre-checks | Empty axis list → error 100 with a per-VI text, e.g. `"MOV No axis specified"`, `"MVR no axis specified"`, `"VEL No axis or velocity specified"`, `"STP no axis specified"`, `"JON No joystick specified"`, `"MAC START no macro name specified"`; an axis that is not in the connected-axes list (driver global filled by `SAI?` at set-up) → error 100 `"Commanded axis not connected: …"`. Nothing is sent in either case. | [Doc6 p.881, p.1215, p.719, p.1210, p.1206, p.415, p.741] |

The exact spacing produced by the substring VIs cannot be verified character by character on the printouts; the frames below follow
the GCS 2.0 syntax, which the controller accepts.

### 3.4 GCS frame catalogue (axis `"1"`)

| Driver VI | Frame sent | Answer | Driver post-processing | Used by |
|---|---|---|---|---|
| `MOV` | `MOV 1 12.5000\n` | none | no `ERR?` | `Z Move Absolute`, focus jobs |
| `MVR` | `MVR 1 -0.2500\n` | none | no `ERR?` | `Z Move Relative` |
| `VEL` | `VEL 1 10.0000\n` then `ERR?\n` | `0\n` | "Sets velocity and checks for error": non-zero code → error | `Z Setup`, `Z Set Speed`, re-init, focus jobs |
| `VEL?` | `VEL? 1\n` | `1=10.000000\n` | value per axis | `Z Get Speed`, focus jobs |
| `POS?` | `POS? 1\n` | `1=12.500000\n` | value per axis | `Z Get Position`, `Z Move Relative`, focus jobs |
| `FRF?` | `FRF? 1\n` | `1=1\n` | Boolean per axis | `Z Get Reference`, reference check |
| `FNL` / `FRF` / `FPL` | `FNL 1\n` / `FRF 1\n` / `FPL 1\n` | none | — | `Z Do Reference`, set-up |
| `STP` | `STP 1\n` (as wired, §5.3) | none | controller sets error 10 | `Z Stop` |
| `ERR?` | `ERR?\n` | `0\n` | empty → error 100 "No answer on ERR?"; code ≠ 0 → error {code, "Controller error" …} | `Z Get Status`, `Z Get Error`, `Z Do Reference`, set-up (purge and step 18), inside `VEL` and `JON` |
| `RMC?` | `RMC?\n` | running macro names, or an empty line | raw string, final LF removed | `Z Get Status`, `Z Stop`, `Z Cleanup`, start-up |
| `MAC START` | `MAC START DH\n` | none | empty name → error 100; no `ERR?` | same as `RMC?` |
| `JAX` | `JAX 1 1 1\n` (`JAX <joystick> <joystick axis> <axes>`) | none | — | joystick enable/disable |
| `JON` | `JON 1 1\n` / `JON 1 0\n` then `ERR?\n` | `0\n` | non-zero code → error | joystick enable/disable |
| `JON?` | `JON? 1\n` | `1=1\n` | integer per joystick | `Z Joystick Enabled ?`, refocus |
| `#5` | `0x05` (no LF) | hexadecimal bit mask + LF, bit *i* = connected axis *i* moving | `Hexadecimal String To Number` → Boolean array | waits (§8) |
| `#7` | `0x07` (no LF) | `0xB1` ready / `0xB0` busy | anything else → error 100 "#7 Unknown reply" | set-up wait **(inferred)** |
| set-up queries | `*IDN?`, `SAI?`, `CST? 1`, `SVO 1 1`, `SVO? 1`, `RON? 1`, `TRS? 1`, `LIM? 1`, `TMN? 1`, `TMX? 1` | per GCS | — | `Z Setup` (§4.2) |

### 3.5 Errors produced by the driver

| Code | Source text (pattern) | Cause | Evidence |
|---|---|---|---|
| controller code (> 0) | contains `"Controller error"`; `GCSTranslateError` appends `"GCS Error: <text>"` | `ERR?` returned a non-zero code (only where `ERR?` is called) | [Doc6 p.706–709, p.150] |
| 100 | `"<CMD> No axis specified"` (§3.3), `"Commanded axis not connected: …"`, `"No answer on ERR?"`, `"#7 Unknown reply"`, `"Assign values: …"` | driver checks | [Doc6 p.881, p.741, p.708, p.32, p.680] |
| −7 | `"Data Receive Timeout (RS-232, COM<n>, baud <b>)"` | VISA read timeout (5000 ms) | [Doc6 p.693] |
| −7 | `"Wait for axes to stop.vi: Timeout."` | wait longer than 300 s (§8) | [Doc6 p.212] |
| VISA status codes | `"Data Send Error (RS-232, COM<n>, baudrate <b>)"`, open errors with `"Errorcode: …"` | port missing, link closed | [Doc6 p.872, p.518] |

`MOV`, `MVR`, `FNL`, `FRF`, `STP`, `JAX` and `MAC START` do **not** query `ERR?`. A command refused by the controller therefore
returns no error to its caller. The controller keeps the error until the next `ERR?`, which may be sent by an unrelated later command
(`Z Get Status`, `Z Get Error`, `Z Set Speed`, `Z Enable/Disable Joystick`, `Z Do Reference`, a focus job's `VEL`), or discarded by
the purge of `Z Setup` / an automatic re-initialisation (§3.2). `ERR?` reads **and clears** the controller error ("STP sets error
code 10, call ERR?.vi to reset error after STP" [Doc6 p.1208]; purge description [Doc6 p.517]).

---

## 4. Initialisation and referencing

### 4.1 `Z Setup` [main VI frame d221, main VI PDF p.6]

Diagram comments: "#TODO: DEBUG WITH USB / Replace by alox.ini COM Settings", "Default speed/range settings", "Set Normal Speed".

```text
(Com, Baudrate, Axis, Speed, SlowSpeed, MinPos, MaxPos) = Z Stage Settings      # local variable
port = Com if Com > 0 else 1
ident, sn, conn_axes, err = Mercury_GCS_Configuration_Setup(
        System no. = Axis, Interface = "RS232", RS232 settings = {COM<port>, Baudrate},
        DaisyChain settings = {F, F, 1}, Reference Move = "Negative (FNL)", Timeout = 5000,
        All axes? = T, Switch servo on? = T)          # Stop refnum, RON settings, Axes to move, USB settings: not wired
Z Session = {System: Axis, Identity: ident, Serial Number: sn, Axes: conn_axes}   # bundled even on error
err = VEL(Axis, conn_axes, [Speed], err)            # "VEL 1 10.0000" + "ERR?"; skipped if err
reply ACK(err); log "Setup"; Z Inited = not err.status
Z Session control (local variable) = Z Session; shift register = Z Session
```

- A USB settings constant `{S/N "0018550449", Search F}` sits on the diagram but is **not wired** [main VI frame d221].
- Which of the two TRUE constants feeds `All axes?` and which feeds `Switch servo on?` is read from terminal positions
  **(inferred)**; both are TRUE, so it does not matter.
- The `Position` and `+/- movement range` outputs of the set-up VI are not used: the controller travel range (`TMN?`/`TMX?`) never
  reaches the software range of §7.
- The previous link is not closed by ALOX; the driver closes the VISA resource of the same system number (§3.2).

### 4.2 `Mercury_GCS_Configuration_Setup.vi` (PI, password-protected)

The diagram is not printed ("password-protected, meaning that you cannot see or modify the diagram" [Doc6 p.113]). Only its
documented steps are known [Doc6 p.112–113]. The GCS frames are derived from the step descriptions **(inferred)**.

| Step | Documented action | GCS frames (axis `1`) |
|---|---|---|
| 1 | `PI Open Interface of one system.vi` | `XXX\n`, `*idn?\n`, `err?\n` (§3.2) |
| 2 | `*IDN?.vi` → `Identification` | `*IDN?\n` |
| 3 | Define the system as `Mercury_GCS` (driver globals) | — |
| 4 | `Define connected axes.vi` (read from controller) → `Conn. Axes` | `SAI?\n` |
| 5 | `CST?.vi`: stage connected? If not, go to step 18 | `CST? 1\n` |
| 6–7 | `SVO.vi` if `Switch servo on?` (TRUE in ALOX), then `SVO?.vi` | `SVO 1 1\n`, `SVO? 1\n` |
| 8–9 | Set the reference mode if specified (`Reference mode` tab / `RON settings`, not wired by ALOX), then read whether it is OFF | (`RON 1 1\n`), `RON? 1\n` |
| 10–11 | If RON is on: reference switch? (`TRS?`). If yes: `FRF`, then `FRF?` | `TRS? 1\n`, `FRF 1\n`, `FRF? 1\n` |
| 12–13 | No reference switch but RON on: limit switch? (`LIM?`). If yes: `FPL` or `FNL` as chosen (ALOX: **Negative (FNL)**), then `FRF?` | `LIM? 1\n`, `FNL 1\n`, `FRF? 1\n` |
| 14 | Read the travel range | `TMN? 1\n`, `TMX? 1\n` |
| 15–16 | If `Move to middle?` and the axis was referenced by a reference move: `MOV` to the middle of the range; wait for motion to stop | (`MOV 1 <mid>\n`), `#5`/`#7` polling |
| 17 | `POS?.vi`. NaN is reported for an axis with RON off, failed referencing, or "designated not to be moved" | `POS? 1\n` |
| 18–19 | `ERR?.vi`, then `GCSTranslateError.vi` | `ERR?\n` |

The referencing commands say "Use #7 polling to determine end of this referencing procedure" [Doc6 p.18, p.236], and
`Wait for controller ready.vi` (`#7` every 50 ms by default [Doc6 p.9–11]) is part of the driver, so the set-up VI most likely
waits with `#7` **(inferred)**.

### 4.3 Hidden set-up options

`Check FRF?` and `Move to middle?` exist only on the set-up VI's front-panel tabs (they are not on its connector), and `RON settings`
is a connector input that ALOX leaves unwired [Doc6 p.111, p.114]. None of their default values is visible, and they decide whether
`Z Setup` moves the stage:

| Option | Documented meaning | Default |
|---|---|---|
| `Check FRF?` | FALSE: reference **always**; TRUE: reference only if `FRF?` returns FALSE [Doc6 p.112] | **(unknown)** |
| `Move to middle?` | after a successful reference move, `MOV` to the middle of the travel range [Doc6 p.113] | **(unknown)** |
| `RON settings` | reference mode to set before referencing | **(unknown)** |

Consequence: every `Z Setup`, and every automatic re-initialisation (§6), may drive the stage into its negative limit switch. Observe
the machine to settle it (questionnaire Q11; Q41 item 7 asks for the values shown on the set-up VI's front panel).

### 4.4 Referencing prerequisites (PI documentation)

| Command | Requirement or effect | Evidence |
|---|---|---|
| `FNL`, `FPL`, `FRF` | Servo must be on; reference mode must be "1" (`RON`) for a reference move | [Doc6 p.18, p.26, p.236] |
| `FNL` | Moves to the negative limit switch and sets the position to `VALUE_AT_REF_POS − DISTANCE_REF_TO_N_LIM`. If soft limits reduce the travel range, FNL/FPL "will provoke an error message" and only `FRF` works. | [Doc6 p.18] |
| `FRF` | Moves to the reference switch; use FNL/FPL for a stage without reference switch | [Doc6 p.236] |
| `FRF?` | TRUE once referenced by FRF/FNL/FPL (or by `POS` with RON off) | [Doc6 p.224] |
| `MOV`, `MVR` | Mercury_GCS: "This command works only in closed-loop operation. Motion commands like MOV [MVR] are not allowed when a joystick is active on the axis" (same statement in `JON.vi`) | [Doc6 p.879, p.1213, p.1204] |
| `MVR` | "Moves specified axes relative to current positions" / "relative to actual position" (PI help text). Whether the Mercury adds the distance to the measured position or to the last commanded target is **(unknown)**; it matters only while a move is running (§5.3) | [Doc6 p.1212, p.1214] |
| `VEL` | Unit mm/s; RAM only (lost at power-off unless saved with `WPA`); "The velocity should not be set to 0" | [Doc6 p.716] |
| `POS?` | "actual or target position, depending on system" | [Doc6 p.668] |

---

## 5. Z command reference

### 5.1 Common pattern

- **Routing.** The TCP loop sends every enum value of `"Z Setup".."Z Do Reference"` to the Z queue [main VI frame d0]. Either
  `Z Do Reference` is the last Z value of the enum (working assumption **(inferred)**), or the Z commands placed after it are never
  routed (a bug). The numeric values are **(unknown)** (questionnaire Q2, [04-data-types.md](04-data-types.md)). `Exit` reaches
  the Z loop from the camera loop ([01-architecture.md](01-architecture.md)).
- **End of each command case** (all 19) [main VI frames d221–d248]:

```text
Enqueue Send <ACK | STR | BOOL | DBL> Response(TCP queue name, Notifier Name, [value], err)
error out = Log Server Error(Server Type = "Z Stage", Extra String = <label>, Max Lines, Core #, err)
stop = False
```

- **Reply kinds.** `ACK` = `Enqueue Send Error Cluster Reponse.vi`, the reply is the error cluster alone. `STR`, `BOOL`, `DBL` = value +
  error cluster. The `Notifier Name` of the request is echoed. The wire format is in [02-tcp-protocol.md](02-tcp-protocol.md).
  **The ACK VI most probably sends nothing when the error is set**: its `error in` also feeds the `Enqueue Element`, which then does
  nothing [Doc6 p.437] **(inferred)**. Every failed ACK command below (9405, set-up failure, timeouts…) would then leave the client
  waiting until its own timeout. Value replies always carry their error.
- **Logging.** `Log Server Error` writes a line only when the error is set, to `<vision>\logs\%y%m%d_z_events.log` with the prefix
  `ERR (Z STAGE)-` and no command name; `Log Server Infos` (only after a silent re-initialisation, §6) appends to
  `<data>\logs\%y%m%d_z_events.log`. Formats, folders and the file-offset defect: [09](09-configuration-logging-errors.md) §7.

### 5.2 Summary

`S` = `Z Session.System`, `A` = `Z Session.Axes`. Frames for axis `"1"`.

| Command | `Command Data` (decode error) | GCS frames in order | Reply | Log label |
|---|---|---|---|---|
| `Z Setup` | not read | §4.2, then `VEL 1 <Speed>`, `ERR?` | ACK | `Setup` |
| `Z Get Serial` | `{Type: Boolean}`, default F (chained) | none | STR: `Type ? Identity : Serial Number` | `Get Serial` |
| `Z Get Status` | not read | `ERR?`, `RMC?`, [`MAC START DH`] | BOOL: no error from `ERR?` | `Get Status` |
| `Z Get Error` | not read | `ERR?` | STR: GCS text or "" | `Get Error` |
| `Z Set Speed` | `{Speed(mm/s): DBL}` (ignored) | `VEL 1 <v>`, `ERR?` | ACK | `Set Speed` |
| `Z Get Speed` | `{Extra Param: String}` unused (ignored) | `VEL? 1` | DBL (0.0 on error) | `Get Speed` |
| `Z Get Reference` | `{Extra Param}` unused (ignored) | `FRF? 1` | BOOL (F on error) | `Get Z Referenced` |
| `Z Do Reference` | bare enum, default `FNL` (ignored) | `FNL 1` or `FRF 1`, `#5` every 10 ms, `ERR?` | ACK | `Get Z Referenced` (sic) |
| `Z Get Position` | `{Extra Param}` unused (ignored) | `POS? 1` | DBL (0.0 on error) | `Get Position` |
| `Z Move Absolute` | `{Pos (mm): DBL}` (chained) | `FRF? 1` [+ re-init], [`MOV 1 <p>`] | ACK (9405 if out of range) | `Move Absolute` |
| `Z Move Relative` | `{Move (mm): DBL}` (ignored) | `FRF? 1` [+ re-init], `POS? 1`, [`MVR 1 <d>`] | ACK (9405) | `Move Relative` |
| `Z Wait End Move` | `{Add. Wait (ms)=0, Polling Cycle (ms)=10, Add. Wait Only=F}` (ignored) | `#5` every *Polling* ms | ACK (−7 after 300 s) | `Wait End Move` |
| `Z Stop` | `{Extra Param}` unused (ignored) | `STP 1`, `RMC?`, [`MAC START DH`] | ACK | `Stop` |
| `Z Enable Joystick` | `{Joystick ID=1, Joystick Axis=1}` (ignored) | `JAX 1 1 1`, `JON 1 1`, `ERR?` | ACK | `Enable Joystick` |
| `Z Disable Joystick` | same (ignored) | `JAX 1 1 1`, `JON 1 0`, `ERR?` | ACK | `Disable Joystick` |
| `Z Joystick Enabled ?` | same, only `Joystick ID` used (ignored) | `JON? 1` | BOOL: value ≠ 0 | `Is Joystick Enabled ?` |
| `Z Set Min Limit` | bare DBL (chained) | none | ACK | `Z Set Min Limit` |
| `Z Set Max Limit` | bare DBL (chained) | none | ACK | `Z Set Max Limit` |
| `Z Cleanup` | not read | close port, `RMC?` (fails **(inferred)**), [`MAC START DH`] | ACK | `Cleanup` |
| `Default` | — | none | **none** | none |
| `Exit` | — | none | none; the Z loop stops, the PI link stays open | none |

Payload typedefs: Set Speed [Doc6 p.278], Get Serial [Doc6 p.282] (Boolean shown as "Serial"), Move Relative [Doc6 p.326], Wait
[Doc6 p.330], Joystick [Doc6 p.338], Move Absolute [Doc6 p.448]. `Z Get Speed`, `Z Get Reference`, `Z Get Position` and `Z Stop`
decode a cluster with one unused string `Extra Param`; two typedefs have that layout ("Z Stage Get Speed Data" [Doc6 p.294], "Z Stage
Stop Data" [Doc6 p.334]) and which one each case uses is not visible, without consequence. The integer width of `Add. Wait (ms)`,
`Polling Cycle (ms)`, `Joystick ID` and `Joystick Axis` is **(unknown)** (I32 or U32; the PI VIs take U32 for the wait inputs and I32
for `JAX`/`JON`). "(ignored)" means the `Variant To Data` error is not wired: a payload of the wrong type silently runs the command
with the defaults listed. "(chained)" means the error is propagated.

### 5.3 Command details

Pseudo-code; every PI call skips itself when `err` is already set (§1.3). Unless stated, a case starts with `err = none`.

**`Z Get Serial`** [main VI frame d222]. No hardware access: returns the strings held by the Z loop's session, i.e. those of the last
`Z Setup` or automatic re-initialisation (empty before, and after `Z Cleanup`). `Type = TRUE` → `Identity` (the `*IDN?` answer), `FALSE` → `Serial Number`.

**`Z Get Status`** [main VI frames d223–d224]

```text
ctrl_err, err = ERR?(S)                    # reads AND clears the controller error
ok = False if err.status else not ctrl_err  # i.e. "ERR? returned 0"
running, err = RMC?(S, err); Running macros = running
Macro running ? = running == "DH" or running == "HOME"
if not Macro running ?: err = MAC START(S, "DH", err)
reply BOOL(ok, err); log "Get Status"
```

After any error from `ERR?` the macro part is skipped (`running` = ""). The first `Z Get Status` after a `Z Stop` returns FALSE with
error 10, unless another `ERR?` reader or a `Z Setup` came in between (§5.3 `Z Stop`).

**`Z Get Error`** [main VI frame d225]: `ERR?`, then `GCSTranslateError`; reply `STR(GCS Error? ? Error Description : "", err)`.
With a pending controller error the reply carries both the text and an error cluster (code = GCS code). Driver errors are translated
too when their code is in the table, e.g. −7 → "Timeout error", 100 → "PI LabVIEW driver reports error. See source control for
details." (§11). The exact `Error Description` string (message only, or the table line with its leading code) is **(unknown)**: the
diagram searches the table for the code and the extraction is not legible [Doc6 p.150].

**`Z Set Speed`** [main VI frame d226]: `VEL(S, A, [Speed])` (+ `ERR?`). `Z Stage Settings.Speed` is not changed, so the next
`Z Setup` or re-initialisation sets the INI speed again; a focus job scans at the INI speed and then restores the speed it read with
`VEL?` ([06](06-autofocus-refocus.md) §7.3). A bad payload sends `VEL 1 0.0000`.

**`Z Get Speed`**, **`Z Get Reference`**, **`Z Get Position`** [main VI frames d227, d228, d231]: one query, element 0 of the result
(0.0 / FALSE when the query failed). `Z Get Position` also writes `Z Pos (mm)`, even on error.

**`Z Do Reference`** [main VI frames d229–d230]

```text
method = Variant To Data(Command Data, enum "FNL")      # error ignored -> FNL
err = FRF(S, A) if method == "FRF" else FNL(S, A)        # cases: "FNL", Default | "FRF"
err = General wait for movement to stop(S, A, Polling cycle = 10 ms, err)   # §8, 300 s timeout
err = ERR?(S, err); err = GCSTranslateError(err)
reply ACK(err); log "Get Z Referenced"
```

No `SVO` or `RON` beforehand: it relies on the state left by `Z Setup`. Any enum value other than `FRF` (for example an `FPL`, if the
enum has one) runs `FNL`. The Z loop is blocked until the axis stops. A second, unwired `FNL` constant is a leftover.

**`Z Move Absolute`** [main VI frames d232–d235], comment "Check Z REF"

```text
session', Z_INITED, REF_WARNING, err = alox_z_check_axis_reference(Z Session, Z Stage Settings)   # §6
Z Inited (local) = Z_INITED; shift register = session'     # the Z Session control is NOT updated
if REF_WARNING: err = Log Server Infos("Z Stage", "Move Absolute: Z Reference resetted", err)
target, err = Variant To Data(Command Data, {Pos (mm)}, err)                 # error chained; with err already set the
                                                                             # node is skipped and target = 0.0 (inferred)
lo, hi = min(MinPos, MaxPos), max(MinPos, MaxPos)                            # LabVIEW "Max & Min"
if lo <= target <= hi:                                                       # inclusive
    err = MOV(S', A', [target], err)
    if not err: Z Pos (mm) = target
else:
    err = {TRUE, 9405, "Z move out of range"}                                # replaces any earlier error
reply ACK(err); log "Move Absolute"                                          # no wait for the end of the move
```

**`Z Move Relative`** [main VI frames d236–d239], comments "Check Z REF", "Z Position ?", "Inside range ?"

```text
(reference check and info log as above, text "Move Relative: Z Reference resetted")
p, err = POS?(S', A', err); current = p[0]           # 0.0 if POS? failed or was skipped
move = Variant To Data(Command Data, {Move (mm)})     # error ignored -> 0.0
target = current + move
if lo <= target <= hi:
    err = MVR(S', A', [move], err)                    # relative move by "move", not MOV to "target"
    if not err: Z Pos (mm) = target
else:
    err = {TRUE, 9405, "Z move out of range"}
reply ACK(err); log "Move Relative"
```

The range check uses the position returned by `POS?` ("actual or target position, depending on system" [Doc6 p.668]) plus the
distance. PI's `MVR.vi` help says the move is relative to the "current"/"actual" position [Doc6 p.1212, p.1214]; GCS 2.0 controllers
commonly add the distance to the last **commanded target** instead **(inferred)**. In that case, if a previous move is still
running, the final target can leave the checked range. Which semantics the Mercury uses is **(unknown)** (§15).

**`Z Wait End Move`** [main VI frame d240]: `General wait for movement to stop(S, A, Additional wait = Add. Wait (ms), Polling cycle
= Polling Cycle (ms), Add. wait only? = Add. Wait Only)`; reply ACK. Behaviour in §8.

**`Z Stop`** [main VI frames d241–d242]

```text
err = STP(S, Affected axes = A)            # All axes? F, Axis identifier? T (defaults) -> "STP 1"
running, err = RMC?(S, err)
if running not in ("DH", "HOME"): err = MAC START(S, "DH", err)     # STP presumably stopped DH (§10)
reply ACK(err); log "Stop"                 # Running macros / Macro running ? not updated
```

`STP` sets controller error 10 ("Controller was stopped by command"; "call ERR?.vi to reset error after STP" [Doc6 p.1208]). `Z Stop`
does not read it, so the next `ERR?` reports it (`Z Get Status` → FALSE + 10; `Z Set Speed` or a joystick command → error 10). PI
recommends `All axes? = TRUE, Axis identifier? = FALSE` (i.e. `STP` without argument) for "all other systems" [Doc6 p.1208]; whether
the Mercury firmware accepts `STP 1` is **(unknown)** (§14 Z7). The same help says that a referencing routine must be stopped with
`#24.vi` [Doc6 p.1208]; this never matters in ALOX because `Z Stop` cannot run while the Z loop is blocked in a reference move.

**`Z Enable Joystick`**, **`Z Disable Joystick`**, **`Z Joystick Enabled ?`**: §9.

**`Z Set Min Limit`** / **`Z Set Max Limit`** [main VI frames d246–d247]

```text
v, err = Variant To Data(Command Data, DBL 0)            # error chained
Z Stage Settings (local) = Bundle(Z Stage Settings, Min|Max Position (mm) = v)   # unconditional: 0.0 on decode error
reply ACK(err); log "Z Set Min Limit" | "Z Set Max Limit"
```

In memory only: no validation, no INI write, nothing sent to the controller. The new limit applies to the next message (§7).

**`Z Cleanup`** [main VI frames d248–d249]

```text
was_connected, err = Close connection if open(S)        # VISA Close
running, err = RMC?(S, err)                              # link just closed: fails (inferred)
if running not in ("DH", "HOME"): err = MAC START(S, "DH", err)
reply ACK(err); log "Cleanup"; error out = err
Z Session control = default {0, [], "", ""}; shift register = default    # Z Inited is NOT reset
```

**`Default`** [main VI frame d220]: nothing, **no reply**. **`Exit`** (visible case): stops the loop, no reply, no close of the
PI link. The Z queue is then destroyed ([01-architecture.md](01-architecture.md)).

---

## 6. Axis-reference check before moves (`alox_z_check_axis_reference.vi`) [Doc6 p.139–143]

Called by `Z Move Absolute` and `Z Move Relative` before anything else. Dated 18/12/2019.

| Input | Output |
|---|---|
| `Z Axis Session In` (Z Session), `Z Stage Settings`, `error in` (not wired by the main VI) | `Z Axis Session Out`, `Z INITED`, `REF WARNING?`, `error out` |

```text
ref[], e = FRF?(System, Axes, error in)           # "Z Ref OK ?"  -> "FRF? 1"
Clear Errors(e)                                   # incoming error and FRF? error both discarded
ref_lost = not ref[0]                             # empty array (failed query) -> FALSE -> ref_lost
REF WARNING? = ref_lost
if ref_lost:                                      # case "Z REF LOST, REINIT Z AXIS"
    Close connection if open(System); Clear Errors                       # "Close Z"
    same call to Mercury_GCS_Configuration_Setup as Z Setup (§4.1)       # "Reinit Z" (possible FNL, §4.3); only the
                                                                         # DaisyChain cluster differs (Device ID = Axis), unused in RS232
    Session Out = {System: Axis, Identity, Serial Number, Axes: Conn. Axes}
    e = VEL(Axis, Conn. Axes, [Speed], e)                                 # "Set Normal Speed"
    Z INITED = not e.status; error out = e
else:                                             # case labelled "XY REFS OK" (copied from an XY VI)
    Session Out = Session In; Z INITED = TRUE; error out = no error
```

Consequences:

- A **failed** `FRF?` (link never opened, cable unplugged, wrong system number) triggers the same full re-initialisation as a lost
  reference. A move sent before any `Z Setup` therefore performs the whole set-up first.
- The re-initialisation may run a reference move (§4.3) and blocks the Z loop for its duration.
- When it happens, `Log Server Infos` writes "Move Absolute: Z Reference resetted" or "Move Relative: Z Reference resetted" (always,
  even without error). The re-initialisation error, if any, is chained into the move: the move is skipped and the reply is that
  error, unless the target is out of range, in which case 9405 replaces it. For `Z Move Absolute` the range test then sees 0.0
  instead of the requested target, because the chained `Variant To Data` is skipped **(inferred)**.
- The re-initialisation also discards any controller error that was pending (purge, §3.2) **(inferred)**.
- The new session goes to the Z loop's shift register only; the `Z Session` control read by the focus jobs is not updated
  ([06-autofocus-refocus.md](06-autofocus-refocus.md) §7.4).

---

## 7. Travel limits

| Limit | Where | Behaviour |
|---|---|---|
| Software range `[min(Min, Max), max(Min, Max)]` | `Z Move Absolute` (target), `Z Move Relative` (`POS?` + move) | Inclusive bounds; checked with LabVIEW's `Max & Min` and `In Range and Coerce` [main VI frame d232]. Out of range → no frame sent, reply error **9405 "Z move out of range"**. Reversed limits are tolerated. A NaN target is out of range **(inferred)**. |
| `Z Set Min Limit` / `Z Set Max Limit` | §5.3 | Change the in-memory settings only; lost at restart (the INI is never written) |
| Controller soft limits / travel range (`TMN?`/`TMX?`, parameters 0x30/0x15) | controller | Read by the set-up VI but never compared with the software range. A target outside the controller range is refused by the controller (GCS error 7, "Position out of limits") and, since `MOV` does not read `ERR?`, reported only by a later `ERR?` **(inferred)**. |
| Autofocus range | focus jobs | Not checked against the software range ([06-autofocus-refocus.md](06-autofocus-refocus.md) §7.4) |

An earlier analysis read the two "beaker" icons as floor/ceil rounding of the limits; the native image shows the `Max & Min`
primitive, so there is **no rounding** [main VI frame d232].

---

## 8. Waiting for the end of a move

`General wait for movement to stop.vi` [Doc6 p.159–161, Mercury_GCS case p.180] with `Wait for axes to stop.vi` [Doc6 p.210–212]:

```text
if error in: return                              # nothing at all
if not Add. wait only?:                          # Mercury_GCS branch
    t0 = now
    loop:
        moving[] = #5(System)                    # 0x05 -> hex mask -> one Boolean per connected axis
        if error: break (error out = error)
        if no axis of "Axes to wait for" is moving: break
        if Stop refnum / Local stop: break, Stopped = TRUE, warning code 0 "Warning: Wait for axes to stop was stopped."
        if now - t0 > Timeout (300 s, not wired -> default): error -7 "Wait for axes to stop.vi: Timeout."; break
        Wait Until Next ms Multiple(Polling cycle)
if not Stopped: Wait(Additional wait time, ms)   # Wait has no error input: also runs after a timeout or a polling error
```

| Item | Value |
|---|---|
| Axes waited for | `All axes?` keeps its default TRUE, so all connected axes are waited for (one axis in practice) |
| Polling period | `Polling Cycle (ms)` of the command, default 10 ms; 10 ms fixed in `Z Do Reference` [main VI frame d229] |
| Timeout | 300 s, fixed (not exposed to the client). PI's help says `General wait for movement to stop.vi` "does not time out" [Doc6 p.159], but its Mercury_GCS branch leaves the `Timeout (s)` input of `Wait for axes to stop.vi` unwired, so that VI's default 300 s applies [Doc6 p.180, p.211–212] |
| Input types | `Additional wait time, ms` and `Polling cycle time, ms` are U32 in the PI VI [Doc6 p.161]; a negative client value would be coerced **(inferred)** |
| `Stopped` | The additional wait runs only when `Stopped` is FALSE [Doc6 p.161]. `Stopped` is set from the stop reference / local stop, i.e. only when the wait is aborted **(inferred)** [Doc6 p.212]; ALOX never wires a stop reference, so the additional wait always runs |
| Status-bit polling (`STA?`) | disabled for Mercury_GCS (`With status bit polling? = F`) [Doc6 p.180, p.210] |
| Load | at 10 ms, up to ~100 `#5` exchanges per second on the serial line |

`Z Move Absolute/Relative` never wait. A client that needs the axis to have arrived must send `Z Wait End Move`. The focus jobs do
not use this VI; they poll `POS?` ([06-autofocus-refocus.md](06-autofocus-refocus.md) §7).

---

## 9. Joystick [main VI frames d243–d245]

Diagram comment: "Joystick ID = 1 (First Joystick) / Joystick Axis = 1 (First Axis of System)".

| Command | Driver calls | Frames |
|---|---|---|
| `Z Enable Joystick` | `JAX(S, JoystickID, Joystick axis, Axes to set = A)`, then `JON(S, [JoystickID], [1])` | `JAX 1 1 1`, `JON 1 1`, `ERR?` |
| `Z Disable Joystick` | same, with mode 0 (`JAX` is sent again) | `JAX 1 1 1`, `JON 1 0`, `ERR?` |
| `Z Joystick Enabled ?` | `JON?(S, [JoystickID], All joysticks? = F)` → BOOL `status[0] ≠ 0` | `JON? 1` → `1=1` |

While a joystick is active, the controller refuses `MOV`/`MVR` (Mercury_GCS: "Motion commands like MOV or OSM (if supported) are not
allowed when a joystick is active on the axis" [Doc6 p.1204]; same statement in `MOV.vi`/`MVR.vi` [Doc6 p.879, p.1213]). The server
does not check it before a move; the refusal (GCS 51 "Selected axis is controlled by joystick" **(inferred code)**) surfaces at the
next `ERR?`. The refocus job checks `JON? 1` itself and fails with 9031
([06-autofocus-refocus.md](06-autofocus-refocus.md)).

---

## 10. Controller macros `DH` and `HOME`

| Place | Condition | Action | Evidence |
|---|---|---|---|
| Start-up (step 2, frame 4) | `RMC?` = `"DH"` or `"HOME"` | `MAC START DH`, errors cleared | [main VI frames d59–d60] |
| `Z Get Status` | `RMC?` ≠ `"DH"` and ≠ `"HOME"` | `MAC START DH` | [main VI frames d223–d224] |
| `Z Stop` | same | `MAC START DH` | [main VI frames d241–d242] |
| `Z Cleanup` | same, after closing the port | `MAC START DH` (not reached in practice **(inferred)**: `RMC?` fails on the closed link) | [main VI frames d248–d249] |

- The macros are stored in the controller. Their content is **(unknown)**; read them with `MAC? DH` and `MAC? HOME` (pipython
  `qMAC('DH')`) before deciding anything (questionnaire Q10). `DH` may move the stage.
- The comparison is an exact string compare on the raw `RMC?` answer (final LF removed). Several running macros, or any other
  macro name, count as "not running" and `MAC START DH` is sent anyway. If another macro is running, the controller probably rejects
  it (GCS 1008 "Controller is (already) running a macro") and the error stays pending, because `MAC START` does not read `ERR?`
  **(inferred)**.
- `MAC START.vi` says "Mercury™: #8.vi is not supported" [Doc6 p.413]. Whether that note covers Mercury_GCS (the help lists
  "Mercury" and "Mercury_GCS" separately) is **(unknown)**; ALOX watches macros only with `RMC?`.
- `STP` stops running macros too according to pipython's `STP` documentation (generic for GCS 2.0), which is presumably why `Z Stop`
  restarts `DH` **(inferred)**.

---

## 11. Error translation and codes

`GCSTranslateError.vi` [Doc6 p.148–154]: only when `error in.status` is TRUE, look up `code` in a 404-line text table of
`"<code> <message>"` entries. The table holds controller codes 0…6005, interface/DLL codes −1…−63 and −1001…−1086, and 100 for the
LabVIEW driver. If found, `GCS Error? = TRUE`, `Error Description` = the message, and `"GCS Error: "` + the description is appended
to `source`. The exact description text (with or without the leading code) and the separator before `"GCS Error: "` are **(inferred)**;
the sketch of §13.3 uses the bare message after a line feed. The same table ships with pipython (`GCSError.translate_error(code)`, module
`pipython.pidevice.gcserror`); do not copy it into the project. pipython's table is a few entries larger (e.g. −64), which only
matters for codes LabVIEW would not translate.

Codes that the Z subsystem can produce or relay:

| Code | Meaning | Produced by |
|---|---|---|
| 9405 | "Z move out of range" (ALOX) | `Z Move Absolute/Relative` |
| 100, −7, NI-VISA status codes | driver checks, timeouts, port errors | driver (§3.5) |
| 2 | Unknown command | the `XXX` purge at open (cleared immediately) **(inferred)** |
| 5 | Unallowable move on unreferenced axis, or servo off | controller, read later by `ERR?` |
| 7 | Position out of limits | controller, read later |
| 10 | Controller was stopped by command | after `STP` |
| 51 | Selected axis is controlled by joystick | controller, after a move with joystick on **(inferred)** |
| 1008 | Controller is (already) running a macro | controller, after `MAC START` **(inferred)** |
| 9031, 9401 | refocus joystick / focus position timeout | focus jobs ([06](06-autofocus-refocus.md)) |

The project-wide error model (cluster, custom codes, logging) is in [09-configuration-logging-errors.md](09-configuration-logging-errors.md).

---

## 12. Concurrency and session ownership

- **Within the Z loop** commands are strictly sequential. `Z Do Reference`, `Z Wait End Move` and a move that triggers a
  re-initialisation block it; further Z commands pile up in the 5-element queue, after which the TCP loop blocks on `Enqueue`
  ([01-architecture.md](01-architecture.md)).
- **Focus jobs** (camera loop) call `VEL?`, `VEL`, `POS?`, `MOV`, `JON?` directly with the `Z Session` **control**, in parallel with
  the Z loop, without any lock. `PI Send String` and `PI Receive String` are separate calls, so a query from one loop and a command
  from the other can interleave on the wire and swap answers **(inferred)**.
- **Stale session.** After an automatic re-initialisation only the shift register is updated; after `Z Cleanup` the control holds
  defaults (System 0, no axes), so every axis-based focus call fails with error 100 "… No axis specified", and the link is closed
  anyway.
- **Python rule:** one `PiStage` object owns the port and the session. Every request/response exchange, and every pair that must stay
  together (`VEL` + `ERR?`, `JON` + `ERR?`), runs under its lock. The focus jobs use the same object and its live session.

---

## 13. Python replacement

### 13.1 Library choice

| Option | Pros | Cons | Verdict |
|---|---|---|---|
| `pipython` + `PISerial` gateway (pyserial, no PI DLL) | Same transport as the LabVIEW VISA path; cross-platform; error table, GCS parsing, `#5`/`#7` helpers included | Its defaults differ from LabVIEW (`ERR?` after every command, float format, `STP` without axis) and must be set explicitly | **Recommended** |
| `pipython` + PI GCS2 DLL (`GCSDevice('C-863.11').ConnectRS232(port, baud)`) | Also gives USB and daisy chain | Needs the PI software installation | Only if USB is required |
| Raw `pyserial` (§13.5) | No vendor dependency; exact frames | Parsing and error texts to write by hand | Fallback, simulator, tests |

### 13.2 LabVIEW driver VI → pipython

| LabVIEW | pipython (`dev` = `GCSCommands` or `GCSDevice`) | Fidelity note |
|---|---|---|
| `PI Open Interface` (RS232) | `PISerial(port, baudrate)` + `settimeout(5000)`, `GCSCommands(GCSMessages(gw))` | pipython sends `CSV?` + `ERR?` when building the command object (falling back to `*IDN?` if `CSV?` fails) and `*IDN?` once more on the first `qSAI()` (device-name check); PISerial default timeout is 7000 ms |
| `Close connection if open` | `gateway.close()` | |
| error check after each VI | `dev.errcheck = False`, then an explicit `ERR?` (sketch `check_error`) where LabVIEW calls it | pipython's default (`True`) sends `ERR?` after every command, which would clear pending errors earlier than LabVIEW |
| `%.;%.4f` | `dev.floatformat = '.4f'` | pipython default `'.12g'`; applied to Python `float` arguments only (an `int` is sent as is), so pass floats |
| `ERR?.vi` | `dev.read('ERR?')` and parse it (sketch `check_error`) | `qERR()` raises `GCSError(-1004)` on an empty answer, where LabVIEW returns error 100 "No answer on ERR?" |
| `*IDN?`, `SAI?`, `CST?`, `SVO`, `SVO?`, `RON?`, `TRS?`, `LIM?`, `TMN?`, `TMX?` | `qIDN()`, `qSAI()`, `qCST(ax)`, `SVO(ax, True)`, `qSVO`, `qRON`, `qTRS`, `qLIM`, `qTMN`, `qTMX` | `qIDN()` keeps the final LF: strip it |
| `MOV`, `MVR`, `VEL`, `VEL?`, `POS?` | `MOV(ax, v)`, `MVR(ax, v)`, `VEL(ax, v)` + `ERR?`, `qVEL(ax)[ax]`, `qPOS(ax)[ax]` | |
| `FRF?`, `FRF`, `FNL`, `FPL` | `qFRF(ax)[ax]`, `FRF(ax)`, `FNL(ax)`, `FPL(ax)` | |
| `STP` (as wired: `STP 1`) | `dev.send('STP 1')` | `dev.STP()` sends `STP` without axis; `dev.HLT(ax)` halts smoothly |
| `#5`, `#7`, `#24` | `IsMoving()`, `IsControllerReady()`, `StopAll(noraise=True)` | `IsMoving()` sends `SAI? ALL` once (cached) to name the bits, an extra frame LabVIEW does not send; `IsControllerReady()` raises `ValueError` on an unknown reply (LabVIEW: 100 "#7 Unknown reply") |
| `RMC?`, `MAC START` | `qRMC()` (strip final LF), `MAC_START('DH')` | `qMAC('DH')` reads a macro's content |
| `JAX`, `JON`, `JON?` | `JAX(1, 1, ax)`, `JON(1, True)` + `ERR?`, `qJON(1)[1]` (bool) | |
| `GCSTranslateError` | `GCSError.translate_error(code)` → `"<text> <code>"` (just `"<code>"` when unknown) | strip the trailing code to get the message (LabVIEW's exact description format: §11) |
| `Mercury_GCS_Configuration_Setup` | `pitools.startup(dev, refmodes='FNL', servostates=True)` approximates it | `startup` starts with a stop-all and references only if `qFRF()` is FALSE; the explicit sequence of §13.3 is closer |
| `General wait for movement to stop` | `pitools.waitontarget(dev, axes, timeout=300, polldelay=0.01)` approximates it | different completion test (on-target instead of `#5`); the sketch polls `IsMoving()` like LabVIEW |

### 13.3 Reference sketch: the controller link (`zstage/pi_stage.py`)

Reference sketch. It reproduces the LabVIEW semantics. It was run with pipython 2.13.0.2 against a simulated single-axis controller
(a `PIGateway` stub) through the handlers of §13.4, not against hardware.

```python
# zstage/pi_stage.py -- reference sketch (LabVIEW-compatible semantics on pipython, no PI DLL)
from __future__ import annotations

import re
import threading
import time
from dataclasses import dataclass, field

from pipython import GCSError
from pipython.pidevice.gcscommands import GCSCommands
from pipython.pidevice.gcsmessages import GCSMessages
from pipython.pidevice.interfaces.piserial import PISerial

BAUD_BY_INDEX = {0: 9600, 1: 19200, 2: 38400, 3: 115200}   # [hardware.z] baudrate_I32, other values -> 9600
ERR_DRIVER = 100            # "PI LabVIEW driver reports error" (e.g. "<CMD> No axis specified")
ERR_TIMEOUT = -7            # "Timeout error"
ERR_NOT_CONNECTED = -4      # Python choice (LabVIEW returns an NI-VISA status code)
ERR_OPEN_PORT = -6          # Python choice (LabVIEW returns an NI-VISA status code)


@dataclass
class ZStageSettings:                       # alox_typedef_ext_zstage_properties.ctl
    com: int = 2
    baudrate: int = 115200                  # real baud rate, decoded from the INI index
    axis: int = 1                           # LabVIEW driver "System no." -- not a GCS axis identifier
    speed: float = 10.0                     # mm/s, "normal speed"
    slow_speed: float = 1.0                 # mm/s, never used
    min_pos: float = 0.0                    # mm, software range
    max_pos: float = 20.0                   # mm, software range

    @classmethod
    def from_ini(cls, read_int, read_float) -> "ZStageSettings":
        """read_int/read_float(section, key, default) follow NI 'Read Key' rules (see 09)."""
        s = "hardware.z"
        return cls(com=read_int(s, "comport_I32", 2),
                   baudrate=BAUD_BY_INDEX.get(read_int(s, "baudrate_I32", 3), 9600),
                   axis=read_int(s, "zaxis_I32", 1),
                   speed=read_float(s, "speed_DBL", 10.0),
                   slow_speed=read_float(s, "slowspeed_DBL", 1.0),
                   min_pos=read_float(s, "zmin_DBL", 0.0),
                   max_pos=read_float(s, "zmax_DBL", 20.0))

    @property
    def port(self) -> str:                  # Windows name; map to /dev/tty* on Linux via configuration
        return f"COM{self.com if self.com > 0 else 1}"


@dataclass
class ZSession:                             # alox_typedef_ext_zstage_session.ctl
    system: int = 0
    axes: list[str] = field(default_factory=list)
    identity: str = ""
    serial_number: str = ""


class StageError(Exception):
    """A LabVIEW error cluster with status TRUE (code + source)."""
    def __init__(self, code: int, source: str):
        super().__init__(f"{code}: {source}")
        self.code = code
        self.source = source


def strip_term(answer: str) -> str:
    """PI Receive String removes only the final line feed."""
    return answer[:-1] if answer.endswith("\n") else answer


def gcs_error_text(code: int) -> str:
    """Description used by GCSTranslateError ('' when the code is unknown)."""
    msg = GCSError.translate_error(code)            # "<text> <code>", or "<code>" when unknown
    suffix = f" {code}"
    return msg[:-len(suffix)] if msg.endswith(suffix) else ""


def translate(err: StageError) -> StageError:
    """GCSTranslateError.vi: append the GCS text to the error source."""
    text = gcs_error_text(err.code)
    return StageError(err.code, f"{err.source}\nGCS Error: {text}") if text else err


def serial_from_idn(identity: str) -> str:
    """(inferred) 3rd field of '*IDN?'; the LabVIEW source of 'SN out' is not visible."""
    parts = [p.strip() for p in identity.split(",")]
    return parts[2] if len(parts) > 2 else ""


class PiStage:
    """The single link to the Mercury controller. Every GCS exchange of the server
    (Z loop and autofocus/refocus jobs) goes through one instance and its lock."""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self._gateway: PISerial | None = None
        self.dev: GCSCommands | None = None
        self.session = ZSession()

    # ---- link -------------------------------------------------------------------------------
    def open(self, port: str, baudrate: int, timeout_ms: int = 5000) -> None:
        self.close()
        try:
            self._gateway = PISerial(port=port, baudrate=baudrate)   # pyserial defaults: 8N1, no handshake
            self._gateway.settimeout(timeout_ms)
            self.dev = GCSCommands(GCSMessages(self._gateway))       # pipython sends CSV? (+ ERR?) here
        except (OSError, ValueError, GCSError) as exc:              # serial.SerialException is an OSError
            self.close()
            raise StageError(ERR_OPEN_PORT, f"Open RS232 {port} @ {baudrate}: {exc}") from exc
        self.dev.errcheck = False        # read ERR? only where the LabVIEW driver VIs do
        self.dev.floatformat = ".4f"     # LabVIEW "%.;%.4f": 4 decimals, '.' separator

    def close(self) -> None:             # "Close connection if open"
        with self.lock:
            if self._gateway is not None:
                try:
                    self._gateway.close()
                except OSError:
                    pass
            self._gateway, self.dev = None, None

    def _call(self, fn_name: str, *args):
        """One pipython call under the lock; every failure becomes a StageError."""
        with self.lock:
            if self.dev is None:
                raise StageError(ERR_NOT_CONNECTED, f"{fn_name}: not connected")
            try:
                return getattr(self.dev, fn_name)(*args)
            except GCSError as exc:
                raise StageError(exc.val, str(exc)) from exc
            except (OSError, ValueError, KeyError) as exc:          # serial failure, unparsable answer
                raise StageError(ERR_DRIVER, f"{fn_name}: {exc}") from exc

    def _ax(self, cmd: str) -> str:
        """LabVIEW VIs refuse to send anything without an axis (error 100)."""
        if not self.session.axes:
            raise StageError(ERR_DRIVER, f"{cmd} No axis specified")
        return self.session.axes[0]

    # ---- one method per LabVIEW PI VI (frames shown for axis "1") ------------------------------
    def check_error(self) -> None:                       # ERR?.vi            "ERR?" -> "0"
        answer = strip_term(self._call("read", "ERR?"))  # errcheck is off: no second ERR?
        if not answer:
            raise StageError(ERR_DRIVER, "No answer on ERR?")
        m = re.match(r"\s*([+-]?\d+)", answer)           # LabVIEW 'Decimal String To Number'
        code = int(m.group(1)) if m else 0
        if code:
            raise StageError(code, "Controller error")

    def mov(self, pos: float) -> None:                   # MOV.vi             "MOV 1 12.5000" (no ERR?)
        self._call("MOV", self._ax("MOV"), float(pos))   # float() so that floatformat '.4f' applies

    def mvr(self, dist: float) -> None:                  # MVR.vi             "MVR 1 -0.2500" (no ERR?)
        self._call("MVR", self._ax("MVR"), float(dist))

    def vel(self, speed: float) -> None:                 # VEL.vi             "VEL 1 10.0000" + "ERR?"
        with self.lock:
            self._call("VEL", self._ax("VEL"), float(speed))
            self.check_error()

    def qvel(self) -> float:                             # VEL?.vi            "VEL? 1" -> "1=10.000000"
        ax = self._ax("VEL?")
        return float(self._call("qVEL", ax)[ax])

    def qpos(self) -> float:                             # POS?.vi            "POS? 1" -> "1=12.500000"
        ax = self._ax("POS?")
        return float(self._call("qPOS", ax)[ax])

    def qfrf(self) -> bool:                              # FRF?.vi            "FRF? 1" -> "1=1"
        ax = self._ax("FRF?")
        return bool(self._call("qFRF", ax)[ax])

    def fnl(self) -> None:                               # FNL.vi             "FNL 1"
        self._call("FNL", self._ax("FNL"))

    def frf(self) -> None:                               # FRF.vi             "FRF 1"
        self._call("FRF", self._ax("FRF"))

    def stp(self) -> None:                               # STP.vi as wired    "STP 1" (pipython STP() sends "STP")
        self._call("send", f"STP {self._ax('STP')}")

    def running_macros(self) -> str:                     # RMC?.vi            "RMC?" -> "DH" or ""
        return strip_term(self._call("qRMC"))

    def mac_start(self, name: str) -> None:              # MAC START.vi       "MAC START DH" (no ERR?)
        self._call("MAC_START", name)

    def jax(self, joy_id: int, joy_axis: int) -> None:   # JAX.vi             "JAX 1 1 1"
        self._call("JAX", joy_id, joy_axis, self._ax("JAX"))

    def jon(self, joy_id: int, enable: bool) -> None:    # JON.vi             "JON 1 1" + "ERR?"
        with self.lock:
            self._call("JON", joy_id, enable)
            self.check_error()

    def qjon(self, joy_id: int) -> bool:                 # JON?.vi            "JON? 1" -> "1=1"
        return bool(self._call("qJON", joy_id)[joy_id])

    def is_moving(self) -> bool:                         # #5.vi              b"\x05" -> hex bit mask
        return any(self._call("IsMoving").values())

    def wait_ready(self, timeout_s: float = 300.0, poll_s: float = 0.05) -> None:   # #7 polling
        t0 = time.monotonic()
        while not self._call("IsControllerReady"):
            if time.monotonic() - t0 > timeout_s:
                raise StageError(ERR_TIMEOUT, "Wait for controller ready: Timeout.")
            time.sleep(poll_s)

    def general_wait(self, add_wait_ms: int = 0, polling_ms: int = 10,
                     add_wait_only: bool = False, timeout_s: float = 300.0) -> None:
        """General wait for movement to stop.vi -> Wait for axes to stop.vi (Mercury_GCS branch)."""
        try:
            if not add_wait_only:
                t0 = time.monotonic()
                while self.is_moving():
                    if time.monotonic() - t0 > timeout_s:
                        raise StageError(ERR_TIMEOUT, "Wait for axes to stop.vi: Timeout.")
                    time.sleep(max(0, polling_ms) / 1000.0)
        finally:
            time.sleep(max(0, add_wait_ms) / 1000.0)     # LabVIEW also waits after a failed poll

    # ---- Mercury_GCS_Configuration_Setup.vi (documented steps, [Doc6 p.112-113]) + "Set Normal Speed"
    def setup(self, s: ZStageSettings, *, reference_move: str = "FNL",
              always_reference: bool = False, ready_timeout_s: float = 300.0) -> None:
        with self.lock:                                  # the focus jobs must not talk to the controller meanwhile
            self.session = ZSession(system=s.axis)       # LabVIEW bundles the outputs even on failure
            self.open(s.port, s.baudrate, 5000)          # step 1 (LabVIEW: "XXX", "*idn?", "err?")
            self._call("read", "ERR?")                   # step 1: purge; the answer is discarded, as in LabVIEW
            identity = strip_term(self._call("qIDN"))    # step 2
            axes = [str(a) for a in self._call("qSAI")]  # step 4: connected axes, e.g. ["1"]
            self.session = ZSession(s.axis, axes, identity, serial_from_idn(identity))
            ax = self._ax("CST?")
            if str(self._call("qCST", ax)[ax]).upper() != "NOSTAGE":       # step 5 (inferred test)
                self._call("SVO", ax, True)                                 # step 6: "SVO 1 1"
                self._call("qSVO", ax)                                      # step 7
                if self._call("qRON", ax)[ax]:                              # step 9: reference mode ON?
                    if always_reference or not self._call("qFRF", ax)[ax]:  # hidden "Check FRF?" option
                        if self._call("qTRS", ax)[ax]:                      # step 10: reference switch?
                            self._call("FRF", ax)                           # step 11
                        elif self._call("qLIM", ax)[ax]:                    # step 12: limit switches?
                            self._call(reference_move, ax)                  # step 13: "FNL 1" in ALOX
                        self.wait_ready(ready_timeout_s)
                        self._call("qFRF", ax)
                self._call("qTMN", ax)                                      # step 14 (unused by ALOX)
                self._call("qTMX", ax)
                self._call("qPOS", ax)                                      # step 17
            try:
                self.check_error()                                          # step 18
            except StageError as exc:
                raise translate(exc) from exc                               # step 19: GCSTranslateError
            self.vel(s.speed)                                               # ALOX: "Set Normal Speed"

    # ---- alox_z_check_axis_reference.vi ---------------------------------------------------------
    def check_axis_reference(self, s: ZStageSettings) -> tuple[bool, bool, StageError | None]:
        """Returns (Z INITED, REF WARNING?, error). Incoming errors are discarded."""
        try:
            referenced = self.qfrf()
        except StageError:
            referenced = False                           # a failed FRF? counts as "reference lost"
        if referenced:
            return True, False, None
        self.close()                                     # "Close Z"; its errors are cleared too
        try:
            self.setup(s)                                # full re-initialisation (may run FNL!)
        except StageError as exc:
            return False, True, exc
        return True, True, None
```

Design choices to confirm: `always_reference=False` (reference only when `FRF?` is FALSE) is a safe default for the hidden
`Check FRF?` option, not an observed fact (§4.3); `Move to middle?` (steps 15–16) and `RON` (step 8) are not implemented because their
LabVIEW defaults are **(unknown)**. The `"NOSTAGE"` test for `CST?`, the `#7` wait after the reference move and its 300 s timeout are
**(inferred)**. Error codes −4 and −6 replace the NI-VISA status codes LabVIEW would return. `setup()` holds the lock for the whole
sequence, which also blocks the focus jobs for the duration of a reference move (LabVIEW does not, §12).

### 13.4 Reference sketch: the Z command handlers (`zstage/handlers.py`)

Reference sketch. The TCP layer decodes `Command Data` (passing the typedef default plus the decoding error when decoding fails),
calls `handle()`, encodes the `Reply`, and applies the project-wide `send_error_replies` / `reply_unknown_commands` switches
([02-tcp-protocol.md](02-tcp-protocol.md), [10-migration-plan.md](10-migration-plan.md) §6).

```python
# zstage/handlers.py -- reference sketch: the Z consumer loop cases [main VI frames d220-d249]
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .pi_stage import PiStage, StageError, ZSession, ZStageSettings, gcs_error_text, translate

ERR_OUT_OF_RANGE = 9405
ERR_UNREFERENCED = 5        # proposed code when implicit re-initialisation is disabled (GCS 5)


@dataclass
class Reply:
    kind: str | None              # "ACK" (error cluster only), "STR", "BOOL", "DBL"; None = no reply
    value: object = None
    error: StageError | None = None


@dataclass
class ZCompat:                    # [compat] switches, defaults = LabVIEW behaviour
    restart_dh_macro: bool = True
    implicit_z_reinit: bool = True


LOG_LABELS = {
    "Z Setup": "Setup", "Z Get Serial": "Get Serial", "Z Get Status": "Get Status",
    "Z Get Error": "Get Error", "Z Set Speed": "Set Speed", "Z Get Speed": "Get Speed",
    "Z Get Reference": "Get Z Referenced", "Z Do Reference": "Get Z Referenced",
    "Z Get Position": "Get Position", "Z Move Absolute": "Move Absolute",
    "Z Move Relative": "Move Relative", "Z Wait End Move": "Wait End Move", "Z Stop": "Stop",
    "Z Enable Joystick": "Enable Joystick", "Z Disable Joystick": "Disable Joystick",
    "Z Joystick Enabled ?": "Is Joystick Enabled ?", "Z Set Min Limit": "Z Set Min Limit",
    "Z Set Max Limit": "Z Set Max Limit", "Z Cleanup": "Cleanup",
}


class ZHandlers:
    """One method per case of the LabVIEW Z loop. 'data' is the decoded Command Data, or the
    typedef default when decoding failed; 'derr' is that decoding error (or None)."""

    def __init__(self, stage: PiStage, settings: ZStageSettings,
                 log_error: Callable[[str, StageError | None], None],
                 log_info: Callable[[str], None], compat: ZCompat | None = None):
        self.stage, self.settings = stage, settings
        self.log_error, self.log_info = log_error, log_info
        self.compat = compat or ZCompat()
        # diagnostics that LabVIEW keeps in front-panel indicators
        self.z_inited = False
        self.z_pos = 0.0
        self.running_macros = ""
        self.macro_running = False

    def handle(self, cmd: str, data: object = None, derr: StageError | None = None) -> Reply:
        fn = getattr(self, "_" + cmd.lower().replace(" ", "_").replace("?", "q"), None)
        if fn is None or cmd not in LOG_LABELS:
            return Reply(None)                           # 'Default' case: no reply, no log
        reply = fn(data, derr)
        self.log_error(LOG_LABELS[cmd], reply.error)     # Log Server Error writes only errors
        return reply

    # ---- helpers ------------------------------------------------------------------------------
    def _restart_dh(self) -> str:
        running = self.stage.running_macros()
        if running not in ("DH", "HOME"):
            self.stage.mac_start("DH")
        return running

    def _check_reference(self, label: str) -> StageError | None:
        if not self.compat.implicit_z_reinit:            # proposed fix: refuse instead of re-initialising
            try:
                if self.stage.qfrf():
                    return None
            except StageError as exc:
                return exc
            return StageError(ERR_UNREFERENCED, "Z axis not referenced")
        self.z_inited, warning, err = self.stage.check_axis_reference(self.settings)
        if warning:
            self.log_info(f"{label}: Z Reference resetted")
        return err

    def _range_then_send(self, target: float, err: StageError | None, send: Callable[[], None]) -> Reply:
        lo = min(self.settings.min_pos, self.settings.max_pos)      # LabVIEW 'Max & Min'
        hi = max(self.settings.min_pos, self.settings.max_pos)
        if not lo <= target <= hi:                                  # inclusive bounds; NaN -> out of range
            return Reply("ACK", error=StageError(ERR_OUT_OF_RANGE, "Z move out of range"))
        if err is None:
            try:
                send()
                self.z_pos = target
            except StageError as exc:
                err = exc
        return Reply("ACK", error=err)

    def _ack(self, *steps: Callable[[], object]) -> Reply:
        """Run steps in order; the first failure stops the chain (LabVIEW error-in semantics)."""
        try:
            for step in steps:
                step()
            return Reply("ACK")
        except StageError as exc:
            return Reply("ACK", error=exc)

    def _query(self, kind: str, fn: Callable[[], object], default: object) -> Reply:
        try:
            return Reply(kind, fn())
        except StageError as exc:
            return Reply(kind, default, exc)               # Index Array of an empty array

    # ---- cases ---------------------------------------------------------------------------------
    def _z_setup(self, _data, _derr) -> Reply:
        reply = self._ack(lambda: self.stage.setup(self.settings))
        self.z_inited = reply.error is None
        return reply

    def _z_get_serial(self, identity_wanted: bool, derr) -> Reply:   # data: 'Type' boolean, default False
        s = self.stage.session
        return Reply("STR", s.identity if identity_wanted else s.serial_number, derr)

    def _z_get_status(self, _data, _derr) -> Reply:
        ok, err, running = False, None, ""
        try:
            self.stage.check_error()                     # reads AND clears the controller error
            ok = True
            running = self.stage.running_macros()
            if self.compat.restart_dh_macro and running not in ("DH", "HOME"):
                self.stage.mac_start("DH")
        except StageError as exc:
            err = exc
        self.running_macros, self.macro_running = running, running in ("DH", "HOME")
        return Reply("BOOL", ok, err)

    def _z_get_error(self, _data, _derr) -> Reply:
        try:
            self.stage.check_error()
            return Reply("STR", "", None)
        except StageError as exc:
            return Reply("STR", gcs_error_text(exc.code), translate(exc))

    def _z_set_speed(self, speed: float, _derr) -> Reply:      # decoding error ignored (speed = 0.0)
        return self._ack(lambda: self.stage.vel(speed))

    def _z_get_speed(self, _data, _derr) -> Reply:
        return self._query("DBL", self.stage.qvel, 0.0)

    def _z_get_reference(self, _data, _derr) -> Reply:
        return self._query("BOOL", self.stage.qfrf, False)

    def _z_do_reference(self, method: str, _derr) -> Reply:   # enum label; decoding error -> "FNL"
        reply = self._ack(self.stage.frf if method == "FRF" else self.stage.fnl,
                          lambda: self.stage.general_wait(polling_ms=10), self.stage.check_error)
        return Reply("ACK", error=translate(reply.error)) if reply.error else reply

    def _z_get_position(self, _data, _derr) -> Reply:
        reply = self._query("DBL", self.stage.qpos, 0.0)
        self.z_pos = reply.value                         # written even on error
        return reply

    def _z_move_absolute(self, target: float, derr) -> Reply:   # decoding error is chained
        err = self._check_reference("Move Absolute")
        if err is not None:
            target = 0.0                                 # chained Variant To Data skipped (inferred)
        else:
            err = derr                                   # target is already the typedef default 0.0 if derr
        return self._range_then_send(target, err, lambda: self.stage.mov(target))

    def _z_move_relative(self, move: float, _derr) -> Reply:    # decoding error ignored
        err = self._check_reference("Move Relative")
        current = 0.0
        if err is None:
            try:
                current = self.stage.qpos()
            except StageError as exc:
                err = exc
        return self._range_then_send(current + move, err, lambda: self.stage.mvr(move))

    def _z_wait_end_move(self, w: tuple[int, int, bool], _derr) -> Reply:   # (Add. Wait, Polling, Only)
        return self._ack(lambda: self.stage.general_wait(add_wait_ms=w[0], polling_ms=w[1], add_wait_only=w[2]))

    def _z_stop(self, _data, _derr) -> Reply:            # STP leaves controller error 10 pending
        return self._ack(self.stage.stp, self._restart_dh if self.compat.restart_dh_macro else (lambda: None))

    def _z_enable_joystick(self, j: tuple[int, int], _derr) -> Reply:     # (Joystick ID, Joystick Axis)
        return self._ack(lambda: self.stage.jax(j[0], j[1]), lambda: self.stage.jon(j[0], True))

    def _z_disable_joystick(self, j: tuple[int, int], _derr) -> Reply:
        return self._ack(lambda: self.stage.jax(j[0], j[1]), lambda: self.stage.jon(j[0], False))

    def _z_joystick_enabled_q(self, j: tuple[int, int], _derr) -> Reply:
        return self._query("BOOL", lambda: self.stage.qjon(j[0]), False)

    def _z_set_min_limit(self, value: float, derr) -> Reply:   # applied even if decoding failed (0.0)
        self.settings.min_pos = value
        return Reply("ACK", error=derr)

    def _z_set_max_limit(self, value: float, derr) -> Reply:
        self.settings.max_pos = value
        return Reply("ACK", error=derr)

    def _z_cleanup(self, _data, _derr) -> Reply:
        self.stage.close()
        reply = self._ack(self._restart_dh) if self.compat.restart_dh_macro else Reply("ACK")  # fails: link closed
        self.stage.session = ZSession()                  # z_inited is NOT reset (as in LabVIEW)
        return reply
```

Notes on the sketch:

- The Z thread calls `handle()` for each dequeued message, in order. `Exit` returns `Reply(None)` here; the loop itself stops on it.
- The start-up macro check (§2.4) needs no code in practice, since the link is not open at start-up **(inferred)**. If it is ever
  wanted, it is `running_macros()` followed by `mac_start("DH")` when the answer is `"DH"` or `"HOME"`, with all errors ignored.
- The sketch keeps **one** live session in `PiStage` (fix Z18): it does not reproduce the stale `Z Session` copy that the LabVIEW focus
  jobs read after an automatic re-initialisation.
- Payload values arrive decoded: `bool` (Get Serial), `float` (speed, positions, limits), enum label `str` (Do Reference),
  `(add_wait_ms, polling_ms, add_wait_only)` and `(joystick_id, joystick_axis)` tuples.
- The `implicit_z_reinit = False` branch returns GCS code 5 for an unreferenced axis; that code is a proposal, not LabVIEW behaviour
  (registered in [09-configuration-logging-errors.md](09-configuration-logging-errors.md) §10.5).

### 13.5 Reference sketch: raw transport without pipython

Reference sketch, for a simulator, for protocol tests, or if `pipython` is not wanted.

```python
import serial

class GcsSerial:
    """Raw GCS 2.0 transport (same rules as PI Send/Receive String)."""
    def __init__(self, port: str, baudrate: int, timeout_s: float = 5.0):
        self.ser = serial.Serial(port, baudrate, bytesize=8, parity="N", stopbits=1,
                                 xonxoff=False, rtscts=False, dsrdtr=False, timeout=timeout_s)
        self.ser.reset_input_buffer()

    def send(self, cmd: str) -> None:              # text command: LF appended
        self.ser.write((cmd + "\n").encode("cp1252"))

    def send_char(self, code: int) -> None:        # #5, #7, #24: one byte, no LF
        self.ser.write(bytes([code]))

    def receive(self) -> str:                      # until a line that does not end with " \n"
        buf = b""
        while True:
            line = self.ser.read_until(b"\n")
            if not line.endswith(b"\n"):
                raise TimeoutError("Data Receive Timeout")      # LabVIEW error -7
            buf += line
            if not line.endswith(b" \n"):
                return buf[:-1].decode("cp1252")             # final LF removed, nothing else

    def query(self, cmd: str) -> str:
        self.send(cmd)
        return self.receive()

    def axis_values(self, answer: str) -> dict[str, str]:      # "1=12.5 \n2=3.0" -> {"1": "12.5", ...}
        return {k.strip(): v.strip() for k, v in
                (ln.split("=", 1) for ln in answer.splitlines() if "=" in ln)}

    def is_moving(self) -> bool:                   # #5: hexadecimal bit mask, bit 0 = first axis
        self.send_char(5)
        return int(self.receive().strip(), 16) != 0

    def is_ready(self) -> bool:                    # #7: 0xB1 ready, 0xB0 busy
        self.send_char(7)
        return self.receive() == "\xb1"
```

Example: `query("POS? 1")` → `"1=12.500000"`; `send(f"MOV 1 {12.5:.4f}")`; `query("ERR?")` → `"0"`.

### 13.6 Hardware validation checklist

Run each item against the LabVIEW server and the Python server on the real machine, and compare the frames (serial sniffer or
pipython's `logfile`), the replies and the stage behaviour. Items 1, 2, 5, 6 and 4 are also asked of the owner as questionnaire Q9,
Q11 and Q35.

1. `*IDN?` answer, `SAI?` answer, baud rate, port (Q9); content of `MAC? DH` and `MAC? HOME` (Q10).
2. `Z Setup` twice in a row: does the stage move to the negative limit each time (Q11)?
3. `Z Move Absolute` inside, on, and outside the software range (9405: reply or silence?).
4. `Z Move Relative` while a previous move is still running.
5. `Z Stop` during a move: is `STP 1` accepted, does the stage stop, what do the next `Z Get Status` and `Z Get Error` return?
6. Move with the joystick enabled; then `Z Get Error`.
7. `Z Do Reference` with `FNL` and `FRF`; timing of the reply versus the end of referencing.
8. Power-cycle the controller, then `Z Move Absolute` (implicit re-initialisation and info log).
9. `Z Cleanup`, then `Z Get Position` and a focus job (expected failures).
10. `Z Stop`, then `Z Setup`, then `Z Get Status`: is error 10 discarded by the set-up purge (Z22)?

---

## 14. Defects and oddities in the LabVIEW implementation

| # | Observation | Evidence | Impact | Recommendation |
|---|---|---|---|---|
| Z1 | ACK replies are probably not sent when the error is set (9405, set-up failure, timeouts, controller errors). | [Doc6 p.437], §5.1 | The client only sees its own timeout | Follow the project switch `send_error_replies` (questionnaire Q17, [02](02-tcp-protocol.md)). |
| Z2 | Before each move, a failed or FALSE `FRF?` closes the link and re-runs the full set-up, possibly with a reference move. Errors are cleared and only an info line is logged. | [Doc6 p.141] | Unexpected motion and multi-second blocking | **Reproduce** by default (switch `implicit_z_reinit`, Q24). The fix branch refuses the move instead. |
| Z3 | The re-initialisation runs before the range check: an out-of-range move can still trigger a reference move, then reply 9405. | [main VI frames d232, d236] | Motion for a rejected request | **Ask**. For `Z Move Absolute` the range can be checked first at no cost. |
| Z4 | `MOV`/`MVR` do not read `ERR?`: a move refused by the controller (unreferenced, joystick on, outside controller limits) is acknowledged OK. The error appears at the next `ERR?` of an unrelated command. | [Doc6 p.881, p.1215] | Silent failed moves; misattributed errors | **Reproduce** (client-visible). Ask whether the client calls `Z Get Error`/`Z Get Status` after moves (§15 item 7, questionnaire Q32). |
| Z5 | `Z Move Relative` checks `POS?` + move, then sends `MVR`. If the controller adds the distance to the last commanded target **(inferred, GCS 2.0)**, the check is wrong while a move is running. | [main VI frame d237], [Doc6 p.1212] | Range check possibly bypassed while a move is running | **Ask** (questionnaire Q35; §15 item 9). Option: check against the commanded target (`MOV?`) or refuse while `#5` reports motion. |
| Z6 | `STP` leaves controller error 10 pending: the next `ERR?` reader (`Z Get Status` → FALSE with error 10, `Z Set Speed`/joystick command → error 10) reports it, unless a set-up purge discards it first (Z22). | [Doc6 p.1208], [main VI frame d241] | Spurious errors after a stop | **Reproduce** (client-visible). |
| Z7 | `STP 1` is sent with an axis argument, while PI's VI documentation recommends `STP` without argument for these systems. Acceptance by the Mercury firmware is unknown. | [Doc6 p.1208–1209] | Possibly a stop that does not stop (safety) | **Verify on hardware first** (questionnaire Q35). If refused, send `STP` (pipython `STP()`) or `HLT 1`. |
| Z8 | `DH` macro restarted by `Z Get Status`, `Z Stop`, `Z Cleanup` when neither `DH` nor `HOME` runs; reverse logic at start-up; exact string compare. Content unknown. | §10 | Unknown controller actions, possibly motion; a status query has a side effect | **Ask** (Q10, switch `restart_dh_macro`). Read the macros first. |
| Z9 | `Z Cleanup` queries `RMC?` on the link it has just closed (error, so probably no reply at all, Z1) **(inferred)**, and does not reset `Z Inited`. | [main VI frame d248] | Cleanup always "fails" | **Reproduce** by default (tied to `restart_dh_macro`, as in the sketch); with the switch off, send nothing after the close and reply success. Reset the internal "initialised" flag (not client-visible). |
| Z10 | Most commands ignore payload decoding errors and run with typedef defaults: `VEL 1 0.0000` (PI: "should not be set to 0"), `FNL`, move by 0, joystick 1/1. `Z Set Min/Max Limit` overwrites the limit with 0.0 and replies with the error. | §5.2, [Doc6 p.716] | Wrong actions on malformed requests | **Fix**: reply the decoding error without acting. |
| Z11 | A Z enum value without a case gets no reply (`Default`). | [main VI frame d220] | Client timeout | Project switch `reply_unknown_commands`. |
| Z12 | `Z Set Min/Max Limit`: no validation, not persisted, controller soft limits unchanged; the INI range is restored at restart. | [main VI frames d246–d247] | Limits silently lost on restart | **Reproduce**; ask whether persistence is wanted. |
| Z13 | `Z Do Reference` detects the end of referencing with `#5` (motion), whereas PI says to poll `#7` for `FNL`/`FRF`. Every enum value except `FRF` runs `FNL`. | [Doc6 p.18, p.236], [main VI frame d229] | Possible early reply while referencing finishes **(inferred)** | **Fix**: wait with `#7` (then `#5`), keep `FNL` as default; verify timing on hardware. |
| Z14 | `Z Get Status` reads and clears the controller error and may start a macro. | §5.3 | A status poll changes state | **Reproduce**; document for client authors. |
| Z15 | Copy-paste labels: `Z Do Reference` logs "Get Z Referenced"; the reference-check VI has a case "XY REFS OK"; info text "Z Reference resetted". | [main VI frame d229], [Doc6 p.142] | Cosmetic | **Reproduce** log texts if logs are parsed by tools; otherwise free. |
| Z16 | Unused USB constant (S/N `0018550449`) and "#TODO: DEBUG WITH USB" comment; `Slow Speed` never used; `Z Setup` relies on the driver to close the previous port. | [main VI frame d221] | None | Do not port; close and reopen explicitly in Python. |
| Z17 | `Baudrate` is a 16-bit ring that cannot hold 115200. | [Doc6 p.140] | Encoding unknown | Store the real rate as an int. |
| Z18 | The focus jobs share the controller without a lock and use the `Z Session` control, stale after a re-initialisation and empty after `Z Cleanup`. | §12 | Interleaved frames, wrong session | **Fix**: one `PiStage` with a lock and a live session ([06](06-autofocus-refocus.md) D20). |
| Z19 | Wait: 300 s timeout not configurable; 10 ms polling (~100 `#5`/s); the additional wait also runs after a timeout. | §8 | Long blocking; serial load | **Reproduce** the defaults; make the timeout configurable. |
| Z20 | `Z Pos (mm)` holds the commanded target or the last read value (0.0 after a failed read), never the measured position after a move. | §2.3 | Misleading display | Diagnostics only; no client impact. |
| Z21 | The source of `Serial Number` (`SN out`) in RS-232 mode is unknown; it may be empty. | §2.2 | `Z Get Serial(Type = F)` content unknown | **Ask** (questionnaire Q35; capture the current reply); the sketch uses the 3rd `*IDN?` field. |
| Z22 | `Z Setup` and the automatic re-initialisation discard any pending controller error (purge `XXX`/`*idn?`/`err?` at open) **(inferred)**; a refused move or the error 10 of `Z Stop` then never reaches the client. | [Doc6 p.517–518], §3.2 | Lost error reports | **Reproduce** (client-visible); log the discarded code in Python. |

---

## 15. Open questions

Question numbers "Qn" in this chapter refer to the owner questionnaire [open-questions.md](open-questions.md).

1. **Enum values** (questionnaire Q2, [04-data-types.md](04-data-types.md)): numeric codes of the 19 Z commands in `Camera Command`,
   confirmation that they all lie in `"Z Setup".."Z Do Reference"`, and the items and representation of the `Z Do Reference` enum
   (does it contain `FPL`?). Without them the Z payloads cannot be decoded.
2. **Integer widths** (questionnaire Q33) of `Add. Wait (ms)`, `Polling Cycle (ms)`, `Joystick ID`, `Joystick Axis` (I32 or U32):
   needed to decode `Z Wait End Move` and the joystick commands.
3. **Controller and stage** (Q9): `*IDN?` reply, firmware, real COM port and baud rate, stage model, reference switch or limit switches
   only. This decides `FRF` versus `FNL` and confirms the C-863 assumption.
4. **Hidden set-up options** (Q11; Q41 for the option values on the PI set-up VI's front panel; Q35 test 6): does `Z Setup`
   reference every time, or only when `FRF?` is FALSE? Does it move to the middle of the range? Observable on the machine.
5. **Macros `DH`/`HOME`** (Q10): content, and whether the automatic restart must be kept.
6. **`STP 1`** (questionnaire Q35): does the controller stop with this frame, or reject it (error 1 or 15)? Must be tested before
   relying on `Z Stop`.
7. **Error replies and error polling** (Q17, Q32): does the client ever receive ACK errors today, and does it call `Z Get Error` or
   `Z Get Status` after moves to detect refused moves? This decides whether the new server may check `ERR?` after `MOV`/`MVR`.
8. **`Z Get Serial`** (questionnaire Q35): what does it return today with `Type = FALSE` (the `SN out` source is invisible)?
9. **`POS?` and `MVR` semantics** (questionnaire Q35) on this controller: does `POS?` return the actual or the target position, and
   does `MVR` add the distance to the measured position or to the last commanded target? Affects the `Z Move Relative` range check
   (Z5) and the focus settle logic ([06-autofocus-refocus.md](06-autofocus-refocus.md) open question 8). Testable on the machine
   (checklist item 4).
