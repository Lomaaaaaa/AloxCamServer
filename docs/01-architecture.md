# System overview and architecture of the main VI

**Summary.** The ALOX Camera Server (`alox_camera_server.vi`) is a top-level LabVIEW 2018 VI that runs as an executable next to its
client (presumably the LabVIEW "ALOX Vision Core"). It accepts one TCP connection on the loopback interface and decodes binary
commands (flattened LabVIEW clusters). It dispatches them to two hardware workers: a **camera consumer loop** that drives the High Mag
and Low Mag cameras through NI-IMAQdx (including LED/bulb control, autofocus and refocus), and a **Z consumer loop** that drives a PI
Mercury Z stage over RS-232. Replies go back on the same socket. The VI is a five-stage sequence: hide the window, reset indicators,
load `alox.ini` (plus a PI macro check), run three parallel loops linked by three bounded named queues, and quit. This chapter
specifies that structure: start-up and shutdown, the queues, the loops and their persistent state, the response path, the
front-panel objects used as shared memory, the concurrency hazards, and a Python thread design. Wire formats, the command catalogue
and the subsystems are in the sibling chapters.

**Sources.** `alox_camera_server.vi` (main VI, modified 25/07/2025): connector pane and front panel [main VI p.1–2]; stage 3 as
displayed [main VI frame d0; p.3–5]; stages 0, 1, 2, 4 [main VI frames d1, d2, d3, d251; p.30, 32, 34, 44]; stage-2 inner sequence
[main VI frames d4–d61; p.34–43]; TCP loop hidden states [main VI frames d62–d72; p.5–6, 8–9]; camera loop `Default`, `Exit`,
timeout and Disable frames [main VI frames d73, d78, d215–d219; p.8, 29]; Z loop `Default` and timeout frames [main VI frames d220,
d250]; Disable structure under the sequence [main VI frames d252, d253]. Sub-VIs and typedefs: `alox_camera_server_globals.vi`
[Doc6 p.418–421]; `HW TCP Server Queue.ctl` [Doc6 p.1040–1043]; `HW TCP Server Command.ctl` [Doc6 p.1044–1047];
`typedef_ext_camera_mgmt_queue_data.ctl` [Doc6 p.285–288]; `typedef_ext_camera_camera_command.ctl` [Doc6 p.422–425];
`HW TCP Response Data.ctl` [Doc6 p.1159–1162]; `Enqueue Send Error Cluster Reponse.vi` [Doc6 p.435–438] and the other
`Enqueue Send <type> Response.vi` [Doc6 p.443–446, 455–458, 463–466, 559–562, 571–574, 1124–1127]; `HW TCP Extract TCP Data.vi`
[Doc6 p.1150–1154]; `HW TCP Send Heartbeat.vi` [Doc6 p.1155–1158]; `HW TCP Send Response.vi` [Doc6 p.1167–1170]; NI `TCP Listen.vi`
[Doc6 p.1146–1149] and its listener cache `Internecine Avoider.vi` [Doc6 p.1141–1145]; `tools_application_executable_folder.vi`
[Doc6 p.87–90]; `tools_windows_user_documents_folder.vi` [Doc6 p.409–412]; NI `config.llb` `Open Config Data` / `Read Key` /
`Close Config Data` [Doc6 p.265–268, 1247–1291, 1226–1229]; PI `RMC?.vi` [Doc6 p.405–408] and `MAC START.vi` [Doc6 p.413–417];
`Log Server Error.vi` [Doc6 p.430–434].

Related chapters: [02-tcp-protocol.md](02-tcp-protocol.md) (framing, heartbeat, reply encoding),
[03-command-reference.md](03-command-reference.md) (every command), [04-data-types.md](04-data-types.md) (cluster layouts, enums,
flattening), [05-camera-acquisition.md](05-camera-acquisition.md), [06-autofocus-refocus.md](06-autofocus-refocus.md),
[07-focus-metrics.md](07-focus-metrics.md), [08-z-stage.md](08-z-stage.md),
[09-configuration-logging-errors.md](09-configuration-logging-errors.md), [10-migration-plan.md](10-migration-plan.md).

---

## 1. Conventions and LabVIEW primer

### 1.1 Conventions

- Untagged statements are observed on the printouts. **(inferred)** marks a deduction from LabVIEW/NI semantics or from
  cross-checking. **(unknown)** marks something the printouts do not show.
- `[main VI frame dN]` is hidden frame N of the HTML export of `alox_camera_server.vi` (`d0` is the diagram as displayed, i.e.
  stage 3). `[main VI p.N]` is a page of the main VI PDF. `[Doc6 p.N]` is a page of "Labview Document 6".
- **HM** = the "High Mag." camera, **LM** = the "Low Mag." camera. Some wire labels use an older, inverted naming ("Low Angle" = HM,
  "High Angle" = LM); this document uses only HM/LM. Names in `code font` are exact LabVIEW labels, enum items or strings.

### 1.2 LabVIEW constructs used by the main VI

See also the primer of [05-camera-acquisition.md](05-camera-acquisition.md) §1.2 (error clusters, `Variant To Data`, coercions).

| Construct | What it does here | What the re-implementation must keep |
|---|---|---|
| **Stacked sequence** `[0..4]` | Five frames executed strictly in order; a frame starts only when every node of the previous one has finished. There are no "sequence locals" [main VI frames d1–d3, d251]: frames exchange data only through front-panel objects. | Start-up steps in order. Stage 4 runs only after the three loops and the three `Release Queue` calls have finished. |
| **While loop + shift register** | A shift register carries a value from one iteration to the next. With nothing wired to its left terminal it is *uninitialised*: it starts from the type default ("Not a Refnum" here) and, in the development environment, keeps its last value between runs. | Loop-local variables. The TCP loop's connection register is always overwritten by the first `Listen`, so its initial value has no effect (inferred). |
| **Named queue** | `Obtain Queue` with the same name returns a reference to the same FIFO anywhere in the application; sub-VIs reach the TCP queue this way, without a wire. `max queue size` bounds it; `Enqueue`/`Dequeue` with timeout −1 wait forever; `Dequeue` reports `timed out? = TRUE` when its timeout expires. | Bounded or unbounded FIFOs with explicit blocking rules (§10). |
| **`Release Queue(force destroy = TRUE)`** | Destroys the queue whatever other references exist. Every waiting or later `Enqueue`/`Dequeue` on it returns an error (code 1122 for a waiter, 1 afterwards, inferred). | It is the stop signal of the TCP loop (§7). |
| **Front-panel object + local variable** | Every indicator or control stores a value; a local variable reads or writes it from anywhere in the diagram, without locking. The server uses panel objects as its global memory. | One shared state object with a lock (§8, §10). |
| **Case on an enum, with ranges** | A case label such as `"Z Setup".."Z Do Reference"` is a range of **numeric** enum values, so routing depends on the item order. The `Camera Command` representation is presumably U16 **(unknown)**; only item names are printed. | Routing tables built from the real enum order ([04-data-types.md](04-data-types.md)). |
| **Variant** | A value plus its type descriptor. `State Data` (internal) and `Command Data` (on the wire) are variants. | Typed decoding per command. |
| **Error cluster** `{status, code, source}` | Most nodes whose `error in.status` is TRUE do nothing and pass the error on (close and release functions still run, inferred); an unwired `error out` means the error is ignored; `Clear Errors` discards it. | Exceptions with explicit, code-specific suppression ([09](09-configuration-logging-errors.md)). |
| **"Use Default If Unwired" tunnel** | A case that does not wire an output tunnel outputs the type default (FALSE for a boolean). The TCP loop's stop flag relies on it (only `Quit` wires TRUE); the camera and Z cases wire explicit constants. | Explicit defaults. |
| **Diagram Disable structure** | Only the `Enabled` frame executes. | Ignore disabled code. |
| **`Tick Count (ms)`** | Free-running U32 millisecond counter (wraps after about 49.7 days). Every protocol timestamp uses it. | A U32 millisecond counter (§6.2, Q5). |
| **Non-reentrant sub-VI** | The LabVIEW default: one instance in memory, so two loops calling it are serialised and share its uninitialised shift registers. Reentrancy of the ALOX VIs is **(unknown)**: VI properties are not printed. | Per-caller state where LabVIEW shared it by accident (§9.3). |

---

## 2. What the server is

### 2.1 Identity

| Item | Value |
|---|---|
| VI | `D:\ALOX\Vision Core\sources\extcamera\alox_camera_server.vi`, title "Alox Camera Server", icon "CAM SERVER" [main VI p.1] |
| Connector pane | No terminal [main VI p.1]: a top-level VI, built into an executable (inferred) |
| VI description and properties | Not printed **(unknown)**: reentrancy, "Automatic error handling", window options |
| Role | TCP server for one client; executes camera and Z-stage commands; returns typed replies |
| Instances | Possibly one per "core" (inferred from `[general] core_I32` → `Core #`). `Core #` is wired to every `Log Server Error` call but feeds only disabled code there, so it has no effect ([09](09-configuration-logging-errors.md) D14). Several instances on one PC would each need their own `[hardware.camera] port_I32`. The deployment is **(unknown)** (Q7) |
| Hardware at start-up | Nothing is opened. The only hardware call is the PI macro check (§3.4.3), made with whatever `Z Session` holds. Cameras and stage are opened on demand by the client commands `Cam Setup` and `Z Setup` |

### 2.2 Context diagram

```
┌─────────────────────────────┐ TCP, 127.0.0.1:port_I32  ┌──────────────────────────┐
│ Client (presumably ALOX     │ ── Command frames ─────▶ │ Alox Camera Server       │
│ Vision Core, LabVIEW); one  │ ◀─ Response frames ───── │ alox_camera_server.vi    │
│ named notifier per call     │   (Notifier Name echoed) │ one per core (inferred)  │
└─────────────────────────────┘                          └───┬───────┬───────┬──────┘
                                                             │       │       │
     NI-IMAQdx (GigE / USB) ◀────────────────────────────────┘       │       │
     ├─ High Mag camera ┐ LED and bulb are driven through            │       │
     └─ Low Mag camera  ┘ camera digital outputs                     │       │
     PI GCS LabVIEW driver, RS-232 ◀─────────────────────────────────┘       │
     └─ PI Mercury controller + Z stage (Z loop, and AF/RF directly)         │
     Files ◀─────────────────────────────────────────────────────────────────┘
     ├─ read:    alox_setup.ini, alox.ini, flat-field, dark-current and simulation images
     └─ written: error and info logs, refocus PNG images
```

Interfaces are specified in: TCP framing and replies, [02-tcp-protocol.md](02-tcp-protocol.md); commands,
[03-command-reference.md](03-command-reference.md); cameras, LED and bulb, [05-camera-acquisition.md](05-camera-acquisition.md);
PI controller, [08-z-stage.md](08-z-stage.md); files and logs, [09-configuration-logging-errors.md](09-configuration-logging-errors.md).
The LabVIEW front panel (live image, focus graph, temperatures, histories) is a local display, invisible in the executable unless
the client sends `Show UI` (§3.2, §8).

---

## 3. The outer sequence: start-up and shutdown

### 3.1 Overview

| Stage | Frame (page) | Content |
|---|---|---|
| 0 | d1 (p.30) | Open the front panel; make it fully transparent when running as an executable |
| 1 | d2 (p.32) | Reset `error out`, `Connected`, `HI Inited`, `LO Inited` |
| 2 | d3 + d4–d61 (p.34–43) | Resolve the settings folder, read `alox.ini` (inner sequence `[0..6]`), PI macro check |
| 3 | d0 (p.3–5) + d62–d250 | Create the three queues, run the three loops until shutdown, release the queues |
| 4 | d251 (p.44) | `Quit LabVIEW` if running as an executable |

Outside the sequence: the terminal of the tab control `Tab` (unwired) and a Diagram Disable structure whose `Enabled` frame is empty
[main VI frames d252, d253]. Neither has any effect.

### 3.2 Stage 0: window [main VI frame d1; p.30]

```
kind ← App.Kind                                         # Open Application Reference (local); errors not wired
transparency ← 100 if kind == "Run Time System" else 0  # percent
vi ← Open VI Reference(Current VI's Path)
vi.FP.Open(Activate = TRUE, State = Standard); vi.FP.Transparency ← transparency
Close Reference(vi); Close Reference(app)
```

In the executable the window is open but invisible. The client command `Show UI` sets the transparency to 0 (show) or 100 (hide)
([03-command-reference.md](03-command-reference.md)). A Python server without a GUI can treat `Show UI` as a no-op replying "no error".

### 3.3 Stage 1: indicator reset [main VI frame d2; p.32]

`error out ← {FALSE, 0, ""}`, `Connected ← FALSE`, `HI Inited ← FALSE`, `LO Inited ← FALSE` (local-variable writes). `Z Inited` is
**not** reset. LabVIEW indicators keep their last value between runs in the development environment, which is presumably why these
writes exist (inferred).

### 3.4 Stage 2: configuration [main VI frames d3–d61; p.34–43]

#### 3.4.1 Locating `alox.ini`

```
exe_dir      ← tools_application_executable_folder()   # exe: folder of the .exe ; dev: Current VI's Path stripped twice  [Doc6 p.87–90]
settings_dir ← Read Key[Path](exe_dir\alox_setup.ini, "folders", "settings", default D:\alox\settings)   # [Doc6 p.409–412]
cfg, err     ← Open Config Data(settings_dir\alox.ini)  # refnum also stored in the front-panel indicator "refnum"
# "create file if necessary" is unwired (TRUE): a missing alox_setup.ini / alox.ini is created empty
# when its folder exists and is writable, and every key then takes its default (09 §6.4)
```

Despite its name, `tools_windows_user_documents_folder.vi` no longer reads the Windows "Documents" folder (that code is disabled), so
the stage label "Load Default settings from Alox configuration file in "C:Users/<USER>/alox/settings/alox.ini"" is obsolete
[main VI p.34]. The folder VI's error output is not wired. INI parsing rules and the two other folders (`vision`, `data`, used by the
logs) are in [09-configuration-logging-errors.md](09-configuration-logging-errors.md).

#### 3.4.2 Inner sequence `[0..6]`

Every inner frame receives the config refnum and the `Open Config Data` error, and publishes its results **only** to front-panel
objects. Frames 1–3 are `For` loops with a `Case` on the iteration index, i.e. N sequential `Read Key` calls updating one cluster.

| Inner frame | Frames | Reads | Writes (front-panel object) |
|---|---|---|---|
| 0 | d4 | `[hardware.camera] port_I32` (503), `[general] core_I32` (1), `[log] maxlines_I32` (1000) | `TCP Port`, `Core #`, `#Lines` |
| 1 | d5–d25 | 19 keys of `[hardware.camera.highmag]` (`frameratemax_DBL` is **not** read) | `High Mag Settings` |
| 2 | d26–d47 | 20 keys of `[hardware.camera.lowmag]` | `Low Mag Settings` |
| 3 | d48–d58 | 7 keys of `[hardware.z]` (baud rate stored as an index 0–3) | `Z Stage Settings` |
| 4 | d59–d60 | no key: PI `RMC?` / `MAC START "DH"` (§3.4.3) | `Running macros` |
| 5 | d3 (visible) | `setLEDline_I32`, `setBULBline_I32` (`[hardware.camera.highmag]`), `setLEDcam_I32` (`[general]`), `lightStatusInverter_BOOL` (`[hardware.camera]`) | `HM setLEDline`, `HM setBULBline`, `Camera Sort: LED`, `Connector`, `LEDStatusInverter` |
| 6 | d61 | — | `Close Config Data` (`write file if changed (T)`; nothing was modified, so nothing is written) |

That is 53 `Read Key` calls. Frame 5 reads `High Mag Settings` / `Low Mag Settings` back through local variables, so it relies on
frames 1–2 having run: `Camera Sort: LED ← (setLEDcam ≠ 0 ? Low Mag Settings : High Mag Settings).Camera Plug Type`,
`Connector ← setLEDcam ≠ 0 ? "LM" : "HM"`. Keys and defaults: [09-configuration-logging-errors.md](09-configuration-logging-errors.md);
LED semantics: [05-camera-acquisition.md](05-camera-acquisition.md) §9.

Error behaviour (observed wiring, consequences inferred): no `Read Key` error reaches an indicator. A missing or unreadable key
gives its default; a failed `Open Config Data` makes **every** value fall back to its default. The error wire is chained only
through the three keys of frame 0, the first four keys of frames 1–2 and the first five of frame 3; in frame 5 the first three
keys receive only the open error. A non-convertible value in a chained key therefore forces the later keys of the same chain to
their defaults even if they are present ([09](09-configuration-logging-errors.md) §5.4). Whether an unwired error opens a dialog
depends on the VI's "Automatic error handling" setting **(unknown)** (Q9). The server always starts, whatever the INI contains.

#### 3.4.3 PI macro check (inner frame 4) [main VI frames d59, d60; p.42]

```
sys ← Z Session (local variable of the front-panel control).System
(running, err) ← RMC?(sys)                     # no error in; "List macros currently running"
Running macros ← running
if running == "DH" or running == "HOME":       # exact string comparison
    err ← MAC START(sys, "DH", err)
Clear Errors(err)
```

This is the only hardware access before stage 3. Nothing has opened a PI connection at this point, so in a freshly started
executable `RMC?` fails, the error is cleared and only `Running macros` changes (inferred). The logic is the **inverse** of
`Z Get Status` / `Z Stop` / `Z Cleanup`, which restart `DH` when it is *not* running ([08-z-stage.md](08-z-stage.md)).
Recommendation: skip this frame unless a PI connection already exists (ask, Q8).

### 3.5 Stage 3: run [main VI frame d0; p.3–5]

Stage 3 creates the queues (§4), writes `Image.Zoom to Fit Mode ← TRUE` once, starts the three loops (§5) and releases the queues
when the loops end (§7). It ends when all three loops and the three `Release Queue` nodes have finished. A diagram comment reads
"#TODO: set low-angle/high angle type".

### 3.6 Stage 4: quit [main VI frame d251; p.44]

`if App.Kind == "Run Time System": Quit LabVIEW`. Nothing else: no `TCP Close`, no IMAQdx close, no PI `CloseConnection`, no
configuration save, no log line. Hardware is released only if the client sent `Cam Cleanup` / `Z Cleanup` before `Exit`, or
implicitly by the process exit (inferred).

---

## 4. The three queues

### 4.1 Overview diagram (stage 3)

```
                    client  (127.0.0.1 : TCP Port)
                          ▲    │
                          │    ▼
             ┌───────────────────────────────────┐ ◀── self-enqueue {Read} / {Listen}, timeout −1 ───┐
  ┌────────▶ │ TCP SERVER PRODUCER LOOP          │                                                   │
  │          │ Dequeue("Camera Server", 50 ms)   │ ──────────────────────────────────────────────────┘
  │          │ states: Listen, Read,             │
  │          │         Send Response             │
  │          └─────┬─────────────────────┬───────┘
  │      Z range   │                     │   camera range + "Log" (+ receiving timestamp)
  │                ▼                     ▼
  │      "Z Stage Mgr" (5)        "Camera Mgr" (5) ◀── self-enqueue: stream, AF Run, RF Run, timeout −1 ──┐
  │                │                     │                                                                │
  │                ▼                     ▼                                                                │
  │       ┌────────────────┐  Exit  ┌─────────────────────┐                                               │
  │       │ Z STAGE        │ ◀───── │ CAMERAS             │ ──────────────────────────────────────────────┘
  │       │ CONSUMER LOOP  │        │ CONSUMER LOOP       │
  │       │ Dequeue(−1)    │        │ Dequeue(−1)         │
  │       └───────┬────────┘        └──────────┬──────────┘
  │               │ replies                    │ replies
  └───────────────┴────────────────────────────┘
     {Send Response, HW TCP Response Data} enqueued into "Camera Server" (5), queue obtained by name, timeout −1
  End of cameras loop → Release("Camera Server", destroy) + Release("Camera Mgr", destroy) → TCP loop stops
  End of Z loop       → Release("Z Stage Mgr", destroy)
```

`Exit` goes from the camera loop to the Z loop through the Z queue. Autofocus and refocus, inside the camera loop, talk to the PI
controller directly, not through the Z queue.

### 4.2 Definitions [main VI frame d0; Doc6 p.418–421]

| Queue | Name (global) | Bound | Element type | Producers | Consumer |
|---|---|---|---|---|---|
| TCP | `TCP Server Queue Name` = `"Camera Server"` | 5 (the same constant feeds the indicator `TCP Max Size`; comment "#Question : Borner TCP Queue Size ?") | `HW TCP Server Queue` = `{State: enum HW TCP Server Command, State Data: variant}` [Doc6 p.1040–1043]; `{Listen, empty}` is enqueued right after `Obtain Queue` | the TCP loop itself (`Listen`, `Read`); every `Enqueue Send <type> Response.vi` (`Send Response`) | TCP loop |
| Camera | `Camera Mgmt Queue Name` = `"Camera Mgr"` | 5 | `Camera Queue Data` (`typedef_ext_camera_mgmt_queue_data.ctl`) [Doc6 p.285–288] | TCP loop (routing); camera loop (self re-enqueue) | camera loop |
| Z | `Z Stage Mgmt Queue Name` = `"Z Stage Mgr"` | 5 | `Camera Queue Data` (same typedef) | TCP loop (routing); camera loop (`Exit` only) | Z loop |

`create if not found?` is unwired (TRUE) on the three `Obtain Queue` nodes and their error terminals are unwired. No `Enqueue Element`
of the main VI or of the `Enqueue Send … Response` VIs has a timeout wired (−1, wait forever).

`HW TCP Server Command` items (all five frames of the TCP `State` case): `Listen` (default), `Read`, `Send Heartbeat`,
`Send Response`, `Quit` [main VI frames d0, d62, d68–d70]. Numeric order **(unknown)**; this enum never goes on the wire.

`Camera Queue Data`, the command envelope (field order confirmed by a positional `Unbundle` [main VI frame d0]):

| # | Field | Type | Role |
|---|---|---|---|
| 0 | `Camera Command` | enum `typedef_ext_camera_camera_command.ctl`: 46 command names observed, full item list **(unknown)** | selects the route and the case |
| 1 | `Notifier Name` | string | correlation id, echoed unchanged in the reply |
| 2 | `Command Data` | variant | command-specific payload |
| 3–4 | `Command Request Timestamp`, `Command Sending Timestamp` | U32 | set by the client (inferred) |
| 5 | `Command Receiving Timestamp` | U32 | overwritten by the router (camera route only) |

Binary layout and payload typedefs: [04-data-types.md](04-data-types.md).

### 4.3 Who holds which reference [main VI frame d0]

| Loop | TCP queue | Camera queue | Z queue | TCP queue **name** (string) |
|---|---|---|---|---|
| TCP loop | dequeue, self-enqueue | enqueue (routing) | enqueue (routing) | — |
| Camera loop | only to destroy it after the loop | dequeue, self-enqueue, destroy after the loop | enqueue `Exit` only | passed to every `Enqueue Send … Response` |
| Z loop | — | wired in, used by no case | dequeue, destroy after the loop | passed to every `Enqueue Send … Response` |

The consumers reply without the TCP queue reference: each `Enqueue Send … Response` call obtains the queue **by name**
(`Obtain Queue(create if not found = FALSE)`) [Doc6 p.437, 445, 573].

### 4.4 Every blocking call

| Site | Call | Timeout | Blocks when |
|---|---|---|---|
| TCP loop | `Dequeue(TCP)` | 50 ms | never long; the timeout enqueues `Read` |
| TCP loop, `Listen` | `TCP Listen` | 2000 ms | no client connects |
| TCP loop, `Read` | `HW TCP Extract TCP Data` | 10 ms header, 200 ms datatype, 5000 ms payload; 25 s for the extra bytes of a `Heartbeat` frame ([02](02-tcp-protocol.md) §4.5) | a frame is arriving |
| TCP loop, `Listen`/`Read`/timeout | `Enqueue(TCP)` (self) | −1 | TCP queue full |
| TCP loop, routing | `Enqueue(Z)` / `Enqueue(camera)` | −1 | Z / camera queue full |
| TCP loop, `Send Response` | `TCP Write` | 1000 ms | client not reading |
| Camera and Z loops | `Dequeue` | −1 | idle |
| Camera and Z loops, replies | `Enqueue Send … Response` → `Enqueue(TCP)` | −1 | TCP queue full |
| Camera loop, stream/AF/RF | `Enqueue(camera)` (self) | −1 | camera queue full |
| Camera loop, `Exit` | `Enqueue(Z)` | −1 | Z queue full |

Long hardware calls inside a case also block their loop: IMAQdx grabs (default timeout 5 s), `Z Do Reference` and
`Z Wait End Move` (up to 300 s, [08](08-z-stage.md) §8), `Wait LED`, AF/RF steps.

### 4.5 Queue indicators

| Indicator | Source | Refresh |
|---|---|---|
| `TCP Queue Size` | `Get Queue Status(TCP)`, taken from the node's **top** output, unlike the two others; which output that is: **(unknown)** | every TCP-loop iteration |
| `TCP Max Size` | constant 5 | once |
| `CAM Queue Size` | `Get Queue Status(camera).# elements`, placed **outside** the loops | once at start-up (always 0) |
| `Z Queue Size` | `Get Queue Status(Z).# elements` | every Z-loop iteration, before the dequeue |

---

## 5. The three loops

### 5.1 Summary

| | TCP SERVER PRODUCER LOOP | CAMERAS CONSUMER LOOP | Z STAGE CONSUMER LOOP |
|---|---|---|---|
| Dequeue | TCP queue, 50 ms | camera queue, −1 (error unwired) | Z queue, −1 (error unwired) |
| Shift registers | `conn` (TCP connection, uninitialised) | HM session, LM session (from the constants `High Mag Session`, `Low Mag Session`); HM and LM temperature histories (DBL[], start empty) | `Z Session` (from the front-panel **control** `Z Session`) |
| Main case | `State` (5 states) | `Camera Command`: 27 named cases + `Default` | `Camera Command`: 19 Z cases + `Exit` + `Default` |
| Outside the case | `TCP Queue Size`; for each dequeued element `Heartbeat ← Tick Count`, `Server State ← State` | error and temperature histories | `Z Queue Size`; `Z Stage Settings` read (local) for every message |
| Stop ("Stop if True") | `Dequeue error.status OR stop_state`; `stop_state` TRUE only in `Quit` | TRUE only in `Exit` | TRUE only in `Exit` |
| After the loop | nothing (no `TCP Close`) | `Release Queue(TCP, destroy)`, `Release Queue(camera, destroy)` | `Release Queue(Z, destroy)` |
| `timed out? = TRUE` case | active: enqueues `Read` (d71) | dead code (d217–d219) | empty (d250) |

### 5.2 TCP producer loop [main VI frames d0, d62–d72; p.3, 5–6, 8–9]

#### 5.2.1 State machine

| State | Frame | Action | Element enqueued next (own queue) |
|---|---|---|---|
| `Listen` (also `Default`) | d62 | `TCP Listen(net address = String To IP("localhost"), port = TCP Port (local, I32 coerced to U16), timeout = 2000 ms)`; `Connected ← NOT error.status`; `conn ← new connection` (old value overwritten, not closed) | success: `{Read}`; failure: the dequeued `{Listen}` again |
| `Read` | d0, d63–d67 | `HW TCP Extract TCP Data(conn)`, then a case on `error.code` | code 0: **nothing**; code 56: `{Read}`; other codes: `TCP Close(conn)`, then `{Listen}` with `error in` = the close error (nothing is enqueued if the close fails) |
| `Send Response` | d69 | `rd ← Variant To Data(State Data, HW TCP Response Data)`; `rd.TCP Sending timestamp ← Tick Count`; `HW TCP Send Response(conn, rd)` | nothing |
| `Send Heartbeat` | d68 | `HW TCP Send Heartbeat(conn)`; comment "#Question : Ajouter une bloucle Heartbeat ?" | nothing; **nothing enqueues this state** |
| `Quit` | d70 | `TCP Close(conn)`; `stop_state ← TRUE` | **nothing enqueues this state** |
| *dequeue timed out (50 ms)* | d71 | `Enqueue({Read})` (error cleared); if the button `CLEAR` is TRUE: `Log.Value ← ""` | `{Read}` |

`Read` with code 0: if the datatype is `Command`, `msg ← Unflatten From String(Data, Camera Queue Data)` (error and "rest of string"
outputs unwired), `Command ← msg.Camera Command`, route (§5.2.2), `Commands History ← first 500 of [command] + Commands History`.
Because the `Unflatten` error is unwired, a payload that does not decode is not rejected: a default element is routed and executed
**(inferred**: `Camera Command = Cam Setup`, empty notifier and variant; [02](02-tcp-protocol.md) §7.6). Any other datatype
(`Heartbeat`, `Image`, `Response`) is read and discarded [main VI frame d67]; a `Heartbeat` frame that carries a payload ends with
error −1 "Extra Data Found" and therefore closes the connection ([02](02-tcp-protocol.md) §9).

Code 56 is a TCP read timeout. Normally it means that no header arrived within 10 ms, i.e. the idle case. It is also returned when
the datatype (200 ms) or the payload (5000 ms) of a started frame times out; the connection then stays open although the stream may
be desynchronised ([02](02-tcp-protocol.md) §5.5). Other codes (typically 62, 64, 66 for a closed or reset connection, inferred) mean
that the connection is lost. `Connected` is **not** reset to FALSE then; it changes only at the next `Listen`.

#### 5.2.2 Routing [main VI frames d0, d65, d66; p.3, 6]

| Case of `Camera Command` | Diagram comment | Action |
|---|---|---|
| `"Z Setup".."Z Do Reference"` | "Z Consumer loop for Z stage commands" | `Enqueue(Z, msg)` (timestamps unchanged) |
| `"Get Heartbeat".."Cam Get Led", "Log"` | "Camera consumer loop for camera and general commands", "Store receiving timestamp for image statistics" | `msg.Command Receiving Timestamp ← Tick Count`; `Enqueue(camera, msg)` |
| `Default` | "Unknown command" | nothing: no reply, the client times out; the command still enters `Commands History` |

The ranges are numeric. Every command handled by the camera loop except `Log` must therefore lie between `Get Heartbeat` and
`Cam Get Led` in the enum, and every Z command between `Z Setup` and `Z Do Reference`; otherwise its case is unreachable. This
includes `Exit`, which must be in the camera range for the server to be stoppable (inferred). The real order is **(unknown)**: it is
the first blocking question of the project (Q1, [04-data-types.md](04-data-types.md)).

#### 5.2.3 Timing behaviour (consequences of the wiring, inferred)

- Only one "control" element (`Listen` or `Read`) circulates: each produces at most one successor, and the timeout case adds `Read`
  only when the queue is empty. `Send Response` elements are interleaved with it in FIFO order.
- **Idle, client connected:** `Read` → 10 ms wait → code 56 → `Read` … The socket is polled about every 10 ms; a pending reply waits
  at most one poll.
- **After a successfully read frame no `Read` is enqueued.** The next read starts only after the queue has stayed empty for 50 ms,
  so consecutive commands are read at least about 50 ms apart, and replies arriving more often than every 50 ms postpone reading.
- **No client:** `Listen` blocks up to 2 s per attempt. Replies dequeued meanwhile are written on an invalid refnum and lost. A
  reply still queued when a new client connects goes to that new client ([02](02-tcp-protocol.md) §4.6). If the port is already
  taken by another program, `Listen` fails at once and is retried without pause ([02](02-tcp-protocol.md) §3).
- **One client:** a second client can complete the TCP handshake (the NI listener stays open between `TCP Listen` calls
  [Doc6 p.1141–1149]) but is served only after the first connection fails and the loop returns to `Listen`.
- Reads and writes share one loop, so frames are never interleaved on the socket. A reply also waits while `Read` receives a frame
  (up to about 5.2 s for a slow frame).
- `Heartbeat` on the panel shows that the TCP loop is alive. The client-level heartbeat is the `Get Heartbeat` **command** (reply: the
  server's `Tick Count`), run by the camera loop; the server never sends heartbeat frames ([02-tcp-protocol.md](02-tcp-protocol.md)).

### 5.3 Camera consumer loop [main VI frames d0, d73–d219]

| Aspect | Behaviour |
|---|---|
| Before the case | a Diagram Disable whose `Enabled` frame is empty [main VI frames d215, d216] |
| Case | 27 named commands + `Default` (no action, no reply) [main VI frame d73]; `Exit` is described in §7.1 [main VI frame d78; p.8]. Per command: [03](03-command-reference.md), [05](05-camera-acquisition.md), [06](06-autofocus-refocus.md) |
| Common case pattern | `Variant To Data(Command Data)`; select the HM or LM session/settings by `Camera Type`; do the work; append a suffix to `error.source` on error; `error out (local) ← Log Server Error(…)`; reply with `Enqueue Send <type> Response`; `stop ← FALSE` |
| Persistent state | the two camera sessions (IMAQdx session, SRC/FFC/DFC images, acquisition and AF/RF states, last focus data, progress; [05](05-camera-acquisition.md) §3.2) live **only** in this loop's shift registers. The stream state machine and the AF/RF sub-VIs also keep private static state ([05](05-camera-acquisition.md) §11, [06](06-autofocus-refocus.md) §1.2) |
| Self re-enqueue | `Cam Stream Next Image` (until its image is ready), `Cam Autofocus` → `Cam Autofocus Run` …, `Cam Refocus` → `Cam Refocus Run` …: the loop puts an element into **its own** queue, timeout −1 |
| Z access | its only use of the Z queue is forwarding `Exit`. AF/RF drive the PI controller **directly** with the `Z Session` and `Z Stage Settings` panel objects read through local variables ([06](06-autofocus-refocus.md) §7) |
| After every iteration (outside the case) | `Array ← Array + [error out]` (append, unbounded); `hm_hist += [HM T° (°C)]`, `lm_hist += [LM T° (°C)]`, `Temperature Graph ← [hm_hist, lm_hist]` (unbounded) |
| `timed out? = TRUE` | enqueue a "Get Image Cmd" element if an acquisition is in progress; unreachable with timeout −1, because the queue is destroyed only after the loop [main VI frames d217–d219; p.29]. Do not re-implement |

The error history `Array` reads `error out` through a local variable with no data dependency on the case, so it records this
iteration's error or the previous one (inferred). `error out` is also written by the Z loop, so Z errors can appear in it (inferred).
Every re-enqueued step is one iteration, so a stream or an autofocus adds many history entries.

### 5.4 Z consumer loop [main VI frames d0, d220–d250]

| Aspect | Behaviour |
|---|---|
| Case | after `Unbundle` of `Camera Command`, `Notifier Name`, `Command Data` (timestamps unused): 19 Z commands + `Exit` (stop, no reply) + `Default` (nothing) [main VI frame d220]. Per command: [08-z-stage.md](08-z-stage.md) |
| Common case pattern | no `error in` on the first node; PI GCS calls; `Enqueue Send <type> Response(TCP Server Queue Name, Notifier Name, value, error)`; `error out (local) ← Log Server Error(Server Type = "Z Stage", …)`; `stop ← FALSE` |
| Persistent state | shift register `Z Session` (`System`, `Axes`, `Identity`, `Serial Number`) [Doc6 p.644–645], initialised from the front-panel control `Z Session` when stage 3 starts. The control's saved default is **(unknown)** (in the development environment it keeps the value written by the previous run, inferred). `Z Setup` and `Z Cleanup` also write the control (local); the automatic re-initialisation inside `Z Move Absolute/Relative` updates only the shift register, so the copy read by AF/RF can become stale |
| Settings | `Z Stage Settings` read (local) for every message; `Z Set Min Limit` / `Z Set Max Limit` write it, in memory only |

Z commands run strictly in arrival order. `Z Do Reference`, `Z Wait End Move`, `Z Setup` (which runs an `FNL` reference move when
the axis is not referenced) and a `Z Move Absolute/Relative` that triggers the automatic re-initialisation block the loop for the
whole move ([08-z-stage.md](08-z-stage.md) §4, §6, §12).

---

## 6. Response path

### 6.1 Mechanism

```
consumer loop (camera or Z)
  └─ Enqueue Send <type> Response.vi (TCP Server Queue Name, Notifier Name, [value], error[, image timestamps])
        q  ← Obtain Queue("Camera Server", create if not found = FALSE)    # never released: one leaked reference per reply
        rd ← HW TCP Response Data {Notifier Name, Response (Flattened) = Flatten({value, error}), 6 × U32}
        Enqueue(q, {State = "Send Response", State Data = variant(rd)}, timeout −1)
TCP loop, state "Send Response"
  └─ rd.TCP Sending timestamp ← Tick Count ; HW TCP Send Response(conn, rd)     # one Response frame
```

| Sub-VI (icon) | `Response (Flattened)` | Sent when the command failed? |
|---|---|---|
| `Enqueue Send Error Cluster Reponse.vi` ("CAM SERVER RESP") [Doc6 p.435–438] | the error cluster alone | **probably not**: `error in` is also wired to the `error in` of `Enqueue Element`, so an error skips the enqueue and the client times out (strongly plausible; confirm with a capture, Q4) |
| `Enqueue Send STR / Boolean / DBL / U32 / DBL 2D Array Response.vi` [Doc6 p.443–446, 455–458, 463–466, 559–562, 1124–1127] | `{Value, error}` | yes: the error is data; the enqueue fails only on a queue error |
| `Enqueue Send Image Response.vi` [Doc6 p.571–574] | `{IMAQ Flatten Image to String(image), error}` | yes |

Encoding: [02-tcp-protocol.md](02-tcp-protocol.md). Reply type of each command: [03-command-reference.md](03-command-reference.md).

### 6.2 Timestamps

| Field | Set by | When |
|---|---|---|
| `Command Request Timestamp`, `Command Sending Timestamp` | client (inferred) | before sending |
| `Command Receiving Timestamp` | router (TCP loop) | camera route only, at routing time ("for image statistics") |
| reply `Command Request`, `TCP Command Sending`, `TCP Command Receiving` | `Enqueue Send Image Response` copies the command's three values | image replies only; 0 in all other replies |
| reply `Pre-flattening`, `Post-Flattening` | `Enqueue Send Image Response` | around `IMAQ Flatten Image to String`; 0 otherwise |
| reply `TCP Sending timestamp` | TCP loop, `Send Response` | every reply, just before the write |

Client and server run on the same PC (loopback only), so the client may compare these values with its own `Tick Count` (inferred).
Whether it does decides whether Python must reproduce LabVIEW's millisecond counter, epoch included (Q5).

### 6.3 Correlation, ordering, missing replies

The client matches a reply to its call by `Notifier Name`, copied unchanged into the reply; there is no other request id. Replies
leave in the order they were enqueued; camera and Z replies interleave, so the reply order can differ from the command order
(inferred). **No reply at all** is produced for: unknown commands, the camera and Z `Default` cases, `Exit`, `Cam Get Image`,
`Cam Get Images Array`, the internal `Cam Autofocus Run` / `Cam Refocus Run`, intermediate `Cam Stream Next Image` steps (one reply
per request, at the end) and, probably, every error-cluster reply carrying an error (§6.1). At shutdown, Z commands queued before
`Exit` still run but their replies are lost (the TCP queue is destroyed, so `Obtain Queue(create = FALSE)` fails), and camera
commands queued behind `Exit` are discarded with the camera queue.

---

## 7. Shutdown chain

### 7.1 Normal path (the only one in the code)

1. The client sends `Exit`; the router puts it into the camera queue (it is in the camera range, inferred).
2. Camera loop, case `Exit`: `Enqueue(Z, {Exit})`, `stop ← TRUE` [main VI frame d78]. No reply.
3. After the camera loop: `Release Queue(TCP, destroy)` and `Release Queue(camera, destroy)`, in parallel.
4. The TCP loop finishes its current state; its next `Dequeue` returns an error, so `error.status = TRUE` stops the loop (the
   timeout case may run once more before; its `Enqueue` fails and the error is cleared, inferred). The connection is **not** closed.
5. The Z loop runs the Z commands queued before `Exit`, then `Exit`: `stop ← TRUE`, `Release Queue(Z, destroy)`. The replies of those
   last commands are lost (§6.3).
6. Stage 3 ends; stage 4 quits the executable. The OS then closes the socket, the serial port and the camera handles (inferred).

The `Quit` TCP state (`TCP Close` + stop) exists, but nothing enqueues it.

### 7.2 When the server never exits (inferred)

| Situation | Effect |
|---|---|
| `Exit` is outside the camera range of the enum, or never reaches the camera loop because that loop is blocked (hardware call, full-queue `Enqueue`) | the camera loop never stops; the process keeps running |
| The camera loop waits on `Enqueue(Z)` for `Exit` (Z queue full, Z loop busy) | delayed until the Z loop dequeues |
| Any deadlock of §9.1 | the process must be killed |
| Development environment | stage 4 does nothing; the VI stops, LabVIEW keeps running |

What shutdown does not do: no `IMAQdx Stop/Unconfigure/Close`, no image disposal, no PI `CloseConnection`, no `TCP Close`, no
settings save, no log entry. Recommendation: in Python, close both cameras, the PI connection and the socket on `Exit` (fix; the
client sees no difference, since `Exit` gets no reply either way).

---

## 8. Front-panel objects that carry state

The panel has a header and a tab control `Tab` with pages `High Mag.`, `Low Mag.`, `T°`, `Z Stage`, `Autofocus`, `Log`,
`Commands` [main VI p.2]. Only `High Mag.` was exported; where the other objects sit is **(unknown)**. "Local" means accessed
through local variables, without locking.

### 8.1 Configuration (written in stage 2)

| Object | Type | Written by | Read by |
|---|---|---|---|
| `TCP Port`, `Core #`, `#Lines` | I32 indicators | stage 2 frame 0 | `TCP Port`: `Listen`. `#Lines` and `Core #`: inputs `Max Lines` and `Core#` of every `Log Server Error` call; `Core#` only feeds disabled code ([09](09-configuration-logging-errors.md) §7.1) |
| `High Mag Settings`, `Low Mag Settings` | cluster indicators (`alox_typedef_ext_camera_properties`) [Doc6 p.648–651] | stage 2 frames 1–2; later `Cam Flip`, `Cam Set FlatField` (local) | every camera command, AF/RF, stage 2 frame 5 |
| `Z Stage Settings` | cluster indicator (`alox_typedef_ext_zstage_properties`) [Doc6 p.652–653] | stage 2 frame 3; later `Z Set Min/Max Limit` (local) | Z loop (every message); AF/RF (camera loop) |
| `HM setLEDline`, `HM setBULBline`, `Camera Sort: LED`, `Connector`, `LEDStatusInverter` (control) | I32, I32, enum, string, boolean | stage 2 frame 5 | `Cam Start` (HM), `Cam Set Led`, `Cam Get Led`; `Connector` is display only |
| `BulbStatusInverter` | boolean control, never initialised | — | `Cam Set Led` (USB path) |
| `refnum` | config refnum | stage 2 | `Cam Set Led`: a `Write Key` on the already closed refnum (no effect) |

### 8.2 Runtime state shared between loops

| Object | Written by | Read by | Hazard |
|---|---|---|---|
| `Z Session` (control, `alox_typedef_ext_zstage_session`) | `Z Setup`, `Z Cleanup` (local, Z loop) | Z-loop register initialisation, stage 2 frame 4, AF/RF (camera loop) | stale after the automatic re-init of `Z Move …` |
| `Z Stage Settings` | Z loop | camera loop (AF/RF) | unsynchronised read/write |
| `error out` (error cluster) | stage 1; nearly every camera and Z case | camera-loop history `Array` | two writers, one unsynchronised reader |
| `Commands History` (`Camera Command[]`, newest first, 500 max) | TCP loop (each decoded command); camera loop (`Cam Autofocus Run`) | itself (read-modify-write) | concurrent read-modify-write can lose entries |
| `Log Ref` target (probably the `String` indicator, inferred) | `Log Server Error` from both consumer loops, through a control reference; `Cam Set Led` overwrites `String` with `"us"`/`"gi"` | — | see [09](09-configuration-logging-errors.md) |

### 8.3 Status and display (one writer each)

| Writer | Objects |
|---|---|
| TCP loop | `Server State` (last state), `Heartbeat` (Tick Count of the last dequeued element), `Command` (last decoded command), `TCP Queue Size`, `Connected`; `TCP Max Size` = 5 |
| Stage 3 / Z loop | `CAM Queue Size` (once) / `Z Queue Size` |
| Camera loop | `HI Inited` (shown "Inited"), `LO Inited`, `High Acq. State` (shown "Acq. State"), `Low Acq. State`, `HM T° (°C)`, `LM T° (°C)`, `BusTypeHM`, `BusTypeLM`, `SerialNumberHigh`, `SerialNumberLow`, `Max Framerate`, `MaxFR real`, `LED cmd`, `LEDStatus`, `Image` (IMAQ display, "Zoom to Fit" set in stage 3), `Focus Graph` and `Autofocus Data` (AF/RF) |
| Z loop / stage 2 | `Z Inited`, `Z Pos (mm)`, `Macro running ?` / `Running macros` |

### 8.4 Histories

| History | Loop | Update rule | Bound |
|---|---|---|---|
| `Commands History` | TCP, plus camera for `Cam Autofocus Run` | prepend, keep 500 | 500 |
| `Array` (errors) | camera, every iteration | append `error out`, "no error" entries included | none |
| `Temperature Graph` (2-D DBL, row 0 = HM, row 1 = LM) | camera, every iteration | append the current HM and LM temperatures | none |
| `Log` (string) | — | no observed writer; cleared when `CLEAR` is TRUE during a TCP dequeue timeout | — |

Unused objects: `Cam Log Data` (written by the `Log` command, read nowhere: a TODO), the `Tab` terminal, and the `CLEAR`/`Log` pair.

---

## 9. Concurrency hazards

### 9.1 Blocking cycles through bounded queues (inferred from the wiring)

Every queue holds 5 elements, every enqueue waits forever, and each loop is the **only** consumer of its queue: a loop that blocks on
its own queue never wakes up.

| # | Cycle | Trigger | Result |
|---|---|---|---|
| H1 | TCP loop → TCP queue | 4 replies already queued when the loop takes its `Read`/`Listen` element, a 5th arrives, then the loop re-enqueues `Read`/`Listen` | TCP loop frozen: no read, no reply; consumers then freeze on their next reply |
| H2 | camera loop → camera queue | a stream/AF/RF step re-enqueues while the router has filled the queue with 5 camera commands | camera loop frozen; the router blocks on the next camera command, so the TCP loop freezes too |
| H3 | TCP → camera queue → camera loop → TCP queue | camera queue full (5 commands) and TCP queue full (5 replies) | both frozen |
| H4 | TCP → Z queue → Z loop → TCP queue | the same with the Z queue | both frozen |
| H5 | long Z operation (`Z Do Reference`, up to 300 s) with 5 more Z commands queued | the router blocks on `Enqueue(Z)` | camera commands and all replies stall until the Z loop dequeues; this becomes H4 if 5 replies pile up in the TCP queue meanwhile and the Z loop then tries to reply |

H2 is the most likely: it needs only a client that keeps several camera commands outstanding (for example progress polls from
several client callers) during an autofocus or a stream. Whether the client ever has several commands outstanding is
**(unknown)** (Q3).

### 9.2 Unsynchronised shared memory

Panel objects are shared through local variables without locking (§8.2). Each single read or write is atomic (inferred), but
read-modify-write sequences are not: `Commands History` (two writers), `Z Stage Settings` (written by the Z loop, read by the camera
loop), `Z Session` (two diverging copies), `error out` (written by both consumers, sampled by the camera loop).

### 9.3 Shared sub-VIs and devices

| Shared resource | Callers | Risk |
|---|---|---|
| `Enqueue Send Error Cluster Reponse.vi`, `Enqueue Send Boolean / DBL Response.vi` (the STR reply is used by the Z loop only) | camera and Z loops | if non-reentrant **(unknown)**, one loop waits while the other is inside; harmless unless the TCP queue is full |
| `Log Server Error.vi` | camera and Z loops | serialised if non-reentrant, otherwise interleaved file and indicator writes (inferred) |
| PI GCS driver VIs (`POS?`, `MOV`, `VEL`, `VEL?`, `JON?`) | Z loop and, through AF/RF, the camera loop | request/answer pairs of the two loops can interleave on the serial link ([06](06-autofocus-refocus.md) §7.4, [08](08-z-stage.md)) |
| Stream state machine, AF and RF sub-VIs | camera loop only, both cameras | private static state shared between HM and LM ([05](05-camera-acquisition.md) §11, [06](06-autofocus-refocus.md) §1.2) |

---

## 10. Python design

### 10.1 Mapping

| LabVIEW | Python | Notes |
|---|---|---|
| Stages 0–2 | `main()`: optional GUI, hidden by default → initial state → `load_config()` (tolerant reader: every failure gives the default and is logged) | PI macro check only if a link exists (§3.4.3) |
| Stage 3 | three threads: `tcp`, `cameras`, `z` | asyncio could serve the socket, but the camera SDKs and the PI driver block, so they need threads anyway |
| Stage 4 | close hardware, then `sys.exit(0)` | adds the missing cleanup |
| TCP queue and `Send Response` state | **removed**: consumers write their reply frame directly through `ClientLink.send()`, which holds one lock per frame (same choice as [02](02-tcp-protocol.md) §11.2) | keeps "frames never interleave"; removes H1, H3, H4. The single-writer variant of [10](10-migration-plan.md) §3.3 (replies queued to the TCP thread) is equivalent if that queue is unbounded and the TCP thread never blocks on it |
| `Listen`/`Read` polling, 50 ms re-arm | one listening socket, an accept loop and a reader that reads frames back to back (short timeouts only to notice the stop event) | removes the ≥ 50 ms gap per command |
| Camera and Z queues bounded at 5, `Enqueue` −1 | unbounded queues | removes H2; memory is bounded in practice by the client's request rate |
| Self re-enqueue (stream, AF/RF `Run`) | the cameras thread runs one pending step when its queue is empty and after each command | [06](06-autofocus-refocus.md) §9.5, [05](05-camera-acquisition.md) §11 |
| `Release Queue(force destroy)` | `LVQueue.destroy()` wakes every waiter with `QueueDestroyed` | shutdown |
| Panel objects, local variables | one `ServerState` with an `RLock`; settings replaced as whole objects | fixes §9.2 |
| Two copies of `Z Session`, PI calls from two loops | one `ZStage` link shared by the z thread and the focus code, one lock per GCS request/answer | [08-z-stage.md](08-z-stage.md) |
| Histories | `deque(maxlen=…)`; temperatures time-stamped | fixes unbounded growth |
| `Tick Count (ms)` | `int(time.monotonic() * 1000) & 0xFFFFFFFF` | LabVIEW's exact epoch only if Q5 requires it |

### 10.2 Thread and queue layout

```
                    client  (127.0.0.1 : port)
                          ▲    │
                          │    ▼
             ┌───────────────────────────────────┐
             │ tcp thread                        │
             │ accept → read frame → decode      │
             │ → route → commands history        │
             └─────┬─────────────────────┬───────┘
         Z set     │                     │   camera set (+ receiving timestamp)
                   ▼                     ▼
          z_q (unbounded)         cam_q (unbounded)       pending stream / AF / RF steps are run by
                   │                     │                the cameras thread itself (no self-enqueue)
                   ▼                     ▼
          ┌────────────────┐  Exit  ┌─────────────────────┐
          │ z thread       │ ◀───── │ cameras thread      │
          │ handlers (08)  │        │ handlers (05, 06)   │
          └───────┬────────┘        └──────────┬──────────┘
                  │                            │
                  │ replies                    │ replies
                  └────────────┬───────────────┘
                               ▼
          ClientLink.send(): one lock, one whole frame at a time ──▶ client
  shared state: ServerState (RLock) replaces the front-panel objects
  shutdown: Exit → cameras thread ends → stop event + queues destroyed → tcp and z threads end
  Z hardware: one ZStage link (lock per GCS request/answer) shared by the z thread and the focus code
```

### 10.3 Reference sketch

Reference sketch. It was run against a fake client: routing, an unknown command, an ignored heartbeat frame, a failing handler,
an undecodable command payload, camera and Z replies overtaking each other, a client that stops reading, reconnection, `Exit`
shutdown, queue bound and destroy. Frame reading and payload decoding are injected (`read_frame`, `is_command`, `decode`) because
they belong to [02-tcp-protocol.md](02-tcp-protocol.md) and [04-data-types.md](04-data-types.md). The two command sets come from the
real enum order (Q1). Handlers and reply encoding are in chapters 03, 05, 06 and 08.

```python
# Reference sketch: server skeleton with threads (wire format in 02 and 04, handlers in 05, 06, 08).
from __future__ import annotations

import collections
import logging
import select
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

log = logging.getLogger("alox.server")


def tick_ms() -> int:                                     # LabVIEW Tick Count (ms), U32
    return int(time.monotonic() * 1000) & 0xFFFFFFFF


IO_TIMEOUT_S = 5.0                                        # LabVIEW: 5000 ms payload read, 1000 ms write


class QueueDestroyed(Exception):
    """get()/put() on a destroyed queue (LabVIEW: Release Queue with force destroy = TRUE)."""


class LVQueue:
    """FIFO with an optional bound (0 = unbounded) and destroy() that wakes every waiter."""

    def __init__(self, maxsize: int = 0):
        self._items: collections.deque = collections.deque()
        self._maxsize, self._destroyed = maxsize, False
        self._cond = threading.Condition()

    def put(self, item: Any, timeout: Optional[float] = None) -> None:
        with self._cond:
            room = lambda: self._destroyed or not self._maxsize or len(self._items) < self._maxsize
            if not self._cond.wait_for(room, timeout):
                raise TimeoutError("queue full")
            if self._destroyed:
                raise QueueDestroyed()
            self._items.append(item)
            self._cond.notify_all()

    def get(self, timeout: Optional[float] = None) -> Any:    # TimeoutError = LabVIEW "timed out?"
        with self._cond:
            if not self._cond.wait_for(lambda: self._destroyed or self._items, timeout):
                raise TimeoutError("queue empty")
            if self._destroyed:
                raise QueueDestroyed()
            item = self._items.popleft()
            self._cond.notify_all()
            return item

    def destroy(self) -> None:
        with self._cond:
            self._destroyed = True
            self._items.clear()
            self._cond.notify_all()


@dataclass
class Msg:                                                # Camera Queue Data (04-data-types.md)
    command: str
    notifier_name: str = ""
    command_data: Any = None
    request_ts: int = 0
    sending_ts: int = 0
    receiving_ts: int = 0


@dataclass
class ServerState:                                        # front-panel equivalents, one lock
    lock: threading.RLock = field(default_factory=threading.RLock)
    connected: bool = False
    heartbeat: int = 0
    commands_history: collections.deque = field(default_factory=lambda: collections.deque(maxlen=500))
    error_out: Any = None
    error_history: collections.deque = field(default_factory=lambda: collections.deque(maxlen=1000))
    temps: tuple = (0.0, 0.0)                             # last (HM, LM) sensor temperatures
    temp_history: collections.deque = field(default_factory=lambda: collections.deque(maxlen=10000))


class ClientLink:
    """The current client connection; one lock per frame, so frames never interleave."""

    def __init__(self) -> None:
        self.conn: Optional[socket.socket] = None
        self.lock = threading.Lock()

    def send(self, frame: bytes) -> bool:
        with self.lock:
            if self.conn is None:
                return False                              # LabVIEW: write on an invalid refnum, ignored
            try:
                self.conn.sendall(frame)                  # timeout set once at accept (IO_TIMEOUT_S)
                return True
            except OSError:                               # DEVIATION: a half-written frame would desynchronise the
                try:                                      # client, so drop the link; the reader sees EOF and cleans up
                    self.conn.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                return False


def tcp_thread(st: ServerState, link: ClientLink, port: int, read_frame: Callable, is_command: Callable,
               decode: Callable, route: Callable, stop: threading.Event, host: str = "127.0.0.1",
               ready: Optional[threading.Event] = None) -> None:
    with socket.create_server((host, port)) as srv:       # one listener for the whole run
        srv.settimeout(0.2)
        if ready is not None:
            ready.set()
        while not stop.is_set():
            try:
                conn, _ = srv.accept()                    # one client at a time
            except (socket.timeout, TimeoutError):
                continue
            conn.settimeout(IO_TIMEOUT_S)                 # one timeout for reads and writes; read_frame must keep it
            with link.lock:
                link.conn = conn
            with st.lock:
                st.connected = True
            try:
                while not stop.is_set():
                    with st.lock:
                        st.heartbeat = tick_ms()          # LabVIEW: Tick Count at every element of the TCP loop
                    if not select.select([conn], [], [], 0.05)[0]:
                        continue                          # nothing to read (LabVIEW code 56)
                    datatype, payload = read_frame(conn)  # whole frame or exception (02)
                    if not is_command(datatype):          # Heartbeat/Image/Response frames ignored
                        continue
                    try:
                        msg = decode(payload)
                    except Exception as exc:              # DEVIATION: LabVIEW routes a default element (02 §7.6)
                        log.error("undecodable command frame dropped: %s", exc)
                        continue
                    route(msg)
            except (ConnectionError, OSError, QueueDestroyed):
                pass                                      # LabVIEW: TCP Close, back to Listen
            finally:
                with link.lock:
                    link.conn = None
                conn.close()
                with st.lock:
                    st.connected = False                  # DEVIATION: LabVIEW leaves Connected TRUE


def consumer(name: str, q: LVQueue, handlers: dict, on_exit: Callable[[], None],
             after_each: Optional[Callable[[], None]] = None) -> None:
    while True:
        try:
            msg = q.get()
        except QueueDestroyed:
            return
        if msg.command == "Exit":
            on_exit()                                     # no reply to Exit
            return
        try:
            handler = handlers.get(msg.command)           # Default case: nothing, no reply
            if handler is not None:
                handler(msg)                              # replies through ClientLink.send
        except Exception:                                 # a failed command never stops the loop
            log.exception("%s command %r failed", name, msg.command)
        if after_each is not None:
            after_each()


def run_server(st: ServerState, link: ClientLink, port: int, read_frame: Callable, is_command: Callable,
               decode: Callable, cam_handlers: dict, z_handlers: dict, z_commands: frozenset,
               cam_commands: frozenset, ready: Optional[threading.Event] = None) -> None:
    cam_q, z_q = LVQueue(), LVQueue()                     # DEVIATION: unbounded (LabVIEW: 5 each)
    stop = threading.Event()

    def route(msg: Msg) -> None:
        if msg.command in z_commands:                     # "Z Setup".."Z Do Reference"
            z_q.put(msg)
        elif msg.command in cam_commands:                 # "Get Heartbeat".."Cam Get Led", "Log"
            msg.receiving_ts = tick_ms()
            cam_q.put(msg)
        else:
            log.warning("unknown command %r dropped (no reply, as in LabVIEW)", msg.command)
        with st.lock:
            st.commands_history.appendleft(msg.command)   # most recent first, unknown ones included

    def camera_history() -> None:                         # LabVIEW: Array and Temperature Graph
        with st.lock:
            st.error_history.append(st.error_out)
            st.temp_history.append((time.time(), *st.temps))

    threads = [
        threading.Thread(target=tcp_thread, name="tcp", daemon=True,
                         args=(st, link, port, read_frame, is_command, decode, route, stop),
                         kwargs={"ready": ready}),
        threading.Thread(target=consumer, name="cameras",
                         args=("camera", cam_q, cam_handlers, lambda: z_q.put(Msg("Exit")), camera_history)),
        threading.Thread(target=consumer, name="z", args=("Z", z_q, z_handlers, lambda: None)),
    ]
    for t in threads:
        t.start()
    threads[1].join()                                     # cameras thread ends on "Exit"
    stop.set()                                            # LabVIEW: Release(TCP queue) stops the TCP loop
    cam_q.destroy()
    threads[2].join()                                     # Z thread ends on the forwarded "Exit"
    z_q.destroy()
    threads[0].join(timeout=5)
    # DEVIATION: close both cameras and the PI connection here (LabVIEW does not).
```

Notes:

- With autofocus/refocus and streaming, the cameras thread uses the stepping driver of
  [06-autofocus-refocus.md](06-autofocus-refocus.md) §9.5 (non-blocking `get` while a job is pending, one step per pass) instead of
  the plain `consumer`. With `LVQueue`, the non-blocking form is `get(timeout=0)`, which raises `TimeoutError` when the queue is
  empty.
- asyncio variant: serve the socket with `asyncio.start_server` and keep the two consumers as threads. A consumer then sends a reply
  with `asyncio.run_coroutine_threadsafe(send(frame), loop)`, where `send` writes and drains under an `asyncio.Lock`. The hardware
  calls still run in the consumer threads.
- `read_frame` must return a whole frame or raise, and must not change the socket timeout: a socket has one timeout for both
  directions, and the consumer threads write on it concurrently. A read timeout inside a frame therefore closes the connection
  (the fix recommended in [02-tcp-protocol.md](02-tcp-protocol.md) §5.5, which also covers LabVIEW's per-part timeouts).
- The reply encoder sets `TCP Sending timestamp` just before calling `ClientLink.send()`. To drop replies that belong to an earlier
  connection ([02-tcp-protocol.md](02-tcp-protocol.md) §13 D10), tag each routed `Msg` with a connection counter and let `send()`
  discard a reply whose counter is not the current one.
- Handlers set `ServerState.error_out` and log as specified in [09-configuration-logging-errors.md](09-configuration-logging-errors.md).
- `host` keeps the LabVIEW loopback-only binding by default (Q6).

### 10.4 Fidelity checklist (behaviour visible to the client)

Switch names are those of [10-migration-plan.md](10-migration-plan.md) §6 and [02-tcp-protocol.md](02-tcp-protocol.md) §11.3; a
behaviour the client can observe keeps the LabVIEW default until Q2/Q4 are answered.

| Behaviour | Decision |
|---|---|
| Loopback-only listener on `port_I32` (default 503), one client at a time | keep (`listen_address`, Q6) |
| Client heartbeat/image/response frames ignored; no server heartbeat frames | keep |
| Camera commands serialised in one thread, Z commands in another | keep |
| `Command Receiving Timestamp` set at routing time, camera commands only; `TCP Sending timestamp` set just before each write | keep |
| No reply to `Exit`, `Cam Get Image`, `Cam Get Images Array` | keep |
| No reply to unknown commands and `Default` cases | keep by default (`reply_unknown_commands`, Q2) |
| Error-cluster replies suppressed when the command fails | keep by default (`send_error_replies`, Q4); recommended once confirmed: send them |
| Undecodable command payload executed as a default element | drop: log and discard the frame |
| ≥ 50 ms gap between command reads; 2 s `Listen` retry | drop (`inbound_pause_ms` = 0; performance only) |
| Deadlocks, unbounded histories, `Connected` left TRUE, no cleanup at `Exit` | drop |

---

## 11. Defects and oddities in the LabVIEW implementation

| # | Defect / oddity | Evidence | Impact | Recommendation |
|---|---|---|---|---|
| A1 | The TCP loop enqueues into its own bounded queue with an infinite timeout | [main VI frames d62–d64, d71] | possible permanent freeze (H1) | **Fix** (no TCP queue in Python) |
| A2 | Stream/AF/RF steps re-enqueue into the camera loop's own bounded queue | [main VI frames d104–d115, d181–d214] | possible permanent freeze (H2) | **Fix** ([05](05-camera-acquisition.md) §11, [06](06-autofocus-refocus.md) §9.5) |
| A3 | Cross-loop cycles through 5-element queues with infinite timeouts | [main VI frame d0], §4.4 | freeze under load (H3, H4) | **Fix** (unbounded consumer queues, direct reply writes) |
| A4 | After a successful read, the next `Read` waits for 50 ms of empty queue | [main VI frames d0, d71] | ≥ 50 ms per command; reads delayed by frequent replies | **Fix** |
| A5 | If `TCP Close` fails after a read error, `Listen` is never enqueued | [main VI frame d64] | the server never accepts a connection again and loops `Read` → error every 50 ms (inferred) | **Fix** (always go back to listening) |
| A6 | `Connected` not reset on disconnection; `Listen` overwrites the old connection refnum without closing it | [main VI frames d62, d64] | misleading status | **Fix** |
| A7 | Listener bound to `localhost` only | [main VI frame d62] | no remote client possible | **Reproduce** by default, address configurable; **ask** (Q6) |
| A8 | Unknown commands and `Default` cases are dropped silently (only `Commands History` records them) | [main VI frames d65, d73, d220] | the client waits for its timeout | **Reproduce** on the wire and log a warning; reply only if the protocol may change (Q2) |
| A9 | `Send Heartbeat` and `Quit` states never enqueued; client heartbeat frames discarded | [main VI frames d67, d68, d70] | dead code | **Reproduce** (ignore the frames); do not implement the dead states |
| A10 | Shutdown only through `Exit` routed to the camera loop and forwarded to Z; no hardware close; connection not closed; replies of commands queued before `Exit` lost | [main VI frames d78, d0, d251] | resources left to the OS; possible hang (§7.2) | **Fix**: shutdown event and explicit cleanup; still no reply to `Exit` |
| A11 | Unbounded `Array` and `Temperature Graph`, one entry per camera-loop iteration (every re-enqueued step, "no error" entries included) | [main VI frame d0] | memory growth over days | **Fix** (bounded, time-stamped) |
| A12 | `Array` samples `error out` without data dependency; `error out` has two writers | [main VI frame d0] | wrong error recorded | **Fix** |
| A13 | `Commands History` read-modified-written by two loops; flooded by `Cam Autofocus Run` | [main VI frames d0, d205] | lost entries; history useless during AF | **Fix** (lock; skip internal steps) |
| A14 | `CAM Queue Size` read once; `TCP Queue Size` taken from an unidentified output | [main VI frame d0] | wrong display | **Fix** (display only) |
| A15 | `Enqueue Send … Response` obtains the TCP queue by name at every call and never releases it | [Doc6 p.437, 445, 573] | one leaked reference per reply (inferred) | n/a in Python |
| A16 | Settings and sessions in panel objects shared through unsynchronised local variables; two diverging copies of `Z Session` | §8.2 | stale or torn reads | **Fix** (one locked state, one Z link) |
| A17 | Stage-2 PI macro check runs before any connection exists, with the inverse logic of `Z Get Status` | [main VI frame d59] | no effect in a fresh process | **Ask** (Q8); default: skip |
| A18 | Every INI read error ignored; chained errors force later keys to their defaults | [main VI frames d4–d61] | silent misconfiguration | **Fix**: per-key default plus a logged warning ([09](09-configuration-logging-errors.md)) |
| A19 | Obsolete labels: stage-2 path label, folder VI name, "Low/High Angle" wire labels | [main VI frames d3, d0; Doc6 p.411] | confusion only | n/a |
| A20 | `Unflatten From String` error unwired in `Read`: an undecodable `Command` payload is routed as a default element (inferred: `Cam Setup`) | [main VI frame d0]; [02](02-tcp-protocol.md) §7.6 | a garbled frame is executed as a real command (inferred) | **Fix**: log and drop the frame |
| A21 | `Send Response` ignores `TCP Write` errors and keeps the connection, even after a partial write | [main VI frame d69]; [02](02-tcp-protocol.md) §13 D12 | client stream desynchronised (inferred) | **Fix**: close the connection after a failed write |

---

## 12. Open questions

Only the items that change the architecture. Brackets give the matching entries of the questionnaire [open-questions.md](open-questions.md).

| # | Question | Why it matters |
|---|---|---|
| Q1 | Item order and representation of `typedef_ext_camera_camera_command.ctl`: is `Exit` between `Get Heartbeat` and `Cam Get Led`? Is `Z Do Reference` the last Z item? Where are `Log`, `Cam Autofocus Run`, `Cam Refocus Run`? [Q2, Q4] | Routing is by numeric range: a misplaced item is never executed, and `Exit` outside the camera range would make the server unstoppable |
| Q2 | Will the client stay unchanged (byte-compatible server), or may the protocol change? [Q1] | Decides whether unknown commands, failed ACKs and `Exit` may get replies |
| Q3 | Does the client ever have several commands outstanding (several callers, pipelining)? [Q4, Q28] | Decides whether the deadlocks H1–H4 can happen and how much reply reordering the client tolerates |
| Q4 | Are error-cluster replies really suppressed when the command fails? [Q4, Q17] | If the client treats timeouts as error signals, sending error replies changes its behaviour |
| Q5 | Does the client compare reply timestamps with its own `Tick Count`? | Decides whether Python must reproduce LabVIEW's millisecond counter, epoch included |
| Q6 | Is the client always on the same PC? (not yet in the questionnaire) | Loopback-only listener (A7) |
| Q7 | Does more than one server instance (`Core #`) run on a PC: one executable per core, each with its own `alox_setup.ini`, `alox.ini` and port? [Q12] | Process model, configuration lookup, shared log files ([09](09-configuration-logging-errors.md) D14) |
| Q8 | Saved default of the `Z Session` control (not yet in Q30), and purpose of the stage-2 macro check [Q10] | Whether stage 2 may touch the PI controller |
| Q9 | Reentrancy of `Log Server Error.vi`, the `Enqueue Send … Response` VIs, the state-machine and focus VIs; "Automatic error handling" of the main VI [Q30] | Whether parallel loops were serialised, and whether unwired errors opened dialogs |
| Q10 | Is a GUI needed (live image, histories, `Show UI`)? [Q15] | Whether §8.3–§8.4 need a display layer |
