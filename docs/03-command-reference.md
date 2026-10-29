# Command reference (API catalogue)

**Summary.** The client controls the ALOX Camera Server with commands that are items of the enum
`typedef_ext_camera_camera_command.ctl`. 46 item names are known from the case labels of the main VI; the complete item list, its
order and its numeric codes are not printed **(unknown)**. Each command travels in a `Command` frame as a flattened `Camera Queue Data` cluster
`{Camera Command, Notifier Name, Command Data (variant), 3 × U32 timestamps}`. The TCP loop routes it by **enum range** to the camera
consumer loop or to the Z consumer loop, or drops it. That loop decodes `Command Data`, acts on the hardware, and normally sends one
reply carrying the request's `Notifier Name`. This chapter is the API contract. It lists every command, including `Exit`, the internal
`… Run` commands, the `Default` cases and the values that are routed nowhere. It explains the routing rule and why the enum order is part
of the contract. Each command then gets a specification at API level: inputs, observable behaviour (hardware calls, state changes,
front-panel side effects, log text), reply, and errors. Algorithms are specified in the subsystem chapters, which are linked from each
command.

**Sources.** `alox_camera_server.vi` (main VI, modified 25/07/2025). Router: [main VI frames d0, d65–d67; p.3, 6]. Camera-loop
command cases: [main VI frames d73–d214; p.8, 11, 14, 17–28]. `Cam Set Exposure` is the case displayed in frame d0, with its hidden
frames d124–d140 [p.3, 5]. Camera-loop timeout frame: [main VI frames d217–d219; p.29]. Z-loop command cases: [main VI frames
d220–d249; p.6, 9, 12, 15], plus its `Exit` case in [main VI frame d0]. Typedefs: `typedef_ext_camera_mgmt_queue_data.ctl`
[Doc6 p.285–288]; `typedef_ext_camera_camera_command.ctl` [Doc6 p.422–425]; `typedef_ext_camera_log_data.ctl` [Doc6 p.289–292];
camera command typedefs [Doc6 p.341–404]; `typedef_ext_camera_stream_data.ctl` [Doc6 p.540–543];
`typedef_ext_camera_autofocus_data.ctl` [Doc6 p.599–602]; `typedef_ext_camera_refocus_data.ctl` [Doc6 p.612–615]; Z command
typedefs `typedef_ext_z_*` [Doc6 p.277–284, 293–296, 325–340, 447–450; pages per typedef in §7]; `alox_typedef_ext_zstage_session.ctl` [Doc6 p.644–647];
`alox_typedef_ext_zstage_properties.ctl` [Doc6 p.652–655]. Sub-VIs called by the cases: `Enqueue Send Error Cluster Reponse.vi`
[Doc6 p.435–438]; `Enqueue Send STR / Boolean / DBL / U32 / DBL 2D Array / Image Response.vi` [Doc6 p.443–446, 455–458, 463–466,
559–562, 1124–1127, 571–574]; `Log Server Error.vi` [Doc6 p.430–434]; `Log Server Infos.vi` [Doc6 p.1218–1221];
`alox_z_check_axis_reference.vi` [Doc6 p.139–143]; `Clear Errors.vi` [Doc6 p.57–59]. PI GCS VIs: Mercury_GCS_Configuration_Setup
[Doc6 p.111], GCSTranslateError [p.148], General wait for movement to stop [p.159–161], Wait for axes to stop [p.210–212],
FRF? [p.224], RMC? [p.405], MAC START [p.413], JON? [p.664], ERR? [p.706], VEL [p.716], Close connection if open [p.1191], JAX [p.1200],
JON [p.1204], STP [p.1208], and POS?, VEL?, MOV, MVR, FRF and FNL from the same library. Refocus `Start` step: [Doc6 p.1034].

Related chapters: [01-architecture.md](01-architecture.md) (loops, queues, shutdown), [02-tcp-protocol.md](02-tcp-protocol.md)
(frames, reply encoding), [04-data-types.md](04-data-types.md) (byte layouts of every payload), [05-camera-acquisition.md](05-camera-acquisition.md),
[06-autofocus-refocus.md](06-autofocus-refocus.md), [07-focus-metrics.md](07-focus-metrics.md), [08-z-stage.md](08-z-stage.md),
[09-configuration-logging-errors.md](09-configuration-logging-errors.md), [10-migration-plan.md](10-migration-plan.md).

---

## 1. Conventions

- Untagged statements are observed on the printouts. **(inferred)** marks a deduction from LabVIEW/NI/PI semantics or from
  cross-checking; **(unknown)** marks what the printouts do not show. `[main VI frame dN]` is hidden frame N of the HTML export of the
  main VI; `[main VI p.N]` is a page of its PDF; `[Doc6 p.N]` is a page of "Labview Document 6".
- **HM** / **LM** = the `"High Mag."` / `"Low Mag."` camera. Wire labels such as "Low Angle Session" (= HM) are obsolete.
- Exact enum items, strings and labels are in `code font`. A string written `"Cam stop"` is reproduced byte for byte.
- Pseudo-code notation:

| Notation | Meaning |
|---|---|
| `VTD(T)` | `Variant To Data(Command Data, type T)`. "VTD error unwired" means that a payload of the wrong type is **not** reported: the command runs with a default value of `T` (`Camera Type = "High Mag."`, 0, FALSE, "") **(inferred, LabVIEW semantics)**. Whether non-zero typedef defaults (for example 5000 for `Acquisition Timeout (ms)`) apply or plain zeros is **(unknown)** ([04-data-types.md](04-data-types.md) §5.1). "VTD error wired" means that the conversion error enters the command's error chain, so the error is returned and the hardware nodes after it do nothing; nodes without an error input (`Bundle`, local-variable writes) still run with the default. |
| `ACK(e)` | `Enqueue Send Error Cluster Reponse.vi`: reply = the error cluster alone |
| `REPLY_STR / _BOOL / _DBL / _U32 / _DBL2D / _IMAGE (v, e)` | `Enqueue Send <type> Response.vi`: reply = `{value, error}` |
| `LSE(extra, e)` | `Log Server Error.vi` with `Extra String = extra`. It writes a log line only when `e.status = TRUE`, then copies `e` into the `error out` indicator (§4.4) |
| `decorate(e, s)` | if `e.status = TRUE`: `e.source ← e.source + s` (no separator). Warnings (status FALSE, code ≠ 0) are not decorated |
| `S`, `A`, `zs` | `Z Session.System` (PI system number), `Z Session.Axes` (GCS axis ids), `Z Stage Settings` (read once per Z message) |
| tick | `Tick Count (ms)`, a free-running U32 millisecond counter |

### 1.1 Reply kinds

| Kind | Reply VI | Content of `Response (Flattened)` | Sent when the command failed? |
|---|---|---|---|
| **ACK** | `Enqueue Send Error Cluster Reponse.vi` [Doc6 p.435–438] | `{status, code, source}` | **Probably not (inferred, strong).** The VI's `error in` wire runs behind the `Flatten To String` node into the `error in` of its `Enqueue Element`, and `error out` comes from that `Enqueue` [Doc6 p.437]. A LabVIEW queue primitive does nothing when its `error in` carries an error, so an error ACK is never queued and the client waits until its own timeout. Successes and warnings are sent. See [02-tcp-protocol.md](02-tcp-protocol.md) §8.4. |
| **STR, BOOL, DBL, U32, DBL2D** | `Enqueue Send <type> Response.vi` | `{value, error}` | yes. The error is data. The value is the type default (`""`, FALSE, 0.0, 0, empty array) when the node that produces it failed **(inferred)**. |
| **IMAGE** | `Enqueue Send Image Response.vi` [Doc6 p.571–574] | `{IMAQ flattened image, error}`; copies the 3 request timestamps | yes. On error the image string is empty **(inferred)**. |
| **none** | — | — | The client waits until its own timeout. |

Every reply carries the request's `Notifier Name` unchanged. All reply timestamps are 0 except in IMAGE replies, and the TCP loop fills
`TCP Sending timestamp` for every reply. Byte layouts: [02-tcp-protocol.md](02-tcp-protocol.md) §8. In this chapter "ACK" always means
"ACK, subject to the caveat above".

---

## 2. Catalogue

Payload typedef names are the cluster labels; byte layouts are in [04-data-types.md](04-data-types.md). "§" points to the detailed
subsection.

| Command (exact item) | Loop | `Command Data` | Reply | Purpose / notes | § |
|---|---|---|---|---|---|
| *(value outside both routes, not `Log`)* | none: router `Default` | — | none | "Unknown command": dropped, recorded in `Commands History` | 3.4 |
| `Get Heartbeat` | camera | not read | U32 | server tick; the application-level heartbeat | 5.1 |
| `Show UI` | camera | bare Boolean | ACK | show/hide the server window | 5.2 |
| `Log` | camera (explicit route) | `Cam Log Data` | ACK | stores log parameters that nothing reads | 5.3 |
| `Exit` | camera, forwarded to Z | not read | none | stops the server | 5.4 |
| `Cam Setup` | camera | `Camera Init Data` | ACK | open and configure one camera | 5.5 |
| `Cam Start` | camera | `Camera Start Data` | ACK | trigger mode, start acquisition | 5.6 |
| `Cam Stop` | camera | `Camera Stop Data` | ACK | stop acquisition if running | 5.7 |
| `Cam Cleanup` | camera | `Camera Release Data` | ACK | unconfigure, close, reset session | 5.8 |
| `Cam Get Acquisition Status` | camera | `Camera Status Data` | BOOL | `AcquisitionInProgress` | 5.9 |
| `Cam Software Trigger` | camera | `Camera Trigger Soft Data` | ACK (always OK) | GigE only | 5.10 |
| `Cam Set Exposure` | camera | `Camera Set Exposure Data` | ACK | clamp, exposure, frame rate | 5.11 |
| `Cam Get Exposure` | camera | `Camera Get Exposure Data` | DBL | | 5.12 |
| `Cam Set Gain` | camera | `Camera Gain Data` | ACK | no clamp | 5.13 |
| `Cam Get Gain` | camera | `Camera Get Gain Data` | DBL | | 5.14 |
| `Cam Stream Next Image` | camera, self re-enqueued | `Camera Stream Data` | IMAGE, once | the image request | 5.15 |
| `Cam Get Image` | camera | `Camera Get Image Data` | none | refreshes the server display only | 5.16 |
| `Cam Get Images Array` | camera | not read | none | "TO BE DEFINED": empty case | 5.17 |
| `Cam Flip` | camera | `Camera Flip Data` | ACK | software flip flags | 5.18 |
| `Cam Set FlatField` | camera | `Camera Ext Set FlatField` | ACK | flat-field settings, reload images | 5.19 |
| `Cam Set Led` | camera | `Camera Set LED Data` | ACK (raw error) | LED/bulb through camera outputs | 5.20 |
| `Cam Get Led` | camera | `Camera Get LED Data` | BOOL | | 5.21 |
| `Cam Autofocus` | camera | `Camera Autofocus Data` | ACK | first AF step, queues `Cam Autofocus Run` | 6.1 |
| `Cam Refocus` | camera | `Camera Refocus Data` | ACK | first RF step, queues `Cam Refocus Run` | 6.2 |
| `Cam Autofocus Run` | camera (internal) | `Camera Autofocus Data` | none | one AF step, re-enqueued until finished | 6.3 |
| `Cam Refocus Run` | camera (internal) | `Camera Refocus Data` | none | one RF step, re-enqueued until finished | 6.3 |
| `Cam Get Autofocus Data` | camera | `Camera Get Autofocus Data` | DBL2D | last focus table `[Z, score]` | 6.4 |
| `Cam Get Autofocus Progress` | camera | `Camera Get Autofocus Data` (same typedef) | DBL | progress in % | 6.4 |
| *(routed to the camera loop, no case)* | camera `Default` | — | none | nothing happens | 3.4 |
| `Z Setup` | Z | not read | ACK | open the PI link (may reference the axis), set speed | 7.2 |
| `Z Get Serial` | Z | `Z Stage Get Serial Data` | STR | identity or serial number (no I/O) | 7.3 |
| `Z Get Status` | Z | not read | BOOL | controller error flag; may start macro `DH` | 7.4 |
| `Z Get Error` | Z | not read | STR | controller error text | 7.5 |
| `Z Set Speed` | Z | `Z Stage Set Speed Data` | ACK | `VEL` | 7.6 |
| `Z Get Speed` | Z | `{Extra Param}` (ignored) | DBL | `VEL?` | 7.7 |
| `Z Get Reference` | Z | `{Extra Param}` (ignored) | BOOL | `FRF?` | 7.8 |
| `Z Do Reference` | Z | bare enum `FNL` / `FRF` | ACK | reference move, blocks until stopped | 7.9 |
| `Z Get Position` | Z | `{Extra Param}` (ignored) | DBL | `POS?` | 7.10 |
| `Z Move Absolute` | Z | `Z Move Abs Data` | ACK | range check, `MOV`, no wait | 7.11 |
| `Z Move Relative` | Z | `Z Stage Move Rel Data` | ACK | range check, `MVR`, no wait | 7.12 |
| `Z Wait End Move` | Z | `Z Stage Wait Data` | ACK | wait for the end of motion | 7.13 |
| `Z Stop` | Z | `{Extra Param}` (ignored) | ACK | `STP`; may start macro `DH` | 7.14 |
| `Z Enable Joystick` | Z | `Z Stage Joystick Data` | ACK | `JAX` + `JON 1` | 7.15 |
| `Z Disable Joystick` | Z | `Z Stage Joystick Data` | ACK | `JAX` + `JON 0` | 7.15 |
| `Z Joystick Enabled ?` | Z | `Z Stage Joystick Data` | BOOL | `JON?` | 7.16 |
| `Z Set Min Limit` | Z | bare DBL | ACK | software limit, memory only | 7.17 |
| `Z Set Max Limit` | Z | bare DBL | ACK | software limit, memory only | 7.17 |
| `Z Cleanup` | Z | not read | ACK | close the PI link, reset session | 7.18 |
| `Exit` (Z case) | Z | not read | none | stops the Z loop; reached through the camera loop's `Exit` **(inferred)** | 7.19 |
| *(routed to the Z loop, no case)* | Z `Default` | — | none | nothing happens | 3.4 |

The camera loop has 27 named cases plus `Default`, and the Z loop has 19 Z cases plus `Exit` and `Default`
[main VI frames d73–d214, d220–d249, d0].

---

## 3. Routing and the `Camera Command` enum

### 3.1 Routing rule [main VI frames d0, d65, d66; p.3, 6]

The TCP loop decodes a `Command` frame with `Unflatten From String(Camera Queue Data)` and selects a case on `Camera Command`:

| Case selector (exact) | Diagram label | Action |
|---|---|---|
| `"Z Setup".."Z Do Reference"` | "Z Consumer loop for Z stage commands" | `Enqueue(Z queue "Z Stage Mgr", msg)`; timestamps unchanged |
| `"Get Heartbeat".."Cam Get Led", "Log"` | "Camera consumer loop for camera and general commands"; "Store receiving timestamp for image statistics" | `msg.Command Receiving Timestamp ← tick`, then `Enqueue(camera queue "Camera Mgr", msg)` |
| `Default` | "Unknown command" | nothing |

A LabVIEW case label `"A".."B"` is an **inclusive range of numeric enum values**, not a list of names. Routing therefore depends on the
numeric order of the enum items. LabVIEW refuses a value that belongs to two cases, so the two ranges are disjoint **(inferred, LabVIEW
semantics)**. Both `Enqueue` calls wait forever: while the target queue holds 5 elements the TCP loop blocks, and so do reading and all
replies ([01-architecture.md](01-architecture.md) §4.5).

### 3.2 Why the item order matters

The printout of `typedef_ext_camera_camera_command.ctl` shows only the control label `Function` and its default value
`Get Heartbeat` [Doc6 p.423]: neither the item list nor the order, values or representation is printed. The 46 names come from the
case labels of the main VI, so the enum could contain further items **(unknown)**. The routing ranges give these constraints:

1. **Camera commands.** Every command handled by the camera loop, except `Log`, must lie in `[Get Heartbeat ; Cam Get Led]`. Otherwise
   its case is dead code and the client times out. That includes `Show UI`, `Exit`, `Cam Setup` and `Cam Start` **(inferred)**.
2. **The case frame order is not the enum order.** In the camera case structure the frames are ordered `Default`, `Show UI`, `Log`,
   `Exit`, `Cam Setup`, `Cam Start`, `Get Heartbeat`, `Cam Get Image`, … `Cam Get Led`, `Cam Autofocus Run`, `Cam Refocus Run`
   [main VI frames d73–d214]. If that were the enum order, `Cam Setup` could never be routed. Frame order is therefore no evidence.
3. **Z commands.** All 19 must lie in `[Z Setup ; Z Do Reference]`. In the Z case structure `Z Do Reference` is the 8th of 19 frames
   [main VI frames d221–d249]. Given point 2 this proves nothing, but if the enum followed that order, the 11 commands from
   `Z Get Position` to `Z Cleanup` would never be routed. Working assumption: `Z Do Reference` is the last Z item **(inferred, must be
   confirmed)**.
4. **`Log`** lies outside both ranges; that is why it is listed explicitly.
5. **`Exit`** has a case in both loops, but only one of them can be reachable. The camera case says "Forward "Exit" command to Z consumer
   loop" [main VI frame d78], so `Exit` is in the camera range **(inferred)**. If it were in the Z range, a client `Exit` would stop
   only the Z loop and the server would never terminate.
6. **`Cam Autofocus Run` / `Cam Refocus Run`** are "only used internally" [main VI frames d205, d210]; their position is **(unknown)**.
   Inside the camera range a client could send them (one step executes from the stored state, with no reply); outside it they are
   dropped.
7. **Values.** The `.ctl` default is `Get Heartbeat` [Doc6 p.423]. A new enum control defaults to its first item, so `Get Heartbeat`
   = 0 is plausible **(inferred, weak)**: defaults can be edited, as the default `Cam Setup` inside `Camera Queue Data` shows
   [Doc6 p.286]. The representation is presumed U16 **(unknown)**.

The numeric codes must come from the `.ctl` file (Edit Items) or from a network capture ([open-questions.md](open-questions.md) Q2,
Q4). The re-implementation must derive both the codec table and the routing table from that one ordered list.

### 3.3 Side effects of the router

For every successfully read `Command` frame, whatever its value [main VI frame d0]:

- front-panel `Command` ← the decoded value; `Commands History` ← the first 500 of `[value] + Commands History` (newest first);
- camera route only: `Command Receiving Timestamp` ← tick (the client's value is overwritten). Re-enqueued elements (stream polls,
  `… Run` steps) keep the value set at the first routing;
- frames of any other datatype (`Heartbeat`, `Image`, `Response`) are read and ignored [main VI frame d67]; a `Heartbeat` frame that
  carries a payload makes the read fail and the connection is closed ([02-tcp-protocol.md](02-tcp-protocol.md) §9).

A payload that does not decode is not rejected: the `Unflatten` error is unwired, so a default element (`Cam Setup`, empty variant) is
routed and executed **(inferred)** ([02-tcp-protocol.md](02-tcp-protocol.md) §7.6).

### 3.4 Values that produce nothing

| Situation | Where | Behaviour |
|---|---|---|
| Value outside both ranges and ≠ `Log` | router `Default` [main VI frame d65] | dropped: no reply, no log. It is still recorded in `Commands History`. |
| Value in the camera range without a case | camera `Default` [main VI frame d73] | nothing; no reply, no log. The camera-loop iteration still appends to the `Array` and `Temperature Graph` histories. |
| Value in the Z range without a case | Z `Default` [main VI frame d220] | nothing; no reply, no log |
| Camera commands still queued when `Exit` is processed | camera queue destroyed after the loop | never executed, no reply |
| Replies still waiting in the TCP queue when `Exit` is processed | TCP queue destroyed after the camera loop | never sent |
| Z commands queued before the forwarded `Exit` | Z loop | executed, but a reply produced after the TCP queue was destroyed is lost: the reply VI's `Obtain Queue` fails. This is the normal case ([01-architecture.md](01-architecture.md) §7.1) |

Each of the 46 known names has a case, so the two `Default` cases matter only for enum items that are not known, or if the item
order puts a name in the wrong range **(inferred)**.

### 3.5 Reference sketch: routing

The ranges are evaluated once, from the ordered item list, into an explicit code → loop table. On the wire this behaves exactly like
the LabVIEW selector for every code, and it meets the "route by explicit name sets" recommendation of
[04-data-types.md](04-data-types.md) §10 (T2).

```python
# Reference sketch: routing table derived ONCE from the ordered item list of typedef_ext_camera_camera_command.ctl.
# The real order is (unknown): fill the list from the .ctl ("Edit Items...") in value order (questionnaire Q2).
from typing import Sequence

def build_routes(items: Sequence[str]) -> dict[int, str]:
    """Code -> 'z' | 'camera', exactly as the LabVIEW case labels "Z Setup".."Z Do Reference" and
    "Get Heartbeat".."Cam Get Led", "Log" (inclusive numeric ranges). A code missing from the map is dropped
    ("Unknown command": no reply, but still recorded in Commands History)."""
    code = {name: i for i, name in enumerate(items)}
    z = range(code["Z Setup"], code["Z Do Reference"] + 1)
    cam = set(range(code["Get Heartbeat"], code["Cam Get Led"] + 1)) | {code["Log"]}
    if cam & set(z):
        raise ValueError("ranges overlap: LabVIEW would refuse this case structure")
    return {**{c: "z" for c in z}, **{c: "camera" for c in cam}}

def self_check(items: Sequence[str], routes: dict[int, str],
               camera_cases: set[str], z_cases: set[str]) -> list[str]:
    """Start-up check (defect C1): every client command that has a case must reach the loop holding that case.
    Pass the 24 camera-range names (Exit included) + "Log", and the 19 Z names; leave out the internal "... Run" commands."""
    code = {name: i for i, name in enumerate(items)}
    return ([f"{n}: case unreachable" for n in sorted(camera_cases) if routes.get(code[n]) != "camera"] +
            [f"{n}: case unreachable" for n in sorted(z_cases) if routes.get(code[n]) != "z"])
```

With a synthetic order in which `Z Do Reference` sits 8th of the 19 Z items (the case-frame order), `self_check` reports the 11 Z
commands of §3.2 point 3 as unreachable. To apply the fix of C16, delete the codes of `Cam Autofocus Run` and `Cam Refocus Run` from
the table after building it.

---

## 4. Execution pattern shared by the commands

### 4.1 Camera commands [main VI frames d73–d214]

```
d ← VTD(<typedef of the command>)               # error wired only where stated
select camera by d.Camera Type: HM session register + High Mag Settings, or LM + Low Mag Settings
do the work: IMAQdx property nodes and VIs, each starting from "no error" (error in unwired)
err' ← decorate(err, <suffix>)
error out ← LSE(<extra>, err')                   # Server Type "Camera", Command and (usually) Camera Type wired
reply(err')                                      # reply kind of §1.1; order of these two steps: §4.2
stop ← FALSE                                     # TRUE only in Exit
```

- With a valid `Camera Type`, `"High Mag."` selects HM and `"Low Mag."` selects LM. The selection differs between commands: some
  case structures have `"High Mag."` as `Default`, some test `== "High Mag."` and fall back to LM, and some have no `Default` at all
  [main VI frames d79, d116, d148, d162, d170, d175, d181, d191]. What an out-of-range value then does is **(unknown)**; it matters only
  for corrupted payloads.
- The VTD error is **wired** in `Log`, `Cam Flip`, `Cam Autofocus`, `Cam Refocus`, both `… Run`, `Cam Get Autofocus Data`,
  `Cam Get Autofocus Progress`, `Cam Set Led` and `Cam Get Led`, and **unwired** in every other camera command that reads data.
- `Log Server Error` usually receives `Camera Type` from the payload. It is unwired in `Get Heartbeat`, `Show UI`, `Log` and in the
  HM branches and the LM `Finished` branch of `Cam Stream Next Image`, so those lines get the default `"High Mag."` (log prefix
  `ERR (HI MAG)`).
- Every camera-loop iteration, whatever the command, appends `error out` to `Array` and the two temperatures to `Temperature Graph`
  ([01-architecture.md](01-architecture.md) §5.3).

### 4.2 Order of reply and decoration (what the client sees in `source`)

| Variant | Commands | Error sent to the client | Error logged |
|---|---|---|---|
| A, standard | `Show UI`, `Log`, `Cam Setup`, `Cam Start`, `Cam Stop`, `Cam Cleanup`, `Cam Get Acquisition Status`, `Cam Set Exposure`, `Cam Set FlatField`, `Cam Set Gain`, `Cam Get Exposure`, `Cam Get Gain`, `Cam Flip`, `Cam Autofocus`, `Cam Refocus` | the decorated error | the decorated error |
| A′ | `Cam Software Trigger` | always "no error" (errors cleared) | nothing |
| B | `Get Heartbeat`, `Cam Get Autofocus Data`, `Cam Get Autofocus Progress` | the raw error (constant "no error" for the heartbeat; the VTD error for the other two) | only the reply VI's own queue error, decorated |
| C | `Cam Set Led` | the raw error (ACK sent before decoration) | the ACK VI's `error out` (= the raw error), decorated |
| D | `Cam Get Led` | the raw error (BOOL) | the reply VI's queue error; its `source` is rebuilt from the raw error's source + suffix |
| E | `Exit`, `Cam Get Image`, `Cam Get Images Array`, `Cam Stream Next Image`, `Cam Autofocus Run`, `Cam Refocus Run` | specific, see each command | specific |

Resolution of a conflict between analyses: `Cam Get Gain` was once classified as variant B. A re-reading of [main VI frame d162]
shows the decorated error feeding both `ENQUEUE DBL RESP` and `SERVER LOG ERROR`, so it is variant A, like `Cam Get Exposure` [d155].

### 4.3 Z commands [main VI frames d220–d249]

```
zs ← Z Stage Settings (local variable, read once per message)
no error in on the first node of the case; PI VIs do nothing when their error in carries an error
… PI GCS calls on (S, A) …
reply(err) ; error out ← LSE(<label>, err) with Server Type "Z Stage"    # Camera Type unwired; Command wired only in Z Do Reference (no effect)
stop ← FALSE                                                             # TRUE only in Exit
```

- No source decoration is done in the Z loop.
- The Z loop executes one command at a time. `Z Setup`, `Z Do Reference`, `Z Wait End Move` and a `Z Move …` that re-initialises the
  axis block it for seconds to minutes. Further Z commands wait in the 5-element Z queue, and then the TCP loop blocks.
- PI frame strings, timeouts and the controller error model: [08-z-stage.md](08-z-stage.md).

### 4.4 Logging

`Log Server Error.vi` [Doc6 p.430–434] writes only when `error in.status = TRUE` and returns `error in` unchanged. The line is
`<dd/mm/yyyy - hh:mm:ss + 3-digit fraction> - ERR (HI MAG)-` (or `ERR (LOW MAG)-`, `ERR (Z STAGE)-`), followed by the command name
(camera server only) and `(<code>) : <source> (<Extra String>)`. It goes to a front-panel string indicator and to the daily file
`logs\<yymmdd>_camera_events.log` or `_z_events.log`. `Log Server Infos.vi` (used only by `Z Move …`) always writes. File location,
the "overwrite at file start" defect and the inverted HI/LO labels of `Log Server Infos`:
[09-configuration-logging-errors.md](09-configuration-logging-errors.md). Each subsection below gives the `Extra String` ("extra")
and the source suffix.

### 4.5 Reference sketch: finishing a camera command

```python
# Reference sketch: decoration, logging and reply ordering of camera commands (variants A, B and C of section 4.2).
from dataclasses import dataclass, replace
from typing import Callable

@dataclass(frozen=True)
class LVError:
    status: bool = False
    code: int = 0
    source: str = ""

def decorate(err: LVError, suffix: str) -> LVError:
    """Append suffix only when status is TRUE (warnings are left untouched); no separator."""
    return replace(err, source=err.source + suffix) if err.status else err

def finish(variant: str, err: LVError, suffix: str, extra: str,
           log: Callable[[str, LVError], None], reply: Callable[[LVError], LVError]) -> LVError:
    """reply(e) queues the reply and returns the reply VI's error out:
    the ACK VI returns e itself; typed reply VIs return only their queue error (normally no error)."""
    if variant == "A":                  # decorated error is both replied and logged
        e = decorate(err, suffix)
        log(extra, e)                   # Log Server Error writes only if e.status
        reply(e)
        return e
    out = reply(err)                    # B (typed reply) and C (ACK): the raw error is sent first
    e = decorate(out, suffix)
    log(extra, e)
    return e
```

---

## 5. General and camera commands

### 5.1 `Get Heartbeat` [main VI frames d96–d97; p.14]

- **Data:** not read.
- **Behaviour:** `v ← tick`; `REPLY_U32(v, constant "no error")`. If queueing the reply fails, that error is decorated with
  `"Get Heartbeat"` and logged (extra `"Heartbeat"`, `Camera Type` unwired). Diagram comment: "Envoie le heartbeat du serveur vers le
  client" (sends the server heartbeat to the client).
- **Reply:** U32 = server tick, error always "no error".
- **Notes:** this command is the only liveness check the client has; the server never sends heartbeat frames
  ([02-tcp-protocol.md](02-tcp-protocol.md) §9). It is executed by the camera loop, so a slow camera command or an AF step delays it.
  Keep it in the camera worker so that it keeps measuring that worker's responsiveness.

### 5.2 `Show UI` [main VI frames d74–d75; p.8]

- **Data:** a bare Boolean (`show`). VTD error unwired, so a non-Boolean payload hides the window **(inferred)**.
- **Behaviour:** `Open VI Reference(own path)` → `FP.Transparency ← (show ? 0 : 100)` → `Close Reference`. The window was opened
  at start-up, with transparency 100 in the run-time executable ([01-architecture.md](01-architecture.md) §3.2). Diagram label:
  "Show Server UI".
- **Reply:** ACK with the error of the property chain. Extra `"Show UI"`, suffix `"Cam Show UI"`, `Camera Type` unwired.
- **Python:** show or hide the diagnostic GUI if there is one; otherwise reply "no error".

### 5.3 `Log` [main VI frames d76–d77; p.8]

- **Data:** `Cam Log Data` = {`Log`: Boolean = FALSE, `Path`: path = empty, `Max Lines`: integer = 0, `Max Count`: integer = 0}
  [Doc6 p.289–292]. VTD error wired.
- **Behaviour:** writes the front-panel control `Cam Log Data`. **Nothing reads this control.** Diagram comments: "Command to setup log",
  "#TODO: Log every cameras/Z actions into files ...".
- **Reply:** ACK with the VTD error. Extra `"Log"`, suffix `"Cam Log"`, `Camera Type` unwired.
- **Routing:** reaches the camera loop through its explicit `"Log"` label (§3.1).

### 5.4 `Exit` [main VI frame d78; p.8; Z case in frame d0]

- **Data:** not read.
- **Behaviour (camera loop):** `Enqueue(Z queue, constant Camera Queue Data with Camera Command = "Exit")`, with the other fields at
  their defaults (empty notifier, empty variant), timeout −1. Then `stop ← TRUE`. Diagram comments: "Exit Server", "Forward "Exit"
  command to Z consumer loop". After the loop: `Release Queue(TCP, force destroy)` and `Release Queue(camera, force destroy)`, which
  stops the TCP loop. The Z loop then reaches its own `Exit` (§7.19).
- **Reply:** none. No log. No hardware release: no `IMAQdx Close`, no PI close, no `TCP Close`.
- **Consequences:** §3.4 (queued commands). Full shutdown chain and failure modes: [01-architecture.md](01-architecture.md) §7.
- **Python:** keep "no reply" (the client cannot tell the difference), but close both cameras, the PI link and the socket.

### 5.5 `Cam Setup` [main VI frames d79–d89; p.11]

- **Data:** `Camera Init Data` = {`Camera Type`, `Camera Name`: string, `Number of buffers`: integer, `Continuous Mode`: Boolean}
  [Doc6 p.341–344]. The internal field order is **ambiguous** (front panel versus `Unbundle By Name`), see
  [04-data-types.md](04-data-types.md). VTD error unwired.
- **Behaviour:**
  1. `IMAQdx Open Camera(Camera Name)`. The previous session of that camera is neither stopped nor closed.
  2. Reads `BusType` (both cameras) and `SerialNumberHigh/Low` (HM only) into front-panel indicators; errors ignored.
  3. GigE plug type (or any value other than `USB`): `PacketSize ← 8996`. USB: nothing on HM, a 100 ms wait on LM.
  4. `IMAQdx Configure Acquisition(Continuous? = Continuous Mode ? "Continuous" : "One Shot", Number of Buffers = 5)`. The client's
     `Number of buffers` is ignored.
  5. Creates `SRC Image` (`IMAQ Create`, name `HI_SRC_…` or `LO_SRC_…`). FFC and DFC images are created and loaded from the
     settings' `Flat Field Path` / `Dark Current Path` only if `Flat Field` is TRUE, the file exists and the path is not empty;
     otherwise the session gets a null image and no error.
- **State:** `HI Inited` / `LO Inited` ← NOT (error of the Open → PacketSize → Configure chain); `High/Low Acq. State` and session
  `Acquisition State` ← `Stopped`; session `Simulation Time` ← tick; session `Session`, `SRC/FFC/DFC Image` replaced.
- **Reply:** ACK with `MergeErrors(configure chain, FFC read, DFC read)`. Extra `"Setup"`; suffix `"Cam Setup"`, plus on LM the
  concatenated sources of the Configure, PacketSize and Open errors.
- **Errors:** IMAQdx and NI Vision codes. A configured camera is the prerequisite of every other camera command.
- **Details:** [05-camera-acquisition.md](05-camera-acquisition.md) §4.2–4.3.

### 5.6 `Cam Start` [main VI frames d90–d95; p.11]

- **Data:** `Camera Start Data` = {`Camera Type`, `Triggered`: Boolean = FALSE} [Doc6 p.345–348]. VTD error unwired.
- **Behaviour:** on the GigE path, `TriggerMode ← "On"` if `Triggered` else `"Off"` (attribute
  `CameraAttributes::AcquisitionControl::TriggerMode`); the USB path writes nothing. Then `IMAQdx Start Acquisition`, which runs even
  if the trigger write failed. **Anomaly:** for HM the GigE/USB choice is taken from `Camera Sort: LED` (the plug type of the camera
  that drives the lights) instead of HM's own `Camera Plug Type` [main VI frame d91].
- **State:** none (session `Acquisition State` stays `Stopped`).
- **Reply:** ACK with `MergeErrors(start error, trigger error)`. Extra `"Start"`; suffix `"CamStart TriggerMode"` + `"HM"` or `"LM"`.
- **Details:** [05-camera-acquisition.md](05-camera-acquisition.md) §4.4.

### 5.7 `Cam Stop` [main VI frames d170–d174; p.23]

- **Data:** `Camera Stop Data` = {`Camera Type`} [Doc6 p.381–384]. VTD error unwired.
- **Behaviour:** reads `AcquisitionInProgress`; if TRUE, `IMAQdx Stop Acquisition`. If FALSE, the error of the read is discarded,
  so a camera that was never set up gives "no error".
- **State:** none. Neither the session `Acquisition State` nor `High/Low Acq. State` is updated.
- **Reply:** ACK. Extra `"Stop"`, suffix `"Cam stop"` (lower-case s).
- **Details:** [05-camera-acquisition.md](05-camera-acquisition.md) §4.5.

### 5.8 `Cam Cleanup` [main VI frames d175–d177; p.24]

- **Data:** `Camera Release Data` = {`Camera Type`} [Doc6 p.385–388]. VTD error unwired.
- **Behaviour:** `HI/LO Inited ← FALSE` (even if what follows fails); `IMAQdx Unconfigure Acquisition` → `IMAQdx Close Camera`;
  `IMAQ Dispose` of SRC, FFC and DFC (errors ignored); the session register ← a session constant shown as an icon, presumably the
  typedef defaults **(inferred)**: `Acquisition State`, `AF State` and `RF State` = `Start`, null images, empty focus data, progress 0.
- **Reply:** ACK with the Unconfigure/Close error. Extra `"Cleanup"`, suffix `"Cam Cleanup"`.
- **Details:** [05-camera-acquisition.md](05-camera-acquisition.md) §4.6.

### 5.9 `Cam Get Acquisition Status` [main VI frames d116–d118; p.19]

- **Data:** `Camera Status Data` = {`Camera Type`} [Doc6 p.353–356]. VTD error unwired.
- **Behaviour:** reads the IMAQdx session property `AcquisitionInProgress`; `Clear Errors(code −1074360316)` ("Invalid camera
  session").
- **Reply:** BOOL = `AcquisitionInProgress`. A camera that was never set up gives FALSE with no error; any other error gives FALSE plus
  the error. Extra `"Get Status"`, suffix `"Cam Acquisition status"`.
- **Details:** [05-camera-acquisition.md](05-camera-acquisition.md) §4.7.

### 5.10 `Cam Software Trigger` [main VI frames d119–d123; p.19–20]

- **Data:** `Camera Trigger Soft Data` = {`Camera Type`} [Doc6 p.357–360]. VTD error unwired.
- **Behaviour:** the camera's own `Camera Plug Type` decides. GigE or any value other than `USB`: write `ValueBool ← TRUE` on
  `CameraAttributes::AcquisitionControl::TriggerSoftware`, then clear **all** errors. USB: nothing is sent. Diagram comment: "Skip
  software trigger errror" (sic).
- **Reply:** ACK, always "no error" (errors are cleared twice). Extra `"Soft Trigger"`; the decoration (attribute name, or
  `":TriggerSoftware:"` on USB) is never reached.
- **Use:** with `Cam Start(Triggered = TRUE)`, send `Cam Stream Next Image` first (it samples the buffer counter) and the trigger
  afterwards, otherwise the request ends with 9009 **(inferred)** ([05-camera-acquisition.md](05-camera-acquisition.md) §4.3, §4.8).

### 5.11 `Cam Set Exposure` [main VI frames d0, d124–d140; p.3, 5]

- **Data:** `Camera Set Exposure Data` = {`Camera Type`, `Exposure`: DBL} [Doc6 p.361–364]. VTD error unwired. Unit: that of the
  camera attribute (µs for SFNC `ExposureTime`) **(inferred)**.
- **Behaviour:**
  1. `exp ← clamp(Exposure, Exposure Lower Limit, 2 000 000)`, bounds included. `Exposure Lower Limit` comes from the settings
     (INI default 177).
  2. Write `exp` to `Camera Exposure Attribute`, as `ValueDBL`, `ValueI64` or `ValueU32` according to `Exposure Type`. Any string
     other than `"I64"` or `"U32"` means DBL.
  3. `fr_calc ← clamp(0.95 × 10⁶ / exp, 0.5, Max Framerate)`.
  4. HM: indicator `Max Framerate` ← `fr_calc`; read the frame-rate attribute's `MaxDBL` into the indicator `MaxFR real`, and write that
     **maximum** to the frame-rate attribute. LM: write `fr_calc` only when `Framerate Type` is `"I64"` or `"U32"`; with `"DBL"` (the
     INI default) nothing is written.
  5. Writes through `ValueI64` / `ValueU32` round to the nearest integer, ties to even, and saturate at the type limits (LabVIEW
     coercion). HM `Max Framerate` is never loaded from the INI (it keeps a diagram constant whose value is **(unknown)**), so on HM
     `fr_calc` only feeds the indicator and the error text.
- **Reply:** ACK with `MergeErrors(exposure error, frame-rate error)`. Extra `"Set Exposure"`. On error the `source` is **replaced**:
  by the exposure attribute name, or by the frame-rate attribute name + the text of `fr_calc`.
- **Details and the HM/LM asymmetry:** [05-camera-acquisition.md](05-camera-acquisition.md) §5.1–5.2.

### 5.12 `Cam Get Exposure` [main VI frames d155–d161; p.21]

- **Data:** `Camera Get Exposure Data` = {`Camera Type`} [Doc6 p.373–376]. VTD error unwired.
- **Behaviour:** reads `Camera Exposure Attribute` with the accessor chosen by `Exposure Type`, converted to DBL.
- **Reply:** DBL (0.0 on error **(inferred)**) with the decorated error. Extra `"Get Exposure"`, suffix = attribute name + type string.

### 5.13 `Cam Set Gain` [main VI frames d148–d154; p.21]

- **Data:** `Camera Gain Data` = {`Camera Type`, `Gain`: DBL} [Doc6 p.369–372]. VTD error unwired.
- **Behaviour:** writes `Gain` to `Camera Gain Attribute` with the accessor chosen by `Gain Type`. There is **no clamping**. With the INI
  default `"I64"` the value is rounded to the nearest integer, ties to even (LabVIEW coercion; Python `round()`); with `"U32"` a
  negative gain saturates to 0.
- **Reply:** ACK. Extra `"Set Gain"`, suffix = attribute name + type string.

### 5.14 `Cam Get Gain` [main VI frames d162–d168; p.21–22]

- **Data:** `Camera Get Gain Data` = {`Camera Type`} [Doc6 p.377–380]. VTD error unwired.
- **Behaviour / reply:** as `Cam Get Exposure` with the gain attribute and `Gain Type`. Extra `"Get Gain"`, suffix = attribute name +
  type string, reply variant A (§4.2). Details for §5.11–5.14: [05-camera-acquisition.md](05-camera-acquisition.md) §5.

### 5.15 `Cam Stream Next Image` [main VI frames d104–d115; p.17–18]

- **Data:** `Camera Stream Data` = {`Camera Type`, `Simulation`: Boolean, `Simulation Path`: path, `Simulation Timer (ms)`: U32,
  `Acquisition Timeout (ms)`: integer = 5000} [Doc6 p.540–543]. VTD error unwired. The whole cluster is passed to the acquisition state
  machine.
- **Behaviour (each time the element is processed):**
  1. Read `CameraAttributes::DeviceControl::DeviceTemperature` → indicator `HM T° (°C)` or `LM T° (°C)`. On error the source is
     decorated with the attribute name and **the error is passed to the state machine**, which then fails the request.
  2. Run one step of `alox_camera_server_get_image_state_machine.vi`: `Acquire` if the session's `Acquisition State` is `Acquire`,
     otherwise `Start`.
  3. Outcome `Acquire` (image not ready): put **the same element** (same notifier, data and timestamps) back at the tail of the camera
     queue (timeout −1); session state `Acquire`; no reply; extra `"Enqueue Image Command"` (logs only an enqueue error).
  4. Outcome `Finished` (only reachable from `Acquire`): front-panel `Image` ← `SRC Image`; `REPLY_IMAGE(SRC Image, error)`; state
     `Stopped`; extra `"Enqueue Image Data"`, which logs only the reply-queue error, with `Camera Type` unwired (HM prefix even for LM).
  5. Any other outcome (`Error`): `REPLY_IMAGE(null image, error)`; state `Stopped`; extra `"Get Image Start"` (failure in `Start`) or
     `"Get Image Acquisition"` (failure in `Acquire`).
- **State:** session `Acquisition State` and `High/Low Acq. State`; `HM/LM T°`; `Image` display.
- **Reply:** exactly **one** IMAGE reply per request. A successful image needs at least two passes through the queue (`Start` never
  returns `Finished`); a failure in `Start` is answered on the first pass. The reply carries the request's three timestamps plus
  pre/post-flattening ticks. Other camera commands are executed between the polls.
- **Errors:** 9009 `"get image error (timeout)"` + buffer diagnostics, once `Acquisition Timeout (ms)` has elapsed since `Start`
  (queue time included); −9011 `"Invalid Type"` (simulation); temperature read errors; IMAQdx/NI Vision codes.
- **Details** (state machine, frame selection, flip, flat-field, custom data, simulation, deadlock risk):
  [05-camera-acquisition.md](05-camera-acquisition.md) §6–§8, §11.

### 5.16 `Cam Get Image` [main VI frames d98–d103; p.14]

- **Data:** `Camera Get Image Data` = {`Camera Type`} [Doc6 p.349–352]. VTD error unwired.
- **Behaviour:** if the session is not in simulation: `IMAQdx Get Image2(Buffer Number Mode = "Last New")` (default timeout 5000 ms)
  into `SRC Image`, software flip, then the front-panel `Image` display. `Clear Errors(−1074360316)`; any other error is decorated with
  `"Cam Get Image"` and written to `error out` only. In simulation: nothing.
- **Reply:** **none**, and no log line. The command only refreshes the server's own display; a client waiting for a reply times out.
  The camera loop's timeout frame would enqueue this command itself while a camera acquires ("Get Image Cmd"), but that frame never
  runs because the camera `Dequeue` has an infinite timeout [main VI frames d217–d219]; do not reimplement it.
- **Details:** [05-camera-acquisition.md](05-camera-acquisition.md) §6.5.

### 5.17 `Cam Get Images Array` [main VI frame d169; p.23]

Empty case labelled "TO BE DEFINED". No data read, no action, **no reply**, no log.

### 5.18 `Cam Flip` [main VI frames d178–d180; p.24]

- **Data:** `Camera Flip Data` = {`Camera Type`, `Horizontal Flip`: Boolean, `Vertical Flip`: Boolean} [Doc6 p.389–392]. VTD error
  wired (type constant `{High Mag., F, F}`).
- **Behaviour:** writes the two flags into `High Mag Settings` or `Low Mag Settings` (memory only, not the INI). Nothing is sent to the
  camera: the flags drive the software flip of streamed images and AF/RF grabs. The write also happens when the VTD failed, so a bad
  payload clears the HM flags **(inferred)**.
- **Reply:** ACK with the VTD error. Extra `"Flip"`, suffix `"Cam Flip"`.
- **Details** (flip semantics: "Horizontal Flip" probably flips top/bottom): [05-camera-acquisition.md](05-camera-acquisition.md) §7.2.

### 5.19 `Cam Set FlatField` [main VI frames d141–d147; p.20]

- **Data:** `Camera Ext Set FlatField` = {`Camera Type`, `Enable`: Boolean, `FFC Path`: path, `DFC Path`: path,
  `Correction Factor`: DBL} [Doc6 p.365–368]. VTD error unwired.
- **Behaviour:** settings ← {`Flat Field` = `Enable`, `Flat Field Path` = `FFC Path`, `Dark Current Path` = `DFC Path`,
  `Flat Field Factor` = `Correction Factor`} (memory only). Re-creates `SRC Image` (error ignored). Loads FFC and DFC into new
  `…FFC_` / `…DFC_` images only if `Enable` is TRUE, the path is not empty and the file exists; otherwise the session gets a null
  image **without error**.
- **Reply:** ACK with `MergeErrors(FFC read, DFC read)`. Extra `"Set FlatField"`, suffix `"Cam Set Field"`.
- **Details** (correction formula, factor 0): [05-camera-acquisition.md](05-camera-acquisition.md) §7.3.

### 5.20 `Cam Set Led` [main VI frames d195–d201; p.26–27]

- **Data:** `Camera Set LED Data` = {`Camera Type`, `Led`: Boolean} [Doc6 p.397–400]. VTD error wired into the I/O chain.
- **Behaviour:** `ledcmd ← Led XOR Settings.Invert LED` → indicator `LED cmd`. The bus path comes from `Camera Sort: LED` (INI
  `setLEDcam_I32`), but the session comes from the client's `Camera Type`.
  - USB path: `String` ← `"us"`; two chains of `LineSelector`, `LineMode = 1`, `LineSource = 1` and `LineInverter` writes, one for
    the LED line and one for the bulb line (always `HM setLEDline` and `HM setBULBline`). `LineInverter` ← `LEDStatusInverter ?
    ledcmd : NOT ledcmd` on the LED line and `BulbStatusInverter ? ledcmd : NOT ledcmd` on the bulb line (wire assignment read at
    pixel level, medium confidence). Then a `Wait LED (ms)` when `ledcmd` is FALSE. `LEDStatusInverter` is a front-panel control
    loaded from INI `lightStatusInverter_BOOL` (default TRUE); `BulbStatusInverter` is never initialised, so its saved default
    applies **(unknown)**.
  - GigE path: `String` ← `"gi"`; `LineSelector = 1`, `LineSource = 1`, `UserOutputSelector = 0`; read `UserOutputValue` (`prev`),
    then write `target = LEDStatusInverter ? ledcmd : NOT ledcmd` **(inferred: the source wire of the write is partly hidden)**;
    `Wait LED (ms)` only if `prev ≠ target`.
  - A `Write Key(lightStatusInverter_BOOL)` on the closed configuration refnum, with an unwired value, has no effect.
- **Reply:** ACK with the **raw** error, sent before decoration (variant C). Extra `"Set LED"`, suffix `"Cam Set Led"`.
- **Details** (truth table, inverters): [05-camera-acquisition.md](05-camera-acquisition.md) §9.1–9.2.

### 5.21 `Cam Get Led` [main VI frames d202–d204; p.27]

- **Data:** `Camera Get LED Data` = {`Camera Type`} [Doc6 p.401–404]. VTD error wired.
- **Behaviour:** USB path: `LineSelector ← HM setLEDline`, `LineMode ← 1`, **`LineSource ← 0`** (a write), then read `LineInverter`.
  GigE path: `LineSelector ← 1`, `LineSource ← 1`, `UserOutputSelector ← 0`, then read `UserOutputValue`. The raw value goes to the
  indicator `LEDStatus`.
- **Reply:** BOOL = `v XOR Settings.Invert LED`, with the raw error (variant D). Extra `"Get LED"`, suffix `"Cam Get Led"`. A `Set`
  followed by a `Get` returns the original `Led` only when `LEDStatusInverter` is TRUE **(inferred)**.
- **Details:** [05-camera-acquisition.md](05-camera-acquisition.md) §9.3.

---

## 6. Autofocus and refocus commands

Payload fields, state machines, progress semantics and Z interaction: [06-autofocus-refocus.md](06-autofocus-refocus.md) §2–§3. The
focus fields `AF State`, `RF State`, `Last Focus Data` and `Focus Progress (%)` live in the HM/LM camera session registers. `Last Focus
Data` and `Focus Progress (%)` are **shared** by autofocus and refocus.

### 6.1 `Cam Autofocus` [main VI frames d181–d185; p.25]

- **Data:** `Camera Autofocus Data` {`Camera Type`, `Autofocus Speed`, `Autofocus Method`, `Duration (s)`, `Z Min`, `Z Max`, `Step`,
  `Subsampling`, `ROI`, `Acquisition Timeout (ms)`} [Doc6 p.599–602]. Internal field order **(unknown)**. VTD error wired.
- **Behaviour:** runs the `Start` step of `alox_camera_server_autofocus.vi` **synchronously**: it clears `Focus Graph`, reads the
  speed (`VEL?`), sets the normal speed (`VEL`), reads the position (`POS?`) and sends `MOV Z Min` unless the stage is already within
  ±1 µm of it ([06-autofocus-refocus.md](06-autofocus-refocus.md) §4.5). The Z link used is the front-panel `Z Session` and
  `Z Stage Settings`, not the Z loop's register.
  Then: `ACK`; then `Enqueue(camera queue, same element with Camera Command = "Cam Autofocus Run")` with the ACK VI's `error out` as
  `error in`, so `Run` is queued only when the start succeeded. Diagram comments: "Start autofocus state machine", "Send ACK Response
  (Error)", "Queue "Autofocus Run command" for updating data & progress".
- **State:** session of `Camera Type`: success → `AF State` ← returned state, `Last Focus Data` and `Focus Progress (%)` ← returned
  values; failure → `AF State` ← `Error`, `Last Focus Data` ← empty, `Focus Progress (%)` ← 0.
- **Reply:** ACK (an acknowledgement of the start, not of the result). Extra `"Autofocus"`, suffix `"Cam Autofocus"`.
- **Note:** nothing prevents a second start while a job runs; the new `Start` resets the procedure while earlier `Run` elements are
  still queued ([06-autofocus-refocus.md](06-autofocus-refocus.md) §10, D4).

### 6.2 `Cam Refocus` [main VI frames d186–d190; p.25–26]

Same as `Cam Autofocus`, with `Camera Refocus Data` [Doc6 p.612–615], `alox_camera_server_refocus.vi`, state field `RF State`, queued
command `Cam Refocus Run`, extra `"Refocus"` and suffix `"Cam Refocus"`. The refocus `Start` step checks the joystick (`JON?`), reads
position and speed and sets the normal speed ([06-autofocus-refocus.md](06-autofocus-refocus.md) §5.5). An enabled joystick makes the
start fail with `{TRUE, 9031, "'refocus' warning (joystick enabled)"}` [Doc6 p.1034]: the error goes into this command's ACK
(probably not sent, §1.1), `RF State` ← `Error` and no `Run` is queued.

### 6.3 `Cam Autofocus Run` / `Cam Refocus Run` (internal) [main VI frames d205–d214; p.27–28]

- **Data:** the start command's payload, decoded again at each step (VTD error wired).
- **Behaviour:** one step of the sub-VI from the stored `AF State` / `RF State`; the session fields are written even on error. If the
  sub-VI does not report `finished`, the element is re-enqueued at the tail of the camera queue (timeout −1) with the step's error as
  `error in`. **A failed step therefore stops the job without running the `Error` (restore) state**
  ([06-autofocus-refocus.md](06-autofocus-refocus.md) §3.4, defect D1). `Cam Autofocus Run` also adds `"Cam Autofocus Run"` to
  `Commands History` at every step. In the LM frame, `Cam Refocus Run` also writes the indicator `Autofocus Data`.
- **Reply:** none. Extra `"Autofocus Run"` / `"Refocus Run"`; suffix `"Autofocus"` / `"Cam Refocus Run"`.
- **Routing:** §3.2 point 6.

### 6.4 `Cam Get Autofocus Data` / `Cam Get Autofocus Progress` [main VI frames d191–d194; p.26]

- **Data:** both use `Camera Get Autofocus Data` = {`Camera Type`} [Doc6 p.393–396]. VTD error wired.
- **Behaviour:** read the session of `Camera Type` (`== "High Mag."` → HM, else LM). No hardware access.
- **Reply:** `Cam Get Autofocus Data` → DBL2D = `Last Focus Data` (N rows × 2 columns `[Z (mm), score]`; empty before any run).
  `Cam Get Autofocus Progress` → DBL = `Focus Progress (%)`. Both are sent with the raw VTD error (variant B). Extras
  `"Autofocus Data"` / `"Autofocus Progress"`; suffixes `"Cam Get Autofocus Data"` / `"Cam Get Autofocus Progress"`. Diagram comments:
  "Send Last autofocus data", "Send autofocus progression".
- **Notes:** the client cannot read the job state or its error. It polls the progress until 100. After a failed step the progress
  freezes at the failed step's value, normally below 100 (a failure in the last measurement leaves ≥ 100). Autofocus reports 100 or
  more at its last measurement and returns to 0 between Slow points; refocus reaches 100 before its final move
  ([06-autofocus-refocus.md](06-autofocus-refocus.md) §2.6–2.8).

---

## 7. Z stage commands

### 7.1 Common facts

- Pattern: §4.3. All PI calls use `S = Z Session.System` and `A = Z Session.Axes` from the **Z loop's shift register**.
- `Z Session` = {`System`: I32, `Axes`: string array, `Identity`: string, `Serial Number`: string} [Doc6 p.644–647]. Its initial value is
  the saved default of the front-panel control `Z Session` **(unknown)**. Before a successful `Z Setup`, PI calls fail (link not open,
  or error 100 "no axis specified" for empty `Axes` **(inferred)**), except `Z Move …`: its reference check treats the failed `FRF?`
  as a lost reference and runs the whole set-up first (§7.11).
- `Z Stage Settings` = {`Com`, `Baudrate`, `Axis`, `Speed (mm/s)`, `Slow Speed (mm/s)`, `Min Position (mm)`, `Max Position (mm)`}
  [Doc6 p.652–655]. It is loaded from the INI section `[hardware.z]` (chapter 09) and modified by `Z Set Min/Max Limit`.
- `{Extra Param}` payloads: `Z Get Speed`, `Z Get Reference`, `Z Get Position` and `Z Stop` decode a cluster holding one string
  `Extra Param` (comment "not used, reserved"), never use it, and leave the decode error unwired. Two typedefs have that layout,
  `Z Stage Get Speed Data` [Doc6 p.293–296] and `Z Stage Stop Data` [Doc6 p.333–336]; the type constants show a generic icon, so which
  one each case uses is not visible [main VI frames d227, d228, d231, d241]. It does not matter on the wire: any payload is accepted.
- Units: mm and mm/s. GCS frame strings (axis `1`, LF, 4 decimals), polling, timeouts and the GCS error table:
  [08-z-stage.md](08-z-stage.md).
- The log label is the `Extra String` of `LSE` with the `ERR (Z STAGE)-` prefix and no command name.

### 7.2 `Z Setup` [main VI frame d221; p.6]

- **Data:** not read. Diagram comments: "#TODO: DEBUG WITH USB / Replace by alox.ini COM Settings", "Default speed/range settings",
  "Set Normal Speed".
- **Behaviour:** `Mercury_GCS_Configuration_Setup(System no. = zs.Axis, Interface = RS232, port = COM<max(zs.Com, 1)>,
  baud = zs.Baudrate, Reference Move = "Negative (FNL)", Timeout = 5000, All axes? = TRUE, Switch servo on? = TRUE)`. `zs.Baudrate` is
  the rate selected by the INI index `baudrate_I32` (0..3 → 9600, 19200, 38400, 115200, other → 9600; default 115200), see
  [08-z-stage.md](08-z-stage.md) §2.1. `zs.Axis` (INI `zaxis_I32`) is used as the PI **system number**, not as a GCS axis id. Depending on
  hidden options of this password-protected PI VI, it may **reference the axis**, which moves the stage to its negative limit
  **(unknown)**. Then `VEL(zs.Axis, Conn. Axes, [zs.Speed])`. The link is not closed first.
- **State:** `Z Session` ← {`System` = `zs.Axis`, `Identity`, `Serial Number`, `Axes` = connected axes}, written to the shift register
  **and** the front-panel control (which AF/RF use), even if the setup failed; `Z Inited` ← NOT error.
- **Reply:** ACK. Label `"Setup"`.

### 7.3 `Z Get Serial` [main VI frame d222; p.6]

- **Data:** `Z Stage Get Serial Data` = {`Type`: Boolean = FALSE} [Doc6 p.281–284]. VTD error wired.
- **Behaviour:** no hardware access.
- **Reply:** STR = `Type ? Z Session.Identity : Z Session.Serial Number` (the values stored by the last `Z Setup`, `""` before), with
  the VTD error. Label `"Get Serial"`.

### 7.4 `Z Get Status` [main VI frames d223–d224; p.6]

- **Data:** not read.
- **Behaviour:** `ERR?` → `ok ← error.status ? FALSE : NOT controller error`; `RMC?` → indicator `Running macros`; indicator
  `Macro running ?` ← (running == `"DH"` or `"HOME"`); if FALSE, `MAC START "DH"`. A controller error makes the `ERR?` error status TRUE,
  so `RMC?` and `MAC START` are skipped **(inferred)**.
- **Side effect:** each call may **start the controller macro `DH`**, whose content is **(unknown)** ([open-questions.md](open-questions.md)
  Q10).
- **Reply:** BOOL = `ok` (TRUE = no controller error), with the error of the whole `ERR?` → `RMC?` → `MAC START` chain. Label
  `"Get Status"`. Reading `ERR?` clears the controller's error register **(inferred, GCS semantics)**.

### 7.5 `Z Get Error` [main VI frame d225; p.6]

- **Data:** not read.
- **Behaviour:** `ERR?` → `GCSTranslateError`.
- **Reply:** STR = the GCS error description if the controller reported an error, otherwise `""`, plus the error cluster (status TRUE
  and the GCS code when there is one **(inferred)**). Label `"Get Error"`.

### 7.6 `Z Set Speed` [main VI frame d226; p.6]

- **Data:** `Z Stage Set Speed Data` = {`Speed(mm/s)`: DBL = 0} [Doc6 p.277–280]. VTD error unwired, so a bad payload sets speed 0.
- **Behaviour:** `VEL(S, A, [Speed])`. `VEL.vi` then queries `ERR?`, so an older pending controller error (for example 10 after
  `Z Stop`) is reported here. `Z Stage Settings.Speed` is not changed; the next `Z Setup` or automatic re-initialisation restores the
  INI speed.
- **Reply:** ACK. Label `"Set Speed"`.

### 7.7 `Z Get Speed` [main VI frame d227; p.9]

- **Data:** `{Extra Param}`, unused (§7.1).
- **Reply:** DBL = first value of `VEL?(S, A)` (0.0 on error **(inferred)**). Label `"Get Speed"`.

### 7.8 `Z Get Reference` [main VI frame d228; p.9]

- **Data:** `{Extra Param}`, unused (§7.1).
- **Reply:** BOOL = first value of `FRF?(S, A)` (TRUE = referenced; FALSE on error **(inferred)**). Label `"Get Z Referenced"`.

### 7.9 `Z Do Reference` [main VI frames d229–d230; p.9]

- **Data:** a **bare enum** whose type constant shows `FNL`. Cases exist for `"FNL", Default` and `"FRF"`; other items (for example
  `FPL`) are **(unknown)** and would run FNL. VTD error unwired, so a bad payload runs FNL.
- **Behaviour:** `FNL(S, A)` (move to the negative limit) or `FRF(S, A)` (reference switch) → `General wait for movement to stop
  (polling 10 ms)` → `ERR?` → `GCSTranslateError`. The Z loop is **blocked** until the axis stops (internal timeout 300 s, error −7).
- **Reply:** ACK. Label `"Get Z Referenced"` (sic, copied from `Z Get Reference`); `Command` is wired to `LSE` here only, without
  effect for the Z server type.

### 7.10 `Z Get Position` [main VI frame d231; p.9]

- **Data:** `{Extra Param}`, unused (§7.1).
- **Behaviour:** `POS?(S, A)`; indicator `Z Pos (mm)` ← first value (0.0 on error, also written).
- **Reply:** DBL = that value. Label `"Get Position"`.

### 7.11 `Z Move Absolute` [main VI frames d232–d235; p.9]

- **Data:** `Z Move Abs Data` = {`Pos (mm)`: DBL = 0} [Doc6 p.447–450]. VTD error **wired** into the chain.
- **Behaviour:**
  1. `alox_z_check_axis_reference.vi` (comment "Check Z REF") [Doc6 p.139–143]: `FRF?`, whose error (and any incoming error) is
     cleared. If the axis is not referenced, or the query failed, the link is closed (error cleared) and re-initialised with the same
     PI configuration call as `Z Setup` (possible reference move), then the normal speed is set; an error of this re-initialisation is
     **returned** and chained into the move (the `MOV` is skipped). Otherwise the step returns "no error".
     `Z Inited` ← its `Z INITED` output; the **shift register** `Z Session` ← its session output (the front-panel control is not updated).
     If it reports `REF WARNING`, `Log Server Infos` writes `"Move Absolute: Z Reference resetted"` (an info line, written even without
     error).
  2. Range check on `Pos` against `[min(Min Position, Max Position) ; max(…)]`, bounds included (primitive `Max & Min`, no rounding).
  3. In range: `MOV(S, A, [Pos])`, with `S`, `A` taken from the check's session output; on success `Z Pos (mm)` ← `Pos`. The VTD
     error is chained before this point, so a bad payload (decoded as 0.0) skips the `MOV` and returns the decode error, or 9405 when
     0.0 is out of range. Out of range: no command is sent and the error becomes
     `{TRUE, 9405, "Z move out of range"}`, replacing any earlier error.
  4. **No wait** for the end of motion, and no `ERR?`: a move refused by the controller is reported only by a later `ERR?` reader.
- **Reply:** ACK (an out-of-range move is therefore probably answered by silence, §1.1). Label `"Move Absolute"`.
- **Conflict resolved:** an earlier analysis read the limit nodes as floor/ceil roundings. The node is the `Max & Min` primitive
  [main VI frame d232].

### 7.12 `Z Move Relative` [main VI frames d236–d239; p.12]

- **Data:** `Z Stage Move Rel Data` = {`Move (mm)`: DBL = 0} [Doc6 p.325–328]. VTD error **unwired** (unlike `Z Move Absolute`).
- **Behaviour:** reference check as in §7.11 (info text `"Move Relative: Z Reference resetted"`); `POS?` → `cur` (0.0 if it failed);
  `target ← cur + Move`; the range check is on `target`. In range: `MVR(S, A, [Move])`, and on success `Z Pos (mm)` ← `target`. Out of
  range: 9405 as above, which also replaces a `POS?` error. No wait.
- **Note:** the check uses the measured position, while `MVR` moves from the last commanded target, so the two differ while a previous
  move is still running.
- **Reply:** ACK. Label `"Move Relative"`.

### 7.13 `Z Wait End Move` [main VI frame d240; p.12]

- **Data:** `Z Stage Wait Data` = {`Add. Wait (ms)`: integer = 0, `Polling Cycle (ms)`: integer = 10, `Add. Wait Only`: Boolean =
  FALSE} [Doc6 p.329–332]. VTD error unwired.
- **Behaviour:** `General wait for movement to stop(S, A, Additional wait time = Add. Wait, Polling cycle time = Polling Cycle,
  Add. wait only? = Add. Wait Only)`. It polls the motion status (`#5`) every polling cycle until all axes stop, then waits
  `Add. Wait (ms)`; with `Add. Wait Only` = TRUE it only waits. The PI description says the VI "does not time out" [Doc6 p.159], but
  its sub-VI `Wait for axes to stop.vi` has an unwired `Timeout (s)` of 300 s that ends with error −7 "Wait for axes to stop.vi:
  Timeout." [Doc6 p.211–212]. The additional wait also runs after that timeout ([08-z-stage.md](08-z-stage.md) §8). Blocks the Z loop.
- **Reply:** ACK. Label `"Wait End Move"`.

### 7.14 `Z Stop` [main VI frames d241–d242; p.12]

- **Data:** `{Extra Param}`, unused (§7.1).
- **Behaviour:** `STP(S, A)` → `RMC?` → `MAC START "DH"` unless `DH` or `HOME` is running. `STP` also stops macros, so `DH` is
  restarted right after. `STP` leaves controller error 10 pending, to be reported by the next `ERR?` reader (§7.4–7.6, §7.15). The
  indicators `Running macros` and `Macro running ?` are not updated.
- **Reply:** ACK. Label `"Stop"`.

### 7.15 `Z Enable Joystick` / `Z Disable Joystick` [main VI frames d243–d244; p.12]

- **Data:** `Z Stage Joystick Data` = {`Joystick ID`: integer = 1, `Joystick Axis`: integer = 1} [Doc6 p.337–340]. Diagram comment:
  "Joystick ID = 1 (First Joystick) / Joystick Axis = 1 (First Axis of System)". VTD error unwired.
- **Behaviour:** `JAX(S, Joystick ID, Joystick Axis, A)` → `JON(S, [Joystick ID], [1])` to enable or `[0]` to disable. `JAX` is sent in
  both cases, and `JON.vi` queries `ERR?` afterwards.
- **Reply:** ACK. Labels `"Enable Joystick"` / `"Disable Joystick"`.

### 7.16 `Z Joystick Enabled ?` [main VI frame d245; p.15]

- **Data:** `Z Stage Joystick Data`; only `Joystick ID` is used. VTD error unwired.
- **Reply:** BOOL = (first value of `JON?(S, [Joystick ID], All joysticks? = FALSE)`) ≠ 0. Label `"Is Joystick Enabled ?"`.

### 7.17 `Z Set Min Limit` / `Z Set Max Limit` [main VI frames d246–d247; p.15]

- **Data:** a **bare DBL** (mm). VTD error wired.
- **Behaviour:** `Z Stage Settings.Min Position (mm)` or `Max Position (mm)` ← value, in memory only. The write happens even when the
  VTD failed, so a bad payload sets the limit to 0.0 **(inferred)**. There is no validation, nothing is sent to the controller and
  nothing is written to the INI. Inverted limits are tolerated because the range check sorts them (§7.11).
- **Reply:** ACK with the VTD error. Labels `"Z Set Min Limit"` / `"Z Set Max Limit"`.

### 7.18 `Z Cleanup` [main VI frames d248–d249; p.15]

- **Data:** not read.
- **Behaviour:** `Close connection if open(S)` → `RMC?` → `MAC START "DH"` unless `DH` or `HOME` is running. After the close, `RMC?`
  most likely fails, so the macro is not started and the command ends in error **(inferred)**.
- **State:** `Z Session` (shift register and front-panel control) ← default {`System` 0, `Axes` [], `""`, `""`}. **`Z Inited` is not
  reset.**
- **Reply:** ACK. Label `"Cleanup"`. If the inference above is right, the client normally receives **no** reply (§1.1).

### 7.19 `Exit` (Z case) and `Default` [main VI frames d0, d220]

`Exit`: `stop ← TRUE`, then `Release Queue(Z, force destroy)`. No reply, no PI close. It is reached through the camera loop's `Exit`
(§5.4) **(inferred, §3.2 point 5)**. `Default`: nothing, no reply.

---

## 8. Error codes a client can observe

| Code | `source` text | Produced by | Note |
|---|---|---|---|
| 9405 | `"Z move out of range"` | `Z Move Absolute`, `Z Move Relative` | ACK, probably not sent (§1.1) |
| 9009 | `"get image error (timeout)"` + `"lastbuffer:…/ buffercount:…/ buffer t-1:…"` | `Cam Stream Next Image` | IMAGE reply with an empty image |
| −9011 | `"Invalid Type"` | `Cam Stream Next Image` in simulation | unsupported PNG pixel type |
| 9031 | `"'refocus' warning (joystick enabled)"` (status TRUE) | `Cam Refocus` (synchronous `Start` step) | ACK, probably not sent (§1.1); logged |
| 9401 | `"z stage get position timeout"` | background AF/RF steps (`Cam Autofocus Run`, `Cam Refocus Run`) | never replied (no reply for `Run`); logged; the job stops (§6.3) |
| −1074360316 | IMAQdx "invalid session" | any camera command on a camera that is not set up | **cleared** in `Cam Get Image` and `Cam Get Acquisition Status`; cleared by `Cam Stop` when nothing is running |
| −7 | `"Wait for axes to stop.vi: Timeout."` | `Z Do Reference`, `Z Wait End Move` | 300 s internal timeout |
| positive GCS codes (e.g. 10 "Controller was stopped by command") | `"Controller error"` + GCS text | every Z command that reads `ERR?` (`Z Get Status`, `Z Get Error`, `Z Set Speed`, `Z Do Reference`, `Z Enable/Disable Joystick`, `Z Setup`, and a `Z Move …` that re-initialises) | may stem from an **earlier** command ([08-z-stage.md](08-z-stage.md)) |
| negative PI driver codes, 100 | PI VI name + text | any Z command with a closed or unconfigured link (100 e.g. "No axis specified" when `Axes` is empty) | ([08-z-stage.md](08-z-stage.md)) |
| 91 | LabVIEW "variant type not compatible" | commands whose VTD error is wired, with a payload of the wrong type | **(inferred)** standard LabVIEW code |
| IMAQdx / NI Vision codes | driver text + command suffix (§5) | camera commands | — |

Codes that LabVIEW never returns, although a client might expect them: unknown command, malformed payload, and "no camera set up"
(cleared). Where the source is decorated or replaced: §4.2 and the subsections.

---

## 9. Defects and oddities in the LabVIEW implementation

Command-level items only. Subsystem defects are listed in [05-camera-acquisition.md](05-camera-acquisition.md) §13,
[06-autofocus-refocus.md](06-autofocus-refocus.md) §10, [08-z-stage.md](08-z-stage.md) and [01-architecture.md](01-architecture.md) §11.

| # | Behaviour | Evidence | Impact | Recommendation |
|---|---|---|---|---|
| C1 | Routing by enum range: any misplaced item is never executed; `Exit` outside the camera range would make the server unstoppable. | [main VI frames d0, d66] | silent loss of commands | **Reproduce** the routing outcome: evaluate the ranges once on the real item list into explicit name/code sets (§3.5, [04](04-data-types.md) T2), and self-check at start-up that every client command reaches a loop with a case for it. |
| C2 | Unknown values, `Default` cases, `Cam Get Image`, `Cam Get Images Array` and `Exit` produce no reply. | [main VI frames d65, d73, d98, d169, d78, d220] | client timeouts | **Reproduce** on the wire, log a warning. **Ask** whether replies may be added (questionnaire Q1). |
| C3 | ACK replies carrying an error are probably never sent (`Z Move …` 9405, `Cam Setup` failures, `Z Cleanup`…). | [Doc6 p.437] | failures look like timeouts | **Ask** (capture first); default **fix** with a compatibility switch ([02-tcp-protocol.md](02-tcp-protocol.md) §8.4). |
| C4 | Unwired VTD errors: wrong payload types run with the defaults (HM camera, speed 0, FNL, …). Wired cases sometimes still act (`Cam Flip`, `Z Set Min/Max Limit` write defaults). | §4.1, §7 | wrong camera or axis action, silently | **Fix**: reject undecodable payloads with an error reply. |
| C5 | `Log` stores parameters that nothing uses (TODO). | [main VI frame d76] | none | **Reproduce** (ACK, no effect); real logging is configured by the INI. |
| C6 | `Show UI` acts on the LabVIEW window. | [main VI frame d74] | none without a GUI | **Reproduce** as an ACK; drive the GUI if one exists. |
| C7 | Source decoration is inconsistent: appended (most commands), replaced (`Cam Set Exposure`), added after the reply (`Cam Set Led`, `Cam Get Led`, AF read-backs), never (`Cam Software Trigger`). | §4.2 | client-visible `source` strings differ | **Reproduce** if the client parses `source` (ask); otherwise unify. |
| C8 | `Cam Software Trigger` always reports success and does nothing on USB. | [main VI frame d119] | silent failure | **Ask** (see 05 #13). |
| C9 | `Cam Start` (HM) chooses the trigger path from `Camera Sort: LED`. | [main VI frame d91] | wrong path when buses differ | **Fix**, confirm (05 #12). |
| C10 | `Z Do Reference` logs under the label `"Get Z Referenced"`. | [main VI frame d229] | misleading log | **Fix** (log label only). |
| C11 | `Z Get Status`, `Z Stop` and `Z Cleanup` start macro `DH` when it is not running; start-up uses the inverse logic; `Z Cleanup` tries it after closing the link. | [main VI frames d223, d241, d248, d59] | unknown motion | **Ask** (questionnaire Q10). |
| C12 | `Z Cleanup` leaves `Z Inited` TRUE; `Z Move …` updates the Z loop's session but not the control used by AF/RF. | [main VI frames d248, d232, d236] | stale status, stale AF/RF link | **Fix** (one locked Z link object). |
| C13 | `Z Move …` neither waits nor reads `ERR?`; a refused move is reported to a later command. | [main VI frames d232, d236] | error attributed to the wrong command | **Reproduce** the reply; document; **ask** before adding an `ERR?`. |
| C14 | `Z Set Speed` does not update `Z Stage Settings.Speed`; re-initialisation restores the INI speed. | [main VI frame d226] | speed silently reverts | **Reproduce**. |
| C15 | `Z Set Min/Max Limit` are not validated or persisted. | [main VI frames d246, d247] | limits lost at restart | **Reproduce** (memory only); reject non-finite values. |
| C16 | Internal `… Run` commands are enum items that a client may be able to send (position unknown). | §3.2 | stray steps | **Fix**: never route them from TCP. |
| C17 | `Cam Autofocus Run` floods `Commands History`; stream polls and AF/RF steps grow the unbounded error and temperature histories. | [main VI frames d0, d205] | memory growth | **Fix** (01 A11, A13). |
| C18 | Self re-enqueue (`Cam Stream Next Image`, `… Run`) with an infinite timeout into the camera loop's own 5-element queue. | [main VI frames d106, d107, d181, d205] | possible deadlock | **Fix** (05 §11, 06 §9.5). |

---

## 10. Open questions

Only items that change the command API. Questionnaire numbers refer to [open-questions.md](open-questions.md).

| # | Question | Why it matters | How to answer |
|---|---|---|---|
| Q1 | Item list, order and representation of `typedef_ext_camera_camera_command.ctl`: are there items beyond the 46 known names? Is `Exit` in `[Get Heartbeat ; Cam Get Led]`? Is `Z Do Reference` the last Z item? Where are `Log`, `Cam Autofocus Run`, `Cam Refocus Run`? | Wire codes and routing (§3.2) | `.ctl` Edit Items, or a capture (questionnaire Q2, Q4) |
| Q2 | Are error ACKs really suppressed, and does the client rely on the resulting timeouts? | C3; affects every ACK command | capture of an out-of-range `Z Move Absolute` (questionnaire Q17) |
| Q3 | Does the client send `Cam Get Image`, `Cam Get Images Array` or `Log`, and does it expect no reply? | C2, C5 | client code or capture |
| Q4 | Does the client parse `error.source` (suffixes, replaced sources)? | C7 | client code |
| Q5 | Complete item list of the `Z Do Reference` enum (`FPL`?) and its representation | Payload decoding; any item other than `FRF` runs FNL | `.ctl` or capture (questionnaire Q2) |
| Q6 | Content of `Command Data` for commands whose data is never read (`Get Heartbeat`, `Exit`, `Z Setup`, `Z Get Status`, `Z Get Error`, `Z Cleanup`, `Cam Get Images Array`) | A strict decoder must accept it (empty variant or something else) | capture |
| Q7 | Field order of `Camera Init Data`, `Camera Autofocus Data` and `Camera Refocus Data` | Decoding of `Cam Setup`, `Cam Autofocus`, `Cam Refocus` | `.ctl` (questionnaire Q3) or capture |
| Q8 | Does `Z Setup` reference the axis every time (hidden options of the PI configuration VI)? | Physical motion on each `Z Setup` and each automatic re-initialisation | observation on the machine (questionnaire Q11) |
