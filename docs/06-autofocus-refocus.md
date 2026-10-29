# Autofocus and refocus

**Summary.** The server offers two Z-scan focus procedures, both started by a TCP command and run in the background by the camera
consumer loop. **Autofocus** (`Cam Autofocus`) scans an absolute Z range `[Z Min, Z Max]`. It runs either step by step ("Slow": move,
settle, grab, measure) or as one continuous move ("Fast": images are grabbed on the fly and their Z is interpolated from time). It only
returns the measured `[Z, score]` curve and never chooses a best position. **Refocus** (`Cam Refocus`) scans `2·Steps+1` positions
centred on the current Z, measures a focus score at each, moves the stage to the best one (strict argmax) and can save every image as a
PNG. Each procedure is a LabVIEW sub-VI that executes **one state-machine step per call** and keeps its working data in persistent
internal memory. The main VI calls it once synchronously, then re-posts an internal "Run" command to its own queue until the sub-VI
reports `Finished`. The client polls the progress and the data with `Cam Get Autofocus Progress` and `Cam Get Autofocus Data`. Both
procedures drive the PI Z stage directly with GCS commands (`POS?`, `MOV`, `VEL`, `VEL?`, `JON?`). This chapter specifies all of it:
states, transitions, persistent state, timing, tolerances, returned data, image logging, defects, and a Python design.

**Sources:** `alox_camera_server_autofocus.vi` [Doc6 p.603–611]; `alox_camera_server_refocus.vi` [Doc6 p.1030–1039];
typedefs `typedef_ext_camera_autofocus_data.ctl` [Doc6 p.599–602], `typedef_ext_autofocus_fast_session.ctl` [Doc6 p.595–598],
`typedef_ext_camera_autofocus_state.ctl` [Doc6 p.636–639], `alox_typedef_autofocus_speed.ctl` [Doc6 p.587–590],
`alox_typedef_autofocus_method.ctl` [Doc6 p.620–623], `typedef_ext_camera_refocus_data.ctl` [Doc6 p.612–615],
`alox_typedef_refocus_method.ctl` [Doc6 p.624–627], `typedef_ext_camera_refocus_state.ctl` [Doc6 p.640–643],
`typedef_ext_camera_refocus_session.ctl` [Doc6 p.660–663], `alox_typedef_semi_job_infos.ctl` [Doc6 p.969–972];
helpers `tools_focus_array_to_xy_graph_cluster.vi` [Doc6 p.591–594], `tools_refocus_arrays_to_xy_graph_cluster.vi` [Doc6 p.1026–1029],
`tools_log_refocus_image.vi` [Doc6 p.982–985], `tools_alox_job_data_filename.vi` [Doc6 p.973–977];
`typedef_ext_camera_getautofocusdata.ctl` [Doc6 p.393–396], camera session typedef `alox_typedef_ext_camera_session.ctl`
[Doc6 p.628–631], Z typedefs `alox_typedef_ext_zstage_session.ctl` [Doc6 p.644–645] and `alox_typedef_ext_zstage_properties.ctl`
[Doc6 p.652–653]; ALOX vision VIs called (specified elsewhere): `alox_vision_flip_image.vi` [Doc6 p.928–933],
`alox_vision_measure_focus_fft.vi` [Doc6 p.946–950], `alox_vision_measure_focus_sobel.vi` [Doc6 p.955–960]; NI VIs used: IMAQdx Get
Image2 [Doc6 p.895–896], IMAQ Flat Field Correction [Doc6 p.934–935], IMAQ GetImageSize [Doc6 p.1006–1009], IMAQ Overlay Rectangle
[Doc6 p.965–968], IMAQ Write Image And Vision Info File 2 [Doc6 p.978–981], IMAQ LowPass [Doc6 p.961–964], Clear Errors
[Doc6 p.57–58]; PI GCS `MOV` [Doc6 p.878], `POS?` [Doc6 p.668], `VEL?` [Doc6 p.710], `VEL` [Doc6 p.716–719], `JON?` [Doc6 p.664];
main VI `alox_camera_server.vi`, camera-loop cases `Cam Autofocus` [main VI frames d181–d185], `Cam Refocus` [d186–d190],
`Cam Get Autofocus Data` [d191–d192], `Cam Get Autofocus Progress` [d193–d194], `Cam Autofocus Run` [d205–d209],
`Cam Refocus Run` [d210–d214] (text in [main VI PDF p.25–28]).

Related chapters: [01-architecture.md](01-architecture.md) (loops and queues), [02-tcp-protocol.md](02-tcp-protocol.md) (framing, response
encoding), [03-command-reference.md](03-command-reference.md), [04-data-types.md](04-data-types.md) (cluster layouts, enums),
[05-camera-acquisition.md](05-camera-acquisition.md) (grab, flip, flat field), [07-focus-metrics.md](07-focus-metrics.md) (score formulas),
[08-z-stage.md](08-z-stage.md) (PI GCS link), [09-configuration-logging-errors.md](09-configuration-logging-errors.md) (error clusters, logs),
[10-migration-plan.md](10-migration-plan.md).

---

## 1. Overview

### 1.1 The two procedures at a glance

| Aspect | Autofocus | Refocus |
|---|---|---|
| Start command | `Cam Autofocus` (data `Camera Autofocus Data`) | `Cam Refocus` (data `Camera Refocus Data`) |
| Sub-VI | `alox_camera_server_autofocus.vi` | `alox_camera_server_refocus.vi` |
| Z range | absolute, `Z Min → Z Max`, given by the client | relative to the current Z: `Zinit ± Steps·Step`, each point clamped to the stage software limits |
| Modes | `Slow` (step, settle, measure) or `Fast` (continuous move, time-interpolated Z) | step-wise only |
| Position check | `POS?` within ±1 µm (both bounds inclusive) | `POS?` within [target − 1 µm, target + 1 µm) **and** a 5 s timeout per move |
| Settle time | fixed 100 ms (Slow only) | `Wait (ms)` from the command |
| ROI | client-supplied rectangle `ROI` | computed from `Fin ROI Width/Height` and the image size |
| Subsampling | client-supplied `Subsampling` | fixed 0.5 |
| Metrics | FFT, Prewitt, Sobel (any other method → Sobel) | FFT, Sobel, Prewitt, Diff, Gradient, Sigma, FULL |
| Best position | **not computed** (the client analyses the data) | strict argmax over the grid, then a move to that position |
| End of run | initial speed restored; the stage stays where the scan ended (Slow: one step beyond `Z Max`; Fast: `Z Max`) | initial speed restored; the stage stays at the best position |
| Joystick check | none | `JON? 1`; joystick active → error 9031 |
| Images saved | never | optional PNG per position (`Save Images`, `Folder`) |
| Output | `Autofocus Data`: N×2 rows `[Z, score]` | same output name and shape (unmeasured rows have score 0) |

### 1.2 Execution model

Both sub-VIs follow the same pattern [Doc6 p.606, p.1033]:

- The state-machine `While` loop has its stop condition wired to a constant `True`. **Each call therefore executes exactly one
  state**, with a parallel `Wait (ms) = 1`, so every call lasts at least 1 ms.
- The caller passes the current state (`AF State In` / `Refocus State In`) and gets the next state back (`AF State Out` /
  `Refocus State Out`). The caller stores this state in the camera session between calls.
- The working data (scan positions, collected scores, initial speed and position, timestamps) and an error cluster live in
  **uninitialised shift registers** of that loop. In LabVIEW, an uninitialised shift register keeps its last value from one call of
  the VI to the next. It acts like a private static variable: the VI is a "functional global". The procedure therefore has **one
  instance for the whole server**, shared by both cameras **(inferred: the VIs are non-reentrant)**. The design only works that
  way: each sub-VI has three call sites in the main VI (start command, `Run` High Mag frame, `Run` Low Mag frame [main VI frames d181,
  d205, d207]), and with reentrant clones the data stored by `Start` would never reach the `Run` steps.
- The main VI drives the procedure with a **self-re-enqueue** pattern: the start command runs the first step, then posts an internal
  `Cam … Run` command at the tail of the camera queue. Each `Run` executes one more step and re-posts itself until the sub-VI reports
  `Finished` (§3).

### 1.3 LabVIEW behaviours that matter here (plain-language glossary)

| LabVIEW behaviour | Meaning for the re-implementation |
|---|---|
| Error cluster `{status, code, source}` and "error in / error out" chaining | Most NI and PI driver VIs **do nothing** when their `error in` carries an error, and pass it on. A chain of calls therefore stops at the first failure, but the later calls still return their default outputs (0, empty array). |
| `Clear Errors.vi` [Doc6 p.57–58] | Discards an error: the failure is ignored and the chain continues. |
| Unwired `error out` | The error is lost. If "Automatic error handling" is enabled in the VI's properties, LabVIEW would instead open a blocking dialog. Whether it is enabled in the focus VIs is **(unknown)** (questionnaire Q41 asks it for the focus VIs; Q30 for the main VI). Concerned here: `JON?` (refocus `Start`), the image display and `IMAQ LowPass` (refocus). |
| `Index Array` on an empty or too-short array | Returns the element type's default value (0.0 for DBL) instead of failing. A failed `POS?` is therefore read as position **0.0 mm**. |
| `In Range and Coerce` | Range test and clamp. Each bound can be inclusive or exclusive (filled or hollow diamond on the icon). |
| `Tick Count (ms)` | Unsigned 32-bit millisecond counter that wraps every ~49.7 days. Use a monotonic clock in Python. |
| Floating-point division by zero | Gives ±Inf or NaN. It does not raise an error. |
| Float → integer conversion | Rounds half to even (Python `round()` / `numpy.rint`), **not** truncation. |
| `Build Array` of two 1-D arrays of different lengths into a 2-D array | The shorter row is **padded with zeros**. |
| Flattened responses | Big-endian (see [02-tcp-protocol.md](02-tcp-protocol.md)). |

---

## 2. Client-facing interface

### 2.1 Commands

| Command | Data (variant `Command Data`) | Action | Response | Log extra string / source suffix |
|---|---|---|---|---|
| `Cam Autofocus` | `Camera Autofocus Data` (§2.2) | runs autofocus step `Start` synchronously, stores the result in the camera session, queues `Cam Autofocus Run` | Error cluster (ACK) | `"Autofocus"` / `"Cam Autofocus"` |
| `Cam Refocus` | `Camera Refocus Data` (§2.3) | runs refocus step `Start` synchronously, stores the result, queues `Cam Refocus Run` | Error cluster (ACK) | `"Refocus"` / `"Cam Refocus"` |
| `Cam Get Autofocus Data` | `Camera Get Autofocus Data` = `{Camera Type}` (`typedef_ext_camera_getautofocusdata.ctl` [Doc6 p.393–396]) | — | **DBL 2-D array** = `Last Focus Data` of the selected camera session | `"Autofocus Data"` / `"Cam Get Autofocus Data"` |
| `Cam Get Autofocus Progress` | `Camera Get Autofocus Data` (same typedef) | — | **DBL** = `Focus Progress (%)` of the selected camera session | `"Autofocus Progress"` / `"Cam Get Autofocus Progress"` |
| `Cam Autofocus Run` (internal) | same variant as the start command | one autofocus step; re-posts itself until finished | **none** | `"Autofocus Run"` / `"Autofocus"` |
| `Cam Refocus Run` (internal) | same variant as the start command | one refocus step; re-posts itself until finished | **none** | `"Refocus Run"` / `"Cam Refocus Run"` |

[main VI frames d181–d214]. `Cam Autofocus Run` / `Cam Refocus Run` carry the red comment "This command is only used internally to run
Autofocus (Refocus) asynchronously" [main VI frames d205, d210]. The camera loop enqueues them into its own queue, so they need no TCP
routing. The TCP loop routes the enum range `"Get Heartbeat".."Cam Get Led"` plus `"Log"` to the camera loop. Whether the two `Run`
values fall inside that range, and could therefore also be sent by a client, depends on the enum order, which is **(unknown)**. See
[03-command-reference.md](03-command-reference.md).

"High Mag." selects the High Mag session/settings; **any other value** selects Low Mag (`Camera Type == "High Mag."` comparison)
[Doc6 p.606, main VI frames d181, d191]. The `Run` frames label the High Mag case "Run Autofocus/Refocus on Low Angle" and the Low
Mag case "… on High Angle" [main VI PDF p.27–28], which matches the comment "LOW ANGLE = HIGH MAG / HIGH ANGLE = LOW MAG" on the
`Cam Set/Get Led` diagrams [main VI PDF p.26–27].

### 2.2 `Camera Autofocus Data` (`typedef_ext_camera_autofocus_data.ctl`) [Doc6 p.600]

Fields in front-panel display order. The internal cluster order, which sets the byte order on the wire, is **(unknown)**: see
[04-data-types.md](04-data-types.md).

| Field | Type | Default | Use |
|---|---|---|---|
| `Camera Type` | enum `High Mag.` / `Low Mag.` | `High Mag.` | camera session and settings selection |
| `Autofocus Speed` | enum `Slow` / `Fast` | `Slow` | scan mode. The case structures are `"Fast"` and `"Slow", Default`, so any value other than `Fast` behaves as `Slow` |
| `Autofocus Method` | enum (7 values, §2.4) | `FFT` | metric. The autofocus only handles `"FFT"`, `"Prewitt"` and `"Sobel", Default`: **Diff, Gradient, Sigma and FULL all run Sobel** [Doc6 p.605–606] |
| `Duration (s)` | integer (exact width **(unknown)**) | 0 | Fast mode: scan duration in **whole seconds** |
| `Z Min` | DBL, mm | 0 | scan start (absolute) |
| `Z Max` | DBL, mm | 0 | scan end (absolute) |
| `Step` | DBL, mm | 0 | Slow mode: step size |
| `Subsampling` | floating point | 0 | `Scale Factor` passed to the metric (the metric VIs default to 0.25, but the typedef default 0 is passed unchanged) |
| `ROI` | 1-D integer array `[Left, Top, Right, Bottom]` | empty | metric `Optional Rectangle`. Empty = whole image |
| `Acquisition Timeout (ms)` | integer | 5000 | **not used** (never unbundled) |

### 2.3 `Camera Refocus Data` (`typedef_ext_camera_refocus_data.ctl`) [Doc6 p.613]

| Field | Type | Default | Use |
|---|---|---|---|
| `Camera Type` | enum | `High Mag.` | camera selection; also selects the ROI formula (§5.6) |
| `Autofocus Method` | enum (7 values) | `FFT` | metric (§5.7) |
| `Refocus Method` | enum `Fin` / `Oxyd` | `Fin` | **ignored**: the selector inside the VI is wired to a constant `Fin` [Doc6 p.1035] |
| `Steps` | integer | 0 | number of steps on each side: N = 2·Steps + 1 positions |
| `Step (mm)` | DBL | 0 | step size |
| `Fin ROI Width (pixels)` | integer | 0 | ROI width (High Mag) / ROI **left** (Low Mag) |
| `Fin ROI Height (pixels)` | integer | 0 | ROI height (High Mag) / ROI **top** (Low Mag) |
| `Oxyd X Pos (pixels)`, `Oxyd Y Pos (pixels)`, `Oxyd Diam (pixels)` | integers | 0 | only used by the dead `Oxyd` branch (§5.9) |
| `Acquisition Timeout (ms)` | integer | 5000 | **not used** |
| `Wait (ms)` | integer | 0 | settle pause after each position is reached |
| `Save Images` | Boolean | False **(inferred from the rendering)** | enables PNG logging (§8) |
| `Folder` | LabVIEW path | empty | folder for the PNG files |

### 2.4 Enumerations

| Enum | Labels | Default | Numeric values |
|---|---|---|---|
| `Autofocus Speed` [Doc6 p.588] | `Slow`, `Fast` | `Slow` | **(unknown)**; probably `Slow`=0, `Fast`=1 **(inferred)** |
| `Autofocus Method` [Doc6 p.621] | `FFT`, `Sobel`, `Prewitt`, `Diff`, `Gradient`, `Sigma`, `FULL` | `FFT` | **(unknown)**. The refocus case structure has exactly these 7 cases and no `Default` case, so the enum has exactly 7 values **(inferred)** [Doc6 p.1035–1036] |
| `Refocus Method` [Doc6 p.625] | `Fin`, `Oxyd` | `Fin` | **(unknown)** |
| `AF State` [Doc6 p.637] | `Start`, `Goto Z Pos`, `Wait Started`, `Measure`, `Finished`, `Error` | `Start` | internal only (never sent to the client) |
| `Refocus State` [Doc6 p.641] | `Start`, `Get Positions`, `Move Position`, `Check Position`, `Mesure Position` (sic), `Get Focus`, `Move Focus`, `Wait Focus`, `Finished`, `Error` | `Start` | internal only |
| `Camera Type` | `High Mag.`, `Low Mag.` | `High Mag.` | `High Mag.`=0, `Low Mag.`=1 **(inferred)**, see [04-data-types.md](04-data-types.md) |

Both state machines also have a `Default` case that passes everything through with progress 0. This suggests that the state enums may
hold values not listed above **(unknown)**. The case is unreachable in practice **(inferred: the state only ever comes from the
session default `Start` or from the sub-VI's own state constants)**.

### 2.5 Camera-session fields used

The camera session cluster (`alox_typedef_ext_camera_session.ctl` [Doc6 p.628–631], one per camera, held in shift registers of the
camera loop; see [05-camera-acquisition.md](05-camera-acquisition.md)) carries four focus fields:

| Field | Type | Default | Written by | Read by |
|---|---|---|---|---|
| `AF State` | `AF State` enum | `Start` | `Cam Autofocus`, `Cam Autofocus Run` | `Cam Autofocus Run` |
| `RF State` | `Refocus State` enum | `Start` | `Cam Refocus`, `Cam Refocus Run` | `Cam Refocus Run` |
| `Last Focus Data` | DBL 2-D array | empty | the four commands above | `Cam Get Autofocus Data` |
| `Focus Progress (%)` | numeric | 0 | the four commands above | `Cam Get Autofocus Progress` |

`Last Focus Data` and `Focus Progress (%)` are **shared** by autofocus and refocus. `Cam Get Autofocus Data/Progress` therefore return
the data of whichever procedure ran last on that camera **(inferred from the wiring)** [main VI frames d181–d193].

### 2.6 Returned data

- **Shape**: N rows × 2 columns, row = `[Z (mm), score]`.
  - Autofocus: one row per measured image, in acquisition order. Slow mode uses the **commanded** Z (`Z Min`, `Z Min+Step`, …), which is
    never re-read from the stage. Fast mode uses the **time-interpolated** Z (§4.5, `Measure`, Fast). Before the first measurement the array is empty.
  - Refocus: one row **per planned position** from the moment the positions are computed. Rows not yet measured have score `0`
    (`Transpose(Build Array(Positions, Scores))`, zero padding) [Doc6 p.1033]. After a complete scan every row holds its measured score.
- **On the wire** (`Cam Get Autofocus Data`): DBL 2-D array response = I32 row count, I32 column count, then rows × columns IEEE-754
  doubles, big-endian, row-major, followed by the error cluster (see [02-tcp-protocol.md](02-tcp-protocol.md)). The error cluster in
  the response is the error of the `Variant To Data` decoding of the request, sent as data [main VI frame d191].
- Empty arrays: the default session value and the value written after a failed start come from an empty 2-D constant (0 × 0). The
  autofocus `AF Data` after `Start` comes from the reset typedef constant (0 × 0 **(inferred)**). A refocus result with no positions
  yet comes from `Transpose(Build Array([], []))`, which LabVIEW may report as 0 × 2 **(inferred)**. The dimension words of an empty
  answer can therefore differ (0/0 vs 0/2) **(inferred)**. A writer that takes both words from the array shape reproduces this.
- **Never returned**: the best Z of a refocus (only stored internally in `Focus Pos` / `Focus Score`, which nothing reads), the state,
  and the error of a background step.

### 2.7 Progress semantics (`Focus Progress (%)`)

The value returned by `Cam Get Autofocus Progress` is the `Progress (%)` output of the **last executed step**:

| Procedure / phase | Progress written |
|---|---|
| AF `Start`, `Goto Z Pos`, `Wait Started` | 0 |
| AF Slow `Measure` | `(Znext − Z Min) / (Z Max − Z Min) · 100`, where `Znext` = next commanded position. The last point has `Znext > Z Max`, so it reports **more than 100** (for example 100.00000000000009, or 111.1 with `Z Max = 1.45` in the §4.5 example) |
| AF Fast `Measure` | `(Zest − Z Min) / (Z Max − Z Min) · 100` = `(ti − T0) / Duration · 100`. The last point has `ti − T0 ≥ Duration`, so it reports **≥ 100** |
| AF `Finished` / `Error` | 100 |
| RF `Start`, `Get Positions` | 0 |
| RF `Move Position`, `Check Position` | `clamp(index / N · 100, 0, 100)`, `N = len(Positions)`. With `N = 0` (`Steps < 0`) this is 0/0 = NaN; the result of the clamp is then **(unknown)** |
| RF `Mesure Position` | `clamp((index+1) / N · 100, 0, 100)`: the last measurement already reports 100 |
| RF `Get Focus`, `Move Focus`, `Wait Focus`, `Finished`, `Error` | 100 |
| Failed first step (`Cam Autofocus` / `Cam Refocus`) | 0 (forced by the main VI) |

Consequences, all following directly from the table:

- In **Slow** autofocus the progress returns to 0 between points, because every `Goto Z Pos` step (at least one per point, and the
  one that sleeps 100 ms) writes 0. A client that polls mostly reads 0 **(inferred)**.
- In both autofocus modes the last `Measure` step reports **100 or more** (see the table), and the next step (`Finished`) writes
  exactly 100. A client that tests `progress >= 100` can stop one step before the initial speed is restored; a client that tests
  `progress == 100` waits for the `Finished` step.
- In refocus the progress reaches **100 at the last `Mesure Position`**, before `Get Focus`, before the move to the best position and
  before the speed is restored.
- After a failure during a background step, the progress **freezes** at the value of the failed step (§3.4). That is below 100,
  except when the failure happens in the last measurement step: the autofocus then shows ≥ 100 and the refocus exactly 100, which a
  client cannot tell from success.
- The client cannot read the state, so it cannot tell success from failure.

### 2.8 Typical client sequence (inferred)

```
Cam Autofocus(params)            -> ACK (error cluster)
repeat: Cam Get Autofocus Progress -> DBL   until it reads 100 (or a client-side timeout)
Cam Get Autofocus Data           -> N x 2 [Z, score]   (autofocus: the client chooses the best Z, then e.g. Z Move Absolute)
```

The client application is not part of the evidence. This sequence is inferred from the commands that exist.

---

## 3. How the main VI drives the state machines

### 3.1 Start commands: `Cam Autofocus` / `Cam Refocus` [main VI frames d181–d190, main VI PDF p.25–26]

Comments on the diagram: "Start autofocus state machine", "Send ACK Response (Error)", "Queue "Autofocus Run command" for updating data &
progress".

```
(params, err) = VariantToData(Command Data, <Camera Autofocus Data | Camera Refocus Data>)   # decoding error wired
(state, data, progress, finished, err1) =
    <autofocus | refocus>.vi(State In = "Start", params,
                             High/Low Mag Session = camera-loop shift registers,
                             High/Low Mag Settings = local variables,
                             Z Session = local variable of the front-panel control "Z Session",
                             Z Stage Settings = local variable,
                             Display Ctrl = reference to the "Image" indicator,
                             Focus Graph = reference to the "Focus Graph" XY graph,
                             error in = err)
# 'finished' is not wired
err2 = err1 with source := source + "Cam Autofocus" | "Cam Refocus"        (only if err1 is an error)
Log Server Error(extra = "Autofocus" | "Refocus", err2)
eR = Enqueue Send Error Cluster Response(err2)                              # the ACK
Enqueue Element(Cam Queue, <same element with Camera Command = "Cam Autofocus Run" | "Cam Refocus Run">, error in = eR)
                                                                            # timeout -1 (blocking); error out not wired
if err2 is no error:  session[cam].<AF State|RF State> = state ; .Last Focus Data = data ; .Focus Progress (%) = progress
else:                 session[cam].<AF State|RF State> = "Error" ; .Last Focus Data = [] ; .Focus Progress (%) = 0
stop = False
```

- The first step runs **synchronously** inside the command. For autofocus that is the `Start` state: read the speed (`VEL?`), set the
  normal speed, read the position (`POS?`), and send `MOV Z Min` unless already within ±1 µm. For refocus: joystick check, read
  position and speed, set the normal speed.
- The `Run` element is enqueued **only if the ACK path carries no error**: the decorated error feeds the ACK VI, whose `error out`
  feeds the `Enqueue` [main VI frame d181]. A queue primitive does not run when its `error in` is an error, and the ACK VI passes its
  `error in` through **(inferred, see [02-tcp-protocol.md](02-tcp-protocol.md))**. A failed start therefore leaves the procedure
  stopped with session state `Error`, progress 0 and no data. Its `Error` (restore) state never runs, so the normal speed set by
  `Start`, or a `MOV Z Min` already sent, stays in effect.
- [02-tcp-protocol.md](02-tcp-protocol.md) notes that `Enqueue Send Error Cluster Response` probably **never sends a response whose
  status is TRUE**. If confirmed, the client of a failed `Cam Autofocus` gets no ACK at all and runs into its own timeout.
- The re-posted element keeps the original `Command Data` variant. Every `Run` step decodes the same parameters again.

### 3.2 Background step: `Cam Autofocus Run` / `Cam Refocus Run` [main VI frames d205–d214, main VI PDF p.27–28]

Comments: "Run Autofocus on Low Angle" (`True` = High Mag), "Run Autofocus on High Angle" (`False` = Low Mag), "Not Finished, Re-enqueue
'Run" command".

```
[AF only] Commands History = first 500 of ("Cam Autofocus Run" + Commands History)     # "Log last 500 commands"
(params, err) = VariantToData(Command Data, ...)
X = session of params.Camera Type ("High Mag." -> HM, else LM)
(state, data, progress, finished, e1) = <sub-VI>(State In = X.<AF State|RF State>, params, ..., error in = err)
X.<AF State|RF State> = state ; X.Last Focus Data = data ; X.Focus Progress (%) = progress     # written even on error
[RF, Low Mag frame only] front-panel indicator "Autofocus Data" = data
if not finished:  e2 = Enqueue Element(Cam Queue, <element with "Cam … Run">, error in = e1)   # timeout -1
else:             e2 = e1
Log Server Error(extra = "Autofocus Run" | "Refocus Run", e2 with source + "Autofocus" | "Cam Refocus Run")
# no response to any client ; stop = False
```

In the Low Mag frame of `Cam Refocus Run`, both session inputs of the sub-VI are wired to the Low Mag session. This has no effect,
because the sub-VI selects its session by `Camera Type` **(inferred)** [main VI frame d210].

### 3.3 Read-back commands [main VI frames d191, d193, main VI PDF p.26]

```
Cam Get Autofocus Data:     (d, err) = VariantToData(...) ; S = (d.Camera Type == "High Mag.") ? HM : LM
                            Enqueue Send DBL 2D Array Response(S.Last Focus Data, error = err)      # comment "Send Last autofocus data"
Cam Get Autofocus Progress: Enqueue Send DBL Response(S.Focus Progress (%), error = err)             # comment "Send autofocus progression"
```

These typed responses are always sent, with the decoding error as data. Decoration and logging only see the error of the send itself
(queue errors).

### 3.4 Resulting behaviour of the polling pattern

1. **Scheduling.** Each `Run` step goes to the tail of the camera queue. Client commands that are already queued (image grabs,
   progress polls…) run in between. Steps follow each other back-to-back when the queue is otherwise empty. Each step lasts at least
   1 ms, plus the PI serial round trips, plus the image grab and processing in measurement steps. Sleeps happen *inside* steps: 100 ms
   on reaching a Slow-AF position, `Wait (ms)` on reaching a refocus position. The whole camera loop is blocked during each step.
2. **Normal end.** The sub-VI returns `Finished = True` from its `Finished` (or `Error`) state. Nothing is re-posted and the session
   keeps `AF State = Finished` / `RF State = Finished`.
3. **Error during a background step.** The sub-VI moves to state `Error` but returns `Finished = False` with `error out` set. That
   error is wired to the `error in` of the re-posting `Enqueue` [main VI frame d205], so the `Enqueue` **does not run** (standard
   LabVIEW error semantics) and the job stops right there. The `Error` state, whose role is to "restore initial speed, initial
   position", is **never executed**. The stage stays where it is, at whatever speed was last set (for example the Fast-AF scan
   speed). The session keeps state `Error` and the progress of the failed step (below 100, except after the last measurement). See
   Defects D1.
4. **Deadlock risk.** The camera loop enqueues into **its own** queue (capacity 5, timeout −1). If the TCP loop has filled the queue
   in the meantime, the only consumer blocks on its own producer call and the camera loop freezes **(inferred)**.
5. **One procedure at a time, not enforced (inferred from the wiring).** Both sub-VIs keep one global internal state, and nothing
   checks for a running job. A second `Cam Autofocus` while one is running resets the internal state and adds a second circulating
   `Run` element, so the job then advances twice per cycle. A start for the other camera resets the internal data while the first
   camera's `Run` elements keep stepping from that camera's stored state with their own parameters. Autofocus and refocus can run
   interleaved on the same Z stage. See Defects D4.
6. **History side effects.** Every `Cam Autofocus Run` step adds an entry to `Commands History` (max 500), so an autofocus quickly
   fills it. Every camera-loop iteration, `Run` steps included, also appends to the unbounded error and temperature histories of the
   camera loop (see [01-architecture.md](01-architecture.md)).

---

## 4. Autofocus state machine (`alox_camera_server_autofocus.vi`)

### 4.1 Interface [Doc6 p.603–604]

| Inputs | Type | Notes |
|---|---|---|
| `AF State In` | `AF State` | default `Start` |
| `Camera Autofocus Data` | §2.2 | |
| `High Mag Session`, `Low Mag Session` | camera session clusters | chosen by `Camera Type` (fields `Session`, `SRC Image`, `FFC Image`, `DFC Image`) |
| `High Mag Settings`, `Low Mag Settings` | camera settings clusters | fields `Horizontal Flip`, `Vertical Flip`, `Flat Field`, `Flat Field Factor` |
| `Z Session` | `{System, Axes, Identity, Serial Number}` | only `System` and `Axes` are used |
| `Z Stage Settings` | `{Com, Baudrate, Axis, Speed (mm/s), Slow Speed (mm/s), Min Position (mm), Max Position (mm)}` | only `Speed (mm/s)` is used (positional unbundle, 4th element) |
| `Display Ctrl` | reference to an Image display | shows the measured image with the ROI |
| `Focus Graph` | reference to an XY graph | shows the curve |
| `error in` | error cluster | |

| Outputs | Type | Notes |
|---|---|---|
| `AF State Out` | `AF State` | next state |
| `Autofocus Data` | DBL 2-D | `AF Data` from the internal memory after the step; empty when `error in` carries an error |
| `Progress (%)` | DBL | §2.7 |
| `Finished` | Boolean | True only in states `Finished` and `Error` |
| `error out` | error cluster | the stored error |

A second graph, `Focus Graph 2`, has an unwired terminal and no function [Doc6 p.605].

In the main VI the `Z Stage Settings` input of both sub-VIs carries a coercion dot [main VI frames d181, d186]: the front-panel cluster
passed in differs in some element type from the sub-VI's typedef (nature not readable). LabVIEW converts such clusters element by
element, so the positional unbundle still gets `Speed (mm/s)` as the 4th element **(inferred)**.

### 4.2 Persistent internal state: `Fast AF Session` [Doc6 p.596]

Despite its name, this memory is used by both modes. It is held in an uninitialised shift register, together with the stored error
cluster (`SR_error`).

| Field | Type | Written by | Read by |
|---|---|---|---|
| `Duration (ms)` | DBL | `Wait Started` (Fast): `Duration (s) × 1000` | `Measure` (Fast) |
| `T0` | DBL (tick count) | `Wait Started` (Fast) | `Measure` (Fast) |
| `Last Timestamp` | integer (tick count) | `Measure` | nobody |
| `Current Z` | DBL mm | `Start` (= `Z Min`), `Measure` Slow (= `Current Z + Step`) | `Goto Z Pos`, `Measure` Slow |
| `Init Speed (mm/s)` | DBL | `Start` (`VEL?`) | `Finished`, `Error` |
| `Init Z (mm)` | DBL | `Start` (`POS?`, before any move) | `Error` |
| `AF Data` | DBL 2-D (rows `[Z, score]`) | `Start` (reset), `Measure` (one row appended) | output `Autofocus Data` |
| `Move Timestamp` | integer (tick count) | `Start` | `Goto Z Pos` (Fast timeout) |

`Start` rebuilds the whole cluster from a typedef constant, assumed to hold all-zero / empty values **(inferred: the constant is shown
as an icon [Doc6 p.607])**. `Start` resets `SR_error`; `Goto Z Pos` (Fast) and `Wait Started` (Fast, still at `Z Min`) also
overwrite it (§4.8).

### 4.3 Outer structure [Doc6 p.606, p.610]

```
if error in is an error:                       # outer case "Error" (p.610)
    AF State Out = AF State In ; Autofocus Data = [] ; Progress = 0 ; Finished = False ; error out = error in
    (the loop does not run: internal memory untouched)
else:                                          # "No Error"
    comment: "TODO: Check AF Zmin/Zmax in inside Z stage Zmin/Zmax range"   (not implemented)
    isHigh = (Camera Type == "High Mag.") ; CAM = isHigh ? High Mag Session : Low Mag Session
    FFC = isHigh ? High Mag Settings : Low Mag Settings
    one loop iteration (label "Fast Autofocus state machine"), Wait 1 ms in parallel:
        case state: Default | Start | Goto Z Pos | Wait Started | Measure | Finished | Error
    Autofocus Data = AF Data ; error out = SR_error
```

### 4.4 State diagram

```
                 err                                                     err
 Start ───────────────────────────────► Error ──► Finished ◄─────────────────────────────┐
   │ ok                                   ▲          ▲  (loops on itself, Finished=True)   │
   ▼                                      │ err      │                                     │
 Goto Z Pos ◄────────────────────────┐    │          │                                     │
   │  Slow: reached (±1 µm) + 100 ms  │    │          │                                     │
   │  ─────────────────────────────► Measure ─(Slow: Znext > Z Max)──┘                     │
   │                                  │ ▲  ─(Slow: Znext ≤ Z Max)──► Goto Z Pos            │
   │  Fast: reached Z Min ±1 µm       │ │                                                  │
   │  VEL fast, MOV Z Max             │ └─(Fast: elapsed < duration)  ↺                    │
   ▼                                  │                                                    │
 Wait Started ─(left Z Min by >1 µm)──┘   (Fast: elapsed ≥ duration) ──────────────────────┘
   ↺ still at Z Min (no timeout)
 Goto Z Pos ↺ not reached (Slow: no timeout; Fast: 30 s since Start → error 9401 → Error)
```

### 4.5 States in detail

Notation: `POS?`, `VEL?`, `VEL`, `MOV` are the PI driver VIs called with `System no. = Z Session.System` and `Axes = Z Session.Axes`.
Values are passed as one-element arrays and element [0] of each reply is used. `inRange(x, lo, hi)` is inclusive on both bounds
[Doc6 p.607].

#### `Start`: "Check we are in start position (Zmin)" [Doc6 p.607]

```
Focus Graph := empty                                        # clears the graph
err := no error                                             # SR_error is ignored (reset)
init_speed := VEL?()[0]      ; err := ClearErrors(err)      # "Get initial speed (will be restored at the end)"
VEL([Z Stage Settings.Speed]) ; err := ClearErrors(err)     # "Set Normal Speed"
pos := POS?()[0]                                            # "check Zmin pos with µm accuracy"; error kept
if not inRange(pos, Z Min − 0.001, Z Min + 0.001):
    MOV([Z Min])                                            # "Move to start position" (skipped if POS? failed)
else: nothing                                               # "Already at start position"
session := { all default, Init Z = pos, Init Speed = init_speed, Current Z = Z Min, Move Timestamp = tick() }
state := err ? Error : Goto Z Pos ; Progress = 0 ; Finished = False ; SR_error := err
```

`Init Z` is the position **before** the move to `Z Min`. If `VEL?` fails its error is discarded and `Init Speed` becomes 0.0. If
`POS?` fails, `Init Z` becomes 0.0 and the state becomes `Error` (Index Array default).

#### `Goto Z Pos`: "Check Current Z (slow) or Start Z (fast) position reached" [Doc6 p.607–608]

```
p := POS?()[0] ; ClearErrors                                # a failed POS? reads as 0.0
reached := inRange(p, Current Z − 0.001, Current Z + 0.001)
case Autofocus Speed:
  "Fast":                                                   # "Fast method: measure continuously while Z is moving"
     if reached:                                            # "Start Position reached => send order to move to Zmax,
                                                            #  then wait until the stages starts moving"
         VEL([ |Z Max − Z Min| / Duration (s) ])            # "Set Fast Speed" — "Fast Z speed(mm/s)"
         MOV([Z Max])                                       # "Move to Zmax"
         next := Wait Started ; e := errors of VEL/MOV      # the previous SR_error is discarded
     else:                                                  # "Wait for Start Position..." / "Check Z timeout"
         if tick() − Move Timestamp > 30000: e := {9401, "z stage get position timeout"} else e := no error
         next := Goto Z Pos
  "Slow", Default:                                          # "Slow Method: move Z step by step, and measure while Z stopped"
     if reached: Wait(100 ms) ; next := Measure             # "Make sure the stages has stopped moving"
     else:       next := Goto Z Pos                         # no timeout
     e := SR_error (unchanged)
state := e ? Error : next ; Progress = 0 ; Finished = False
```

The 30 s timeout counts from `Start` (`Move Timestamp` is set only there). The Slow branch has **no timeout**.

#### `Wait Started`: "Wait until the stages have started to move from Zmin to ZMax" [Doc6 p.608–609]

```
"Fast":  p := POS?()[0] ; ClearErrors
         if inRange(p, Z Min − 0.001, Z Min + 0.001):       # "Waiting until z stage starts moving..."
             state := Wait Started ; SR_error := no error   # no timeout
         else:                                              # "Z stage has started to move, store t0 timestamp and measure"
             T0 := tick() ; Duration (ms) := Duration (s) × 1000 ; state := Measure
"Slow", Default:  state := Measure                          # not reached in a normal Slow run
Progress = 0 ; Finished = False                             # this state never selects Error
```

#### `Measure`: "Measure Focus for current Z position, store each measure in data array" [Doc6 p.605–606]

Common acquisition and measurement chain (Fast and Slow):

```
err := SR_error                                             # if set, the NI VIs below do nothing and the score is 0
img := IMAQdx Get Image2(CAM.Session, Image In = CAM.SRC Image, Mode = "Next")   # Timeout, Buffer Number not wired
       # a case structure selects simulation vs camera, but its selector is a constant False: simulation is never used
ti  := tick()                                               # flat sequence after the grab (runs even if the grab failed)
img := alox_vision_flip_image(img, Horizontal Flip, Vertical Flip)
if Flat Field: img := IMAQ Flat Field Correction(img, FFC Image, DFC Image, {Line Scan = False, Correction Factor = Flat Field Factor})
score := case Autofocus Method:
           "FFT":            alox_vision_measure_focus_fft(img, Optional Rectangle = ROI, Scale Factor = Subsampling)
           "Prewitt":        alox_vision_measure_focus_sobel(img, ROI, Subsampling, Method = Prewitt)     # FULL not wired: sub-VI default, presumably False (inferred)
           "Sobel", Default: alox_vision_measure_focus_sobel(img, ROI, Subsampling, Method = Sobel, FULL = False)
IMAQ Overlay Rectangle(img, ArrayToCluster(ROI), red, Frame) ; ClearErrors   # overlay errors ignored
Display Ctrl.Value := img                                   # in the error chain
```

The flip and flat-field steps are specified in [05-camera-acquisition.md](05-camera-acquisition.md), the metrics in
[07-focus-metrics.md](07-focus-metrics.md). The metric VIs return score 0 when they receive or raise an error.

**Fast** ("Measure focus asynchronously while moving"):

```
dZ   := Z Max − Z Min
Zest := Z Min + dZ · ((DBL(ti) − T0) / Duration (ms))       # comment: "current estimated Z = zmin + dZ*((ti-t0)/duration)";
                                                            # wired in that order: divide, then multiply [Doc6 p.606]
AF Data += [Zest, score] ; Last Timestamp := ti ; Focus Graph := AF Data
Progress := (Zest − Z Min) / dZ · 100
next := (ti − T0 ≥ Duration (ms)) ? Finished : Measure
state := err ? Error : next ; Finished = False
```

**Slow** ("Measure focus at current position, then move to next position, unitl reaching Zmax"):

```
Zpos  := Current Z                                          # commanded value, not read back
AF Data += [Zpos, score] ; Last Timestamp := ti ; Focus Graph := AF Data
Znext := Zpos + Step ; Current Z := Znext
MOV([Znext])                                                # "Move to next Z position" — always sent (unless err)
Progress := (Znext − Z Min) / (Z Max − Z Min) · 100
next := (Znext ≤ Z Max) ? Goto Z Pos : Finished
state := err ? Error : next ; Finished = False
```

- The append has no error case around it, so a failed measurement still adds a row with score 0 **(inferred from the absence of a
  case structure)**.
- The positions come from **repeated addition**, which decides whether `Z Max` itself is measured. Example: `Z Min = 1.0`, `Step = 0.1`,
  `Z Max = 1.5` measures 1.0, 1.1, 1.2000000000000002, 1.3000000000000003, 1.4000000000000004 only, because the fifth addition gives
  1.5000000000000004 > 1.5. A re-implementation must use the same arithmetic (`z = z + step`, IEEE double) to get the same point set.
- The final `MOV` sends the stage **one step beyond the last measured point**, which is always **beyond `Z Max`** (that is the
  stop condition). No later state brings it back.

#### `Finished`: "Finished. Restore initial speed" [Doc6 p.609]

`VEL([Init Speed])` with errors cleared; `state := Finished`; `Progress = 100`; `Finished = True`; `error out = SR_error`. The stage is
left where the scan ended.

#### `Error`: "Error happened. Restore initial speed, initial position" [Doc6 p.609]

`VEL([Init Speed])`, Clear Errors, `MOV([Init Z])`, Clear Errors (no wait for the move); `state := Finished`; `Progress = 100`;
`Finished = True`; `error out = SR_error`, so the original error is still reported. The main VI never executes this state (§3.4).

#### `Default`

Pass-through: state, memory and error unchanged; `Progress = 0`; `Finished = False`.

### 4.6 Output summary per state

| State executed | Progress | Finished | error out | Next state |
|---|---|---|---|---|
| (error in set) | 0 | F | error in | unchanged; data empty |
| `Start` | 0 | F | `POS?`/`MOV` errors (stored error reset) | `Goto Z Pos` / `Error` |
| `Goto Z Pos` Fast | 0 | F | `VEL`/`MOV` or 9401 (stored error reset) | `Wait Started` / `Goto Z Pos` / `Error` |
| `Goto Z Pos` Slow | 0 | F | stored error | `Measure` / `Goto Z Pos` / `Error` |
| `Wait Started` Fast | 0 | F | not left: no error; left: stored error | `Measure` / `Wait Started` |
| `Wait Started` Slow | 0 | F | stored error | `Measure` |
| `Measure` Fast | `(Zest−Zmin)/dZ·100` | F | image chain | `Measure` / `Finished` / `Error` |
| `Measure` Slow | `(Znext−Zmin)/dZ·100` | F | image chain + `MOV` | `Goto Z Pos` / `Finished` / `Error` |
| `Finished` | 100 | T | stored error | `Finished` |
| `Error` | 100 | T | stored error | `Finished` |

### 4.7 Slow vs Fast in practice

| | Slow | Fast |
|---|---|---|
| Stage motion | absolute `MOV` per point at the normal speed (`Z Stage Settings.Speed`) | one `MOV Z Max` at `abs(Z Max − Z Min) / Duration (s)` mm/s |
| Points | `Z Min, Z Min+Step, …` while `≤ Z Max` (repeated addition) | one per `Measure` step until `Duration` has elapsed. The count depends on frame rate and processing time **(inferred)** |
| Z of a point | commanded position, reached within ±1 µm, then 100 ms settle | `Z Min + dZ·(ti − T0)/Duration`. `ti` is taken after the grab returns, `T0` when the stage was first seen > 1 µm away from `Z Min`. Acceleration, exposure time and transfer latency are not modelled, which gives a systematic bias of about speed × latency **(inferred)** |
| Timeouts | none | 30 s to reach `Z Min` (from `Start`); none for "start moving" |
| End position | `last point + Step` (one step beyond) | `Z Max` |
| Scan direction | `Z Max < Z Min` measures one point, then finishes **(inferred)** | works in both directions (`abs` in the speed) **(inferred)** |

### 4.8 Error handling summary

- **Ignored** (Clear Errors): `VEL?` and `VEL` in `Start`; `POS?` in `Goto Z Pos` and `Wait Started`; ROI overlay; every restore
  command in `Finished` / `Error`.
- **Stored and leading to `Error`**: `POS?`/`MOV` in `Start`; `VEL`/`MOV`/timeout 9401 in `Goto Z Pos` Fast; grab, flip, flat field,
  metric, image display and `MOV` in `Measure`.
- Error **9401 "z stage get position timeout"** is the only error code created by this VI.
- The stored error is cleared by `Start`, by `Goto Z Pos` Fast, and by `Wait Started` Fast while the stage has not left `Z Min`. It is
  passed through everywhere else, and it short-circuits the next `Measure`.

---

## 5. Refocus state machine (`alox_camera_server_refocus.vi`)

### 5.1 Interface [Doc6 p.1030–1031]

The inputs are the same as for the autofocus (§4.1), with `Refocus State In` and `Camera Refocus Data`. `Z Stage Settings` supplies
`Speed (mm/s)` (positional unbundle) and `Min Position (mm)` / `Max Position (mm)` (by name). Outputs: `Refocus State Out`,
`Autofocus Data` (N×2), `Progress (%)`, `Finished`, `error out`.

### 5.2 Persistent internal state: `Refocus Session` [Doc6 p.661]

| Field | Type | Written by | Read by |
|---|---|---|---|
| `Positions` | DBL[] mm | `Start` (cleared), `Get Positions` | moves, checks, output |
| `Scores` | DBL[] | `Start`, `Get Positions` (cleared), `Mesure Position` (append) | `Get Focus`, output |
| `Index Pos` | integer (I32 **(inferred)**) | `Get Positions` (0), `Mesure Position` (+1), `Get Focus` (argmax) | moves, checks, image file name |
| `Focus Pos`, `Focus Score` | DBL | `Get Focus` | nobody |
| `Init Z (mm)` | DBL | `Start` (`POS?`) | `Get Positions`, `Error` |
| `Init Speed (mm/s)` | DBL | `Start` (`VEL?`) | `Finished`, `Error` |
| `Move timestamp` | integer (tick count) | `Move Position`, `Move Focus` | `Check Position`, `Wait Focus` |

The stored error cluster is a second uninitialised shift register. Only `Start` resets it.

### 5.3 Outer structure [Doc6 p.1032–1033, p.1038]

```
if error in is an error:                        # outer case "Error" (p.1038)
    Refocus State Out = "Error" ; Autofocus Data = [] ; Progress = 0 ; Finished = TRUE ; error out = error in
else:
    CAM/FFC selected by Camera Type as for the autofocus ; one loop iteration with Wait 1 ms ("Refocus state machine")
    Autofocus Data = Transpose(Build Array(Positions, Scores))     # zero padding of missing scores
```

Note the difference from the autofocus: on an incoming error the refocus reports `Finished = True` and state `Error`.

### 5.4 State diagram

```
Start ─(joystick on: 9031, or POS?/VEL?/VEL error)─► Error (absorbing, Finished=True)
  │ ok                                                   ▲ any error in a later state
  ▼                                                      │
Get Positions ──► Move Position ──► Check Position ──(reached + Wait ms)──► Mesure Position
                       ▲                 ↺ not reached (5 s per move → 9401)     │
                       └──────────────(index+1 < N)──────────────────────────────┤
                                                                                 │ index+1 ≥ N
Finished (absorbing) ◄──(reached + Wait ms)── Wait Focus ◄── Move Focus ◄── Get Focus
                             ↺ not reached (5 s → 9401)
```

### 5.5 States in detail

#### `Start`: "Init Refocus, store initial position and speed, set normal speed" [Doc6 p.1034]

```
err := no error ; session := Refocus Session typedef constant   # shown as an icon: empty arrays and zeros (inferred) [Doc6 p.1034]
status := JON?(System, JoystickID = [1], All joysticks? = False)       # error in/out not wired: a failed query gives an
                                                                       # empty array, read as 0 (inferred)
if status[0] != 0:                                                     # "Joystick Enabled"
    err := {status TRUE, code 9031, source "'refocus' warning (joystick enabled)"}
    # session keeps its defaults: Init Z = 0, Init Speed = 0 (inferred)
else:                                                                  # "Joystick Disabled"
    Focus Graph := empty                                               # "Init Graph", errors cleared
    Init Z     := POS?()[0]                                            # "Get Initial Pos."
    Init Speed := VEL?()[0]                                            # "Get initial speed (will be restored at the end)"
    VEL([Z Stage Settings.Speed])                                      # "Set Normal Speed"   (chained: one failure skips the rest)
    # a Diagram Disable structure contains a disabled "MAC START SLOWACC"; the active frame does nothing
state := err ? Error : Get Positions ; Progress = 0 ; Finished = False
```

The text says "warning", but the status is TRUE, so the joystick case is a blocking error.

#### `Get Positions`: "Pre-compute positions" [Doc6 p.1034]

```
N       := Steps · 2 + 1                                   # label "Iterations"
Z start := Init Z − Steps · Step (mm)                      # label "Z start"
for i in 0 .. N−1:
    Z := Z start + i · Step (mm)                           # multiplication, not repeated addition
    Positions[i] := InRangeAndCoerce(Z, Min Position, Max Position)   # inclusive; comment "Zmin <= Z <= Zmax ?"
    # out-of-range case: empty frame with comment "#TODO: Log Z limit reached"
Scores := [] ; Index Pos := 0
state := Move Position (unconditional) ; Progress = 0 ; Finished = False
```

Close to a limit, several positions are clamped to the same value and are measured several times. `Steps < 0` produces no positions;
`Steps = 0` produces one position (the current Z).

#### `Move Position`: "Move to next Z position" [Doc6 p.1035]

```
MOV([Positions[Index Pos]])                                 # absolute, not waited for; an out-of-range index reads 0.0
Move timestamp := tick()                                    # flat sequence after the MOV
Progress := clamp(Index Pos / N · 100, 0, 100)
state := err ? Error : Check Position ; Finished = False
```

#### `Check Position`: "Check next Z position reached" [Doc6 p.1035]

```
if tick() − Move timestamp > 5000: err := {9401, "z stage get position timeout"}    # replaces the current error
pos := POS?()[0] ; ClearErrors                                                      # "check Z pos with µm accuracy"
reached := (target − 0.001) ≤ pos < (target + 0.001)        # upper bound EXCLUDED (hollow diamond), target = Positions[Index Pos]
if reached: Wait(Wait (ms))                                  # "Make sure the Z stage has stopped moving"
Progress := clamp(Index Pos / N · 100, 0, 100)
state := err ? Error : (reached ? Mesure Position : Check Position)
```

The pause runs even when the timeout fired in the same call. The state then becomes `Error` anyway.

#### `Mesure Position`: "Measure Focus for current Z position, store each measure in data array" [Doc6 p.1035–1036]

```
t0  := tick()                                               # before the grab
img := IMAQdx Get Image2(CAM.Session, CAM.SRC Image, Mode = "Next")    # simulation case selector = constant False
img := flip(img) ; if Flat Field: img := IMAQ Flat Field Correction(...)       # as in the autofocus
(W, H) := IMAQ GetImageSize(img)
ROI := Fin ROI (§5.6)                                       # the "Refocus Method" selector is a constant "Fin"
(img, score) := metric(img, Autofocus Method, ROI, Scale Factor = 0.5)          # §5.7
IMAQ Overlay Rectangle(img, ROI, red, Frame)                # in the error chain
tools_log_refocus_image(Log = Save Images, img, Folder, Index = Index Pos)      # §8 (before the increment)
Scores := Scores + [score] ; Index Pos := Index Pos + 1
Focus Graph := (Positions, Scores) as XY points             # only measured points; errors cleared
Display Ctrl.Value := img                                   # "Refresh Image", error not wired
err.source := FormatIntoString(U32(t0 − tick()))            # t0 is the upper (x) input of the subtraction [Doc6 p.1035];
                                                            # overwrites the error source on every measurement (Defect D10)
Progress := clamp(Index Pos / N · 100, 0, 100)              # with the incremented index
state := err ? Error : (Index Pos ≥ N ? Get Focus : Move Position)
```

As in the autofocus, a failed measurement still appends score 0 and still increments the index, because the append is unconditional
**(inferred from the absence of a case structure)**.

The second `Tick Count` of the `source` computation sits outside any sequence, so LabVIEW may run it before or after `t0` is taken.
The text is therefore the decimal form of `2³² − elapsed` when it runs after `t0` (a value near 4 294 967 296), or a small number
when it runs first **(inferred)**.

#### `Get Focus`: "Get best focus position/score/index" [Doc6 p.1036]

```
best_pos := 0 ; best_score := −Inf ; best_idx := 0
for i over min(len(Positions), len(Scores)):
    if Scores[i] > best_score: (best_pos, best_score, best_idx) := (Positions[i], Scores[i], i)   # strict ">": first maximum wins
Focus Pos := best_pos ; Focus Score := best_score ; Index Pos := best_idx
state := err ? Error : Move Focus ; Progress = 100
```

There is no sub-step interpolation: the best position is one of the grid points. NaN scores never win. If every score is NaN, index 0 is
selected.

#### `Move Focus`, `Wait Focus` [Doc6 p.1037]

`Move Focus` ("Move to found focus position") sends `MOV([Positions[Index Pos]])`, sets `Move timestamp` and goes to `Wait Focus`.
`Wait Focus` ("Wait for the Z stage to be in best focus position") is identical to `Check Position`: same 5 s / 9401 timeout, same
`[t − 1 µm, t + 1 µm)` window, `Wait (ms)` once reached. It then goes to `Finished`. Both report progress 100.

#### `Finished`: "Finished. Restore initial speed" [Doc6 p.1037]

`VEL([Init Speed])`, error cleared (a disabled `MAC START FASTACC` sits next to it). State stays `Finished`, `Progress = 100`,
`Finished = True`, `error out` = stored error. The stage stays at the best position.

#### `Error`: "Error happened. Restore initial speed, initial position" [Doc6 p.1033]

`VEL([Init Speed])`, Clear Errors, `MOV([Init Z])`, Clear Errors (disabled `MAC START FASTACC`). State stays `Error` (absorbing),
`Progress = 100`, `Finished = True`, `error out` = stored error. If this state ran after the joystick error, it would send `VEL 0` and
`MOV 0.0` from the default session (zeros **(inferred)**, see `Start`). The same holds after a failed `POS?` in `Start`. That never
happens in the current program, because the main VI never executes the `Error` state (§3.1, §3.4). It **would** happen in a
re-implementation that fixes D1 without a guard (Defects D11).

#### `Default` [Doc6 p.1032]

Pass-through, `Progress = 0`, `Finished = False`.

### 5.6 ROI computation (`"Fin", Default` branch) [Doc6 p.1035]

Comment: "Refocus Fin ( Rectangular ROI centered in image)". `W`, `H` = image width and height after flip and flat field.

| `Camera Type` | ROI `[Left, Top, Right, Bottom]` |
|---|---|
| `"High Mag.", Default` | `hw = round(FinW·0.5)`, `hh = round(FinH·0.5)`, `cx = round(W·0.5)`, `cy = round(H·0.5)` → `[cx−hw, cy−hh, cx+hw, cy+hh]` (label "ROI Rectangle") |
| `"Low Mag."` | `[FinW, FinH, W, H]`: **not centred**. `FinW`/`FinH` act as left/top margins |

- The four `x*0.5` expression nodes take and return integers. The rounding is assumed to be LabVIEW's half-to-even conversion
  **(inferred)**. Example: `FinW = 101` gives `hw = 50`, `FinH = 103` gives `hh = 52`.
- The rectangle is not clipped to the image. Behaviour of the metric for a rectangle partly outside the image: see
  [07-focus-metrics.md](07-focus-metrics.md) **(unknown in NI terms)**.

### 5.7 Metric per `Autofocus Method` (refocus) [Doc6 p.1035–1036]

All calls use `Scale Factor = 0.5` and the ROI above. Formulas are in [07-focus-metrics.md](07-focus-metrics.md).

| Method | Call |
|---|---|
| `FFT` | `measure_focus_fft(img, ROI, 0.5)` |
| `Sobel` | `measure_focus_sobel(img, ROI, 0.5, Method = Sobel, FULL = False)` |
| `Prewitt` | `measure_focus_sobel(…, Method = Prewitt, FULL = False)` |
| `Diff` | `measure_focus_sobel(…, Method = Differentiation, FULL = False)` |
| `Gradient` | `measure_focus_sobel(…, Method = Gradient, FULL = False)` |
| `Sigma` | `IMAQ LowPass(img, {X 20, Y 20, Tolerance 40 %})` **in place on the camera image** (error not wired), then `measure_focus_sobel(…, Method = Sigma, FULL = False)` |
| `FULL` | in parallel: `measure_focus_sobel(img, ROI, 0.5, FULL = True)` and `measure_focus_fft(img, Optional Rectangle not wired = whole image, 0.5)`; score = product; only the Sobel error is propagated |

Because the Sigma filter works in place, the displayed and saved image is the filtered one.

### 5.8 Output summary per state

| State | Progress | Finished | Next (no error) | Hardware access |
|---|---|---|---|---|
| (error in set) | 0 | **T** | `Error` | none |
| `Start` | 0 | F | `Get Positions` | `JON?`, `POS?`, `VEL?`, `VEL` |
| `Get Positions` | 0 | F | `Move Position` | none |
| `Move Position` | idx/N·100 | F | `Check Position` | `MOV` |
| `Check Position` | idx/N·100 | F | `Check Position` / `Mesure Position` | `POS?` (+ `Wait (ms)`) |
| `Mesure Position` | (idx+1)/N·100 | F | `Move Position` / `Get Focus` | camera, disk (optional) |
| `Get Focus` | 100 | F | `Move Focus` | none |
| `Move Focus` | 100 | F | `Wait Focus` | `MOV` |
| `Wait Focus` | 100 | F | `Wait Focus` / `Finished` | `POS?` (+ `Wait (ms)`) |
| `Finished` | 100 | T | `Finished` | `VEL` |
| `Error` | 100 | T | `Error` | `VEL`, `MOV` |

### 5.9 Dead `Oxyd` branch (documentation only) [Doc6 p.1036]

Comment: "Refocus Oxyd (Circular ROI centered on VCSEL/MESA)". It builds a circular ROI from `Oxyd X/Y Pos` and `Oxyd Diam` through a
sub-VI "OXYD ROI", computes two scores per position ("FFT OXYD" and "SOBEL OXYD"), and draws an oval overlay. These sub-VIs are not in
the printouts **(unknown)**. Two scores per position would misalign `Scores` with `Positions`. The branch cannot execute, because its
selector is a constant. **Do not implement** unless the users ask for it (Open questions).

---

## 6. Image capture, ROI and subsampling (both procedures)

### 6.1 Capture chain

1. `IMAQdx Get Image2` with `Buffer Number Mode = "Next"`. The timeout is not wired, so the VI default of **5000 ms** applies
   [Doc6 p.895–896]. The image goes into the session's `SRC Image` buffer, and flip and flat field also work in place on that buffer.
   It is shared with the other image commands, so after a focus run it holds the last focus frame, corrected, with the ROI overlay
   attached. `Cam Stream Next Image` overwrites it and clears all overlays before its next answer (see
   [05-camera-acquisition.md](05-camera-acquisition.md) §7.5). The acquisition must already be configured and running
   (`Cam Setup` / `Cam Start`). The focus VIs never start it. Behaviour if acquisition is stopped: **(unknown)** (error or timeout).
2. Software flip (`Horizontal Flip`, `Vertical Flip` of the camera settings), then optional flat-field correction (`Flat Field`,
   `Flat Field Factor`, session images `FFC Image` / `DFC Image`). See [05-camera-acquisition.md](05-camera-acquisition.md).
3. Focus metric on the ROI with subsampling (§6.3).
4. Red `Frame` rectangle overlay of the ROI. In NI Vision an overlay is metadata attached to the image, not drawn into the pixels
   **(inferred from NI semantics)**.
5. The image goes to the `Image` indicator, and the curve to the `Focus Graph` XY graph (UI only).

The **simulation mode** of the camera session (`Simulation`, `Simulation Path`) is **never used** by the focus VIs: the selector of
their simulation case is a constant `False` [Doc6 p.605, p.1035]. A simulated camera therefore cannot run an autofocus or refocus.

"Next" returns the buffer that follows the last buffer handed to the application. With a continuously running ring acquisition, that
buffer may have been exposed **before** the call, for example before the stage settled, if frames have been waiting in the ring
**(inferred)**. Keep this in mind when replacing IMAQdx (see Defects D16).

### 6.2 Image vs position timing

| Procedure | When the image is grabbed | Z recorded |
|---|---|---|
| AF Slow | in the step after the one that saw `abs(POS? − target) ≤ 1 µm` and slept 100 ms | commanded target |
| AF Fast | at every `Measure` step, back-to-back, while the stage moves | `Z Min + dZ·(ti − T0)/Duration`, `ti` = time after the grab returned |
| Refocus | in the step after the one that saw `target − 1 µm ≤ POS? < target + 1 µm` and slept `Wait (ms)` | commanded target (`Positions[i]`) |

Whether `POS?` on the Mercury controller returns the actual or the target position decides whether the "reached" check means anything.
The PI documentation of `POS?.vi` says "actual or target position, depending on system" [Doc6 p.668]. The actual position is expected
for this controller **(inferred)**. See Open questions.

With a step of 1 µm or less, the previous position already lies inside the tolerance window of the new target, so the "reached" check
passes at once and the measurement does not wait for the move **(inferred)**.

### 6.3 ROI and subsampling conventions

- ROI = 1-D integer array `[Left, Top, Right, Bottom]` (NI rectangle). An **empty** array means the whole image. A non-empty array,
  `[0,0,0,0]` included, is passed to `IMAQ Extract 2`. Inclusive/exclusive right and bottom edges and out-of-image behaviour: see
  [07-focus-metrics.md](07-focus-metrics.md).
- Subsampling = `Scale Factor`, nearest-neighbour resampling to `rint(W·s) × rint(H·s)` (ties to even) after ROI extraction (see
  [07-focus-metrics.md](07-focus-metrics.md)). Autofocus: the client's `Subsampling`. With its typedef default 0 the target size is
  0 × 0, which probably makes the metric fail with score 0 **(inferred)**. Refocus: fixed 0.5.
- Autofocus overlay: `Array To Cluster(ROI)` gives `{Left, Top, Right, Bottom}`. Missing elements become 0 **(inferred)**.

### 6.4 UI side effects (optional in Python)

- `Focus Graph` is cleared in `Start` and redrawn after every measurement. Autofocus: rows converted to `{X = Z, Y = score}` points by
  `tools_focus_array_to_xy_graph_cluster.vi` [Doc6 p.591–593]. Refocus: `zip(Positions, Scores)`, measured points only, by
  `tools_refocus_arrays_to_xy_graph_cluster.vi` [Doc6 p.1028].
- `Image` indicator: the measured image with the ROI frame.
- Python: expose these through an optional observer (preview callbacks). They are not part of the protocol.

---

## 7. Z stage interaction

### 7.1 Commands sent directly by the focus VIs

The focus VIs call the PI GCS driver directly with `System no. = Z Session.System` and `Axes = Z Session.Axes`. They do **not** go
through the Z command queue. Frames are shown for axis `"1"` (GCS 2.0 syntax, LF terminator, 4 decimals with a forced decimal point:
see [08-z-stage.md](08-z-stage.md)).

| State | Driver VI | GCS frame | pipython equivalent | Error handling |
|---|---|---|---|---|
| AF `Start` | `VEL?` | `VEL? 1` → `1=<v>` | `dev.qVEL(ax)[ax]` | ignored (value 0.0 on failure) |
| AF `Start` | `VEL` | `VEL 1 <Speed>` then `ERR?` | `dev.VEL(ax, v)` | ignored |
| AF `Start` | `POS?` | `POS? 1` → `1=<z>` | `dev.qPOS(ax)[ax]` | → `Error` |
| AF `Start` | `MOV` | `MOV 1 <Z Min>` (only if not already there) | `dev.MOV(ax, z)` | → `Error` |
| AF `Goto Z Pos`, `Wait Started` | `POS?` | `POS? 1` | `qPOS` | ignored (reads 0.0) |
| AF `Goto Z Pos` Fast | `VEL`, `MOV` | `VEL 1 <fast speed>` + `ERR?`, `MOV 1 <Z Max>` | `VEL`, `MOV` | → `Error` |
| AF `Measure` Slow | `MOV` | `MOV 1 <Znext>` | `MOV` | → `Error` |
| AF `Finished` | `VEL` | `VEL 1 <Init Speed>` + `ERR?` | `VEL` | ignored |
| AF `Error` | `VEL`, `MOV` | `VEL 1 <Init Speed>`, `MOV 1 <Init Z>` | `VEL`, `MOV` | ignored |
| RF `Start` | `JON?` | `JON? 1` → `1=<0/1>` | `dev.qJON([1])[1]` | not wired (failure = "disabled") |
| RF `Start` | `POS?`, `VEL?`, `VEL` | `POS? 1`, `VEL? 1`, `VEL 1 <Speed>` + `ERR?` | `qPOS`, `qVEL`, `VEL` | chained → `Error` |
| RF `Move Position`, `Move Focus` | `MOV` | `MOV 1 <target>` | `MOV` | → `Error` |
| RF `Check Position`, `Wait Focus` | `POS?` | `POS? 1` | `qPOS` | ignored (reads 0.0) |
| RF `Finished` | `VEL` | `VEL 1 <Init Speed>` + `ERR?` | `VEL` | ignored |
| RF `Error` | `VEL`, `MOV` | `VEL 1 <Init Speed>`, `MOV 1 <Init Z>` | `VEL`, `MOV` | ignored |

Driver facts that shape the behaviour (see [08-z-stage.md](08-z-stage.md)):

- `VEL.vi` "Sets velocity and checks for error" [Doc6 p.716]: it sends `ERR?` afterwards and turns a non-zero controller code into an
  error. A controller error left pending by an earlier command (for example a refused `MOV`) is therefore reported by the next `VEL`.
- `MOV.vi` does **not** query `ERR?`. A move refused by the controller (axis not referenced, target outside the controller travel
  range, an active joystick: PI's help states it explicitly for Mercury_GCS: "Motion commands like MOV are not allowed when a joystick
  is active on the axis" [Doc6 p.879]; the same for `MVR` [Doc6 p.1213] and in `JON.vi` [Doc6 p.1204]; see
  [08-z-stage.md](08-z-stage.md) §9) is not detected by the `MOV` itself. It only shows up as a position that is never reached
  **(inferred)**:
  - refocus: 9401 after 5 s (`Check Position` / `Wait Focus`);
  - Fast AF, `MOV Z Min` refused: 9401 after 30 s; `MOV Z Max` refused: the stage never leaves `Z Min` and `Wait Started` waits
    **forever**;
  - Slow AF: `Goto Z Pos` waits **forever**.
  The pending controller error is reported by the next `VEL` (with `ERR?`), which in the autofocus is only the ignored restore in
  `Finished` / `Error`.
- `VEL` and `MOV` values are sent with 4 decimals. A fast-AF speed below 0.00005 mm/s would be sent as `0.0000` **(inferred)**.
  Whether the firmware accepts `VEL 1 0.0000` is **(unknown)** (questionnaire Q35).

### 7.2 Tolerances and timeouts

| Item | Value | Where |
|---|---|---|
| Position window | ±0.001 mm, both bounds inclusive | AF `Start`, `Goto Z Pos`, `Wait Started` [Doc6 p.607–608] |
| Position window | `[t − 0.001, t + 0.001)` mm | RF `Check Position`, `Wait Focus` [Doc6 p.1035, p.1037] |
| Settle | 100 ms | AF Slow, when the position is reached |
| Settle | `Wait (ms)` | RF, when the position is reached |
| Timeout to reach `Z Min` | 30 000 ms since `Start` → 9401 | AF Fast only |
| Timeout per move | 5 000 ms since the `MOV` → 9401. A move that takes longer (long distance at low speed) fails even if the stage is still moving correctly **(inferred)** | RF |
| No timeout | — | AF Slow `Goto Z Pos`; AF Fast `Wait Started` |
| Error 9401 | `"z stage get position timeout"` | both |
| Error 9031 | `"'refocus' warning (joystick enabled)"` | RF `Start` |
| Grab timeout | IMAQdx default 5000 ms (`Acquisition Timeout (ms)` of the command unused) | both |

### 7.3 Speeds

- **Normal speed** = `Z Stage Settings.Speed (mm/s)` (INI `hardware.z/speed_DBL`, default 10). It is set by `VEL` at the start of both
  procedures. `Z Set Speed` does not change this setting (see [08-z-stage.md](08-z-stage.md)), so the focus procedures always scan at
  the INI speed.
- **Fast AF speed** = `|Z Max − Z Min| / Duration (s)`. `Duration (s) = 0` gives an infinite speed and `Z Max = Z Min` with duration 0
  gives NaN. Either way the controller rejects `VEL`, the state becomes `Error` and the job stops **(inferred)**. `Z Max = Z Min` with
  `Duration > 0` gives `VEL 1 0.0000`. If the controller rejects it, the job stops in `Error`; if it accepts it, the `MOV Z Max`
  targets the current position, the stage never leaves `Z Min` and `Wait Started` waits forever (controller behaviour
  **(unknown)**, questionnaire Q35).
- **Restored speed** = the value read by `VEL?` in `Start`, so a speed set earlier with `Z Set Speed` comes back at the end.
- `Slow Speed (mm/s)` is never used.

### 7.4 Concurrency and session source

- The focus VIs run in the camera loop, in parallel with the Z loop, and nothing locks the controller. LabVIEW's PI driver sends a
  command and reads its answer in two separate calls, so concurrent `Z …` commands from a client can interleave with focus queries
  **(inferred)**. In Python, all GCS traffic must go through **one** link object with an application-level lock held for each
  request/response pair (see [08-z-stage.md](08-z-stage.md)).
- `Z Session` is read from the **front-panel control's local variable**. `Z Setup` and `Z Cleanup` update it, but the automatic
  re-initialisation inside `Z Move Absolute/Relative` updates only the Z loop's shift register. After `Z Cleanup` it holds defaults
  (`System 0`, no axes), and every focus GCS call fails. Neither procedure checks `Z Inited` [main VI frames d181, d205].
- The software travel limits (`Min/Max Position`) are honoured only by the refocus (clamping). The autofocus does not check
  `[Z Min, Z Max]` against them: see its "TODO" comment [Doc6 p.606]. The Z loop's 9405 "Z move out of range" check is also bypassed.
- `Z Stage Settings` is read from a local variable at every step, so limits changed with `Z Set Min Limit` / `Z Set Max Limit` (see
  [08-z-stage.md](08-z-stage.md)) apply to the next refocus `Get Positions`. Positions already computed are not re-clamped.

### 7.5 Final stage state

| Outcome | Position | Speed |
|---|---|---|
| AF Slow finished | last measured point + `Step` | restored |
| AF Fast finished | `Z Max` | restored |
| RF finished | best grid position | restored |
| Any failure in a background step (LabVIEW) | wherever it was | unchanged (possibly the fast-scan speed) |
| Failure if the `Error` state were executed | `Init Z` (move not waited for) | restored |

---

## 8. Refocus image logging

### 8.1 `tools_log_refocus_image.vi` [Doc6 p.982–984]

Inputs `Log` (= `Save Images`), `Image In`, `Folder`, `Index` (= `Index Pos` before the increment, 0-based), `error in`.

```
if not Log: pass-through
else:
    name0 := tools_alox_job_data_filename(SEMI Job = typedef constant (shown as an icon; assumed empty, inferred),
                                          Timestamp = Get Date/Time In Seconds (now; ignored inside the helper, §8.2),
                                          Time format string = "%y%m%d_%H%M%S_refocus_")       # its error in/out not wired
    name  := name0 + NumberToDecimalString(Index, width = 2) + ".png"
    path  := Build Path(Folder, name)
    IMAQ Write Image And Vision Info File 2(Image, path, Image Quality = 1)    # error chained -> refocus goes to Error on failure
```

### 8.2 `tools_alox_job_data_filename.vi` [Doc6 p.973–976]

```
if error in: return ""
TS := FormatDateTimeString(Time format string, now)        # the "Timestamp" input exists but is NOT wired: current local time
TS := TS with the first "," removed, then the first "." removed
if SEMI Job == {"", "", "", "", ""}:   Output := TS + "_mesa"
else:
    f(x) := x if x != "" else "0"
    head := Process Job ID' + "_" + Lot ID' + "_" + Wafer ID'          # format "%s_%s_%s" with f() applied
    if Zone ID != "0": Output := head + "_" + TS + "_" + (Zone ID == "" ? "0" : "Zone" + Zone ID) + "_mapping"
    else:              Output := head + "_" + TS + "_oxidation"
error out := no error                                       # internal errors are discarded
```

`Control Job ID` is computed but never used. The SEMI branches cannot occur in the refocus, whose job constant is presumed empty
**(inferred)**. They are listed for completeness, because the same helper is reused elsewhere.

### 8.3 Resulting file names and format

- Name: **`<yymmdd>_<HHMMSS>_refocus__mesa<idx>.png`**, for example `231008_143210_refocus__mesa 3.png`.
  - Local time at the moment of writing, 1 s resolution.
  - **Double underscore** before `mesa`: the time format ends with `_` and the helper appends `_mesa`.
  - `<idx>`: `Number To Decimal String` with width 2. Values < 10 are padded to 2 characters, with **spaces** in LabVIEW
    **(inferred; to be confirmed on a LabVIEW machine)**. Values ≥ 100 use 3 characters.
- Folder: `Folder` from the command, used as is. Nothing creates it **(inferred)**. A missing folder or an empty path makes the write
  fail, and the refocus goes to `Error` (exact NI error code **(unknown)**).
- Format: PNG written by NI Vision with an NI-specific "Vision Info" block that carries the overlays, here the red ROI rectangle
  ("this vision information can only be saved with the PNG file format" [Doc6 p.978]). The pixels are the corrected image (and, for
  `Sigma`, the filtered image), without the overlay drawn in. `Image Quality = 1` has no documented effect for PNG **(inferred)**.
  `Use bit depth?` keeps its default False. Handling of 16-bit images: **(unknown)**, see
  [05-camera-acquisition.md](05-camera-acquisition.md).
- The NI "Vision Info" chunk layout is not publicly documented. A Python writer can store the ROI in a standard PNG text chunk instead.
  That is a deliberate, visible difference (ask whether any tool reads the NI metadata).

---

## 9. Python design

### 9.1 Recommended architecture

- **Keep the step functions.** Implement each LabVIEW sub-VI as a class with a `step(state, params, …) -> StepResult` method. The
  persistent shift registers become instance attributes. Hardware access goes through small injected interfaces: the Z stage
  (`qpos`, `qvel`, `vel`, `mov`, `joystick_enabled`), the camera (`grab_next`, `correct`) and the metrics (`measure`). These classes
  have no queues and no threads, so they can be unit-tested with fakes.
- **One `FocusController` per server** owns the two steppers and **at most one active job**. It writes `AF/RF State`,
  `Last Focus Data` and `Focus Progress (%)` into the per-camera focus fields read by `Cam Get Autofocus Data/Progress`.
- **Drive steps from the camera consumer loop**, which already serialises camera access, and drop the self-re-enqueue. After every
  dequeued command, and whenever the queue is empty, the loop executes exactly one focus step. Client commands still interleave with
  steps as in LabVIEW, the deadlock of §3.4 disappears, and no extra thread touches the cameras.
- **Recommended fixes** (Defects §10): let the job continue after an error, so that the `Error` restore state runs; once a job has
  recorded an error, the restore steps update only `AF/RF State` and the stage, while `Focus Progress (%)` and `Last Focus Data` keep
  the values LabVIEW leaves (those of the failed step; 0 and empty after a failed start), so that the restore never makes a failed job report 100 (D1);
  guard restores against values that were never read; validate parameters at the start command without changing the ACK by default
  (D5, D8); enforce one job at a time, by default by replacing the running job (D4); serialise GCS traffic with the Z loop through a
  shared lock.
- **Worker-thread variant** (§9.6): acceptable if the camera loop must stay responsive during long settles. It needs a per-camera lock
  around grabs and a cancellation event.

### 9.2 Reference sketch: common definitions and autofocus stepper

Reference sketch (exercised with a simulated stage and camera; error codes and texts as in LabVIEW):

```python
from __future__ import annotations

import enum
import math
import time
from dataclasses import dataclass, field
from typing import Optional, Protocol, Sequence

import numpy as np

POS_TOL_MM = 0.001            # +/- 1 um position window
AF_START_TIMEOUT_MS = 30_000  # Fast AF: time allowed to reach Z Min
AF_SETTLE_S = 0.100           # Slow AF: settle time once the position is reached
RF_POS_TIMEOUT_MS = 5_000     # Refocus: time allowed per move
RF_SCALE = 0.5                # Refocus: fixed subsampling factor


class LVError(Exception):
    """Local stand-in for errors.AloxError (09-configuration-logging-errors.md section 10.2).
    FocusError (07's FocusMetricError) and StageError are subclasses, so these except clauses catch them."""
    def __init__(self, code: int, source: str):
        super().__init__(f"{code}: {source}")
        self.code, self.source = code, source


def now_ms() -> float:
    """Replacement for LabVIEW 'Tick Count (ms)' (monotonic, no 32-bit wrap)."""
    return time.monotonic() * 1000.0


def lv_div(a: float, b: float) -> float:
    """LabVIEW floating-point division: never raises (x/0 -> +-Inf, 0/0 -> NaN)."""
    try:
        return a / b
    except ZeroDivisionError:
        return math.nan if (a == 0 or math.isnan(a)) else math.copysign(math.inf, a)


def in_range_incl(x: float, lo: float, hi: float) -> bool:
    return lo <= x <= hi


class ZStage(Protocol):            # thin wrapper over the PI GCS link (08-z-stage.md)
    def qpos(self) -> float: ...   # POS? <axis>
    def qvel(self) -> float: ...   # VEL? <axis>
    def vel(self, v: float) -> None: ...   # VEL <axis> v  (+ ERR? check)
    def mov(self, z: float) -> None: ...   # MOV <axis> z  (no ERR? check)
    def joystick_enabled(self, joy_id: int = 1) -> bool: ...   # JON? 1


class Camera(Protocol):            # 05-camera-acquisition.md
    def grab_next(self, camera_type: str) -> np.ndarray: ...            # IMAQdx Get Image2, mode Next
    def correct(self, img: np.ndarray, camera_type: str) -> np.ndarray: ...  # flip + optional flat field


class Metrics(Protocol):           # 07-focus-metrics.md
    def measure(self, img: np.ndarray, method: str, roi: Sequence[int],
                scale: float) -> tuple: ...   # -> (image, score); 'Sigma' filters the image


@dataclass
class StepResult:
    state: enum.Enum
    data: np.ndarray            # N x 2, rows [Z (mm), score]
    progress: float             # %
    finished: bool
    error: Optional[LVError]


class AFState(enum.Enum):
    START = "Start"
    GOTO_Z_POS = "Goto Z Pos"
    WAIT_STARTED = "Wait Started"
    MEASURE = "Measure"
    FINISHED = "Finished"
    ERROR = "Error"


@dataclass
class AFParams:                      # 'Camera Autofocus Data'
    camera_type: str = "High Mag."
    speed: str = "Slow"              # "Slow" | "Fast"  (anything but "Fast" behaves as Slow)
    method: str = "FFT"              # "FFT" | "Prewitt" | anything else -> Sobel
    duration_s: int = 0
    z_min: float = 0.0
    z_max: float = 0.0
    step: float = 0.0
    subsampling: float = 0.0
    roi: list = field(default_factory=list)   # [] or [Left, Top, Right, Bottom]
    acquisition_timeout_ms: int = 5000        # not used by the LabVIEW code


@dataclass
class AFSession:                     # 'Fast AF Session' (persistent between steps)
    duration_ms: float = 0.0
    t0: float = 0.0
    last_timestamp: float = 0.0
    current_z: float = 0.0
    init_speed: Optional[float] = None   # None = could not be read (LabVIEW: 0.0)
    init_z: Optional[float] = None
    af_data: list = field(default_factory=list)   # [(z, score), ...]
    move_timestamp: float = 0.0


class Autofocus:
    """alox_camera_server_autofocus.vi: one call = one state-machine step.
    self.s and self.err model the two uninitialised shift registers (persistent, one instance
    for the whole server, shared by both cameras)."""

    def __init__(self, z: ZStage, camera: Camera, metrics: Metrics, ui=None):
        self.z, self.camera, self.metrics, self.ui = z, camera, metrics, ui
        self.s = AFSession()
        self.err: Optional[LVError] = None

    def data(self) -> np.ndarray:
        if not self.s.af_data:       # LabVIEW: empty constant, 0 x 0 (inferred, section 2.6)
            return np.empty((0, 0))
        return np.asarray(self.s.af_data, dtype=float).reshape(-1, 2)

    def step(self, state: AFState, p: AFParams, normal_speed: float,
             error_in: Optional[LVError] = None) -> StepResult:
        if error_in is not None:     # outer 'Error' case: nothing runs, registers untouched
            return StepResult(state, np.empty((0, 0)), 0.0, False, error_in)
        time.sleep(0.001)            # 'Wait (ms) = 1' (in LabVIEW it runs in parallel: step >= 1 ms)
        handler = {
            AFState.START: self._start, AFState.GOTO_Z_POS: self._goto_z_pos,
            AFState.WAIT_STARTED: self._wait_started, AFState.MEASURE: self._measure,
            AFState.FINISHED: self._finished, AFState.ERROR: self._error,
        }[state]
        nxt, progress, finished = handler(p, normal_speed)
        return StepResult(nxt, self.data(), progress, finished, self.err)

    def _qpos_or_zero(self) -> float:
        try:
            return self.z.qpos()
        except LVError:
            return 0.0               # Clear Errors + Index Array on an empty array -> 0.0

    def _start(self, p: AFParams, normal_speed: float):
        if self.ui:
            self.ui.clear_focus_graph()
        self.err = None              # starts from a 'no error' constant
        init_speed = None
        try:
            init_speed = self.z.qvel()   # 'Get initial speed (will be restored at the end)'
        except LVError:
            pass                     # error cleared (LabVIEW keeps 0.0)
        try:
            self.z.vel(normal_speed) # 'Set Normal Speed' (Z Stage Settings.Speed)
        except LVError:
            pass                     # error cleared
        pos = None                   # LabVIEW: 0.0 when POS? fails
        try:
            pos = self.z.qpos()
            if not in_range_incl(pos, p.z_min - POS_TOL_MM, p.z_min + POS_TOL_MM):
                self.z.mov(p.z_min)  # 'Move to start position'
        except LVError as e:
            self.err = e
        self.s = AFSession(init_z=pos, init_speed=init_speed,
                           current_z=p.z_min, move_timestamp=now_ms())
        return (AFState.ERROR if self.err else AFState.GOTO_Z_POS), 0.0, False

    def _goto_z_pos(self, p: AFParams, normal_speed: float):
        cz = self.s.current_z
        reached = in_range_incl(self._qpos_or_zero(), cz - POS_TOL_MM, cz + POS_TOL_MM)
        if p.speed == "Fast":
            err = None               # stored error is discarded in this branch
            if reached:
                try:
                    self.z.vel(lv_div(abs(p.z_max - p.z_min), float(p.duration_s)))  # 'Set Fast Speed'
                    self.z.mov(p.z_max)                                             # 'Move to Zmax'
                except LVError as e:
                    err = e
                nxt = AFState.WAIT_STARTED
            else:
                if now_ms() - self.s.move_timestamp > AF_START_TIMEOUT_MS:
                    err = LVError(9401, "z stage get position timeout")
                nxt = AFState.GOTO_Z_POS
            self.err = err
        else:                        # "Slow", Default: no timeout, stored error passes through
            if reached:
                time.sleep(AF_SETTLE_S)   # 'Make sure the stages has stopped moving'
                nxt = AFState.MEASURE
            else:
                nxt = AFState.GOTO_Z_POS
        return (AFState.ERROR if self.err else nxt), 0.0, False

    def _wait_started(self, p: AFParams, normal_speed: float):
        if p.speed == "Fast":
            if in_range_incl(self._qpos_or_zero(), p.z_min - POS_TOL_MM, p.z_min + POS_TOL_MM):
                self.err = None      # still at Z Min: wait (no timeout)
                return AFState.WAIT_STARTED, 0.0, False
            self.s.t0 = now_ms()     # stage has left Z Min
            self.s.duration_ms = float(p.duration_s * 1000)
        return AFState.MEASURE, 0.0, False   # this state never selects 'Error'

    def _measure(self, p: AFParams, normal_speed: float):
        err, score, img = self.err, 0.0, None
        if err is None:
            try:
                img = self.camera.grab_next(p.camera_type)
            except LVError as e:
                err = e
        ti = now_ms()                # taken after the grab returns (also on error)
        if err is None:
            try:
                img = self.camera.correct(img, p.camera_type)
                method = p.method if p.method in ("FFT", "Prewitt") else "Sobel"
                img, score = self.metrics.measure(img, method, p.roi, p.subsampling)
                if self.ui:
                    self.ui.show_image(img, roi=p.roi)   # red ROI frame overlay
            except LVError as e:
                err, score = e, 0.0  # NI measurement VIs return 0 on error
        dz = p.z_max - p.z_min
        if p.speed == "Fast":
            elapsed = ti - self.s.t0
            z = p.z_min + dz * lv_div(elapsed, self.s.duration_ms)
            progress = lv_div(z - p.z_min, dz) * 100.0
            nxt = AFState.FINISHED if elapsed >= self.s.duration_ms else AFState.MEASURE
        else:
            z = self.s.current_z     # commanded position, not re-read
            z_next = z + p.step      # repeated addition (keep it: decides the last point)
            self.s.current_z = z_next
            if err is None:
                try:
                    self.z.mov(z_next)   # unconditional, also when z_next > z_max
                except LVError as e:
                    err = e
            progress = lv_div(z_next - p.z_min, dz) * 100.0
            nxt = AFState.GOTO_Z_POS if z_next <= p.z_max else AFState.FINISHED
        self.s.af_data.append((z, score))   # appended even when err is set (score 0)
        self.s.last_timestamp = ti
        if self.ui:
            self.ui.plot_focus(self.s.af_data)
        self.err = err
        return (AFState.ERROR if err else nxt), progress, False

    def _restore(self, position: bool) -> None:
        # DEVIATION: values that could not be read are not restored (LabVIEW sends 0.0).
        if self.s.init_speed is not None:
            try:
                self.z.vel(self.s.init_speed)
            except LVError:
                pass                 # error cleared
        if position and self.s.init_z is not None:
            try:
                self.z.mov(self.s.init_z)
            except LVError:
                pass                 # error cleared

    def _finished(self, p: AFParams, normal_speed: float):
        self._restore(position=False)    # 'Finished. Restore initial speed'
        return AFState.FINISHED, 100.0, True

    def _error(self, p: AFParams, normal_speed: float):
        self._restore(position=True)     # 'Restore initial speed, initial position'
        return AFState.FINISHED, 100.0, True
```

### 9.3 Reference sketch: refocus stepper

Reference sketch (continues the module of §9.2):

```python
class RFState(enum.Enum):
    START = "Start"
    GET_POSITIONS = "Get Positions"
    MOVE_POSITION = "Move Position"
    CHECK_POSITION = "Check Position"
    MESURE_POSITION = "Mesure Position"     # original spelling
    GET_FOCUS = "Get Focus"
    MOVE_FOCUS = "Move Focus"
    WAIT_FOCUS = "Wait Focus"
    FINISHED = "Finished"
    ERROR = "Error"


@dataclass
class RFParams:                       # 'Camera Refocus Data'
    camera_type: str = "High Mag."
    method: str = "FFT"               # FFT | Sobel | Prewitt | Diff | Gradient | Sigma | FULL
    refocus_method: str = "Fin"       # ignored by the LabVIEW code
    steps: int = 0
    step_mm: float = 0.0
    fin_roi_w: int = 0
    fin_roi_h: int = 0
    oxyd_x: int = 0
    oxyd_y: int = 0
    oxyd_diam: int = 0
    acquisition_timeout_ms: int = 5000  # not used
    wait_ms: int = 0
    save_images: bool = False
    folder: str = ""


@dataclass
class ZLimits:                        # subset of 'Z Stage Settings'
    speed: float
    min_pos: float
    max_pos: float


@dataclass
class RFSession:                      # 'Refocus Session' (persistent between steps)
    positions: list = field(default_factory=list)
    scores: list = field(default_factory=list)
    index_pos: int = 0
    focus_pos: float = 0.0
    focus_score: float = 0.0
    init_z: Optional[float] = None    # None = never read (LabVIEW: 0.0)
    init_speed: Optional[float] = None
    move_timestamp: float = 0.0


def lv_round(x: float) -> int:
    """LabVIEW float -> integer conversion: round half to even."""
    return int(np.rint(x))


def fin_roi(camera_type: str, width: int, height: int, fin_w: int, fin_h: int) -> list:
    """ROI [Left, Top, Right, Bottom] of the 'Fin' refocus method."""
    if camera_type == "Low Mag.":
        return [fin_w, fin_h, width, height]          # not centred (as in LabVIEW)
    hw, hh = lv_round(fin_w * 0.5), lv_round(fin_h * 0.5)
    cx, cy = lv_round(width * 0.5), lv_round(height * 0.5)
    return [cx - hw, cy - hh, cx + hw, cy + hh]


class Refocus:
    """alox_camera_server_refocus.vi: one call = one state-machine step."""

    def __init__(self, z: ZStage, camera: Camera, metrics: Metrics, image_logger=None, ui=None):
        self.z, self.camera, self.metrics = z, camera, metrics
        self.image_logger, self.ui = image_logger, ui
        self.s = RFSession()
        self.err: Optional[LVError] = None

    def data(self) -> np.ndarray:
        """Transpose(Build Array(Positions, Scores)): missing scores are padded with 0."""
        n = max(len(self.s.positions), len(self.s.scores))
        out = np.zeros((n, 2))
        out[:len(self.s.positions), 0] = self.s.positions
        out[:len(self.s.scores), 1] = self.s.scores
        return out

    def step(self, state: RFState, p: RFParams, zl: ZLimits,
             error_in: Optional[LVError] = None) -> StepResult:
        if error_in is not None:     # outer 'Error' case
            return StepResult(RFState.ERROR, np.empty((0, 0)), 0.0, True, error_in)
        time.sleep(0.001)
        handler = getattr(self, "_" + state.name.lower())
        nxt, progress, finished = handler(p, zl)
        return StepResult(nxt, self.data(), progress, finished, self.err)

    # helpers
    def _sel(self, nxt: RFState) -> RFState:
        return RFState.ERROR if self.err else nxt

    def _progress(self, idx: int) -> float:
        n = len(self.s.positions)
        # DEVIATION: LabVIEW computes 0/0 = NaN when n == 0 (Steps < 0); its clamped value is unknown.
        return 0.0 if n == 0 else min(max(idx / n * 100.0, 0.0), 100.0)

    def _target(self) -> float:
        i, pos = self.s.index_pos, self.s.positions
        return pos[i] if 0 <= i < len(pos) else 0.0     # Index Array out of range -> 0.0

    def _move_to_target(self):
        if self.err is None:
            try:
                self.z.mov(self._target())
            except LVError as e:
                self.err = e
        self.s.move_timestamp = now_ms()

    def _target_reached(self, p: RFParams) -> bool:
        if now_ms() - self.s.move_timestamp > RF_POS_TIMEOUT_MS:
            self.err = LVError(9401, "z stage get position timeout")
        try:
            pos = self.z.qpos()
        except LVError:
            pos = 0.0                                    # error cleared
        t = self._target()
        ok = (t - POS_TOL_MM) <= pos < (t + POS_TOL_MM)   # upper bound excluded
        if ok:   # 'Make sure the Z stage has stopped moving'; negative values: validate at start (D8)
            time.sleep(max(p.wait_ms, 0) / 1000.0)
        return ok

    # states
    def _start(self, p: RFParams, zl: ZLimits):
        self.err, self.s = None, RFSession()
        try:
            joystick_on = self.z.joystick_enabled(1)
        except LVError:
            joystick_on = False                          # JON? error not wired
        if joystick_on:
            self.err = LVError(9031, "'refocus' warning (joystick enabled)")
        else:
            if self.ui:
                self.ui.clear_focus_graph()
            try:
                self.s.init_z = self.z.qpos()
                self.s.init_speed = self.z.qvel()
                self.z.vel(zl.speed)                     # 'Set Normal Speed'
            except LVError as e:
                self.err = e
        return self._sel(RFState.GET_POSITIONS), 0.0, False

    def _get_positions(self, p: RFParams, zl: ZLimits):
        z_init = self.s.init_z if self.s.init_z is not None else 0.0
        z_start = z_init - float(p.steps) * p.step_mm
        self.s.positions = [min(max(z_start + float(i) * p.step_mm, zl.min_pos), zl.max_pos)
                            for i in range(2 * p.steps + 1)]
        self.s.scores, self.s.index_pos = [], 0
        return RFState.MOVE_POSITION, 0.0, False         # unconditional

    def _move_position(self, p: RFParams, zl: ZLimits):
        self._move_to_target()
        return self._sel(RFState.CHECK_POSITION), self._progress(self.s.index_pos), False

    def _check_position(self, p: RFParams, zl: ZLimits):
        nxt = RFState.MESURE_POSITION if self._target_reached(p) else RFState.CHECK_POSITION
        return self._sel(nxt), self._progress(self.s.index_pos), False

    def _mesure_position(self, p: RFParams, zl: ZLimits):
        idx, err, score, img, roi = self.s.index_pos, self.err, 0.0, None, None
        if err is None:
            try:
                img = self.camera.correct(self.camera.grab_next(p.camera_type), p.camera_type)
                h, w = img.shape[:2]
                roi = fin_roi(p.camera_type, w, h, p.fin_roi_w, p.fin_roi_h)
                img, score = self.metrics.measure(img, p.method, roi, RF_SCALE)
                if p.save_images and self.image_logger:
                    self.image_logger.save(img, p.folder, idx, roi)
            except LVError as e:
                err = e
        self.s.scores.append(score)
        self.s.index_pos = idx + 1
        if self.ui:
            self.ui.plot_focus(list(zip(self.s.positions, self.s.scores)))
            if img is not None:
                self.ui.show_image(img, roi=roi)
        # DEVIATION (D10): LabVIEW overwrites err.source here with str(U32(t0 - now)) on every measurement.
        self.err = err
        nxt = RFState.GET_FOCUS if self.s.index_pos >= len(self.s.positions) else RFState.MOVE_POSITION
        return self._sel(nxt), self._progress(self.s.index_pos), False

    def _get_focus(self, p: RFParams, zl: ZLimits):
        best_pos, best_score, best_idx = 0.0, -math.inf, 0
        for i, (z, s) in enumerate(zip(self.s.positions, self.s.scores)):
            if s > best_score:                           # strict: first maximum wins
                best_pos, best_score, best_idx = z, s, i
        self.s.focus_pos, self.s.focus_score, self.s.index_pos = best_pos, best_score, best_idx
        return self._sel(RFState.MOVE_FOCUS), 100.0, False

    def _move_focus(self, p: RFParams, zl: ZLimits):
        self._move_to_target()
        return self._sel(RFState.WAIT_FOCUS), 100.0, False

    def _wait_focus(self, p: RFParams, zl: ZLimits):
        nxt = RFState.FINISHED if self._target_reached(p) else RFState.WAIT_FOCUS
        return self._sel(nxt), 100.0, False

    def _finished(self, p: RFParams, zl: ZLimits):
        if self.s.init_speed is not None:
            try:
                self.z.vel(self.s.init_speed)
            except LVError:
                pass
        return RFState.FINISHED, 100.0, True

    def _error(self, p: RFParams, zl: ZLimits):
        # DEVIATION: LabVIEW would send VEL 0 / MOV 0.0 when the values were never read.
        if self.s.init_speed is not None:
            try:
                self.z.vel(self.s.init_speed)
            except LVError:
                pass
        if self.s.init_z is not None:
            try:
                self.z.mov(self.s.init_z)
            except LVError:
                pass
        return RFState.ERROR, 100.0, True
```

Notes on the sketches:

- They reproduce the LabVIEW decisions, including the points listed in §10 as "reproduce". The deviations are marked `DEVIATION`.
- The `Default` case is not modelled: a Python `Enum` cannot hold an unknown state.
- The metric object implements the method tables of §4.5 and §5.7. It returns the possibly filtered image (`Sigma`) together with the
  score.
- Parameter validation (§10, D8) belongs in the start command, before `step()` is called.

### 9.4 Reference sketch: image logger

Reference sketch (Pillow; `ERR_IMAGE_WRITE` is a placeholder because the NI error code is unknown):

```python
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
from PIL import Image, PngImagePlugin

ERR_IMAGE_WRITE = -1   # placeholder: choose the code with 09-configuration-logging-errors.md


def refocus_image_name(index: int, now: Optional[datetime] = None, pad: str = " ") -> str:
    """'<yymmdd>_<HHMMSS>_refocus__mesa<index, width 2>.png' (SEMI job empty)."""
    ts = (now or datetime.now()).strftime("%y%m%d_%H%M%S_refocus_")
    ts = ts.replace(",", "", 1).replace(".", "", 1)     # no effect with this format
    return f"{ts}_mesa{str(index).rjust(2, pad)}.png"


class RefocusImageLogger:
    def __init__(self, pad: str = " "):
        self.pad = pad                       # confirm space vs '0' on a LabVIEW machine

    def save(self, img: np.ndarray, folder: str, index: int, roi) -> Path:
        path = Path(folder) / refocus_image_name(index, pad=self.pad)
        if not folder or not path.parent.is_dir():           # LabVIEW does not create folders
            raise LVError(ERR_IMAGE_WRITE, f"refocus image: invalid folder {folder!r}")
        meta = PngImagePlugin.PngInfo()
        if roi is not None:                                   # replaces the NI 'Vision Info' overlay
            meta.add_text("ALOX_ROI", ",".join(str(int(v)) for v in roi))
        try:
            Image.fromarray(img).save(path, format="PNG", pnginfo=meta)   # uint8 -> 'L', uint16 -> 'I;16'
        except OSError as e:
            raise LVError(ERR_IMAGE_WRITE, f"refocus image: {e}") from e
        return path
```

`refocus_image_name(3, datetime(2023, 10, 8, 14, 32, 10))` returns `"231008_143210_refocus__mesa 3.png"`.

### 9.5 Reference sketch: controller and camera-loop driver

Reference sketch (continues the module; the queue is the camera command queue of [01-architecture.md](01-architecture.md)):

```python
import queue
import threading


@dataclass
class FocusFields:                    # focus fields of the camera session (one per camera)
    af_state: AFState = AFState.START
    rf_state: RFState = RFState.START
    last_focus_data: np.ndarray = field(default_factory=lambda: np.empty((0, 0)))
    focus_progress: float = 0.0
    last_error: Optional[LVError] = None   # NEW (not in LabVIEW): kept for logs / extensions


@dataclass
class FocusJob:
    kind: str                         # "AF" | "RF"
    params: object                    # AFParams | RFParams
    failed: bool = False              # an error was recorded: progress and data stay frozen (D1)


class FocusController:
    """Holds the two steppers (one instance each for the whole server) and at most one job."""

    def __init__(self, af: Autofocus, rf: Refocus, fields: dict, z_settings):
        self.af, self.rf, self.fields = af, rf, fields   # fields: {"High Mag.": FocusFields, "Low Mag.": ...}
        self.z_settings = z_settings      # callable -> object with speed, min_pos, max_pos
        self.job: Optional[FocusJob] = None
        self.lock = threading.RLock()     # protects fields/job if another thread reads them

    def _fields(self, camera_type: str) -> FocusFields:
        return self.fields["High Mag." if camera_type == "High Mag." else "Low Mag."]

    def _run(self, kind: str, state, params, error_in=None) -> StepResult:
        zs = self.z_settings()
        if kind == "AF":
            return self.af.step(state, params, zs.speed, error_in)
        return self.rf.step(state, params, ZLimits(zs.speed, zs.min_pos, zs.max_pos), error_in)

    def _store(self, kind: str, params, r: StepResult, failed: bool = False) -> None:
        f = self._fields(params.camera_type)
        if kind == "AF":
            f.af_state = r.state
        else:
            f.rf_state = r.state
        if not failed:
            # After a recorded error the restore steps (progress 100) must not reach the client:
            # it keeps the values LabVIEW leaves (failed step's values; 0 and empty after a failed start).
            f.last_focus_data, f.focus_progress = r.data, r.progress
        if r.error is not None:
            f.last_error = r.error

    def start(self, kind: str, params, error_in=None) -> Optional[LVError]:
        """'Cam Autofocus' / 'Cam Refocus': first step done synchronously, result = ACK error.
        The caller applies the D4 policy first (default: abort() the running job and replace it;
        reject only with focus_reject_concurrent_start = TRUE) and the D5/D8 checks."""
        with self.lock:
            r = self._run(kind, AFState.START if kind == "AF" else RFState.START, params, error_in)
            if r.error is None:
                self._store(kind, params, r)
                self.job = FocusJob(kind, params)
            else:
                f = self._fields(params.camera_type)   # as LabVIEW: state Error, progress 0, no data
                if kind == "AF":
                    f.af_state = AFState.ERROR
                else:
                    f.rf_state = RFState.ERROR
                f.last_focus_data, f.focus_progress, f.last_error = np.empty((0, 0)), 0.0, r.error
                # DEVIATION: LabVIEW never runs the restore state after a failed first step.
                # failed=True: the restore steps keep progress 0 and the empty data.
                self.job = FocusJob(kind, params, failed=True) if error_in is None else None
            return r.error

    def step(self) -> None:
        """'Cam Autofocus Run' / 'Cam Refocus Run': exactly one step of the active job."""
        with self.lock:
            job = self.job
            if job is None:
                return
            f = self._fields(job.params.camera_type)
            state = f.af_state if job.kind == "AF" else f.rf_state
            r = self._run(job.kind, state, job.params)
            self._store(job.kind, job.params, r, job.failed)   # the failed step itself is stored, as in LabVIEW
            # DEVIATION: LabVIEW stops re-enqueueing as soon as r.error is set, so the
            # 'Error' (restore) state never runs. Here the job continues until finished,
            # but its later steps only update the state and the stage (job.failed).
            if r.error is not None:
                job.failed = True
            if r.finished:
                self.job = None

    def abort(self) -> None:
        """Cancellation (server exit, new job replacing an old one): run the restore state once.
        The aborted job's progress and data are left as they are."""
        with self.lock:
            job = self.job
            if job is None:
                return
            job.failed = True
            f = self._fields(job.params.camera_type)
            if job.kind == "AF":
                f.af_state = AFState.ERROR
            else:
                f.rf_state = RFState.ERROR
            self.step()
            self.job = None


def camera_consumer_loop(cam_q: "queue.Queue", dispatch, focus: FocusController) -> None:
    """One focus step after every command, and continuous stepping while the queue is empty."""
    while True:
        try:
            msg = cam_q.get(block=focus.job is None)
        except queue.Empty:
            msg = None
        if msg is not None and dispatch(msg):     # dispatch returns True on 'Exit'
            focus.abort()
            break
        if focus.job is not None:
            focus.step()
```

Command handlers on top of the controller (pseudo-code, response helpers from [02-tcp-protocol.md](02-tcp-protocol.md), logging from
[09-configuration-logging-errors.md](09-configuration-logging-errors.md)):

```
Cam Autofocus / Cam Refocus:
    params, err = decode(Command Data) ; run = True
    if no err and focus.job is set:                          # D4
        if compat.focus_reject_concurrent_start:             # default FALSE
            err = 95xx error ; run = False                   # the running job and its fields are left untouched
        else: focus.abort()                                  # default: replace the running job, ACK as LabVIEW
    if no err and AF and compat.af_check_z_limits and [Z Min, Z Max] not inside [Min, Max Position]:
        err = {9405, "Z move out of range"}                  # D5 (default FALSE: no check, as LabVIEW)
    if no err and params invalid (D8 list):
        if compat.focus_validate_params: err = 95xx error    # default FALSE
        else:                                                # success ACK as LabVIEW, but no motion
            log warning ; fields[cam] := {state Finished, progress 0, data empty} ; run = False
    if run: err = focus.start("AF" | "RF", params, err)      # err set above: state Error, progress 0, no data, no job
    err = append_source(err, "Cam Autofocus" | "Cam Refocus") ; log(extra = "Autofocus" | "Refocus", err)
    respond_error_cluster(err)
Cam Get Autofocus Data:     respond_dbl_2d(fields[cam].last_focus_data, error = decode error)
Cam Get Autofocus Progress: respond_dbl(fields[cam].focus_progress, error = decode error)
(no Cam Autofocus Run / Cam Refocus Run handlers: stepping is done by the loop; ignore them if they ever arrive)
```

### 9.6 Worker-thread variant with cancellation

If long settles, for example a large `Wait (ms)`, must not block the camera loop, a single `FocusWorker` thread can call
`controller.step()` in a loop instead. Requirements:

- Hold a **per-camera lock** around `grab_next` + `correct` (shared with `Cam Get Image`, `Cam Stream Next Image`, …) and the
  **GCS lock** around each Z request (shared with the Z loop).
- Use a `threading.Event` for cancellation. On cancellation (server `Exit`, new job replacing the old one, explicit abort) call
  `controller.abort()`, which runs the restore state once, in a `finally` block.
- Replace `time.sleep` in the steppers with `cancel_event.wait(timeout)` so cancellation is prompt.
- Readers (`Cam Get Autofocus Data/Progress`) take `controller.lock` and get copies of the arrays.
- Add an overall job timeout. LabVIEW has none, and AF Slow / AF Wait Started can wait forever.

### 9.7 Test plan

| Test | Set-up (fake Z stage moving at a set speed, fake camera whose score peaks at Z = z0) | Expected |
|---|---|---|
| AF Slow point set | `Z Min = 1.0`, `Step = 0.1`, `Z Max = 1.5` | rows at 1.0 … 1.4000000000000004 (5 rows); last `MOV` = 1.5000000000000004; progress pattern 0 / value / 0 …; last `Measure` 100.00000000000009, then 100 |
| AF Fast | `Duration = 1 s`, range 0.5 mm | `VEL` = 0.5 mm/s; rows with increasing interpolated Z; ends after ≥ 1 s; stage at `Z Max`; speed restored |
| AF Fast timeout | stage never reaches `Z Min` | error 9401 after 30 s |
| AF Fast duration 0 | `Duration = 0` | LabVIEW: `VEL` rejected → `Error`, job stops. Python (D8): success ACK, no motion, progress 0, warning logged (`focus_validate_params` = FALSE); 95xx error (TRUE) |
| AF failure mid-scan | grab fails at the 3rd point | restore runs (initial speed, `MOV Init Z`); progress and data stay at the failed step's values, never 100 (D1) |
| RF nominal | `Steps = 3`, `Step = 0.01`, start 1.25 | 7 positions 1.22 … 1.28; progress 100 from the 7th `Mesure Position` on; best = grid point closest to z0; stage left there; speed restored |
| RF clamping | start near `Max Position` | duplicated clamped positions |
| RF joystick | `JON? 1` = 1 | 9031, state `Error`, no `MOV`/`VEL` from the restore (guard); progress stays 0, data empty (D1) |
| RF per-move timeout | stage stalls | 9401 after 5 s |
| RF ROI | High Mag 2448×2048, `FinW = 101`, `FinH = 103` | `[1174, 972, 1274, 1076]`; Low Mag `[FinW, FinH, W, H]` |
| Data padding | mid-scan read | rows `[Z_i, 0]` for unmeasured positions |
| Image names | index 3 at 2023-10-08 14:32:10 | `231008_143210_refocus__mesa 3.png` |
| Interleaving | progress polls during a job | answered between steps; no deadlock with a full queue |

---

## 10. Defects and oddities in the LabVIEW implementation

| # | Observation | Evidence | Impact | Recommendation |
|---|---|---|---|---|
| D1 | After an error in a background step the job stops without running the `Error` state. Speed and position are not restored, the progress freezes below 100 and the state stays `Error`. | [main VI frames d205, d210] + [Doc6 p.609, p.1033] | stage left at the fast-scan speed or at an arbitrary Z; a client waiting for 100 % waits forever | **Fix**: keep stepping until `finished` (§9.5), so that the restore runs. The `Error` state writes `Progress = 100` (§2.7, §4.5, §5.5), so a client polling for 100 would take the failed job for a success. Rule: once a job has recorded an error (in a background step, or at the start, where `start()` keeps the job as a `DEVIATION`), the restore steps update only `AF/RF State` and the stage. `Focus Progress (%)` and `Last Focus Data` keep the values LabVIEW leaves (those of the failed step; 0 and empty after a failed start), and the client times out as today (`job.failed` in §9.5). Ask whether moving back to `Init Z` after a failure is wanted (questionnaire Q39). |
| D2 | The client cannot see the state or the error of a running job. Progress returns to 0 between Slow-AF points, exceeds 100 on the last autofocus point (both modes), and reaches 100 in refocus at the last measurement, before the move to best focus. | §2.7 | completion and failure detection are unreliable | **Reproduce** the progress values (protocol compatibility); **ask** about adding a status command (the new `last_error` field can feed it; questionnaire Q39). |
| D3 | Self-re-enqueue with a blocking `Enqueue` into the loop's own 5-slot queue. | [main VI frames d181, d205] | possible deadlock of the camera loop | **Fix**: step from the loop (§9.5). Questionnaire Q28. |
| D4 | Nothing prevents a second focus job: AF+AF (double stepping), AF on both cameras (shared internal memory), AF+RF (two jobs moving one stage). | §3.4 | corrupted data, erratic stage motion | **Fix**: one job at a time. By default **replace**: abort the old job (its restore state runs once, its progress and data stay as they are), then run the new start and ACK it as LabVIEW does. Rejecting the new start with an error (a 95xx code, [09-configuration-logging-errors.md](09-configuration-logging-errors.md) §10.5) would turn a success ACK into an error ACK, which is silence while `send_error_replies` = FALSE. It is therefore the compatibility switch `focus_reject_concurrent_start` (default FALSE; [10-migration-plan.md](10-migration-plan.md) §6; questionnaire Q39). |
| D5 | Autofocus does not check `[Z Min, Z Max]` against `Min/Max Position` (comment "TODO"), and the Slow scan ends one step beyond the last point. | [Doc6 p.606, p.605] | the stage can leave the software limits | **Switch**: rejecting the start changes its ACK, so the check is the compatibility switch `af_check_z_limits` (default FALSE = LabVIEW, no check; TRUE = reject the start with 9405 "Z move out of range" as in [08-z-stage.md](08-z-stage.md); [10-migration-plan.md](10-migration-plan.md) §6; questionnaire Q39). **Ask** whether the final over-travel `MOV` must be kept (questionnaire Q22). |
| D6 | No timeout in AF Slow `Goto Z Pos` or AF Fast `Wait Started`. A refused `MOV` is not detected (no `ERR?`). | [Doc6 p.608], [Doc6 p.878] | infinite job | **Fix**: add timeouts (for example 30 s, as in Fast `Goto Z Pos`) with error 9401 (questionnaire Q22). |
| D7 | `Acquisition Timeout (ms)` is ignored by both commands, and `Refocus Method` is ignored (constant `Fin`). | [Doc6 p.600, p.613, p.1035] | client parameters have no effect | **Reproduce**: accept and ignore. Document it (questionnaire Q19 for `Oxyd`). |
| D8 | No parameter validation: `Step ≤ 0` (Slow) loops forever; `Duration = 0` → `VEL` Inf/NaN; `Subsampling = 0` → metric failure; `Steps < 0` → no positions, then `MOV 0.0 mm` (out-of-range index default) and progress 0/0; `Step ≤ 1 µm` → no waiting for moves; Fast with `Z Max = Z Min` → `VEL 0` and possibly an endless `Wait Started` (§7.3). | §4.5, §5.5, §6.2, §7.3 | hangs, stage sent to 0 mm | **Fix**: validate at the start command. Invalid: `Step ≤ 0` (Slow AF), `Duration = 0` (Fast AF), `Subsampling = 0` (AF), `Steps < 0` (refocus). With the compatibility switch `focus_validate_params` = FALSE (default), reply exactly as LabVIEW (success ACK) but never move the stage: end the job without motion, leave the progress at 0 (data empty) and log a warning. The client sees a job that never completes, as it does today in most of these cases. TRUE replies with an error (a 95xx code, [09-configuration-logging-errors.md](09-configuration-logging-errors.md) §10.5) ([10-migration-plan.md](10-migration-plan.md) §6; questionnaire Q39). |
| D9 | `Init Speed` = 0 when `VEL?` fails (AF), and the end sends `VEL 0`. | [Doc6 p.607] | stage left at a bad speed | **Fix**: skip the restore (sketch). |
| D10 | Refocus overwrites `error.source` with the decimal text of `U32(t0 − now)` at **every** measurement, even without an error. The operands are reversed (values near 2³²) and the second `Tick Count` is not sequenced (§5.5). | [Doc6 p.1035] | the real error source (IMAQdx, PNG write) is lost; debugging leftover | **Fix**: do not reproduce; log the measurement duration separately. Ask only if a client parses `source`. |
| D11 | Joystick active → `Error` with an empty session. If executed, the `Error` state would send `VEL 0` and `MOV 0.0 mm`. Latent today because of D1. | [Doc6 p.1034, p.1033] | dangerous move once D1 is fixed | **Fix**: restore only values that were actually read (sketch). Questionnaire Q20. |
| D12 | Low Mag "Fin" ROI is `[FinW, FinH, W, H]`, not centred, unlike High Mag. | [Doc6 p.1035] | different ROI semantics per camera | **Ask** (intended margins or bug, questionnaire Q21). Reproduce until answered. |
| D13 | A failed `POS?` reads as 0.0 mm. It can satisfy the "reached" test for a target within 1 µm of 0, and in Fast AF `Wait Started` it looks like "left `Z Min`" (unless `Z Min` ≈ 0), which starts the timed measurement early. | §1.3, [Doc6 p.607, p.1035] | false "reached" | **Fix**: treat a failed `POS?` as "not reached" (timeouts still apply). |
| D14 | The camera simulation mode is ignored (constant `False` selector). | [Doc6 p.605, p.1035] | cannot run a focus without hardware | **Fix**: use the simulator when the session is simulated (useful for tests). Ask if the LabVIEW behaviour must be kept (questionnaire Q40). |
| D15 | Measurement errors still append a row with score 0 (AF and RF), and the RF index still advances. | §4.5, §5.5 | zero rows in the data after a failure | **Reproduce** (visible in `Last Focus Data`). |
| D16 | Fast-AF Z is estimated from a software timestamp taken after `Get Image2` returns. `T0` is taken when the stage is first seen > 1 µm from `Z Min`. "Next" may return frames exposed earlier. Acceleration is ignored. | [Doc6 p.606, p.608] | systematic Z bias of the Fast curve | **Reproduce** the formula. **Ask** whether hardware timestamps (GenICam chunk data) or synchronous `POS?` reads are acceptable improvements. |
| D17 | Image name: double underscore, space-padded index (inferred), 1 s resolution, `Timestamp` input ignored, folder not created, NI-specific metadata. | §8 | file-name compatibility | **Reproduce** the name (after confirming the padding). **Ask** about the NI Vision Info metadata (questionnaire Q26, Q38). |
| D18 | Sigma method: the low-pass filter modifies the camera image in place (displayed and saved filtered) with an even 20×20 kernel and no error wiring. FULL: Sobel and FFT run in parallel on shared NI work images, and the FFT uses the whole image. | [Doc6 p.1036], [07-focus-metrics.md](07-focus-metrics.md) | non-deterministic FULL scores; filtered saved images | **Fix** the race (sequential, separate buffers). Keep the formulas as specified in 07. Ask whether saved Sigma images should be unfiltered (questionnaire Q27). |
| D19 | Every `Cam Autofocus Run` step is written to `Commands History` (500 entries). Every step also grows the unbounded error and temperature histories of the camera loop. | [main VI frame d205], [01-architecture.md](01-architecture.md) | history flooded, memory growth | **Fix**: do not record internal steps. |
| D20 | Focus GCS traffic runs in parallel with the Z loop without a lock. The `Z Session` comes from a front-panel local variable that can be stale or default. | §7.4 | interleaved serial exchanges, wrong axis/system | **Fix**: one shared, locked Z link object; use its live session. |
| D21 | Tick Count arithmetic: the timeouts subtract in U32, which wraps correctly (for refocus, `Move timestamp` is stored in an integer field, I32 **(inferred)**, and coerced back to U32, assumed lossless **(inferred)**), but Fast-AF `ti − T0` is converted to DBL first, which breaks across the 49.7-day wrap. | [Doc6 p.606, p.608, p.1035] | rare wrong Zest | **Fix**: monotonic clock. |
| D22 | Disabled code: `MAC START SLOWACC/FASTACC` in refocus; unwired `Focus Graph 2`; unused `Last Timestamp`, `Focus Pos`, `Focus Score`; dead `Oxyd` branch. | [Doc6 p.1032–1037, p.605] | none | **Do not implement** (keep `focus_pos`/`focus_score` for a future status command). |
| D23 | Refocus `Check/Wait` still sleeps `Wait (ms)` when the 5 s timeout fires in the same call. | [Doc6 p.1035] | small extra delay before `Error` | Reproduce or not; no functional impact. |

---

## 11. Open questions

"Questionnaire Qn" in this chapter refers to [open-questions.md](open-questions.md).

1. **Wire layout of the start commands.** What are the numeric values of `Autofocus Speed`, `Autofocus Method` (7 values) and
   `Refocus Method`, and the internal field order of `Camera Autofocus Data` / `Camera Refocus Data` (integer widths of `Duration (s)`,
   `ROI`, `Steps`, `Wait (ms)`)? Without them the TCP decoder cannot be written. Needs the `.ctl` files or a network capture (see
   [04-data-types.md](04-data-types.md); questionnaire Q2, Q33, and Q7 of [03-command-reference.md](03-command-reference.md)).
2. **Client completion logic.** How does the existing client decide that a focus run is over or has failed: `progress == 100`,
   `>= 100`, a timeout? Does it rely on the oscillating Slow-AF progress or on refocus reporting 100 early (D2)? Is a new status/abort
   command acceptable? Where is the best Z chosen after an autofocus (questionnaire Q22, Q39, Q32; Q31 asks for the client code)?
3. **Failure policy.** After a failure in a background step, should the server restore speed and **move back to `Init Z`** (the
   intent of the `Error` state, D1)? Or should it only stop, as the LabVIEW program effectively does (questionnaire Q39)?
4. **ACK of a failed start.** Does the client receive an error response when `Cam Autofocus` / `Cam Refocus` fails, or nothing (see
   [02-tcp-protocol.md](02-tcp-protocol.md))? This decides what the re-implementation must send (questionnaire Q17).
5. **Concurrent starts.** For a start command received while a job runs: reject, replace, or (LabVIEW) silently mix (D4,
   questionnaire Q39)?
6. **Low Mag refocus ROI.** Is `[FinW, FinH, W, H]` intended (D12, questionnaire Q21)?
7. **Slow-AF end position.** Does any client rely on the stage ending one step beyond the last point (D5, questionnaire Q22)?
8. **`POS?` semantics on the Mercury controller.** Actual or target position? If it is the target position, the settle logic is
   meaningless and must be redesigned (§6.2, questionnaire Q35).
9. **Refocus image files.** Index padding (space or zero), whether downstream tools parse the names, and whether the NI "Vision Info"
   PNG metadata must be preserved (D17, questionnaire Q26, Q38).
10. **`Oxyd` refocus.** Is it needed? Its sub-VIs ("OXYD ROI", "FFT OXYD", "SOBEL OXYD") are not in the printouts, so it cannot be
    specified from the evidence (questionnaire Q19).
11. **Simulation.** Should autofocus/refocus work with a simulated camera in the new server (D14, questionnaire Q40)?
