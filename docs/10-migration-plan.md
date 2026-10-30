# Migration plan

## Summary

This chapter turns the analysis (chapters 01–09) into a plan for rewriting the LabVIEW **Alox Camera Server** in another language.

- **Recommendation:** rewrite it in **Python 3.11+**. Use NumPy/OpenCV for images, PI's official `pipython` for the Z stage, a
  GenICam stack or the camera vendor SDK for the cameras, and the standard library for TCP, queues and INI files.
- **Design:** keep the current external contract, meaning the TCP wire format and the command semantics, so the existing
  ALOX Vision Core client keeps working unchanged.
- **Approach:** proceed by phases, validating each one against the LabVIEW server: protocol first, then Z stage, cameras, focus
  algorithms, and finally deployment.

The main risks are not in the algorithms, which are small and now fully specified. They are in a few **facts the printouts
cannot show**: the enum values on the wire, NI's proprietary image serialisation, and the exact camera hardware. These are
listed in the questionnaire ([`open-questions.md`](open-questions.md)) and must be answered before the protocol layer is
frozen.

## 1. What is being migrated

| Item | Size / facts | Chapter |
|---|---|---|
| Main VI `alox_camera_server.vi` | Outer 5-stage sequence; 3 named queues of max. 5 elements; 3 parallel loops (TCP producer, cameras consumer, Z consumer); diagram exported as 254 frames | [01](01-architecture.md) |
| TCP protocol | Big-endian frames `[I32 length][U16 datatype][payload]`; payloads are LabVIEW-flattened clusters, including a flattened **variant**; responses are correlated by a "notifier name"; listens on `port_I32` (default 503) | [02](02-tcp-protocol.md) |
| Command API | 46 known command names: 27 handled by the camera loop (including `Exit` and the 2 internal AF/RF "Run" steps) and 19 by the Z loop. The enum may contain more items. Commands are routed to the camera or Z loop **by enum range**. | [03](03-command-reference.md) |
| Data types | About 45 ALOX typedefs (command payloads, sessions, settings, enums) | [04](04-data-types.md) |
| Cameras | 2 cameras (High Mag / Low Mag) through NI-IMAQdx (GigE or USB plug types; a FireWire bus type is shown on the panel). Includes flip, flat-field/dark-current correction, custom image metadata, simulation from a PNG folder, and LED/bulb through camera output lines. | [05](05-camera-acquisition.md) |
| Autofocus / refocus | Two step-wise state machines driven by self-re-enqueued commands. They drive the PI stage directly and return a `[Z, score]` table. | [06](06-autofocus-refocus.md) |
| Focus metrics | FFT magnitude sum and edge-detection histogram mean (Sobel, Prewitt, Roberts, Sigma, Differentiation, Gradient, FULL) on a resampled region of interest | [07](07-focus-metrics.md) |
| Z stage | PI Mercury controller (GCS 2.0, probably C-863) over RS-232 through PI's LabVIEW driver; MOV/MVR/VEL/POS?/FRF/FNL/STP/JON/JAX, macros `DH`/`HOME` | [08](08-z-stage.md) |
| Configuration and logs | `alox_setup.ini` (next to the exe) → `[folders] settings` → `alox.ini` (about 53 keys, read once at start-up, never written); daily event logs | [09](09-configuration-logging-errors.md) |

The rest of the printouts (about 190 of the 273 printed items) are **vendor libraries**: NI `vi.lib` (config, error, TCP,
VISA), NI Vision / IMAQdx, OpenG and the PI GCS LabVIEW driver. They are not translated: they are **replaced** by
equivalent libraries (section 4).

## 2. Choice of language

### 2.1 Criteria

| Criterion | Weight | Why it matters here |
|---|---|---|
| Hardware SDK support (cameras, PI controller) | High | The server is mostly glue around two hardware families. |
| Image processing and numerics | High | The focus metrics, flat-field and resampling must match NI Vision closely. |
| Bit-exact control over binary serialisation | High | The client expects LabVIEW-flattened data. |
| Concurrency model | Medium | There are 3 independent device loops and one socket. Throughput is modest: one client, images on request or streamed. |
| Speed of development, readability, test tooling | Medium | The goal is to replace LabVIEW with maintainable code. |
| Deployment on the lab PC (Windows, possibly as a service or hidden exe) | Medium | The LabVIEW exe runs with an invisible window. |

### 2.2 Options

| Option | Strengths | Weaknesses | Verdict |
|---|---|---|---|
| **Python 3.11+** | Official `pipython` from PI. Python SDKs from all major camera vendors (pypylon, VmbPy, PySpin) and generic GenICam (Harvesters, Aravis). NumPy/SciPy/OpenCV make the NI Vision functions one-liners, which makes numerical validation easy. `struct` covers LabVIEW flattening. Fast to write and test. | Python is slower than compiled code. This is irrelevant here: the heavy work runs in NumPy/OpenCV/SDK code that releases the GIL. A single exe needs PyInstaller. | **Recommended** |
| C# / .NET 8 | Strong typing, good Windows-service and single-exe story, vendor .NET SDKs (pylon .NET, Vimba .NET), OpenCvSharp, PI GCS2 DLL usable through P/Invoke | No official PI .NET wrapper, so P/Invoke would be needed. Numerical prototyping and validation against NI are heavier. | Good alternative if the team is .NET-centric |
| C++ | Maximum performance, all SDKs are native | Much more code and a higher maintenance cost for no measurable benefit at this throughput | Not recommended |
| Stay on LabVIEW (refactor) | No migration risk | Doesn't meet the goal (licences, maintainability, tooling) | Out of scope |

**Decision (pending Q14 in the questionnaire):** Python. If the answer to Q14 imposes .NET, the architecture below carries over
one-to-one: threads with queues, a codec module and device adapters.

## 3. Target architecture

### 3.1 Principles

1. **Keep the external contract unchanged by default.** That means the same port, frames, enum values, payload layouts, response
   types, notifier-name correlation and error codes. This lets the existing client work without modification and allows a
   side-by-side comparison with the LabVIEW server.
2. **Keep the concurrency shape:** one thread per device family consuming its own queue, plus one TCP thread. It serialises
   access to each device exactly like LabVIEW does and keeps the behaviour easy to compare.
3. **Isolate the hardware** behind small interfaces (`CameraBackend`, `StageBackend`) with **simulated implementations**. The
   whole server can then be tested on any PC; the LabVIEW code already has a camera simulation mode.
4. **Make each LabVIEW quirk a named, configurable behaviour** when the right choice is not obvious (section 6). Then "bug-for-bug
   compatible" and "fixed" are switches, not rewrites.
5. **Replace front-panel indicators used as shared memory** with an explicit `ServerState` object protected by a lock. These
   indicators include `HI Inited`, `Z Session`, the LED status, the temperature histories and the command history.

### 3.2 Package layout (proposed)

```text
aloxcam/
  __main__.py            entry point: resolve folders, load config, start the server, handle shutdown
  config.py              NI-compatible INI reader (alox_setup.ini, alox.ini) → typed settings     [09]
  lvflatten.py           LabVIEW flatten/unflatten (big-endian), variant type descriptors          [02][04]
  messages.py            dataclasses for every typedef + enums (values from the .ctl files)        [04]
  protocol.py            framing, datatypes, request decoding, response encoding, heartbeat        [02]
  server.py              TCP state machine (Listen/Read/Send Response), routing by command range   [01][02]
  state.py               ServerState (replaces front-panel indicators/locals), histories            [01]
  errors.py              ErrorCluster, AloxError hierarchy, Code (error codes)                       [09]
  logs.py                event logs (file names, folders, format)                                   [09]
  camera/
    backend.py           CameraBackend interface (open/configure/start/stop/get_image/attributes/outputs)
    genicam.py           implementation on Harvesters or the vendor SDK                               [05]
    simulation.py        PNG-folder simulation backend                                               [05]
    pipeline.py          flip, flat-field/dark-current, type conversion, image metadata              [05]
    handlers.py          one function per camera command                                             [03]
  focus/
    metrics.py           FFT and edge metrics, NI-compatible resampling/ROI                          [07]
    autofocus.py         step-wise autofocus state machine                                           [06]
    refocus.py           step-wise refocus state machine + image logging                             [06]
  zstage/
    backend.py           StageBackend interface + simulated stage
    pi_stage.py          pipython implementation (RS-232), macros, reference check                  [08]
    handlers.py          one function per Z command                                                  [03]
tests/                   unit tests + golden frames from network captures + NI reference images
tools/
  replay_client.py       sends recorded/scripted commands to any server (LabVIEW or Python) and diffs replies
```

The reference sketches in the other chapters use these module names: `lvflatten.py` + `protocol.py` = the codec sketched as
`lvwire.py` in [02](02-tcp-protocol.md) §11.1; `messages.py` = `lvtypes.py` in [04](04-data-types.md) §9.2; `zstage/pi_stage.py`
and `zstage/handlers.py` as in [08](08-z-stage.md) §13. The names in `errors.py` are those of
[09](09-configuration-logging-errors.md) §10.2.

### 3.3 Runtime view

```text
            ┌────────────── server thread (TCP) ───────────────┐
 client ⇄ socket ⇄ framing ⇄ decode Camera Queue Data ─ route ─┬─▶ cam_q ─▶ camera thread ─▶ CameraBackend ×2
            │      ▲                                           │              │  (AF/RF steps re-enqueued)
            │      └──── tcp_q ◀── responses (notifier name) ──┼──────────────┘
            │                                                  └─▶ z_q ───▶ Z thread ─────▶ StageBackend (pipython)
            └──────────────────────────────────────────────────┘                   ▲
                                                         AF/RF call StageBackend directly (as in LabVIEW)
```

- **Queues:** use `queue.Queue` objects. Commands re-enqueued internally go to an **unbounded** internal queue or are handled by a
  step loop. This removes the LabVIEW deadlocks caused by self-enqueueing into queues limited to 5 elements (see [01](01-architecture.md)).
- **Shutdown:** `Exit` is routed to the camera loop, which forwards it to the Z loop. The queues are then closed so that every
  thread unblocks. This is the same chain as LabVIEW's `Release Queue (force destroy)`.
- **Responses:** LabVIEW pushes every reply onto the TCP loop's queue, and only that loop writes to the socket. The rewrite may
  instead let each worker write its reply frame directly while holding a per-connection send lock. This is the choice made in
  [01](01-architecture.md) §10 and [02](02-tcp-protocol.md) §11.2. Both designs keep frames from interleaving. A queue to the
  TCP thread is equivalent provided it is unbounded and the TCP thread never blocks on it.

## 4. Library mapping (LabVIEW → Python)

| LabVIEW / vendor library | Used for | Python replacement |
|---|---|---|
| Queues, notifiers, While loops | Producer/consumer loops, synchronous replies | `threading`, `queue.Queue`, `threading.Event` |
| TCP primitives, `tcp.llb` (`TCP Listen`) | Server socket | `socket` (`create_server`, timeouts) or `asyncio` streams |
| Flatten/Unflatten To String, Variant To Data | Wire format | `lvflatten.py` (`struct`, big-endian, I32 length prefixes, variant type descriptors) |
| `config.llb` (Open/Read Key/Close Config Data) | `alox_setup.ini`, `alox.ini` | Small NI-compatible reader, or `configparser` with NI rules: case-insensitive, first occurrence wins, `;` comments, quotes, `.`/`,` decimals |
| `error.llb` (error cluster, `Error Cluster From Error Code`, `Clear Errors`) | Error propagation and replies | `errors.py`: exceptions inside, `ErrorCluster` at the TCP boundary, same numeric codes |
| NI-IMAQdx (Open/Configure/Start/Get Image2/Stop/Close, attribute nodes) | Camera control | Vendor SDK (pypylon / VmbPy / PySpin) or Harvesters + GenTL producer, or Aravis. The choice depends on Q7. |
| NI Vision (Symmetry, Flat Field Correction, Resample, Extract, FFT, EdgeDetection, Histogram, LowPass, Overlay, Write PNG + Vision Info) | Image pipeline and focus metrics | NumPy, OpenCV (`cv2.flip`, `cv2.resize(INTER_NEAREST)`, `cv2.Sobel`…), `numpy.fft`, PNG via OpenCV/Pillow; NI-specific conventions are documented in [07](07-focus-metrics.md) |
| `IMAQ Flatten Image to String` | Image replies to the client | (a) call NI Vision's flatten function through `ctypes` if the NI Vision Runtime stays installed; (b) re-implement the format from captures; (c) change the client. Depends on Q1/Q4/Q6. |
| PI GCS LabVIEW driver (`Mercury_GCS_Configuration_Setup`, MOV, MVR, VEL, POS?, FRF, FNL, STP, JON, JAX, `#5`, `#24`, MAC START, RMC?, ERR?) | Z stage | `pipython` (`GCSDevice.ConnectRS232`, `MOV`, `MVR`, `VEL`, `qPOS`, `qFRF`, `FNL`, `STP`, `JON`, `JAX`, `IsMoving`, `StopAll`, `MAC_START`, `qRMC`, `qERR`; `pitools` for start-up and waiting) |
| VISA serial (inside the PI driver) | RS-232 | Handled by `pipython` (or `pyserial` directly) |
| OpenG string tools | Line/array helpers | `str.splitlines`, `str.split` |
| Front panel (window hidden in the exe) | Diagnostics | Logs; optionally a small diagnostic page later (Q15) |

## 5. Phased plan

Each phase ends with something runnable and a validation step against the LabVIEW server. The effort figures are **indicative**,
for one developer comfortable with Python and the hardware. They assume the P1 questions are answered when the phase starts.

| Phase | Content | Exit criteria | Indicative effort |
|---|---|---|---|
| **0. Inputs** | Collect the P1 answers: enum lists (Q2), Cam Setup field order (Q3), network capture (Q4), camera/controller models (Q7–Q9), INI files (Q12), language and deployment (Q14), client code (Q31), client calling pattern and timeouts (Q32), field order and widths of the AF/RF/wait/joystick clusters (Q33), NI MAX camera files and buffer settings (Q34), Z-stage test session including `STP 1` (Q35). Record a reference capture session. | Questionnaire P1 items answered; a `.pcapng` with all command types is available. | Owner's time |
| **1. Protocol skeleton** | `lvflatten`, `messages`, `protocol`, `server` (Listen/Read/Send Response, routing, history), `config`, `logs`; simulated camera and stage backends; `replay_client`. | Every request in the capture decodes. Deterministic responses (`Get Heartbeat`, acknowledgements, getters with fixed values) are **byte-identical** to the LabVIEW ones except for the U32 tick fields (`TCP Sending timestamp`, the `Get Heartbeat` value). The client connects and runs with simulated devices. | 2–3 weeks |
| **2. Z stage** | `pi_stage` on `pipython`; the 19 Z commands; reference check before moves; wait-end-move; macros per Q10/Q11. | Scripted sequence (setup, reference, absolute/relative moves, out-of-range, stop, joystick) gives the same replies and positions as LabVIEW on the real controller. | 1–1.5 weeks |
| **3. Cameras** | GenICam/vendor backend; Cam Setup/Start/Stop/Cleanup, Get Image, Stream Next Image, Exposure/Gain/Frame rate, Flat field, Flip, LED/bulb, acquisition status, temperatures; image replies (option chosen in section 4); simulation backend. | Images received by the real client are identical in content and metadata; exposure and gain round-trip; LED/bulb toggles the right lines. | 2–3 weeks |
| **4. Focus** | `metrics`, `autofocus`, `refocus`, refocus image logging. | Metrics match NI reference outputs within tolerance on reference images (Q29); autofocus/refocus `[Z, score]` tables match LabVIEW runs on the same sample, and refocus ends at the same best position. | 1.5–2 weeks |
| **5. Hardening and deployment** | Error-handling review, configurable quirks (section 6), PyInstaller exe or Windows service, start-up/shutdown, log rotation, install notes. | Soak test (several hours of client traffic, including streaming and AF/RF), clean shutdown via `Exit`, restart without manual steps. | 1–1.5 weeks |
| **6. Parallel run and cut-over** | Run the Python server on the lab PC for real work, with the LabVIEW exe kept as fallback. | Agreed period without regression; LabVIEW server retired. | Calendar time |

**Total: roughly 8–11 developer-weeks** of implementation and validation, plus the owner's time for phase 0 and for testing on the
machine.

## 6. Behaviour fidelity: reproduce or fix

The analysis found about 20 places where the LabVIEW code does something that looks unintended. Each is described in its
chapter, and the decisions are collected in the questionnaire (part D, Q16–Q28). The plan:

- **Default rule:** anything the client can observe (reply or no reply, reply type, error code, timing contract) is
  **reproduced** until the owner or a network capture confirms the client does not depend on it.
- Each such behaviour gets a **named switch**, defaulting to the LabVIEW behaviour the client observes. The table below is the
  complete registry. The switches live in a `[compat]` section of the new Python-only settings file (proposed
  `<settings>\alox_camera_server.ini`, [09](09-configuration-logging-errors.md) §12 item 2). `alox.ini` is never written (Q36).

| Switch (proposed) | LabVIEW behaviour | Default | Fixed behaviour |
|---|---|---|---|
| `send_error_replies` | LabVIEW **sends** error acknowledgements: the network capture shows a `Cam Set Led` error reaching the client ([answers/capture-analysis.md](answers/capture-analysis.md) §3.2). This corrects the earlier inference from [Doc6 p.437] ([02](02-tcp-protocol.md) §8.4). | TRUE (LabVIEW) | FALSE only to suppress error ACKs |
| `reply_unknown_commands` | Unknown or unhandled commands get no reply ([02](02-tcp-protocol.md) §11.3, [03](03-command-reference.md) C2) | FALSE (LabVIEW) | TRUE: reply with an "unknown command" error |
| `listen_address` | Listens on `127.0.0.1` only ([02](02-tcp-protocol.md) §3) | `127.0.0.1` (LabVIEW) | Unchanged. Another address only if remote clients are needed: it exposes an unauthenticated control port (Q36) |
| `inbound_pause_ms` | ≈ 50 ms pause after each inbound frame ([02](02-tcp-protocol.md) §4.6) | 0 (carve-out (b) below) | 0 |
| `close_on_extra_heartbeat_data` | A `Heartbeat` frame with a payload closes the connection ([02](02-tcp-protocol.md) §9) | TRUE (LabVIEW) | TRUE (harmless) |
| `exposure_sets_max_framerate` | After `Cam Set Exposure` on **High Mag**, the frame rate is set to the camera maximum (`MaxDBL`). Low Mag writes the computed 0.95/exposure rate only when `Framerate Type` is I64/U32, and nothing with the INI default DBL ([05](05-camera-acquisition.md) §5.2) | TRUE (LabVIEW) | FALSE: High Mag applies the computed 0.95 / exposure frame rate (Q16) |
| `strict_new_frame` | `Cam Stream Next Image` may return a frame whose exposure began before the request ([05](05-camera-acquisition.md) §6.4, #2; Q25) | FALSE (LabVIEW) | TRUE: return the first frame strictly after the request |
| `request_buffer_zero` | Buffer 0 is requested, so during the first 5 frames after `Cam Start` the first frame ever acquired is returned (stale) (inferred; [05](05-camera-acquisition.md) §6.4, #1) | TRUE (LabVIEW) | FALSE: return the newest frame (Q5, Q32) |
| `temperature_error_fails_image` | A camera-temperature read error fails the image request ([05](05-camera-acquisition.md) #4) | TRUE (LabVIEW) | FALSE: log the temperature error and return the image (Q40) |
| `flatfield_check_on_set` | `Cam Set FlatField` accepts a missing correction file; every later acquisition then fails ([05](05-camera-acquisition.md) #17) | FALSE (LabVIEW) | TRUE: return the file error at set time |
| `focus_reject_concurrent_start` | A second focus start while a job runs is acknowledged, and both jobs run interleaved ([06](06-autofocus-refocus.md) D4) | FALSE: replace the running job (abort it, start the new one, success ACK as in LabVIEW) | TRUE: reject the new start with an error (Q39) |
| `focus_validate_params` | Invalid focus parameters get a success ACK, then the job hangs, fails or sends the stage to 0 mm ([06](06-autofocus-refocus.md) D8) | FALSE: success ACK as in LabVIEW, no motion, warning logged | TRUE: error reply (Q39) |
| `af_check_z_limits` | Autofocus does not check `[Z Min, Z Max]` against the stage limits ([06](06-autofocus-refocus.md) D5) | FALSE (LabVIEW, no check) | TRUE: reject the start with 9405 (Q39) |
| `restart_dh_macro` | `DH` macro restarted by Get Status / Stop / Cleanup ([08](08-z-stage.md) Z8) | TRUE (LabVIEW) | FALSE: do not restart it (Q10) |
| `implicit_z_reinit` | Axis silently re-initialised before a move if not referenced ([08](08-z-stage.md) Z2) | TRUE (LabVIEW) | FALSE: return an error instead (Q24) |

`restart_dh_macro` and `implicit_z_reinit` match the fields of `ZCompat` in [08](08-z-stage.md) §13.4. With their default,
`focus_reject_concurrent_start` and `focus_validate_params` keep the reply LabVIEW sends and fix only the internal defect
(interleaved jobs, motion on invalid parameters).

- **Internal defects** that the client cannot observe are **fixed** directly. These include the self-enqueue deadlocks, unbounded
  histories, the log file overwritten at offset 0, swapped HI/LO labels, the race in the FULL focus mode, reopening a camera
  without closing it, and repeated buffer allocations.
- **Two carve-outs** from the default rule. Chapters 01, 02, 03, 04, 05 and 08 already apply them.
  - **(a) Malformed or out-of-contract input.** The reactions to it are not part of the contract and are fixed directly. This
    covers undecodable frames, payloads of the wrong type, invalid enum values and invalid frame lengths
    ([01](01-architecture.md) A20, [02](02-tcp-protocol.md) D5–D7, [03](03-command-reference.md) C4/C16,
    [04](04-data-types.md) T1/T6/T8, [05](05-camera-acquisition.md) #29/#32, [08](08-z-stage.md) Z10). Any error reply still
    follows `send_error_replies`.
  - **(b) Pure performance artefacts.** They are not part of the timing contract and are removed directly: the ≈ 50 ms pause
    after each inbound frame, the 10 ms polling and the 2 s `Listen` ([02](02-tcp-protocol.md) D8/D9, [01](01-architecture.md)
    A4). Hence `inbound_pause_ms` = 0.

## 7. Validation strategy

| Level | How | Source of truth |
|---|---|---|
| Codec | Unit tests: flatten/unflatten round-trips for every typedef; decode every request of the capture; byte-compare deterministic replies. | Network capture (Q4), enum lists (Q2) |
| Server behaviour | `replay_client` runs the same scripted session against the LabVIEW server and the Python server and diffs the replies (both using simulated cameras where possible). | LabVIEW server on the lab PC |
| Numerics | Compare focus scores, resampling and flat-field output with NI Vision on reference images; check that the argmax of autofocus/refocus curves is identical. | Reference outputs exported from LabVIEW (Q29) |
| Hardware | Checklist per command on the real cameras and Z stage (section 5, phases 2–3). | Owner on the machine |
| System | Soak test with the real client, including streaming, AF/RF, disconnect/reconnect, and `Exit`. | Owner on the machine |

## 8. Risks

| # | Risk | Impact | Likelihood | Mitigation |
|---|---|---|---|---|
| R1 | **Enum order/values unknown.** `Camera Command` decides both decoding and **routing by range**; with the wrong order, some Z commands would never reach the Z loop. | Blocking | Certain until answered | Q2 (Edit Items screenshots) and Q4 (capture) before phase 1 is frozen |
| R2 | **NI image serialisation** (`IMAQ Flatten Image to String`) is proprietary and undocumented. | High if the client stays unchanged and NI must be removed | Medium | Prefer calling the NI runtime through `ctypes` (Q6); otherwise derive the format from captures, or adapt the client (Q1) |
| R3 | **LabVIEW variant flattening** (type descriptors) in incoming commands | Medium | Medium | Implement a type-descriptor parser for the descriptor kinds actually used; build test vectors from the capture |
| R4 | **Camera interface.** A FireWire/IIDC camera may lack a usable modern SDK; GigE/USB3 attribute names differ between models. | High | Unknown (Q7) | Identify models first; choose the vendor SDK; map IMAQdx attribute paths to GenICam node names from `alox.ini` |
| R5 | **Numerical parity with NI Vision** (nearest-neighbour rounding, edge kernels and saturation, Sigma filter) | Medium (focus position drift) | Medium | Reference images (Q29); compare argmax rather than raw scores; document tolerances |
| R6 | **Hidden client dependencies on quirks** (timeouts instead of error replies, missing replies) | Medium | Medium | Compatibility switches defaulting to LabVIEW behaviour (section 6); capture-based checks; client code and calling pattern (Q31, Q32) |
| R7 | **Unknown PI macros** `DH`/`HOME` restarted automatically. They may move the stage. | Medium (safety) | Unknown (Q10) | Read the macros (`MAC?`) before enabling the restart |
| R8 | **Material not in the printouts:** the Oxyd refocus sub-VIs; `Mercury_GCS_Configuration_Setup.vi` (password-protected, only its documented steps are known) | Low/Medium | Certain | Oxyd branch is dead code (keep "Fin" only unless Q19 says otherwise); PI set-up re-implemented with `pipython`/`pitools` from the documented steps |
| R9 | **Timing semantics** of IMAQdx buffers (`LastBufferNumber`, "Next" vs "Buffer Number" modes) differ from vendor SDKs | Medium | Medium | Implement "next frame after request" explicitly; copy the NI MAX buffer settings (Q34); test on the real cameras |

## 9. Missing inputs (from the owner)

All open items are in the fill-in questionnaire [`open-questions.md`](open-questions.md). The ones that block the design are:

- **Q1:** whether the client stays unchanged.
- **Q2:** enum lists and representation.
- **Q3:** the Cam Setup cluster order.
- **Q4:** a network capture.
- **Q6:** whether the NI runtime is allowed.
- **Q7–Q9:** camera, light wiring and PI controller identity.
- **Q12:** the real INI files.
- **Q14:** language and deployment constraints.
- **Q29:** reference data.
- **Q31:** the client code.
- **Q32:** the client's calling pattern and timeouts.
- **Q33:** field order and widths of the AF/RF/wait/joystick clusters.
- **Q34:** the NI MAX camera files and buffer settings.
- **Q35:** a Z-stage test session, including `STP 1`.
