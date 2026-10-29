# Camera acquisition and image pipeline

**Summary.** This chapter specifies how the ALOX Camera Server drives its two cameras ("High Mag." and "Low Mag.") through NI-IMAQdx. It covers the session lifecycle (`Cam Setup`, `Cam Start`, `Cam Stop`, `Cam Cleanup`), how GenICam attributes are accessed (exposure, gain, frame rate, trigger, temperature, digital I/O), the per-request acquisition state machine behind `Cam Stream Next Image`, and the processing applied to every streamed image (software flip, flat-field / dark-current correction, custom metadata). It also covers the folder-based simulation mode, LED/bulb control through the camera output lines, and the error codes. It ends with a Python mapping (Harvesters/GenTL, vendor SDKs, Aravis, and the FireWire/IIDC risk), concurrency notes, the defects found in the LabVIEW code, and the open questions. The TCP framing and the command envelope are covered in [02-tcp-protocol.md](02-tcp-protocol.md) and [03-command-reference.md](03-command-reference.md); binary layouts are in [04-data-types.md](04-data-types.md).

**Sources.** `alox_camera_server.vi` (main VI): camera consumer loop frames d79–d216 (`Cam Setup` d79–d89, `Cam Start` d90–d95, `Cam Get Image` d98–d103, `Cam Stream Next Image` d104–d115, `Cam Get Acquisition Status` d116–d118, `Cam Software Trigger` d119–d123, `Cam Set Exposure` d0 + d124–d140, `Cam Set FlatField` d141–d147, `Cam Set Gain` d148–d154, `Cam Get Exposure` d155–d161, `Cam Get Gain` d162–d168, `Cam Get Images Array` d169, `Cam Stop` d170–d174, `Cam Cleanup` d175–d177, `Cam Flip` d178–d180, `Cam Set Led`/`Cam Get Led` d195–d204, Disable structure d215–d216); loop timeout frames d217–d219; start-up INI frames d3–d47; main VI PDF p.2–5, 11, 14, 17–24, 26–27, 29. `alox_camera_server_get_image_state_machine.vi` [Doc6 p.552–558]; `alox_camera_server_simulation.vi` [Doc6 p.919–923]; `alox_vision_flip_image.vi` [Doc6 p.928–933]. Typedefs: `alox_typedef_camera_type.ctl` [Doc6 p.616–619], `alox_typedef_ext_camera_session.ctl` [Doc6 p.628–631], `typedef_ext_camera_get_image_state.ctl` [Doc6 p.632–635], `alox_typedef_ext_camera_properties.ctl` [Doc6 p.648–651], `typedef_ext_camera_stream_data.ctl` [Doc6 p.540–543], camera command typedefs [Doc6 p.341–404], `typedef_ext_camera_camera_command_type_LED.ctl` [Doc6 p.887–890]. NI VIs: IMAQdx Open Camera [Doc6 p.1183–1188], Configure Acquisition [Doc6 p.1175–1182], Start Acquisition [Doc6 p.1171–1174], Get Image2 and Buffer Number Mode [Doc6 p.891–898], Stop / Unconfigure / Close [Doc6 p.575–586], IMAQ Write Custom Data [Doc6 p.544–547], IMAQ Clear Overlay [Doc6 p.548–551], IMAQ Read Image And Vision Info 2 [Doc6 p.899–902], IMAQ GetImageInfo [Doc6 p.907–910], IMAQ Cast Image [Doc6 p.911–914], IMAQ ExtractSingleColorPlane [Doc6 p.915–918], IMAQ Symmetry [Doc6 p.924–927], IMAQ Flat Field Correction [Doc6 p.934–937], IMAQ Create [Doc6 p.1002–1003], IMAQ Dispose [Doc6 p.1014], Clear Errors [Doc6 p.57–59]. Reply VIs (only as far as they affect camera commands): `Enqueue Send Error Cluster Reponse.vi` [Doc6 p.435–437], `Enqueue Send Image Response.vi` [Doc6 p.571–573].

---

## 1. Conventions and LabVIEW primer

### 1.1 Conventions

- Untagged statements are observed on the printouts. **(inferred)** marks a deduction from LabVIEW/NI semantics or from cross-checking. **(unknown)** marks something the printouts do not show.
- Citations: `[main VI frame dN]` refers to hidden frame N of the HTML export of `alox_camera_server.vi` (`d0` is the main diagram as displayed). `[main VI p.N]` refers to a page of the main VI PDF. `[Doc6 p.N]` refers to a page of "Labview Document 6" (sub-VIs, typedefs and NI library VIs).
- **HM** means the "High Mag." camera and **LM** the "Low Mag." camera. The LabVIEW comments also call HM the "Low Angle" camera and LM the "High Angle" camera ("LOW ANGLE = HIGH MAG / HIGH ANGLE = LOW MAG" [main VI frames d195, d202]). The shift-register wire labels "Low Angle Cam Session" (on the HM session) and "High Angle Cam Session" (on the LM session) follow the same inverted naming [main VI frames d104, d141].
- `Camera Type` is the enum `High Mag.` / `Low Mag.`. `High Mag.` is the default and has the value 0 (inferred: the state machine tests `Camera Type == 0` to produce the text "High-Mag" [Doc6 p.554]).

### 1.2 LabVIEW behaviour that matters for fidelity

| LabVIEW construct | What it means here | Python equivalent |
|---|---|---|
| Error cluster `{status: bool, code: I32, source: string}` | It flows through nodes. A node whose `error in.status` is TRUE does nothing and passes the error on. `status=FALSE` with `code≠0` is a warning. An unwired `error out` means the error is silently ignored. | Exceptions carrying `code` and `source`. Ignored errors become explicit `try/except` blocks for the specific codes. |
| Case structure driven by an error cluster (`Error` / `No Error` frames) | This pattern is used to append text to `source` only when there is an error. Warnings take the `No Error` path. | `if err is not None: err.source += suffix` |
| `Select` / Case driven directly by an error cluster | Tests `status` only. An error with `status=FALSE` and a non-empty `source` counts as "no error". | Test `err is not None` |
| `Clear Errors` with a specific code [Doc6 p.57–59] | Clears the error only if its code matches. With code 0 it clears everything. | `except CamError as e: if e.code != CODE: raise` |
| `Merge Errors` | Returns the first error in input order. | First non-`None` error |
| Uninitialised shift register in a single-iteration loop | This is a static variable private to the VI. It persists between calls. If the VI is non-reentrant (the LabVIEW default), every caller shares it. | An attribute of an explicit object, one instance per camera |
| IMAQ image | A reference to a named image buffer. A Vision VI with an unwired `Image Dst` writes its result into `Image Src` (in place) (inferred, standard NI convention). | A `numpy.ndarray` held in the session and replaced explicitly |
| IMAQdx property node | Each line executes top to bottom. `ActiveAttribute ← "<name>"` selects a camera attribute, then `ValueDBL` / `ValueI64` / `ValueU32` / `ValueBool` / `ValueString` reads or writes it, and `MaxDBL` reads its maximum. | GenICam node-map access by feature name |
| `Variant To Data` with an unwired error output | If the variant holds the wrong type, default values are used silently: `Camera Type = High Mag.`, FALSE, empty strings (inferred). Whether a saved non-zero default such as `Acquisition Timeout (ms) = 5000` or a plain 0 is used is **(unknown)** ([04-data-types.md](04-data-types.md)). | Validate the payload. Reject it, or apply the defaults deliberately. |
| `Tick Count (ms)` | A free-running U32 millisecond counter. It wraps after about 49.7 days. | `int(time.monotonic()*1000) & 0xFFFFFFFF` |
| `In Range and Coerce` | Clamp to `[lower, upper]` | `min(max(x, lo), hi)` |
| DBL → I64/U32 coercion | Rounds to nearest with ties to even, and saturates to the target range (inferred, standard LabVIEW). | `round()` plus saturation |

---

## 2. Camera model and interface assumptions

| Aspect | What the LabVIEW server does or assumes | Evidence |
|---|---|---|
| Driver | Every camera access goes through NI-IMAQdx (`IMAQdx.llb`): Open/Configure/Start/Get Image2/Stop/Unconfigure/Close, plus IMAQdx property nodes. | [main VI frames d79, d90, d170, d175], [Doc6 p.1171–1188] |
| Camera identification | The camera is opened by the **IMAQdx camera name sent by the client** in `Cam Setup` (`Camera Init Data.Camera Name`), coerced to an IMAQdx session. It is **not** opened by the INI `name_STR`, which only feeds `Settings.Camera Name` and the `name` custom-data key. Names are the IMAQdx/NI MAX camera names (for example `cam0`) (inferred). | [main VI frame d79] |
| Control mode | `Camera Control Mode` is unwired, so the default `Controller` (exclusive control) applies. | [Doc6 p.1184, p.1188] |
| Camera-side configuration | `IMAQdx Open Camera` "Opens a camera, queries the camera for its capabilities, loads a camera configuration file". Every feature the server does not set therefore comes from the camera file saved in NI MAX: pixel format, ROI, binning, trigger source and selector, line configuration, buffer overwrite mode. The contents of these files are **(unknown)**. | [Doc6 p.1183] |
| Bus type | The `Camera Plug Type` setting (`GiGe` or `USB`, from INI `cameraconnectortype_STR`) selects between GigE-only actions (jumbo packet size, `TriggerMode`, `TriggerSoftware`) and USB variants. The IMAQdx `BusType` property is read at setup and only displayed (`BusTypeHM`, `BusTypeLM`). The printed front panel shows `BusTypeHM = FireWire`, but the same panel shows `SerialNumberHigh = SerialNumberLow = 0`, `Acq. State = Start` and an empty `Camera Name`, i.e. values that were not saved from a running camera. `FireWire` may therefore be only the indicator's saved default (inferred), and the real HM bus is **(unknown)**. | [main VI frames d79, d84; p.2] |
| Acquisition model | Continuous grab into a 5-buffer ring when the client sends `Continuous Mode = TRUE` (§4.3). The streaming state machine waits for the cumulative buffer counter `LastBufferNumber` to advance and never triggers the camera itself. | [main VI frame d79], [Doc6 p.554–556] |
| Pixel type | The server never sets it. `IMAQdx Get Image2`: "If the image type does not match the video format of the camera, this VI changes the image type to a suitable format." The source image is created as `Grayscale (U8)`, which suggests monochrome 8-bit cameras (inferred). The actual pixel format is **(unknown)**. | [Doc6 p.895, p.1003], [main VI frame d79] |
| Temperature | Every `Cam Stream Next Image` pass reads `CameraAttributes::DeviceControl::DeviceTemperature`. A camera without this feature breaks streaming (see §5.7 and §13 #4). | [main VI frames d104, d110] |

### 2.1 Camera attributes used by the server

IMAQdx names GenICam features `CameraAttributes::<Category>::<Feature>`. Session-level IMAQdx properties have no prefix.

| IMAQdx attribute / property (exact text) | Access | Used by | GenICam / Python equivalent |
|---|---|---|---|
| `BusType`, `SerialNumberHigh`, `SerialNumberLow` (session properties) | read, errors ignored | `Cam Setup` (display only; LM reads only `BusType`) | GenTL device info (transport layer type, serial number) |
| `PacketSize` (session property) | write `8996` | `Cam Setup`, GigE only | `GevSCPSPacketSize` (inferred); needs a jumbo-frame NIC |
| `AcquisitionInProgress` (session property) | read | `Cam Stop`, `Cam Get Acquisition Status`, dead timeout frame | Backend "grabbing" flag |
| `LastBufferNumber`, `BufferCount` (session properties) | read | state machine | Backend cumulative frame counter. The meaning of `BufferCount` is **(unknown)**; it only appears in an error text. |
| `CameraAttributes::AcquisitionControl::TriggerMode` | write `ValueString` `"On"`/`"Off"` | `Cam Start`, GigE only | `TriggerMode` (enumeration) |
| `CameraAttributes::AcquisitionControl::TriggerSoftware` | write `ValueBool ← TRUE` | `Cam Software Trigger`, GigE only | `TriggerSoftware.execute()` (command) |
| `CameraAttributes::DeviceControl::DeviceTemperature` | read `ValueDBL` | `Cam Stream Next Image` | `DeviceTemperature` |
| `CameraAttributes::DigitalIOControl::LineSelector`, `::LineMode`, `::LineSource` | write `ValueU32` | `Cam Set Led`, `Cam Get Led` | Enumerations written **by integer value** |
| `CameraAttributes::DigitalIOControl::LineInverter` | write / read `ValueBool` | LED, USB path | `LineInverter` |
| `CameraAttributes::DigitalIOControl::UserOutputSelector` | write `ValueU32` | LED, GigE path | `UserOutputSelector` (by integer value) |
| `CameraAttributes::DigitalIOControl::UserOutputValue` | read, then write `ValueBool` | LED, GigE path | `UserOutputValue` |
| `CameraAttributes::DigitalIOControl::LineStatus` | constant placed on the diagram but unwired | — | not used |
| `Settings.Camera Exposure Attribute` (INI `exposure_STR`, default `CameraAttributes::Exposure::Value`) | write/read, typed by `Exposure Type` | Set/Get Exposure | configurable (see §12.4) |
| `Settings.Camera Gain Attribute` (INI `gain_STR`, default `CameraAttributes::Gain::Value`) | write/read, typed by `Gain Type` | Set/Get Gain | configurable |
| `Settings.Camera Framerate Attribute` (INI `framerate_STR`, default `CameraAttributes::AcquisitionControl::AcquisitionFrameRate`) | write, plus `MaxDBL` read on HM | Set Exposure | `AcquisitionFrameRate` |

Sources: [main VI frames d0, d79, d90, d104, d116, d119, d170, d195, d197, d202, d203; p.19–20, 26–27], [Doc6 p.554–556], INI defaults [main VI frames d14–d20].

The default exposure and gain attribute names (`CameraAttributes::Exposure::Value`, `CameraAttributes::Gain::Value`) do not follow the SFNC naming used by all the other hard-coded attributes (`AcquisitionControl::…`, `DeviceControl::…`, `DigitalIOControl::…`). The `<Feature>::Value` shape is the one IMAQdx uses for non-GenICam cameras, such as IIDC (FireWire) or DirectShow (USB) devices (inferred). These INI defaults may simply be stale: the attribute names in the deployed `alox.ini` decide which features are actually written (see §14).

---

## 3. Data structures used by the camera subsystem

The binary layouts of the TCP payloads are in [04-data-types.md](04-data-types.md). This section gives the semantics.

### 3.1 Camera Settings (`alox_typedef_ext_camera_properties.ctl`) [Doc6 p.648–651]

There is one instance per camera: the front-panel indicators `High Mag Settings` and `Low Mag Settings`. Step 2 of the start-up sequence fills them from `alox.ini` sections `[hardware.camera.highmag]` / `[hardware.camera.lowmag]` [main VI frames d5–d47]. At run time only `Cam Flip` (flip flags) and `Cam Set FlatField` (flat-field fields) modify them. They are never written back to disk. Every command reads them through local variables. The INI parsing rules are in [09-configuration-logging-errors.md](09-configuration-logging-errors.md).

Cluster order comes from the positional Unbundle [main VI frame d79], [Doc6 p.554]:

| # | Field | Type | INI key | INI default (HM / LM) | Used by |
|---|---|---|---|---|---|
| 0 | Camera Name | string | `name_STR` | `HighMag` / `LowMag` | custom-data key `name` only |
| 1 | Buffers | I32 | `buffers_I32` | 5 | **not used** (the ring size is the constant 5) |
| 2 | Timeout | I32 | `timeout_I32` | 5000 | **not used** (unbundled but unwired [Doc6 p.555]) |
| 3 | Flat Field | bool | `flatfieldcorrection_BOOL` | FALSE | `Cam Setup` (load FFC/DFC), state machine and AF/RF (apply) |
| 4 | Flat Field Path | path | `flatfieldimage_PATH` | empty | `Cam Setup` |
| 5 | Dark Current Path | path | `darkfieldimage_PATH` | empty | `Cam Setup` |
| 6 | Flat Field Factor | DBL | `flatfieldfactor_DBL` | 1 | correction factor, coerced to SGL |
| 7–10 | Camera Exposure Attribute, Camera Gain Attribute, Exposure Type, Gain Type (order among these four is **(unknown)**) | string ×4 | `exposure_STR`, `gain_STR`, `exposuretype_STR`, `gaintype_STR` | `CameraAttributes::Exposure::Value`, `CameraAttributes::Gain::Value`, `DBL`, `I64` | Set/Get Exposure, Set/Get Gain |
| 11 | Horizontal Flip | bool | `hflip_BOOL` | FALSE | flip (state machine, `Cam Get Image`, AF/RF) |
| 12 | Vertical Flip | bool | `vflip_BOOL` | FALSE | flip |
| 13 | Camera Framerate Attribute (name inferred from position) | string | `framerate_STR` | `CameraAttributes::AcquisitionControl::AcquisitionFrameRate` | Set Exposure |
| 14 | Framerate Type (name inferred from position) | string | `frameratetype_STR` | `DBL` | Set Exposure |
| 15 | Max Framerate | DBL | `frameratemax_DBL` (**LM only**) | HM: never read from the INI; keeps the value of an unprintable constant **(unknown)**, probably 0 (inferred). LM: 24.6 | Set Exposure |
| 16 | Wait LED (ms) | U32 | `waitled_U32` | 0 | Cam Set Led |
| 17 | Invert LED | bool | `invertled_BOOL` | FALSE / **TRUE** | Cam Set/Get Led |
| 18 | Camera Plug Type | enum `GiGe` (default), `USB`, others **(unknown)** | `cameraconnectortype_STR` | `gige` | Cam Setup, Cam Start, Software Trigger; also via `Camera Sort: LED` |
| 19 | Exposure Lower Limit | DBL | `LimitMinExposure_DBL` | 177 | Set Exposure |

`cameraconnectortype_STR` becomes `USB` only for the exact strings `USB`, `USb`, `UsB`, `Usb`, `uSB`, `usb`. Every other value, including `uSb` and `usB`, becomes `GiGe` [main VI frames d24–d25, d46–d47].

### 3.2 Camera Session (`alox_typedef_ext_camera_session.ctl`) [Doc6 p.628–631]

Two shift registers of the camera loop hold the sessions: the upper one for HM, the lower one for LM (wire routing checked in [main VI frames d90, d98, d104, d116, d141, d175, d178]). They are initialised from the diagram constants `High Mag Session` / `Low Mag Session`. Those constants are drawn as icons, so their values are **(unknown)**; they are probably the typedef defaults (inferred).

| Field | Type | Typedef default | Written by | Read by |
|---|---|---|---|---|
| Session | IMAQdx session | none | Cam Setup; reset by Cam Cleanup | every hardware command |
| SRC Image | IMAQ image | null | Cam Setup, Cam Set FlatField (`IMAQ Create`) | destination of every acquisition (state machine, Cam Get Image, AF/RF); source of the image response |
| FFC Image | IMAQ image | null | Cam Setup, Cam Set FlatField | flat-field correction |
| DFC Image | IMAQ image | null | Cam Setup, Cam Set FlatField | flat-field correction |
| Simulation | bool | FALSE | state machine (`Start`; `Acquire` in simulation) | state machine (`Acquire`), `Cam Get Image` |
| Simulation Path | path | empty | state machine (`Start`; `Acquire` in simulation) | not read (the simulation uses the path from the request) |
| Simulation Index | integer | 0 | state machine | state machine |
| Simulation Time | numeric, not U32 (there is a coercion dot where a U32 tick is written) | 0 | Cam Setup (Tick Count), state machine | state machine |
| Last Focus Data | 2-D DBL | empty | AF/RF | see [06-autofocus-refocus.md](06-autofocus-refocus.md) |
| Acquisition State | enum (§3.4) | `Start` | Cam Setup (`Stopped`), Cam Stream Next Image | Cam Stream Next Image |
| AF State, RF State | enums | `Start` | AF/RF | see chapter 06 |
| Focus Progress (%) | numeric | 0 | AF/RF | see chapter 06 |

### 3.3 Camera Stream Data (`typedef_ext_camera_stream_data.ctl`) [Doc6 p.540–543]

This is the payload of `Cam Stream Next Image`. The whole cluster is passed to the state machine.

| # | Field | Type | Default | Meaning |
|---|---|---|---|---|
| 0 | Camera Type | enum | `High Mag.` | camera to use |
| 1 | Simulation | bool | FALSE | serve PNG files instead of camera frames |
| 2 | Simulation Path | path | empty | folder containing `*.png` |
| 3 | Simulation Timer (ms) | U32 | 0 | minimum period between two simulated frames |
| 4 | Acquisition Timeout (ms) | I32 (inferred: an integer with a coercion dot against U32) | **5000** | maximum wait for a new camera buffer, measured from `Start` |

### 3.4 Get Image State (`typedef_ext_camera_get_image_state.ctl`) [Doc6 p.632–635]

The observed values (state-machine frames) are `Start`, `Acquire`, `Finished`, `Error`, `Stopped`, and the state machine also has a `Default` frame. The `.ctl` default is `Stopped`; the session field default is `Start`. Numeric values and order are **(unknown)**. They only matter if the enum is ever serialised, which the observed code does not do.

### 3.5 Front-panel state belonging to the camera subsystem

These are module-level state in the LabVIEW program. None is returned to the TCP client by any observed command.

| Object | Written by | Meaning |
|---|---|---|
| `HI Inited`, `LO Inited` | step 1 (FALSE), Cam Setup (`NOT error`), Cam Cleanup (FALSE) | camera configured |
| `High Acq. State`, `Low Acq. State` | Cam Setup, Cam Stream Next Image | copy of `Session.Acquisition State` |
| `BusTypeHM` / `BusTypeLM` (U32 terminals [main VI frame d79]; the panel shows a bus name, so a ring or enum (inferred)), `SerialNumberHigh`, `SerialNumberLow` (U32) | Cam Setup | camera info (display) |
| `HM T° (°C)`, `LM T° (°C)`, `Temperature Graph` | Cam Stream Next Image; camera loop | temperature (§5.7) |
| `Max Framerate` (DBL), `MaxFR real` (U32) | Cam Set Exposure (HM only) | display |
| `Image` (IMAQ display, `Zoom to Fit` set TRUE at start) | Cam Get Image, Cam Stream Next Image (`Finished`), AF/RF | preview |
| `Camera Sort: LED` (enum, same type as Camera Plug Type (inferred)), `Connector` (`"HM"`/`"LM"`) | step 2 (INI) | which camera drives the lights (§9) |
| `HM setLEDline`, `HM setBULBline`, `LEDStatusInverter` (switch), `BulbStatusInverter` (switch) | step 2 (INI), except `BulbStatusInverter`, which keeps its saved front-panel value | LED/bulb I/O (§9) |
| `LED cmd`, `LEDStatus`, `String` | Cam Set Led / Cam Get Led | trace indicators |

Sources: [main VI frames d79, d84, d104, d175, d195, d202; p.2], start-up sequence [main VI frames d2, d3].

### 3.6 Camera commands covered by this chapter

All of them are executed by the camera consumer loop ([01-architecture.md](01-architecture.md)). Field order, enum values and byte layouts are in [04-data-types.md](04-data-types.md); the envelope and the reply framing are in [02-tcp-protocol.md](02-tcp-protocol.md). "Error cluster" replies are probably not sent at all when they carry an error (see §4.1).

| Command | `Command Data` typedef (fields) | Reply | Section |
|---|---|---|---|
| `Cam Setup` | `Camera Init Data` {Camera Type, Camera Name, Number of buffers, Continuous Mode} (order ambiguous) | error cluster | §4.2 |
| `Cam Start` | `Camera Start Data` {Camera Type, Triggered} | error cluster | §4.4 |
| `Cam Stop` | `Camera Stop Data` {Camera Type} | error cluster | §4.5 |
| `Cam Cleanup` | `Camera Release Data` {Camera Type} | error cluster | §4.6 |
| `Cam Get Acquisition Status` | `Camera Status Data` {Camera Type} | Boolean | §4.7 |
| `Cam Software Trigger` | `Camera Trigger Soft Data` {Camera Type} | error cluster, always "no error" | §4.8 |
| `Cam Set Exposure` | `Camera Set Exposure Data` {Camera Type, Exposure: DBL} | error cluster | §5.2 |
| `Cam Get Exposure` | `Camera Get Exposure Data` {Camera Type} | DBL | §5.3 |
| `Cam Set Gain` / `Cam Get Gain` | `Camera Gain Data` {Camera Type, Gain: DBL} / `Camera Get Gain Data` {Camera Type} | error cluster / DBL | §5.4 |
| `Cam Stream Next Image` | `Camera Stream Data` (§3.3) | Image (exactly once) | §6.2 |
| `Cam Get Image` | `Camera Get Image Data` {Camera Type} | **none** | §6.5 |
| `Cam Get Images Array` | not read | **none** | §6.1 |
| `Cam Flip` | `Camera Flip Data` {Camera Type, Horizontal Flip, Vertical Flip} | error cluster | §7.2 |
| `Cam Set FlatField` | `Camera Ext Set FlatField` {Camera Type, Enable, FFC Path, DFC Path, Correction Factor: DBL} | error cluster | §7.3 |
| `Cam Set Led` / `Cam Get Led` | `Camera Set LED Data` {Camera Type, Led} / `Camera Get LED Data` {Camera Type} | error cluster / Boolean | §9 |

Typedef pages: [Doc6 p.341–404, p.540–543]. `Cam Autofocus`, `Cam Refocus`, their `… Run` variants and the autofocus read-back commands are in [06-autofocus-refocus.md](06-autofocus-refocus.md); `Get Heartbeat`, `Show UI`, `Log` and `Exit` are in [03-command-reference.md](03-command-reference.md).

---

## 4. IMAQdx session lifecycle

### 4.1 Overview

| Command | IMAQdx / NI Vision calls (in order) | Session changes | Reply |
|---|---|---|---|
| `Cam Setup` | Open Camera → read BusType (and serial on HM) → [GiGe] PacketSize=8996 / [USB, LM only] wait 100 ms → Configure Acquisition(Continuous? from client, 5 buffers) → IMAQ Create SRC → [conditional] IMAQ Create + Read FFC/DFC | Session, SRC/FFC/DFC replaced; `Acquisition State=Stopped`; `Simulation Time=tick`; `HI/LO Inited` | error cluster |
| `Cam Start` | [GiGe] `TriggerMode=On/Off` → Start Acquisition | none | error cluster |
| `Cam Stop` | read `AcquisitionInProgress` → if TRUE, Stop Acquisition | none | error cluster |
| `Cam Cleanup` | Unconfigure → Close → Dispose SRC, FFC, DFC | session reset to a constant; `HI/LO Inited=F` | error cluster |
| `Cam Get Acquisition Status` | read `AcquisitionInProgress` | none | Boolean |
| `Cam Software Trigger` | [GiGe] `TriggerSoftware` | none | error cluster (always "no error") |
| `Exit` | **none** (no Stop/Close, no Dispose) | — | none |

```
not set up ──Cam Setup (ok)──► CONFIGURED (Inited=T, Acquisition State=Stopped)
CONFIGURED ──Cam Start──► ACQUIRING (AcquisitionInProgress=T)
ACQUIRING ──Cam Stop──► CONFIGURED            (Acquisition State is NOT updated)
any ──Cam Cleanup──► not set up (session = default constant)
any ──Cam Setup──► re-opens WITHOUT closing the previous session
```

None of these commands checks the current state first. Every call goes straight to the driver, and the driver's error (if any) becomes the reply.

**Error-cluster replies on error.** In `Enqueue Send Error Cluster Reponse.vi` the `error in` wire is flattened into the reply **and**, by a wire running under the `Flatten To String` node, also feeds the `error in` of the `Enqueue Element` that queues the reply [Doc6 p.437]. A LabVIEW queue primitive does nothing when its `error in` has `status = TRUE`, so an error reply is probably never sent: the client only sees its own timeout **(inferred; the wire is partly hidden, confirm with a capture)**. This applies to every "error cluster" reply of §3.6; Boolean, DBL and Image replies carry the error as data and are always sent. The pseudo-code below writes "reply error cluster(err)" for the LabVIEW call; read it with this caveat. The decision (reproduce or always reply) belongs to [02-tcp-protocol.md](02-tcp-protocol.md).

### 4.2 `Cam Setup` [main VI frames d79–d89; p.11]

Payload: `Camera Init Data` {Camera Type, Camera Name, Number of buffers, Continuous Mode}. The cluster order is ambiguous, see [04-data-types.md](04-data-types.md). The `Variant To Data` error is unwired.

```
d ← VariantToData(Command Data, Camera Init Data)
S ← settings of d.Camera Type (local High/Low Mag Settings)    # "High Mag." is also the Default case
(sess, e) ← IMAQdx Open Camera(Session In = d.Camera Name)      # Camera Control Mode unwired = Controller
BusType{HM|LM} ← IMAQdx property(sess).BusType                  # error ignored
HM only: SerialNumberHigh, SerialNumberLow ← IMAQdx property(sess)   # errors ignored
case S.Camera Plug Type:
    "GiGe", Default: (sess, e) ← IMAQdx property(sess, error in = e): PacketSize ← 8996
    "USB":           HM: nothing; LM: Wait(100 ms)
mode ← d.Continuous Mode ? "Continuous" : "One Shot"
(sess, e) ← IMAQdx Configure Acquisition(sess, Continuous? = mode, Number of Buffers = 5, error in = e)
{HI|LO} Inited ← NOT e.status
{High|Low} Acq. State ← "Stopped"
src ← IMAQ Create(Image Name = "HI_SRC_"|"LO_SRC_" + <VI path>)     # Image Type unwired = Grayscale (U8) [Doc6 p.1003]
if exists(S.Flat Field Path) AND S.Flat Field Path ≠ <empty path> AND S.Flat Field:
    ffc ← IMAQ Create("HI_FFC_"|"LO_FFC_" + <VI path>)
    (ffc, eF) ← IMAQ Read Image And Vision Info 2(File Path = S.Flat Field Path, Image = ffc)
else: ffc ← null image ; eF ← no error
same for DFC with S.Dark Current Path, prefix "HI_DFC_"|"LO_DFC_"  → (dfc, eD)
session{HM|LM} ← session with Acquisition State = "Stopped", Simulation Time = TickCount(),
                  Session = sess, SRC Image = src, FFC Image = ffc, DFC Image = dfc
err ← MergeErrors(e, eF, eD)
if err: err.source ← err.source + "Cam Setup" + extra
        # extra = "" for HM; LM: concatenated sources of (Configure, PacketSize, Open) errors
Log Server Error(Extra String = "Setup", Camera Type = d.Camera Type); reply error cluster(err)
```

Notes:
- The ring always has **5 buffers**. `Number of buffers` from the client and `Settings.Buffers` are ignored [main VI frame d79].
- Open → PacketSize → Configure are chained, so a failed Open skips the rest. `Inited` reflects only that chain. FFC/DFC read errors do not affect `Inited`, but they are returned to the client.
- Because the three LM errors are chained, each of them carries the first failure's `source`. A failed LM Open therefore yields `source = s + "Cam Setup" + s + s + s`, where `s` is the driver text (inferred from error-cluster semantics).
- The previous session is neither stopped nor closed before `Open Camera`. What IMAQdx does when a camera that is already open is opened again is **(unknown)**.
- The image names are a fixed prefix plus the VI path (inferred: `Format Into String` with an initial string and a path argument). The name is only there to make images unique per process. `IMAQ Create` with an existing name probably reuses that image (inferred, NI behaviour not shown).
- FFC/DFC images are created as U8, then read with `Use File Path Image Type` unwired (its default is **(unknown)**). The pixel type of the loaded correction images is therefore **(unknown)** [main VI frame d79, zoom of the Read node].
- `Simulation`, `Simulation Index` and `Simulation Path` are not modified.

### 4.3 Acquisition modes and their consequences

`IMAQdx Configure Acquisition` [Doc6 p.1179]: "Snap: Continuous = 0; Buffer Count = 1 / Sequence: Continuous = 0; Buffer Count > 1 / Grab: Continuous = 1; Buffer Count > 1".

- With `Continuous Mode = TRUE`, the acquisition is a **Grab** (continuous, 5-buffer ring). The streaming state machine only works in this mode.
- With `Continuous Mode = FALSE`, the 5 buffers make it a **Sequence**: 5 frames are acquired and then the acquisition stops (inferred from the NI text). After that `LastBufferNumber` no longer advances, and every `Cam Stream Next Image` ends with error 9009 once its timeout expires (inferred).
- With `Cam Start(Triggered = TRUE)`, frames arrive only on a trigger. The server never sets `TriggerSource` / `TriggerSelector`, so they come from the camera file or the camera's saved state **(unknown)**. With a software trigger the client must send `Cam Stream Next Image` first (which samples the buffer counter) and `Cam Software Trigger` afterwards. Otherwise the request times out with 9009 (inferred).

### 4.4 `Cam Start` [main VI frames d90–d95; p.11]

Payload: `Camera Start Data` {Camera Type, Triggered}.

```
d ← VariantToData(Command Data, Camera Start Data)
val ← d.Triggered ? "On" : "Off"
attr ← "CameraAttributes::AcquisitionControl::TriggerMode"
case d.Camera Type:
  "Low Mag.":
      plug ← Low Mag Settings.Camera Plug Type
      if plug is "GiGe"/Default: eT ← IMAQdx property(LM.Session): ActiveAttribute ← attr ; ValueString ← val
      else ("USB"):              eT ← no error
      eS ← IMAQdx Start Acquisition(LM.Session, error in = ClearErrors(eT))   # Start runs even if TriggerMode failed
      tag ← "LM"
  "High Mag.", Default:
      plug ← Camera Sort: LED            # ⚠ NOT High Mag Settings.Camera Plug Type (see §13)
      (same TriggerMode write / Start) ; tag ← "HM"
err ← MergeErrors(eS, eT)                # a Start error takes precedence; a TriggerMode error is still reported
if err: err.source ← err.source + "CamStart TriggerMode" + tag
Log Server Error(Extra "Start"); reply error cluster(err)
```

`Acquisition State` is not touched (it stays `Stopped` after `Cam Setup`).

### 4.5 `Cam Stop` [main VI frames d170–d174; p.23]

```
(running, e0) ← IMAQdx property(session).AcquisitionInProgress
if running: e ← IMAQdx Stop Acquisition(session, error in = e0)
else:       e ← no error                     # a failed read (running = default FALSE) is discarded
if e: e.source ← e.source + "Cam stop"       # lower-case "s"
Log Server Error(Extra "Stop"); reply error cluster(e)
```

The session registers are not modified: `Acquisition State` and `High/Low Acq. State` keep their values. There is no Unconfigure and no Close.

### 4.6 `Cam Cleanup` [main VI frames d175–d177; p.24]

Payload: `Camera Release Data` {Camera Type}.

```
{HI|LO} Inited ← FALSE                       # even if the close fails
e ← IMAQdx Unconfigure Acquisition(session)  # error in unwired
e ← IMAQdx Close Camera(session, error in = e)
IMAQ Dispose(SRC), IMAQ Dispose(FFC), IMAQ Dispose(DFC)      # errors ignored
session{HM|LM} ← constant session (typedef defaults (inferred): Acquisition State = Start, null images, ...)
if e: e.source ← e.source + "Cam Cleanup"
Log Server Error(Extra "Cleanup"); reply error cluster(e)
```

There is no explicit Stop. NI documents that `IMAQdx Close Camera` "Stops an acquisition in progress, releases resources … and closes" [Doc6 p.583].

### 4.7 `Cam Get Acquisition Status` [main VI frames d116–d118; p.19]

```
(inProgress, e) ← IMAQdx property(session).AcquisitionInProgress
e ← ClearErrors(e, specific code = -1074360316)      # comment "Invalid camera session"
if e: e.source ← e.source + "Cam Acquisition status"
Log Server Error(Extra "Get Status"); reply Boolean(inProgress, e)
```

For a camera that was never set up, the reply is `FALSE` with no error.

### 4.8 `Cam Software Trigger` [main VI frames d119–d123; p.19–20]

Comment: "Skip software trigger errror" (sic).

```
plug ← (Camera Type == "Low Mag." ? Low Mag Settings : High Mag Settings).Camera Plug Type
case plug:
  "USB":           nothing is sent to the camera ; txt ← ":TriggerSoftware:"
  "GiGe", Default: txt ← "CameraAttributes::AcquisitionControl::TriggerSoftware"
                   e ← IMAQdx property(session): ActiveAttribute ← txt ; ValueBool ← TRUE
                   e ← ClearErrors(e)                 # clears ALL errors
(decoration with txt: never reached) ; Log Server Error(Extra "Soft Trigger")
reply error cluster(ClearErrors(e))                   # the client always receives "no error"
```

### 4.9 Server shutdown

`Exit` forwards `Exit` to the Z loop and stops the camera loop. No IMAQdx call is made and no image is disposed [main VI frame d78]. Step 4 of the main sequence does not clean up either. The process exit releases everything implicitly (inferred). See [01-architecture.md](01-architecture.md).

---

## 5. GenICam attribute handling

### 5.1 Access pattern and value type selection

Every configurable attribute is written or read with one IMAQdx property node: `ActiveAttribute ← <name>`, then `Value<T>`. Each node's `error in` is unwired, so each node starts from "no error" [main VI frames d124–d165]. The value accessor is chosen by a string setting (`Exposure Type`, `Gain Type`, `Framerate Type`) through a case-sensitive string Case:

| Setting value | Accessor | Conversion |
|---|---|---|
| `"I64"` | `ValueI64` | DBL → I64: round half to even, saturate (inferred) |
| `"U32"` | `ValueU32` | DBL → U32: round half to even, saturate at 0 and 2³²−1 (inferred) |
| `"DBL"` **and any other string** (Default case, including `"dbl"` or empty) | `ValueDBL` | none |

Reads always convert to DBL before replying.

### 5.2 `Cam Set Exposure` [main VI frames d0 (visible frame), d124–d140; p.3, p.5]

Payload: `Camera Set Exposure Data` {Camera Type, Exposure: DBL}. Units are those of the attribute (µs for SFNC `ExposureTime`, inferred from the 0.95·10⁶/exp frame-rate formula).

```
S   ← (Camera Type == "High Mag.") ? High Mag Settings : Low Mag Settings
exp ← clamp(Exposure, lower = S.Exposure Lower Limit, upper = 2 000 000)   # a DBL constant 176 sits unwired next to it [d0]
e1  ← write(S.Camera Exposure Attribute, S.Exposure Type, exp)
if e1: e1.source ← S.Camera Exposure Attribute                 # source REPLACED, not appended
fr_calc ← clamp(0.95 × (1 / (exp / 1 000 000)), lower = 0.5, upper = S.Max Framerate)
s_fr    ← FormatIntoString(fr_calc)                            # default DBL format, system decimal separator (inferred)
case Camera Type:
  "High Mag.", Default:
      Max Framerate (indicator) ← fr_calc
      maxdbl ← IMAQdx property: ActiveAttribute ← S.Camera Framerate Attribute ; read MaxDBL   # error ignored
      MaxFR real (U32 indicator) ← maxdbl
      e2 ← write(S.Camera Framerate Attribute, S.Framerate Type, maxdbl)    # all 3 types write maxdbl
      (Clear Errors on e2 with unwired output: no effect)
  "Low Mag.":
      no MaxDBL read, no indicators
      if S.Framerate Type is "I64" or "U32": e2 ← write(S.Camera Framerate Attribute, type, fr_calc)
      else ("DBL"/Default): NOTHING is written       # the active Diagram Disable frame is empty [d137–d138]
if e2: e2.source ← S.Camera Framerate Attribute + s_fr          # replaced; no separator
err ← MergeErrors(e1, e2)
Log Server Error(Extra "Set Exposure"); reply error cluster(err)
```

| Behaviour | HM | LM |
|---|---|---|
| Frame-rate value written | camera maximum `MaxDBL` read after setting the exposure | computed `fr_calc` |
| `Framerate Type = "DBL"` (INI default) | written (`ValueDBL`) | **not written** |
| `Max Framerate` setting | never loaded from the INI (value **(unknown)**). It only affects the display and the error text. | INI `frameratemax_DBL`, default 24.6 |

With the default INI, LM therefore only writes the exposure (inferred). The behaviour of `In Range and Coerce` when `upper < lower` (HM with `Max Framerate = 0`) is **(unknown)**; it only affects the display and the error text. The server never sets vendor "enable" features such as `AcquisitionFrameRateEnable`. On cameras that need one, the frame-rate write may have no effect or fail (inferred).

### 5.3 `Cam Get Exposure` [main VI frames d155–d161; p.21]

```
(v, e) ← read(S.Camera Exposure Attribute, S.Exposure Type) → DBL      # value 0 on error (inferred)
if e: e.source ← e.source + S.Camera Exposure Attribute + S.Exposure Type
Log Server Error(Extra "Get Exposure"); reply DBL(v, e)
```

### 5.4 `Cam Set Gain` / `Cam Get Gain` [main VI frames d148–d154, d162–d168; p.21–22]

Set: payload `Camera Gain Data` {Camera Type, Gain: DBL}. The gain is **not clamped**. `e ← write(S.Camera Gain Attribute, S.Gain Type, Gain)`; on error `source += S.Camera Gain Attribute + S.Gain Type`; Extra "Set Gain"; reply error cluster.
Get: same as Get Exposure with the gain attribute and type; Extra "Get Gain"; reply DBL. The reply carries the decorated error [main VI frame d162].

The INI default `Gain Type` is `"I64"`, so by default the gain is rounded to an integer.

### 5.5 Limits summary

| Quantity | Lower | Upper | Source |
|---|---|---|---|
| Exposure written | `Exposure Lower Limit` (INI `LimitMinExposure_DBL`, default 177) | 2 000 000 (constant) | [main VI frame d0] |
| Computed frame rate | 0.5 | `Max Framerate` | [main VI frames d0, d132] |
| Gain | none | none | [main VI frame d148] |
| Writes outside the camera range | rejected by the camera/driver, and the error is returned | | (inferred) |

### 5.6 Trigger

`TriggerMode` (`Cam Start`) and `TriggerSoftware` (`Cam Software Trigger`) are written only on the GigE path (§4.4, §4.8). The USB path never touches triggering.

### 5.7 Temperature [main VI frames d104, d110, d115; d0]

- Every time a `Cam Stream Next Image` message is processed, including every re-enqueued poll (§6.2), the server reads `CameraAttributes::DeviceControl::DeviceTemperature` (`ValueDBL`) on the requested camera and writes `HM T° (°C)` or `LM T° (°C)`.
- On error, `source += "CameraAttributes::DeviceControl::DeviceTemperature"`, and **this error is wired to `error in` of the acquisition state machine**. Any temperature read failure therefore ends the request with state `Error` and an empty image (§6.3, outer error case).
- On every iteration of the camera loop (any command), outside the command Case, the current `HM T°` and `LM T°` values are appended to two unbounded arrays shown as `Temperature Graph` (row 0 = HM, row 1 = LM) [main VI frame d0].
- No command returns the temperature to the client.

---

## 6. Image acquisition

### 6.1 The acquisition paths

| Path | Who triggers it | IMAQdx call | Processing | Result |
|---|---|---|---|---|
| `Cam Stream Next Image` (§6.2–6.4) | client | state machine: polls `LastBufferNumber`, then `Get Image2` in mode `Buffer Number` with number 0 | flip → [flat-field] → custom data → clear overlay | image reply |
| `Cam Get Image` (§6.5) | client | `Get Image2` mode `Last New`, timeout default 5000 ms | flip only | front-panel `Image` display; **no reply** |
| Autofocus / refocus measure steps | internal | `Get Image2` mode `Next` | flip → [flat-field] → focus metric | see [06-autofocus-refocus.md](06-autofocus-refocus.md) |
| Camera-loop timeout frame | never (the dequeue timeout is −1) | would enqueue a constant labelled "Get Image Cmd" (probably `Cam Get Image` (inferred); the enum value is unreadable) for each camera whose `AcquisitionInProgress` is TRUE | — | dead code [main VI frames d217–d219; p.29] |
| `Cam Get Images Array` | client | none | — | case labelled "TO BE DEFINED", empty: no reply, no log, sessions unchanged [main VI frame d169; p.23] |

### 6.2 `Cam Stream Next Image` orchestration (main VI) [main VI frames d104–d115; p.17–18]

Payload: `Camera Stream Data` (§3.3). This is a **non-blocking poll by re-enqueueing**: as long as the image is not ready, the same message (same notifier, data and timestamps) is put back at the end of the camera queue, and other camera commands are processed in between. Exactly one reply is sent per request.

```
S ← session of sd.Camera Type ; R ← settings of sd.Camera Type
(T, eT) ← read DeviceTemperature on S.Session ; {HM|LM} T° ← T
if eT: eT.source ← eT.source + "CameraAttributes::DeviceControl::DeviceTemperature"
case S.Acquisition State:
  "Acquire":                                                     # "Acquiring Image..."
      (st, S, e) ← state_machine(sd, State In = "Acquire", S, R, error in = eT)
      case st:
        "Finished":  Image (display) ← S.SRC Image                 # "Image Acquired / Return it"
                     e2 ← reply Image(S.SRC Image, e, 3 request timestamps)
                     Log Server Error(Extra "Enqueue Image Data", error in = e2)
                     S.Acquisition State ← "Stopped"
        "Acquire":   e3 ← Enqueue(Cam Queue, same message, error in = e)   # "Image Not Ready / Re-Enqueue command"
                     Log Server Error(Extra "Enqueue Image Command", error in = e3)
                     S.Acquisition State ← "Acquire"
        "Error", Default: reply Image(null image, e, timestamps)    # "Acquisition Error / Return Immediately"
                     Log Server Error(Extra "Get Image Acquisition", error in = e)
                     S.Acquisition State ← "Stopped"
  "Stopped", Default (also Start, Finished, Error):              # "Acquisition stopped, start it"
      (st, S, e) ← state_machine(sd, State In = "Start", S, R, error in = eT)
      case st:
        "Acquire":   re-enqueue as above ; Log "Enqueue Image Command" ; S.Acquisition State ← "Acquire"
        "Error", Default: reply Image(null image, e) ; Log "Get Image Start" ; S.Acquisition State ← "Stopped"
{High|Low} Acq. State ← S.Acquisition State
```

| State before | State machine called with | Result | Action | State after | Reply |
|---|---|---|---|---|---|
| `Acquire` | `Acquire` | `Finished` | display + image reply | `Stopped` | image + error |
| `Acquire` | `Acquire` | `Acquire` | re-enqueue | `Acquire` | none yet |
| `Acquire` | `Acquire` | `Error`/other | — | `Stopped` | null image + error |
| anything else | `Start` | `Acquire` | re-enqueue | `Acquire` | none yet |
| anything else | `Start` | `Error`/other | — | `Stopped` | null image + error |

Notes:
- `Start` never returns `Finished`, so a request always takes at least two passes through the queue, in simulation as well.
- The 3 timestamps of the request are copied into the image reply. `Enqueue Send Image Response.vi` serialises `SRC Image` with `IMAQ Flatten Image to String` (Type = "Image and Vision Info", Compression = None, Quality = 1000, Process-Unique Name = FALSE) between two `Tick Count` samples (pre/post-flattening timestamps) [Doc6 p.571–573]. The format is NI-proprietary; see [02-tcp-protocol.md](02-tcp-protocol.md). The request's `error` is wired to the flatten node's `error in`, so on error the flatten does not run and the reply carries an empty image string plus the error (inferred from NI error semantics).
- Two pending requests for the **same** camera share `Acquisition State`. The second one advances the acquisition the first one started. Whichever sees `Finished` first gets the image, and the other then starts a new acquisition (inferred).
- On LM, the `Log Server Error` of the `Finished` case leaves `Camera Type` unwired, so the default `High Mag.` is used [main VI frame d113]. Every HM call also leaves it unwired, which correctly defaults to HM.

### 6.3 `alox_camera_server_get_image_state_machine.vi` [Doc6 p.552–558]

**Interface.**

| Inputs | Type / default | Outputs |
|---|---|---|
| Camera Stream Data | §3.3 | State Out (enum §3.4) |
| State In | enum §3.4, default `Start` | Camera Session out (§3.2) |
| Camera Session in | §3.2 | error out |
| Camera Settings | §3.1 | |
| error in (no error) | error cluster | |

The image is **not** an output. It is written in place into the IMAQ buffer `Camera Session.SRC Image`, and the caller reads it from there.

**Persistent state.** Inside a one-iteration `While` loop (constant TRUE wired to "Stop if True") there are four shift registers: `state` and `session` (initialised from the inputs) and **`t0`** and **`t0 Buffer Number`**, which are **uninitialised**. The last two are static variables of the VI, written only in `Start` and read in `Acquire`. The VI's reentrancy setting was not printed **(unknown)**. With the LabVIEW default (non-reentrant), both cameras and every caller share a single `t0` / `t0 Buffer Number` (inferred).

**Timing.** A `Wait (ms)` of 1 runs in parallel with the state code, so every call lasts at least 1 ms.

**Outer error case.** If `error in` has `status=TRUE`, the VI returns `State Out = "Error"`, `error out = error in`, and the session unchanged. Nothing else runs [Doc6 p.557].

**States** (inside `No Error`):

```
Start:                                                            [Doc6 p.556]
    first_call ← sd.Simulation AND NOT sess.Simulation            # label "First Call ?"
    sess.Simulation Index ← first_call ? 0 : sess.Simulation Index
    sess.Simulation Time  ← first_call ? TickCount() : sess.Simulation Time
    sess.Simulation ← sd.Simulation ; sess.Simulation Path ← sd.Simulation Path
    if not sd.Simulation:
        (last, err) ← IMAQdx property(sess.Session).LastBufferNumber   # error in unwired, error out wired
        t0 ← TickCount() ; t0_buf ← last
    else:
        t0 ← TickCount() ; t0_buf ← 0 ; err ← no error
    State Out ← err ? "Error" : "Acquire" ; error out ← err

Acquire, camera (sess.Simulation = FALSE):                         [Doc6 p.554–555]
    (last, count) ← IMAQdx property(sess.Session): LastBufferNumber, BufferCount   # errors IGNORED
    if last > t0_buf:                                              # "Buffer Available" (U32 compare)
        (img, bufnum, err) ← IMAQdx Get Image2(Session, Image In = SRC Image,
                                Buffer Number Mode = "Buffer Number",
                                Buffer Number In = UNWIRED (= 0), Timeout (ms) = 5000)
        if err: State Out ← "Error" ; error out ← err
        else:
            img ← flip(img, Settings.Horizontal Flip, Settings.Vertical Flip)           # §7.2
            if Settings.Flat Field: img ← FlatFieldCorrection(img, FFC Image, DFC Image,
                                                               factor = SGL(Settings.Flat Field Factor))   # §7.3
            for i in 0..6: write custom data / clear overlay                            # §7.5
            State Out ← err ? "Error" : "Finished"
    else:                                                          # "No new buffer available"
        elapsed ← (TickCount() − t0) mod 2^32
        msg ← "lastbuffer:" + str(last) + "/ buffercount:" + str(count) + "/ buffer t-1:" + str(t0_buf)
        if elapsed > sd.Acquisition Timeout (ms):
            State Out ← "Error" ; error out ← {TRUE, 9009, "get image error (timeout)" + msg}
        else:
            State Out ← "Acquire" ; error out ← {FALSE, 0, msg}    # "no error", but with a non-empty source

Acquire, simulation (sess.Simulation = TRUE):                      [Doc6 p.555]
    (img, idx, time, err) ← alox_camera_server_simulation(Timer (ms) = sd.Simulation Timer (ms),
                                Simulation Path = sd.Simulation Path, Image In = SRC Image,
                                Index In = sess.Simulation Index, Time In = sess.Simulation Time)   # §8
    sess.Simulation ← sd.Simulation ; sess.Simulation Path ← sd.Simulation Path
    sess.Simulation Index ← idx ; sess.Simulation Time ← time
    if Settings.Flat Field: img ← FlatFieldCorrection(... same wiring ...)
    State Out ← err ? "Error" : "Finished"          # no flip, no custom data, no overlay clear

Finished: State Out ← "Finished"      Error: State Out ← "Error"      Stopped: State Out ← "Stopped"
Default:  State Out ← State In        (all of these pass the session and the error through)
```

The `Get Image2` timeout is the constant 5000 ms, independent of `Settings.Timeout` and of `Acquisition Timeout (ms)`. `Get Image2` is only called once the counter shows a new buffer, so it normally returns at once (inferred).

The exact spacing in the 9009 text (whether a blank precedes "/") is **(unknown)**. The text on [Doc6 p.555] reads `lastbuffer:`, `/ buffercount:`, `/ buffer t-1:`. Numbers are formatted in decimal (inferred: `Format Into String` without a format string).

### 6.4 Calling contract

```
state ← "Start"
repeat: (state, session, err) ← step(stream_data, state, session, settings, err)
until state ∈ {Finished, Error, Stopped}
image ← session.SRC Image
```

The main VI does not loop in place. It performs one step per queue pass (§6.2). In camera mode, `Acquire` succeeds on the first poll at which the cumulative buffer counter exceeds the value sampled in `Start`. The request fails with 9009 once more than `Acquisition Timeout (ms)` has elapsed since `Start`, and that time includes the time spent waiting in the queue.

**Which frame is returned (important for fidelity).** `Get Image2` is called in `Buffer Number` mode with `Buffer Number In` unwired, which requests cumulative buffer **0**. NI documents an `OverwriteMode` attribute for buffers that were already overwritten ("get newest", "get oldest" or "fail"), and its default is reported to be "get newest" (inferred, NI documentation not part of the printouts). Consequences (inferred):
- In steady state, buffer 0 was overwritten long ago, so the newest frame is returned (≥ `t0_buf + 1`). This is the evident intent.
- During the first 5 frames after `Cam Start`, buffer 0 is still in the ring, so the **first frame ever acquired** is returned (stale) and the `buffer` custom-data key reads 0.
- The accepted frame `t0_buf + 1` may have started its exposure before the request arrived, for example before a Z move or an LED change.
- What `LastBufferNumber` returns before the first frame is **(unknown)**. If it is a sentinel such as 0xFFFFFFFF, the first request after `Cam Start` can only end in 9009.

### 6.5 `Cam Get Image` [main VI frames d98–d103; p.14]

Payload: `Camera Get Image Data` {Camera Type}.

```
if not session.Simulation:
    (img, e) ← IMAQdx Get Image2(Session, Image In = SRC Image, Buffer Number Mode = "Last New")  # timeout unwired: 5000 ms default [Doc6 p.896]
    Image (display) ← flip(img, H, V)                     # flip error unwired
    e ← ClearErrors(e, -1074360316)                       # "Invalid camera session"
    if e: e.source ← e.source + "Cam Get Image"
    error out (indicator) ← e
# simulation: nothing
```

No reply is sent to the client and nothing is logged. The command only refreshes the server's display. The semantics of `Last New` are **(unknown)**. The name suggests the newest buffer that has not already been returned, waiting up to the timeout if there is none (inferred). If that is right, this command can block the whole camera loop for up to 5 s.

---

## 7. Image processing chain

### 7.1 Order of operations

| Step | Camera path (state machine) | Simulation path | AF/RF path (chapter 06) | `Cam Get Image` |
|---|---|---|---|---|
| 1 | `Get Image2` into `SRC Image` | read PNG into `SRC Image`, convert to U8 (§8) | `Get Image2 Next` into `SRC Image` | `Get Image2 Last New` |
| 2 | flip (H/V from Settings) | — | flip | flip |
| 3 | flat-field if `Settings.Flat Field` | flat-field if `Settings.Flat Field` | flat-field if `Settings.Flat Field` | — |
| 4 | 6 custom-data keys + clear overlay | — | ROI overlay drawn, focus metric | — |

Every step works **in place** on the same IMAQ buffer, `SRC Image` (unwired `Image Dst`) [Doc6 p.554]. The flip is therefore applied **before** the flat-field correction, so the FFC/DFC reference images must be stored in the **post-flip** orientation.

### 7.2 Flip (`alox_vision_flip_image.vi`) [Doc6 p.928–933]

```
if error in: Image Dst Out ← Image Src (unchanged) ; error out ← error in
elif not (H or V): unchanged ; error out ← no error
elif H and V: IMAQ Symmetry(Type = "Central")
elif H:       IMAQ Symmetry(Type = "Horizontal")
else:         IMAQ Symmetry(Type = "Vertical")
```

| ALOX flags | IMAQ Symmetry type | Pixel effect (inferred NI semantics: "Horizontal" uses the horizontal axis as the mirror axis) | numpy |
|---|---|---|---|
| H only | Horizontal | **top ↔ bottom** | `img[::-1, :]` |
| V only | Vertical | left ↔ right | `img[:, ::-1]` |
| H and V | Central | 180° rotation | `img[::-1, ::-1]` |

The ALOX "Horizontal Flip" therefore probably produces an up-down flip, which is the opposite of the usual naming (inferred). Reproduce it as is and validate on a test image. The printed NI page only says "Transforms an image through its symmetry", with `Type of Symmetry` default `Horizontal` [Doc6 p.924–925].

**`Cam Flip` command** [main VI frames d178–d180; p.24]. Payload `Camera Flip Data` {Camera Type, Horizontal Flip, Vertical Flip} (type constant `{High Mag., F, F}`); here the `Variant To Data` error **is** wired.

```
(d, err) ← VariantToData(Command Data, {High Mag., FALSE, FALSE})
case d.Camera Type:  "Low Mag." → X = Low Mag Settings ;  "High Mag.", Default → X = High Mag Settings
X (local variable) ← Bundle(X, Horizontal Flip = d.Horizontal Flip, Vertical Flip = d.Vertical Flip)
if err: err.source ← err.source + "Cam Flip"
Log Server Error(Extra "Flip"); reply error cluster(err)
```

Nothing is sent to the camera: the flags only drive the software flip of later frames (stream, `Cam Get Image`, AF/RF). `Bundle` and the local-variable write have no error input, so a malformed payload still runs them with the defaults and clears both flags of the HM camera **(inferred)**. The flags are not saved to `alox.ini`.

### 7.3 Flat-field / dark-current correction

**Loading** (`Cam Setup` §4.2, and `Cam Set FlatField` [main VI frames d141–d147]):
- `Cam Set FlatField` {Camera Type, Enable, FFC Path, DFC Path, Correction Factor (DBL, typedef default 0)} copies the four values into `Settings.Flat Field`, `Flat Field Path`, `Dark Current Path` and `Flat Field Factor`. It then **recreates `SRC Image`** (IMAQ Create, same name, error unwired), and loads FFC and DFC under the same conditions as `Cam Setup`: `Enable AND file exists AND path ≠ empty`. Otherwise it uses null images with **no error**. On error `source += "Cam Set Field"`; Extra "Set FlatField"; reply error cluster. No image is disposed.
- If `Enable = TRUE` but a file is missing, the settings say `Flat Field = TRUE` while the session holds null images. Every later acquisition then fails inside the correction (inferred).

**Call** [Doc6 p.554–555]:

```
IMAQ Flat Field Correction(Image Src = image, Image Dst = unwired (in place),
                           Flat Field Image = session.FFC Image, Dark Field Image = session.DFC Image,
                           Correction Options = {Line Scan Image? = FALSE, Correction Factor = SGL(Settings.Flat Field Factor)},
                           ROI Descriptor = unwired (whole image), Optimize Correction? = unwired (default TRUE [Doc6 p.935]))
```

The printed NI page only says: "Corrects image intensity using flat field and dark field images and returns a corrected image. Recreate the flat field image whenever the imaging setup changes. Use the IMAQ Compute Average Image VI or the IMAQ Compute Median Image VI to create the image for the Flat Field Image input." Its defaults are `Line Scan Image? = False`, `Correction Factor = 1`, `Optimize Correction? = True`; it also has an unwired `Flat Field Image` output [Doc6 p.934–936].

**Formula (inferred).** The formula itself is not in the printouts. NI's online description is that the "Correction Factor is used to bias the mean computed from the flat field image; typically 1 to 1.5; increasing it brightens the corrected image" (inferred, external documentation). The most likely formula is:

```
corrected(x,y) = (I(x,y) − D(x,y)) × k × m / (F(x,y) − D(x,y)),   m = mean of (F − D) over the image,
result saturated to the source pixel type
```

The choice between mean and median, the handling of `F = D`, the rounding, and the effect of `Optimize Correction?` are **(unknown)**. They must be validated numerically against LabVIEW outputs (see §14). With this formula, `k = 0` produces a black image. The INI default is `k = 1`, but a client that sends `Cam Set FlatField` with the typedef default `Correction Factor = 0` would get black images (inferred). Images of different sizes or types make the NI VI fail, and the request then ends in `Error` (inferred).

### 7.4 Pixel types and casts

- Camera path: no explicit cast. `Get Image2` converts the destination image to the camera's video format [Doc6 p.895]. The FFC/DFC images must match it (inferred).
- Simulation path: every image is forced to U8 (§8.2).
- `Flat Field Factor` is coerced DBL → SGL (float32) [Doc6 p.554].

### 7.5 Custom data and overlay (camera path only) [Doc6 p.554]

The block sits in the active ("Enabled") frame of a Diagram Disable structure commented "Custom Datas + clear overlay". The disabled frame only passes the error through. A `For` loop with N = 7 chains the image and the error through shift registers, so an error skips the remaining iterations (inferred):

| i | Operation | Key | Value |
|---|---|---|---|
| 0 | IMAQ Write Custom Data | `buffer` | `Buffer Number Out` of Get Image2, formatted `%d` |
| 1 | IMAQ Write Custom Data | `timestamp` | current local time, `Format Date/Time String` `"%H:%M:%S:%3u"` |
| 2 | IMAQ Write Custom Data | `name` | `Settings.Camera Name` (INI `name_STR`) |
| 3 | IMAQ Write Custom Data | `type` | `"High-Mag"` if `Camera Type == 0`, otherwise `"Low-Mag"` |
| 4 | IMAQ Write Custom Data | `simulation` | `"FALSE"` (constant) |
| 5 | IMAQ Write Custom Data | `flatfield` | `"TRUE"` / `"FALSE"` from `Settings.Flat Field` |
| 6 | IMAQ Clear Overlay | — | `Groups` unwired: clears all overlays (for example the AF/RF ROI rectangles drawn on the same buffer) |

IMAQ Write Custom Data: "If the key does not exist, it is created. If it does exist, its current value is overwritten" [Doc6 p.544]. `%3u` is LabVIEW's fractional-seconds code with 3 digits. LabVIEW emits it with a leading decimal separator, so the text is probably `HH:MM:SS:.mmm`, or `HH:MM:SS:,mmm` with a French decimal separator (inferred; check on a real image). The keys travel with the image in the "Image and Vision Info" flattening (inferred), see [02-tcp-protocol.md](02-tcp-protocol.md). Simulated images get no keys. Whether the PNG read clears keys left over from earlier camera frames is **(unknown)**.

---

## 8. Simulation mode

### 8.1 Activation

Simulation is selected per request by `Camera Stream Data.Simulation = TRUE` and is copied into the session at `Start` (§6.3). On the transition camera → simulation (`first_call`), the index is reset to 0 and the time to now. Switching folders while staying in simulation keeps the index; the modulo brings it back into range at the next advance.

Prerequisites imposed by the main VI (inferred from the observed wiring):
- The temperature read on the camera session runs before every step, and its error is wired to the state machine (§5.7). Simulation therefore only works when the camera **has been set up and answers `DeviceTemperature`**.
- The PNG is read into `SRC Image`, which only exists after `Cam Setup` or `Cam Set FlatField`.

Only `Cam Stream Next Image` uses the simulator. `Cam Get Image` does nothing when the session is in simulation (§6.5), and autofocus/refocus always grab from the camera, because their simulation case is selected by a constant FALSE ([06-autofocus-refocus.md](06-autofocus-refocus.md)).

### 8.2 `alox_camera_server_simulation.vi` [Doc6 p.919–923]

| Input | Type / default | Output |
|---|---|---|
| Timer (ms) | U32, 0 | |
| Simulation Path | path (folder) | |
| Image In | IMAQ image | Image Out |
| Index In | U32, 0 | Index Out (U32) |
| Time In | U32 (ms), 0 | Time Out (U32) |
| error in | | error out |

All numeric terminals are U32 [Doc6 p.921]. The session field `Simulation Time` is not U32 (§3.2), so LabVIEW coerces it on the way in and out; its exact type, and hence the effect for tick values above 2³¹, is **(unknown)**.

```
if error in: pass everything through (Index Out = Index In, Time Out = Time In)
files ← List Folder(Simulation Path, "*.png")       # re-listed on EVERY call; order: List Folder's (alphabetical (inferred))
n ← len(files) ; now ← TickCount()
if now >= (Time In + Timer (ms)) mod 2^32:          # "≥" node; U32 addition; "Timer Elapsed => Next buffer"
    out ← Index In mod n ; next ← (out + 1) mod n ; next_t ← now
else:
    out ← Index In ; next ← Index In ; next_t ← Time In      # no modulo here
img ← IMAQ Read Image And Vision Info 2(BuildPath(Simulation Path, files[out]), Image In,
                                         Use File Path Image Type = TRUE)
case IMAQ GetImageInfo(img).Image Type:
    0 (Grayscale U8)              "Mono8":  unchanged
    1 (I16), 7 (U16)              "Mono16": IMAQ Cast Image(→ Grayscale (U8), # Shifts = 8)   # value >> 8, truncated [Doc6 p.911]
    4 (RGB U32)                   "RGB":    IMAQ ExtractSingleColorPlane(Color Plane = Luminance)
    Default                       "Not supported": error {TRUE, -9011, "Invalid Type"}
Image Out ← img ; Index Out ← next ; Time Out ← next_t
```

Behaviour (inferred unless stated):
- With `Timer = 0`, every request serves the next file, cycling round. Otherwise the same file is re-read until `Timer` ms have passed since the last advance. A file is always re-read from disk (observed: no cache).
- An empty folder makes the integer modulo by 0 return `x` in LabVIEW, the index falls out of range, `BuildPath` yields the folder itself, the read fails, and the request ends in `Error`. If files were removed and `Index In ≥ n` on the "timer not elapsed" branch, the read also fails.
- A 12-bit image stored in 16 bits becomes very dark (maximum 15) after the 8-bit shift.
- 16-bit-per-channel colour PNGs (NI RGB U64) end in -9011.
- `U32` overflow in `Time In + Timer` near the 49.7-day tick wrap is not handled.
- An isolated `Grayscale (U8)` constant on the diagram is not wired.

---

## 9. LED and bulb control through camera outputs

### 9.1 Configuration (start-up, [main VI frame d3]; see chapter 09)

| INI key | Default | Becomes |
|---|---|---|
| `[general] setLEDcam_I32` | 0 | 0 → the lights are driven by the HM camera: `Camera Sort: LED ← High Mag Settings.Camera Plug Type`, `Connector ← "HM"`. Any other value → LM plug type, `"LM"`. |
| `[hardware.camera.highmag] setLEDline_I32` | 0 | `HM setLEDline` (LED output line, USB path) |
| `[hardware.camera.highmag] setBULBline_I32` | 0 | `HM setBULBline` (bulb output line, USB path) |
| `[hardware.camera] lightStatusInverter_BOOL` | TRUE | `LEDStatusInverter` (front-panel switch) |
| `invertled_BOOL` (per camera) | HM FALSE / LM TRUE | `Settings.Invert LED` |
| `waitled_U32` (per camera) | 0 | `Settings.Wait LED (ms)` |
| — | saved front-panel value **(unknown)** (printed in the "down" position, presumably FALSE (inferred)) | `BulbStatusInverter` |

The diagram comments document the GenICam enumeration integers used: "Line Selector 0: Line 0 / 1: Line 1 … Line Source: 0: Off / 1: User Output 0 … UserOutputSelector: 0: User Output 0 / 1: User Output 1 … UserOutputValue: 0: Led Off / 1: Led On" [main VI p.26].

### 9.2 `Cam Set Led` [main VI frames d195–d201; p.26–27]

Payload: `Camera Set LED Data` {Camera Type, Led}. The `Variant To Data` error is wired into the I/O chain.

```
S    ← settings of Camera Type ; sess ← session of Camera Type        # chosen by the CLIENT's Camera Type
ledcmd ← S.Invert LED ? NOT Led : Led ;  LED cmd (indicator) ← ledcmd
n ← NOT ledcmd
Write Key(refnum, "hardware.camera", "lightStatusInverter_BOOL", value unwired)   # no effect (inferred): refnum already closed at start-up, nothing saved to disk
case Camera Sort: LED:                                               # bus path chosen by the INI, not by Camera Type
  "USB":  String ← "us"
      for (line, inv) in [(HM setLEDline, LEDStatusInverter), (HM setBULBline, BulbStatusInverter)]:   # LED chain, then bulb chain (error-chained)
          LineSelector ← line (U32) ; LineMode ← 1 ; LineSource ← 1
          LineInverter ← inv ? NOT n : n
      wait ← n
  "GiGe", Default:  String ← "gi"
      target ← LEDStatusInverter ? NOT n : n
      LineSelector ← 1 ; LineSource ← 1 ; UserOutputSelector ← 0
      prev ← UserOutputValue (read) ; UserOutputValue ← target (write)
      wait ← (prev ≠ target)
if wait: Wait(S.Wait LED (ms))                     # runs even if the I/O writes failed
eR ← reply error cluster(raw error)                # the reply is sent BEFORE the source decoration
if eR: eR.source ← eR.source + "Cam Set Led" ; Log Server Error(Extra "Set LED", eR)
```

Equivalent truth table (inferred from the pseudo-code):

| Path | Value written | Delay |
|---|---|---|
| USB | `LineInverter(LED line) = ledcmd XOR NOT LEDStatusInverter`; `LineInverter(bulb line) = ledcmd XOR NOT BulbStatusInverter` | when `ledcmd = FALSE` |
| GigE | `UserOutputValue = ledcmd XOR NOT LEDStatusInverter` on line 1 / user output 0 | when the output actually changes |

With the defaults (`LEDStatusInverter = TRUE`), the value written equals `ledcmd`. The line numbers are always the `HM …line` values, whichever camera is used. In the GigE frame, the wire from `target` to the write terminal is hidden behind a node. The write value is deduced as `target` with medium confidence [main VI frame d197].

### 9.3 `Cam Get Led` [main VI frames d202–d204; p.27]

```
USB:  LineSelector ← HM setLEDline ; LineMode ← 1 ; LineSource ← 0 (!) ; v ← LineInverter (read)
GiGe/Default: LineSelector ← 1 ; LineSource ← 1 ; UserOutputSelector ← 0 ; v ← UserOutputValue (read)
LEDStatus (indicator) ← v
reply Boolean(S.Invert LED ? NOT v : v, raw error) ; then decorate "Cam Get Led" ; Log "Get LED"
```

On USB, reading the state **writes `LineSource = 0` ("Off")**, while `Cam Set Led` writes 1 [main VI p.26–27]. A `Set` followed by a `Get` returns the original `Led` value only if `LEDStatusInverter = TRUE` (inferred).

---

## 10. Errors

### 10.1 Codes produced or filtered by the camera subsystem

| Code | Text / meaning | Origin | Handling |
|---|---|---|---|
| **9009** | `"get image error (timeout)" + "lastbuffer:<n>/ buffercount:<n>/ buffer t-1:<n>"` | state machine, no new buffer within `Acquisition Timeout (ms)` [Doc6 p.555] | returned with a null image |
| **-9011** | `"Invalid Type"` (unsupported PNG pixel type) | simulation VI [Doc6 p.921] | → state `Error` → null image. The printed constant is negative (**-9011**), not 9011. |
| **-1074360316** (0xBFF69004) | IMAQdx error, commented "Invalid camera session" | IMAQdx calls on a session that is not open | **cleared** by `Clear Errors` in `Cam Get Image`, `Cam Get Acquisition Status` and the dead timeout frame [main VI p.14, 17, 19, 29]. `Cam Stop` has no `Clear Errors`, but it discards the `AcquisitionInProgress` read error (any code) because the failed read returns FALSE (§4.5) [main VI p.23]. Returned everywhere else. |
| any IMAQdx / NI Vision code | driver text in `source` | Open, Configure, Start, Stop, Close, attribute access, Get Image2, Read Image, Flat Field, Custom Data | returned with the command suffix appended (error-cluster replies carrying an error are probably never delivered, §4.1) |
| (all) | — | `Cam Software Trigger` | all errors cleared; always replies "no error" |
| (status FALSE, code 0, non-empty source) | `"lastbuffer:…"` diagnostic | state machine while waiting | not an error; do not reproduce |

The full IMAQdx/NI Vision code list is not reproduced here; see the NI-IMAQdx and NI Vision error references.

### 10.2 Source decoration and log labels

Suffixes are appended to `error.source` with no separator, and only when `status = TRUE`. `Log Server Error` writes only on error ([09-configuration-logging-errors.md](09-configuration-logging-errors.md)).

| Command | Suffix appended to `source` | `Extra String` of the log |
|---|---|---|
| Cam Setup | `"Cam Setup"` + (LM: sources of the Configure, PacketSize and Open errors) | `Setup` |
| Cam Start | `"CamStart TriggerMode"` + `"HM"`/`"LM"` | `Start` |
| Cam Stop | `"Cam stop"` | `Stop` |
| Cam Cleanup | `"Cam Cleanup"` | `Cleanup` |
| Cam Get Acquisition Status | `"Cam Acquisition status"` | `Get Status` |
| Cam Software Trigger | attribute or `":TriggerSoftware:"` (never reached) | `Soft Trigger` |
| Cam Set Exposure | **replaced** by the exposure attribute; frame rate: **replaced** by framerate attribute + `fr_calc` text | `Set Exposure` |
| Cam Get Exposure / Set Gain / Get Gain | attribute + type string | `Get Exposure` / `Set Gain` / `Get Gain` |
| Cam Set FlatField | `"Cam Set Field"` | `Set FlatField` |
| Cam Flip | `"Cam Flip"` | `Flip` |
| Cam Set Led / Cam Get Led | `"Cam Set Led"` / `"Cam Get Led"` (after the reply was sent) | `Set LED` / `Get LED` |
| Cam Get Image | `"Cam Get Image"` (only in the `error out` indicator) | none |
| Cam Stream Next Image | temperature error: `"CameraAttributes::DeviceControl::DeviceTemperature"` | `Enqueue Image Command`, `Enqueue Image Data`, `Get Image Start`, `Get Image Acquisition` |

---

## 11. Concurrency and shared state

**LabVIEW execution model** (observed unless tagged):
- A single **camera consumer loop** executes every camera command one at a time, in FIFO order. The camera queue is bounded at 5 elements ([01-architecture.md](01-architecture.md)). Hardware access to both cameras is therefore serialised, and a slow command blocks everything else: `Cam Setup`, `Cam Get Image` (up to 5 s (inferred)), `Cam Set Led` (`Wait LED`), AF/RF steps.
- **Self re-enqueue** (`Cam Stream Next Image`, `Cam Autofocus Run`, `Cam Refocus Run`) uses `Enqueue Element` with an infinite timeout on the loop's own bounded queue. If the TCP producer has filled the remaining slots, the only consumer blocks on itself and the server freezes (inferred deadlock).
- Each poll costs one queue round-trip, a temperature read, at least 1 ms in the state machine, and one entry in each of the unbounded histories (`Array` of errors, `Temperature Graph`) [main VI frame d0].
- **Static state of the state machine VI** (`t0`, `t0 Buffer Number`) is shared by both cameras if the VI is non-reentrant **(unknown)**. If HM and LM streams interleave, one camera's `Start` overwrites the other's reference, so frames can be accepted too early or timeouts measured from the wrong instant (inferred). AF and RF also keep their state in static registers, so only one AF/RF can run at a time (chapter 06).
- **`SRC Image` per camera** is the single destination for streaming, `Cam Get Image` and AF/RF grabs. Everything runs on the same loop and the stream reply flattens the image in the same step that acquired it, so there is no data race in LabVIEW. The front-panel `Image` display shows whichever path wrote last.
- **Settings** are front-panel indicators accessed through local variables with no locking. After start-up only the camera loop writes them (`Cam Flip`, `Cam Set FlatField`) (inferred safe).

**Python design rules:**
1. Use one `CameraController` per camera. It owns the driver handle, a grab thread that keeps a ring of the last N frames with a cumulative counter (the equivalent of `LastBufferNumber`), its **own** state-machine instance (`t0`, `t0_buf`), and a lock around driver calls.
2. Keep one command dispatcher if strict FIFO ordering of client commands must be preserved, but **never block it on its own queue**. Re-enqueue with `put_nowait` into an unbounded internal deque, or turn pending stream requests into waiters on the controller's `threading.Condition` and reply from the grab thread.
3. Copy frames out of driver buffers (`.copy()`) before releasing the buffer. Serialise the reply image at reply time, as LabVIEW does.
4. Bound every history (`deque(maxlen=…)`) and timestamp the temperature samples.
5. Make the driver calls of `Cam Setup`, `Cam Cleanup` and shutdown idempotent: close an existing handle before re-opening it, and stop and close both cameras on `Exit`.

---

## 12. Python mapping

### 12.1 Library options

| Option | Buses | OS | Fit for this server |
|---|---|---|---|
| **Harvesters** (`harvesters.core`) + vendor **GenTL producer** (`.cti`) | GigE Vision, USB3 Vision (whatever the producer supports) | Windows, Linux | Vendor-neutral and closest to IMAQdx's attribute-by-name model (`ia.remote_device.node_map.<Feature>`). Recommended default. |
| **pypylon** | Basler GigE / USB3 | Windows, Linux | Use if the cameras are Basler. Features via `cam.<Feature>.Value`; enumerations by integer via `SetIntValue`. |
| **VmbPy** (Vimba X) | Allied Vision GigE / USB3 | Windows, Linux | Use if the cameras are Allied Vision. `cam.get_feature_by_name(...)`, `cam.start_streaming(handler, buffer_count=5)`. |
| **Aravis** (PyGObject, `gi.repository.Aravis`) | GigE Vision, USB3 Vision | mainly Linux | `Aravis.Camera.new(name)`, `set_float/get_float/get_float_bounds/set_integer/set_boolean/execute_command`. **No FireWire.** |
| **NI-IMAQdx C API** through `ctypes` | everything IMAQdx supports, including IIDC 1394 | Windows, with an NI Vision Acquisition Software runtime licence | Highest fidelity: same attribute names, `LastBufferNumber`, buffer modes and error codes. Keeps the NI dependency. |
| libdc1394 | IIDC 1394 | Linux / macOS | Only for FireWire. Python bindings are scarce (inferred). |

### 12.2 FireWire / IIDC risk

- The printed front panel shows `BusTypeHM = FireWire` [main VI p.2], and the default exposure/gain attribute names have a non-SFNC `<Feature>::Value` shape (§2.1). Both are weak hints that at least one camera may be, or once was, an IEEE 1394 (IIDC) camera (inferred): the panel values were not saved from a running camera (§2), and the attribute-name shape also fits other non-GenICam cameras. The server code itself only knows `GiGe` and `USB`.
- IIDC cameras are not GenICam/GenTL devices. Their features (Shutter, Gain, Trigger, …) are register-based with raw integer units, and they are not reachable through Harvesters or Aravis. Current vendor SDKs mostly dropped 1394 support (inferred; verify with the camera vendor).
- How the `GiGe` (Default) branch behaves on a 1394 camera (`PacketSize`, SFNC `TriggerMode`, `DigitalIOControl` lines) is **(unknown)**.
- Mitigation, in order of preference: (1) read the real bus type and model of both cameras (NI MAX or the `BusType` indicator); (2) if a camera is 1394, either keep NI-IMAQdx through its C API for that camera, use the vendor's legacy SDK, or replace the camera with a GigE/USB3 model; (3) put every attribute name in configuration with an explicit IMAQdx → backend translation table (§12.4).

### 12.3 IMAQdx → Python equivalents

| LabVIEW / IMAQdx | Python (Harvesters shown) |
|---|---|
| Open Camera(name, Controller) | `h = Harvester(); h.add_file(cti); h.update(); ia = h.create({"serial_number": sn})` (or `user_defined_name`). An NI MAX name such as `cam0` is an NI alias, not a device property, so map each name the client sends to a serial number in configuration (inferred) |
| camera file loaded by Open Camera | load a GenICam **UserSet** or apply an explicit feature list at open time; the NI MAX file contents must be captured first |
| Configure Acquisition(Continuous, 5) | `ia.num_buffers = 5` (plus the One Shot emulation, see the sketch) |
| Start / Stop Acquisition | `ia.start()` / `ia.stop()` plus a grab thread |
| Unconfigure + Close | `ia.destroy()`; `h.reset()` at shutdown |
| `LastBufferNumber` | cumulative counter maintained by the grab thread |
| Get Image2 (Buffer Number / Next / Last) | look up in the frame ring kept by the grab thread |
| `ActiveAttribute` + `Value<T>` | `getattr(nm, feature).value` (float or int); enumerations by integer via `set_int_value` (GenApi `IEnumeration::SetIntValue`) |
| `MaxDBL` | `getattr(nm, feature).max` |
| `ValueBool ← TRUE` on a command | `getattr(nm, feature).execute()` |
| `PacketSize` | `nm.GevSCPSPacketSize.value = 8996` (inferred feature name; GigE only) |
| IMAQ Write Custom Data | `dict` attached to the frame and serialised with the reply |
| IMAQ Clear Overlay | nothing (overlays are display-only in Python) |

### 12.4 Reference sketch: shared helpers and the frame ring

```python
# Reference sketch — camera subsystem helpers (pure Python / numpy).
from __future__ import annotations

import collections
import math
import threading
import time
from typing import Optional

import numpy as np

U32 = 0xFFFFFFFF


def tick_ms() -> int:
    """LabVIEW 'Tick Count (ms)': free-running U32 millisecond counter."""
    return int(time.monotonic() * 1000) & U32


class CamError(Exception):
    """Error with a LabVIEW-compatible code and source text (see chapter 09)."""

    def __init__(self, code: int, source: str):
        super().__init__(f"{code}: {source}")
        self.code = code
        self.source = source


def value_kind(type_str: str) -> str:
    """Case on the configured type string: case-sensitive, 'DBL' is also the Default case."""
    return type_str if type_str in ("I64", "U32") else "DBL"


def lv_to_int(x: float, kind: str) -> int:
    """LabVIEW DBL -> I64/U32 coercion: round half to even, then saturate (inferred)."""
    lo, hi = (0, U32) if kind == "U32" else (-(2 ** 63), 2 ** 63 - 1)
    if math.isnan(x):
        raise ValueError("NaN cannot be coerced")    # LabVIEW result: unknown
    if math.isinf(x):
        return hi if x > 0 else lo
    return min(max(round(x), lo), hi)                # Python round() = half to even


def genicam_name(imaqdx_attr: str, overrides: Optional[dict[str, str]] = None) -> str:
    """'CameraAttributes::AcquisitionControl::TriggerMode' -> 'TriggerMode'.
    IIDC-shaped names such as 'CameraAttributes::Gain::Value' have no automatic
    GenICam equivalent and must be mapped explicitly in configuration."""
    if overrides and imaqdx_attr in overrides:
        return overrides[imaqdx_attr]
    parts = imaqdx_attr.split("::")
    if len(parts) >= 2 and parts[-1] in ("Value", "Mode"):
        raise KeyError(f"no automatic GenICam name for {imaqdx_attr!r}; add an override")
    return parts[-1]


class FrameRing:
    """Emulates the IMAQdx buffer ring and LastBufferNumber of a continuous (Grab) acquisition."""

    def __init__(self, num_buffers: int = 5):
        self.num_buffers = num_buffers
        self._frames: collections.deque = collections.deque(maxlen=num_buffers)
        self._cond = threading.Condition()
        self._next_number = 0

    def push(self, frame: np.ndarray) -> None:
        """Called by the grab thread for every frame (cumulative numbering starts at 0)."""
        with self._cond:
            self._frames.append((self._next_number, frame))
            self._next_number = (self._next_number + 1) & U32
            self._cond.notify_all()

    @property
    def last_buffer_number(self) -> Optional[int]:
        """None before the first frame (IMAQdx value in that case: unknown)."""
        with self._cond:
            return self._frames[-1][0] if self._frames else None

    def get(self, number: Optional[int] = None) -> tuple[int, np.ndarray]:
        """number=None -> newest. A number no longer in the ring -> newest
        (IMAQdx OverwriteMode 'get newest', reported default: inferred)."""
        with self._cond:
            if not self._frames:
                raise CamError(-1, "no buffer acquired yet")  # real IMAQdx code: unknown
            if number is not None:
                for n, frame in self._frames:
                    if n == number:
                        return n, frame
            return self._frames[-1]
```

### 12.5 Reference sketch: GenTL backend with Harvesters

```python
# Reference sketch — one camera through a GenTL producer (harvesters >= 1.4).
# Verify the exact harvesters/genicam signatures against the installed versions.
# Uses FrameRing, genicam_name, value_kind and lv_to_int from §12.4.
from __future__ import annotations

import threading

from harvesters.core import Harvester


class HarvestersCamera:
    def __init__(self, harvester: Harvester, search_key: dict, overrides: dict | None = None):
        self._ia = harvester.create(search_key)            # ~ IMAQdx Open Camera (exclusive)
        self.nm = self._ia.remote_device.node_map
        self.overrides = overrides or {}
        self.ring = FrameRing(5)
        self._grabbing = False
        self._max_frames = None
        self._thread = None

    # --- lifecycle -------------------------------------------------------
    def configure(self, continuous: bool, num_buffers: int = 5) -> None:
        self._ia.num_buffers = num_buffers
        self.ring = FrameRing(num_buffers)
        # IMAQdx "One Shot" with N > 1 buffers = Sequence: N frames then stop (inferred)
        self._max_frames = None if continuous else num_buffers

    def start(self) -> None:
        self._ia.start()
        self._grabbing = True
        self._thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._thread.start()

    def _grab_loop(self) -> None:
        count = 0
        while self._grabbing and (self._max_frames is None or count < self._max_frames):
            try:
                with self._ia.fetch(timeout=0.5) as buf:    # raises on timeout
                    comp = buf.payload.components[0]
                    frame = comp.data.reshape(comp.height, comp.width).copy()  # mono formats only
            except Exception:
                continue                                    # timeout: poll again while grabbing
            self.ring.push(frame)
            count += 1

    @property
    def acquisition_in_progress(self) -> bool:
        return self._grabbing

    def stop(self) -> None:
        self._grabbing = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        self._ia.stop()

    def close(self) -> None:
        if self._grabbing:
            self.stop()
        self._ia.destroy()

    # --- attributes --------------------------------------------------------
    def _node(self, attr: str):
        return getattr(self.nm, genicam_name(attr, self.overrides))

    def write(self, attr: str, type_str: str, value: float) -> None:
        # IMAQdx probably converts between ValueDBL/I64/U32 and the attribute type (inferred);
        # a GenICam IInteger node rejects a float, so the type string must match the node type.
        kind = value_kind(type_str)
        self._node(attr).value = float(value) if kind == "DBL" else lv_to_int(value, kind)

    def read(self, attr: str, type_str: str) -> float:
        return float(self._node(attr).value)               # type_str matters only to IMAQdx

    def read_max(self, attr: str) -> float:
        return float(self._node(attr).max)

    def write_enum_int(self, attr: str, value: int) -> None:
        self._node(attr).set_int_value(int(value))         # GenApi IEnumeration::SetIntValue

    def write_bool(self, attr: str, value: bool) -> None:
        self._node(attr).value = bool(value)

    def read_bool(self, attr: str) -> bool:
        return bool(self._node(attr).value)

    def execute(self, attr: str) -> None:
        self._node(attr).execute()
```

Driver exceptions must be translated into `CamError` with the codes the existing client expects. Which IMAQdx codes the client tests for is an open question (§14).

### 12.6 Reference sketch: image processing and simulation

```python
# Reference sketch — flip, flat-field (inferred formula), simulation, custom-data timestamp.
# Uses CamError, U32 and tick_ms from §12.4.
from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np


def flip_image(img: np.ndarray, horizontal: bool, vertical: bool) -> np.ndarray:
    """alox_vision_flip_image.vi / IMAQ Symmetry (inferred NI semantics)."""
    if horizontal and vertical:
        return img[::-1, ::-1]      # "Central"
    if horizontal:
        return img[::-1, :]         # "Horizontal": mirror about the horizontal axis (top <-> bottom)
    if vertical:
        return img[:, ::-1]         # "Vertical": left <-> right
    return img


def flat_field_correct(img, flat, dark, factor: float) -> np.ndarray:
    """(inferred) (I - D) * k * mean(F - D) / (F - D), saturated to the source type.
    Validate against LabVIEW output before use (mean vs median, F == D, rounding)."""
    if flat is None or dark is None:
        raise CamError(-1, "flat field: FFC/DFC image not loaded")      # NI code unknown
    if flat.shape != img.shape or dark.shape != img.shape:
        raise CamError(-1, "flat field: image size mismatch")           # NI code unknown
    k = float(np.float32(factor))                                       # DBL -> SGL coercion
    i, f, d = (a.astype(np.float64) for a in (img, flat, dark))
    den = f - d
    m = den.mean()
    out = np.zeros_like(i)
    np.divide((i - d) * (k * m), den, out=out, where=den != 0)          # F == D: behaviour unknown
    info = np.iinfo(img.dtype)
    return np.clip(np.rint(out), info.min, info.max).astype(img.dtype)


def to_u8_like_simulation(img: np.ndarray) -> np.ndarray:
    """Type handling of alox_camera_server_simulation.vi."""
    if img.ndim == 2 and img.dtype == np.uint8:
        return img                                                     # Mono8
    if img.ndim == 2 and img.dtype in (np.uint16, np.int16):
        return (img.astype(np.uint16) >> 8).astype(np.uint8)           # Mono16: 8 shifts, truncate
    if img.ndim == 3 and img.dtype == np.uint8:
        code = cv2.COLOR_BGRA2GRAY if img.shape[2] == 4 else cv2.COLOR_BGR2GRAY
        return cv2.cvtColor(img, code)                                 # RGB -> Luminance (inferred weights)
    raise CamError(-9011, "Invalid Type")


def simulate_next_image(folder: str, timer_ms: int, index_in: int, time_in: int):
    """One call of alox_camera_server_simulation.vi -> (image, index_out, time_out)."""
    files = sorted((p.name for p in Path(folder).iterdir()
                    if p.is_file() and p.suffix.lower() == ".png"), key=str.lower)  # order (inferred)
    n = len(files)
    now = tick_ms()
    if now >= ((time_in + timer_ms) & U32):          # ">=" and U32 add (wraps), as in LabVIEW
        out_idx = index_in % n if n else index_in    # LabVIEW: x mod 0 = x
        next_idx = (out_idx + 1) % n if n else out_idx + 1
        next_time = now
    else:
        out_idx, next_idx, next_time = index_in, index_in, time_in
    if not 0 <= out_idx < n:
        raise CamError(-1, f"no PNG at index {out_idx} in {folder}")   # LabVIEW: file error, code unknown
    img = cv2.imread(str(Path(folder) / files[out_idx]), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise CamError(-1, f"cannot read {files[out_idx]}")
    return to_u8_like_simulation(img), next_idx, next_time


def lv_timestamp_text(now: float | None = None, decimal_sep: str = ".") -> str:
    """Custom-data 'timestamp' = LabVIEW '%H:%M:%S:%3u' in local time.
    (inferred) %3u emits the decimal separator followed by 3 digits; check on a real image."""
    t = time.time() if now is None else now
    ms = int((t % 1.0) * 1000)
    return time.strftime("%H:%M:%S", time.localtime(t)) + ":" + decimal_sep + f"{ms:03d}"
```

### 12.7 Reference sketch: per-camera acquisition state machine

```python
# Reference sketch — alox_camera_server_get_image_state_machine.vi, one instance PER CAMERA.
# Uses tick_ms, U32, CamError (§12.4) and flip_image, flat_field_correct, simulate_next_image,
# lv_timestamp_text (§12.6). session.camera is the camera object (None when not set up).
import time
from enum import Enum

INVALID_SESSION = -1074360316                  # IMAQdx "Invalid camera session"


class AcqState(str, Enum):
    START = "Start"
    ACQUIRE = "Acquire"
    FINISHED = "Finished"
    ERROR = "Error"
    STOPPED = "Stopped"


class GetImageStateMachine:
    def __init__(self, faithful_buffer0: bool = False):
        self.t0 = 0
        self.t0_buf = -1                       # -1: buffer 0 counts as new (design choice)
        self.faithful_buffer0 = faithful_buffer0

    def step(self, state, stream, session, settings, error=None):
        """Returns (state_out, error_or_None); mutates `session` (src_image, custom_data, simulation fields)."""
        if error is not None:                  # outer Case on 'error in'
            return AcqState.ERROR, error
        time.sleep(0.001)                      # Wait (ms) = 1
        if state == AcqState.START:
            return self._start(stream, session)
        if state == AcqState.ACQUIRE:
            return self._acquire(stream, session, settings)
        return state, None                     # Finished / Error / Stopped / Default: unchanged

    def _start(self, stream, session):
        if stream.simulation and not session.simulation:          # "First Call ?"
            session.simulation_index = 0
            session.simulation_time = tick_ms()
        session.simulation = stream.simulation
        session.simulation_path = stream.simulation_path
        self.t0 = tick_ms()
        if stream.simulation:
            self.t0_buf = 0
            return AcqState.ACQUIRE, None
        if session.camera is None:                 # LastBufferNumber read fails; this error IS wired
            return AcqState.ERROR, CamError(INVALID_SESSION, "LastBufferNumber")  # real source text: unknown
        last = session.camera.ring.last_buffer_number
        self.t0_buf = -1 if last is None else last
        return AcqState.ACQUIRE, None

    def _acquire(self, stream, session, settings):
        try:
            if session.simulation:
                session.simulation = stream.simulation
                session.simulation_path = stream.simulation_path
                img, session.simulation_index, session.simulation_time = simulate_next_image(
                    stream.simulation_path, stream.simulation_timer_ms,
                    session.simulation_index, session.simulation_time)
                if settings.flat_field:
                    img = flat_field_correct(img, session.ffc_image, session.dfc_image,
                                             settings.flat_field_factor)
                session.src_image = img                # no flip, no custom data in simulation
                return AcqState.FINISHED, None
            ring = None if session.camera is None else session.camera.ring
            last = None if ring is None else ring.last_buffer_number   # read errors are IGNORED here
            if last is None or not last > self.t0_buf:                 # no new buffer yet
                elapsed = (tick_ms() - self.t0) & U32
                if elapsed > stream.acquisition_timeout_ms:
                    count = 0 if ring is None else ring.num_buffers    # IMAQdx BufferCount: meaning unknown
                    msg = (f"lastbuffer:{0 if last is None else last}"
                           f"/ buffercount:{count}/ buffer t-1:{self.t0_buf & U32}")
                    return AcqState.ERROR, CamError(9009, "get image error (timeout)" + msg)
                return AcqState.ACQUIRE, None
            number, frame = ring.get(0 if self.faithful_buffer0 else None)
            frame = flip_image(frame, settings.horizontal_flip, settings.vertical_flip)
            if settings.flat_field:
                frame = flat_field_correct(frame, session.ffc_image, session.dfc_image,
                                           settings.flat_field_factor)
            session.src_image = frame
            session.custom_data = {
                "buffer": str(number),
                "timestamp": lv_timestamp_text(),
                "name": settings.camera_name,
                "type": "High-Mag" if stream.camera_type == 0 else "Low-Mag",   # enum value, High Mag. = 0
                "simulation": "FALSE",
                "flatfield": "TRUE" if settings.flat_field else "FALSE",
            }
            return AcqState.FINISHED, None
        except CamError as e:
            return AcqState.ERROR, e
```

`faithful_buffer0=True` reproduces the LabVIEW request for buffer 0 (§6.4). The default returns the newest frame, which is the evident intent. `ring.get(None)` always returns a frame numbered `> t0_buf` here, because the check just before it guarantees one exists.

### 12.8 Reference sketch: `Cam Set Led`

`Cam Set Exposure`, `Cam Get Led` and the other commands translate line by line from the pseudo-code of §4, §5 and §9. `Cam Set Led` is shown because its error and wait semantics are easy to get wrong.

```python
# Reference sketch — Cam Set Led, faithful to [main VI frames d195–d201]. Returns the raw error or None.
import time

DIO = "CameraAttributes::DigitalIOControl::"


def cam_set_led(cam, s, led: bool, camera_sort_led: str, hm_led_line: int, hm_bulb_line: int,
                led_status_inverter: bool, bulb_status_inverter: bool, payload_error=None):
    ledcmd = (not led) if s.invert_led else led          # Led XOR Invert LED
    n = not ledcmd
    err = payload_error                                   # the Variant To Data error feeds the I/O chain
    if camera_sort_led == "USB":
        if err is None:
            try:
                for line, inverter in ((hm_led_line, led_status_inverter),
                                       (hm_bulb_line, bulb_status_inverter)):
                    cam.write_enum_int(DIO + "LineSelector", line)
                    cam.write_enum_int(DIO + "LineMode", 1)
                    cam.write_enum_int(DIO + "LineSource", 1)
                    cam.write_bool(DIO + "LineInverter", ledcmd if inverter else n)
            except CamError as e:
                err = e                                   # the rest of the error chain is skipped
        wait = n
    else:                                                 # "GiGe" and Default
        target = ledcmd if led_status_inverter else n
        previous = False                                  # value returned by a failed property read (inferred)
        if err is None:
            try:
                cam.write_enum_int(DIO + "LineSelector", 1)
                cam.write_enum_int(DIO + "LineSource", 1)
                cam.write_enum_int(DIO + "UserOutputSelector", 0)
                previous = cam.read_bool(DIO + "UserOutputValue")
                cam.write_bool(DIO + "UserOutputValue", target)
            except CamError as e:
                err = e
        wait = previous != target
    if wait:                                              # Wait (ms) has no error input: it runs even after an error
        time.sleep(s.wait_led_ms / 1000.0)
    return err                                            # replied BEFORE the "Cam Set Led" suffix is appended
```

---

## 13. Defects and oddities in the LabVIEW implementation

| # | Behaviour | Evidence | Impact | Recommendation |
|---|---|---|---|---|
| 1 | `Get Image2` in `Buffer Number` mode with `Buffer Number In` unwired requests buffer 0. | [Doc6 p.554] | Stale first frame during the first 5 frames after `Cam Start` (inferred) | **Fix** (return newest > `t0_buf`); keep a fidelity flag; **ask** whether the client relies on `buffer` |
| 2 | Freshness: frame `t0_buf + 1` is accepted although its exposure may predate the request. | [Doc6 p.554] | Images taken before a Z move or LED change (inferred) | **Ask** (wait for `t0_buf + 2` or use the trigger) |
| 3 | `t0` / `t0 Buffer Number` are static registers shared by both cameras (if non-reentrant). | [Doc6 p.554] | Interleaved HM/LM streams corrupt each other (inferred) | **Fix** (per-camera state) |
| 4 | The temperature read error feeds the state machine's `error in`. | [main VI frames d104, d110] | A camera without `DeviceTemperature`, or a transient read error, fails every stream request | **Ask**; probably **fix** (log the temperature error, do not fail the image) |
| 5 | Simulation needs a set-up, real camera (temperature read, `SRC Image`). | [main VI frames d79, d104], §8.1 | Simulation unusable without hardware (inferred) | **Fix** (hardware-free simulation) unless the client depends on the failure |
| 6 | Self re-enqueue with infinite timeout into a bounded queue; polling through the queue. | [main VI frames d106, d107, d111] | Possible deadlock; CPU; unbounded histories grow per poll | **Fix** (non-blocking wait) |
| 7 | `Cam Get Image` and `Cam Get Images Array` ("TO BE DEFINED", empty) send no reply; `Last New` with a 5 s default timeout can block the loop. | [main VI frames d98, d169] | Client waits until its own timeout | **Ask** (reply with an error/ack, or drop) |
| 8 | `Number of buffers` (client) and `Settings.Buffers` / `Settings.Timeout` are ignored; the ring is fixed at 5. | [main VI frame d79] | Documented constant | **Reproduce** (log a warning if a different value is requested) |
| 9 | `Cam Setup` re-opens without closing the previous session; `Cam Set FlatField` re-creates `SRC Image` without disposing. | [main VI frames d79, d141] | Resource leaks or "already open" errors (inferred) | **Fix** |
| 10 | `Exit` does not stop or close cameras. | [main VI frame d78] | Relies on process exit | **Fix** (clean shutdown) |
| 11 | `Cam Stop` does not update `Acquisition State` and discards an `AcquisitionInProgress` read error. | [main VI frame d170] | A stream after `Cam Stop` waits for the timeout and then returns 9009 | **Reproduce** the reply; set the state in Python |
| 12 | `Cam Start` on HM chooses GigE/USB from `Camera Sort: LED` instead of HM's own plug type. | [main VI frame d91] | Wrong trigger path when the lights are on LM and the buses differ | **Fix**; **ask** for confirmation |
| 13 | `Cam Software Trigger` does nothing on USB and always replies "no error". | [main VI frame d119] | Silent failure | **Ask** |
| 14 | `Cam Set Exposure` is asymmetric: HM writes `MaxDBL`; LM writes the computed rate only for I64/U32; HM `Max Framerate` is never loaded. | [main VI frames d0, d132, d137–d138, d5] | Different frame-rate behaviour per camera | **Reproduce** by default; **ask** for the intent |
| 15 | Exposure/frame-rate errors **replace** `source` instead of appending. | [main VI frame d0] | Loses the driver text | **Reproduce** (client-visible) or **ask** |
| 16 | Gain is not clamped. | [main VI frame d148] | The driver rejects out-of-range values | **Reproduce** |
| 17 | `Cam Set FlatField` with `Enable` and a missing file returns no error, yet every later acquisition fails in the correction. | [main VI frames d141–d146] | Delayed, confusing errors (inferred) | **Fix** (error at set time) |
| 18 | Flat-field factor 0 (typedef default of `Cam Set FlatField`) gives a black image with the inferred formula. | [Doc6 p.365–368 via typedef], §7.3 | Wrong images | **Ask** / validate |
| 19 | Flip happens before flat-field, so FFC/DFC must be in flipped orientation; "Horizontal Flip" probably flips top/bottom. | [Doc6 p.554, p.930] | Orientation surprises | **Reproduce**; validate on a test image |
| 20 | Timestamp format `"%H:%M:%S:%3u"` probably yields `HH:MM:SS:.mmm`. | [Doc6 p.554] | Odd string | **Reproduce**; verify |
| 21 | Status FALSE with a non-empty `source` while waiting. | [Doc6 p.555] | A log that prints `source` without testing `status` shows it | **Do not reproduce** |
| 22 | Simulation: no flip, no custom data, folder re-listed on each request, 16-bit images shifted by 8. | [Doc6 p.555, p.921] | Simulated images differ from camera images | **Reproduce** (option to apply the flip) |
| 23 | `Cam Get Led` (USB) writes `LineSource = 0`. | [main VI frame d202; p.27] | Reading changes the line configuration | **Ask** |
| 24 | LED line numbers always come from the HM section; the bus path comes from `Camera Sort: LED` but the session from the client's `Camera Type`. | [main VI frames d195, d202] | Wrong camera or attributes if the client names the other camera | **Ask** |
| 25 | `Write Key(lightStatusInverter_BOOL)` with an unwired value, on the configuration refnum that start-up already closed. | [main VI frames d195, d61] | No effect (inferred) | **Do not reproduce** |
| 26 | LM `Finished` logs with `Camera Type` unwired (HM prefix). | [main VI frame d113] | Wrong log prefix | **Fix** |
| 27 | Attribute read errors in `Acquire` are ignored (`LastBufferNumber`, `BufferCount`), whereas the same read in `Start` is checked. | [Doc6 p.554, p.556] | A session that breaks after `Start` while `DeviceTemperature` still answers is reported as 9009 after the timeout (inferred); otherwise the temperature read fails first | **Fix** (report the driver error) or **reproduce** the code only |
| 28 | Dead code: camera-loop timeout frame (auto `Cam Get Image`), Diagram Disable structures whose active frame is empty (before the command Case, and in `Cam Setup`). | [main VI frames d83, d88, d215–d219] | None | **Do not reproduce** |
| 29 | `Variant To Data` errors are unwired in most camera commands, so a mismatched payload runs on HM defaults. | [main VI frames d79, d90, d98, d104, d141, d148] | Wrong camera silently | **Fix** (reject), see [03-command-reference.md](03-command-reference.md) |
| 30 | `Cam Cleanup` sets `Inited = FALSE` even when the close fails; there is no explicit Stop. | [main VI frame d175] | Minor | **Reproduce** |
| 31 | Error-cluster replies that carry an error are probably never queued (the reply VI's `Enqueue Element` receives the same `error in`). | [Doc6 p.437], §4.1 | Camera failures (`Cam Setup`, `Cam Start`, `Set Exposure`…) reach the client as timeouts | **Ask** (capture first); see [02-tcp-protocol.md](02-tcp-protocol.md) |
| 32 | `Cam Flip` writes the flags even when the payload cannot be decoded, so a bad payload clears the HM flip flags (inferred). | [main VI frame d178] | Silent orientation change | **Fix** (reject the payload) |

---

## 14. Open questions

These are limited to questions that block or change the re-implementation.

1. **Camera hardware.** What are the exact models, buses (GigE / USB3 / FireWire) and vendors of the HM and LM cameras? `BusTypeHM` prints `FireWire` on a panel that was not saved from a running camera. The answer decides the Python library (§12.1–12.2).
2. **Deployed configuration.** The real `alox.ini` camera sections are needed: attribute names (`exposure_STR`, `gain_STR`, `framerate_STR`), type strings, `cameraconnectortype_STR`, flat-field paths and factor, LED lines, `setLEDcam_I32`. So are the NI MAX camera files loaded by `IMAQdx Open Camera` (pixel format, ROI, trigger source and selector, line configuration, buffer `OverwriteMode`).
3. **Pixel format and bit depth** of the acquired images (U8 or U16). This drives the flat-field saturation, the image serialisation and the FFC/DFC file formats.
4. **IMAQdx buffer semantics**, confirmed on the installed driver: `LastBufferNumber` before the first frame, the default `OverwriteMode`, the meaning of `Last New`, and the meaning of `BufferCount`.
5. **Reentrancy** of `alox_camera_server_get_image_state_machine.vi` (VI properties not printed), which determines whether HM and LM share `t0`.
6. **Flat-field formula.** A validation set is needed: raw frame, FFC, DFC, factor, and the LabVIEW-corrected output.
7. **Client dependencies.** Does the client read the custom-data keys (in particular `buffer` and `timestamp`)? Does it rely on getting the oldest buffer right after `Cam Start`? Does it call `Cam Get Image` and expect no reply? Which IMAQdx error codes does it test? Does it rely on getting **no** reply when a camera command fails (§4.1)? A capture of one failing `Cam Setup` or `Cam Start` answers the last point.
8. **Simulation without hardware.** Is it required? The LabVIEW server needs a set-up camera that answers `DeviceTemperature`.
9. **LED/bulb wiring and intent.** Which physical lines drive the LED and the bulb? What are the intended semantics of `LEDStatusInverter` / `BulbStatusInverter` / `Invert LED`? Is the `LineSource = 0` write in `Cam Get Led` intended? Should `Wait LED` apply when the light is switched off?
10. **Software trigger on USB.** Is it expected to work? Should `Cam Start` on HM use HM's plug type?
11. **HM `Max Framerate`.** What value does the `Camera Settings` constant hold (it is never read from the INI)?
