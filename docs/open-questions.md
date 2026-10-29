# Open questions — questionnaire

These questions came out of the analysis of the LabVIEW printouts of `alox_camera_server.vi` and its sub-VIs.
Some of the answers decide how the new server is designed; the others only refine details.

## How to fill this in

- **Tick a box:** replace `[ ]` with `[x]`.
- **Write your answer** on the `Answer:` line. Any language is fine and short answers are fine.
- **Files** (screenshots, `.ini` files, captures): add them to the `docs/answers/` folder, or send them in the chat, and
  write the file name on the `Answer:` line.
- **Skipping:** leave anything you don't know empty. Write `?` if you'd like it discussed.
- **Editing on GitHub:** open this file, click the pencil icon, edit, then click *Commit changes*.

Priority: **P1** blocks or changes the design; **P2** is a detail that won't stop the work.

---

## A. Client compatibility and wire protocol

### Q1 (P1): Will the client stay unchanged?

The client is presumably the LabVIEW "ALOX Vision Core", which uses notifiers and "synchronous calls". If it stays as it is,
the new server must reproduce LabVIEW's binary format exactly. That means flattened clusters, flattened variants with type
descriptors, and NI's proprietary image format (`IMAQ Flatten Image to String`).

- [ ] The client stays unchanged, so the new server must be byte-compatible.
- [ ] The client can be modified, so we may switch to a simpler protocol (e.g. JSON/msgpack plus raw or PNG images).
- [ ] The client will be rewritten too.

What language is the client written in? Answer:

### Q2 (P1): Ordered enum items and their representation

The printouts never show complete enum lists. The numbers sent over TCP depend on the order of the items. The routing of
commands also depends on it: the server sends `"Z Setup".."Z Do Reference"` to the Z loop and `"Get Heartbeat".."Cam Get Led"`
plus `"Log"` to the camera loop, by **range**. With the wrong order, some Z commands would never be executed.

How to get it: in LabVIEW, open each `.ctl`, right-click the enum, choose **Edit Items…** and take a screenshot. Then choose
**Properties → Data Type** and note U8, U16 or U32.

| Enum (file) | Screenshot file name or list of items | Representation (U8/U16/U32) |
|---|---|---|
| `typedef_ext_camera_camera_command.ctl` (**most important**) | | |
| `HW TCP Datatype.ctl` | | |
| `HW TCP Server Command.ctl` | | |
| `alox_typedef_camera_type.ctl` | | |
| Camera Plug Type (in the camera properties typedef) | | |
| `typedef_ext_camera_camera_command_type_LED.ctl` | | |
| `alox_typedef_autofocus_method.ctl` | | |
| `alox_typedef_autofocus_speed.ctl` | | |
| `typedef_ext_camera_autofocus_state.ctl` | | |
| `alox_typedef_refocus_method.ctl` | | |
| Z Do Reference enum (in `typedef_ext_z_*` / main VI) | | |

### Q3 (P1): Field order of `typedef_ext_camera_init_data.ctl` (`Cam Setup` payload)

The front panel shows *Camera Type, Camera Name, Number of buffers, Continuous Mode*. The main VI's Unbundle suggests
*Camera Type, Continuous Mode, Number of buffers, Camera Name*. The order matters for decoding the bytes.

How to get it: right-click the cluster border, choose **Reorder Controls in Cluster…**, and take a screenshot.

Answer:

### Q4 (P1): Network capture of a real session

A Wireshark capture (`.pcapng`) of the real client talking to the current LabVIEW server would confirm in one go the enum values,
the variant format, the heartbeat period, the image format, and whether error replies are sent. Ideally it would include:
heartbeat, `Cam Setup`, `Cam Set Exposure`, one image request, one Z move (and one deliberately out of range), and one autofocus.

- [ ] I can provide a capture (file name below).
- [ ] Not possible.

Answer:

### Q5 (P2): Does the client use the metadata embedded in images?

The server attaches custom data to each image: `buffer`, `timestamp`, `name`, `type`, `simulation` and `flatfield`.

- [ ] Yes. Which keys?
- [ ] No.
- [ ] Don't know.

Answer:

### Q6 (P1): Will the new server run on Windows with the NI Vision Runtime installed?

If it does, the server can call NI's image-flatten function and send byte-identical image replies. If not, the NI image format
has to be reverse-engineered, or the client changed.

- [ ] Yes, NI Vision Runtime can stay installed.
- [ ] No, NI must be removed completely.

Answer:

---

## B. Hardware

### Q7 (P1): Exact camera models

Front panel shows `BusTypeHM = FireWire`; settings mention GiGe and USB. A FireWire camera would be a significant risk for a modern
software stack. The answer decides which camera library we use: Harvesters/GenTL, pypylon, VmbPy or Aravis.

| | Brand | Model | Interface (GigE / USB3 / FireWire) | Pixel format (Mono8 / Mono16 / other) |
|---|---|---|---|---|
| High Mag camera | | | | |
| Low Mag camera | | | | |

### Q8 (P1): LED and bulb wiring

Which camera output line drives the LED, and which drives the bulb? Is the signal inverted
(`LEDStatusInverter` / `BulbStatusInverter`)?

| Light | Camera (HM / LM) | Output line | Active high or low |
|---|---|---|---|
| LED | | | |
| Bulb | | | |

### Q9 (P1): PI controller and Z stage

| Item | Answer |
|---|---|
| Reply of `*IDN?` (controller model, e.g. C-863.11 / C-663) | |
| Connection: native RS-232 or USB (virtual COM port)? | |
| Real baud rate (INI default is 115200) | |
| COM port number | |
| Z stage model | |
| Reference switch, or limit switches only? | |

### Q10 (P2): Controller macros `DH` and `HOME`

The server restarts macro `DH` automatically (in Get Status, Stop and Cleanup). What do the macros contain? You can read them with
`MAC? DH` and `MAC? HOME`, for example in PI MikroMove.

- [ ] Keep the automatic restart of `DH`.
- [ ] Drop it.

Content of `DH`:

Content of `HOME`:

### Q11 (P2): `Z Setup` referencing

`Z Setup` references the axis with FNL, which drives the stage to its negative limit.

- [ ] Fine every time `Z Setup` is called (current behaviour).
- [ ] Only reference when the axis is not referenced yet.

Answer:

---

## C. Configuration and deployment

### Q12 (P1): Real configuration files

Please attach the files actually in use. If they contain anything confidential, remove those values first.

| File | Attached as | Where it lives on the machine |
|---|---|---|
| `alox_setup.ini` (next to the .exe) | | |
| `alox.ini` (settings folder, default `D:\alox\settings`) | | |
| Folder where the server .exe is installed | | |

### Q13 (P2): Target operating system

- [ ] Windows only.
- [ ] Windows and Linux.

Answer:

### Q14 (P1): Language and deployment constraints

The analysis will recommend **Python**: NumPy/OpenCV for image processing, the official PI library (`pipython`), and the camera
vendor SDKs.

- [ ] Python is fine.
- [ ] Preference for another language (which, and why).
- [ ] Must be delivered as a single packaged `.exe`.
- [ ] Must run as a Windows service (no window).

Team skills / constraints:

### Q15 (P2): User interface

The LabVIEW front panel shows the live image, focus graph, temperatures, command history and errors. The server window is made
invisible when running as an `.exe`.

- [ ] No GUI needed: headless service plus log files is enough.
- [ ] A small diagnostic GUI or web page would be useful.
- [ ] Need an equivalent of the full front panel.

Answer:

---

## D. Reproduce or fix? (LabVIEW behaviour that looks wrong)

For each line, tick one column. If you're unsure, leave it blank and the recommended default will be used.

| # | Behaviour found in the LabVIEW code | Recommended default | Reproduce | Fix | Ask author |
|---|---|---|---|---|---|
| Q16 | `Cam Set Exposure` sets the frame rate to the sensor **maximum**. The computed value (0.95 / exposure) is only displayed. | Reproduce and ask | [ ] | [ ] | [ ] |
| Q17 | When a command fails, the error reply is probably **never sent**, because the error is passed to the queue function, which then does nothing. The client times out instead. | Fix (always reply) | [ ] | [ ] | [ ] |
| Q18 | `Log Server Error` writes at the **beginning** of the log file, overwriting it. HI/LO labels are swapped in `Log Server Infos`. | Fix | [ ] | [ ] | [ ] |
| Q19 | Refocus **ignores "Refocus Method"**: the "Oxyd" branch is never used. If Oxyd is needed, the OXYD sub-VIs must be exported (they're not in the PDFs). | Keep "Fin" only | [ ] | [ ] | [ ] |
| Q20 | Refocus with joystick active goes to Error and then **sets speed 0 and moves to Z = 0 mm**. | Fix (restore only values actually read) | [ ] | [ ] | [ ] |
| Q21 | The Low Mag refocus region of interest is **not centred**, unlike High Mag. | Ask | [ ] | [ ] | [ ] |
| Q22 | Autofocus has no timeout in slow mode, makes its last move one step beyond Zmax, and returns only the [Z, score] table, **without computing the best Z**. | Reproduce and add a timeout | [ ] | [ ] | [ ] |
| Q23 | `Cam Set Led` contains a write of `lightStatusInverter_BOOL` to the INI file **that has no effect**. | Ask (was saving intended?) | [ ] | [ ] | [ ] |
| Q24 | Before each move, the Z axis is **silently re-initialised** if not referenced. MOV/MVR don't wait; `Z Wait End Move` does. | Reproduce | [ ] | [ ] | [ ] |
| Q25 | Get Image may return a frame whose exposure **started before the request**. | Fix (first frame strictly after the request) | [ ] | [ ] | [ ] |
| Q26 | Refocus PNG names look like `…_refocus__mesa` with a space-padded index. | Reproduce if other tools read these names | [ ] | [ ] | [ ] |
| Q27 | Focus metrics: the FFT score includes the DC term; FULL mode has a race condition; the 20×20 low-pass may have no effect. | Reproduce the maths, fix the race | [ ] | [ ] | [ ] |
| Q28 | Several self re-enqueues into queues limited to 5 elements can **deadlock** the server under load. | Fix | [ ] | [ ] | [ ] |

Q22: where is the best Z chosen today? In the client, or elsewhere? Answer:

Q21 / Q23 / anything else to tell the author:

---

## E. Reference data for validation

### Q29 (P1): Access to LabVIEW 2018 + NI Vision

A machine with LabVIEW 2018 + Vision would let us export reference outputs (resampling, edge detection, FFT, flat-field)
from test images, so the new code can be compared number by number.

- [ ] Yes, a LabVIEW 2018 + Vision machine is available.
- [ ] No, but I can provide sample images (raw + flat-field + dark-current) with the LabVIEW results.
- [ ] Neither.

Answer:

### Q30 (P2): Small details visible only in LabVIEW

| Item | How to check | Answer |
|---|---|---|
| Default value of input `FULL` in `alox_vision_measure_focus_sobel.vi` (probably False) | Open the VI, look at the control | |
| Is `alox_camera_server_get_image_state_machine.vi` reentrant? | VI Properties → Execution | |
| Are `alox_vision_measure_focus_fft.vi` / `_sobel.vi` reentrant? | VI Properties → Execution | |
| Is "Automatic error handling" enabled in `alox_camera_server.vi`? | VI Properties → Execution | |
| Value of `Max Framerate` in the `Camera Settings` constant (High Mag); it is not read from the INI | Open the constant on the diagram, stage 2 | |

---

## Anything else?

Answer:
