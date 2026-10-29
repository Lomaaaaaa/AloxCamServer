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

If time is short, start with **Q1**, **Q4** (network capture) and **Q31** (client code): together they settle many of the
other questions.

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
| `HW TCP Server Command.ctl` (internal, never sent on the network: low priority) | | |
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

The other command clusters are asked in Q33.

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

`Z Setup` may reference the axis with FNL, which drives the stage to its negative limit. Whether it does so every time or only
when the axis is not yet referenced depends on hidden options of PI's set-up VI (see Q41 and Q35).

- [ ] Reference every time `Z Setup` is called.
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
Exception: for behaviour the client can see (Q16, Q17, Q24, Q25), a blank line keeps the LabVIEW behaviour. The fix is switched
on only after the network capture (Q4) or your answer shows that the client does not depend on it.

| # | Behaviour found in the LabVIEW code | Recommended default | Reproduce | Fix | Ask author |
|---|---|---|---|---|---|
| Q16 | On the **High Mag** camera, `Cam Set Exposure` sets the frame rate to the camera **maximum**; the computed value (0.95 / exposure) is only displayed. On Low Mag the computed value is written only if the INI frame-rate type is I64 or U32; with the default (DBL) nothing is written. | Reproduce and ask | [ ] | [ ] | [ ] |
| Q17 | When a command fails, the error reply is probably **never sent**, because the error is passed to the queue function, which then does nothing. The client times out instead. | Reproduce until the capture (Q4) confirms; then fix (always reply) | [ ] | [ ] | [ ] |
| Q18 | `Log Server Error` writes at the **beginning** of the log file, overwriting it. HI/LO labels are swapped in `Log Server Infos`. | Fix | [ ] | [ ] | [ ] |
| Q19 | Refocus **ignores "Refocus Method"**: the "Oxyd" branch is never used. If Oxyd is needed, the OXYD sub-VIs must be exported (they're not in the PDFs). | Keep "Fin" only | [ ] | [ ] | [ ] |
| Q20 | Refocus with the joystick active stops with an error. Today nothing else happens, but the error-recovery step that was meant to run would **set speed 0 and move to Z = 0 mm**. This would become real as soon as the "job stops silently after an error" defect is fixed. | Fix (restore only values actually read) | [ ] | [ ] | [ ] |
| Q21 | The Low Mag refocus region of interest is **not centred**, unlike High Mag. | Ask | [ ] | [ ] | [ ] |
| Q22 | Autofocus has no timeout in slow mode, makes its last move one step beyond Zmax, and returns only the [Z, score] table, **without computing the best Z**. | Reproduce and add a timeout | [ ] | [ ] | [ ] |
| Q23 | `Cam Set Led` contains a write of `lightStatusInverter_BOOL` to the INI file **that has no effect**. | Ask (was saving intended?) | [ ] | [ ] | [ ] |
| Q24 | Before each move, the Z axis is **silently re-initialised** if not referenced. MOV/MVR don't wait; `Z Wait End Move` does. | Reproduce | [ ] | [ ] | [ ] |
| Q25 | `Cam Stream Next Image` (the image request) may return a frame whose exposure **started before the request**. | Reproduce until confirmed; then fix (first frame strictly after the request) | [ ] | [ ] | [ ] |
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

More items of this kind are in Q41.

---

## F. Additional questions (from the detailed chapters)

These questions were added after the detailed chapters were written. Q1–Q30 keep their numbers.

### Q31 (P1): Can you share the client code (the part of ALOX Vision Core that talks to the camera server)?

The new server must behave exactly as the client expects. Many open points are decided only inside the client: how long it
waits for each reply, whether it sends several commands at once, what it does when a reply never comes, which commands it
really uses, whether it checks error codes or texts, and how it decides that an autofocus is finished. One look at the client
answers Q32, Q37 and part of Q39 without guesswork. It also lets us test the new server against the real calling code.

How to get it: best, zip the client VIs that send commands and wait for replies, and put the zip in `docs/answers/`. These are
the "synchronous calls" folder (`D:\ALOX\Vision Core\sources\synchronous calls\`) and the VIs that use it. Or print them as a
VI documentation PDF, the same way the server was printed (**File → Print → VI documentation**, including block diagrams and
hidden frames).

- [ ] Zip of the client VIs attached (file name below).
- [ ] PDF printout attached (file name below).
- [ ] The client is not written in LabVIEW: language below, matching source files attached.
- [ ] The code is confidential: I answer Q32 by hand instead.

Answer:

*Background: [02](02-tcp-protocol.md) §10, §14; [01](01-architecture.md) §12; [03](03-command-reference.md) §10;
[05](05-camera-acquisition.md) §14; [06](06-autofocus-refocus.md) §11.*

### Q32 (P1): How does the client call the server: several commands at once, timeouts, missing replies, timestamps?

The LabVIEW server reads a new command only about every 50 ms. It can lock up when many requests pile up (Q28), and it probably
sends no reply when a command fails (Q17). Whether this matters depends on how the client calls it: one command at a time and
wait, or several in parallel (for example progress polls while images are requested). The client's timeouts tell us how fast
the new server must answer and how a missing reply is noticed. If the client compares the timestamps in the replies with its
own clock (Tick Count), the new server must use LabVIEW's exact millisecond counter.

How to get it: from the client code (Q31) or from the developer's knowledge. The network capture (Q4) also shows (a) and (f).
Answer yes / no / don't know, and add details where asked.

| # | Question | Answer |
|---|---|---|
| (a) | Does the client ever send a new command before the reply to the previous one has arrived, for example from two loops or VIs at the same time? | |
| (b) | How long does it wait for a reply? One value, or one per command (in ms)? | |
| (c) | What does it do when no reply arrives: error message, retry, reconnect, carry on? | |
| (d) | After a Z move, does it call `Z Get Error` or `Z Get Status` to check that the move was accepted? | |
| (e) | Does it use the timestamps contained in the replies, for example to compute delays against its own Tick Count? | |
| (f) | Does it ever close and reopen the connection while running? | |

*Background: [01](01-architecture.md) §12; [02](02-tcp-protocol.md) §14; [08](08-z-stage.md) §15;
[06](06-autofocus-refocus.md) §11.*

### Q33 (P1): Field order and number types of the other command clusters (autofocus, refocus, Z wait, joystick)

This is the same problem as Q3, for the other commands. On the network a command is just its fields, one after the other in the
cluster's internal order. Each whole number can be 4 bytes (I32/U32) or another size. The printouts show the panels but neither
the internal order nor the integer types. If one of them is wrong, autofocus, refocus, `Z Wait End Move` and the joystick
commands are decoded with wrong values, and the bytes alone cannot reveal the mistake.

How to get it: open each file below in LabVIEW. Right-click the cluster border, choose **Reorder Controls in Cluster…** and
take a screenshot. Then, for each numeric control listed, right-click → **Representation** and note the type (I32, U32, I16,
DBL…). Put the screenshots in `docs/answers/`. If a LabVIEW 2018 machine is available (Q29), the "golden vector" export
described in [04](04-data-types.md) §7.3 answers this too.

| Typedef (file) | Numeric controls to check | Screenshot of the order (file name) | Types of the numeric controls |
|---|---|---|---|
| `typedef_ext_camera_autofocus_data.ctl` | Duration (s), ROI, Subsampling, Acquisition Timeout (ms) | | |
| `typedef_ext_camera_refocus_data.ctl` | Steps, Fin ROI Width/Height, Oxyd X/Y/Diam, Acquisition Timeout (ms), Wait (ms) | | |
| `typedef_ext_z_move_wait_data.ctl` | Add. Wait (ms), Polling Cycle (ms) | | |
| `typedef_ext_z_joystick_data.ctl` | Joystick ID, Joystick Axis | | |

*Background: [04](04-data-types.md) §5.5, §7.2, §11; [06](06-autofocus-refocus.md) §11; [08](08-z-stage.md) §15;
[03](03-command-reference.md) §10.*

### Q34 (P1): Camera configuration stored in NI MAX (camera files and buffer settings)

The LabVIEW server sets only exposure, gain, frame rate and trigger mode. Everything else comes from the camera files that
NI MAX loads when a camera is opened: pixel format (8 or 16 bit), image size, trigger source, the output lines used for the LED
and the bulb, network packet size, and how old frames in the buffer are overwritten. The new server will not use NI MAX, so
these settings must be copied into its own configuration. Otherwise images, triggers or lights may behave differently.

How to get it: on the lab PC open **NI MAX → Devices and Interfaces** and select each camera (High Mag and Low Mag). Take
screenshots of the **Acquisition Attributes** and **Camera Attributes** tabs, with every tree expanded and every page shown. If
you can find the saved camera files (`*.icd`, usually in a public `NI-IMAQdx\Data` folder), copy them to `docs/answers/`.

| | Screenshots (file names) | Camera file `*.icd` (file name) |
|---|---|---|
| High Mag camera | | |
| Low Mag camera | | |

NI-IMAQdx version (shown under **Software** in NI MAX). Answer:

*Background: [05](05-camera-acquisition.md) §6.4, §14; [10](10-migration-plan.md) §8 (R9).*

### Q35 (P1): Short test session on the Z stage (stop, joystick, position readings, relative moves, set-up)

Some reactions of the PI controller cannot be read from the printouts, and they decide whether the new server is safe and
correct. Does the stop command the server sends (`STP 1`) really stop the stage? What happens to a move while the joystick is
on? Is the position read during a move the real position or the target? Does a relative move add to the target or to the
current position? Does `Z Setup` drive the stage to its limit switch every time? The answers change `Z Stop`,
`Z Move Relative`, `Z Setup` and the settling logic of autofocus/refocus.

How to get it: with nothing fragile on the stage, open the command entry / terminal window of PIMikroMove for the axis used by
the server, and run the tests below. Write the answers in the table, or save a PIMikroMove log in `docs/answers/`.

| # | Test | What to note | Answer |
|---|---|---|---|
| 1 | Start a long move (`MOV 1 <far position>`) and send `STP 1`. | Does the stage stop? What does `ERR?` return afterwards? | |
| 2 | During a slow move, send `POS? 1` several times. | Do the values change (real position) or stay at the target? | |
| 3 | Start a slow move and immediately send `MVR 1 0.1`. | Is the final position the old target + 0.1, or the position at that moment + 0.1? | |
| 4 | Switch the joystick on (`JON 1 1`), send `MOV 1 <position>`, then `ERR?`. Then send `JON 1 0`. | Does the stage move? Which error number is returned? | |
| 5 | Note the speed with `VEL? 1`. Send `VEL 1 0.0000`, then `ERR?`, then restore the normal speed. | Which error number is returned? | |
| 6 | With the LabVIEW server and its client, run `Z Setup` twice in a row. | Does the stage drive to its limit switch each time? | |
| 7 | With the LabVIEW server and its client, run `Z Get Serial` with Type = FALSE. | What text is returned? | |

*Background: [08](08-z-stage.md) §13.6, §14 (Z5, Z7, Z21), §15; [06](06-autofocus-refocus.md) §7.1, §11;
[03](03-command-reference.md) §7.12, §10.*

### Q36 (P2): Network and installation: same PC, several server copies, shared `alox.ini`

The LabVIEW server accepts connections only from the same PC ("localhost"). The new server keeps that unless remote clients are
needed, because opening it to the network exposes an unprotected control port. The INI key `core_I32` suggests several copies
of the server may run on one PC (one per "core"), each needing its own port and settings, while today all of them write into the
same daily log file. The new server also needs a few new settings (the compatibility switches). If `alox.ini` is shared with
other ALOX programs, we will put those settings in a separate file and never touch `alox.ini`.

| # | Question | Answer |
|---|---|---|
| (a) | Is the client always on the same PC as the server? If not, from which PC does it connect? | [ ] yes [ ] no, from: |
| (b) | How many copies of the camera server run on one PC? For each: which port, and which `alox_setup.ini` / `alox.ini`? | |
| (c) | Do other ALOX programs read or write `alox.ini`? | [ ] yes, which: [ ] no [ ] don't know |
| (d) | May we add a new file next to `alox.ini` for the new settings (proposed name `alox_camera_server.ini`)? | [ ] yes [ ] no, put it in: |

*Background: [01](01-architecture.md) §12 (A7); [02](02-tcp-protocol.md) §11.3, §13 (D16);
[09](09-configuration-logging-errors.md) §12 (D14).*

### Q37 (P2): Does the client use error codes or error texts?

When a command fails, the reply (when one is sent, see Q17) carries an error number and a text. Several commands build that
text inconsistently, and some NI error numbers cannot be produced without the NI drivers. If the client only checks
"error yes/no", the new server can tidy this up. If it tests particular numbers (for example 9405 "Z move out of range", 9009
image timeout, 9031 joystick enabled) or shows or parses the text, they must stay identical. For new error situations we propose
numbers 9500–9599.

How to get it: from the client code (Q31) or from experience.

| # | Question | Answer |
|---|---|---|
| (a) | Does the client react to particular error numbers? Which ones? | [ ] no, only "error yes/no" [ ] yes, these: [ ] don't know |
| (b) | Does it show the error text to the user, or analyse it? | [ ] shows it [ ] analyses it [ ] neither [ ] don't know |
| (c) | Does any ALOX program already use error numbers 9500–9599? | [ ] yes, which: [ ] no [ ] don't know |

*Background: [03](03-command-reference.md) §9 (C7), §10; [05](05-camera-acquisition.md) §14;
[09](09-configuration-logging-errors.md) §12.*

### Q38 (P2): Do other programs read the files the server writes (event logs, refocus images)?

The server writes daily event logs (`…_camera_events.log`, `…_z_events.log`). During refocus it also saves PNG images with names
like `…_refocus__mesa` and NI-specific embedded information. If another tool reads these files, their names, folders and line
format must stay exactly the same, including the decimal separator, and so must the PNG information. Otherwise the new server
can fix the known log defects (Q18) and write standard PNG files.

How to get it: list any program, script or person that reads these files, and what they read. Put one example of each log file
and one refocus PNG, with its exact file name, in `docs/answers/`.

- [ ] Nobody and nothing reads these files automatically.

| File | Read by (program, script or person) | What is read | Example attached (file name) |
|---|---|---|---|
| Camera event log `…_camera_events.log` | | | |
| Z event log `…_z_events.log` | | | |
| Refocus PNG images | | | |

*Background: [09](09-configuration-logging-errors.md) §12; [06](06-autofocus-refocus.md) §11 (D17); Q18 and Q26 above.*

### Q39 (P2): Autofocus and refocus in daily use

Focus jobs run in the background, and the client sees only a progress value and the [Z, score] table. Several choices for the
new server depend on how they are used. Which methods matter (the edge filters, "Sigma" and "FULL" are the hardest to reproduce
exactly)? Must the scores equal LabVIEW's to the last digit, or only find the same best position? What should happen when a job
fails, or when a second job is started while one is running?

| # | Question | Answer |
|---|---|---|
| (a) | Which `Autofocus Method` values and which speeds (Slow / Fast) are used in production? | |
| (b) | Must the focus scores match LabVIEW's exactly (for example because they are compared with old data or with thresholds), or is finding the same best position enough? | [ ] exactly [ ] same best position is enough |
| (c) | How does the client know that a job is finished or has failed: does it wait for progress = 100, or use a timeout? | |
| (d) | After a failure, should the stage go back to where it started? | [ ] yes [ ] no |
| (e) | If a second autofocus or refocus is started while one is running, should the server refuse it with an error, or stop the first and start the new one? | [ ] refuse [ ] stop the first, start the new one |
| (f) | If the client can be changed: would a new "status / abort" command be useful? | [ ] yes [ ] no |

*Background: [06](06-autofocus-refocus.md) §11 (D1, D2, D4, D5, D8); [07](07-focus-metrics.md) §12.*

### Q40 (P2): Camera behaviours to confirm

A few camera behaviours look unintended. The new server will copy them unless you say otherwise.

| # | Question | Answer |
|---|---|---|
| (a) | Is a simulation mode that works with no camera at all needed, including autofocus/refocus on recorded images? | [ ] yes [ ] no |
| (b) | Should `Cam Software Trigger` also work with USB cameras? Today it does nothing there and still reports success. | [ ] yes [ ] no, keep as today |
| (c) | For `Cam Start` on the High Mag camera, the trigger path is chosen from the camera that drives the lights (`setLEDcam_I32`), not from the High Mag camera's own connection. Is that intended? | [ ] yes, intended [ ] no, use the High Mag camera's own connection |
| (d) | If the camera temperature cannot be read, should image requests fail, as they do today? | [ ] yes, as today [ ] no, return the image anyway |
| (e) | `Cam Get Led` on a USB camera writes `LineSource = 0`. Is that intended? | [ ] yes [ ] no |
| (f) | Should the "Wait LED" delay also apply when the light is switched off? | [ ] yes [ ] no |

*Background: [05](05-camera-acquisition.md) §13 (#4, #5, #12, #13, #23), §14; [06](06-autofocus-refocus.md) §11 (D14).*

### Q41 (P2): More details visible only in LabVIEW (complement to Q30)

These values were not printed, but they change details of the new server: what happens at start-up, whether the Z set-up moves
the stage, and how the focus jobs recover from errors. Fill in the table, using screenshots where easier.

| # | Item | How to check | Answer |
|---|---|---|---|
| 1 | Saved default of the front-panel control `Z Session` in `alox_camera_server.vi` | Open the VI without running it and take a screenshot of the control | |
| 2 | What is the start-up check in stage 2 for? It restarts macro `DH` when `DH` or `HOME` is already running, the reverse of `Z Get Status`. | Author's knowledge | |
| 3 | Saved default of the `BulbStatusInverter` switch on the main VI's front panel | Open the VI without running it | |
| 4 | Content of the `AF Session` and `Refocus Session` constants used in the `Start` state of `alox_camera_server_autofocus.vi` and `alox_camera_server_refocus.vi` | Right-click the constant, untick **View As Icon**, take a screenshot | |
| 5 | "Automatic error handling" of `alox_camera_server_autofocus.vi`, `alox_camera_server_refocus.vi` and `Log Server Error.vi` | VI Properties → Execution | |
| 6 | Reentrancy of `Log Server Error.vi` and the `Enqueue Send … Response` VIs | VI Properties → Execution | |
| 7 | Values of `Check FRF?` and `Move to middle?`, and content of `RON settings`, on the front panel of PI's `Mercury_GCS_Configuration_Setup.vi` | Open the VI: the front panel opens even though the diagram is password-protected | |
| 8 | Default of the input `Use File Path Image Type` of **IMAQ Read Image And Vision Info 2**. It decides the pixel type of the loaded flat-field images. | Context Help on the node | |

*Background: [01](01-architecture.md) §3.4.3, §12; [03](03-command-reference.md) §5.20; [05](05-camera-acquisition.md) §4.2,
§9.1; [06](06-autofocus-refocus.md) §1.3, §4.2, §5.5; [07](07-focus-metrics.md) §7; [08](08-z-stage.md) §4.3;
[09](09-configuration-logging-errors.md) D20.*

---

## Anything else?

Answer:
