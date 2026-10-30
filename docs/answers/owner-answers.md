# Owner answers: summary and impact

This page collects the answers given on 2026-10-30 in [`../open-questions.md`](../open-questions.md), in the chat and in the
`alox_mx.zip` archive. It records what each answer settles and what it changes in the specification. The screenshots and INI
files themselves are in this folder.

## 1. Enum items (wire codes)

LabVIEW numbers enum items from 0 in the order shown by *Edit Items*. All enums below are assumed to be **U16**, LabVIEW's
default; only `Camera Command` was confirmed as U16 by the owner.

### Camera Command (`typedef_ext_camera_camera_command.ctl`), U16, 46 items

Source: [`enums/camera_command_1.png`](enums/camera_command_1.png), [`enums/camera_command_2.png`](enums/camera_command_2.png).

| Code | Item | Code | Item | Code | Item |
|---|---|---|---|---|---|
| 0 | Get Heartbeat | 16 | Cam Autofocus | 32 | Z Get Position |
| 1 | Cam Setup | 17 | Cam Set FlatField | 33 | Z Move Absolute |
| 2 | Cam Start | 18 | Cam Refocus | 34 | Z Move Relative |
| 3 | Cam Get Image | 19 | Cam Flip | 35 | Z Wait End Move |
| 4 | Cam Stream Next Image | 20 | Cam Get Autofocus Data | 36 | Z Stop |
| 5 | Cam Get Acquisition Status | 21 | Cam Get Autofocus Progress | 37 | Z Enable Joystick |
| 6 | Cam Software Trigger | 22 | Cam Autofocus Run | 38 | Z Disable Joystick |
| 7 | Cam Set Exposure | 23 | Cam Refocus Run | 39 | Z Joystick Enabled ? |
| 8 | Cam Get Exposure | 24 | Cam Set Led | 40 | Z Set Min Limit |
| 9 | Cam Set Gain | 25 | Cam Get Led | 41 | Z Set Max Limit |
| 10 | Cam Get Gain | 26 | Z Setup | 42 | Z Cleanup |
| 11 | Cam Get Images Array | 27 | Z Get Serial | 43 | Z Get Reference |
| 12 | Cam Stop | 28 | Z Get Status | 44 | Z Do Reference |
| 13 | Cam Cleanup | 29 | Z Get Error | 45 | Log |
| 14 | Exit | 30 | Z Set Speed | | |
| 15 | Show UI | 31 | Z Get Speed | | |

**Routing consequences** (docs/03 §3.2):

- The camera range `"Get Heartbeat".."Cam Get Led"` is codes 0–25. It contains `Exit` (14), so the server can be stopped.
  It also contains `Show UI` and the internal `Cam Autofocus Run` / `Cam Refocus Run` (22, 23).
- The Z range `"Z Setup".."Z Do Reference"` is codes 26–44: **all 19 Z commands reach the Z loop.**
- `Log` (45) is routed to the camera loop by its explicit case.
- No known item is routed nowhere; only codes ≥ 46 would be dropped.

### Other enums

| Enum (file) | Items in code order | Screenshot |
|---|---|---|
| `HW TCP Datatype.ctl` ("TCP DataType") | 0 Command, 1 Response, 2 Image, 3 Heartbeat, **4 Statut** (new item, not seen in the printouts) | [`enums/hw_tcp_datatype.png`](enums/hw_tcp_datatype.png) |
| `HW TCP Server Command.ctl` ("State", internal) | 0 Listen, 1 Read, 2 Send Heartbeat, 3 Send Response, 4 Quit | [`enums/hw_tcp_server_command.png`](enums/hw_tcp_server_command.png) |
| `alox_typedef_camera_type.ctl` (strict) | 0 High Mag., 1 Low Mag. | [`enums/camera_type.png`](enums/camera_type.png) |
| `typedef_ext_camera_camera_command_type_LED.ctl` ("Camera Output: LED") | 0 GiGe, 1 USB | [`enums/camera_output_led.png`](enums/camera_output_led.png) |
| `alox_typedef_autofocus_method.ctl` (strict) | 0 FFT, 1 Sobel, 2 Prewitt, 3 Diff, 4 Gradient, 5 Sigma, 6 FULL | [`enums/autofocus_method.png`](enums/autofocus_method.png) |
| `alox_typedef_autofocus_speed.ctl` (strict) | 0 Slow, 1 Fast | [`enums/autofocus_speed.png`](enums/autofocus_speed.png) |
| `typedef_ext_camera_autofocus_state.ctl` (strict, "AF State") | 0 Start, 1 Goto Z Pos, 2 Wait Started, 3 Measure, 4 Finished, 5 Error | [`enums/autofocus_state.png`](enums/autofocus_state.png) |
| `alox_typedef_refocus_method.ctl` (strict) | 0 Fin, 1 Oxyd | [`enums/refocus_method.png`](enums/refocus_method.png) |

Still unknown: the Z Do Reference enum, the representation of the enums other than `Camera Command` (U16 assumed), and the
Camera Plug Type ring/enum values. The INI uses the strings `"USB"` and `"GiGe"`.

## 2. Cluster order

`typedef_ext_camera_init_data.ctl` (the `Cam Setup` payload) is **Camera Type, Camera Name, Number of buffers, Continuous Mode**
(cluster order 0–3) [`camera_init_data_cluster_order.png`](camera_init_data_cluster_order.png). This is the front-panel order.
The alternative order inferred from the main VI's Unbundle (docs/04) is **wrong** and must be dropped.

The other clusters asked in Q33 (autofocus, refocus, Z wait, joystick) are still open.

## 3. VI properties

`alox_camera_server_get_image_state_machine.vi` is **non-reentrant** and has **automatic error handling enabled**
[`get_image_state_machine_vi_properties.png`](get_image_state_machine_vi_properties.png). Its uninitialised shift registers
(`t0`, `t0 Buffer Number`) are therefore shared by both cameras, as docs/05 feared.

## 4. Hardware

| Item | Answer | Consequence |
|---|---|---|
| Cameras (HM and LM) | IDS or Allied Vision, USB3 or GigE (or both), Mono8, Sony IMX264 sensor (5.1 MP, 2448 × 2048) | **No FireWire**: risk R4 is retired. Python backends: IDS peak (`ids_peak`) and Allied Vision VmbPy, or Harvesters with the vendors' GenTL producers. |
| Lights | LED is used with the Low Mag camera, bulb with the High Mag camera; otherwise off | Matches `setLEDline_I32` / `setBULBline_I32` (docs/05 §9) |
| PI controller | **C-663** (Mercury Step), GCS, RS-232 through a Moxa serial device server (virtual COM port), 115200 Bd, COM from the INI | docs/08 assumed a C-863: same GCS command set; `pipython` supports the C-663 |
| Z stage | PI **L-509.40S** linear stage | Reference/limit switches still unknown |
| OS | Windows only | |
| Language | Python | |
| GUI | Small diagnostic page or GUI for troubleshooting only | |
| LabVIEW 2018 + Vision machine | Available | Golden vectors (docs/04 §7.3) and NI reference outputs (docs/07) can be produced |

## 5. PI controller macros

`DH` is a **controller-side daemon**. It runs forever, because its last line restarts itself:

```text
JRC 5 POS? 1 < 5.05     if Z < 5.05 mm, skip the next 4 lines
DIO 1 1                 (Z >= 5.05) pulse digital output 1: high 1 s, then low, wait 2 s
DEL 1000
DIO 1 0
DEL 2000
JRC 5 POS? 1 > 5.05     if Z > 5.05 mm, skip the next 4 lines
DIO 1 0                 (Z <= 5.05) pulse digital output 1: low 1 s, then high, wait 2 s
DEL 1000
DIO 1 1
DEL 2000
JRC 2 DIO? 1 = 0        if digital input 1 is low, skip the next line
MAC START HOME          (input 1 high) run HOME
MAC START DH            restart DH
```

`HOME` drives the stage to Z = 0 when digital input 1 is set, for example by a hardware button:

```text
JAX 1 1 1               joystick 1, axis 1 → controller axis 1
JON 1 0                 joystick off
MOV 1 0                 move to Z = 0 mm
WAC ONT? 1 = 1          wait until on target
MAC START DH            back to the DH daemon
```

Consequences:

- Commands that stop the controller (`STP`, `#24`) also stop the running macro. That is why the LabVIEW server restarts `DH`
  after `Z Get Status`, `Z Stop` and `Z Cleanup`, and why stage 2 checks whether a macro is running. The new server **keeps the
  automatic restart** (`restart_dh_macro` = TRUE).
- Digital output 1 signals whether Z is above or below 5.05 mm, and digital input 1 triggers a move to 0 mm. Both work only
  while `DH` runs.
- A `HOME` run moves the stage with no command from the server. Any position the server cached can be stale.

## 6. Configuration files

Five real `alox.ini` variants were supplied ([`ini/`](ini/)): `alox.ini`, `alox_mx.ini`, `alox_usbIDS.ini`,
`alox_GiGeIDS.ini` and `alox_usbAllied.ini`. They confirm or reveal the following:

- `alox.ini` is **shared with the rest of ALOX Vision Core**: user rights, overlays, XY stage, monochromator, web service,
  and so on. The new server must keep its own settings in a separate file and never write `alox.ini` (docs/10 §6).
- **Decimal commas** (`24,600000`), **trailing spaces** after values, **quoted strings**, NI "common" paths (`"/D/ALOX/profiles"`,
  `"/C/Users/..."`), **duplicate keys** (`nbpoints9_U32` four times, `xaxis_I32` twice: the first occurrence wins) and CRLF line
  ends. These are exactly the NI rules of docs/09; the files become test fixtures.
- `[hardware.camera] port_I32 = 503` (TCP port) and `[general] core_I32 = 1`.
- Camera `name_STR` values are swapped in `alox.ini` (`lowmag = "HighMag"`, `highmag = "LowMagg"`) and normal in the other
  variants. These are NI MAX camera names.
- `lightStatusInverter_BOOL` is missing in four variants, so the LabVIEW default (TRUE) applies there.
- `[hardware.z]`: `baudrate_I32 = 3` (index → 115200), `comport_I32` = 2 or 4, `zmax_DBL` = 80 or 90, autofocus Z window 57–63 mm.
- `alox_setup.ini` was not supplied. The defaults `D:\alox\settings`, `D:\alox\02-vision`, `D:\alox\02-Vision\data` are kept.

## 7. Decisions for Q16–Q28

The owner's instruction is "do what you must". The recommended defaults of the questionnaire are therefore applied:

| Q | Decision |
|---|---|
| Q16 | Reproduce (switch `exposure_sets_max_framerate` = TRUE) |
| Q17 | Reproduce until a capture confirms (switch `send_error_replies` = FALSE) |
| Q18 | Fix (append to the log, correct HI/LO labels) |
| Q19 | Keep "Fin" only |
| Q20 | Fix (restore only values actually read) |
| Q21 | Reproduce (non-centred Low Mag ROI), flagged |
| Q22 | Reproduce, add a timeout. The best Z is chosen by the client software, so the server keeps returning the `[Z, score]` table. |
| Q23 | No INI write (alox.ini is shared) |
| Q24 | Reproduce (`implicit_z_reinit` = TRUE) |
| Q25 | Reproduce until confirmed (`strict_new_frame` = FALSE) |
| Q26 | Reproduce the PNG names |
| Q27 | Reproduce the maths, fix the FULL race |
| Q28 | Fix (no deadlock) |
| Q11 | Reference the axis at every `Z Setup` |

## 8. Still open

Ordered by value:

1. **A network capture (Q4)** or **golden vectors** exported on the LabVIEW machine (docs/04 §7.3). Either one settles the
   variant byte format, the NI image format and whether error replies are sent.
2. **Q33**: cluster order of the autofocus, refocus, Z wait and joystick payloads (same screenshot method as Cam Init Data).
3. **Q31**: the client VIs (the "synchronous calls" folder).
4. The Z Do Reference enum, Q35 (Z stage test session) and Q34 (NI MAX camera settings).
