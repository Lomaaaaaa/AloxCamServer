# Configuration, folders, logging and error handling

**Summary.** The server reads two INI files and writes none. `alox_setup.ini` sits next to the executable and maps three
logical folders (`settings`, `vision`, `data`) to disk paths. `alox.ini`, in the `settings` folder, holds 53 keys that are read once
at start-up into front-panel objects: TCP port, camera settings, LED lines, Z-stage link and limits. Both files are parsed by NI's
`config.llb`. Its rules differ from Python's `configparser`: names are case-insensitive, the first occurrence wins, `;` starts a
comment, quotes and `\xx` escapes are handled, and a decimal comma is accepted. Every read error is silently replaced by a hard-coded
default. Events go to daily log files: errors in `<vision>\logs`, information lines in `<data>\logs`. The error log has a defect: each
line is written at offset 0, so it overwrites the start of the file instead of being appended. Errors travel as LabVIEW error clusters
`{status, code, source}`. The server defines a handful of custom codes (9009, −9011, 9031, 9401, 9405, −1) and passes NI-IMAQdx and PI
codes through unchanged. This chapter specifies all of this and proposes a Python exception design. Inside the server it uses
exceptions; at the TCP and log boundaries it rebuilds the same codes and source texts.

**Sources.** ALOX VIs: `tools_application_executable_folder.vi` [Doc6 p.87–90], `tools_windows_user_documents_folder.vi`
[Doc6 p.409–412], `tools_alox_shared_vision_folder.vi` [Doc6 p.426–429], `tools_alox_shared_data_folder.vi` [Doc6 p.1222–1225],
`Log Server Error.vi` [Doc6 p.430–434], `Log Server Infos.vi` [Doc6 p.1218–1221]. Main VI: stage 2 (configuration)
[main VI frames d3–d61; p.34–43], the `Write Key` in `Cam Set Led` [main VI frame d195; p.26], cleared IMAQdx errors
[main VI p.14, 17, 19, 29]. NI `config.llb`: `Open Config Data` [Doc6 p.265–268], `Load` [Doc6 p.252–256], `New Config to Queue`
[Doc6 p.257–260], `String to Config` [Doc6 p.98–101], `Parse Key Value Pair` [Doc6 p.71–74], `Get Key` [Doc6 p.1091–1094],
`Get Section` [Doc6 p.1095–1098], `Read Key` and its instances [Doc6 p.1247–1291], `Read Value Without Comment`
[Doc6 p.1280–1283], `Remove Quotes` [Doc6 p.1268–1271], `Unescape String` [Doc6 p.1263–1266], `Common Path to Specific Path`
[Doc6 p.1251–1254], `Write Key` instances [Doc6 p.1052–1085, 1128–1131], `Close Config Data` [Doc6 p.1226–1229],
`Save Config File` / `Config to String` [Doc6 p.1230–1242]. NI `error.llb`: `Clear Errors.vi` [Doc6 p.57–59],
`Error Cluster From Error Code.vi` [Doc5 p.1–6]. NI `Application Directory.vi` [Doc6 p.61–63]. Custom error constants:
`alox_camera_server_get_image_state_machine.vi` [Doc6 p.555], `alox_camera_server_simulation.vi` [Doc6 p.921],
`alox_camera_server_autofocus.vi` [Doc6 p.608], refocus VI [Doc6 p.1034–1037], `HW TCP Extract TCP Data.vi` [Doc6 p.1153].

Related chapters: [01-architecture.md](01-architecture.md) (start-up sequence, shared state),
[02-tcp-protocol.md](02-tcp-protocol.md) (error cluster on the wire, reply VIs), [03-command-reference.md](03-command-reference.md)
(per-command logging and source decoration), [04-data-types.md](04-data-types.md), [05-camera-acquisition.md](05-camera-acquisition.md),
[06-autofocus-refocus.md](06-autofocus-refocus.md) (refocus image files), [07-focus-metrics.md](07-focus-metrics.md),
[08-z-stage.md](08-z-stage.md) (PI error codes), [10-migration-plan.md](10-migration-plan.md).

---

## 1. Conventions and LabVIEW primer

### 1.1 Conventions

- Untagged statements are observed on the printouts. **(inferred)** marks a deduction from LabVIEW/NI semantics or from
  cross-checking. **(unknown)** marks something the printouts do not show.
- `[Doc6 p.N]` is a page of "Labview Document 6" (sub-VIs), `[Doc5 p.N]` of "Labview Document 5". `[main VI frame dN]` is hidden frame
  N of the HTML export of `alox_camera_server.vi`, and `[main VI p.N]` is a page of the main VI PDF.
- `<settings>`, `<vision>`, `<data>` denote the folders resolved in §3. `<exe>` is the executable's folder.

### 1.2 LabVIEW notions used in this chapter

| Notion | Meaning | Python equivalent |
|---|---|---|
| **Path** | A typed file path, distinct from a string. An *empty path* is valid and relative; *Not a Path* (NaP) is an invalid-path sentinel. `Build Path(base, name)` appends one component; `Strip Path` removes the last one. | `pathlib.Path`; use `None` for NaP |
| **Config refnum** | `Open Config Data` loads the whole file into an in-memory structure held in a one-element queue. Every `Read Key` locks it, reads and puts it back. Nothing reaches the disk until `Close Config Data` (only if something changed) or `Save Config File`. | An in-memory parsed object; no file handle stays open |
| **Error cluster** | `{status: bool, code: I32, source: string}`. `status = TRUE` is an error; `status = FALSE` with `code ≠ 0` is a *warning*; `FALSE/0` is "no error". It flows through nodes along the error wire (§8). | `ErrorCluster` value object plus exceptions (§10) |
| **Diagram Disable structure** | Only the `Enabled` frame runs; the `Disabled` frame is dead code kept in the source. | Ignore disabled code |
| **`App.Kind`** | Property telling whether the code runs in the development environment or as a built executable (`Run Time System`). | `getattr(sys, "frozen", False)` for a frozen build |
| **`Format Date/Time String`** | `strftime`-like. `%3u` prints fractional seconds with 3 digits, preceded by the decimal separator (separator follows the system locale **(inferred)**). | `datetime.strftime` plus manual milliseconds |

---

## 2. Files and folders at a glance

```
<exe>\alox_setup.ini                    [folders] settings / vision / data      (read; created empty if missing)
   │
   ├─ settings → <settings>\alox.ini    53 keys, read once at start-up          (read; created empty if missing; never written)
   ├─ vision   → <vision>\logs\<yymmdd>_<type>_events.log    Log Server Error   (written at offset 0 — defect D10)
   └─ data     → <data>\logs\<yymmdd>_<type>_events.log      Log Server Infos   (appended)
```

| File or folder | Default location | Read / written by | When |
|---|---|---|---|
| `alox_setup.ini` | `<exe>\alox_setup.ini` | the three folder VIs (§3) | start-up (settings), then **on every log line** (vision, data) |
| `alox.ini` | `D:\alox\settings\alox.ini` | stage 2 of the main VI (§5) | once, at start-up |
| Error log | `D:\alox\02-vision\logs\<yymmdd>_camera_events.log` and `…_z_events.log` | `Log Server Error.vi` (§7) | every logged error |
| Information log | `D:\alox\02-Vision\data\logs\<yymmdd>_z_events.log` | `Log Server Infos.vi` (§7) | silent Z re-referencing in `Z Move …` |
| Flat-field / dark-current images | INI `flatfieldimage_PATH` / `darkfieldimage_PATH`, or `Cam Set FlatField` | `Cam Setup` (read) | [05-camera-acquisition.md](05-camera-acquisition.md) |
| Simulation images | `Simulation Path` sent by the client | camera simulation (read) | [05-camera-acquisition.md](05-camera-acquisition.md) |
| Refocus images | `Folder` sent by the client in `Cam Refocus` | refocus (write, PNG) | [06-autofocus-refocus.md](06-autofocus-refocus.md) |

No other configuration file exists. A text search of all printouts finds only `alox_setup.ini` [Doc6 p.411, 428, 1224] and
`alox.ini` [main VI p.34] (plus PI's own `WG.INI`/`PiStages.dat` in vendor help text). Recognising the icons of `config.llb` VIs on
every ALOX diagram finds 3 + 1 `Open Config Data`, 3 + 53 `Read Key`, 1 `Write Key`, 3 + 1 `Close Config Data` and no
`Save Config File`.

---

## 3. Folder resolution

### 3.1 Executable folder: `tools_application_executable_folder.vi` [Doc6 p.87–90]

| Item | Value |
|---|---|
| Interface | in: `error in`; out: `Executable Filepath` (Path), `error out` |
| `No Error` case | `App.Kind == Run Time System` ? `Application Directory` (folder containing the `.exe` [Doc6 p.61]) : `Strip Path(Strip Path(Current VI's Path))` |
| `Error` case | `Executable Filepath` ← a path constant (empty path or NaP **(unknown)**); `error out` ← `error in` |
| Development value | The VI lives in `D:\ALOX\Vision Core\sources\tools\`, so it returns `D:\ALOX\Vision Core\sources` **(inferred)**. The diagram comments ("remove this vi name, llb name") date from an older layout. |

The `App.Kind` test is needed because, inside a built executable, `Current VI's Path` points *inside* the `.exe`.

### 3.2 The three folder resolvers

All three do: `exe ← tools_application_executable_folder(error in)`; `cfg ← Open Config Data(Build Path(exe, "alox_setup.ini"))`;
`folder ← Read Key (Path)(cfg, "folders", <key>, <default>)`; `Close Config Data(cfg)`. `create file if necessary` and
`write file if changed` are unwired, so both default to TRUE.

| VI | Key | Hard-coded default | Output label | Callers |
|---|---|---|---|---|
| `tools_windows_user_documents_folder.vi` [Doc6 p.411] | `settings` | `D:\alox\settings` | `My Documents` (misleading) | main VI stage 2, once [main VI frame d3] |
| `tools_alox_shared_vision_folder.vi` [Doc6 p.428] | `vision` | `D:\alox\02-vision` | `Alox Shared Data Folder` | `Log Server Error.vi`, at each logged error [Doc6 p.432] |
| `tools_alox_shared_data_folder.vi` [Doc6 p.1224] | `data` | `D:\alox\02-Vision\data` | `Alox Shared Data Folder` | `Log Server Infos.vi`, at each call [Doc6 p.1220] |

- Despite its name, `tools_windows_user_documents_folder.vi` no longer reads the Windows "Documents" folder. The registry read
  (`HKEY_CURRENT_USER\Software\Microsoft\Windows\CurrentVersion\Explorer\Shell Folders`, value `Personal`) sits in a `Disabled`
  frame. So the stage-2 label "Load Default settings from Alox configuration file in "C:Users/<USER>/alox/settings/alox.ini""
  [main VI p.34] is obsolete.
- The two other VIs share a copied description ("returns the <ALOX DATA> folder path … where alox Vision saves all data (images,
  logs, process csv files)") and the same output label.
- The defaults differ in case (`02-vision` versus `02-Vision`). That is harmless on Windows but matters on Linux.
- No VI caches its result. `alox_setup.ini` is opened and parsed again for every log line **(inferred from the call sites)**, so
  editing it while the server runs moves the logs.
- No other ALOX VI calls a folder resolver. The refocus, simulation and flat-field paths come from the client or from `alox.ini`.

### 3.3 Error behaviour of the folder resolvers

| Situation | Result |
|---|---|
| `error in` already in error | No file access. `Read Key` returns the default; `error out` = `error in`. Result: the **default folder**. |
| `alox_setup.ini` missing, `<exe>` writable | `Load.vi` **creates an empty file** ("File does not exist … Create file") [Doc6 p.254]. Key absent → default. No error. |
| `alox_setup.ini` missing, `<exe>` read-only (e.g. under `C:\Program Files`) | Creation fails → error → default folder plus an error **(inferred)**. |
| Key present with an empty value (`settings =`) | An **empty path**, not the default and not an error (NI raises "Config type invalid" only for a non-empty NaP) [Doc6 p.1249]. `alox.ini` then becomes the relative path `alox.ini` and cannot be opened **(inferred)**, so every `alox.ini` key takes its default. |
| Value not convertible to a path | Error "Config type invalid" (source `Read Key (Path).vi`) [Doc6 p.1249]. |

The main VI leaves the `error out` of `tools_windows_user_documents_folder.vi` unwired [main VI frame d3], so these errors are never
reported.

### 3.4 Python reference sketch: folders

```python
# Reference sketch: folder resolution, done ONCE at start-up (LabVIEW re-reads alox_setup.ini per log line).
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from niconfig import NIConfig                    # section 6.6

FOLDER_DEFAULTS = {"settings": r"D:\alox\settings",
                   "vision": r"D:\alox\02-vision",
                   "data": r"D:\alox\02-Vision\data"}


def executable_folder() -> Path:
    """tools_application_executable_folder.vi."""
    override = os.environ.get("ALOX_EXE_DIR")    # proposal: lets tests and development point elsewhere
    if override:
        return Path(override)
    if getattr(sys, "frozen", False):            # PyInstaller and similar ~ App.Kind == "Run Time System"
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parents[1]   # development: adapt to the package layout


@dataclass(frozen=True)
class Folders:
    exe: Path
    settings: Path
    vision: Path
    data: Path


def resolve_folders(warn: Callable[[str], None]) -> Folders:
    exe = executable_folder()
    setup = exe / "alox_setup.ini"
    try:
        cfg = NIConfig.load(setup)               # deviation: no empty file is created (D2)
    except OSError as exc:
        warn(f"{setup}: {exc}; built-in folder defaults used")
        cfg = NIConfig.parse("")
    found = {}
    for key, default in FOLDER_DEFAULTS.items():
        value = cfg.read_path("folders", key, default)
        if value == "":                          # deviation: LabVIEW keeps the empty path (D3)
            warn(f"[folders] {key} is empty; {default} used")
            value = default
        found[key] = Path(value)                 # on Linux, map 'D:\...' explicitly (open question 7)
    return Folders(exe, found["settings"], found["vision"], found["data"])
```

---

## 4. `alox_setup.ini`

| Section | Key | NI type | Default | Meaning |
|---|---|---|---|---|
| `folders` | `settings` | Path | `D:\alox\settings` | folder holding `alox.ini` |
| `folders` | `vision` | Path | `D:\alox\02-vision` | its `logs` subfolder receives the error log |
| `folders` | `data` | Path | `D:\alox\02-Vision\data` | its `logs` subfolder receives the information log |

```ini
; <exe>\alox_setup.ini   (development: D:\ALOX\Vision Core\sources\alox_setup.ini)
[folders]
settings = D:\alox\settings
vision = D:\alox\02-vision
data = D:\alox\02-Vision\data
```

Path values are read **raw** (§6.2). Write them **without quotes**: `settings = "D:\alox\settings"` would keep the quotes inside the
path. The NI common form `/D/alox/settings` is also accepted (§6.3). The file is never written; it is only created empty when it is
missing.

---

## 5. `alox.ini`

### 5.1 Opening, reading and closing (stage 2 of the main VI)

```
settings ← tools_windows_user_documents_folder()            # error out unwired          [main VI frame d3]
cfg, err_open ← Open Config Data(Build Path(settings, "alox.ini"))    # create if necessary = TRUE (unwired)
refnum (front-panel indicator) ← cfg                        # reused, uselessly, by Cam Set Led (§5.5)
inner sequence, frames 0..6, run in order:
  0  3 keys  → TCP Port, Core #, #Lines                     [main VI frame d4]
  1 19 keys  → High Mag Settings (For loop, N = 21)         [main VI frames d5–d25]
  2 20 keys  → Low Mag Settings  (For loop, N = 21)         [main VI frames d26–d47]
  3  7 keys  → Z Stage Settings  (For loop, N = 7)          [main VI frames d48–d58]
  4  no key: PI macro check (RMC? / MAC START "DH")         [main VI frames d59–d60] — see 08-z-stage.md
  5  4 keys  → HM setLEDline, HM setBULBline, Camera Sort: LED, Connector, LEDStatusInverter   [main VI frame d3]
  6  Close Config Data(cfg)                                 [main VI frame d61]
```

- `Close Config Data` has the input `write file if changed (T)` [Doc6 p.1226]. It is unwired, and nothing was modified, so nothing
  is written **(inferred)**. (An earlier analysis named this input "write configuration file?" with default FALSE. The printed
  connector pane shows the name above. Either way, nothing is written.)
- A missing `alox.ini` in an existing folder is **created empty**, and every key takes its default.
- Results go only to front-panel objects, which the loops read later through local variables
  ([01-architecture.md](01-architecture.md) §3.4). `found?` outputs are never wired.

### 5.2 Complete key schema

Defaults are the constants wired on the diagram. They apply when the key or section is missing or unreadable. The type suffix
(`_I32`, `_DBL`, …) is part of the key name.

**General keys**

| Section | Key | NI type | Default | Front-panel target | Used by |
|---|---|---|---|---|---|
| `hardware.camera` | `port_I32` | I32 | 503 | `TCP Port` | TCP listener on 127.0.0.1, I32 → U16 ([02](02-tcp-protocol.md) §3) |
| `general` | `core_I32` | I32 | 1 | `Core #` | `Core#` input of the log VIs, used only in disabled code: **no effect** (§7.1) |
| `log` | `maxlines_I32` | I32 | 1000 | `#Lines` | `Max Lines` of the log VIs: trims the on-screen log (§7.5) |
| `general` | `setLEDcam_I32` | I32 | 0 | `Camera Sort: LED`, `Connector` | 0 → lights driven through the HM camera (`"HM"`); ≠ 0 → LM (`"LM"`) ([05](05-camera-acquisition.md) §9) |
| `hardware.camera` | `lightStatusInverter_BOOL` | Boolean | TRUE | `LEDStatusInverter` (a control) | `Cam Set Led` ([05](05-camera-acquisition.md) §9) |

**Camera keys**: section `hardware.camera.highmag` → `High Mag Settings`; section `hardware.camera.lowmag` → `Low Mag Settings`.
Field semantics: [05-camera-acquisition.md](05-camera-acquisition.md) §3.

| Key | NI type | Default HM / LM | Field | Notes |
|---|---|---|---|---|
| `name_STR` | String | `HighMag` / `LowMag` | `Camera Name` | image custom data `name` only; the camera is opened by the name sent in `Cam Setup` |
| `buffers_I32` | I32 | 5 | `Buffers` | not used (ring fixed at 5) |
| `timeout_I32` | I32 | 5000 | `Timeout` | not used |
| `flatfieldcorrection_BOOL` | Boolean | FALSE | `Flat Field` | |
| `flatfieldimage_PATH` | Path | empty path | `Flat Field Path` | |
| `darkfieldimage_PATH` | Path | empty path | `Dark Current Path` | |
| `flatfieldfactor_DBL` | Double | 1 | `Flat Field Factor` | |
| `exposure_STR` | String | `CameraAttributes::Exposure::Value` | `Camera Exposure Attribute` | IMAQdx attribute path |
| `gain_STR` | String | `CameraAttributes::Gain::Value` | `Camera Gain Attribute` | |
| `exposuretype_STR` | String | `DBL` | `Exposure Type` | `DBL` / `I64` / `U32`, case-sensitive, anything else = `DBL` |
| `gaintype_STR` | String | `I64` | `Gain Type` | idem |
| `hflip_BOOL` | Boolean | FALSE | `Horizontal Flip` | |
| `vflip_BOOL` | Boolean | FALSE | `Vertical Flip` | |
| `framerate_STR` | String | `CameraAttributes::AcquisitionControl::AcquisitionFrameRate` | `Camera Framerate Attribute` | |
| `frameratetype_STR` | String | `DBL` | `Framerate Type` | |
| `frameratemax_DBL` | Double | **not read** / 24.6 | `Max Framerate` | HM keeps the value of an unprintable constant **(unknown)** [main VI frames d5–d25, d42] |
| `LimitMinExposure_DBL` | Double | 177 | `Exposure Lower Limit` | |
| `waitled_U32` | U32 | 0 | `Wait LED (ms)` | |
| `invertled_BOOL` | Boolean | **FALSE / TRUE** | `Invert LED` | different defaults |
| `cameraconnectortype_STR` | String | `gige` | `Camera Plug Type` | derived, §5.3 |
| `setLEDline_I32` | I32 | 0 (HM only) | `HM setLEDline` | read in frame 5 |
| `setBULBline_I32` | I32 | 0 (HM only) | `HM setBULBline` | read in frame 5 |

**Z-stage keys**: section `hardware.z` → `Z Stage Settings`. Semantics: [08-z-stage.md](08-z-stage.md).

| Key | NI type | Default | Field | Notes |
|---|---|---|---|---|
| `comport_I32` | I32 | 2 | `Com` | COM port number; `≤ 0` → COM1 in `Z Setup` |
| `baudrate_I32` | I32 | 3 | `Baudrate` | **index**, §5.3 |
| `zaxis_I32` | I32 | 1 | `Axis` | used as the PI driver's "System no." |
| `speed_DBL` | Double | 10 | `Speed (mm/s)` | |
| `slowspeed_DBL` | Double | 1 | `Slow Speed (mm/s)` | never used |
| `zmin_DBL` | Double | 0 | `Min Position (mm)` | software travel range |
| `zmax_DBL` | Double | 20 | `Max Position (mm)` | software travel range |

Total: 3 + 19 + 20 + 7 + 4 = 53 `Read Key` calls.

### 5.3 Derived values

| Key | Conversion | Evidence |
|---|---|---|
| `cameraconnectortype_STR` | Case-sensitive string case: `"USB"`, `"USb"`, `"UsB"`, `"Usb"`, `"uSB"`, `"usb"` → `USB`; `"gige"` and everything else (including `GigE`, `uSb`, `usB`, empty) → `GiGe` | [main VI frames d24–d25, d46–d47] |
| `baudrate_I32` | `0` or any other value → 9600, `1` → 19200, `2` → 38400, `3` → 115200. A literal `115200` gives **9600**. | [main VI frames d51–d54] |
| `setLEDcam_I32` | `use_LM ← value ≠ 0`; `Camera Sort: LED ← (use_LM ? Low : High) Mag Settings.Camera Plug Type`; `Connector ← use_LM ? "LM" : "HM"`. Frame 5 runs after frames 1–2, so it reads their results. | [main VI frame d3] |
| `port_I32` | I32 coerced to the U16 port of `TCP Listen` | [main VI frame d62] |

### 5.4 Error propagation while loading

Every `Read Key` error output ends unwired or in an unwired shift register, so nothing is ever reported. The error wire still
matters, because a `Read Key` whose `error in` carries an error does not read and returns its default.

| Frame | Keys whose `error in` is chained (in order) | Keys with no `error in` |
|---|---|---|
| 0 | `port_I32` (gets the `Open Config Data` error) → `core_I32` → `maxlines_I32` | — |
| 1 (HM) | `name_STR` → `buffers_I32` → `timeout_I32` → `flatfieldcorrection_BOOL` (the first one gets the open error) | the 15 others |
| 2 (LM) | same four | the 16 others |
| 3 (Z) | `comport_I32` → `baudrate_I32` → `zaxis_I32` → `speed_DBL` → `slowspeed_DBL` | `zmin_DBL`, `zmax_DBL` |
| 5 | `setLEDline_I32`, `setLEDcam_I32`, `setBULBline_I32` each get the open error (in parallel) | `lightStatusInverter_BOOL` |

Consequences **(inferred from NI semantics)**:

1. If `Open Config Data` fails, **every** key takes its default. Chained keys skip; the others fail on the invalid refnum.
2. A non-convertible value (e.g. `port_I32 = abc`) makes **all later keys of the same chain** fall back to their defaults even when
   they are present and valid.
3. A non-convertible value in a key without `error in` affects only that key.
4. The server always starts, whatever the INI contains.

### 5.5 Nothing is ever persisted

- Run-time changes stay in memory: `Cam Flip` (flip flags), `Cam Set FlatField` (flat-field settings), `Z Set Min Limit` /
  `Z Set Max Limit` (travel range). They are lost at restart ([03-command-reference.md](03-command-reference.md)).
- The only `Write Key` is in `Cam Set Led` [main VI frame d195; p.26]: section `hardware.camera`, key `lightStatusInverter_BOOL`,
  refnum = local variable `refnum`. Its `value` input is **unwired**, and so are `error in` and every output. It has no effect
  **(inferred)**: (1) the refnum was closed at stage 2, frame 6, so the call fails silently; (2) even with a valid refnum, `Write Key`
  changes only memory, and no `Close Config Data` / `Save Config File` follows; (3) the value written would be the instance default.
  See D8.

### 5.6 Reference `alox.ini` with every default

```ini
; <settings>\alox.ini — hard-coded defaults of alox_camera_server.vi (stage 2)
[general]
core_I32 = 1
setLEDcam_I32 = 0

[log]
maxlines_I32 = 1000

[hardware.camera]
port_I32 = 503
lightStatusInverter_BOOL = TRUE

[hardware.camera.highmag]
name_STR = "HighMag"
buffers_I32 = 5
timeout_I32 = 5000
flatfieldcorrection_BOOL = FALSE
flatfieldimage_PATH =
darkfieldimage_PATH =
flatfieldfactor_DBL = 1
exposure_STR = "CameraAttributes::Exposure::Value"
gain_STR = "CameraAttributes::Gain::Value"
exposuretype_STR = "DBL"
gaintype_STR = "I64"
hflip_BOOL = FALSE
vflip_BOOL = FALSE
framerate_STR = "CameraAttributes::AcquisitionControl::AcquisitionFrameRate"
frameratetype_STR = "DBL"
LimitMinExposure_DBL = 177
waitled_U32 = 0
invertled_BOOL = FALSE
cameraconnectortype_STR = "gige"
setLEDline_I32 = 0
setBULBline_I32 = 0
; frameratemax_DBL is not read for High Mag

[hardware.camera.lowmag]
name_STR = "LowMag"
buffers_I32 = 5
timeout_I32 = 5000
flatfieldcorrection_BOOL = FALSE
flatfieldimage_PATH =
darkfieldimage_PATH =
flatfieldfactor_DBL = 1
exposure_STR = "CameraAttributes::Exposure::Value"
gain_STR = "CameraAttributes::Gain::Value"
exposuretype_STR = "DBL"
gaintype_STR = "I64"
hflip_BOOL = FALSE
vflip_BOOL = FALSE
framerate_STR = "CameraAttributes::AcquisitionControl::AcquisitionFrameRate"
frameratetype_STR = "DBL"
frameratemax_DBL = 24.6
LimitMinExposure_DBL = 177
waitled_U32 = 0
invertled_BOOL = TRUE
cameraconnectortype_STR = "gige"

[hardware.z]
comport_I32 = 2
baudrate_I32 = 3
zaxis_I32 = 1
speed_DBL = 10
slowspeed_DBL = 1
zmin_DBL = 0
zmax_DBL = 20
```

---

## 6. NI configuration-file semantics to reproduce

### 6.1 Parsing (`String to Config`, `Parse Key Value Pair`) [Doc6 p.71–74, 98–101, 257–260]

| Rule | Detail |
|---|---|
| Lines | The text is split on the regex `[\n\r]+`: CRLF, LF and CR all work, and empty lines disappear. |
| Section header | A line matching `^[\t\s]*\[[^\]]+\]`. The name is the text between the brackets (whether spaces inside the brackets are trimmed: **(unknown)**). |
| Before the first section | Lines go into an unnamed first section ("the first section is for ini file's comment" [Doc6 p.259]). Keys there cannot be read. |
| Key/value line | A line matching `^[\t\s]*[^\t\s;=\n\r][^=\n\r]*[\s\t]*=.*`. It splits at the **first** `=`, and the key is trimmed. A line starting with `#` and containing `=` is a **key** (named `# …`), not a comment. |
| Value | If the right-hand side starts with a quote, the whole text is kept, quotes and any `;` included (exact test **(inferred)**). Otherwise the value is the text before the first `;`, trimmed, and the rest is a comment. |
| Other lines | Kept as comments (e.g. `; text`, `# text`). |
| Lookup | `Get Section` / `Get Key` compare **lower-cased** names and return the **first** match [Doc6 p.1093, 1097]. The requested key is trimmed first. A duplicated section hides the second copy completely. |
| Missing section or key | The default value, `found? = FALSE`, **no error** [Doc6 p.1288]. |
| Encoding | Bytes in the Windows ANSI code page (cp1252 on Western systems) **(inferred)**. A UTF-8 BOM breaks the first section header, because the line no longer starts with `[` **(inferred)**. |

`configparser` differs on almost every row (case-sensitive sections, last occurrence wins, inline `;` off by default, no NI quoting or
escapes, no decimal comma, `#` comments, `DuplicateSectionError`). Use the dedicated reader of §6.6.

### 6.2 Value conversion per `Read Key` instance [Doc6 p.1247–1291]

| Instance | Text used | Conversion | On failure |
|---|---|---|---|
| String (`read raw string? = F`) | stored value | `Remove Quotes` (leading/trailing blanks, then one pair of `"…"` or `'…'`), then `Unescape String`: `\\` → `\`, and `\XX` with `XX` in `[01][0-9A-F]` → the character with that hex code. Any other `\` is kept. | — |
| I32 / U32 | `Read Value Without Comment` (Remove Quotes, then cut at the first `;`) | `Decimal String To Number`: leading blanks, optional sign, decimal digits, stops at the first other character (`12abc` → 12, `0x10` → 0, `3.7` → 3) | nothing parsed → default + error "Config type invalid" |
| Double | idem | Two scans, `%.;%f` (point) and `%,;%f` (comma); the one consuming more characters wins. Both `24.6` and `24,6` give 24.6. | default + error |
| Boolean | idem | Text exactly `0` or `1` → `== "1"`; otherwise a boolean scan of `TRUE` / `FALSE` (case-insensitive **(inferred)**) | default + error (`yes`, `on` fail) |
| Path | stored value **raw**: no unquoting, no unescaping, no comment cut beyond §6.1 | `Common Path to Specific Path` (§6.3), then string → path. An empty value gives an **empty path** (not the default). | non-empty NaP → error |
| any | — | `error in` in error → nothing read, value = default, error passed through | — |

The error is "Config type invalid" with source `Read Key (<type>).vi`. Its numeric code comes from an NI error ring that the
printout does not show **(unknown)**.

### 6.3 Paths (`Common Path to Specific Path`, Windows branch) [Doc6 p.1251–1254]

| Stored text | Result |
|---|---|
| `D:\alox\settings` (native) | unchanged |
| `/D/alox/settings` (NI "common" form, written by `Write Key (Path)`) | every `/` becomes `\`; a single leading `\` becomes `<letter>:` → `D:\alox\settings` |
| `\\server\share\x` (UNC) | unchanged |
| `relative\dir` | unchanged; a relative path then fails in file functions **(inferred)** |
| empty | empty path |

### 6.4 File creation, closing and writing

| Operation | NI behaviour | Used by this server |
|---|---|---|
| `Open Config Data(create file if necessary = T)` on a missing file | creates an **empty file** on disk, no error [Doc6 p.254] | yes, for both INI files |
| `Open Config Data` with an empty path or NaP and no creation | error "File not found" (LabVIEW code 7 **(inferred)**) [Doc6 p.254] | no |
| `Close Config Data(write file if changed = T)` | rewrites the whole file only if data changed, then releases the refnum [Doc6 p.1226–1228] | yes, nothing ever changed |
| `Write Key` | memory only; `Boolean` → `TRUE`/`FALSE`; `I32`/`U32` → decimal; `Double` → 6 decimals with the **system** decimal separator (`24,600000` on a French Windows); `String` → escaped and double-quoted; `Path` → NI common form, unquoted [Doc6 p.1052–1085, 1128–1131] | one call, ineffective (§5.5) |

A Python server should never write these files. If another LabVIEW tool rewrites `alox.ini`, the reader must accept quoted strings,
common-form paths and decimal commas, which §6.6 does.

### 6.5 Safe authoring rules (for people editing the INI files)

1. Keep each key in its exact section, and keep the type suffix in the key name. Case does not matter.
2. Put comments on their own lines, starting with `;`. Do not add a comment after a quoted value.
3. Path keys (`[folders] *`, `*_PATH`): no quotes. Native `D:\x` or common `/D/x` both work.
4. String keys: quotes are optional. In a string, `\\` means one backslash and `\0A`…`\1F` are control characters, so a Windows path
   in a `_STR` key can be corrupted.
5. Numbers: decimal only, no thousands separator. `baudrate_I32` is an **index 0–3**, not a baud rate.
6. Save as ANSI (cp1252) or plain ASCII, without a BOM.

### 6.6 Python reference sketch: NI-compatible reader and settings loader

```python
# Reference sketch: read-only INI reader with NI config.llb (LabVIEW 2018) semantics.
from __future__ import annotations

import re
from pathlib import Path
from typing import Optional

_SECTION = re.compile(r"[ \t]*\[([^\]]+)\]")              # String to Config [Doc6 p.100]
_KEY_VALUE = re.compile(r"[ \t]*[^ \t;=\n\r][^=\n\r]*=")   # idem (prefix up to the first '=')
_ESCAPE = re.compile(r"\\(\\|[01][0-9A-F])")              # Unescape String [Doc6 p.1265]


class ConfigTypeInvalid(ValueError):
    """NI 'Config type invalid' (numeric code not printed)."""


class NIConfig:
    def __init__(self, sections: list[tuple[str, list[tuple[str, str]]]]):
        self._sections = sections                          # [(name, [(key, value), ...]), ...]

    @classmethod
    def load(cls, path: Path, *, create_if_missing: bool = False,
             encoding: str = "cp1252") -> "NIConfig":
        path = Path(path)
        if not path.exists():
            if not create_if_missing:                      # NI default is TRUE: it creates an empty file
                raise FileNotFoundError(path)
            path.touch()
        return cls.parse(path.read_bytes().decode(encoding, errors="replace"))

    @classmethod
    def parse(cls, text: str) -> "NIConfig":
        text = text.removeprefix("\ufeff")                 # deviation: tolerate a UTF-8 BOM
        sections: list[tuple[str, list[tuple[str, str]]]] = [("", [])]   # entry 0: lines before any section
        for line in re.split(r"[\n\r]+", text):
            m = _SECTION.match(line)
            if m:
                sections.append((m.group(1), []))
                continue
            if not _KEY_VALUE.match(line):
                continue                                   # empty line or comment
            key, _, rest = line.partition("=")
            rest = rest.strip(" \t")
            if rest.startswith('"'):                       # quoted: keep everything, ';' included
                value = rest
            else:
                value = rest.partition(";")[0].strip(" \t")
            sections[-1][1].append((key.strip(" \t"), value))
        return cls(sections)

    def raw(self, section: str, key: str) -> Optional[str]:
        """First matching section, first matching key, both case-insensitive. None if absent."""
        s, k = section.lower(), key.strip(" \t").lower()
        for name, pairs in self._sections[1:]:
            if name.lower() == s:
                for kk, vv in pairs:
                    if kk.lower() == k:
                        return vv
                return None
        return None

    @staticmethod
    def _remove_quotes(v: str) -> str:                     # Remove Quotes [Doc6 p.1270]
        v = v.strip(" \t")
        for q in ('"', "'"):
            if len(v) >= 2 and v[0] == q and v[-1] == q:
                return v[1:-1]
        return v

    def _without_comment(self, section: str, key: str) -> Optional[str]:   # Read Value Without Comment
        v = self.raw(section, key)
        return None if v is None else self._remove_quotes(v).partition(";")[0].strip(" \t")

    def read_str(self, section: str, key: str, default: str) -> str:
        v = self.raw(section, key)
        if v is None:
            return default
        return _ESCAPE.sub(lambda m: "\\" if m.group(1) == "\\" else chr(int(m.group(1), 16)),
                           self._remove_quotes(v))

    def read_int(self, section: str, key: str, default: int, *, unsigned: bool = False) -> int:
        v = self._without_comment(section, key)
        if v is None:
            return default
        m = re.match(r"[ \t]*([+-]?[0-9]+)", v)            # Decimal String To Number: stops at first non-digit
        if not m:
            raise ConfigTypeInvalid(f"[{section}] {key} = {v!r}")
        lo, hi = (0, 2**32 - 1) if unsigned else (-2**31, 2**31 - 1)
        return min(max(int(m.group(1)), lo), hi)           # saturation (inferred)

    def read_float(self, section: str, key: str, default: float) -> float:
        v = self._without_comment(section, key)
        if v is None:
            return default
        best: Optional[tuple[int, float]] = None
        for sep in (".", ","):                             # '%.;%f' then '%,;%f', longest match wins
            e = re.escape(sep)
            m = re.match(rf"[ \t]*([+-]?(?:[0-9]+(?:{e}[0-9]*)?|{e}[0-9]+)(?:[eE][+-]?[0-9]+)?"
                         rf"|[+-]?(?:inf|nan))", v, re.IGNORECASE)
            if m and (best is None or m.end() > best[0]):
                best = (m.end(), float(m.group(1).replace(",", ".")))
        if best is None:
            raise ConfigTypeInvalid(f"[{section}] {key} = {v!r}")
        return best[1]

    def read_bool(self, section: str, key: str, default: bool) -> bool:
        v = self._without_comment(section, key)
        if v is None:
            return default
        if v in ("0", "1"):
            return v == "1"
        m = re.match(r"[ \t]*(true|false)", v, re.IGNORECASE)   # case rule (inferred)
        if not m:
            raise ConfigTypeInvalid(f"[{section}] {key} = {v!r}")
        return m.group(1).lower() == "true"

    def read_path(self, section: str, key: str, default: str) -> str:
        """Windows path string. Raw value: no unquoting, no unescaping."""
        v = self.raw(section, key)
        if v is None:
            return default
        s = v.replace("/", "\\")                           # Common Path to Specific Path (Windows)
        if s.startswith("\\") and not s.startswith("\\\\") and len(s) >= 2:
            s = s[1] + ":" + s[2:]                         # "/D/alox/x" -> "D:\alox\x"
        return s                                           # "" stays "" (empty path, not the default)
```

Loader pattern: each key is read on its own, and an invalid value gives its default plus a warning. This is the recommended fix
for D4. To reproduce LabVIEW exactly, apply the chains of §5.4 instead.

```python
# Reference sketch: tolerant typed read used to build the settings objects of chapters 01, 05 and 08.
from typing import Callable


def make_getter(cfg: NIConfig, warn: Callable[[str], None]):
    readers = {"I32": cfg.read_int,
               "U32": lambda s, k, d: cfg.read_int(s, k, d, unsigned=True),
               "DBL": cfg.read_float, "BOOL": cfg.read_bool, "STR": cfg.read_str, "PATH": cfg.read_path}

    def get(section: str, key: str, default):
        kind = key.rsplit("_", 1)[-1].upper()              # the suffix gives the NI instance
        try:
            return readers[kind](section, key, default)
        except ConfigTypeInvalid as exc:
            warn(f"alox.ini: invalid value {exc}; default {default!r} used")
            return default
    return get

# get = make_getter(NIConfig.load(folders.settings / "alox.ini"), log.warning)
# port = get("hardware.camera", "port_I32", 503)
# baud = {0: 9600, 1: 19200, 2: 38400, 3: 115200}.get(get("hardware.z", "baudrate_I32", 3), 9600)
# plug = "USB" if get("hardware.camera.lowmag", "cameraconnectortype_STR", "gige") in {"USB", "USb", "UsB", "Usb", "uSB", "usb"} else "GiGe"
```

(The settings loader of [08-z-stage.md](08-z-stage.md) takes `read_int`/`read_float` callables with this behaviour.)

---

## 7. Event logs

### 7.1 The two log VIs

| Input | Type, default | `Log Server Error.vi` [Doc6 p.430–434] | `Log Server Infos.vi` [Doc6 p.1218–1221] |
|---|---|---|---|
| `Command` | enum `Camera Command`, default `Get Heartbeat` | printed (Camera server type only) | absent |
| `Camera Type` | enum, default `High Mag.` | selects the HI/LOW tag | selects the HI/LO tag |
| `Server Type` | enum: `Camera` (default), `Monochromator`, `XY Stages`, `Z Stage` (order **(unknown)**) | tag and file name | tag and file name |
| `Log Ref` | reference to a string indicator | on-screen log | on-screen log |
| `Extra String` | string | printed in parentheses | printed in parentheses |
| `Max Lines` | I32, default 1000 (callers wire `#Lines`) | on-screen trimming | on-screen trimming |
| `Core#` | I32 (callers wire `Core #`) | used only in a `Disabled` frame (`logs\core%d`) → **no effect** | idem |
| `error in` → `error out` | error cluster | logs **only if `status = TRUE`**; `error out = error in` | logs **always**; `error out = error in` |

This server uses only `Server Type = "Camera"` (camera loop) and `"Z Stage"` (Z loop). The `Monochromator` and `XY Stages` branches
show that the VIs are shared with other ALOX servers.

### 7.2 Algorithm

```
Log Server Error(Command, Camera Type, Server Type, Log Ref, Extra, Max Lines, Core#, error in):
  if not error_in.status: return error_in
  frame 0 "Add log line to log indicator":
     tag  ← Server Type == "Camera" ? (Camera Type == "Low Mag." ? "ERR (LOW MAG)-" : "ERR (HI MAG)-")
           : {"Z Stage": "ERR (Z STAGE)-", "Monochromator": "ERR (MONO WL)-", "XY Stages": "ERR (XY STAGE)-"}
             .get(Server Type, "ERR -")
     stamp ← FormatDateTime("%d/%m/%Y - %H:%M:%S%3u - " + tag, now)          # local time (inferred)
     name  ← Server Type == "Camera" ? item name of Command : ""
     line  ← stamp + name + Format("(%d) : %s (%s)", code, source, Extra)
     old   ← Log Ref.Value ; if number_of_lines(old) > Max Lines: old ← ""
     Log Ref.Value ← old + EOL + line + EOL ; Clear Errors
  frame 1 "Add log line to daily log file":
     folder ← tools_alox_shared_vision_folder() \ "logs"                      # Disabled frame would add "core<Core#>"
     file   ← RemoveChars(FormatDateTime("%y%m%d_<stem>_events", now), ".", ",") + ".log"
     f ← Open/Create/Replace File(folder\file, "open or create", "read/write")
     Set File Position(f, from = start, offset 0)                            # DEFECT: should be "end"
     Write to Text File(f, line + EOL) ; Close File(f) ; Clear Errors
  return error_in

Log Server Infos(...): same, except: no status test; no command name; tags "INFO (HI)-" when Camera Type == "Low Mag.",
  "INFO (LO)-" otherwise (swapped, DEFECT), "INFO -" for non-camera servers; folder from tools_alox_shared_data_folder();
  Set File Position(from = end) → append.
```

The file chain starts from its own error wire: a write failure never changes `error out`. Both `Clear Errors` calls swallow every
indicator and file error, so a failed write loses the line silently [Doc6 p.432, 1220].

### 7.3 File names, folders and rotation

| Server Type | File stem | Error log (`Log Server Error`) | Info log (`Log Server Infos`) |
|---|---|---|---|
| `Camera` (and any unlisted value) | `camera` | `<vision>\logs\<yymmdd>_camera_events.log` | `<data>\logs\<yymmdd>_camera_events.log` (unused here) |
| `Z Stage` | `z` | `<vision>\logs\<yymmdd>_z_events.log` | `<data>\logs\<yymmdd>_z_events.log` |
| `Monochromator` | `wavelength` | (other servers) | (other servers) |
| `XY Stages` | `xy` | (other servers) | (other servers) |

- **Rotation**: daily only, through the date in the name (`%y%m%d`, local date of the write). There is no size limit and no
  retention or deletion.
- `logs` is **not created**: if it is missing, `Open/Create/Replace File` fails and the line is lost **(inferred)**.
- The file is opened and closed for every line, so several processes can share it. Several server instances (one per `Core #`,
  **(inferred)**) write to the same daily file.
- Line terminator: LabVIEW's End-of-Line constant, CRLF on Windows **(inferred)**. Text in the system code page **(inferred)**.
- Effect of the offset-0 defect: every error line overwrites the first `len(line)+2` bytes. The file keeps the length of its longest
  content, the newest line sits on top, and the remains of older lines follow. Example after two errors (the second line is shorter):
  `…ERR (Z STAGE)-(1) : short (x)\r\nut of range (Move Absolute)\r\n`.

### 7.4 Line format

```
error line : <dd/mm/YYYY> - <HH:MM:SS><sep><mmm> - <tag><command name>(<code>) : <source> (<extra>)
info line  : <dd/mm/YYYY> - <HH:MM:SS><sep><mmm> - <tag>(<code>) : <source> (<extra>)
```

| Field | Content |
|---|---|
| `<sep><mmm>` | `%3u`: decimal separator **(inferred: system locale, `.` or `,`)** plus 3 digits |
| `<tag>` | §7.2; note there is no space between the tag and what follows |
| `<command name>` | Camera server only: the `Camera Command` item name, e.g. `Cam Setup`. If a caller leaves `Command` unwired, it prints the default `Get Heartbeat`. |
| `<code>`, `<source>` | of the error cluster (already decorated by the caller, [03-command-reference.md](03-command-reference.md) §4) |
| `<extra>` | `Extra String` of the caller, e.g. `Setup`, `Move Absolute` ([03-command-reference.md](03-command-reference.md)) |

Examples (times illustrative):

```
29/10/2026 - 16:31:05.123 - ERR (Z STAGE)-(9405) : Z move out of range (Move Absolute)
29/10/2026 - 16:31:07.456 - ERR (LOW MAG)-Cam Setup(<code>) : <driver source>Cam Setup (Setup)
29/10/2026 - 16:32:00.789 - INFO -(0) :  (Move Absolute: Z Reference resetted)
```

The only `Log Server Infos` calls are in `Z Move Absolute` / `Z Move Relative` when the axis was re-referenced silently
(`Extra String` = `"Move Absolute: Z Reference resetted"` / `"Move Relative: Z Reference resetted"`, `Server Type = "Z Stage"`)
[main VI frames d232–d239]. With no error the line shows `(0) :  (` with two spaces, because the source is empty.

### 7.5 On-screen log (front panel)

| Object | Behaviour |
|---|---|
| `Log Ref` target | Probably the string indicator **`String`**, not `Log` **(inferred)** ([01-architecture.md](01-architecture.md) §8). `Cam Set Led` overwrites `String` with `"us"`/`"gi"`. |
| Trimming | Before appending, if the text has **more than `Max Lines` lines** (`[log] maxlines_I32`, default 1000), it is cleared entirely. This is not a rolling window. Each entry adds `EOL + line + EOL`, so one blank line sits between entries, and the indicator is wiped after about 500 entries **(inferred: exact line counting not visible)**. |
| `Log` indicator, `CLEAR` button | `Log` is never written; `CLEAR` empties it [main VI frame d71]. |
| `Log` command | Stores `{Log, Path, Max Lines, Max Count}` in the `Cam Log Data` control, which nothing reads ("#TODO: Log every cameras/Z actions into files") [main VI frames d76–d77]. No logging effect ([03-command-reference.md](03-command-reference.md)). |

A headless Python server has no indicator. If a diagnostic page is built (questionnaire Q15), keep the last `maxlines_I32` lines in a
ring buffer.

### 7.6 Who logs

- Camera and Z commands call `Log Server Error` at the end of each handler, with a per-command `Extra String`. Some calls leave
  `Camera Type` unwired, so a Low Mag error can be tagged `HI MAG` (e.g. the `Finished` branch of `Cam Stream Next Image`) [main VI frames d104–d115].
  Per-command details: [03-command-reference.md](03-command-reference.md).
- `Cam Get Image`, `Cam Get Images Array`, `Exit` and `Default` never log.
- Autofocus and refocus runs log through the same VI (`Extra` = `"Autofocus Run"` / `"Refocus Run"`).
- Nothing else is logged: no start-up line, no connection events, no INI problems.

### 7.7 Python reference sketch: event logs

```python
# Reference sketch: event logs with LabVIEW file names and line format.
from __future__ import annotations

import threading
from datetime import datetime
from pathlib import Path
from typing import Optional

from errors import ErrorCluster                                        # section 10.2

STEM = {"Camera": "camera", "Monochromator": "wavelength", "XY Stages": "xy", "Z Stage": "z"}
ERR_TAG = {"Monochromator": "ERR (MONO WL)-", "XY Stages": "ERR (XY STAGE)-", "Z Stage": "ERR (Z STAGE)-"}


class EventLogs:
    def __init__(self, vision_dir: Path, data_dir: Path, *, decimal_sep: str = ".",
                 append_errors: bool = True, fix_info_labels: bool = True, encoding: str = "cp1252"):
        self.vision_dir, self.data_dir = Path(vision_dir), Path(data_dir)
        self.decimal_sep, self.append_errors = decimal_sep, append_errors
        self.fix_info_labels, self.encoding = fix_info_labels, encoding
        self._lock = threading.Lock()                                  # LabVIEW VIs serialise callers (inferred)

    def _stamp(self, now: datetime, tag: str) -> str:                  # "%d/%m/%Y - %H:%M:%S%3u - <tag>"
        return f"{now:%d/%m/%Y - %H:%M:%S}{self.decimal_sep}{now.microsecond // 1000:03d} - {tag}"

    def _write(self, folder: Path, server_type: str, line: str, now: datetime, append: bool) -> None:
        name = f"{now:%y%m%d}_{STEM.get(server_type, 'camera')}_events".replace(".", "").replace(",", "") + ".log"
        path = folder / "logs" / name
        data = (line + "\r\n").encode(self.encoding, errors="replace")
        with self._lock:
            try:
                path.parent.mkdir(parents=True, exist_ok=True)         # deviation: LabVIEW does not create it
                if append:
                    with open(path, "ab") as f:
                        f.write(data)
                else:                                                  # LabVIEW Log Server Error: offset 0
                    with open(path, "r+b" if path.exists() else "w+b") as f:
                        f.seek(0)
                        f.write(data)
            except OSError:
                pass                                                   # LabVIEW: Clear Errors (report in a diagnostic log)

    def error(self, err: ErrorCluster, server_type: str, extra: str, *,
              command: Optional[str] = "Get Heartbeat", camera_type: str = "High Mag.") -> ErrorCluster:
        """Log Server Error.vi: writes only when status is TRUE; returns err unchanged."""
        if not err.status:
            return err
        if server_type == "Camera":
            tag = "ERR (LOW MAG)-" if camera_type == "Low Mag." else "ERR (HI MAG)-"
        else:
            tag = ERR_TAG.get(server_type, "ERR - ")                   # trailing space of the default tag: (unknown)
        now = datetime.now()
        name = (command or "") if server_type == "Camera" else ""
        line = self._stamp(now, tag) + name + f"({err.code}) : {err.source} ({extra})"
        self._write(self.vision_dir, server_type, line, now, append=self.append_errors)
        return err

    def info(self, err: ErrorCluster, server_type: str, extra: str, *,
             camera_type: str = "High Mag.") -> ErrorCluster:
        """Log Server Infos.vi: always writes (appends); returns err unchanged."""
        if server_type == "Camera":
            low = camera_type == "Low Mag."
            if self.fix_info_labels:
                tag = "INFO (LO)-" if low else "INFO (HI)-"
            else:
                tag = "INFO (HI)-" if low else "INFO (LO)-"            # LabVIEW: labels swapped
        else:
            tag = "INFO -"
        now = datetime.now()
        line = self._stamp(now, tag) + f"({err.code}) : {err.source} ({extra})"
        self._write(self.data_dir, server_type, line, now, append=True)
        return err
```

Keep these files byte-compatible in name and line format, because other ALOX tools may read them (open question 3). Send anything
new to a **separate** diagnostic log, through Python's `logging` module: INI warnings, resolved folders, connections, exceptions with
tracebacks. Proposed location: `<vision>\logs\<yymmdd>_camera_server_diag.log`.

---

## 8. LabVIEW error-handling semantics

### 8.1 Propagation rules

| Rule | Detail | Python consequence |
|---|---|---|
| Error-in gating | NI and ALOX sub-VIs wrap their body in a `Case` on `error in`. In `Error` they do nothing, return **default outputs** (0, FALSE, empty, the `default value` of `Read Key`) and pass the error on. | A chain of wired nodes becomes one `try` block. The first failure skips the rest. |
| Primitives without error terminals | `Build Path`, `Select`, arithmetic, `Flatten To String`… always run. | Compute them unconditionally. |
| Unwired `error out` | The error is ignored, unless the VI property "Automatic error handling" shows a dialog. That property is not printed **(unknown)**. | Explicit, logged suppression. |
| Unwired `error in` | The node starts from "no error". Most command handlers do this, so each command starts clean. | A fresh `try` per command. |
| Warnings | `status = FALSE`, `code ≠ 0`: not an error. Nodes run, `Log Server Error` ignores them, decoration skips them. | Never raise. Carry them in the cluster. |
| `Merge Errors` | Returns the first error in input order, or else the first warning **(inferred: NI documented behaviour)**. | `merge_errors()` |
| `Clear Errors` [Doc6 p.57–59] | With `specific error code to clear = 0` (default): returns no error. Otherwise it clears only a matching code (`specific error cleared? = TRUE`) and passes any other error. | `except` on a precise type/code, never a bare `except` |
| `Error Cluster From Error Code` [Doc5 p.3–5] | Code 0 → `error in` unchanged. `error in` already an error → unchanged ("An error in trumps any error code input"). A warning on top of an existing warning keeps the existing one. Otherwise `{status = NOT is warning, code, source = caller VI name + extra}`. `extra` = `"<ERR>\n" + message` (trailing whitespace trimmed) when a message is given, and `"<APPEND>\n"` + call chain only when `show call chain?` is TRUE (unused here). | `error_cluster_from_error_code()` |
| Uninitialised shift registers | An error kept in an uninitialised shift register persists between calls of a non-reentrant VI (autofocus/refocus sessions). | Per-job state ([06-autofocus-refocus.md](06-autofocus-refocus.md)) |

`<ERR>` and `<APPEND>` are markers for NI's error-handler dialogs. They reach the client verbatim inside `source`, e.g.
`HW TCP Extract TCP Data.vi<ERR>\nExtra Data Found` [Doc6 p.1153]. The exact caller-name text (library prefix or none) is
**(unknown)**.

### 8.2 Error patterns of this server

| Pattern | Where | Detail |
|---|---|---|
| Decorate then log then reply | every command handler | `source += <suffix>` only when `status = TRUE`; `Log Server Error`; then the reply. Order variants: [03-command-reference.md](03-command-reference.md) §4. |
| Error-only replies probably not sent | `Enqueue Send Error Cluster Reponse.vi` | [02-tcp-protocol.md](02-tcp-protocol.md) §8.4 (Q17) |
| `error out` front-panel object | stage 1 resets it; handlers write it | appended every camera-loop iteration to the unbounded `Array` history ([01-architecture.md](01-architecture.md)) |
| INI reads | stage 2 | all errors ignored, defaults used (§5.4) |
| Folder VI errors | stage 2 | `error out` unwired [main VI frame d3] |
| PI macro check | stage 2, frame 4 | `Clear Errors` (all) [main VI frames d59–d60] |
| TCP loop timeout case | `Enqueue Read` | `Clear Errors` [main VI frame d71] |
| IMAQdx "Invalid camera session" | `Cam Get Image`, `Cam Get Acquisition Status`, `Cam Stop`, camera-loop timeout case | `Clear Errors(−1074360316)` only [main VI p.14, 17, 19, 29] |
| `Cam Start` | TriggerMode write | cleared before `Start Acquisition` [main VI frames d90–d95] |
| `Cam Software Trigger` | all errors | `Clear Errors` twice: always replies OK [main VI frame d119] |
| Autofocus / refocus | `VEL?`, `VEL`, `POS?`, overlay, graph errors | cleared ([06-autofocus-refocus.md](06-autofocus-refocus.md)) |
| Log VIs | their own file and indicator errors | `Clear Errors` [Doc6 p.432, 1220] |
| Non-error with text | `get_image_state_machine` while waiting | `{FALSE, 0, "lastbuffer:…/ buffercount:…/ buffer t-1:…"}` [Doc6 p.555] |

---

## 9. Error code register

### 9.1 Codes defined by the ALOX code

| Code | `source` text | Status | Raised by | Reaches | Ref. |
|---|---|---|---|---|---|
| **9009** | `"get image error (timeout)"` + `"lastbuffer:<n>/ buffercount:<n>/ buffer t-1:<n>"` | TRUE | `alox_camera_server_get_image_state_machine.vi`, no new buffer within `Acquisition Timeout (ms)` | image reply of `Cam Stream Next Image`, error log | [Doc6 p.555]; [05](05-camera-acquisition.md) |
| **−9011** | `"Invalid Type"` | TRUE | `alox_camera_server_simulation.vi`, unsupported PNG pixel type | image reply (simulation), error log | [Doc6 p.921]; [05](05-camera-acquisition.md) |
| **9031** | `"'refocus' warning (joystick enabled)"` | **TRUE** (despite "warning") | refocus, joystick active (`JON?`) | refocus `Error` state, error log (`Cam Refocus Run` has no reply) | [Doc6 p.1034]; [06](06-autofocus-refocus.md) |
| **9401** | `"z stage get position timeout"` | TRUE | autofocus Fast mode (30 000 ms) [Doc6 p.608]; refocus `Check Position` / `Wait Focus` (5000 ms) [Doc6 p.1035, 1037] | AF/RF `Error` state, error log | [06](06-autofocus-refocus.md) |
| **9405** | `"Z move out of range"` | TRUE | `Z Move Absolute` / `Z Move Relative`, target outside `[min(zmin, zmax), max(zmin, zmax)]`. It replaces any earlier error. | ACK reply (probably not sent), Z error log | [main VI frames d233, d236]; [08](08-z-stage.md) |
| **−1** | `"<caller>.vi<ERR>\nExtra Data Found"` | TRUE | `HW TCP Extract TCP Data.vi`, heartbeat frame with a payload, via `Error Cluster From Error Code` | TCP loop: connection closed and re-listen | [Doc6 p.1153]; [02](02-tcp-protocol.md) §5.3 |

Note on −9011: there is a conflict between the source reports. One analysis gives 9011. The printed constant reads **−9011**
[Doc6 p.921], and that is the value to reproduce. LabVIEW reserves 5000–9999 and −8999…−8000 for user codes, so −9011 is outside the
user ranges (probably a sign slip, see D16). The other ALOX codes follow a pattern: 90xx for camera/vision, 94xx for the Z stage
**(inferred)**.

### 9.2 Vendor and LabVIEW codes the server handles or passes through

| Code | Meaning | Handling | Ref. |
|---|---|---|---|
| −1074360316 (0xBFF69004) | NI-IMAQdx error, commented "Invalid camera session" | cleared where listed in §8.2; otherwise passed through, decorated | [main VI p.14, 17, 19, 29] |
| other IMAQdx / NI Vision codes (negative, around −1074 million) | driver errors | passed through, decorated, logged, replied | [05](05-camera-acquisition.md) |
| 56 | LabVIEW TCP timeout | `Read` re-queued; `Listen` retried | [main VI frame d63]; [02](02-tcp-protocol.md) §4.4 |
| any other TCP read code (66, 62 **(inferred)**) | connection lost | `TCP Close` then `Listen` | [main VI frame d64] |
| 114 | NI `TCP Listen` listener conflict (internal to `Internecine Avoider.vi`) | not reachable with one port | [Doc6 p.1143] |
| 1122 / 1 **(inferred)** | queue destroyed / invalid refnum | ends the TCP loop at shutdown | [01](01-architecture.md) §1.2 |
| 91 **(inferred)** | `Variant To Data` type mismatch, where its error is wired | replied for those commands | [03](03-command-reference.md) §8 |
| "Config type invalid" (code **(unknown)**) | `Read Key` conversion failure | ignored (§5.4) | [Doc6 p.1249–1290] |
| 7 **(inferred)** | "File not found" from `Load.vi` | not reachable (files are created) | [Doc6 p.254] |
| PI controller codes > 0 (e.g. 10 "Controller was stopped by command") | read by `ERR?`; source contains "Controller error" plus the GCS text | passed through | [08](08-z-stage.md) |
| 100 | PI LabVIEW driver error (no axis, no answer on `ERR?`, …) | passed through | [08](08-z-stage.md) |
| −7 | PI timeout (serial read 5 s, wait-for-stop 300 s) | passed through | [08](08-z-stage.md) |
| NI-VISA codes (e.g. −1073807343) | serial port open errors | passed through | [08](08-z-stage.md) |

Positive codes can collide between families. PI controller code 7 ("Position out of limits") equals LabVIEW code 7 ("File not
found"). Only the `source` text tells them apart. The vendor tables are not copied here. PI's table ships with `pipython`
(`pipython.pidevice.gcserror`), and NI codes are in the NI-IMAQdx / LabVIEW error documentation.

---

## 10. Python error strategy (wire-compatible)

### 10.1 Principles

| LabVIEW construct | Python rule |
|---|---|
| Error wire through N nodes | One `try` block; the first exception skips the rest. |
| Sub-VI "Error" case returning defaults | Do not emulate defaults inside the code. Where a caller relies on the default value (e.g. `Z Get Position` replying 0.0 on error), produce it explicitly at the handler. |
| `Clear Errors` (all) | `except <SpecificError>: log.debug(...)`. List the swallowed families explicitly (§8.2). |
| `Clear Errors(code)` | `except CameraError as e: if e.code != Code.IMAQDX_INVALID_SESSION: raise` |
| Unwired `error out` | The same as `Clear Errors`, but always log at WARNING (it was probably unintended). |
| Warning (`status = FALSE`, `code ≠ 0`) | Never raised; returned in the `ErrorCluster`. |
| Error cluster on the wire / in logs | Built **only** at the boundaries: the reply VIs ([02](02-tcp-protocol.md) §8), `EventLogs` (§7.7) and the image metadata. |
| Codes and source texts | Exactly as in §9 and in the per-command decorations of [03-command-reference.md](03-command-reference.md). Never translate codes into new ones. |
| Long-running loops | Catch per message, convert, reply/log, continue. Only `Exit` stops a loop. |
| Vendor exceptions | Converted at the hardware wrapper into the matching subclass, keeping the vendor code (§10.4). |

### 10.2 Classes and helpers

The sibling chapters use local names in their sketches: `CamError` ([05](05-camera-acquisition.md)), `LVError`
([03](03-command-reference.md), [06](06-autofocus-refocus.md)), `StageError` ([08](08-z-stage.md)), `FocusMetricError`
([07](07-focus-metrics.md)), `ProtocolError` ([02](02-tcp-protocol.md)). In the implementation they map onto this one module
(`errors.py` in [10-migration-plan.md](10-migration-plan.md) §3.2).

```python
# Reference sketch: LabVIEW error clusters and the Python exception hierarchy (errors.py).
from __future__ import annotations

import struct
from dataclasses import dataclass, replace
from enum import IntEnum


class Code(IntEnum):
    """Codes the server produces or tests (section 9)."""
    EXTRA_DATA_FOUND = -1               # HW TCP Extract TCP Data
    GET_IMAGE_TIMEOUT = 9009            # get_image_state_machine
    SIM_INVALID_TYPE = -9011            # simulation (negative as printed)
    REFOCUS_JOYSTICK = 9031             # refocus
    Z_POSITION_TIMEOUT = 9401           # autofocus / refocus
    Z_OUT_OF_RANGE = 9405               # Z Move Absolute / Relative
    TCP_TIMEOUT = 56                    # LabVIEW TCP Read timeout
    IMAQDX_INVALID_SESSION = -1074360316
    PI_DRIVER = 100                     # PI LabVIEW driver error
    PI_TIMEOUT = -7


@dataclass(frozen=True)
class ErrorCluster:
    """LabVIEW error cluster {status, code, source}; the only form that crosses the wire or reaches a log."""
    status: bool = False
    code: int = 0
    source: str = ""                    # kept even when status is FALSE (it is flattened as-is)

    @property
    def is_warning(self) -> bool:
        return not self.status and self.code != 0

    def flatten(self, encoding: str = "cp1252") -> bytes:
        """Big-endian: U8 status, I32 code, I32 length + bytes (02-tcp-protocol.md section 6)."""
        src = self.source.encode(encoding, errors="replace")
        return struct.pack(">?ii", self.status, self.code, len(src)) + src


NO_ERROR = ErrorCluster()


class AloxError(Exception):
    """Base exception. Carries a LabVIEW-compatible code and source text."""

    def __init__(self, code: int, source: str):
        super().__init__(f"{int(code)}: {source}")
        self.code, self.source = int(code), source

    def cluster(self) -> ErrorCluster:
        return ErrorCluster(True, self.code, self.source)


class ConfigError(AloxError): ...       # INI problems (logged, never replied)
class ProtocolError(AloxError): ...     # framing / decoding (02)
class CameraError(AloxError): ...       # IMAQdx-equivalent and acquisition errors (05)
class FocusError(AloxError): ...        # autofocus, refocus, focus metrics (06, 07)
class StageError(AloxError): ...        # PI GCS controller and driver errors (08)


def decorate(err: ErrorCluster, suffix: str) -> ErrorCluster:
    """Main-VI pattern: append a suffix to source only when status is TRUE (no separator)."""
    return replace(err, source=err.source + suffix) if err.status else err


def merge_errors(*errs: ErrorCluster) -> ErrorCluster:
    """Merge Errors: first error in input order, else first warning, else no error."""
    for e in errs:
        if e.status:
            return e
    for e in errs:
        if e.code != 0:
            return e
    return NO_ERROR


def clear_errors(err: ErrorCluster, specific_code: int = 0) -> tuple[ErrorCluster, bool]:
    """Clear Errors.vi: returns (error out, specific error cleared?)."""
    if specific_code == 0:
        return NO_ERROR, False
    if err.code == specific_code:
        return NO_ERROR, True
    return err, False


def error_cluster_from_error_code(code: int, message: str = "", *, is_warning: bool = False,
                                  error_in: ErrorCluster = NO_ERROR, caller: str = "") -> ErrorCluster:
    """Error Cluster From Error Code.vi with show call chain? = FALSE."""
    if code == 0 or error_in.status or (is_warning and error_in.code != 0):
        return error_in
    extra = f"<ERR>\n{message}\n\n".rstrip(" \t\r\n") if message else ""
    return ErrorCluster(not is_warning, code, caller + extra)
```

Example: a whole Z handler in this style.

```python
# Reference sketch: handler shape (Z Move Absolute, simplified; full logic in 08-z-stage.md).
def z_move_absolute(ctx, msg) -> None:
    err = NO_ERROR
    try:
        target = msg.data.pos_mm
        lo, hi = sorted((ctx.z_settings.min_pos, ctx.z_settings.max_pos))
        if not lo <= target <= hi:
            raise StageError(Code.Z_OUT_OF_RANGE, "Z move out of range")
        ctx.stage.mov(target)
    except AloxError as exc:
        err = exc.cluster()
    ctx.reply_error_cluster(msg, err)                  # honours the compatibility switch of 02 §11.3
    ctx.logs.error(err, "Z Stage", "Move Absolute")
```

### 10.3 Boundary rules

1. **Replies**: the handler builds the cluster (decorated as in [03-command-reference.md](03-command-reference.md)) and hands it to
   the reply encoder. Whether an error-only ACK is sent follows the switch of [02-tcp-protocol.md](02-tcp-protocol.md) §11.3.
2. **Logs**: `EventLogs.error()` receives the same decorated cluster and the per-command `Extra String`. Unexpected exceptions
   (bugs) become a cluster with a generic code (§10.5) and a traceback in the diagnostic log; the loop goes on.
3. **Image metadata and typed replies**: errors travel as data inside `{value, error}` ([02-tcp-protocol.md](02-tcp-protocol.md) §8.3).
   On error the value is the LabVIEW default (0, FALSE, empty).
4. **Start-up**: INI and folder problems never stop the server (LabVIEW behaviour). They go to the diagnostic log as warnings.

### 10.4 Mapping vendor errors

| Source | Python exception | Cluster code | Cluster source |
|---|---|---|---|
| `pipython.GCSError` (attribute `val`) | `StageError` | `val` unchanged (controller > 0, interface/DLL < 0) | as specified in [08-z-stage.md](08-z-stage.md) (`"Controller error"` + GCS text) |
| serial / VISA failures (`OSError`, `serial.SerialException`) | `StageError` | per [08-z-stage.md](08-z-stage.md) (−7 timeout; port errors) | idem |
| camera SDK exceptions | `CameraError` | the NI-IMAQdx code when the server logic depends on it (−1074360316 for "camera not open"); otherwise a code from §10.5 | SDK message + LabVIEW-style suffixes |
| NumPy / OpenCV failures in focus metrics | `FocusError` | §10.5 (NI codes **(unknown)**, [07](07-focus-metrics.md)) | short text |
| file I/O (refocus PNG, flat-field load) | `CameraError` / `FocusError` | §10.5 | path + OS message |

### 10.5 Codes for conditions LabVIEW reports with NI codes we cannot reproduce

Some failures surface in LabVIEW as NI-internal codes that a Python stack cannot reproduce: NI Vision file errors, IMAQdx open
failures with no SDK equivalent, and unexpected exceptions. **Proposal (to confirm, open question 5):** reserve one block inside
LabVIEW's user range that ALOX does not use, e.g. **9500–9599**, and define the codes in `Code`. Examples: 9500 internal error
(unexpected exception), 9501 image file read/write failed, 9502 camera SDK error without an IMAQdx equivalent, 9503 focus metric
failed. Every observed ALOX code (§9.1) keeps its value. NI-IMAQdx/PI codes keep theirs wherever the Python stack can know them.

---

## 11. Defects and oddities in the LabVIEW implementation

| # | Behaviour | Evidence | Impact | Recommendation |
|---|---|---|---|---|
| D1 | `tools_windows_user_documents_folder.vi` returns `[folders] settings`; the stage-2 label still names `C:Users/<USER>/alox/settings/alox.ini`. | [Doc6 p.411]; [main VI p.34] | misleading documentation | **Fix** names in the new code (`settings_folder()`) |
| D2 | A missing `alox_setup.ini` or `alox.ini` is created empty (`create file if necessary` unwired = TRUE). | [Doc6 p.254, 411]; [main VI frame d3] | an empty file hides a wrong path; the defaults are used silently | **Fix**: do not create; warn; use defaults |
| D3 | An empty `[folders]` value yields an empty (relative) path instead of the default. | [Doc6 p.1249] | `alox.ini` not found → all defaults; logs lost | **Fix**: treat empty as missing, warn |
| D4 | INI read errors are ignored, and chained errors force later keys to their defaults (§5.4). | [main VI frames d4–d61] | silent misconfiguration | **Fix**: per-key default plus warning (also 01 A18) |
| D5 | `frameratemax_DBL` is never read for High Mag. | [main VI frames d5–d25] | HM `Max Framerate` from an unknown constant | see [05](05-camera-acquisition.md); **ask** |
| D6 | `cameraconnectortype_STR` recognises 6 of the 8 case variants of "usb" (`uSb`, `usB` → GiGe). | [main VI frames d24, d46] | surprising for hand-edited files | **Fix**: case-insensitive `usb` (no realistic file depends on it) |
| D7 | `baudrate_I32` is an index; a literal baud rate silently gives 9600. | [main VI frames d51–d54] | wrong serial speed | **Reproduce** the index; warn when the value is not in 0–3 |
| D8 | `Cam Set Led` calls `Write Key(lightStatusInverter_BOOL)` with no value, on a closed refnum, with no save. | [main VI frame d195; p.26] | none (dead code) | **Do not reproduce**; **ask** whether persistence was intended (Q23) |
| D9 | `alox_setup.ini` is re-opened (and possibly re-created) for every log line. | [Doc6 p.428, 432, 1220, 1224] | needless disk I/O; folders can change at run time | **Fix**: resolve once at start-up |
| D10 | `Log Server Error` writes each line at offset 0 (`Set File Position(from = start)`), overwriting the head of the file. | [Doc6 p.432] | error history destroyed | **Fix**: append (Q18) |
| D11 | `Log Server Infos` swaps the labels: Low Mag → `INFO (HI)`, High Mag → `INFO (LO)`. | [Doc6 p.1220] | wrong labels (camera infos unused here) | **Fix** (Q18) |
| D12 | Errors and infos go to different folders (`<vision>\logs` versus `<data>\logs`). | [Doc6 p.432, 1220] | two places to look | **Reproduce** (configurable); **ask** whether intended |
| D13 | The `logs` folder is never created, and all file errors are cleared. | [Doc6 p.432, 1220] | lines silently lost | **Fix**: create at start-up; report failures in the diagnostic log |
| D14 | `Core #` (`[general] core_I32`) only feeds disabled code; all instances share one daily file. | [Doc6 p.432, 1220] | interleaved lines from several instances | **Reproduce** naming; **ask** about per-core files |
| D15 | Unwired `Camera Type` / `Command` inputs: Low Mag errors tagged `HI MAG`; unwired `Command` would print `Get Heartbeat`. | [main VI frames d104–d115]; [Doc6 p.431] | misleading log lines | **Fix** when the camera is known |
| D16 | Custom code −9011 is negative, outside LabVIEW's user ranges, unlike the other ALOX codes. | [Doc6 p.921] | none functional | **Reproduce** −9011 on the wire |
| D17 | Code 9031 is called a "warning" but has `status = TRUE`. | [Doc6 p.1034] | refocus aborts | **Reproduce** |
| D18 | `get_image_state_machine` returns `{FALSE, 0, "<diagnostic text>"}`: "no error" with a source text. | [Doc6 p.555] | a reply could carry the text | **Reproduce** (keep `source` verbatim) |
| D19 | The on-screen log is wiped completely past `Max Lines`, adds blank lines, and its target (`String`) is overwritten by `Cam Set Led`. `Log` is never written. | [Doc6 p.432]; [main VI frame d71] | no reliable on-screen history | **Fix** (ring buffer) or drop if headless |
| D20 | "Automatic error handling" state unknown: if enabled, unwired errors would open modal dialogs in the hidden window. | not printed | possible hang of the LabVIEW server | n/a for Python; **ask** (Q30) for root-cause analysis only |
| D21 | Error-only ACK replies are probably never sent. | [Doc6 p.437] | client timeouts | see [02-tcp-protocol.md](02-tcp-protocol.md) §8.4 (Q17) |

---

## 12. Open questions

1. **Real configuration files (questionnaire Q12).** What are the actual `alox_setup.ini` and `alox.ini`? This tells us their
   encoding, their quoting style, and whether they hold keys this server does not read. It also lets us run a golden test of the
   reader against them.
2. **Is `alox.ini` shared with other ALOX programs** (Vision Core, the monochromator or XY servers suggested by the log VI)? This
   decides where the Python-only settings go: a separate file such as `<settings>\alox_camera_server.ini` (proposed) or a
   `[compat]` section in `alox.ini` ([10-migration-plan.md](10-migration-plan.md) §6). It also confirms that the server must never
   write `alox.ini`.
3. **Do other tools read the event logs?** If so, the file names, the folders (D12) and the line format must stay byte-exact,
   including the `%3u` decimal separator of the production machine's locale. And may D10/D11 be fixed (Q18)?
4. **Does the client test error codes or parse `source`?** For example, does it check for 9405 or 9009, or display text with
   `<ERR>`? This decides how strictly decorations must match and whether a missing NI code can be replaced.
5. **May the Python server use the code block 9500–9599 (§10.5)** for conditions it cannot express with NI codes? Or does ALOX
   already use those codes elsewhere?
6. **Does more than one server instance (`Core #`) run on one machine?** If so, were per-core log folders intended (D14)?
7. **Installation folder and target OS (Q13).** Where does the executable live, and is that folder writable? It affects D2. On
   Linux, how should the `D:\…` defaults map?
