# Data types (typedefs) and serialisation

**Summary.** The values that cross the TCP link are LabVIEW **type definitions** (typedefs), and so is most of the server's internal state. A typedef is a named cluster (a record) or enum stored in a `.ctl` file. This chapter is the reference for all of them. For each typedef it gives:

- the exact field labels in **cluster order**, which is the order that defines the bytes;
- the LabVIEW type and flattened size of each field, and the defaults;
- the evidence for both the types and the order.

It covers the inbound envelope `Camera Queue Data`, the 28 command-payload typedefs and 3 bare payload types, the reply clusters, every enum, and the internal settings and session clusters. It also records the ambiguities with the decision taken and how to confirm each one. These are the field order of `Camera Init Data`, of the autofocus/refocus parameters and of two Z clusters, and the representation and codes of every enum. Section 9 gives Python dataclasses and a flatten/unflatten reference sketch that plugs into the wire codec of [02-tcp-protocol.md](02-tcp-protocol.md).

**Sources.** Typedef front panels in Labview Document 6:

- **Envelope and command payloads:** `typedef_ext_camera_mgmt_queue_data.ctl` [Doc6 p.285–288] and `typedef_ext_camera_log_data.ctl` [Doc6 p.289–292]; the camera payload typedefs `typedef_ext_camera_*_data.ctl` [Doc6 p.341–404, 540–543]; `typedef_ext_camera_autofocus_data.ctl` [Doc6 p.599–602] and `typedef_ext_camera_refocus_data.ctl` [Doc6 p.612–615]; the Z payload typedefs `typedef_ext_z_*_data.ctl` [Doc6 p.277–284, 293–296, 325–340, 447–450].
- **Enums:** `typedef_ext_camera_camera_command.ctl` [Doc6 p.422–425], `alox_typedef_camera_type.ctl` [Doc6 p.616–619], `alox_typedef_autofocus_speed.ctl` [Doc6 p.587–590], `alox_typedef_autofocus_method.ctl` [Doc6 p.620–623], `alox_typedef_refocus_method.ctl` [Doc6 p.624–627], `typedef_ext_camera_get_image_state.ctl` [Doc6 p.632–635], `typedef_ext_camera_autofocus_state.ctl` [Doc6 p.636–639], `typedef_ext_camera_refocus_state.ctl` [Doc6 p.640–643], `typedef_ext_camera_camera_command_type_LED.ctl` [Doc6 p.887–890], `HW TCP Datatype.ctl` [Doc6 p.1163–1166] and `HW TCP Server Command.ctl` [Doc6 p.1044–1047].
- **Replies and queues:** `HW TCP Response Data.ctl` [Doc6 p.1159–1162], `HW TCP Server Queue.ctl` [Doc6 p.1040–1043], and `Wait for STR / Boolean / DBL / U32 / Image / DBL 2D Array Notifier Data.ctl` [Doc6 p.439–442, 451–454, 459–462, 536–539, 563–566, 1048–1051].
- **Internal clusters:** `alox_typedef_ext_camera_properties.ctl` [Doc6 p.648–651], `alox_typedef_ext_camera_session.ctl` [Doc6 p.628–631], `alox_typedef_ext_zstage_session.ctl` [Doc6 p.644–647], `alox_typedef_ext_zstage_properties.ctl` [Doc6 p.652–655], `typedef_ext_autofocus_fast_session.ctl` [Doc6 p.595–598], `typedef_ext_camera_refocus_session.ctl` [Doc6 p.660–663] and `alox_typedef_semi_job_infos.ctl` [Doc6 p.969–972].

Usage evidence (Unbundle/Bundle nodes, `Variant To Data` type constants, wire colours, coercion dots):

- `alox_camera_server.vi` [main VI frames d0, d66, d74–d79, d90, d98, d104, d116, d119, d141, d148–d155, d162, d170, d175, d178, d181–d214, d217, d221–d248];
- `alox_camera_server_get_image_state_machine.vi` [Doc6 p.552–558] and `alox_camera_server_simulation.vi` [Doc6 p.919–923];
- `alox_camera_server_autofocus.vi` [Doc6 p.603–611] and `alox_camera_server_refocus.vi` [Doc6 p.1030–1039];
- `Log Server Error.vi` [Doc6 p.430–434], `Enqueue Send Error Cluster Reponse.vi` [Doc6 p.435–438], `Enqueue Send Image Response.vi` [Doc6 p.571–574] and `alox_z_check_axis_reference.vi` [Doc6 p.139–143].

Related chapters: [01-architecture.md](01-architecture.md), [02-tcp-protocol.md](02-tcp-protocol.md) (framing, variant, replies), [03-command-reference.md](03-command-reference.md) (what each command does), [05-camera-acquisition.md](05-camera-acquisition.md), [06-autofocus-refocus.md](06-autofocus-refocus.md), [07-focus-metrics.md](07-focus-metrics.md), [08-z-stage.md](08-z-stage.md), [09-configuration-logging-errors.md](09-configuration-logging-errors.md), [10-migration-plan.md](10-migration-plan.md).

---

## 1. Conventions and LabVIEW primer

### 1.1 Conventions

- Untagged statements are observed on the printouts. **(inferred)** marks a deduction from LabVIEW semantics or from cross-checking. **(unknown)** marks something the printouts do not show.
- Citations: `[Doc6 p.N]` is a page of "Labview Document 6", and `[main VI frame dN]` is hidden frame N of the HTML export of `alox_camera_server.vi`. `d0` is the diagram as displayed.
- **Cluster order** is the internal element order of a cluster, and it defines the flattened bytes. **Panel order** is the top-to-bottom order of the controls on the typedef's front panel, which is the only thing a printout shows.
- Field labels in `code font` are exact. They keep the original spacing and spelling, for example `Speed(mm/s)` with no space, `Z Joystick Enabled ?` and `Mesure Position`.
- Hex bytes are in wire order. Examples assume **U16 enums** **(unknown, §3.3)**.

### 1.2 LabVIEW concepts that matter for the data model

The general flattening rules (big-endian, length-prefixed strings, no padding) are in [02-tcp-protocol.md](02-tcp-protocol.md) §6. This table adds what matters for the typedefs themselves.

| LabVIEW concept | What it means here | Python |
|---|---|---|
| Cluster | An ordered record. It flattens as the concatenation of its elements in **cluster order**, with no padding. The cluster order is set in the editor ("Reorder Controls in Cluster") and is **independent of the screen layout**. | `@dataclass` plus an explicit field order (`LV_LAYOUT`) |
| Type definition (`.ctl`) | A named cluster or enum shared by the sender and the receiver. The flattened **data** of a typedef is identical to that of the same untyped cluster. The typedef's name and labels appear only in the type descriptors of a variant. | one class per typedef |
| Enum | Items are numbered 0, 1, 2… in list order. The value is stored as U8, U16 (the LabVIEW default) or U32. A printout shows only the current item, never the list or the representation. Inserting an item renumbers every item after it. | `IntEnum` when codes are known, otherwise an `int` and a code table |
| Ring | A number with display labels. Its values need not be consecutive. | `int` |
| Numeric representation | Never printed. Here it is deduced from wire colours (orange = floating point, blue = integer, green = Boolean, blue-green = path, pink = string or a cluster that holds non-numeric data, brown = all-numeric cluster) and from **coercion dots**: LabVIEW draws a dot where a value enters a terminal of another numeric type. When two numbers of different types meet at an arithmetic or comparison node, LabVIEW converts to the type with more bits, and to the unsigned type when the widths are equal, so the dot sits on the input that was converted. | `struct` formats |
| Variant / `Variant To Data` | `Command Data` is a variant: a value plus its type description. Each command converts it back with `Variant To Data(type = <typedef constant>)`. When the conversion fails, the output is a default value **(inferred)**. If the error output is unwired, the command then runs with those defaults (§5.1). | decoder per command (§9) |
| Error cluster | `{status: Boolean, code: I32, source: string}` (§4.3) | tuple or exception ([09](09-configuration-logging-errors.md)) |
| Default values | Each control of a typedef has a saved default, for example `Acquisition Timeout (ms)` = 5000. A diagram constant of the typedef carries its own value. | dataclass defaults |
| Queues and notifiers | `Camera Queue Data` travels **by value** through the camera and Z queues, and re-queued messages keep every field ([01](01-architecture.md) §4). The reply typedefs (`Wait for <type> Notifier Data`) are presumably what the client's notifiers carry: single-value mailboxes keyed by `Notifier Name` **(inferred**, [02](02-tcp-protocol.md) §10). | dataclass instances |
| Uninitialised shift register | A loop register with no initial value keeps its last value from one call of the VI to the next: a static variable, shared by all callers of a non-reentrant VI. It holds state that never appears in a typedef on the wire, for example the autofocus/refocus working sessions and the `t0` of the acquisition state machine. | object attributes ([05](05-camera-acquisition.md) §6.3, [06](06-autofocus-refocus.md) §4.2, §5.2) |

### 1.3 How field types and orders were established

| Evidence | What it proves | Where it was used |
|---|---|---|
| **Positional** `Unbundle` / `Bundle` (one terminal per element, each showing a type glyph) | **Cluster order** and coarse types | `Camera Queue Data` [main VI frame d0]; Camera Properties [main VI frame d79]; Z Stage Properties [Doc6 p.141], [main VI frame d221] |
| `Unbundle By Name` / `Bundle By Name` | Labels and types of the elements shown. Grown by dragging, the node adds elements in cluster order, but the developer can pick any label on any terminal, so the order is **circumstantial** only. In the autofocus VI an `Unbundle By Name` of `Camera Autofocus Data` lists `Autofocus Speed` **above** `Camera Type` [Doc6 p.606], whereas the panel shows `Camera Type` first [Doc6 p.600]: for that typedef the node order or the panel order (or both) differs from the cluster order. | most payloads |
| Front-panel layout | Cluster order only if the cluster was auto-arranged and never reordered. Counter-example: on the Camera Properties panel [Doc6 p.649] `Camera Plug Type` is drawn second (top right) and `Camera Exposure Attribute` / `Camera Gain Attribute` are drawn above `Buffers` / `Timeout`, yet the positional `Unbundle` [main VI frame d79] has the integers at positions 1–2, four strings at 7–10 and the only enum at 18 of 20. | all typedefs |
| Diagram cluster constant | It shows the typedef's panel layout **(inferred)**, so it has the same limits as the panel. | `Cam Flip`, `Log`, reply typedefs |
| Wire colour and coercion dots | Numeric type (§1.2) | DBL / U32 / I32 decisions below |
| Type constant of `Variant To Data` (label or icon number) | Which typedef each command expects | §5.1 |

The `.ctl` icons carry numbers ("3", "7"…) that are leftovers of LabVIEW's default "Control N" icon. They have **no meaning** (`init_data` and `getimage_data` both show "3"), but they identify the typedef constants drawn as icons on the main diagram [Doc6 p.341–404].

---

## 2. Type map

| Typedef (cluster/enum label) | File, pages, saved on | Role | On the wire? |
|---|---|---|---|
| `Camera Queue Data` | `typedef_ext_camera_mgmt_queue_data.ctl` [Doc6 p.285–288], 20/05/2022 11:12 | inbound envelope; element of the camera **and** Z queues | yes (inbound payload) |
| `Response Data` | `HW TCP Response Data.ctl` [Doc6 p.1159–1162], 01/02/2019 | outbound envelope | yes |
| `Wait for <type> Notifier Data` (6 typedefs) | `synchronous calls\` [Doc6 p.439–442, 451–454, 459–462, 536–539, 563–566, 1048–1051], 2019 | `{Value, error}` inside `Response (Flattened)` | yes (nested) |
| Camera payloads (18 typedefs) | `typedef_ext_camera_*_data.ctl` [Doc6 p.289–292, 341–404, 540–543], mostly saved together on 02/11/2022 08:44 (`getimage_data`: 31/03/2021, `log_data`: 13/03/2019) | `Command Data` of camera commands | yes (variant data) |
| `Camera Autofocus Data`, `Camera Refocus Data` | [Doc6 p.599–602] 08/06/2023 16:31; [Doc6 p.612–615] 08/06/2023 14:32 | `Command Data` of `Cam Autofocus` / `Cam Refocus` (and their `Run` variants) | yes |
| Z payloads (8 typedefs) | `typedef_ext_z_*_data.ctl` [Doc6 p.277–284, 293–296, 325–340, 447–450], March 2019 | `Command Data` of Z commands | yes |
| Bare payload types | Boolean (`Show UI`), enum (`Z Do Reference`), DBL (`Z Set Min/Max Limit`) | `Command Data` without a cluster | yes |
| Enums | §6 | command codes, payload fields, internal states | partly |
| `HW TCP Server Queue` | [Doc6 p.1040–1043] | element of the internal TCP queue | no |
| Camera Properties ("Camera Settings") | `alox_typedef_ext_camera_properties.ctl` [Doc6 p.648–651], 27/09/2022 10:00 | per-camera configuration | no |
| `Camera Session` | `alox_typedef_ext_camera_session.ctl` [Doc6 p.628–631] | per-camera runtime state | no |
| `Z Session`, `Z Stage Properties` | [Doc6 p.644–647], [Doc6 p.652–655] | PI connection state, Z configuration | no |
| `Fast AF Session`, `Refocus Session`, `SEMI Job Infos` | [Doc6 p.595–598], [Doc6 p.660–663], [Doc6 p.969–972] | focus working memory, image file naming | no |

Save dates used as circumstantial evidence below: the command enum (20/05/2022 11:02) precedes `Camera Queue Data` (11:12); `typedef_ext_camera_camera_command_type_LED.ctl` shares 27/09/2022 10:00 with the properties typedef [Doc6 p.888, p.649] (§6.1); the autofocus-method enum shares 08/06/2023 14:32 with `Camera Refocus Data` [Doc6 p.621, p.613] (§5.5).

---

## 3. Flattened form of the field types

### 3.1 Primitive field types

| LabVIEW type | Flattened bytes (big-endian) | Python | Notes |
|---|---|---|---|
| Boolean | 1: `00` / `01` | `bool` | any non-zero byte reads as TRUE **(inferred)** |
| Enum | 2 if U16 (assumed), 1 if U8, 4 if U32 | `int` / `IntEnum` | §3.3 |
| I32 / U32 | 4 | `int` | I32 and U32 give the same bytes for 0…2³¹−1 |
| DBL | 8, IEEE-754 | `float` | |
| String | I32 length + bytes | `str` (cp1252, inferred) | no terminator |
| Path | `PTH0` + U32 + body, 12 + Σ(1 + component length) | `LVPath` | §3.2, **(inferred)** |
| 1-D I32 array | I32 count + count × I32 | `list[int]` | `ROI` |
| Cluster | concatenation in cluster order | dataclass | no padding |
| Variant | header, data, attribute count | see 02 | [02](02-tcp-protocol.md) §7.3 |

### 3.2 Paths (`PTH0`) **(inferred)**

Paths appear in `Cam Set FlatField`, `Cam Stream Next Image`, `Log` and `Cam Refocus`. The printouts show no flattened path. The layout below is LabVIEW's flattened-path format as reconstructed by community readers of LabVIEW data (for example pylabview).

| Offset | Size | Content |
|---|---|---|
| 0 | 4 | ASCII `PTH0` |
| 4 | 4 | U32: number of bytes that follow |
| 8 | 2 | U16 path type: 0 absolute, 1 relative, 2 "Not A Path", 3 UNC |
| 10 | 2 | U16 component count |
| 12 | … | per component: U8 length + bytes (cp1252) |

| Path (Windows text) | Flattened |
|---|---|
| empty path (control default) | `50 54 48 30 00 00 00 04 00 00 00 00` |
| `C:\f.png` | `50 54 48 30 00 00 00 0c 00 00 00 02 01 43 05 66 2e 70 6e 67` (components `C`, `f.png`) |
| `<Not A Path>` | `50 54 48 30 00 00 00 04 00 02 00 00` |
| `\\srv\share\a.png` | type 3, components `srv`, `share`, `a.png` |
| `..\x\y` | type 1, components `""`, `x`, `y` (an empty component means "up one level") |

The following details are **(inferred)** and must be validated, for example with one capture of a `Cam Set FlatField` that carries real paths, or with `Flatten To String` of path constants (§7.3):

- the drive is stored without its colon (`C`);
- the type value used for UNC paths and for `<Not A Path>`;
- the empty component for "..";
- whether an empty path has type 0.

A component cannot exceed 255 bytes. The reference sketch keeps paths as `LVPath(kind, components)`, which is lossless, and converts to text only at the edges (§9.2).

### 3.3 Enums

- **Representation: (unknown).** U16 is assumed for every enum because it is the LabVIEW default. A U8 or U32 enum shifts every following byte. The decoders then fail to consume the payload exactly, so the error is **detectable**.
- **Codes: (unknown)** except `Camera Type`. The codes are the 0-based positions in the item list. The printouts show only names, and the order of the frames of a case structure is **not** the enum order. Proof: in the camera loop the frames `Show UI`, `Log`, `Exit`, `Cam Setup` and `Cam Start` come before `Get Heartbeat` [main VI frames d74–d96], although `Get Heartbeat` is the first value of the camera routing range [main VI frame d66], which must contain `Cam Setup` ([01](01-architecture.md) §5.2.2).
- **`Camera Type`: `High Mag.` = 0, `Low Mag.` = 1** **(inferred, strong)**. The acquisition state machine compares `Camera Type = 0` to choose the custom-data text `High-Mag` (otherwise `Low-Mag`) [Doc6 p.554]. `High Mag.` is the default everywhere. The enum has **exactly these two items** **(inferred, strong)**: several case structures on `Camera Type` have only the frames `"High Mag."` and `"Low Mag."` and no `Default` [main VI frames d116–d117, d148, d162], and LabVIEW accepts an enum case structure without `Default` only if every item has a frame.
- **Out-of-range values.** What `Unflatten` / `Variant To Data` does with an out-of-range enum value is **(unknown)**. The handlers do not agree on what to do with an unexpected `Camera Type`: case structures `"High Mag.", Default` / `"Low Mag."` select High Mag [main VI frame d79]; comparisons `Camera Type == "High Mag."` select Low Mag (focus VIs [Doc6 p.606], `Cam Get Autofocus Data` [main VI frame d191]); the case structures without `Default` above have no defined behaviour **(unknown)**. `Cam Set Exposure` mixes two of them: the camera comes from a `"High Mag.", Default` case, the `Exposure Lower Limit` from a `== "High Mag."` comparison [main VI frame d0]. The re-implementation must reject out-of-range values (§10, T6).

---

## 4. Envelope, reply and queue clusters

### 4.1 `Camera Queue Data` (`typedef_ext_camera_mgmt_queue_data.ctl`)

| # | Field | LabVIEW type | Bytes | Default | Evidence |
|---|---|---|---|---|---|
| 0 | `Camera Command` | enum `typedef_ext_camera_camera_command.ctl` | 2 (U16 assumed) | `Cam Setup` (the `.ctl` itself defaults to `Get Heartbeat`) | [Doc6 p.286, p.423] |
| 1 | `Notifier Name` | string | 4 + n | `""` | [Doc6 p.286] |
| 2 | `Command Data` | variant | variable, no overall length | empty | [Doc6 p.286] |
| 3 | `Command Request Timestamp` | U32 | 4 | 0 | glyph `U32` [main VI frame d0] |
| 4 | `Command Sending Timestamp` | U32 | 4 | 0 | idem |
| 5 | `Command Receiving Timestamp` | U32 | 4 | 0 | idem; the router writes `Tick Count` into it with no coercion dot [main VI frame d66] |

**Order:** proven by the positional `Unbundle` of the TCP loop (glyphs `enum, abc, variant, U32, U32, U32`) [main VI frame d0]. It matches the panel. Wire semantics, size and parsing: [02-tcp-protocol.md](02-tcp-protocol.md) §7.1–§7.4.

Internal uses of the same cluster:

| Use | What is built | Evidence |
|---|---|---|
| Re-queue of `Cam Stream Next Image` | the dequeued element, unchanged (same notifier, data and timestamps) | [main VI frames d104–d115] |
| Start of a background focus run | `Bundle(element, Camera Command ← "Cam Autofocus Run" / "Cam Refocus Run")`. Notifier, data and timestamps are kept. | [main VI frames d181, d186] |
| `Exit` forwarded to the Z loop | `Bundle By Name(<Camera Queue Data constant>, Camera Command = "Exit")`. The other fields keep the constant's values, presumably empty **(inferred)**. | [main VI frame d78] |
| Dead code in the camera loop's timeout case | one constant "Get Image Cmd" per camera, with `Command Data = Camera Get Image Data {High Mag.}` or `{Low Mag.}` (built with a positional `Bundle`). The `Camera Command` value of the constant is not legible. | [main VI frame d217] |

### 4.2 Reply clusters

| # | `Response Data` (`HW TCP Response Data.ctl`) | Type |
|---|---|---|
| 0 | `Notifier Name` | string |
| 1 | `Response (Flattened)` | string (a flattened `Wait for <type> Notifier Data`) |
| 2–7 | `Command Request timestamp`, `TCP Command Sending timestamp`, `TCP Command Receiving timestamp`, `Pre-flattening data timestamp`, `Post-Flattening data timestamp`, `TCP Sending timestamp` | U32 ×6 |

No positional node of this cluster is printed. The panel [Doc6 p.1160] is a single neat column (two strings, then six numbers), and the diagram constant of the reply VIs shows the same layout [Doc6 p.437], which is not independent evidence (§1.3). That the two strings come first is therefore **(inferred, strong)**, and the order of the six U32 is **(inferred)** (capture check C9 in [02](02-tcp-protocol.md) §12).

| Typedef | Field 0 | Field 1 | Pages |
|---|---|---|---|
| `Wait for STR Notifier Data` | `Value` (string) | `error in (no error)` | [Doc6 p.440] |
| `Wait for Boolean Notifier Data` | `Value` (Boolean) | `error` | [Doc6 p.452] |
| `Wait for DBL Notifier Data` | `Value` (DBL) | `error in (no error)` | [Doc6 p.460] |
| `Wait for U32 Notifier Data` | `Value` (U32) | `error` | [Doc6 p.537] |
| `Wait for Image Notifier Data` | `Flattened image` (string) | `error in (no error)` | [Doc6 p.564] |
| `Wait for DBL 2D Array Notifier Data` | `Value` (2-D DBL) | `error` | [Doc6 p.1049] |

The order "value, then error" is the panel order, and the typedef constants of the reply VIs show the same layout [Doc6 p.445, 457, 465, 561, 573, 1126] **(inferred, strong)**. One captured reply of any type would confirm it. Bytes, timestamp semantics and worked examples are in [02-tcp-protocol.md](02-tcp-protocol.md) §8.

### 4.3 Error cluster

| # | Field | Type | Bytes |
|---|---|---|---|
| 0 | `status` | Boolean (TRUE = error) | 1 |
| 1 | `code` | I32 | 4 |
| 2 | `source` | string | 4 + n |

Empty: `00 00000000 00000000` (9 bytes). `status = FALSE` with `code ≠ 0` is a warning. Codes, source decoration and the Python strategy are in [09-configuration-logging-errors.md](09-configuration-logging-errors.md).

### 4.4 `HW TCP Server Queue` (internal)

| # | Field | Type | Default |
|---|---|---|---|
| 0 | `State` | enum `HW TCP Server Command` (§6.1) | `Listen` |
| 1 | `State Data` | variant: empty, or `Response Data` for `Send Response` | empty |

This is the element of the TCP loop's own queue; the cluster label is just `Cluster` [Doc6 p.1041]. It never leaves the process ([01-architecture.md](01-architecture.md) §4).

---

## 5. Command payloads (`Command Data`)

### 5.1 Command → payload map

"VTD error" says what happens to the error of `Variant To Data` when the variant holds another type:

- **ignored**: the error output is unwired, so the command runs with default values and reports success **(inferred)**.
- **propagated**: the error enters the command's error chain. Nodes that have no error input still use the defaults **(inferred)**, for example `Bundle` in `Cam Flip` or `Z Set Min Limit`.

The reply types are summarised from [03-command-reference.md](03-command-reference.md), where ACK means "error-cluster reply" (which is probably not sent when it carries an error, [02](02-tcp-protocol.md) §8.4).

| Command | `Command Data` type | VTD error | Reply | Evidence |
|---|---|---|---|---|
| `Get Heartbeat` | not read | — | U32 | [main VI frame d96] |
| `Show UI` | **bare Boolean** | ignored | ACK | [main VI frame d74] |
| `Log` | `Cam Log Data` | propagated | ACK | [main VI frame d76] |
| `Exit` | not read | — | none | [main VI frame d78] |
| `Cam Setup` | `Camera Init Data` | ignored | ACK | [main VI frame d79] |
| `Cam Start` | `Camera Start Data` | ignored | ACK | [main VI frame d90] |
| `Cam Get Image` | `Camera Get Image Data` | ignored | none | [main VI frame d98] |
| `Cam Stream Next Image` | `Camera Stream Data` | ignored | Image | [main VI frame d104] |
| `Cam Get Acquisition Status` | `Camera Status Data` | ignored | Boolean | [main VI frame d116] |
| `Cam Software Trigger` | `Camera Trigger Soft Data` | ignored | ACK (always OK) | [main VI frame d119] |
| `Cam Set Exposure` | `Camera Set Exposure Data` | ignored | ACK | [main VI frame d0] |
| `Cam Set FlatField` | `Camera Ext Set FlatField` | ignored | ACK | [main VI frame d141] |
| `Cam Set Gain` | `Camera Gain Data` | ignored | ACK | [main VI frame d148] |
| `Cam Get Exposure` / `Cam Get Gain` | `Camera Get Exposure Data` / `Camera Get Gain Data` | ignored | DBL | [main VI frames d155, d162] |
| `Cam Get Images Array` | not read ("TO BE DEFINED") | — | none | [main VI frame d169] |
| `Cam Stop` / `Cam Cleanup` | `Camera Stop Data` / `Camera Release Data` | ignored | ACK | [main VI frames d170, d175] |
| `Cam Flip` | `Camera Flip Data` | propagated | ACK | [main VI frame d178] |
| `Cam Autofocus` / `Cam Refocus` | `Camera Autofocus Data` / `Camera Refocus Data` | propagated | ACK | [main VI frames d181, d186] |
| `Cam Autofocus Run` / `Cam Refocus Run` (internal) | same as the start command (re-queued element) | propagated | none | [main VI frames d205, d210] |
| `Cam Get Autofocus Data` / `Cam Get Autofocus Progress` | `Camera Get Autofocus Data` (one typedef for both) | propagated, and sent as the reply's error | DBL 2-D / DBL | [main VI frames d191, d193] |
| `Cam Set Led` / `Cam Get Led` | `Camera Set LED Data` / `Camera Get LED Data` | propagated | ACK / Boolean | [main VI frames d195, d202] |
| `Z Setup`, `Z Get Status`, `Z Get Error`, `Z Cleanup` | not read | — | ACK / Boolean / STR / ACK | [main VI frames d221, d223, d225, d248] |
| `Z Get Serial` | `Z Stage Get Serial Data` | propagated | STR | [main VI frame d222] |
| `Z Set Speed` | `Z Stage Set Speed Data` | ignored | ACK | [main VI frame d226] |
| `Z Get Speed`, `Z Get Reference`, `Z Get Position`, `Z Stop` | `{Extra Param}` ("not used, reserved") | ignored; value unused | DBL / Boolean / DBL / ACK | [main VI frames d227, d228, d231, d241] |
| `Z Do Reference` | **bare enum** (`FNL` / `FRF`) | ignored, so `FNL` runs **(inferred)** | ACK | [main VI frame d229] |
| `Z Move Absolute` | `Z Move Abs Data` | propagated | ACK | [main VI frame d232] |
| `Z Move Relative` | `Z Stage Move Rel Data` | ignored | ACK | [main VI frame d236] |
| `Z Wait End Move` | `Z Stage Wait Data` | ignored | ACK | [main VI frame d240] |
| `Z Enable Joystick`, `Z Disable Joystick`, `Z Joystick Enabled ?` | `Z Stage Joystick Data` | ignored | ACK / ACK / Boolean | [main VI frames d243–d245] |
| `Z Set Min Limit` / `Z Set Max Limit` | **bare DBL** | propagated, but the limit is written anyway (0.0 on failure **(inferred)**) | ACK | [main VI frames d246, d247] |

Whether the "defaults" after a failed `Variant To Data` are the typedef's saved defaults (for example 5000 for `Acquisition Timeout (ms)`) or zeros is **(unknown)**. It matters only if the silent behaviour is reproduced, which is not recommended (§10). What the client puts in `Command Data` for commands whose data is never read is **(unknown)** ([02](02-tcp-protocol.md) §12, C5).

### 5.2 Single-field camera typedefs `{Camera Type}`

Nine typedefs contain only `Camera Type` (enum, default `High Mag.`, 2 bytes if U16). They carry no hidden non-numeric field: their cluster wires are brown, the colour of an all-numeric cluster [main VI frames d116, d155].

| Typedef (cluster label) | File, pages | Command(s) |
|---|---|---|
| `Camera Get Image Data` | `typedef_ext_camera_getimage_data.ctl` [Doc6 p.349–352] | `Cam Get Image` |
| `Camera Status Data` | `typedef_ext_camera_status_data.ctl` [Doc6 p.353–356] | `Cam Get Acquisition Status` |
| `Camera Trigger Soft Data` | `typedef_ext_camera_trigsoft_data.ctl` [Doc6 p.357–360] | `Cam Software Trigger` |
| `Camera Get Exposure Data` | `typedef_ext_camera_getexposure_data.ctl` [Doc6 p.373–376] | `Cam Get Exposure` |
| `Camera Get Gain Data` | `typedef_ext_camera_getgain_data.ctl` [Doc6 p.377–380] | `Cam Get Gain` |
| `Camera Stop Data` | `typedef_ext_camera_stop_data.ctl` [Doc6 p.381–384] | `Cam Stop` |
| `Camera Release Data` | `typedef_ext_camera_release_data.ctl` [Doc6 p.385–388] | `Cam Cleanup` |
| `Camera Get Autofocus Data` | `typedef_ext_camera_getautofocusdata.ctl` [Doc6 p.393–396] | `Cam Get Autofocus Data`, `Cam Get Autofocus Progress` |
| `Camera Get LED Data` | `typedef_ext_camera_getled_data.ctl` [Doc6 p.401–404] | `Cam Get Led` |

### 5.3 `Camera Init Data` (`typedef_ext_camera_init_data.ctl`, [Doc6 p.341–344]): `Cam Setup`

| Field | LabVIEW type | Bytes | Default | Type evidence |
|---|---|---|---|---|
| `Camera Type` | enum | 2 | `High Mag.` | [Doc6 p.342] |
| `Camera Name` | string | 4 + n | `""` | pink terminal [main VI frame d79] |
| `Number of buffers` | integer, I32 assumed (I32 or U32 **(unknown)**) | 4 | 0 | blue terminal [main VI frame d79]. **Ignored by the server**, which always configures 5 buffers ([05](05-camera-acquisition.md) §4.2). |
| `Continuous Mode` | Boolean (check box `OFF/ON`) | 1 | FALSE | green wire [main VI frame d79] |

**Cluster order: ambiguous.** The two candidates are:

- panel order: `Camera Type, Camera Name, Number of buffers, Continuous Mode` [Doc6 p.342];
- `Unbundle By Name` order: `Camera Type, Continuous Mode, Number of buffers, Camera Name` [main VI frame d79].

The decision and how to confirm it are in §7.1. Both candidates have the same size, 11 + len(name) bytes.

### 5.4 Other camera payloads

Every typedef below except `Cam Log Data` starts with field 0 `Camera Type` (enum, 2 bytes, default `High Mag.`), which is not repeated in the table.

| Typedef (file, pages) → command | # | Field | Type | Bytes | Default | Type evidence |
|---|---|---|---|---|---|---|
| `Camera Start Data` (`typedef_ext_camera_start_data.ctl` [Doc6 p.345–348]) → `Cam Start` | 1 | `Triggered` | Boolean | 1 | FALSE **(inferred** from the panel rendering) | green wire [main VI frame d90] |
| `Camera Set Exposure Data` (`…setexposure_data.ctl` [Doc6 p.361–364]) → `Cam Set Exposure` | 1 | `Exposure` | **DBL** | 8 | 0 | orange wire into `In Range and Coerce` with no coercion dot, while the other inputs are DBL (`Exposure Lower Limit`, constant 2000000) [main VI frame d0] |
| `Camera Gain Data` (`…gain_data.ctl` [Doc6 p.369–372]) → `Cam Set Gain` | 1 | `Gain` | **DBL** | 8 | 0 | no coercion dot into `ValueDBL`, coercion dots into `ValueI64` / `ValueU32` [main VI frames d149, d152] |
| `Camera Ext Set FlatField` (`…setflatfield_data.ctl` [Doc6 p.365–368]) → `Cam Set FlatField` | 1 | `Enable` | Boolean | 1 | FALSE | written to `Flat Field` [main VI frame d141] |
| | 2 | `FFC Path` | path | 12 + Σ | empty | written to `Flat Field Path` |
| | 3 | `DFC Path` | path | 12 + Σ | empty | written to `Dark Current Path` |
| | 4 | `Correction Factor` | **DBL** | 8 | 0 | into `Flat Field Factor` (DBL) with no coercion dot [main VI frame d141] |
| `Camera Flip Data` (`…flip_data.ctl` [Doc6 p.389–392]) → `Cam Flip` | 1 | `Horizontal Flip` | Boolean | 1 | FALSE | constant `{High Mag., F, F}` [main VI frame d178]. This flag probably produces a **top-bottom** flip **(inferred**, [05](05-camera-acquisition.md) §7.2). |
| | 2 | `Vertical Flip` | Boolean | 1 | FALSE | idem |
| `Camera Set LED Data` (`…setled_data.ctl` [Doc6 p.397–400]) → `Cam Set Led` | 1 | `Led` | Boolean | 1 | FALSE | green wire [main VI frame d195] |
| `Camera Stream Data` (`…stream_data.ctl` [Doc6 p.540–543]) → `Cam Stream Next Image` | 1 | `Simulation` | Boolean | 1 | FALSE | [Doc6 p.555] |
| | 2 | `Simulation Path` | path (folder of PNG files) | 12 + Σ | empty | path wire [Doc6 p.555] |
| | 3 | `Simulation Timer (ms)` | **U32** | 4 | 0 | no coercion dot into the U32 input `Timer (ms)` of `alox_camera_server_simulation.vi` [Doc6 p.555, p.921] |
| | 4 | `Acquisition Timeout (ms)` | integer, I32 **(inferred)** | 4 | **5000** | blue wire with a coercion dot where it is compared (`>`) with a U32 tick difference [Doc6 p.555]: it is converted to U32, so it is an integer of at most 32 bits other than U32 (§1.2); I32 is the LabVIEW default |
| `Cam Log Data` (`…log_data.ctl` [Doc6 p.289–292]) → `Log`, stored in a control that nothing reads | 0 | `Log` | Boolean | 1 | FALSE | constant `{F, <path>, 0, 0}` [main VI frame d76] |
| | 1 | `Path` | path | 12 + Σ | empty | idem |
| | 2 | `Max Lines` | integer, I32 **(inferred)** | 4 | 0 | blue constant; I32 by analogy with `Max Lines` of `Log Server Error.vi` [Doc6 p.432] |
| | 3 | `Max Count` | integer, I32 **(inferred)** | 4 | 0 | blue constant |

`Show UI` carries a **bare Boolean** (1 byte), not a cluster. TRUE makes the window opaque and FALSE makes it transparent [main VI frame d74].

**Order evidence for this section (inferred, strong).** No positional node is printed for these typedefs. In each of them the panel order agrees with every `Unbundle By Name`: `Cam Start` [main VI frame d90], `Cam Set FlatField` [main VI frame d141], `Cam Flip` [main VI frame d178], `Cam Stream Next Image` [Doc6 p.555]. In the stream data, a 4-element `Unbundle By Name` covers fields 0–3, and `Acquisition Timeout (ms)`, last on the panel, is read by a separate node. `Cam Log Data` is never unbundled: only its panel and a constant with the same layout are printed [main VI frame d76], so its order is weaker **(inferred)**; it is never used anyway. A wrong order shows in the bytes only when it moves a string or path (length prefix, `PTH0` signature) or yields an out-of-range enum value. Otherwise it is silent, so these orders also need the confirmation of §7.3. The single-field typedefs have no order question.

### 5.5 Autofocus and refocus parameters

The semantics of every field are in [06-autofocus-refocus.md](06-autofocus-refocus.md) §2.2–§2.4. **The cluster order of both typedefs is (unknown).** The working assumption is the panel order shown below, which has not been confirmed (§7.2, questionnaire Q33).

**`Camera Autofocus Data`** (`typedef_ext_camera_autofocus_data.ctl`, [Doc6 p.599–602])

| Panel # | Field | Type | Bytes | Default |
|---|---|---|---|---|
| 0 | `Camera Type` | enum | 2 | `High Mag.` |
| 1 | `Autofocus Speed` | enum (§6.1) | 2 | `Slow` |
| 2 | `Autofocus Method` | enum (§6.1) | 2 | `FFT` |
| 3 | `Duration (s)` | integer, width **(unknown)**, I32 assumed | 4 | 0 |
| 4 | `Z Min` | DBL | 8 | 0 |
| 5 | `Z Max` | DBL | 8 | 0 |
| 6 | `Step` | DBL | 8 | 0 |
| 7 | `Subsampling` | floating point, DBL **(inferred**: wired to the DBL `Scale Factor` of the focus VIs) | 8 | 0 |
| 8 | `ROI` | 1-D integer array, I32 **(inferred**: feeds the I32 `Optional Rectangle`) | 4 + 4n | empty |
| 9 | `Acquisition Timeout (ms)` | integer, I32 assumed | 4 | 5000 (not used) |

The panel is not a single column: `Z Min | Z Max` and `Step | Subsampling` sit side by side [Doc6 p.600]. The `Unbundle By Name` nodes of the autofocus VI list `Autofocus Speed, Camera Type` [Doc6 p.606], `Z Max, Z Min, Step, ROI, Subsampling, Autofocus Method` [Doc6 p.605], `Z Max, Z Min, ROI, Subsampling, Autofocus Method` [Doc6 p.606] and `Z Max, Z Min, Duration (s)` [Doc6 p.607]. Neither the panel nor these nodes prove the cluster order (§1.3), and they contradict each other on `Camera Type` / `Autofocus Speed`. Three separate nodes put `Z Max` above `Z Min`, the reverse of the panel, which hints that `Z Max` may precede `Z Min` in the cluster **(inferred, weak)**. Both are DBL, so a swap is silent on the wire and **reverses the scan**: this is the most consequential case of A4 (§7.2).

**`Camera Refocus Data`** (`typedef_ext_camera_refocus_data.ctl`, [Doc6 p.612–615])

| Panel # | Field | Type | Bytes | Default |
|---|---|---|---|---|
| 0 | `Camera Type` | enum | 2 | `High Mag.` |
| 1 | `Autofocus Method` | enum | 2 | `FFT` |
| 2 | `Refocus Method` | enum (§6.1) | 2 | `Fin` (ignored by the server) |
| 3 | `Steps` | integer, I32 assumed | 4 | 0 |
| 4 | `Step (mm)` | DBL | 8 | 0 |
| 5 | `Fin ROI Width (pixels)` | integer, I32 assumed | 4 | 0 |
| 6 | `Fin ROI Height (pixels)` | integer, I32 assumed | 4 | 0 |
| 7 | `Oxyd X Pos (pixels)` | integer, I32 assumed | 4 | 0 |
| 8 | `Oxyd Y Pos (pixels)` | integer, I32 assumed | 4 | 0 |
| 9 | `Oxyd Diam (pixels)` | integer, I32 assumed | 4 | 0 |
| 10 | `Acquisition Timeout (ms)` | integer, I32 assumed | 4 | 5000 (not used) |
| 11 | `Wait (ms)` | integer, I32 assumed | 4 | 0 |
| 12 | `Save Images` | Boolean | 1 | FALSE **(inferred** from the rendering) |
| 13 | `Folder` | path | 12 + Σ | empty |

The refocus VI's `Unbundle By Name` lists `Camera Type, Refocus Method, Fin ROI Width (pixels), Fin ROI Height (pixels), Oxyd X Pos (pixels), Oxyd Y Pos (pixels), Oxyd Diam (pixels), Autofocus Method, Save Images, Folder` [Doc6 p.1035]. That would be consistent with `Autofocus Method` having been appended late to the cluster. It is not proof. Because most fields are fixed-size, a wrong order of the integer fields **cannot be detected from the bytes**. Do not ship autofocus/refocus decoding before the order is confirmed (§7.3).

### 5.6 Z payloads

The semantics are in [08-z-stage.md](08-z-stage.md). Every single-field cluster flattens exactly like its field.

| Typedef (cluster label) | File, pages | Fields (cluster order), type, default | Bytes | Commands |
|---|---|---|---|---|
| `Z Stage Get Serial Data` | `typedef_ext_z_getserial_data.ctl` [Doc6 p.281–284] | `Type`: Boolean (switch labelled "Serial"), FALSE. TRUE returns `Identity`, FALSE returns `Serial Number` [main VI frame d222]. | 1 | `Z Get Serial` |
| `Z Stage Set Speed Data` | `typedef_ext_z_set_speed_data.ctl` [Doc6 p.277–280] | `Speed(mm/s)`: DBL, 0 | 8 | `Z Set Speed` |
| `Z Stage Get Speed Data` | `typedef_ext_z_get_speed_data.ctl` [Doc6 p.293–296] | `Extra Param`: string, `""` (unused) | 4 + n | one of the two identical `{Extra Param}` typedefs (with `Z Stage Stop Data`) decoded by `Z Get Speed`, `Z Get Reference`, `Z Get Position` and `Z Stop`. Which typedef each case uses is not visible, because the type constants show a generic icon [main VI frames d227, d228, d231, d241]. This has no wire consequence ([03-command-reference.md](03-command-reference.md) §7.1, [08-z-stage.md](08-z-stage.md) §5.2). |
| `Z Stage Stop Data` | `typedef_ext_z_stop_data.ctl` [Doc6 p.333–336] | `Extra Param`: string (unused) | 4 + n | see the row above |
| `Z Move Abs Data` | `typedef_ext_z_move_absolute_data.ctl` [Doc6 p.447–450] | `Pos (mm)`: DBL, 0 | 8 | `Z Move Absolute` |
| `Z Stage Move Rel Data` | `typedef_ext_z_move_relative_data.ctl` [Doc6 p.325–328] | `Move (mm)`: DBL, 0 | 8 | `Z Move Relative` |
| `Z Stage Wait Data` | `typedef_ext_z_move_wait_data.ctl` [Doc6 p.329–332] | `Add. Wait (ms)`: integer, **0**; `Polling Cycle (ms)`: integer, **10**; `Add. Wait Only`: Boolean, FALSE. The order is the panel order **(inferred)**; the integers are 32-bit **(inferred)**. | 9 | `Z Wait End Move` |
| `Z Stage Joystick Data` | `typedef_ext_z_joystick_data.ctl` [Doc6 p.337–340] | `Joystick ID`: integer, **1**; `Joystick Axis`: integer, **1**. The order is the panel order **(inferred)**. | 8 | `Z Enable Joystick`, `Z Disable Joystick`, `Z Joystick Enabled ?` |
| (bare enum, no typedef printed) | [main VI frames d229, d230] | reference method: `FNL` (default), `FRF`. Other items are **(unknown)**; any value other than `FRF` runs `FNL`. | 2 | `Z Do Reference` |
| (bare DBL) | [main VI frames d246, d247] | new limit in mm | 8 | `Z Set Min Limit`, `Z Set Max Limit` |

The Wait and Joystick integers feed PI VIs whose inputs are U32 (wait) and I32 (`JAX`/`JON`). Whether coercion dots are present is not legible, so the widths stay **(inferred)**.

### 5.7 Sizes and worked examples (data part of the variant)

The examples assume U16 enums and the working orders. They were computed with the §9.2 sketch. On the wire these bytes sit between the variant header and its `00 00 00 00` attribute count ([02](02-tcp-protocol.md) §7.3).

| Value | Size | Hex |
|---|---|---|
| `Camera Stop Data{Low Mag.}` | 2 | `00 01` |
| `Camera Start Data{High Mag., TRUE}` | 3 | `00 00 01` |
| `Camera Flip Data{Low Mag., FALSE, TRUE}` | 4 | `00 01 00 01` |
| `Camera Set Exposure Data{Low Mag., 1500.0}` | 10 | `00 01 40 97 70 00 00 00 00 00` |
| `Camera Gain Data{High Mag., 2.5}` | 10 | `00 00 40 04 00 00 00 00 00 00` |
| `Camera Init Data{High Mag., "cam0", 5, TRUE}`, Unbundle order | 15 | `00 00 01 00 00 00 05 00 00 00 04 63 61 6d 30` |
| the same, panel order | 15 | `00 00 00 00 00 04 63 61 6d 30 00 00 00 05 01` |
| `Camera Stream Data` (defaults) | 23 | `00 00 00 50 54 48 30 00 00 00 04 00 00 00 00 00 00 00 00 00 00 13 88` |
| `Camera Stream Data{High Mag., TRUE, D:\sim, 200, 5000}` | 29 | `00 00 01 50 54 48 30 00 00 00 0a 00 00 00 02 01 44 03 73 69 6d 00 00 00 c8 00 00 13 88` |
| `Camera Ext Set FlatField{High Mag., TRUE, C:\f.png, <empty>, 1.0}` | 43 | `00 00 01 50 54 48 30 00 00 00 0c 00 00 00 02 01 43 05 66 2e 70 6e 67 50 54 48 30 00 00 00 04 00 00 00 00 3f f0 00 00 00 00 00 00` |
| `Cam Log Data` (defaults) | 21 | `00 50 54 48 30 00 00 00 04 00 00 00 00 00 00 00 00 00 00 00 00` |
| `Show UI` TRUE | 1 | `01` |
| `Camera Autofocus Data` (defaults; with a 4-element ROI: 66 bytes) | 50 | 46 × `00`, then `00 00 13 88` |
| `Camera Refocus Data` (defaults) | 59 | 38 × `00`, `00 00 13 88`, 5 × `00`, `50 54 48 30 00 00 00 04 00 00 00 00` |
| `Z Stage Wait Data` (defaults) | 9 | `00 00 00 00 00 00 00 0a 00` |
| `Z Stage Joystick Data` (defaults) | 8 | `00 00 00 01 00 00 00 01` |
| `Z Move Abs Data{12.5}` | 8 | `40 29 00 00 00 00 00 00` |
| `Z Set Min Limit` 0.5 | 8 | `3f e0 00 00 00 00 00 00` |

---

## 6. Enumerations

### 6.1 Summary

| Enum (label; file) | Pages | Items observed (exact) | Default | Representation | Codes | On the wire |
|---|---|---|---|---|---|---|
| `Function` (`Camera Command`; `typedef_ext_camera_camera_command.ctl`) | [Doc6 p.422–425] | 46 names (§6.2) | `Get Heartbeat` in the `.ctl`; `Cam Setup` inside `Camera Queue Data` | U16 assumed **(unknown)** | **(unknown)** | envelope field 0 |
| `TCP DataType` (`HW TCP Datatype.ctl`) | [Doc6 p.1163–1166] | `Command`, `Response`, `Image`, `Heartbeat` | `Command` | U16 **(inferred, strong**: the reader takes 2 bytes) | **(unknown)**; `Command` = 0 plausible **(inferred, weak)** | frame header ([02](02-tcp-protocol.md) §5.2) |
| `Camera Type` (`alox_typedef_camera_type.ctl`) | [Doc6 p.616–619] | `High Mag.`, `Low Mag.`: exactly 2 **(inferred, strong**, §3.3) | `High Mag.` | U16 assumed | `High Mag.` = 0, `Low Mag.` = 1 **(inferred, strong)** [Doc6 p.554] | first field of every camera payload |
| `Autofocus Speed` (`alox_typedef_autofocus_speed.ctl`) | [Doc6 p.587–590] | `Slow`, `Fast` | `Slow` | U16 assumed | **(unknown)** | `Camera Autofocus Data` |
| `Autofocus Method` (`alox_typedef_autofocus_method.ctl`) | [Doc6 p.620–623] | `FFT`, `Sobel`, `Prewitt`, `Diff`, `Gradient`, `Sigma`, `FULL`: exactly 7 **(inferred**: the refocus case structure covers them with no `Default` [Doc6 p.1035–1036]) | `FFT` | U16 assumed | **(unknown)** | AF and RF data |
| `Refocus Method` (`alox_typedef_refocus_method.ctl`) | [Doc6 p.624–627] | `Fin`, `Oxyd` | `Fin` | U16 assumed | **(unknown)** | `Camera Refocus Data` |
| Z reference method (bare enum) | [main VI frames d229, d230] | `FNL`, `FRF`, others **(unknown)** | `FNL` | U16 assumed | **(unknown)** | `Z Do Reference` |
| `State` (`HW TCP Server Command.ctl`) | [Doc6 p.1044–1047] | `Listen`, `Read`, `Send Heartbeat`, `Send Response`, `Quit` | `Listen` | — | — | no (TCP queue) |
| `State` (`typedef_ext_camera_get_image_state.ctl`) | [Doc6 p.632–635] | `Start`, `Acquire`, `Finished`, `Error`, `Stopped` (the state machine also has a `Default` frame) | `Stopped` in the `.ctl`; `Start` in the session | — | — | no |
| `AF State` (`typedef_ext_camera_autofocus_state.ctl`) | [Doc6 p.636–639] | `Start`, `Goto Z Pos`, `Wait Started`, `Measure`, `Finished`, `Error` | `Start` | — | — | no |
| `Refocus State` (`typedef_ext_camera_refocus_state.ctl`) | [Doc6 p.640–643] | `Start`, `Get Positions`, `Move Position`, `Check Position`, `Mesure Position`, `Get Focus`, `Move Focus`, `Wait Focus`, `Finished`, `Error` | `Start` | — | — | no |
| `Camera Output: LED` (`typedef_ext_camera_camera_command_type_LED.ctl`) | [Doc6 p.887–890] | `GiGe`, `USB` (observed in case structures) | `GiGe` | — | — | no. It is the type of `Camera Plug Type` and `Camera Sort: LED` **(inferred**: same items, same save minute as the properties typedef). |
| `Server Type` (input of `Log Server Error.vi`) | [Doc6 p.430–434] | `Camera`, `Monochromator`, `XY Stages`, `Z Stage` | `Camera` | — | — | no (log prefix, [09](09-configuration-logging-errors.md)) |
| `Baudrate` (a **ring**, in Z Stage Properties) | [Doc6 p.652–653] | displays `115200` | — | U16 | **(unknown)**: 115200 does not fit in a U16 | no (§8.3) |

`Exposure Type`, `Gain Type` and `Framerate Type` are **strings** (`"DBL"`, `"I64"`, `"U32"`), not enums ([05](05-camera-acquisition.md) §5.1).

### 6.2 The `Camera Command` contract

The 46 item names below are observed in case selectors and labels. The grouping follows the routing rules ([02](02-tcp-protocol.md) §7.2, [01](01-architecture.md) §5.2.2). It is not the enum order.

| Routing constraint | Names |
|---|---|
| Must lie in the camera range `"Get Heartbeat".."Cam Get Led"` (otherwise the case is unreachable) **(inferred)** | `Get Heartbeat` (range start), `Show UI`, `Exit`, `Cam Setup`, `Cam Start`, `Cam Get Image`, `Cam Stream Next Image`, `Cam Get Acquisition Status`, `Cam Software Trigger`, `Cam Set Exposure`, `Cam Set FlatField`, `Cam Set Gain`, `Cam Get Exposure`, `Cam Get Gain`, `Cam Get Images Array`, `Cam Stop`, `Cam Cleanup`, `Cam Flip`, `Cam Autofocus`, `Cam Refocus`, `Cam Get Autofocus Data`, `Cam Get Autofocus Progress`, `Cam Set Led`, `Cam Get Led` (range end) |
| Routed to the camera loop by a separate selector value, so outside the camera range **(inferred)** | `Log` |
| Internal ("only used internally"); position **(unknown)**. A client can send them only if they lie inside a range. | `Cam Autofocus Run`, `Cam Refocus Run` |
| Must lie in the Z range `"Z Setup".."Z Do Reference"` **(inferred)** | `Z Setup` (range start), `Z Get Serial`, `Z Get Status`, `Z Get Error`, `Z Set Speed`, `Z Get Speed`, `Z Get Reference`, `Z Get Position`, `Z Move Absolute`, `Z Move Relative`, `Z Wait End Move`, `Z Stop`, `Z Enable Joystick`, `Z Disable Joystick`, `Z Joystick Enabled ?`, `Z Set Min Limit`, `Z Set Max Limit`, `Z Cleanup`, `Z Do Reference` (range end) |

- The two ranges are **disjoint**: LabVIEW refuses overlapping selector values in one case structure **(inferred)**. `Exit` is also handled by the Z loop, which receives it from the camera loop and never from the router.
- If the `.ctl` default `Get Heartbeat` is item 0 (a new enum defaults to its first item), then `Log` and every item outside the camera range come **after** `Cam Get Led` **(inferred, weak)**. The developer did change defaults elsewhere (`Cam Setup` in the envelope).
- The re-implementation needs a frozen `name → code` table, filled from the `.ctl` (questionnaire Q2), from a capture, or from a golden vector. The capture method reads the bytes before the I32 length of the notifier name in each command frame ([02](02-tcp-protocol.md) §12, C2); it yields only the codes of the commands captured. `Flatten To String(To Variant(<Camera Command constant>))` yields the whole list at once, because an enum type descriptor carries the item names in value order **(inferred**, [02](02-tcp-protocol.md) §7.3; §7.3). Routing must use **explicit name sets**, not numeric ranges (§10, T2).

---

## 7. Ambiguities, decisions and how to confirm

### 7.1 Field order of `Camera Init Data`

| For the panel order | For the `Unbundle By Name` order |
|---|---|
| The panel shows `Camera Type, Camera Name, Number of buffers, Continuous Mode` [Doc6 p.342]. | The only `Unbundle By Name` of the typedef shows all four elements, as `Camera Type, Continuous Mode, Number of buffers, Camera Name` [main VI frame d79]. A node grown by dragging fills its terminals in cluster order. |
| `Unbundle By Name` order is not reliable in this code: for `Camera Autofocus Data` a node lists `Autofocus Speed` above `Camera Type` while the panel shows the reverse, so one of the two heuristics fails there (§1.3, §5.5). | This panel is **not** auto-arranged: the `Camera Type` control is clipped by the cluster border and the controls are indented irregularly [Doc6 p.342]. Its order therefore proves nothing. |
| | The same developer placed a field that was presumably added late high on the panel but at the end of the cluster order (`Camera Plug Type` in Camera Properties, §8.1) **(inferred)**. |

**Decision.** The printouts cannot settle the order, so:

1. **The decoder accepts both orders.** It tries `LV_LAYOUT` first and keeps the first order that consumes the data exactly. The two orders put the I32 length prefix of `Camera Name` at different offsets, so with realistic values only the true order fits: a frame in the `LV_LAYOUT` order is always decoded correctly, and a frame in the other order is misread only in these cases (checked by exhaustive tests of the §9.2 sketch, §9.4):
   - true order = panel order: a one-character name with `Number of buffers` = 1 (for names of printable characters);
   - if `LV_LAYOUT` is ever switched to the panel order while the true order is the `Unbundle By Name` order: `Continuous Mode` FALSE with an empty name (only `Number of buffers` comes out wrong, and the server ignores it), or with `Number of buffers` ≥ 256.

   A camera name of two or more printable characters is never misread in the first case, and fewer than 256 buffers with a non-empty name never in the second.
2. **`LV_LAYOUT` = the `Unbundle By Name` order.** This is the working assumption (medium-low confidence). It is tried first, and the test tools use it when they encode. The decoder logs a warning when a frame matches the other order.
3. **Confirm** with any of the following: the `.ctl` (right-click the cluster border, "Reorder Controls in Cluster…", questionnaire Q3); the first captured `Cam Setup` (the matched order, or the element order in the variant's type descriptors); or a golden vector (§7.3).

### 7.2 Other open points of the data model

| # | Point | Working assumption | If wrong | How to confirm |
|---|---|---|---|---|
| A1 | Representation of `Camera Type` and the other enums inside `Command Data` | U16 | every payload shifts by 1 or 2 bytes; **detectable** (inexact consumption) | type code of the enum's type descriptor (`0x15` U8 / `0x16` U16 / `0x17` U32, [02](02-tcp-protocol.md) §7.3); `.ctl` Properties → Data Type |
| A2 | Codes and representation of `Camera Command` | none | no frame can be decoded; routing | Q2; capture (02 C2) |
| A3 | Codes of `Autofocus Speed`, `Autofocus Method`, `Refocus Method` and the Z reference enum | none (kept raw) | silently wrong metric, speed or referencing move | `.ctl` "Edit Items…" (Q2); item names in the enum type descriptors of a capture or golden vector **(inferred)** |
| A4 | Cluster order of `Camera Autofocus Data` / `Camera Refocus Data` | panel order | **silent** swap of same-size fields, e.g. `Z Min` / `Z Max` (reversed scan, §5.5) | Reorder Controls in Cluster (questionnaire Q33); type descriptors; golden vector |
| A5 | Integer widths: `Duration (s)`, `ROI`, `Steps`, Fin/Oxyd fields, `Wait (ms)`, the three `Acquisition Timeout (ms)` (stream, AF, RF: the stream one is only known to be an integer of at most 32 bits other than U32), `Number of buffers`, `Max Lines`, `Max Count`, the Z wait and joystick fields | 32-bit | a 16-bit field changes the size (detectable). I32 versus U32 is invisible for values 0…2³¹−1. | type descriptors; for the AF/RF, Z wait and joystick fields, Representation in the `.ctl` (Q33) |
| A6 | Order of `Z Stage Wait Data` and `Z Stage Joystick Data`, and of the multi-field camera payloads of §5.4 | panel order | mostly **silent** (same total size), for example `Joystick ID` and `Joystick Axis` swapped | idem (Q33 covers the Z wait and joystick typedefs) |
| A7 | `Subsampling`: DBL or SGL | DBL | 4-byte shift (detectable) | idem |
| A8 | Path layout details (§3.2) | as in §3.2 | wrong path text | a capture with paths, or golden vectors |
| A9 | Order of the six reply timestamps | panel order | diagnostic timestamps swapped | [02](02-tcp-protocol.md) §12, C9 |
| A10 | Defaults produced by a failed `Variant To Data` (saved typedef defaults or zeros) | not needed if failures are rejected | — | a LabVIEW test, only if the silent behaviour is reproduced |

### 7.3 Recommended confirmation: golden vectors

A LabVIEW 2018 machine (questionnaire Q29) settles A1 and A3–A8 in one pass, without hardware or a capture, and A2 with one more constant. This is a recommendation:

1. For each payload typedef, set every field to a **distinctive** value. Examples: `Low Mag.`; integers 0x01020304, 0x05060708…; DBL 1.5, 2.5…; strings `"A"`, `"BB"`; paths `C:\a\b.png` and `\\srv\s\c`; a 4-element `ROI`.
2. Save the hex of `Flatten To String(value)`, which is the data part, and of `Flatten To String(To Variant(value))`, which is the variant header plus data **(inferred**: flattening a variant gives the same bytes as inside `Camera Queue Data`).
3. Compare with `flatten()` of §9.2. The variant hex also yields the header **templates** a test client needs ([02](02-tcp-protocol.md) §7.4), and its enum type descriptors list the item names in value order, which gives the code tables of A3 **(inferred**, [02](02-tcp-protocol.md) §7.3). Doing the same with a `Camera Command` constant gives A2.

---

## 8. Internal-state clusters (never serialised)

These clusters never leave the process. Their cluster order matters only when reading the LabVIEW diagrams. Python dataclasses are in §9.3.

### 8.1 Camera Properties ("Camera Settings", `alox_typedef_ext_camera_properties.ctl`)

The field list, types, INI keys and loaded defaults are in [05-camera-acquisition.md](05-camera-acquisition.md) §3.1. The parsing rules are in [09](09-configuration-logging-errors.md). Additional facts:

- **Cluster order** comes from the positional `Unbundle` glyphs in `Cam Setup`: `abc, I32, I32, TF, path, path, DBL, abc, abc, abc, abc, TF, TF, abc, abc, DBL, U32, TF, enum, DBL` (20 elements) [main VI frame d79]. The names of elements 0–19 are mapped by type and use **(inferred)**. The order among the four strings 7–10 (exposure/gain attribute names and types) is **(unknown)**.
- The panel layout differs from the cluster order. `Camera Plug Type` is drawn at the top right [Doc6 p.649], yet it is element 18, the only enum of the cluster. `Camera Exposure Attribute` and `Camera Gain Attribute` are drawn above `Buffers` / `Timeout`, yet the integers are elements 1–2 and the next strings come at 7–10. `Exposure Lower Limit`, drawn next to `Flat Field Factor`, is presumably element 19 **(inferred)**.
- **Typedef defaults:** every number is 0, every string and path empty, every Boolean FALSE, and `Camera Plug Type` = `GiGe` [Doc6 p.649]. The values used at run time come from `alox.ini`, except `Max Framerate` for High Mag, which comes from a diagram constant ([05](05-camera-acquisition.md) §3.1).

### 8.2 `Camera Session` (`alox_typedef_ext_camera_session.ctl`)

Panel order [Doc6 p.629]:

- `Session` (IMAQdx session);
- `SRC Image`, `FFC Image`, `DFC Image` (IMAQ images);
- `Simulation` (Boolean) and `Simulation Path` (path);
- `Simulation Index` (integer) and `Simulation Time` (numeric, not U32);
- `Last Focus Data` (2-D DBL);
- `Acquisition State` (`Get Image State`), `AF State` and `RF State`, each defaulting to `Start`;
- `Focus Progress (%)` (numeric).

Semantics: [05](05-camera-acquisition.md) §3.2 and [06](06-autofocus-refocus.md) §2.5. The cluster order is **(unknown)** and irrelevant. The sub-VIs access the cluster through `Unbundle By Name`.

### 8.3 `Z Session` and `Z Stage Properties`

| `Z Session` (`alox_typedef_ext_zstage_session.ctl`, [Doc6 p.645]) | Type | Default | Meaning |
|---|---|---|---|
| `System` | I32 | 0 | PI "System no.". `Z Setup` fills it with `Z Stage Properties.Axis`. |
| `Axes` | 1-D string array | empty | connected GCS axis identifiers |
| `Identity` | string | `""` | `*IDN?` answer |
| `Serial Number` | string | `""` | serial number from the PI setup VI |

| # | `Z Stage Properties` (`alox_typedef_ext_zstage_properties.ctl`, [Doc6 p.653]) | Type | Typedef default |
|---|---|---|---|
| 0 | `Com` | I32 | 0 |
| 1 | `Baudrate` | **U16 ring** | displays `115200` |
| 2 | `Axis` | I32 | 0 |
| 3 | `Speed (mm/s)` | DBL | 0 |
| 4 | `Slow Speed (mm/s)` | DBL | 0 |
| 5 | `Min Position (mm)` | DBL | 0 |
| 6 | `Max Position (mm)` | DBL | 0 |

The cluster order of `Z Stage Properties` is proven by positional `Unbundle` nodes (`I32, U16, I32, DBL, DBL, DBL, DBL`) [Doc6 p.141], [main VI frame d221]. **Baud-rate trap:** a U16 cannot hold 115200 (115200 mod 65536 = 49664). The panel of `alox_z_check_axis_reference.vi` shows `<49664` [Doc6 p.140]. Store the real rate in Python. The INI index mapping and the start-up constant are in [08](08-z-stage.md) and [09](09-configuration-logging-errors.md).

### 8.4 Focus working memory

- `Fast AF Session` [Doc6 p.596]: `Duration (ms)`, `T0`, `Last Timestamp`, `Current Z`, `Init Speed (mm/s)`, `Init Z (mm)`, `AF Data` (2-D DBL) and `Move Timestamp`.
- `Refocus Session` [Doc6 p.661]: `Positions`, `Scores`, `Index Pos`, `Focus Pos`, `Focus Score`, `Init Z (mm)`, `Init Speed (mm/s)` and `Move timestamp`.
- `SEMI Job Infos` [Doc6 p.970]: five strings, `Control Job ID`, `Process Job ID`, `Wafer ID`, `Lot ID` and `Zone ID`.

All three are described, with Python equivalents, in [06-autofocus-refocus.md](06-autofocus-refocus.md) §4.2, §5.2 and §8. Scalar front-panel state is described in [01-architecture.md](01-architecture.md) §8.

---

## 9. Python reference sketches

### 9.1 Mapping rules

| LabVIEW | Python |
|---|---|
| cluster typedef | `@dataclass`, plus `LV_LAYOUT = ((attribute, kind), …)` in **cluster** order |
| enum on the wire | `IntEnum` when the codes are known (`CameraType`); otherwise `int` plus the `ENUM_ITEMS` table (labels in value order, to fill) |
| Boolean / I32 / U32 / DBL | `bool` / `int` / `int` / `float` |
| string | `str` decoded as cp1252. `Notifier Name` is kept as raw bytes ([02](02-tcp-protocol.md) §10). |
| path | `LVPath(kind, components)`, lossless; `to_windows()` at the edges |
| 1-D I32 array / 2-D DBL array | `list[int]` / `list[list[float]]` or `numpy` |
| variant | `lvwire.VariantHeader` plus the typed data ([02](02-tcp-protocol.md) §11.1) |
| error cluster | `(status, code, source)` or an exception ([09](09-configuration-logging-errors.md)) |
| IMAQ image, IMAQdx session, PI session | `numpy` arrays and driver handles ([05](05-camera-acquisition.md), [08](08-z-stage.md)) |

Chapter 06 models the focus parameters with string fields holding the item labels (`AFParams`, `RFParams`, [06](06-autofocus-refocus.md) §9). Convert a decoded `CameraAutofocusData` with `enum_label("Autofocus Speed", d.autofocus_speed)` and similar calls once the code tables are filled.

### 9.2 `lvtypes.py`: payload dataclasses and (un)flattening

```python
"""Reference sketch: 'Command Data' typedefs of the ALOX Camera Server and their LabVIEW flattened layout.

Builds on lvwire.py (02-tcp-protocol.md, section 11.1): Reader, Writer, ProtocolError, ENCODING.
Each class lists its fields in CLUSTER order in LV_LAYOUT. That order, not the attribute order, defines the
bytes. Items marked (unknown) or (inferred) must be confirmed from the .ctl files or a capture (section 7).
"""
from __future__ import annotations

import enum
import logging
import struct
from dataclasses import dataclass, field
from typing import Callable, ClassVar, Optional, Union

from lvwire import ENCODING, ProtocolError, Reader, Writer

log = logging.getLogger("lvtypes")

Layout = tuple[tuple[str, str], ...]          # ((attribute, kind), ...) in cluster order

# ----------------------------------------------------------------------------- configuration (unknown)
ENUM_SIZE = 2                                 # representation of the enums inside Command Data: 1, 2 or 4 bytes
_ENUM_FMT = {1: ">B", 2: ">H", 4: ">I"}


# ----------------------------------------------------------------------------- enums
class CameraType(enum.IntEnum):               # alox_typedef_camera_type.ctl; values (inferred, strong)
    HIGH_MAG = 0                              # "High Mag." (default)
    LOW_MAG = 1                               # "Low Mag."


# Item lists in VALUE order, to fill from the .ctl files ("Edit Items..."). None = order unknown:
# the codes then stay raw integers. Observed items are given in the comments.
ENUM_ITEMS: dict[str, Optional[list[str]]] = {
    "Camera Type": ["High Mag.", "Low Mag."],  # (inferred, strong)
    "Autofocus Speed": None,                   # Slow (default), Fast
    "Autofocus Method": None,                  # FFT (default), Sobel, Prewitt, Diff, Gradient, Sigma, FULL
    "Refocus Method": None,                    # Fin (default), Oxyd
    "Z Reference Method": None,                # FNL (default), FRF; other items (unknown)
}


def enum_label(enum_name: str, code: int) -> Optional[str]:
    items = ENUM_ITEMS.get(enum_name)
    return items[code] if items is not None and 0 <= code < len(items) else None


def _camera_type(code: int) -> Union[CameraType, int]:
    try:
        return CameraType(code)
    except ValueError:
        return code                           # out of range: keep the raw value, let the handler reject it


# ----------------------------------------------------------------------------- paths ("PTH0", inferred)
PATH_ABSOLUTE, PATH_RELATIVE, PATH_NOT_A_PATH, PATH_UNC = 0, 1, 2, 3


@dataclass(frozen=True)
class LVPath:
    """A LabVIEW path as it travels: path type + components. Lossless; convert at the edges."""
    kind: int = PATH_ABSOLUTE
    components: tuple[str, ...] = ()

    @property
    def is_empty(self) -> bool:               # LabVIEW <empty path>
        return self.kind == PATH_ABSOLUTE and not self.components

    def to_windows(self) -> Optional[str]:
        """(inferred) Windows text form; None for <Not A Path>, "" for the empty path."""
        if self.kind == PATH_NOT_A_PATH:
            return None
        if self.kind == PATH_UNC:
            return "\\\\" + "\\".join(self.components)
        if self.kind == PATH_ABSOLUTE:
            if not self.components:
                return ""
            drive, *rest = self.components
            return drive + ":\\" + "\\".join(rest)
        return "\\".join(".." if c == "" else c for c in self.components)

    @classmethod
    def from_windows(cls, text: Optional[str]) -> "LVPath":
        if text is None:
            return cls(PATH_NOT_A_PATH)
        s = text.replace("/", "\\")
        if s == "":
            return cls()
        if s.startswith("\\\\"):
            return cls(PATH_UNC, tuple(c for c in s[2:].split("\\") if c))
        if len(s) >= 2 and s[1] == ":":
            return cls(PATH_ABSOLUTE, (s[0],) + tuple(c for c in s[2:].split("\\") if c))
        return cls(PATH_RELATIVE, tuple("" if c == ".." else c for c in s.split("\\") if c not in ("", ".")))


def read_path(r: Reader) -> LVPath:
    if r.take(4) != b"PTH0":
        raise ProtocolError("path: 'PTH0' expected")
    body = Reader(r.take(r.u32()))
    if not body.data:                         # defensive: zero-length body read as the empty path
        return LVPath()
    kind, count = body.u16(), body.u16()
    comps = tuple(body.take(body.take(1)[0]).decode(ENCODING, "replace") for _ in range(count))
    if body.pos != len(body.data):
        raise ProtocolError("path: unexpected bytes after the components")
    return LVPath(kind, comps)


def write_path(w: Writer, p: LVPath) -> Writer:
    body = struct.pack(">HH", p.kind, len(p.components))
    for c in p.components:
        b = c.encode(ENCODING)
        if len(b) > 255:
            raise ValueError("path component longer than 255 bytes")
        body += bytes([len(b)]) + b
    return w.raw(b"PTH0" + struct.pack(">I", len(body)) + body)


# ----------------------------------------------------------------------------- generic (un)flatten
def _read_enum(r: Reader) -> int:
    return struct.unpack(_ENUM_FMT[ENUM_SIZE], r.take(ENUM_SIZE))[0]


def _write_enum(w: Writer, v: int) -> Writer:
    return w.raw(struct.pack(_ENUM_FMT[ENUM_SIZE], int(v)))


def _read_i32_array(r: Reader) -> list[int]:
    n = r.i32()
    if n < 0:
        raise ProtocolError("negative array size")
    return [r.i32() for _ in range(n)]


def _write_i32_array(w: Writer, values) -> Writer:
    w.i32(len(values))
    for v in values:
        w.i32(v)
    return w


_READ = {"bool": Reader.bool, "i32": Reader.i32, "u32": Reader.u32, "dbl": Reader.dbl, "str": Reader.string,
         "enum": _read_enum, "path": read_path, "i32[]": _read_i32_array}
_WRITE = {"bool": Writer.bool, "i32": Writer.i32, "u32": Writer.u32, "dbl": Writer.dbl, "str": Writer.string,
          "enum": _write_enum, "path": write_path, "i32[]": _write_i32_array}


def unflatten(cls, r: Reader, layout: Optional[Layout] = None):
    """Unflatten From String of one cluster (data part of the variant), in cluster order."""
    values = {}
    for attr, kind in layout or cls.LV_LAYOUT:
        v = _READ[kind](r)
        values[attr] = _camera_type(v) if attr == "camera_type" else v
    return cls(**values)


def flatten(obj, layout: Optional[Layout] = None) -> bytes:
    """Flatten To String of one cluster: data only, no type descriptor (test client / simulator)."""
    w = Writer()
    for attr, kind in layout or type(obj).LV_LAYOUT:
        _WRITE[kind](w, getattr(obj, attr))
    return w.bytes()


# ----------------------------------------------------------------------------- camera payloads
@dataclass
class CameraOnlyData:
    """Base of the nine single-field typedefs {Camera Type} (see COMMAND_DATA)."""
    camera_type: CameraType = CameraType.HIGH_MAG
    LV_LAYOUT: ClassVar[Layout] = (("camera_type", "enum"),)


class CameraGetImageData(CameraOnlyData): """typedef_ext_camera_getimage_data.ctl - Cam Get Image"""
class CameraStatusData(CameraOnlyData): """typedef_ext_camera_status_data.ctl - Cam Get Acquisition Status"""
class CameraTriggerSoftData(CameraOnlyData): """typedef_ext_camera_trigsoft_data.ctl - Cam Software Trigger"""
class CameraGetExposureData(CameraOnlyData): """typedef_ext_camera_getexposure_data.ctl - Cam Get Exposure"""
class CameraGetGainData(CameraOnlyData): """typedef_ext_camera_getgain_data.ctl - Cam Get Gain"""
class CameraStopData(CameraOnlyData): """typedef_ext_camera_stop_data.ctl - Cam Stop"""
class CameraReleaseData(CameraOnlyData): """typedef_ext_camera_release_data.ctl - Cam Cleanup"""
class CameraGetAutofocusData(CameraOnlyData): """typedef_ext_camera_getautofocusdata.ctl - Get AF Data/Progress"""
class CameraGetLedData(CameraOnlyData): """typedef_ext_camera_getled_data.ctl - Cam Get Led"""


@dataclass
class CameraInitData:                         # typedef_ext_camera_init_data.ctl - Cam Setup
    camera_type: CameraType = CameraType.HIGH_MAG
    camera_name: str = ""                     # IMAQdx camera name
    number_of_buffers: int = 0                # ignored by the server (5 buffers, constant)
    continuous_mode: bool = False
    LAYOUT_UNBUNDLE: ClassVar[Layout] = (("camera_type", "enum"), ("continuous_mode", "bool"),
                                         ("number_of_buffers", "i32"), ("camera_name", "str"))
    LAYOUT_PANEL: ClassVar[Layout] = (("camera_type", "enum"), ("camera_name", "str"),
                                      ("number_of_buffers", "i32"), ("continuous_mode", "bool"))
    LV_LAYOUT: ClassVar[Layout] = LAYOUT_UNBUNDLE      # working assumption, section 7.1


def decode_init_data(r: Reader) -> CameraInitData:
    """Accept both candidate field orders; LV_LAYOUT is tried first and wins when both fit exactly.
    r must hold exactly the variant data (lvwire.decode_command passes Reader(data_bytes)).
    Residual ambiguities (empty or one-character names, >= 256 buffers): section 7.1."""
    start = r.pos
    candidates = (CameraInitData.LV_LAYOUT,) + tuple(
        lay for lay in (CameraInitData.LAYOUT_UNBUNDLE, CameraInitData.LAYOUT_PANEL)
        if lay is not CameraInitData.LV_LAYOUT)
    for layout in candidates:
        r.pos = start
        try:
            obj = unflatten(CameraInitData, r, layout)
        except ProtocolError:
            continue
        if r.pos == len(r.data):
            if layout is not CameraInitData.LV_LAYOUT:
                log.warning("Camera Init Data uses the other field order: update LV_LAYOUT")
            return obj
    r.pos = start
    raise ProtocolError("Camera Init Data: no candidate field order matches")


@dataclass
class CameraStartData:                        # typedef_ext_camera_start_data.ctl - Cam Start
    camera_type: CameraType = CameraType.HIGH_MAG
    triggered: bool = False
    LV_LAYOUT: ClassVar[Layout] = (("camera_type", "enum"), ("triggered", "bool"))


@dataclass
class CameraSetExposureData:                  # typedef_ext_camera_setexposure_data.ctl - Cam Set Exposure
    camera_type: CameraType = CameraType.HIGH_MAG
    exposure: float = 0.0
    LV_LAYOUT: ClassVar[Layout] = (("camera_type", "enum"), ("exposure", "dbl"))


@dataclass
class CameraGainData:                         # typedef_ext_camera_gain_data.ctl - Cam Set Gain
    camera_type: CameraType = CameraType.HIGH_MAG
    gain: float = 0.0
    LV_LAYOUT: ClassVar[Layout] = (("camera_type", "enum"), ("gain", "dbl"))


@dataclass
class CameraSetFlatFieldData:                 # typedef_ext_camera_setflatfield_data.ctl - Cam Set FlatField
    camera_type: CameraType = CameraType.HIGH_MAG
    enable: bool = False
    ffc_path: LVPath = LVPath()
    dfc_path: LVPath = LVPath()
    correction_factor: float = 0.0
    LV_LAYOUT: ClassVar[Layout] = (("camera_type", "enum"), ("enable", "bool"), ("ffc_path", "path"),
                                   ("dfc_path", "path"), ("correction_factor", "dbl"))


@dataclass
class CameraFlipData:                         # typedef_ext_camera_flip_data.ctl - Cam Flip
    camera_type: CameraType = CameraType.HIGH_MAG
    horizontal_flip: bool = False
    vertical_flip: bool = False
    LV_LAYOUT: ClassVar[Layout] = (("camera_type", "enum"), ("horizontal_flip", "bool"), ("vertical_flip", "bool"))


@dataclass
class CameraSetLedData:                       # typedef_ext_camera_setled_data.ctl - Cam Set Led
    camera_type: CameraType = CameraType.HIGH_MAG
    led: bool = False
    LV_LAYOUT: ClassVar[Layout] = (("camera_type", "enum"), ("led", "bool"))


@dataclass
class CameraStreamData:                       # typedef_ext_camera_stream_data.ctl - Cam Stream Next Image
    camera_type: CameraType = CameraType.HIGH_MAG
    simulation: bool = False
    simulation_path: LVPath = LVPath()
    simulation_timer_ms: int = 0              # U32
    acquisition_timeout_ms: int = 5000        # I32 (inferred)
    LV_LAYOUT: ClassVar[Layout] = (("camera_type", "enum"), ("simulation", "bool"), ("simulation_path", "path"),
                                   ("simulation_timer_ms", "u32"), ("acquisition_timeout_ms", "i32"))


@dataclass
class CamLogData:                             # typedef_ext_camera_log_data.ctl - Log (stored, never used)
    log: bool = False
    path: LVPath = LVPath()
    max_lines: int = 0                        # I32 (inferred)
    max_count: int = 0                        # I32 (inferred)
    LV_LAYOUT: ClassVar[Layout] = (("log", "bool"), ("path", "path"), ("max_lines", "i32"), ("max_count", "i32"))


@dataclass
class ShowUIData:                             # Show UI: a bare Boolean, not a cluster
    show: bool = False
    LV_LAYOUT: ClassVar[Layout] = (("show", "bool"),)


@dataclass
class CameraAutofocusData:                    # typedef_ext_camera_autofocus_data.ctl; ORDER (unknown): panel order
    camera_type: CameraType = CameraType.HIGH_MAG
    autofocus_speed: int = 0                  # enum code, ENUM_ITEMS["Autofocus Speed"]
    autofocus_method: int = 0                 # enum code, ENUM_ITEMS["Autofocus Method"]
    duration_s: int = 0                       # integer width (unknown), I32 assumed
    z_min: float = 0.0
    z_max: float = 0.0
    step: float = 0.0
    subsampling: float = 0.0                  # DBL (inferred)
    roi: list[int] = field(default_factory=list)   # [] or [Left, Top, Right, Bottom]
    acquisition_timeout_ms: int = 5000        # not used by the server
    LV_LAYOUT: ClassVar[Layout] = (("camera_type", "enum"), ("autofocus_speed", "enum"),
                                   ("autofocus_method", "enum"), ("duration_s", "i32"), ("z_min", "dbl"),
                                   ("z_max", "dbl"), ("step", "dbl"), ("subsampling", "dbl"), ("roi", "i32[]"),
                                   ("acquisition_timeout_ms", "i32"))


@dataclass
class CameraRefocusData:                      # typedef_ext_camera_refocus_data.ctl; ORDER (unknown): panel order
    camera_type: CameraType = CameraType.HIGH_MAG
    autofocus_method: int = 0
    refocus_method: int = 0                   # ignored by the server
    steps: int = 0
    step_mm: float = 0.0
    fin_roi_width: int = 0
    fin_roi_height: int = 0
    oxyd_x_pos: int = 0
    oxyd_y_pos: int = 0
    oxyd_diam: int = 0
    acquisition_timeout_ms: int = 5000        # not used by the server
    wait_ms: int = 0
    save_images: bool = False
    folder: LVPath = LVPath()
    LV_LAYOUT: ClassVar[Layout] = (("camera_type", "enum"), ("autofocus_method", "enum"),
                                   ("refocus_method", "enum"), ("steps", "i32"), ("step_mm", "dbl"),
                                   ("fin_roi_width", "i32"), ("fin_roi_height", "i32"), ("oxyd_x_pos", "i32"),
                                   ("oxyd_y_pos", "i32"), ("oxyd_diam", "i32"), ("acquisition_timeout_ms", "i32"),
                                   ("wait_ms", "i32"), ("save_images", "bool"), ("folder", "path"))


# ----------------------------------------------------------------------------- Z payloads
@dataclass
class ZGetSerialData:                         # typedef_ext_z_getserial_data.ctl
    type: bool = False                        # label "Type": TRUE -> Identity, FALSE -> Serial Number
    LV_LAYOUT: ClassVar[Layout] = (("type", "bool"),)


@dataclass
class ZSetSpeedData:                          # typedef_ext_z_set_speed_data.ctl
    speed_mm_s: float = 0.0
    LV_LAYOUT: ClassVar[Layout] = (("speed_mm_s", "dbl"),)


@dataclass
class ZExtraParamData:                        # typedef_ext_z_get_speed_data.ctl / typedef_ext_z_stop_data.ctl
    extra_param: str = ""                     # "not used, reserved"
    LV_LAYOUT: ClassVar[Layout] = (("extra_param", "str"),)


@dataclass
class ZMoveAbsData:                           # typedef_ext_z_move_absolute_data.ctl
    pos_mm: float = 0.0
    LV_LAYOUT: ClassVar[Layout] = (("pos_mm", "dbl"),)


@dataclass
class ZMoveRelData:                           # typedef_ext_z_move_relative_data.ctl
    move_mm: float = 0.0
    LV_LAYOUT: ClassVar[Layout] = (("move_mm", "dbl"),)


@dataclass
class ZWaitData:                              # typedef_ext_z_move_wait_data.ctl; order (inferred): panel order
    add_wait_ms: int = 0
    polling_cycle_ms: int = 10
    add_wait_only: bool = False
    LV_LAYOUT: ClassVar[Layout] = (("add_wait_ms", "i32"), ("polling_cycle_ms", "i32"), ("add_wait_only", "bool"))


@dataclass
class ZJoystickData:                          # typedef_ext_z_joystick_data.ctl; order (inferred): panel order
    joystick_id: int = 1
    joystick_axis: int = 1
    LV_LAYOUT: ClassVar[Layout] = (("joystick_id", "i32"), ("joystick_axis", "i32"))


@dataclass
class ZReferenceData:                         # Z Do Reference: a bare enum, not a cluster
    method: int = 0                           # code of "FNL" (default item): (unknown), 0 if it is the first item
    LV_LAYOUT: ClassVar[Layout] = (("method", "enum"),)


@dataclass
class ZLimitData:                             # Z Set Min Limit / Z Set Max Limit: a bare DBL
    value_mm: float = 0.0
    LV_LAYOUT: ClassVar[Layout] = (("value_mm", "dbl"),)


# ----------------------------------------------------------------------------- command -> typedef
COMMAND_DATA: dict[str, Optional[type]] = {
    "Get Heartbeat": None, "Exit": None, "Cam Get Images Array": None,       # Command Data never read
    "Show UI": ShowUIData, "Log": CamLogData,
    "Cam Setup": CameraInitData, "Cam Start": CameraStartData, "Cam Get Image": CameraGetImageData,
    "Cam Stream Next Image": CameraStreamData, "Cam Get Acquisition Status": CameraStatusData,
    "Cam Software Trigger": CameraTriggerSoftData, "Cam Set Exposure": CameraSetExposureData,
    "Cam Set FlatField": CameraSetFlatFieldData, "Cam Set Gain": CameraGainData,
    "Cam Get Exposure": CameraGetExposureData, "Cam Get Gain": CameraGetGainData,
    "Cam Stop": CameraStopData, "Cam Cleanup": CameraReleaseData, "Cam Flip": CameraFlipData,
    "Cam Autofocus": CameraAutofocusData, "Cam Autofocus Run": CameraAutofocusData,
    "Cam Refocus": CameraRefocusData, "Cam Refocus Run": CameraRefocusData,
    "Cam Get Autofocus Data": CameraGetAutofocusData, "Cam Get Autofocus Progress": CameraGetAutofocusData,
    "Cam Set Led": CameraSetLedData, "Cam Get Led": CameraGetLedData,
    "Z Setup": None, "Z Get Status": None, "Z Get Error": None, "Z Cleanup": None,  # never read
    "Z Get Serial": ZGetSerialData, "Z Set Speed": ZSetSpeedData,
    "Z Get Speed": ZExtraParamData, "Z Get Reference": ZExtraParamData, "Z Get Position": ZExtraParamData,
    "Z Stop": ZExtraParamData, "Z Do Reference": ZReferenceData,
    "Z Move Absolute": ZMoveAbsData, "Z Move Relative": ZMoveRelData, "Z Wait End Move": ZWaitData,
    "Z Enable Joystick": ZJoystickData, "Z Disable Joystick": ZJoystickData, "Z Joystick Enabled ?": ZJoystickData,
    "Z Set Min Limit": ZLimitData, "Z Set Max Limit": ZLimitData,
}
UNUSED_DATA = {"Z Get Speed", "Z Get Reference", "Z Get Position", "Z Stop"}   # read by LabVIEW, value unused


def make_decoder_for(command_codes: dict[str, int]) -> Callable[[int], Optional[Callable[[Reader], object]]]:
    """Build the decoder_for() expected by lvwire.decode_command(). command_codes maps each 'Camera Command'
    name to its numeric code (from the .ctl or a capture). None = keep the raw data bytes."""
    names = {code: name for name, code in command_codes.items()}

    def decoder_for(code: int):
        name = names.get(code)
        cls = COMMAND_DATA.get(name)
        if cls is None or name in UNUSED_DATA:
            return None
        if cls is CameraInitData:
            return decode_init_data
        return lambda r, cls=cls: unflatten(cls, r)

    return decoder_for
```

### 9.3 Internal-state dataclasses

The camera session is modelled by the camera and focus objects of [05](05-camera-acquisition.md) §12 and [06](06-autofocus-refocus.md) §9 (fields in §8.2).

```python
"""Reference sketch: internal-state clusters of the ALOX Camera Server (never serialised on the wire).

Defaults are the TYPEDEF defaults; the values loaded at start-up come from alox.ini (chapter 09).
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class CameraSettings:                 # alox_typedef_ext_camera_properties.ctl, cluster order 0..19
    camera_name: str = ""
    buffers: int = 0                  # I32, not used
    timeout: int = 0                  # I32, not used
    flat_field: bool = False
    flat_field_path: str = ""         # LabVIEW path
    dark_current_path: str = ""       # LabVIEW path
    flat_field_factor: float = 0.0
    camera_exposure_attribute: str = ""   # elements 7..10: order among these four (unknown)
    camera_gain_attribute: str = ""
    exposure_type: str = ""           # "DBL" | "I64" | "U32" (string, not an enum)
    gain_type: str = ""
    horizontal_flip: bool = False
    vertical_flip: bool = False
    camera_framerate_attribute: str = ""
    framerate_type: str = ""
    max_framerate: float = 0.0
    wait_led_ms: int = 0              # U32
    invert_led: bool = False
    camera_plug_type: str = "GiGe"    # enum: "GiGe" | "USB" | others (unknown)
    exposure_lower_limit: float = 0.0


@dataclass
class ZSession:                       # alox_typedef_ext_zstage_session.ctl
    system: int = 0                   # I32: PI "System no." (filled from Z Stage Settings.Axis)
    axes: list[str] = field(default_factory=list)   # connected GCS axis identifiers
    identity: str = ""                # *IDN? answer
    serial_number: str = ""


@dataclass
class ZStageSettings:                 # alox_typedef_ext_zstage_properties.ctl, cluster order 0..6
    com: int = 0                      # I32, COM port number
    baudrate: int = 115200            # LabVIEW: U16 ring showing "115200"; keep the real rate here
    axis: int = 0                     # I32, used as the PI system number
    speed_mm_s: float = 0.0
    slow_speed_mm_s: float = 0.0      # not used
    min_position_mm: float = 0.0
    max_position_mm: float = 0.0
```

### 9.4 Usage and tests

```python
import lvwire as L, lvtypes as T

decoder_for = T.make_decoder_for(COMMAND_CODES)        # COMMAND_CODES: name -> code, from the .ctl (unknown today)
msg = L.decode_command(payload, decoder_for)           # 02-tcp-protocol.md section 7.4
if isinstance(msg.data, T.CameraSetExposureData):
    ...                                                # handler: 05-camera-acquisition.md section 5.2
frame_data = L.write_variant(template, T.flatten(T.CameraStopData(T.CameraType.LOW_MAG)))   # test client side
```

Both sketches were run together with the `lvwire.py` of [02](02-tcp-protocol.md) §11.1. The tests covered:

- a `flatten` / `unflatten` round trip of every class, with defaults and with non-default values (the §5.7 table is their output);
- both `Camera Init Data` orders: every pair (encoding order, `LV_LAYOUT`) over empty and printable names of 1–40 characters, `Number of buffers` 0–299 and a few larger values, both `Continuous Mode` values (the residual cases are listed in §7.1);
- paths (empty, absolute, `<Not A Path>`, UNC, relative with "..");
- `decode_command` on synthetic frames, with a fabricated variant header that is **not** a real LabVIEW header, for a bare DBL, a `Cam Setup` in panel order, an unused `{Extra Param}` and an empty variant;
- `ENUM_SIZE = 1`.

---

## 10. Defects and oddities in the LabVIEW implementation

| # | Defect / oddity | Effect | Recommendation |
|---|---|---|---|
| T1 | `Variant To Data` errors are unwired in most commands (§5.1) | A payload of the wrong type runs the command with default values **(inferred)**; where the error is unwired the command also reports success. Examples: `Cam Setup` on the High Mag camera with an empty name; `Z Do Reference` running **FNL** (a move to the negative limit). Even where the error is propagated, nodes without an error input still run: `Z Set Min Limit` resets the limit to 0 and `Cam Flip` clears both flags of the High Mag camera **(inferred)**. | **Fix**: reject the command with an error reply, as the commands with a wired error already do (LabVIEW code 91 **(inferred)**, [09](09-configuration-logging-errors.md) §9.2). A conforming client never sends a wrong type, so this is not expected to be observable (malformed-input handling is fixed directly per the carve-out in [10-migration-plan.md](10-migration-plan.md) §6. Any error reply is subject to `send_error_replies`). |
| T2 | Routing uses numeric ranges of `Camera Command` (§6.2) | Inserting an enum item silently re-routes or disables commands | **Reproduce** the routing outcome, but implement it with explicit name sets evaluated once on the real item list, and freeze the code table ([03](03-command-reference.md) §3.5) |
| T3 | Visual order ≠ cluster order in at least one typedef (Camera Properties), and either the panel or the `Unbundle By Name` order is wrong for `Camera Autofocus Data`; `Camera Init Data` is open | A trap for anyone reading the printouts | Document only; validate with golden vectors (§7.3) |
| T4 | Payload fields that are ignored: `Number of buffers` (5 fixed), the AF/RF `Acquisition Timeout (ms)`, `Refocus Method` and with it the three `Oxyd …` fields ([06](06-autofocus-refocus.md) §5.9), `Joystick Axis` in `Z Joystick Enabled ?`, `Extra Param`, the whole `Cam Log Data` | The client believes it controls something it does not | **Reproduce** (accept and ignore). **Ask** whether `Number of buffers` should be honoured. |
| T5 | One typedef (`Camera Get Autofocus Data`) serves two commands; `{Extra Param}` layouts are duplicated across Z typedefs | none on the wire | Reproduce (share one class) |
| T6 | Handlers treat unexpected `Camera Type` values differently: `Default` = High Mag in some case structures, `≠ "High Mag."` = Low Mag in comparisons (focus VIs, `Cam Get Autofocus Data`), undefined in case structures without `Default`; `Cam Set Exposure` combines a High Mag camera with the Low Mag lower limit (§3.3) | One invalid value selects different cameras, or a mix of both, depending on the command **(inferred)** | **Fix**: validate the enum on decode (two items; malformed-input handling is fixed directly per the carve-out in [10-migration-plan.md](10-migration-plan.md) §6. Any error reply is subject to `send_error_replies`) |
| T7 | `Baudrate` is a U16 ring that cannot hold 115200 (§8.3) | The displayed and stored values disagree | **Fix**: store the real baud rate |
| T8 | The envelope's default command is `Cam Setup` | A failed `Unflatten` of a frame yields a `Cam Setup` with default data **(inferred**, [02](02-tcp-protocol.md) §7.6) | **Fix**: reject undecodable frames (malformed-input handling is fixed directly per the carve-out in [10-migration-plan.md](10-migration-plan.md) §6. Any error reply is subject to `send_error_replies`) |
| T9 | Inconsistent labels (`error` vs `error in (no error)`, `Speed(mm/s)`, `Type` meaning "return Identity") | none on the wire (labels are not flattened) | Reproduce the semantics; use clear Python names |

---

## 11. Open questions

Only the points that change the re-implementation are listed. The numbers refer to [open-questions.md](open-questions.md).

1. **Item order and representation of `Camera Command`** (Q2). Without them no frame can be decoded and the routing cannot be checked. This is the most important missing input.
2. **Representation of the enums inside `Command Data`**, and the codes of `Autofocus Speed`, `Autofocus Method`, `Refocus Method` and the `Z Do Reference` enum (Q2).
3. **Field order of `Camera Init Data`** (Q3). The decoder copes with both orders, but the test tools and the documentation need the real one.
4. **Field order and integer widths of `Camera Autofocus Data` and `Camera Refocus Data`**, and the order of `Z Stage Wait Data` and `Z Stage Joystick Data`. This is asked as questionnaire Q33 (Q3 covers only `Camera Init Data`). A wrong order here cannot be detected from the bytes.
5. **Golden vectors** (Q29): can the owner run `Flatten To String` on each payload typedef and on a `Camera Command` constant, as described in §7.3? That one exercise answers points 1–4 **(inferred**: enum type descriptors carry the item names) and the path details of §3.2, and provides the variant templates needed by a test client.
6. **What the client puts in `Command Data`** for the commands whose data is never read (`Get Heartbeat`, `Exit`, `Z Setup`…), and whether it ever sends `Cam Autofocus Run` / `Cam Refocus Run` itself (Q4, [02](02-tcp-protocol.md) C5).
