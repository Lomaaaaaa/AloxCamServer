# Network capture analysis

The owner recorded the traffic between the real ALOX Vision Core client and the LabVIEW camera server at the fab, on
2026-10-30. The capture was made with RawCap on the loopback interface and is 3.5 GB. `tools/extract_capture.py` reduced it
on the lab PC to the camera-server port (503): `capture_503.jsonl.gz`, 5 MB.

This page records what the capture proves and what changed in the code because of it.

## 1. The capture

| | Value |
|---|---|
| Duration | 796 s (about 13 min) |
| Connections on port 503 | 2. The first was already open when RawCap started and ends with `Exit`. The second is the restart after it. |
| Frames recovered | 5,158 commands (client → server) and 4,323 responses (server → client) |
| Lost data | Client → server: 1 gap of 1.7 KB. Server → client: 4,176 gaps, about 576 MB, almost all inside image replies (RawCap "Buffer is full"). |
| Complete images | 3 (5,127,485 bytes each); 14 more are kept as their first 256 bytes |
| Not in the capture | `Cam Set Exposure`, `Z Move …`, `Z Wait End Move`, autofocus, refocus, flat field, flip. The door was open, so no job ran. |

## 2. Decoding results

Every frame was decoded with the Phase 1 codec (`aloxcam.wire`), matching each reply to its request by notifier name.

| Check | Result |
|---|---|
| Commands decoded (21 command types) | **5,158 / 5,158** after the `Camera Type` fix (§3.1); before it, every camera command failed |
| `Cam Setup` cluster order | Camera Type, Camera Name, Number of buffers, Continuous Mode: confirmed (`High Mag.`, `"HighMag"`, 5, TRUE) |
| Payload re-encoded by the codec | Byte-identical to LabVIEW for every command type that has a payload |
| Replies matched to a request | **4,309**, none unmatched |
| Non-image replies re-encoded by the codec | **4,306 / 4,306 byte-identical** (17 command types) |
| Type descriptors in the variants | They match the typedef layouts of [04-data-types.md](../04-data-types.md) field for field (I32, U32, DBL, BOOL, string, path, typedef names) |

One request and its reply per command type are kept as regression tests in `tests/golden/capture_vectors.json`
(`tests/wire/test_capture_vectors.py`).

## 3. Corrections to the specification

### 3.1 `Camera Type` is a U32 enum

The type descriptor of `alox_typedef_camera_type.ctl` in every camera command has type code **0x17 (enum U32)**. The
specification had assumed U16, the LabVIEW default. Every camera payload is therefore 2 bytes longer than in the examples of
[04](../04-data-types.md) §5.7.

- **Fixed:** `src/aloxcam/data/wire_enums.toml` now sets `camera_type` to `size = 4`.
- The spec examples stay as unit tests of the codec mechanics, run with a U16 table. The real width is covered by the capture
  vectors.
- **Still open:** the autofocus and refocus enums (`Autofocus Speed`, `Autofocus Method`, `Refocus Method`, AF state) did not
  appear in the capture. They are also `alox_typedef_…` strict type definitions, so they may be U32 too. **To check:** right-click
  each enum in LabVIEW → Representation.

### 3.2 LabVIEW sends error replies

The specification inferred from the wiring of `Enqueue Send Error Cluster Reponse.vi` [Doc6 p.437] that an acknowledgement
carrying an error is probably never sent, so the client would time out instead. The capture proves the opposite. A
`Cam Set Led` failed, and the client received this reply:

```text
{status TRUE, code −1074360304, source "Property Node (arg 2) in alox_camera_server.vi"}
```

Only `Enqueue Send Error Cluster Reponse.vi` sends a bare error cluster, so error ACKs are sent.

- **Fixed:** `[compat] send_error_replies` now defaults to **TRUE**, which is the LabVIEW behaviour. FALSE remains available to
  suppress error ACKs.
- Defect D1 of [02](../02-tcp-protocol.md), C3 of [03](../03-command-reference.md) and questionnaire Q17 are therefore not
  defects.

### 3.3 Timestamps

The envelope carries three timestamps, and each reply carries six.

- **Requests:** the client always fills the *sending* timestamp. For `Cam Stream Next Image` it also fills the *request*
  timestamp. The *receiving* timestamp is always 0.
- **Non-image replies:** only the last one (*TCP sending*) is set; the other five are 0.
- **Image replies:** all six are set.

The values are LabVIEW millisecond tick counts (`Get Heartbeat` returned about 63,800,000, roughly 17.7 h after boot).

## 4. How the real client uses the server

Answers questionnaire Q32 (a), (b) and (e) in part.

- **Pipelining:** the client sends several commands without waiting for the replies. For example `Cam Get Acquisition Status`
  and `Cam Set Gain` leave at the same instant, and so do the two `Exit` commands. Replies are matched by notifier name, and the
  Python server supports this (`test_pipelined`).
- **Polling loops in normal operation:**
  - every ≈ 0.9 s: `Cam Set Led`, `Cam Get Led`, `Cam Stream Next Image`, `Z Get Position`;
  - every ≈ 0.5 s: `Cam Get Exposure`, alternating between the two cameras;
  - every 60 s: `Get Heartbeat`, `Z Get Status`, `Z Get Reference` (`[hardware.camera] heartbeat_I32 = 60000`).
- **Heartbeat:** the client never sends `Heartbeat` frames (datatype 3). Its liveness check is the `Get Heartbeat` command.
- **Shutdown:** `Z Enable Joystick`, `Z Stop`, `Z Cleanup`, `Cam Cleanup` × 2, `Exit` × 2.
- **Start-up:** `Show UI` (FALSE), `Cam Setup` × 2, `Cam Start` × 2, `Z Setup`, `Z Set Speed` (5 mm/s), `Cam Set Gain` × 2.
- **Reply times in the LabVIEW server:**

  | Command | Median | Maximum |
  |---|---|---|
  | `Cam Get Exposure` | 1 ms | 64 ms |
  | `Z Get Position` | 3 ms | 81 ms |
  | `Cam Set Led` | 29 ms | |
  | `Cam Get Led` | 39 ms | |
  | `Cam Stream Next Image` | 128 ms | |
  | `Cam Start` | 287 ms | |
  | `Z Setup` | 282 ms | |
  | `Cam Setup` | 2.0–2.5 s | |

  `Z Setup` took only 282 ms, so no long reference move happened during it. The stage was at 0 mm, so it is either already at
  its negative limit or not referenced by FNL in this set-up (questionnaire Q11, Q35 test 6).

## 5. NI image format (`IMAQ Flatten Image to String`)

The three complete image replies show the structure of the image string for a 2448 × 2048 Mono8 image (5,127,485 bytes):

| Part | Content |
|---|---|
| LabVIEW class header (big-endian) | I32-prefixed strings: the image name `LO_SRC_C:\Program Files\alox\AloxCamServer.exe\D\ALOX\Vision Core\sources\extcamera\alox_camera_server.vi`, then `nivissvc.*`, then the class name `LV_ImageDTClassInfo` |
| NI image header (little-endian) | Includes the width 2448 (`90 09`), the line width 2496 (`c0 09`, 64-byte aligned), the height 2048 (`00 08`), the border 3, memory addresses of the source process, and the image name again |
| Pixels | Raw image memory, border and padding included: 2496 × (2048 + 2 × 3) = 5,126,784 bytes |
| Custom data (little-endian lengths) | Driver keys `IMAQdxActualHeight`, `IMAQdxLostPackets`, `IMAQdxTimestampHigh`, `IMAQdxTimestampLow`, then the server's keys `buffer` (`"1263407"`), `flatfield` (`"FALSE"`), `name` (`"LowMag"`), `simulation` (`"FALSE"`), `timestamp` (`"09:34:09:.727"`), `type` (`"Low-Mag"`) |

The error cluster after the image string was `{FALSE, 0, ""}` in all three. These samples are what phase 3 needs to build a pure
Python encoder for this format and check it byte for byte. The alternative is to call NI Vision; the owner accepts either.

The image name also shows where the LabVIEW server is installed: `C:\Program Files\alox\AloxCamServer.exe`.

## 6. Still to capture later

A second, short capture with the door closed would cover:

- `Cam Set Exposure`;
- one `Z Move Absolute` inside the range and one outside it (the 9405 error ACK is now expected);
- `Z Wait End Move`;
- one autofocus and one refocus. This would also show the representation of the autofocus enums (§3.1).
