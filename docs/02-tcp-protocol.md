# TCP protocol (wire format)

**Summary.** The ALOX Camera Server is a TCP server that accepts one client at a time on the loopback interface. Every message in both directions is a frame `I32 length | U16 datatype | payload`, written big-endian with LabVIEW's `Flatten To String`. The client sends `Command` frames. Their payload is the flattened `Camera Queue Data` cluster: the command enum, a notifier name, a flattened LabVIEW **variant** that holds the command parameters, and three U32 timestamps. The server answers with `Response` frames. Their payload is the flattened `HW TCP Response Data` cluster: the client's notifier name copied back unchanged, a nested flattened reply (an error cluster alone, or a `{value, error}` cluster) whose type depends on the command, and six U32 timestamps. This chapter specifies:

- the connection model and the TCP state machine of the main VI;
- the byte layout of frames, envelopes and reply payloads, and the LabVIEW flattening rules needed to reproduce them byte for byte;
- the variant format, which is inferred;
- heartbeat handling and the notifier correlation used by the client;
- Python reference sketches;
- what only a network capture can confirm.

The command catalogue is in [03-command-reference.md](03-command-reference.md). The `Command Data` typedefs and their byte layouts are in [04-data-types.md](04-data-types.md).

**Sources.**

- `alox_camera_server.vi`, TCP SERVER PRODUCER LOOP: main diagram [main VI frame d0; main VI p.3] and hidden frames d62–d72 [main VI p.5–9]. Queue creation is in frame d0; port loading is in frame d4.
- extshared VIs: `HW TCP Extract TCP Data.vi` [Doc6 p.1150–1154], `HW TCP Send Heartbeat.vi` [Doc6 p.1155–1158], `HW TCP Send Response.vi` [Doc6 p.1167–1170], `HW TCP Response Data.ctl` [Doc6 p.1159–1162], `HW TCP Datatype.ctl` [Doc6 p.1163–1166], `HW TCP Server Queue.ctl` [Doc6 p.1040–1043], `HW TCP Server Command.ctl` [Doc6 p.1044–1047].
- Reply VIs: `Enqueue Send Error Cluster Reponse.vi` [Doc6 p.435–438], `Enqueue Send STR Response.vi` [Doc6 p.443–446], `Enqueue Send Boolean Response.vi` [Doc6 p.455–458], `Enqueue Send DBL Response.vi` [Doc6 p.463–466], `Enqueue Send U32 Response.vi` [Doc6 p.559–562], `Enqueue Send Image Response.vi` [Doc6 p.571–574], `Enqueue Send DBL 2D Array Response.vi` [Doc6 p.1124–1127].
- Reply typedefs `Wait for STR / Boolean / DBL / U32 / Image / DBL 2D Array Notifier Data.ctl` [Doc6 p.439–442, 451–454, 459–462, 536–539, 563–566, 1048–1051].
- `IMAQ Flatten Image to String` [Doc6 p.567–570].
- NI `TCP Listen.vi`, `Internecine Avoider.vi`, `TCP Listen Internal List.vi`, `TCP Listen List Operations.ctl` [Doc6 p.1132–1149].
- `typedef_ext_camera_mgmt_queue_data.ctl` [Doc6 p.285–288], `typedef_ext_camera_camera_command.ctl` [Doc6 p.422–425], `alox_camera_server_globals.vi` [Doc6 p.418–421].

---

## 1. Conventions and LabVIEW primer

### 1.1 Conventions

- Untagged statements are observed on the printouts. **(inferred)** marks a deduction from LabVIEW/NI semantics or from cross-checking. **(unknown)** marks something the printouts do not show.
- Citations: `[main VI frame dN]` is hidden frame N of the HTML export of `alox_camera_server.vi`, and `d0` is the main diagram as displayed. `[main VI p.N]` is a page of the main VI PDF. `[Doc6 p.N]` is a page of "Labview Document 6", which holds the sub-VIs, typedefs and NI library VIs.
- Hex bytes are written in wire order. `tt tt` is the U16 code of a `HW TCP Datatype` item and `cc cc` is the code of a `Camera Command` item. Both are **(unknown)** (§5.2, §7.2).
- "Client" means the program that connects to the server. It is presumably the LabVIEW "ALOX Vision Core" **(inferred)**. Its code is not in the printouts.

### 1.2 LabVIEW behaviour that matters on the wire

| LabVIEW construct | Behaviour relevant here | Python equivalent |
|---|---|---|
| `Flatten To String` / `Unflatten From String` | The byte-order input is never wired, so the default **big-endian** applies everywhere. "Prepend array or string size" defaults to TRUE, so every string and array carries I32 length prefix(es). A cluster flattens as the concatenation of its elements in **cluster order** (which can differ from the on-screen order) with no padding or alignment. `Unflatten From String` ignores bytes left over at the end and fails if the input is too short (error 116 **(inferred)**). | `struct` with `>` formats |
| Enum | Values are consecutive integers from 0, in the order of the item list. The value is flattened as an unsigned integer of the enum's representation: U8, U16 (the LabVIEW default) or U32. The printouts show item names only, never the list or the representation. | `IntEnum` plus a code table to fill |
| Variant | A container that holds a value of any type together with its type description. `Variant To Data` converts it back to an expected type. Flattened layout: §7.3. | raw bytes plus a per-command decoder |
| Error cluster | `{status: Boolean, code: I32, source: string}`. A node whose `error in.status` is TRUE does not execute and passes the error on; close and release functions (`TCP Close`, `Release Queue`) are the usual exception and run anyway. `status = FALSE` with `code ≠ 0` is a warning. Primitives such as `Flatten To String` have no error terminals, so an error wire drawn across them only passes behind. | `(bool, int, str)` tuple / exception |
| Named queue (`Obtain Queue`) | Any VI can obtain the same queue by name. `max queue size` bounds it. `Enqueue` with timeout −1 blocks while the queue is full. `Dequeue` with a timeout returns `timed out? = TRUE` when nothing arrives. `Release Queue(force destroy)` makes pending and later operations fail. | `queue.Queue` / `asyncio.Queue` |
| Named notifier | A single-slot "latest value" mailbox. `Send Notification` overwrites it, and `Wait on Notification` blocks until a new value arrives or the timeout expires. Used by the client to wait for replies **(inferred)**. | `Future` / `Event` per call |
| Uninitialised shift register | A loop register keeps its value from one iteration to the next. The TCP connection register starts as "Not a Refnum". | object attribute |
| `TCP Read` modes | **Standard**: on timeout, returns the bytes received so far (consumed) plus error 56. **Buffered**: on timeout, returns nothing and leaves the bytes in the receive buffer, plus error 56. The default timeout is 25000 ms. | `readexactly` plus `wait_for` (§5.5) |
| `TCP Write` | Returns error 56 if the data cannot be written within the timeout. A partial write is then possible **(inferred)**. | `write` plus `drain` with a timeout |
| `Tick Count (ms)` | A free-running U32 millisecond counter that wraps after about 49.7 days. Client and server run on the same PC (loopback only), so they share this clock **(inferred)**. | `int(time.monotonic()*1000) & 0xFFFFFFFF` |
| Sub-VI output not written in the executed case | The caller receives the value left by the previous call **(inferred, non-reentrant VI)**. | return `None` |

---

## 2. Overview

```
 client (same PC, presumably LabVIEW)                         alox_camera_server.vi
 ────────────────────────────────────                         ─────────────────────────────────────────────────────
 Obtain Notifier("<unique name>")                             TCP SERVER PRODUCER LOOP  (queue "Camera Server", size 5)
 Flatten(Camera Queue Data) ─► [N][Command][payload] ───────► Read: HW TCP Extract TCP Data ─► Unflatten ─► route:
                                                                "Z Setup".."Z Do Reference"           ─► queue "Z Stage Mgr"
                                                                "Get Heartbeat".."Cam Get Led", "Log"  ─► queue "Camera Mgr"
                                                                anything else                          ─► dropped
                                                              consumer loops execute the command, then call
                                                              Enqueue Send <type> Response.vi ─► {Send Response, Response Data}
 Wait on Notification ◄─ (by name) ◄─ [N][Response][payload] ◄─ Send Response: HW TCP Send Response
```

The protocol has three layers:

1. **Framing**: length, datatype and payload (§5), built and parsed by the `HW TCP …` VIs of `extshared`.
2. **Envelopes**: `Camera Queue Data` inbound (§7) and `HW TCP Response Data` outbound (§8).
3. **Per-command content**: the `Command Data` variant and the type of the reply value. See [03-command-reference.md](03-command-reference.md) and [04-data-types.md](04-data-types.md).

The loops, queues and shutdown sequence are described in [01-architecture.md](01-architecture.md).

---

## 3. Connection model

| Aspect | Behaviour | Evidence |
|---|---|---|
| Role | TCP server. The client connects. | [main VI frame d62] |
| Listen address | `String To IP("localhost")`, i.e. 127.0.0.1 (name resolved by the OS **(inferred)**). **Only local clients can connect.** | [main VI frame d62] |
| Port | Front-panel `TCP Port`, loaded at start-up from `alox.ini` `[hardware.camera] port_I32`, default **503** (see [09-configuration-logging-errors.md](09-configuration-logging-errors.md)). Coerced I32 → U16 at the `TCP Listen` input. | [main VI frames d4, d62] |
| Accept | `TCP Listen.vi` with `timeout ms = 2000`. On any error (normally timeout 56) the dequeued `Listen` element is queued again. `resolve remote address` is left at its default (TRUE). The remote address and port are not used. | [main VI frame d62], [Doc6 p.1146–1148] |
| Port already used by another program | Creating the listener fails, `TCP Wait On Listen` is then skipped, so `Listen` fails at once and is re-queued without the 2 s wait: the loop retries continuously and never serves a client **(inferred)**. | [Doc6 p.1143, p.1148] + **(inferred)** |
| Listener lifetime | `TCP Listen.vi` calls `Internecine Avoider.vi`, which keeps a table of listeners per (address, port, service). The listener is created at the first `Listen` and **reused** by every later `Listen`. An invalid listener is deleted and recreated. | [Doc6 p.1141–1148] |
| Concurrent clients | **One at a time.** The connection refnum lives in one uninitialised shift register. `Listen` only runs while no connection is being served. | [main VI frames d0, d62] |
| A second client while one is served | The listening socket stays open, so the OS completes the second connection in its backlog. It is accepted at the next `Listen`, after the first client has gone. Its bytes then wait in order **(inferred)**. | [Doc6 p.1143] + **(inferred)** |
| Disconnection detection | Any `Read` error other than 0 and 56 → `TCP Close` → `Listen`. | [main VI frame d64] |
| Idle timeout, keep-alive | None. | [main VI frames d62–d72] |
| `Connected` indicator | FALSE at start-up, then `NOT error.status` after each `Listen`. **Not** reset on disconnection. | [main VI frames d2, d62, d64] |
| Server-initiated close | Never in normal operation. The `Quit` state (`TCP Close` and stop) exists but nothing queues it. After `Exit` the process ends without closing the socket. | [main VI frame d70], [01-architecture.md](01-architecture.md) |
| Port conflict | `Internecine Avoider.vi` returns error 114 if the port is already registered with another address. This cannot happen with a single listening port. | [Doc6 p.1143] |

---

## 4. TCP server state machine

### 4.1 The TCP queue

| Property | Value | Evidence |
|---|---|---|
| Name | `"Camera Server"` (global `TCP Server Queue Name`) | [Doc6 p.419] |
| Max size | 5. The constant is also displayed in `TCP Max Size`, with the comment "#Question : Borner TCP Queue Size ?" ("bound the TCP queue size?"). | [main VI frame d0] |
| Element | `HW TCP Server Queue` = `{State: enum HW TCP Server Command, State Data: variant}`. Default `State` = `Listen`. | [Doc6 p.1041, p.1045] |
| Initial content | `{Listen, empty variant}`, enqueued right after `Obtain Queue` | [main VI frame d0] |
| Producers | The TCP loop itself (`Listen`, `Read`), and the `Enqueue Send <type> Response.vi` VIs, which obtain the queue **by name** from the camera and Z loops | [main VI frame d0], [Doc6 p.437] |
| Consumer | The TCP loop. `Dequeue Element` with a timeout of **50 ms**. | [main VI frame d0] |

### 4.2 States

The state enum is `HW TCP Server Command`. The case structure has exactly five frames.

| State | Frame | Action | Element queued afterwards |
|---|---|---|---|
| `Listen` (shares its frame with `Default`) | d62 | `(conn', e) ← TCP Listen(127.0.0.1, TCP Port, 2000 ms)`; `Connected ← NOT e.status`; `conn ← conn'`. The previous refnum is neither closed nor kept. | `Select(e)`: on error, the dequeued element itself (`Listen`) is queued again; on success, `{Read}` |
| `Read` | d0 (visible) | `HW TCP Extract TCP Data(conn)`, then a case on `error.code` (§4.4) | depends on the code |
| `Send Response` | d69 | `rd ← Variant To Data(State Data, HW TCP Response Data)`; `rd.TCP Sending timestamp ← Tick Count`; `HW TCP Send Response(conn, rd)`. All errors are unwired (ignored). | nothing |
| `Send Heartbeat` | d68 | `HW TCP Send Heartbeat(conn)`. Comment: "#Question : Ajouter une bloucle Heartbeat ?" ("add a heartbeat loop?"). **Nothing queues this state.** | nothing |
| `Quit` | d70 | `TCP Close(conn)`; stop the TCP loop. **Nothing queues this state.** | — |
| *(queue empty for 50 ms)* | d71 | `Enqueue {Read}`, then the error is cleared. If the front-panel button `CLEAR` is TRUE, the `Log` indicator is emptied (d72: FALSE does nothing). | `{Read}` |

On every dequeued element the loop writes `Heartbeat ← Tick Count` (a front-panel liveness indicator, not a network heartbeat) and `Server State ← State`. The loop stops when `Dequeue` returns an error, or after `Quit`. The `Dequeue` error happens when the camera loop destroys the TCP queue after `Exit`.

### 4.3 Pseudo-code

```
TCP_Q ← ["Listen"]; conn ← NotARefnum
loop:
    (elem, timed_out, err_dq) ← Dequeue(TCP_Q, 50 ms)
    if timed_out:                                   # d71
        Enqueue(TCP_Q, Read)                        # infinite timeout
        if CLEAR: Log ← ""
    else:
        Heartbeat ← TickCount ; Server State ← elem.State
        case elem.State:
          Listen, Default:                          # d62
            (conn2, e) ← TCP_Listen("127.0.0.1", TCP_Port, timeout=2000)
            Connected ← not e.status
            Enqueue(TCP_Q, elem if e.status else Read)
            conn ← conn2
          Read:                                     # d0
            (conn, datatype, data, e) ← Extract_TCP_Data(conn)              # §5.3
            case e.code:
              0:  if datatype == Command:
                      msg ← Unflatten(data, Camera Queue Data)             # error output unwired
                      Command ← msg.Camera Command
                      route(msg)                                           # §7.2
                      Commands History ← ([msg.Camera Command] + Commands History)[:500]
                  # any other datatype: nothing                            # d67
              56: Enqueue(TCP_Q, Read)                                     # d63 "TCP Timeout"
              other: (conn, ec) ← TCP_Close(conn)                          # d64
                     Enqueue(TCP_Q, Listen, error_in=ec)                   # skipped if Close failed
          Send Response:                                                   # d69
            rd ← VariantToData(elem.State Data, HW TCP Response Data)
            rd.TCP Sending timestamp ← TickCount
            conn ← HW_TCP_Send_Response(conn, rd)                          # errors ignored
          Send Heartbeat: conn ← HW_TCP_Send_Heartbeat(conn)               # d68, unreachable
          Quit: TCP_Close(conn); stop ← TRUE                               # d70, unreachable
    if err_dq.status or stop: exit loop
```

### 4.4 Outcomes of `Read`

| `Extract TCP Data` result | Typical cause | Action | Evidence |
|---|---|---|---|
| code 0, datatype `Command` | complete command frame | decode and route (§7). **No `Read` is queued.** | [main VI frame d0] |
| code 0, any other datatype | `Heartbeat`, `Image` or `Response` frame from the client | nothing: the frame is discarded | [main VI frame d67] |
| code 56 | fewer than 4 header bytes arrived within 10 ms (idle line), or a later read of the frame timed out (including the 25 s extra-bytes read of a `Heartbeat` frame) | `Enqueue {Read}`; the connection stays open even if a frame was cut (§5.5) | [main VI frame d63] |
| any other code | 66 (connection closed by peer), 62 (connection aborted) **(inferred codes)**; −1 "Extra Data Found" (heartbeat frame with a payload, §5.3); possibly an `Unflatten` failure on an out-of-range datatype (§5.2) **(inferred)**; negative byte count when N < 2 **(inferred)** | `TCP Close`, then `Enqueue {Listen}`. The enqueue takes the close error as its `error in`, so **if `TCP Close` fails, `Listen` is never queued** (defect D3, §13). | [main VI frame d64] |

### 4.5 Timeouts

| Operation | Timeout | Evidence |
|---|---|---|
| `Dequeue` of the TCP queue | 50 ms | [main VI frame d0] |
| `TCP Listen` | 2000 ms | [main VI frame d62] |
| Header (4 bytes, Buffered) | 10 ms | [Doc6 p.1152] |
| Datatype (2 bytes, Buffered) | 200 ms | [Doc6 p.1152] |
| Payload of `Command`, `Response` or any datatype not listed below (Buffered) | 5000 ms | [Doc6 p.1152] |
| Payload of `Image` (Standard) | 5000 ms | [Doc6 p.1152] |
| Extra bytes of a `Heartbeat` frame (Standard, unwired default) | 25000 ms | [Doc6 p.1153] |
| `TCP Write` of a reply or a heartbeat | 1000 ms | [Doc6 p.1157, p.1169] |
| `Enqueue` into the TCP, camera and Z queues | −1 (infinite) | [main VI frames d0, d62–d71], [Doc6 p.437] |

### 4.6 Resulting behaviour

- **Polling.** While a client is connected and idle, the loop cycles `Read` → 56 → `Read` with a 10 ms wait each time.
- **Inbound throughput.** A successful read does not queue the next `Read`. The next read happens only when the TCP queue has stayed empty for 50 ms (frame d71). Each inbound frame is therefore followed by a pause of about 50 ms before the socket is read again. A client that pipelines commands sees them read at most about 20 per second **(inferred from the observed logic)**.
- **Serialisation.** Reads and writes run in one loop, so frames are never interleaved. A reply waits for the current `Read` (≤ 10 ms when idle, up to about 5.2 s while a frame arrives, up to 25 s for the extra bytes of a `Heartbeat` frame) and, without a client, for the 2 s `Listen`. Replies are sent in the order the consumers queued them.
- **Replies without a client.** A reply dequeued while no client is connected is written to a closed or invalid refnum. The error is ignored and the reply is lost. Replies are not bound to a connection: the reply to a previous client's command, if it is sent after a reconnection, goes to the **new** client **(inferred)**.
- **Self-deadlock risk.** The loop enqueues `Read`/`Listen` into its own 5-element queue with an infinite timeout. If the consumers fill the queue with `Send Response` elements first, the loop blocks forever, because it is the queue's only reader **(inferred)**. See [01-architecture.md](01-architecture.md).

---

## 5. Frame layout

### 5.1 Header

| Offset | Size | Type | Field |
|---|---|---|---|
| 0 | 4 | I32, big-endian | `N` = 2 + payload length. It counts the datatype field but not itself. |
| 4 | 2 | U16, big-endian | `Datatype` (enum `HW TCP Datatype`) |
| 6 | N − 2 | bytes | payload |

There is no delimiter, checksum, version field or sequence number. The writer computes `N = String Length(Flatten(datatype)) + String Length(Flatten(payload))` as an I32 [Doc6 p.1169]. The reader unflattens the 4 header bytes with an integer type constant, I32 **(inferred)** [Doc6 p.1152].

### 5.2 `HW TCP Datatype`

The front-panel default of `HW TCP Datatype.ctl` (label "TCP DataType") is `Command` [Doc6 p.1164].

| Item | Sent by the server | Server's reaction when received | Evidence |
|---|---|---|---|
| `Command` | never | payload read (Buffered, 5000 ms), decoded and routed | [Doc6 p.1152], [main VI frame d0] |
| `Response` | **every reply** (`HW TCP Send Response`) | payload read like `Command`, then ignored | [Doc6 p.1169], [main VI frame d67] |
| `Image` | never: images travel inside `Response` frames (§8.5) | payload read in **Standard** mode (5000 ms); the reception time is measured (output `Temps de réception image`, unused by the main VI **(inferred)**); then ignored | [Doc6 p.1152] |
| `Heartbeat` | never: the `Send Heartbeat` state is never queued | no payload expected, frame ignored. A payload produces error −1 "Extra Data Found", and the server **closes the connection** (§4.4). | [Doc6 p.1153], [main VI frames d64, d67] |

- **Representation: U16** **(inferred, strong)**. The reader reads exactly 2 bytes for the datatype and then `N − 2` payload bytes, and the writer counts the flattened enum's length in `N`. With a 1-byte enum the payload would lose its first byte; with a 4-byte enum the datatype read would be cut. Only a 2-byte enum makes both VIs agree.
- **Numeric values: (unknown).** `Command = 0` is plausible but weak evidence: `Command` is the control's default, and a new enum defaults to its first item, but defaults can be edited **(inferred)**. The `Extract` case frames are listed `"Command", Default` / `"Image"` / `"Heartbeat"`, which suggests `{Command, Response, Image, Heartbeat}` or `{Command, Image, Heartbeat, Response}`, but case frame order is not proof of enum order **(inferred)**. The re-implementation must keep the codes in a table filled from a capture or from `HW TCP Datatype.ctl` (questionnaire Q2 in [open-questions.md](open-questions.md)).
- A value outside the enum either makes the datatype `Unflatten` fail (error 116), in which case the payload is not read and the main VI closes the connection, or yields an out-of-range value that takes the `"Command", Default` frame, in which case the payload is read and then ignored by the main VI. Which one LabVIEW does is **(unknown)**; it matters only for a misbehaving client.

### 5.3 Reading a frame (`HW TCP Extract TCP Data.vi`)

Connector: inputs `TCP Network Connection Refnum` and `error in`; outputs refnum, `Datatype`, `Data` (string), `error out` and `Temps de réception image` (U32 ms) [Doc6 p.1150]. The main VI leaves `error in` unwired.

```
(hdr, err) ← TCP_Read(conn, 4 bytes, Buffered, 10 ms, error_in)
if err:                                   # "Error" frame: Data and Datatype are NOT written (§1.2 last row)
    return conn, <stale>, <stale>, err
N ← Unflatten(hdr, I32)                   # error of this Unflatten unwired
(d, err) ← TCP_Read(conn, 2 bytes, Buffered, 200 ms, err)
(Datatype, err) ← Unflatten(d, HW TCP Datatype, err)
rest ← N − 2
case Datatype:
  "Command", Default:                     # also covers Response
      (Data, err) ← TCP_Read(conn, rest, Buffered, 5000 ms, err)
  "Image":                                # 3-frame sequence
      t0 ← TickCount ; (Data, err) ← TCP_Read(conn, rest, Standard, 5000 ms, err)
      Temps de réception image ← TickCount − t0
  "Heartbeat":
      Data ← ""
      if rest ≠ 0:
          (_, e1) ← TCP_Read(conn, rest, Standard, 25000 ms, err)        # mode and timeout unwired
          err ← Merge_Errors(e1, Error_Cluster_From_Error_Code(−1, "Extra Data Found"))
return conn, Datatype, Data, err
```

[Doc6 p.1152–1153]. `Merge Errors` returns the first error in input order. If the extra-bytes read itself failed, its error wins. Otherwise the result is error −1 "Extra Data Found". The extra-bytes read uses the `TCP Read` defaults (Standard mode, 25000 ms), so a `Heartbeat` frame announcing more bytes than it carries blocks the whole TCP loop, replies included, for up to 25 s, and then returns error 56, which the main VI treats as an idle timeout **(inferred)**.

The header is unflattened with "data includes array or string size?" set to FALSE, which has no effect on a scalar [Doc6 p.1152].

### 5.4 Writing a frame

| VI | Bytes written | Notes | Evidence |
|---|---|---|---|
| `HW TCP Send Response.vi` | `Flatten(I32 N) ‖ Flatten(enum "Response") ‖ Flatten(Response Data)` | One `TCP Write` with a 1000 ms timeout. `error in` is passed to `TCP Write`, so nothing is sent if an error comes in. The main VI wires no error. | [Doc6 p.1169] |
| `HW TCP Send Heartbeat.vi` | `00 00 00 02` ‖ `Flatten(enum "Heartbeat")` = `00 00 00 02 tt tt` | `TCP Write`, 1000 ms. Never called in practice (§9). | [Doc6 p.1157] |

### 5.5 Buffered versus Standard reads, and stream desynchronisation

- The 10 ms header read is a **non-destructive poll**. In Buffered mode a timeout consumes nothing, so a partly received header stays in the buffer for the next attempt.
- After the header, each later timeout leaves the stream in the wrong place. Datatype: the 4 header bytes are already consumed. Payload, Buffered: header and datatype are consumed. Payload, Standard (`Image`): a partial payload is consumed too. The main VI treats any of these as code 56 and simply reads again, so the next "header" is taken from the middle of a frame **(inferred)**. Over loopback this needs a payload that takes more than 5 s to arrive.
- Recommendation for the re-implementation: wait for a header with no timeout (or a long one), read the rest of the frame with a timeout, and **close the connection** on a mid-frame timeout or on an implausible `N` (N < 2 or above a size limit). Do not try to resynchronise.

---

## 6. LabVIEW flattening rules (reference)

Every structure in this chapter follows these rules, with no exception observed.

| LabVIEW type | Flattened form (big-endian) | Python `struct` |
|---|---|---|
| I8 / U8 / I16 / U16 / I32 / U32 / I64 / U64 | 1 / 1 / 2 / 2 / 4 / 4 / 8 / 8 bytes | `>b >B >h >H >i >I >q >Q` |
| SGL / DBL | IEEE-754, 4 / 8 bytes | `>f >d` |
| Boolean | 1 byte: `00` = FALSE, `01` = TRUE. On reading, any non-zero byte is TRUE **(inferred)**. | `>B` |
| Enum | Unsigned integer of the enum's representation (U8 / U16 / U32); value = 0-based item position. U16 is assumed for every enum of this protocol **(unknown)**. | `>H` |
| String | I32 byte count, then the raw bytes. No terminator. Windows ANSI code page, cp1252 **(inferred)**. | `>i` + bytes |
| 1-D array | I32 element count, then the elements. Each string element carries its own I32 prefix. | |
| 2-D array | I32 rows, I32 columns, then the elements in row-major order | `>ii` + `>{r*c}d` |
| Cluster | elements concatenated in **cluster order**, with no padding | |
| Error cluster | `U8 status` (Boolean) · `I32 code` · `I32 length + source bytes` (9 bytes when empty) | `>Bi` + string |
| Path | `"PTH0"` · U32 length of the rest · U16 path type · U16 component count · components as (U8 length + bytes) **(inferred)**. Used only inside `Command Data`; details in [04-data-types.md](04-data-types.md). | |
| Variant | version, type descriptors, data, attributes (§7.3) **(inferred)** | |
| Typedef | adds nothing to the flattened **data**. It only appears in the type descriptors of a variant **(inferred)**. | |

The "string inside a string" pattern recurs (§8): `Response (Flattened)` is itself the output of a `Flatten To String`. On the wire it is therefore a length-prefixed string whose content holds further length-prefixed strings (for example a STR value, or the IMAQ image).

---

## 7. Inbound command payload

### 7.1 `Camera Queue Data`

The main VI decodes the payload of a `Command` frame with `Unflatten From String(Data, type = constant "CAMERA MGR QUEUE DATA")`. Byte order and the size flag are unwired, so their defaults apply [main VI frame d0]. The typedef is `typedef_ext_camera_mgmt_queue_data.ctl` [Doc6 p.285–288]. The cluster order is confirmed by the positional `Unbundle` in the main VI (glyphs: enum, string, variant, U32, U32, U32) [main VI frame d0].

| # | Field | Type | Flattened | Notes |
|---|---|---|---|---|
| 0 | `Camera Command` | enum `typedef_ext_camera_camera_command.ctl` (label "Function") | 2 bytes if U16 **(unknown)** | default `Cam Setup` inside this cluster, `Get Heartbeat` in the `.ctl` [Doc6 p.286, p.423] |
| 1 | `Notifier Name` | string | I32 + bytes | correlation key; copied unchanged into the reply (§10) |
| 2 | `Command Data` | variant | §7.3, no overall length prefix | type depends on the command ([04-data-types.md](04-data-types.md)) |
| 3 | `Command Request Timestamp` | U32 | 4 | client Tick Count **(inferred)**; copied into image replies |
| 4 | `Command Sending Timestamp` | U32 | 4 | client Tick Count **(inferred)**; copied into image replies |
| 5 | `Command Receiving Timestamp` | U32 | 4 | **overwritten** by the server with `Tick Count` when it routes the command to the camera loop ("Store receiving timestamp for image statistics") [main VI frame d66]. Not touched on the Z route. |

Size: `2 + (4 + len(name)) + len(variant) + 12` bytes, assuming a U16 command enum.

### 7.2 Command code and routing

The router is a case structure on `Camera Command` inside `Read` > `0` > `"Command"`:

| Case selector | Label / comment | Action | Evidence |
|---|---|---|---|
| `"Z Setup".."Z Do Reference"` | "Z Consumer loop for Z stage commands" | `Enqueue(Z queue "Z Stage Mgr", msg)`; timestamps unchanged | [main VI frame d0] |
| `"Get Heartbeat".."Cam Get Led", "Log"` | "Camera consumer loop for camera and general commands" | `msg.Command Receiving Timestamp ← Tick Count`; `Enqueue(camera queue "Camera Mgr", msg)` | [main VI frame d66] |
| `Default` | "Unknown command" | nothing: no reply, no log. The command is still added to `Commands History`. | [main VI frame d65] |

Consequences for the wire:

- Routing works by enum **ranges** (a LabVIEW case label `"A".."B"` is an inclusive numeric range), so the numeric order of the `Camera Command` items is part of the contract. Every camera command except `Log` must lie between `Get Heartbeat` and `Cam Get Led`, `Log` lies outside that range, and every Z command must lie between `Z Setup` and `Z Do Reference`; otherwise its consumer case is unreachable **(inferred)**. `Exit` must be in the camera range for the server to be stoppable **(inferred)**. The Z case frames list `Z Do Reference` in the middle, so either it is the last Z item or several Z commands are never routed; case frame order is no proof of enum order **(unknown)**. The constraints are detailed in [03-command-reference.md](03-command-reference.md) §3.2.
- The item values, the order and the representation are **(unknown)**. The codes must come from the `.ctl` or a capture (questionnaire Q2). The codec table and the routing table must be derived from the same ordered list. The names and the per-command contracts are in [03-command-reference.md](03-command-reference.md).
- Both enqueues use an infinite timeout. While the camera or Z queue is full (5 elements), the TCP loop blocks, and with it reading and **all replies**.

### 7.3 Flattened variant format (inferred)

The printouts show no flattened variant, so the whole layout below is **(inferred)**. It comes from LabVIEW's published type-descriptor format and from community reverse engineering of LabVIEW ≥ 8 variants. **It must be confirmed with a capture before byte-exact work.**

| Part | Size | Content |
|---|---|---|
| version | U32 | LabVIEW version that flattened the data, e.g. `0x18008000` for LabVIEW 2018 (18.0). It depends on the client's runtime and must not be validated strictly. |
| type descriptor count | U32 | number of type descriptors (TDs) in the list |
| TDs | variable | Each TD = `U16 size` (counts the whole TD, size word included; a size word with bit 15 set announces a 32-bit size, not expected here) · `U8 flags` · `U8 type code` · type-specific content. A cluster, array or typedef TD refers to its element TDs, and a typedef TD (`0xF1`) also carries the typedef's name. Sizes are even. |
| top-level type list | U16 count + count × U16 | indices into the TD list. Normally one index, pointing at the variant's type. Some descriptions call the count a "has top type" flag followed by one index; the bytes are the same for a count of 0 or 1. |
| data | variable | the value flattened with the §6 rules, according to the top-level TD |
| attribute count | U32 | `0` expected. Each attribute = string name + flattened variant value. No attribute use is seen in the server. |

Common type codes (low byte of the second word of a TD) **(inferred)**:

| Code | Type | Code | Type |
|---|---|---|---|
| `0x00` | void (empty variant) | `0x21` | Boolean |
| `0x03` / `0x07` | I32 / U32 | `0x30` | string |
| `0x06` | U16 | `0x32` | path |
| `0x0A` | DBL | `0x40` | array |
| `0x15` / `0x16` / `0x17` | enum U8 / U16 / U32 (followed by the item names) | `0x50` | cluster |
| `0x53` | variant | `0xF1` | typedef wrapper |

A variant has **no overall length prefix**. In LabVIEW, the only way to reach the three timestamps after it is to decode the data completely.

### 7.4 Parsing and emitting `Command Data`

**Parsing (server).** The frame length delimits the payload exactly, and the client flattens the whole cluster with nothing after it **(inferred)**. The last 16 bytes of a `Command` payload are therefore `U32 attribute count (0)` plus the three U32 timestamps. The recommended parse:

1. Read the command code (2 bytes) and the notifier name.
2. Walk the variant header: version, TD count, TDs by their size words, then the top-level index list.
3. Data = the bytes from there up to `len(payload) − 16`. Check that the attribute count is 0, then read the timestamps.
4. Decode the data with the decoder of the command (field lists in [04-data-types.md](04-data-types.md)). Require that it consumes **exactly** the data bytes. Commands whose data the server never reads (e.g. `Get Heartbeat`, `Exit`, `Z Setup`) keep the raw bytes.

This locates the timestamps without understanding the TDs. Applied to the first real frames, it also answers several open points:

- the **representation of `Camera Type` and the other enums**: the type code of the enum TD is `0x15` (U8), `0x16` (U16) or `0x17` (U32);
- the **field order of `Camera Init Data`**: the order of the element TDs;
- whether an empty variant uses a void TD.

**Emitting (test client or simulator).** The TD part depends only on the type and on the client's LabVIEW build, never on the value. Capture one frame per command, store `version … top-level list` as a **template**, and emit `template ‖ data ‖ 00 00 00 00`. Generating TDs from scratch would need the exact typedef names and labels and is not recommended.

### 7.5 Worked examples

**(a) `Cam Set Exposure`, Low Mag., 1500 (schematic).** The data layout follows `Camera Set Exposure Data` = `{Camera Type: enum, Exposure: DBL}` ([04-data-types.md](04-data-types.md)). Only the data bytes are certain, and only if the enum is U16.

```
NN NN NN NN                 N = 2 + payload length
tt tt                       datatype "Command" (00 00 likely)                   (unknown)
cc cc                       Camera Command = "Cam Set Exposure"                 (unknown)
00 00 00 02 4e 31           Notifier Name "N1" (any unique text)
vv vv vv vv                 variant version, e.g. 18 00 80 00                   (inferred)
kk kk kk kk                 TD count                                            (inferred)
…                           TDs of typedef "Camera Set Exposure Data"           (unknown bytes)
00 01 ii ii                 1 top-level type, index ii                          (inferred)
00 01                       Camera Type = Low Mag. (value 1, U16)               (inferred)
40 97 70 00 00 00 00 00     Exposure = 1500.0 (DBL)
00 00 00 00                 variant attribute count = 0                         (inferred)
rr rr rr rr                 Command Request Timestamp
ss ss ss ss                 Command Sending Timestamp
xx xx xx xx                 Command Receiving Timestamp (overwritten by the server)
```

**(b) A fully inferred frame for a bare-DBL command** (`Z Set Min Limit` = 0.5, notifier `"Z#17"`, request/sending timestamps 1000/1001). This command's `Command Data` is a plain DBL ([03-command-reference.md](03-command-reference.md)). Everything except the DBL, the strings and the timestamps is **(inferred)**. The `Reader`/`Writer` of §11.1 reproduces it.

```
00 00 00 34  tt tt  cc cc                       N = 52, datatype, command code
00 00 00 04 5a 23 31 37                         "Z#17"
18 00 80 00  00 00 00 01  00 04 00 0a           version 18.0, 1 TD: size 4, flags 0, code 0x0A (DBL)
00 01 00 00                                     1 top-level type: index 0
3f e0 00 00 00 00 00 00                         0.5
00 00 00 00                                     0 attributes
00 00 03 e8  00 00 03 e9  00 00 00 00           timestamps 1000, 1001, 0
```

### 7.6 Malformed or unexpected commands: what LabVIEW does

| Situation | LabVIEW behaviour | Evidence |
|---|---|---|
| Payload too short for `Camera Queue Data` (including an empty `Command` frame, N = 2) | The `Unflatten` error is unwired. The output is then a default value, and it is still routed and executed **(inferred: the type's default, i.e. `Camera Command = Cam Setup`, empty notifier name, empty variant)**. | [main VI frame d0] |
| Extra bytes at the end of the payload | ignored | [main VI frame d0] (remainder unwired) |
| Command value outside both routing ranges (and not `Log`) | dropped silently; the client times out | [main VI frame d65] |
| Command value beyond the last enum item | either an `Unflatten` error (then the default element above is executed) or an out-of-range value that is dropped like the previous row **(unknown)** | [main VI frame d0] |
| Routed command with no case in the consumer loop | consumer `Default` frame: nothing, no reply | [main VI frames d73, d220] ([03-command-reference.md](03-command-reference.md)) |
| `Command Data` of the wrong type | `Variant To Data` fails. Its error is unwired in most commands, so the command runs with the typedef defaults (`Camera Type = High Mag.`, 0, FALSE, "") **(inferred)**. Per-command wiring: [03-command-reference.md](03-command-reference.md). | [main VI frames d79, d90, d141, d148] |

Recommendation: **fix**. Reject an undecodable frame with an error reply when the notifier name could be decoded, or log it otherwise. Never execute defaults.

---

## 8. Outbound response payload

### 8.1 `HW TCP Response Data`

| # | Field | Type | Flattened | Evidence |
|---|---|---|---|---|
| 0 | `Notifier Name` | string | I32 + bytes; copied **byte for byte** from the command | [Doc6 p.1160, p.437] |
| 1 | `Response (Flattened)` | string | I32 + bytes; content in §8.3 | [Doc6 p.1160] |
| 2 | `Command Request timestamp` | U32 | 4 | [Doc6 p.1160, p.573] |
| 3 | `TCP Command Sending timestamp` | U32 | 4 | idem |
| 4 | `TCP Command Receiving timestamp` | U32 | 4 | idem |
| 5 | `Pre-flattening data timestamp` | U32 | 4 | idem |
| 6 | `Post-Flattening data timestamp` | U32 | 4 | idem |
| 7 | `TCP Sending timestamp` | U32 | 4 | [main VI frame d69] |

- The two strings come first: the diagram constants show 2 strings, then 6 numbers [Doc6 p.437]. The order of the six U32 is the front-panel order, and the panel is a neat vertical stack, so it most likely matches the cluster order **(inferred)**. A wrong order would only swap diagnostic timestamps.
- U32 is shown by the `Tick Count` wires, which have no coercion dot [Doc6 p.573].
- Payload size: `(4 + len(name)) + (4 + len(response)) + 24`, so `N = payload + 2`.

### 8.2 Timestamps in replies

| Field | Image replies | All other replies |
|---|---|---|
| Command Request | copied from the command's `Command Request Timestamp` | 0 |
| TCP Command Sending | copied from the command's `Command Sending Timestamp` | 0 |
| TCP Command Receiving | copied from `Command Receiving Timestamp`, i.e. the server's Tick Count when the request was **first** routed (re-queued stream requests keep it) | 0 |
| Pre-flattening | server Tick Count before `IMAQ Flatten Image to String` | 0 |
| Post-Flattening | server Tick Count after it | 0 |
| TCP Sending | server Tick Count in the `Send Response` state | same |

Evidence: [Doc6 p.437, 445, 457, 465, 561, 1126] (constant zeros), [Doc6 p.573] (image), [main VI frame d69] (sending). Inside `Enqueue Send Image Response.vi` each timestamp input is bundled into the field of the same name [Doc6 p.573]. In the main VI the three wires from the request cross before reaching those inputs, so the one-to-one mapping in the table is **(inferred)** [main VI frame d104]. With both clocks on one PC, the client can compute queueing, flattening and transmission latencies **(inferred)**.

### 8.3 `Response (Flattened)` per reply type

Each `Enqueue Send <type> Response.vi` bundles its value and error into the constant of its `Wait for <type> Notifier Data` typedef and flattens it. The order is **Value, then error** in every typedef [Doc6 p.440, 452, 460, 537, 564, 1049].

| Reply type | Typedef (field labels) | Bytes of `Response (Flattened)` | Used by (details in [03](03-command-reference.md)) |
|---|---|---|---|
| Error cluster only | the `error in` cluster itself | `U8 status · I32 code · I32 len + source` (9 bytes when OK) | acknowledgements of all "set/do" commands |
| STR | `Wait for STR Notifier Data` {Value, error in (no error)} | `I32 len + text` · error | `Z Get Serial`, `Z Get Error` |
| Boolean | `Wait for Boolean Notifier Data` {Value, error} | `U8 00/01` · error | `Cam Get Acquisition Status`, `Cam Get Led`, `Z Get Status`, `Z Get Reference`, `Z Joystick Enabled ?` |
| DBL | `Wait for DBL Notifier Data` {Value, error in (no error)} | `8-byte IEEE-754` · error | `Cam Get Exposure`, `Cam Get Gain`, `Cam Get Autofocus Progress`, `Z Get Speed`, `Z Get Position` |
| U32 | `Wait for U32 Notifier Data` {Value, error} | `U32` · error | `Get Heartbeat` (server Tick Count) |
| DBL 2-D array | `Wait for DBL 2D Array Notifier Data` {Value, error} | `I32 rows · I32 cols · rows × cols DBL (row-major)` · error. An empty table may be 0 × 0 or 0 × 2 **(inferred)**. | `Cam Get Autofocus Data` (N × 2: [Z, score], see [06](06-autofocus-refocus.md) §2.6) |
| Image | `Wait for Image Notifier Data` {Flattened image, error in (no error)} | `I32 len + IMAQ flattened image` · error | `Cam Stream Next Image` |

The field labels differ ("error" or "error in (no error)"), but a label never reaches the wire.

### 8.4 The reply VIs, and when a reply is really sent

All seven VIs do the same thing:

1. `Obtain Queue(TCP Server Queue Name, create if not found = FALSE)`.
2. Build `HW TCP Response Data` from its typedef constant.
3. `Enqueue Element({State = "Send Response", State Data = variant(Response Data)})` with the default infinite timeout.

None of them calls `Release Queue`, so one queue reference leaks per reply. That is harmless in LabVIEW and should not be reproduced.

| VI (icon text) | Pages | `error in` of the `Enqueue` | Reply sent when the command failed? |
|---|---|---|---|
| `Enqueue Send Error Cluster Reponse.vi` ("CAM SERVER RESP") | [Doc6 p.435–438] | the VI's own `error in`, the same error that is flattened as the reply. The wire runs behind the `Flatten To String` node into `Enqueue Element`; the `Obtain Queue` error is unwired. | **Probably not** **(inferred, strong)**. `Enqueue` does not execute when `error in.status = TRUE`, so an error acknowledgement is never queued and the client waits until its own timeout. Successes and warnings are sent. `error out` = the `Enqueue` error output, i.e. `error in` when it carried an error. |
| `Enqueue Send STR / Boolean / DBL / U32 / DBL 2D Array Response.vi` | [Doc6 p.443–446, 455–458, 463–466, 559–562, 1124–1127] | error of `Obtain Queue` | yes: the command's error travels **as data** inside the reply |
| `Enqueue Send Image Response.vi` ("ENQUEUE IMAGE RESP") | [Doc6 p.571–574] | error of `Obtain Queue`, routed through the frame sequence and behind the reply's `Flatten To String` | yes. The command's error is the `error in` of `IMAQ Flatten Image to String`, which then does not run, so the image string is empty **(inferred)**; the flatten's `error out` is bundled into the reply as data and does not reach the `Enqueue`. |

The same wiring pattern (error wire passing behind a `Flatten To String` node) appears in `HW TCP Send Heartbeat.vi` [Doc6 p.1157] and `HW TCP Send Response.vi` [Doc6 p.1169]. There it certainly reaches `TCP Write`, which supports this reading of p.437.

**Decision for the re-implementation (questionnaire Q17).** Use the project switch `send_error_replies` ([10-migration-plan.md](10-migration-plan.md) §6). Recommended value: TRUE, i.e. always send the error acknowledgement. The reply type does not change (an error cluster, like a successful acknowledgement), so a client that decodes successful acknowledgements also decodes failed ones **(inferred)**. Until Q17 is answered or a capture shows the current behaviour, the switch keeps the LabVIEW behaviour (FALSE), as the migration plan requires for everything the client can observe. Confirm with a capture (command `Z Move Absolute` out of range, error 9405).

### 8.5 Image replies (`IMAQ Flatten Image to String`)

| Input | Value used by the server | VI default | Evidence |
|---|---|---|---|
| `Type of Flatten` | `Image and Vision Info` | `Image and Vision Info` | [Doc6 p.568, p.573] |
| `Compression` | `None` | `None` | idem |
| `Quality` | 1000 (irrelevant without compression) | 750 | idem |
| `Flatten with Process-Unique Name` | FALSE | FALSE | idem |

- The VI's diagram is a single Call Library Function node into NI Vision [Doc6 p.569]. The byte format is **NI-proprietary and undocumented** (unknown).
- "Image and Vision Info" carries the pixels plus the Vision information attached to the image **(inferred)**: overlays, calibration and the custom-data keys written by the acquisition state machine (see [05-camera-acquisition.md](05-camera-acquisition.md) §7.5).
- The client presumably rebuilds the image with `IMAQ Unflatten Image from String` **(inferred)**.

Re-implementation options:

| Option | Fidelity | Cost / risk |
|---|---|---|
| Call NI Vision's flatten function from Python (`ctypes` on the NI Vision C library; the function name and signature must be checked in `nivision.h` **(unknown)**) | Byte-identical only if it is the same code path as the VI **(unknown)** | needs Windows, the NI Vision Runtime and a licence (questionnaire Q6) |
| Reverse-engineer the format from captured replies of known images (U8/U16, several sizes, with and without custom data) | good for the cases captured | high effort; unknown structures (overlays, calibration) |
| Change the client to an open format (header + raw pixels, or PNG) | none: breaks compatibility | needs client changes (questionnaire Q1) |

The reply on error is `Flattened image = ""`. Its Response (Flattened) starts `00 00 00 00`, followed by the error cluster **(inferred)**.

### 8.6 Worked examples

**Complete reply to `Get Heartbeat`** (U32 value 123456; notifier `"NotifA"`; `TCP Sending timestamp` = 12345678; 57 bytes):

```
00 00 00 35                          N = 53
tt tt                                datatype "Response"                          (unknown)
00 00 00 06 4e 6f 74 69 66 41        Notifier Name "NotifA"
00 00 00 0d                          Response (Flattened): 13 bytes
   00 01 e2 40                         Value (U32) = 123456
   00 00 00 00 00 00 00 00 00          error {FALSE, 0, ""}
00 00 00 00                          Command Request timestamp
00 00 00 00                          TCP Command Sending timestamp
00 00 00 00                          TCP Command Receiving timestamp
00 00 00 00                          Pre-flattening data timestamp
00 00 00 00                          Post-Flattening data timestamp
00 bc 61 4e                          TCP Sending timestamp = 12345678
```

**`Response (Flattened)` contents** (computed with the §11.1 codec):

| Reply | Hex |
|---|---|
| error only, OK | `00 00000000 00000000` |
| error only, `{TRUE, 9405, "Z Move Absolute"}` (illustrative source text) | `01 000024bd 0000000f 5a204d6f7665204162736f6c757465` |
| STR `"hello"`, OK | `00000005 68656c6c6f 00 00000000 00000000` |
| Boolean TRUE, OK | `01 00 00000000 00000000` |
| Boolean FALSE with error `{TRUE, −1074360316, "Cam Acquisition status"}` (illustrative) | `00 01 bff69004 00000016 43616d204163717569736974696f6e20737461747573` |
| DBL 1500.0, OK | `4097700000000000 00 00000000 00000000` |
| U32 123456, OK | `0001e240 00 00000000 00000000` |
| DBL 2-D `[[1.0, 0.5], [2.0, 0.75]]`, OK | `00000002 00000002 3ff0000000000000 3fe0000000000000 4000000000000000 3fe8000000000000 00 00000000 00000000` |
| Image on error `{TRUE, 1, "e"}` (illustrative) | `00000000 01 00000001 00000001 65` |

---

## 9. Heartbeat

There are three distinct things called "heartbeat":

| Mechanism | What it is | Evidence |
|---|---|---|
| `Heartbeat` frames | A 6-byte frame `00 00 00 02 tt tt` built by `HW TCP Send Heartbeat.vi`. **The server never sends one:** the `Send Heartbeat` state exists, but nothing queues it, and its comment asks whether a heartbeat loop should be added. A `Heartbeat` frame **received** from the client is read and ignored. If it carries a payload, the read ends with the internal error −1 "Extra Data Found" (nothing is sent back) and the server closes the connection. | [Doc6 p.1153, p.1157], [main VI frames d64, d67, d68] |
| `Get Heartbeat` command | The application-level liveness check ("Envoie le heartbeat du serveur vers le client", i.e. "sends the server heartbeat to the client"). Routed to the camera loop; replies **U32 = server Tick Count (ms)** with no error. It is queued behind other camera commands, so a slow camera command delays it. | [main VI frame d96] |
| `Heartbeat` indicator | front-panel U32 set to Tick Count at every element dequeued by the TCP loop | [main VI frame d0] |

Re-implementation: answer `Get Heartbeat` from the camera worker, as LabVIEW does, so that it keeps measuring that loop's responsiveness. Accept and ignore incoming `Heartbeat` frames. Send none unless a capture shows the client expects them (questionnaire Q4).

---

## 10. Notifier-name correlation and the client's synchronous-call pattern

The client is not in the printouts (questionnaire Q31). The pattern below is **(inferred)** from the field names (`Notifier Name`, `Wait for <type> Notifier Data`), from the folder that holds the reply typedefs (`D:\ALOX\Vision Core\sources\synchronous calls\` [Doc6 p.439]) and from LabVIEW conventions:

```
client caller                    client TCP reader                    server
─────────────                    ─────────────────                    ──────
name ← unique string
Obtain Notifier(name)
send Command{cmd, name, data, ts} ─────────────────────────────────► execute
Wait on Notification(name, timeout)                                  Response{name, flat{value,error}, ts}
                                 read Response frame ◄────────────────
                                 unflatten Response Data
                                 Send Notification(Obtain Notifier(name), Response (Flattened))
◄─ wakes up: Unflatten(Response (Flattened), Wait for <type> Notifier Data) → value, error
   (timeout → client-side error; no reply ever arrives)
```

Rules the server must follow for an unchanged client:

1. **Copy `Notifier Name` byte for byte.** Never trim it, re-encode it or normalise its case. Treat it as opaque bytes, as the codec of §11.1 does: cp1252 leaves five byte values undefined (0x81, 0x8D, 0x8F, 0x90, 0x9D), so a decode/encode round trip is not always lossless.
2. **Send exactly the reply type the client expects for each command** ([03-command-reference.md](03-command-reference.md)). Any other type makes the client's `Unflatten` fail **(inferred)**.
3. **Send at most one reply per request.** `Cam Stream Next Image` re-queues the same message internally until the image is ready, and still sends exactly one reply ([05-camera-acquisition.md](05-camera-acquisition.md) §6.2).
4. **Replies may arrive in an order different from the requests.** The camera and Z loops run in parallel, and stream and autofocus requests are re-queued. Correlation is by name only. Within one loop the order is FIFO.
5. **Requests that never get a reply in LabVIEW:**
   - unknown commands;
   - the camera and Z `Default` cases;
   - `Exit`, `Cam Get Image`, `Cam Get Images Array`, `Cam Autofocus Run`, `Cam Refocus Run`;
   - error acknowledgements, probably (§8.4; confirm by capture C6).

   The client times out in these cases. Whether it expects a reply to these commands is **(unknown)**.

Reference sketch of the client side, for a Python test client:

```python
# Reference sketch: synchronous call over the ALOX protocol (test client side).
import asyncio, itertools
import lvwire as L

class Client:
    def __init__(self, reader, writer):
        self.r, self.w, self.pending, self.seq = reader, writer, {}, itertools.count()

    async def reader_task(self):
        while True:
            datatype, size = L.split_header(await self.r.readexactly(6))
            payload = await self.r.readexactly(size)
            if datatype == L.DATATYPE["Response"]:
                name, flattened, _ts = L.decode_response_data(payload)
                fut = self.pending.pop(name, None)
                if fut is not None and not fut.done():
                    fut.set_result(flattened)          # unknown names are ignored

    async def call(self, command: int, variant: bytes, timeout: float = 5.0) -> bytes:
        name = f"py-{next(self.seq)}".encode("ascii")    # notifier names are opaque bytes
        fut = asyncio.get_running_loop().create_future()
        self.pending[name] = fut
        self.w.write(L.encode_frame(L.DATATYPE["Command"], L.encode_command(command, name, variant)))
        await self.w.drain()
        try:
            return await asyncio.wait_for(fut, timeout)   # decode with Reader per expected type
        finally:
            self.pending.pop(name, None)
```

---

## 11. Python reference sketches

The codec, the server and the client of §10 were run against each other over loopback with synthetic frames: the hex examples of §7.5 (b) and §8.6, a header split across two TCP segments, an ignored `Heartbeat` frame, a notifier name containing byte 0x81, truncated payloads (every truncation must raise `ProtocolError`), a frame with N < 2 (connection closed), and a second client waiting for the first to disconnect. The variant headers used were the inferred ones of §7.3, not captured ones. The placeholders marked **(unknown)** must be replaced before use against the real client.

### 11.1 Codec (`lvwire.py`)

```python
"""Reference sketch: wire codec of the ALOX Camera Server (LabVIEW 2018 'Flatten To String', big-endian).

Placeholders marked (unknown) must be filled from a network capture or from the .ctl files.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Callable, Optional, Sequence

ENCODING = "cp1252"          # (inferred) LabVIEW strings are bytes in the Windows ANSI code page
MAX_PAYLOAD = 256 * 2**20    # sanity limit; LabVIEW has none

# HW TCP Datatype: names observed, numeric values (unknown). Command = 0 is likely (enum default).
DATATYPE = {"Command": 0, "Response": 1, "Image": 2, "Heartbeat": 3}     # PLACEHOLDER (unknown)

OK = (False, 0, "")          # LabVIEW "no error" cluster {status, code, source}


class ProtocolError(Exception):
    """Malformed frame or payload (LabVIEW would report error 74/116 or ignore it)."""


# ----------------------------------------------------------------------------- primitives
class Writer:
    def __init__(self) -> None:
        self.buf = bytearray()

    def raw(self, b: bytes) -> "Writer":
        self.buf += b
        return self

    def bool(self, v: bool) -> "Writer":
        return self.raw(b"\x01" if v else b"\x00")

    def u16(self, v: int) -> "Writer":
        return self.raw(struct.pack(">H", v))

    def i32(self, v: int) -> "Writer":
        return self.raw(struct.pack(">i", v))

    def u32(self, v: int) -> "Writer":
        return self.raw(struct.pack(">I", v & 0xFFFFFFFF))

    def dbl(self, v: float) -> "Writer":
        return self.raw(struct.pack(">d", v))

    def string(self, s: str | bytes) -> "Writer":
        b = s.encode(ENCODING) if isinstance(s, str) else bytes(s)
        return self.i32(len(b)).raw(b)

    def error(self, err: tuple[bool, int, str]) -> "Writer":
        status, code, source = err
        return self.bool(status).i32(code).string(source)

    def dbl_2d(self, rows: Sequence[Sequence[float]]) -> "Writer":
        n_rows = len(rows)
        n_cols = len(rows[0]) if n_rows else 0
        self.i32(n_rows).i32(n_cols)
        for row in rows:
            if len(row) != n_cols:
                raise ValueError("ragged 2-D array")
            self.raw(struct.pack(f">{n_cols}d", *row))
        return self

    def bytes(self) -> bytes:
        return bytes(self.buf)


class Reader:
    def __init__(self, data: bytes, pos: int = 0) -> None:
        self.data, self.pos = bytes(data), pos

    def take(self, n: int) -> bytes:
        if n < 0 or self.pos + n > len(self.data):
            raise ProtocolError(f"need {n} bytes at offset {self.pos}, have {len(self.data) - self.pos}")
        b = self.data[self.pos:self.pos + n]
        self.pos += n
        return b

    def bool(self) -> bool:
        return self.take(1) != b"\x00"          # any non-zero byte reads as TRUE

    def u16(self) -> int:
        return struct.unpack(">H", self.take(2))[0]

    def i32(self) -> int:
        return struct.unpack(">i", self.take(4))[0]

    def u32(self) -> int:
        return struct.unpack(">I", self.take(4))[0]

    def dbl(self) -> float:
        return struct.unpack(">d", self.take(8))[0]

    def string_bytes(self) -> bytes:
        return self.take(self.i32())

    def string(self) -> str:
        return self.string_bytes().decode(ENCODING, "replace")

    def error(self) -> tuple[bool, int, str]:
        return self.bool(), self.i32(), self.string()

    def dbl_2d(self) -> list[list[float]]:
        n_rows, n_cols = self.i32(), self.i32()
        return [list(struct.unpack(f">{n_cols}d", self.take(8 * n_cols))) for _ in range(n_rows)]


# ----------------------------------------------------------------------------- frames
def encode_frame(datatype: int, payload: bytes = b"") -> bytes:
    """I32 N (= 2 + len(payload)) | U16 datatype | payload."""
    return struct.pack(">iH", 2 + len(payload), datatype) + payload


def heartbeat_frame() -> bytes:
    return encode_frame(DATATYPE["Heartbeat"])


def split_header(header6: bytes) -> tuple[int, int]:
    """Return (datatype, payload_length) from the 6 header bytes."""
    n, datatype = struct.unpack(">iH", header6)
    if n < 2 or n - 2 > MAX_PAYLOAD:
        raise ProtocolError(f"bad frame length {n}")
    return datatype, n - 2


# ----------------------------------------------------------------------------- flattened variant (layout inferred)
@dataclass
class VariantHeader:
    version: int                 # LabVIEW version that flattened the data, e.g. 0x18008000 (inferred)
    type_descriptors: list[bytes]
    top_indices: list[int]
    raw: bytes                   # version .. indices; constant for a given type and client build

    @property
    def top_type_code(self) -> Optional[int]:
        if not self.top_indices:
            return None
        return self.type_descriptors[self.top_indices[0]][3]


def read_variant_header(r: Reader) -> VariantHeader:
    start = r.pos
    version, n_td = r.u32(), r.u32()
    tds = []
    for _ in range(n_td):
        size_word = r.take(2)
        size = struct.unpack(">H", size_word)[0]
        if size < 4 or size & 0x8000:          # bit 15 set = 32-bit size form, not expected here
            raise ProtocolError(f"unsupported type descriptor size {size:#x}")
        tds.append(size_word + r.take(size - 2))
    indices = [r.u16() for _ in range(r.u16())]
    if any(i >= n_td for i in indices):
        raise ProtocolError("variant top index out of range")
    return VariantHeader(version, tds, indices, r.data[start:r.pos])


def write_variant(header_template: bytes, data: bytes, n_attributes: int = 0) -> bytes:
    """Emit a variant from a captured header template (see section 7.4)."""
    if n_attributes:
        raise NotImplementedError("variant attributes are not used by this protocol")
    return header_template + data + struct.pack(">I", 0)


# ----------------------------------------------------------------------------- inbound command
@dataclass
class CameraQueueData:
    command: int                 # enum Camera Command, U16 assumed (unknown)
    notifier_name: bytes         # opaque: echoed byte for byte in the reply
    variant: VariantHeader
    data: object                 # decoded Command Data (None when the command has no decoder)
    data_bytes: bytes
    request_ts: int
    sending_ts: int
    receiving_ts: int


DataDecoder = Callable[[Reader], object]


def decode_command(payload: bytes, decoder_for: Callable[[int], Optional[DataDecoder]]) -> CameraQueueData:
    """Unflatten 'Camera Queue Data'. The variant has no length prefix, so its data is delimited from the
    END of the payload: U32 attribute count (0) + 3 x U32 timestamps = 16 bytes.
    Decoders must raise ProtocolError on malformed data (Reader does)."""
    r = Reader(payload)
    command = r.u16()
    notifier = r.string_bytes()
    header = read_variant_header(r)
    if len(payload) - r.pos < 16:
        raise ProtocolError("payload too short for the variant trailer and the timestamps")
    data_bytes = payload[r.pos:len(payload) - 16]
    tail = Reader(payload, len(payload) - 16)
    if tail.u32() != 0:
        raise ProtocolError("variant attributes present (unexpected)")
    timestamps = (tail.u32(), tail.u32(), tail.u32())
    decoder = decoder_for(command)          # None: Command Data not read by the server (kept raw)
    data = None
    if decoder is not None:
        dr = Reader(data_bytes)
        data = decoder(dr)
        if dr.pos != len(data_bytes):
            raise ProtocolError(f"command {command}: {len(data_bytes) - dr.pos} unexpected data bytes")
    return CameraQueueData(command, notifier, header, data, data_bytes, *timestamps)


def encode_command(command: int, notifier: str | bytes, variant: bytes, ts: Sequence[int] = (0, 0, 0)) -> bytes:
    """Client side (test tool): Flatten 'Camera Queue Data'; 'variant' is already flattened."""
    w = Writer().u16(command).string(notifier).raw(variant)
    for t in ts:
        w.u32(t)
    return w.bytes()


# ----------------------------------------------------------------------------- outbound response
def flat_error(err=OK) -> bytes:
    return Writer().error(err).bytes()


def flat_str(value: str, err=OK) -> bytes:
    return Writer().string(value).error(err).bytes()


def flat_bool(value: bool, err=OK) -> bytes:
    return Writer().bool(value).error(err).bytes()


def flat_dbl(value: float, err=OK) -> bytes:
    return Writer().dbl(value).error(err).bytes()


def flat_u32(value: int, err=OK) -> bytes:
    return Writer().u32(value).error(err).bytes()


def flat_dbl_2d(rows: Sequence[Sequence[float]], err=OK) -> bytes:
    return Writer().dbl_2d(rows).error(err).bytes()


def flat_image(imaq_flattened: bytes, err=OK) -> bytes:
    """imaq_flattened = output of IMAQ Flatten Image to String (NI format), b'' on error."""
    return Writer().string(imaq_flattened).error(err).bytes()


def encode_response_data(notifier: bytes | str, flattened: bytes, ts: Sequence[int] = (0,) * 6) -> bytes:
    """HW TCP Response Data. ts order (cluster order inferred from the front panel):
    Command Request, TCP Command Sending, TCP Command Receiving, Pre-flattening, Post-Flattening, TCP Sending."""
    if len(ts) != 6:
        raise ValueError("6 timestamps expected")
    w = Writer().string(notifier).string(flattened)
    for t in ts:
        w.u32(t)
    return w.bytes()


def response_frame(notifier: bytes | str, flattened: bytes, ts: Sequence[int] = (0,) * 6) -> bytes:
    return encode_frame(DATATYPE["Response"], encode_response_data(notifier, flattened, ts))


def decode_response_data(payload: bytes) -> tuple[bytes, bytes, tuple[int, ...]]:
    r = Reader(payload)
    notifier = r.string_bytes()
    flattened = r.string_bytes()
    return notifier, flattened, tuple(r.u32() for _ in range(6))
```

Usage on the server: `decode_command(payload, decoder_for)`, where `decoder_for(code)` returns a function `Reader → dataclass` built from the layouts of [04-data-types.md](04-data-types.md), or `None` for commands whose data is not read. Replies: `response_frame(msg.notifier_name, flat_dbl(value, err))`. Large 2-D arrays and images can use `numpy` (`np.asarray(a, ">f8").tobytes()`), which gives the same bytes.

### 11.2 TCP front end (`server.py`)

This sketch keeps the observable behaviour of §3–§4: loopback only, one client at a time, whole frames, other datatypes ignored, replies serialised, and the `TCP Sending timestamp` set at send time. It drops the artificial 50 ms pause and the 10 ms polling, and closes the connection on a mid-frame timeout.

```python
"""Reference sketch: TCP front end of the camera server (asyncio). One client at a time, like the LabVIEW loop."""
from __future__ import annotations

import asyncio
import logging
import time
from typing import Awaitable, Callable, Optional, Sequence

from lvwire import (DATATYPE, CameraQueueData, ProtocolError, decode_command, response_frame, split_header)

log = logging.getLogger("tcp")


def tick_ms() -> int:
    """LabVIEW Tick Count (ms): free-running U32."""
    return int(time.monotonic() * 1000) & 0xFFFFFFFF


class TcpFrontEnd:
    def __init__(self, port: int, route: Callable[[CameraQueueData], Awaitable[None]],
                 decoder_for, host: str = "127.0.0.1") -> None:
        self.host, self.port = host, port          # LabVIEW: "localhost", [hardware.camera] port_I32
        self.route = route                          # puts the message on the camera or Z queue
        self.decoder_for = decoder_for
        self._writer: Optional[asyncio.StreamWriter] = None
        self._one_client = asyncio.Lock()           # later clients wait in the backlog, as in LabVIEW
        self._send_lock = asyncio.Lock()            # whole frames only, never interleaved

    async def serve_forever(self) -> None:
        server = await asyncio.start_server(self._handle, self.host, self.port)
        async with server:
            await server.serve_forever()

    async def send_response(self, notifier: bytes, flattened: bytes, ts: Sequence[int] = (0,) * 6) -> None:
        ts = list(ts)
        ts[5] = tick_ms()                           # "TCP Sending timestamp", set for every reply
        frame = response_frame(notifier, flattened, ts)
        async with self._send_lock:
            writer = self._writer
            if writer is None:                      # LabVIEW writes to a dead refnum: reply lost
                log.warning("reply for %r dropped: no client", notifier)
                return
            writer.write(frame)
            try:
                await asyncio.wait_for(writer.drain(), 1.0)   # LabVIEW: 1000 ms. D12 recommends a longer, size-based timeout for image replies (01 §10.3 uses 5 s).
            except (asyncio.TimeoutError, ConnectionError) as exc:
                log.error("send failed (%s): closing the connection", exc)
                writer.close()

    async def _handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        async with self._one_client:
            self._writer = writer
            try:
                while True:
                    header = await reader.readexactly(4)                        # idle wait
                    header += await asyncio.wait_for(reader.readexactly(2), 0.2)
                    datatype, size = split_header(header)
                    payload = await asyncio.wait_for(reader.readexactly(size), 5.0)
                    if datatype != DATATYPE["Command"]:
                        continue                    # LabVIEW reads and ignores Heartbeat/Image/Response
                    try:
                        msg = decode_command(payload, self.decoder_for)
                    except ProtocolError as exc:
                        log.error("undecodable command frame: %s", exc)   # LabVIEW: undefined (see section 7.6)
                        continue
                    await self.route(msg)
            except (asyncio.IncompleteReadError, asyncio.TimeoutError, ConnectionError, ProtocolError) as exc:
                log.info("connection ended: %r", exc)
            finally:
                self._writer = None
                writer.close()
```

`route` must not block for long, because LabVIEW's routing `Enqueue` blocks while the consumer queue is full. The camera and Z workers are threads ([01-architecture.md](01-architecture.md) §10.2). They send a reply with `asyncio.run_coroutine_threadsafe(front.send_response(...), loop).result()`, as described in the 01 §10.3 notes. A single-writer variant, where replies go through an unbounded queue to the TCP task and only that task writes, is equally acceptable ([10-migration-plan.md](10-migration-plan.md) §3.3). Both keep frames from interleaving. For an image reply, the worker fills `ts[0..4]` (§8.2) before calling `send_response`. The sketch does not implement the fix for D10 (§13): to drop replies that belong to an earlier connection, tag each routed message with a connection counter and have `send_response` discard replies whose counter differs from the current one.

### 11.3 Compatibility switches

This table lists the switches owned by the protocol layer. The complete registry of `[compat]` switches is in [10-migration-plan.md](10-migration-plan.md) §6. They live in the new Python-only settings file, not in `alox.ini` ([09-configuration-logging-errors.md](09-configuration-logging-errors.md) §12 item 2; questionnaire Q36). The first two switches are the project-wide ones of [10-migration-plan.md](10-migration-plan.md) §6. Following its rule, a behaviour the client can observe keeps the LabVIEW default until the owner or a capture confirms the client does not depend on it.

| Switch | LabVIEW behaviour | Initial default | Recommended once confirmed |
|---|---|---|---|
| `send_error_replies` | Error-cluster replies with `status = TRUE` are probably not sent (§8.4; inferred from [Doc6 p.437], to be confirmed by capture C6) | FALSE (LabVIEW) | TRUE: always reply (questionnaire Q17, capture C6) |
| `reply_unknown_commands` | Unknown or unrouted commands: no reply | FALSE (LabVIEW) | TRUE: reply with an error cluster if the client accepts one for that command; otherwise log only |
| `listen_address` | `127.0.0.1` | `127.0.0.1` | `127.0.0.1` (changing it exposes an unauthenticated control port; questionnaire Q36) |
| `inbound_pause_ms` | ≈ 50 ms pause after each inbound frame (§4.6) | 0 (not observable by a sane client, D8) | 0 |
| `close_on_extra_heartbeat_data` | a heartbeat with a payload closes the connection | TRUE | TRUE (harmless) |
| `datatype_codes`, `command_codes`, `enum_size` | fixed by the `.ctl` files (unknown) | loaded from configuration, filled from a capture | — |

---

## 12. What only a network capture (or the `.ctl` files) can confirm

A Wireshark capture of the real client talking to the current LabVIEW server (questionnaire Q4) settles most points in one pass. Opening the `.ctl` files (Edit Items, Reorder Controls in Cluster) settles the enum points without a capture.

| # | Point | How to read it in a capture |
|---|---|---|
| C1 | Numeric codes of `HW TCP Datatype` | bytes 4–5 of a command frame (client) and of a reply frame (server) |
| C2 | Representation and codes of `Camera Command` | bytes 6–7 (U16) or 6 (U8) or 6–9 (U32) of each command frame: the I32 length of the notifier name follows the code. One frame per command gives the whole table. |
| C3 | Variant layout (version, TD list, top-level index list, attribute count) | `Command` payloads; check that the last 16 bytes are `00 00 00 00` plus 3 timestamps |
| C4 | Representation of `Camera Type` and the other enums inside `Command Data`; field order of `Camera Init Data` | type codes and order of the TDs |
| C5 | Content of `Command Data` for commands whose data is not read (`Get Heartbeat`, `Exit`, `Z Setup` …): empty variant or not | those frames |
| C6 | Whether error acknowledgements are sent | provoke `Z Move Absolute` out of range (error 9405) or a camera error, and look for a reply with that notifier name |
| C7 | Whether the client sends `Heartbeat` frames, and how often | frames with `N = 2` |
| C8 | Byte format of `IMAQ Flatten Image to String`, including custom data | replies to `Cam Stream Next Image` for known images (U8/U16, several sizes, flat-field on/off) |
| C9 | Order of the six reply timestamps | an image reply: expect Request ≤ Command Sending ≤ Command Receiving ≤ Pre-flattening ≤ Post-Flattening ≤ TCP Sending |
| C10 | Client timeouts and the client's reaction to missing or unexpected replies | timing between request and next action; a session where a command fails |
| C11 | Notifier-name format; whether the client pipelines requests or reconnects | any session |
| C12 | String encoding (cp1252) | a notifier name or error source with non-ASCII characters, if any |

---

## 13. Defects and oddities in the LabVIEW implementation

| # | Defect / oddity | Effect | Recommendation |
|---|---|---|---|
| D1 | Error acknowledgements are probably never queued, because `error in` gates the `Enqueue` (§8.4) | the client sees a timeout instead of the error | **ask** and confirm by capture (Q17, C6); recommended **fix** through `send_error_replies`, which keeps the LabVIEW behaviour until confirmed (§11.3) |
| D2 | Unknown commands, the consumer `Default` cases, `Exit`, `Cam Get Image`, `Cam Get Images Array` and the internal `… Run` commands send no reply | client timeouts | unknown and unrouted commands: recommended **fix** (error reply, log) through `reply_unknown_commands`; the others: **reproduce** until the client's expectations are known (**ask**) |
| D3 | `Read` error → `TCP Close` → `Enqueue Listen` gated by the close error | if `TCP Close` fails, the server never listens again: `Read` fails, `Close` fails, … forever | **fix**: always go back to listening |
| D4 | `Connected` is not reset on disconnection; `Connector` is not the remote address | misleading UI | **fix** (diagnostics only) |
| D5 | Mid-frame timeouts are handled like an idle timeout (code 56 → `Read`) | stream desynchronisation; garbage "headers" | **fix**: close the connection (§5.5) |
| D6 | No limit on `N`; N < 2 gives a negative read size | a huge allocation or a dropped connection | **fix**: validate `N` |
| D7 | `Unflatten` error unwired when decoding a command; wrong-type variants run with defaults | malformed frames can execute `Cam Setup` or a command with default parameters **(inferred)** | **fix**: reject and log |
| D8 | ≈ 50 ms pause after each inbound frame; 10 ms polling | throughput limited to about 20 commands/s; latency | **fix** (not observable by a sane client) |
| D9 | Reads, writes and `Listen` share one loop; the 2 s `Listen` and the 5 s payload reads delay replies | latency | **fix** (async I/O) |
| D10 | Replies are not bound to a connection: queued replies go to whatever client is connected, or are lost | a stale reply after a reconnection | **fix**: drop replies that belong to a previous connection |
| D11 | The TCP loop enqueues into its own bounded queue with an infinite timeout; routing enqueues block while the consumer queues are full | possible permanent deadlock | **fix**: unbounded internal queues or non-blocking self-posts ([01-architecture.md](01-architecture.md)) |
| D12 | `TCP Write` timeout of 1000 ms, even for multi-megabyte image replies; write errors ignored | a partial frame corrupts the stream **(inferred)** | **fix**: longer or size-based timeout; close on failure |
| D13 | A `Heartbeat` frame with a payload closes the connection (error −1 "Extra Data Found"); if the announced payload is incomplete, the loop blocks for up to 25 s first and then keeps the connection (§5.3) | surprising; a stalled loop | **reproduce** the close (or ignore the payload); **fix** the 25 s stall |
| D14 | `Send Heartbeat` and `Quit` states are dead code | none | do not reproduce |
| D15 | A queue reference leaks per reply (`Obtain Queue` without `Release Queue`) | memory growth in LabVIEW | do not reproduce |
| D16 | The listener is bound to `localhost` and accepts no second client while one is served (backlog only) | remote clients impossible; a second local client hangs | **reproduce** (security), document (questionnaire Q36) |
| D17 | A `Listen` failure other than a timeout (port taken by another program) is retried at once, forever, with no log **(inferred)** (§3) | busy loop; the server looks alive but never accepts | **fix**: fail at start-up (or retry with a delay) and log the error |

---

## 14. Open questions

Only the items that change the re-implementation are listed. The questionnaire numbers refer to [open-questions.md](open-questions.md).

1. **Is the client unchanged and LabVIEW-based?** This decides between byte-compatible LabVIEW flattening, including variants and NI images, and a simpler protocol (Q1).
2. **Enum tables**: the codes and representations of `HW TCP Datatype` and `Camera Command` (Q2, C1, C2). Without them no frame can be decoded. The routing ranges also depend on the `Camera Command` order.
3. **Variant layout**: confirm §7.3 and collect one header template per command (Q4, C3–C5).
4. **Error acknowledgements and unknown commands**: are error acknowledgements sent today, and does the client rely on timing out (Q17, C6)? The answer sets `send_error_replies` and `reply_unknown_commands`.
5. **Image format**: will the NI Vision Runtime remain available, so that `IMAQ Flatten Image to String` can be reproduced byte for byte, or must the client change (Q5, Q6, C8)?
6. **Client heartbeat**: does the client send `Heartbeat` frames, and does it expect any from the server (questionnaire Q4, capture C7)?
7. **Client timeouts and pipelining**: per-command timeouts, and whether several requests are outstanding at once (questionnaire Q32, Q31; captures C10, C11). This sets how strictly the reply latency of the LabVIEW loop must be respected.
