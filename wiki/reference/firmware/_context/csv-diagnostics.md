<!--
WIKI-MIGRATION REFERENCE COPY
Original path:  code/firmware/docs/csv-diagnostics.md
Original kind:  spec
Copied on:      2026-05-21
Status:         unreviewed
Maps to:        reference/firmware/csv-diagnostics.md
-->

> **Reference material.** Verbatim copy of `code/firmware/docs/csv-diagnostics.md` kept in `_context/`
> as source for the published wiki page. Edit the parent wiki page, not
> this file. The original at `code/firmware/docs/csv-diagnostics.md` is still canonical; this file is
> scaffolding the user will delete manually once the wiki pages exist.
<!--
CONSISTENCY-CHECK 2026-05-21
Verified against: feat/wiki-setup @ 0a1a138
- [ok]    24-column CSV header matches csv_log.cpp kHeader byte-for-byte (millis..bl_knee), incl. fr/fl/br/bl ordering; kColumnCount=24, kMaxRowBytes=192 (csv_log.h:27,40).
- [ok]    Tunables in diagnostics.h: kSampleIntervalMs=200, kBufBytes=8*1024, kHttpPort=80 (diagnostics.h:14-24). HTTP routes GET / , GET /log, POST /log/clear all present in diagnostics.cpp:49-51; /log/clear returns "OK\n".
- [ok]    Enum legends correct: robot_state 0..4 incl. 4=REST (data.h RobotState); gait 0=NONE,1=WALK,2=TROT,3=CRAB (movements.h GaitType).
- [drift] Doc says POST /log/clear returns "200 OK\n"; handleClear() actually sends body "OK\n" with HTTP 200 (diagnostics.cpp:33) — fine, but the literal body is "OK\n" not "200 OK\n".
- [drift] Doc names module file "src/brain/diagnostics.h" tunables and the pure core "csv_log.{h,cpp}, csv_log_ring.inl" — all present. But the original-path header (line 3) points at code/firmware/docs/csv-diagnostics.md, which exists; OK.
- [todo] "~140 bytes/row", "~11.7 s window", and ESP32 "~320 KB / ~14% used" figures are estimates not derivable from source here — leave for Phase B / on-hardware measurement.
- [todo] Ring eviction edge-cases ("evict just enough", multi-row eviction, row>capacity dropped) are claimed unit-tested in test/test_csv_log — not re-run in this audit.
-->
# CSV Diagnostics over HTTP

Live state observability for the FaceHugger ESP32. The robot keeps a rolling
RAM buffer of CSV-formatted state samples and serves them over HTTP on port
**80**, independent of the WebSocket control path on port **81**.

Use it during bring-up and tuning to answer "what was the robot doing in the
last 10 seconds?" without instrumenting individual prints.

---

## Quick start

Once the firmware is flashed:

1. Join the WiFi AP **`FaceHugger_Net`** (password `12345678`).
2. From any host on the network (laptop, phone, etc.):
   ```bash
   curl http://192.168.4.1/log              # dump header + rolling buffer
   curl http://192.168.4.1/log | column -ts, # nicely aligned columns
   curl -X POST http://192.168.4.1/log/clear # wipe and start fresh
   ```
3. Or load `http://192.168.4.1/` in a browser for a help page listing routes.

The default IP is `192.168.4.1` (the soft-AP default for arduino-esp32); use
your robot's actual AP IP if it differs.

---

## HTTP API

| Method | Path         | Description                                                    |
|--------|--------------|----------------------------------------------------------------|
| GET    | `/`          | Plain-text help listing the routes.                            |
| GET    | `/log`       | CSV header line, then the rolling buffer contents. Chunked.    |
| POST   | `/log/clear` | Wipe the buffer. Returns `200 OK\n`.                           |

`/log/clear` is intentionally **POST** rather than GET — clearing is a
state-changing action and we don't want a browser prefetch or `curl --retry`
firing it twice.

---

## CSV columns

24 columns, one row per sample. The header is emitted once at the top of
every `/log` response so the contents are self-describing.

| Column                                         | Type   | Meaning                                                                 |
|------------------------------------------------|--------|-------------------------------------------------------------------------|
| `millis`                                       | uint32 | ESP32 uptime at sample time, milliseconds.                              |
| `robot_state`                                  | enum   | `RobotState` value: 0=IDLE, 1=WALK, 2=ACTION, 3=FAILSAFE, 4=REST.       |
| `gait`                                         | enum   | `GaitType`: 0=NONE, 1=WALK, 2=TROT, 3=CRAB.                             |
| `is_moving`                                    | 0 / 1  | True if the joystick reports motion intent.                             |
| `is_inverted`                                  | 0 / 1  | True after `invertRobot()`.                                             |
| `last_cmd_ms`                                  | uint32 | `millis()` of the last received WebSocket command (deadman timer).      |
| `target_x`, `target_y`, `target_yaw`           | float  | Raw joystick targets in [-1, +1]. 2 decimals.                           |
| `active_x`, `active_y`, `active_yaw`           | float  | Smoothed (low-pass) joystick values used by the gait. 2 decimals.       |
| `fr_hip`, `fr_thigh`, `fr_knee`                | float  | Front-right leg servo angles last commanded, degrees. 1 decimal.        |
| `fl_hip`, `fl_thigh`, `fl_knee`                | float  | Front-left.                                                             |
| `br_hip`, `br_thigh`, `br_knee`                | float  | Back-right.                                                             |
| `bl_hip`, `bl_thigh`, `bl_knee`                | float  | Back-left.                                                              |

**Leg-naming note.** Columns use the firmware convention (`fr`/`fl`/`br`/`bl`
= `leg1..leg4` in `SpinalCord`), *not* the URDF/Python convention
(`fl`/`fr`/`bl`/`br`). Downstream Python plotting code should remap names
explicitly. See [`code/simulation/docs/MERGE_AND_CONVENTION.md`](../../simulation/docs/MERGE_AND_CONVENTION.md)
for the full convention table.

**Precision note.** Servo angles are stored internally as `uint16_t` (whole
degrees) by `Servo::setServoAngle`. The CSV emits them as `90.0` etc. for
formatting consistency — the trailing `.0` is cosmetic, not real precision.

---

## Tunable parameters

All tunables live at the top of [`src/brain/diagnostics.h`](../src/brain/diagnostics.h)
as `constexpr` values. Changing them requires a firmware rebuild + flash.

```cpp
namespace diagnostics {

constexpr unsigned long kSampleIntervalMs = 200;   // sampling period
constexpr std::size_t   kBufBytes         = 8 * 1024;  // 8 KB ring
constexpr int           kHttpPort         = 80;    // HTTP port

}
```

### Picking the right values

Rolling window is approximately:

```
window_seconds ≈ (kBufBytes / row_bytes) × kSampleIntervalMs / 1000
```

A row is currently ~140 bytes (see `csv_log::kMaxRowBytes = 192` for the
hard upper bound). With defaults:

```
(8192 / 140) × 200 / 1000 ≈ 11.7 s of history.
```

A few common adjustments:

| Goal                                 | Suggested change                                                |
|--------------------------------------|-----------------------------------------------------------------|
| Higher temporal resolution           | `kSampleIntervalMs = 100` (~5.8 s window at same buffer).       |
| Longer history at same density       | `kBufBytes = 16 * 1024` (~23 s window).                         |
| Lower CPU / network footprint        | `kSampleIntervalMs = 500` (~29 s window, sparser samples).      |

ESP32 has ~320 KB of usable RAM (~14% used at rest with WiFi + WebSocket).
Doubling the buffer to 16 KB is safe; 64 KB starts to crowd things.

`kHttpPort` rarely needs to change — but if you do, make sure it doesn't
collide with **81** (WebSocket).

---

## Architecture

Two modules, both under [`src/brain/`](../src/brain/):

```
┌──────────────────────────┐     ┌──────────────────────────────┐
│ csv_log  (pure logic)    │     │ diagnostics  (Arduino glue)  │
│ - State POD              │     │ - WebServer on port 80       │
│ - header() + format_row()│ ──▶ │ - 8 KB Ring<N> in BSS        │
│ - Ring<N> template       │     │ - rate-gated tick() sampler  │
│   (drop-oldest-row)      │     │ - GET /log, POST /log/clear  │
│ - native-unit-tested     │     │ - Arduino-only, hardware-run │
└──────────────────────────┘     └──────────────────────────────┘
```

- **`csv_log`** is dependency-free C++ (`<cstdio>`, `<cstdint>`, `<cstddef>`,
  `<cstring>`). It builds and runs under the PlatformIO `native` env, so the
  row formatter and ring buffer are unit-tested on the host with Unity.
  Header and row formatter share a column list pinned by a comma-count
  invariant test, so they can't drift apart.
- **`diagnostics`** is the Arduino-side integration: owns a `WebServer`, a
  `Ring<kBufBytes>`, and the `millis()`-based rate gate. Three public verbs:
  `begin(sc)`, `tick()`, `handleHttp()`.

State flows like this:

```
SpinalCord  ── snapshot() ───►  Snapshot POD  ── tick() ──►  csv_log::State
                                                                  │
                                                          format_row()
                                                                  │
                                                                  ▼
                                                          Ring<8192>.append()
```

Concurrency: ESP32 user code runs single-threaded in `loop()`. Both
`diagnostics::tick()` and the HTTP route handlers (via `handleHttp()` →
`gServer.handleClient()`) mutate the ring, but they run serially on the
same thread, so no locking is required.

---

## Ring buffer semantics

The buffer is **row-aligned circular**: when a new row would overflow the
ring, the oldest whole rows are evicted (tail advances past one `\n` at a
time) until enough room exists. Rows are never split mid-content.

- New rows must end with `\n` — `Ring::append()` rejects malformed input.
- Reading via `Ring::view(d1, s1, d2, s2)` returns up to two contiguous
  segments to handle wrap; the HTTP handler streams them in order.
- Eviction is "evict just enough", not "evict to half" — minimum churn.

Edge cases (all unit-tested):

- Empty ring → empty view.
- Row exactly equals capacity → accepted, fills the buffer.
- Row larger than capacity → silently dropped.
- Row without `\n` → silently dropped.
- Single append needing multi-row eviction → handled.

---

## Building and testing

From [`code/firmware/`](../):

```bash
# Build the target firmware (includes diagnostics):
pio run -e upesy_wroom

# Run native unit tests for csv_log (no hardware needed):
pio test -e native -f test_csv_log

# Flash and watch:
pio run -e upesy_wroom -t upload
pio device monitor
```

Native tests verify the CSV formatter and ring buffer in isolation. The HTTP
server itself is tested manually on hardware — see "Quick start" above.

---

## What this is NOT

- **Not a long-term log**. No filesystem persistence, no LittleFS. Restart
  the ESP32 → buffer is empty. By design — this is for live debugging, not
  flight recording.
- **Not a high-rate telemetry stream**. 200 ms ÷ 24 columns is plenty for
  watching gait + joystick behaviour; it's not a substitute for an
  oscilloscope on the PWM lines.
- **Not authenticated**. Anyone on `FaceHugger_Net` can read `/log` and
  clear the buffer. The AP is closed-network by nature (single-purpose,
  manually-joined), so this is acceptable for the project's threat model.

---

## Related

- [`code/firmware/src/brain/diagnostics.{h,cpp}`](../src/brain/) — the module.
- [`code/firmware/src/brain/csv_log.{h,cpp}`, `csv_log_ring.inl`](../src/brain/) — the pure-logic core.
- [`code/firmware/test/test_csv_log/test_csv_log.cpp`](../test/test_csv_log/test_csv_log.cpp) — Unity tests.
- [`code/API_SPEC.md`](../../API_SPEC.md) — WebSocket control protocol (port 81, separate path).
