# CSV diagnostics

Live state observability for the FaceHugger ESP32. The robot keeps a rolling RAM buffer of CSV-formatted state samples and serves them over HTTP on port **80**, independent of the WebSocket control path on port **81**.

Use it during bring-up and tuning to answer "what was the robot doing in the last 10 seconds?" without instrumenting individual print statements.

---

## Quick start

Once the firmware is flashed:

1. Join the WiFi AP **`FaceHugger_Net`** (password `12345678`).
2. From any host on the network (laptop, phone, etc.):
   ```bash
   curl http://192.168.4.1/log               # dump header + rolling buffer
   curl http://192.168.4.1/log | column -ts, # nicely aligned columns
   curl -X POST http://192.168.4.1/log/clear # wipe and start fresh
   ```
3. Or load `http://192.168.4.1/` in a browser for a help page listing routes.

The default IP is `192.168.4.1` (the soft-AP default for arduino-esp32); use your robot's actual AP IP if it differs.

---

## HTTP API

| Method | Path         | Description                                                    |
|--------|--------------|----------------------------------------------------------------|
| GET    | `/`          | Plain-text help listing the routes.                            |
| GET    | `/log`       | CSV header line, then the rolling buffer contents. Chunked.    |
| POST   | `/log/clear` | Wipe the buffer. Returns body `OK\n` with HTTP 200.            |

`/log/clear` is intentionally POST rather than GET - clearing is a state-changing action and we don't want a browser prefetch or `curl --retry` firing it twice.

---

## CSV columns

24 columns, one row per sample. The header is emitted once at the top of every `/log` response so the contents are self-describing.

| Column                               | Type   | Meaning                                                            |
|--------------------------------------|--------|--------------------------------------------------------------------|
| `millis`                             | uint32 | ESP32 uptime at sample time, milliseconds.                         |
| `robot_state`                        | enum   | `RobotState`: 0=IDLE, 1=WALK, 2=ACTION, 3=FAILSAFE, 4=REST.       |
| `gait`                               | enum   | `GaitType`: 0=NONE, 1=WALK, 2=TROT, 3=CRAB.                       |
| `is_moving`                          | 0 / 1  | True if the joystick reports motion intent.                        |
| `is_inverted`                        | 0 / 1  | True after `invertRobot()`.                                        |
| `last_cmd_ms`                        | uint32 | `millis()` of the last received WebSocket command (deadman timer). |
| `target_x`, `target_y`, `target_yaw` | float  | Raw joystick targets in [-1, +1]. 2 decimals.                      |
| `active_x`, `active_y`, `active_yaw` | float  | Smoothed (low-pass) joystick values used by the gait. 2 decimals.  |
| `fr_hip`, `fr_thigh`, `fr_knee`      | float  | Front-right leg servo angles last commanded, degrees. 1 decimal.   |
| `fl_hip`, `fl_thigh`, `fl_knee`      | float  | Front-left.                                                        |
| `br_hip`, `br_thigh`, `br_knee`      | float  | Back-right.                                                        |
| `bl_hip`, `bl_thigh`, `bl_knee`      | float  | Back-left.                                                         |

**Leg-naming note.** Columns use the firmware convention (`fr`/`fl`/`br`/`bl` = `leg1..leg4` in `SpinalCord`), not the URDF/Python convention (`fl`/`fr`/`bl`/`br`). Downstream Python plotting code should remap names explicitly. See [Conventions](../conventions.md) for the full leg-naming and angle-space tables.

**Precision note.** Servo angles are stored internally as `uint16_t` (whole degrees) by `Servo::setServoAngle`. The CSV emits them as `90.0` etc. for formatting consistency - the trailing `.0` is cosmetic, not real precision.

---

## Tunable parameters

All tunables live at the top of `src/brain/diagnostics.h` as `constexpr` values. Changing them requires a firmware rebuild and flash.

```cpp
namespace diagnostics {

constexpr unsigned long kSampleIntervalMs = 200;   // sampling period
constexpr std::size_t   kBufBytes         = 8 * 1024;  // 8 KB ring
constexpr int           kHttpPort         = 80;    // HTTP port

}
```

The rolling window is approximately:

```
window_seconds = (kBufBytes / row_bytes) x kSampleIntervalMs / 1000
```

A row is approximately 140 bytes (`csv_log::kMaxRowBytes = 192` is the hard upper bound). With defaults that gives approximately 11.7 s of history.

Common adjustments:

| Goal                           | Suggested change                                              |
|--------------------------------|---------------------------------------------------------------|
| Higher temporal resolution     | `kSampleIntervalMs = 100` (~5.8 s window at same buffer).     |
| Longer history at same density | `kBufBytes = 16 * 1024` (~23 s window).                       |
| Lower CPU / network footprint  | `kSampleIntervalMs = 500` (~29 s window, sparser samples).    |

The ESP32 has approximately 320 KB of usable RAM (approximately 14% used at rest with WiFi and WebSocket active). Doubling the buffer to 16 KB is safe; 64 KB starts to crowd things.

`kHttpPort` rarely needs to change, but if you do change it, make sure it does not collide with **81** (WebSocket).

---

## Architecture

Two modules, both under `src/brain/`:

- **`csv_log`** (`csv_log.h`, `csv_log.cpp`, `csv_log_ring.inl`) is dependency-free C++ using only `<cstdio>`, `<cstdint>`, `<cstddef>`, and `<cstring>`. It builds and runs under the PlatformIO `native` environment, so the row formatter and ring buffer are unit-tested on the host with Unity. The header and row formatter share a column list pinned by a comma-count invariant test, preventing them from drifting apart.

- **`diagnostics`** (`diagnostics.h`, `diagnostics.cpp`) is the Arduino-side integration. It owns a `WebServer` on port 80, a `Ring<kBufBytes>` in BSS, and a `millis()`-based rate gate. Three public verbs: `begin(sc)`, `tick()`, and `handleHttp()`.

State flows from `SpinalCord` through a `snapshot()` call to produce a `Snapshot` POD, which `tick()` passes to `csv_log::format_row()`, and the resulting line is appended to `Ring<8192>`. The HTTP handler streams the ring contents in response to `GET /log`.

Concurrency: ESP32 user code runs single-threaded in `loop()`. Both `diagnostics::tick()` and the HTTP route handlers run serially on the same thread, so no locking is required.

---

## Ring buffer semantics

The buffer is row-aligned circular: when a new row would overflow the ring, the oldest whole rows are evicted (tail advances past one `\n` at a time) until enough room exists. Rows are never split mid-content.

- New rows must end with `\n` - `Ring::append()` rejects malformed input.
- Reading via `Ring::view(d1, s1, d2, s2)` returns up to two contiguous segments to handle wrap; the HTTP handler streams them in order.
- Eviction is "evict just enough", not "evict to half", minimising churn.

Edge cases (all unit-tested):

- Empty ring returns an empty view.
- A row exactly equal to capacity is accepted and fills the buffer.
- A row larger than capacity is silently dropped.
- A row without `\n` is silently dropped.
- A single append needing multi-row eviction is handled correctly.

---

## Building and testing

From `code/firmware/`:

```bash
# Build the target firmware (includes diagnostics):
pio run -e upesy_wroom

# Run native unit tests for csv_log (no hardware needed):
pio test -e native -f test_csv_log

# Flash and watch:
pio run -e upesy_wroom -t upload
pio device monitor
```

Native tests verify the CSV formatter and ring buffer in isolation. The HTTP server itself is tested manually on hardware - see Quick start above.

---

## What this is not

- **Not a long-term log.** No filesystem persistence, no LittleFS. Restart the ESP32 and the buffer is empty. This is by design - the feature is for live debugging, not flight recording.
- **Not a high-rate telemetry stream.** 200 ms at 24 columns is sufficient for watching gait and joystick behaviour; it is not a substitute for an oscilloscope on the PWM lines.
- **Not authenticated.** Anyone on `FaceHugger_Net` can read `/log` and clear the buffer. The AP is a closed, manually-joined network, so this is acceptable for the project's threat model.

---

## Related

- `code/firmware/src/brain/diagnostics.{h,cpp}` - the Arduino-side module.
- `code/firmware/src/brain/csv_log.{h,cpp}`, `csv_log_ring.inl` - the pure-logic core.
- `code/firmware/test/test_csv_log/test_csv_log.cpp` - Unity tests.
- [WebSocket API](../api.md) - control protocol (port 81, separate path).
