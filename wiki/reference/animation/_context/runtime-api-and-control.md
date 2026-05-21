<!--
WIKI-MIGRATION REFERENCE COPY
Original path:  doc/gait-design/specs/runtime-api-and-control.md
Original kind:  spec
Copied on:      2026-05-21
Status:         unreviewed
Maps to:        reference/firmware/api.md
-->

> **Reference material.** Verbatim copy of `doc/gait-design/specs/runtime-api-and-control.md` kept in `_context/`
> as source for the published wiki page. Edit the parent wiki page, not
> this file. The original at `doc/gait-design/specs/runtime-api-and-control.md` is still canonical; this file is
> scaffolding the user will delete manually once the wiki pages exist.
# Spec: Runtime API & Control Interface

> Defines the REST API, command queuing, and control flow between the app/controller and the ESP32 gait engine.
> See also: `esp32-playback-engine.md` (the engine these commands control).

---

## 1. Overview

The ESP32 runs a REST API (WiFi). An app or controller sends gait commands. The playback engine executes them. Commands are queued and processed at gait cycle boundaries for smooth transitions.

---

## 2. REST API Endpoints

### `POST /gait` — Queue a gait command

**Request body:**
```json
{
  "name": "walk_forward",
  "speed": 1.0,
  "inverted": false
}
```

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `name` | string | yes | — | Gait name (must match a registered gait) or `"stop"` |
| `speed` | float | no | 1.0 | Speed multiplier (0.5–2.0 range, stored as ×100 uint8) |
| `inverted` | bool | no | false | Apply angle inversion for upside-down mode |

**Responses:**

| Status | Body | Meaning |
|--------|------|---------|
| 200 | `{"status": "queued", "queue_depth": 2}` | Command added to queue |
| 200 | `{"status": "started"}` | Robot was idle, gait started immediately |
| 400 | `{"error": "unknown_gait", "name": "xyz"}` | Gait name not in registry |
| 503 | `{"error": "queue_full"}` | Command queue is full (capacity 4) |

**Special name: `"stop"`**
- Clears the command queue
- Finishes the current cycle
- Returns to IDLE at neutral pose

### `GET /gait/status` — Current playback state

**Response:**
```json
{
  "state": "playing",
  "current_gait": "walk_forward",
  "cycle_progress": 0.65,
  "speed": 1.0,
  "inverted": false,
  "queue": ["turn_left"],
  "queue_depth": 1
}
```

| Field | Type | Description |
|-------|------|-------------|
| `state` | string | `"idle"` or `"playing"` |
| `current_gait` | string | Name of active gait, or `null` if idle |
| `cycle_progress` | float | 0.0–1.0, position within current cycle |
| `speed` | float | Current speed multiplier |
| `inverted` | bool | Current inversion state |
| `queue` | [string] | Names of queued gaits (in order) |
| `queue_depth` | int | Number of commands in queue |

### `GET /gait/list` — Available gaits

**Response:**
```json
{
  "gaits": [
    {"name": "walk_forward", "duration_ms": 800},
    {"name": "turn_left", "duration_ms": 600},
    {"name": "turn_right", "duration_ms": 600},
    {"name": "stand", "duration_ms": 400}
  ]
}
```

### `POST /gait/clear` — Clear command queue

Empties the queue without stopping the current gait. The current cycle continues to loop.

**Response:**
```json
{"status": "cleared", "commands_dropped": 3}
```

---

## 3. Command Flow

```
App sends POST /gait {"name": "turn_left"}
    ↓
ESP32 REST handler validates gait name
    ↓
If state == IDLE:
    Start gait immediately, respond {"status": "started"}
If state == PLAYING:
    Push to command queue, respond {"status": "queued"}
If queue full:
    Respond 503 {"error": "queue_full"}
    ↓
At next cycle boundary (current gait finishes one full loop):
    Pop command from queue
    Switch to new gait
    Reset cycle timer
```

---

## 4. Control Patterns

### Basic walking

```
POST /gait {"name": "walk_forward"}    → starts walking
// ... some time later ...
POST /gait {"name": "stop"}            → finishes cycle, stops
```

### Turn while walking

```
POST /gait {"name": "walk_forward"}    → walking
POST /gait {"name": "turn_left"}       → queued, starts after current walk cycle ends
POST /gait {"name": "walk_forward"}    → queued, resumes walking after turn
```

### Emergency stop vs graceful stop

- **Graceful:** `POST /gait {"name": "stop"}` — finishes current cycle, neutral pose
- **Emergency:** Could add a `POST /gait/emergency_stop` that immediately writes neutral pose to all servos (mid-cycle, potentially jerky). Not in v1 — add if needed.

### Speed adjustment

```
POST /gait {"name": "walk_forward", "speed": 1.2}   → slightly faster walk
```

Speed applies to the queued command. To change speed of the currently playing gait mid-cycle, a separate `PATCH /gait/speed` endpoint could be added later.

### Flip mode

```
POST /gait {"name": "walk_forward", "inverted": true}   → upside-down walk
```

---

## 5. Error Handling

| Scenario | Behavior |
|----------|----------|
| Unknown gait name | 400 error, command rejected |
| Queue full | 503 error, command rejected. App should retry or clear queue. |
| WiFi disconnect mid-gait | Gait continues looping indefinitely. Add a watchdog timer (stop if no command received in N seconds). |
| Servo overcurrent / stall | Not handled in v1. ESP32 doesn't have current sensing on basic PWM. |

### Watchdog (recommended for v1)

If no REST API request is received within a configurable timeout (e.g. 5 seconds), automatically queue a STOP command. Prevents the robot from walking indefinitely if the controller disconnects.

---

## 6. Implementation Notes

- The REST API runs on the ESP32's secondary core (or in the main loop with async handling)
- The gait playback engine runs on a hardware timer interrupt at 50 Hz
- Shared state between the API thread and the timer (command queue, current state) needs a mutex or atomic operations
- Use ArduinoJSON for parsing request bodies and building responses
- Gait name lookup: linear search through `GAIT_REGISTRY` (12 entries max, O(1) effectively)

---

## 7. Future Considerations

- **WebSocket streaming:** Push real-time servo angles to the app for visualization / debugging
- **Gait upload via WiFi:** Send `.gait` JSON over the API and parse at runtime (requires JSON parser + dynamic memory allocation — possible but adds complexity)
- **Sensor integration:** IMU feedback could adjust speed or trigger gait switches autonomously
- **Sequence macros:** Define named sequences like "patrol" = [walk_forward × 5, turn_left, walk_forward × 5, turn_right] that the ESP32 plays without per-cycle commands
