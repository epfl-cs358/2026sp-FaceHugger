<!--
WIKI-MIGRATION REFERENCE COPY
Original path:  doc/gait-design/specs/esp32-playback-engine.md
Original kind:  spec
Copied on:      2026-05-21
Status:         unreviewed
Maps to:        reference/animation/fhc-format.md
-->

> **Reference material.** Verbatim copy of `doc/gait-design/specs/esp32-playback-engine.md` kept in `_context/`
> as source for the published wiki page. Edit the parent wiki page, not
> this file. The original at `doc/gait-design/specs/esp32-playback-engine.md` is still canonical; this file is
> scaffolding the user will delete manually once the wiki pages exist.
# Spec: ESP32 Playback Engine

> Defines the firmware-side gait playback: interpolation, state machine, command queue, inversion, speed scaling.
> See also: `gait-file-format.md` (data it consumes), `runtime-api-and-control.md` (how commands reach it).

---

## 1. Overview

The playback engine is a 50 Hz timer loop that:
1. Computes the current position within the active gait cycle
2. Interpolates servo angles from sparse keyframes
3. Writes PWM signals to 12 servos
4. At cycle boundaries, checks a command queue for gait switches

---

## 2. Core Loop (50 Hz / 20ms tick)

```
every 20ms:
  if (state == IDLE) return;

  // 1. Compute cycle position
  uint32_t elapsed = millis() - cycle_start_ms;
  uint32_t scaled  = (elapsed * speed_x100) / 100;
  uint32_t cycle_time = scaled % current_gait->duration_ms;

  // 2. Detect cycle boundary (did we wrap?)
  bool new_cycle = (scaled / current_gait->duration_ms) > completed_cycles;
  if (new_cycle) {
      completed_cycles++;
      handle_cycle_boundary();  // check command queue
  }

  // 3. For each servo track
  for (int i = 0; i < 12; i++) {
      gait_track_t* track = &current_gait->tracks[i];

      // Binary search for surrounding keyframes
      keyframe_t* kf_a = find_keyframe_before(track, cycle_time);
      keyframe_t* kf_b = kf_a + 1;  // next keyframe

      // Normalized time between keyframes (0.0–1.0)
      float t_norm = (float)(cycle_time - kf_a->time_ms)
                   / (float)(kf_b->time_ms - kf_a->time_ms);

      // Interpolate
      float angle = interpolate(
          kf_a->angle_x10 / 10.0f,
          kf_b->angle_x10 / 10.0f,
          t_norm,
          track->interp
      );

      // Apply inversion
      if (inverted) angle = 270.0f - angle;

      // Write to servo
      set_servo_angle(track->servo_id, angle);
  }
```

---

## 3. Interpolation Functions

### Linear
```c
float interp_linear(float a, float b, float t) {
    return a + (b - a) * t;
}
```

### Cubic (smoothstep)
```c
float interp_cubic(float a, float b, float t) {
    float s = t * t * (3.0f - 2.0f * t);  // smoothstep: 3t² - 2t³
    return a + (b - a) * s;
}
```
Smoothstep gives zero velocity at keyframe boundaries — servos accelerate and decelerate naturally. This is the recommended default for gait animations.

### Constant (step)
```c
float interp_constant(float a, float b, float t) {
    (void)b; (void)t;
    return a;  // hold until next keyframe
}
```

### Dispatch
```c
float interpolate(float a, float b, float t, uint8_t type) {
    switch (type) {
        case INTERP_LINEAR:   return interp_linear(a, b, t);
        case INTERP_CUBIC:    return interp_cubic(a, b, t);
        case INTERP_CONSTANT: return interp_constant(a, b, t);
        default:              return interp_linear(a, b, t);
    }
}
```

---

## 4. Gait State Machine

### States

| State | Description |
|-------|-------------|
| `IDLE` | No gait playing. All servos hold neutral pose. |
| `PLAYING` | Active gait cycle looping. Interpolation engine running. |

### Transitions

```
IDLE + PLAY(gait) → PLAYING
    Set current_gait, cycle_start_ms = millis(), completed_cycles = 0

PLAYING + cycle boundary + queue has gait → PLAYING (new gait)
    Set current_gait to queued gait, reset cycle_start_ms

PLAYING + cycle boundary + queue has STOP → IDLE
    Servos are at neutral pose (cycle just ended at t=0 equivalent)

PLAYING + cycle boundary + queue empty → PLAYING (same gait)
    Continue looping, reset cycle_start_ms
```

### Why no TRANSITIONING state

All gait cycles start and end at the same neutral pose. When one cycle ends and another begins, the servo positions are identical (neutral). No blending or interpolation between gaits is needed. This is the key architectural simplification.

---

## 5. Command Queue

### Design

```c
#define CMD_QUEUE_SIZE 4

typedef enum { CMD_PLAY, CMD_STOP } cmd_type_t;

typedef struct {
    cmd_type_t type;
    const gait_t* gait;    // NULL for STOP
    uint8_t speed_x100;    // 100 = normal speed
    bool inverted;
} gait_cmd_t;

// Circular buffer
gait_cmd_t cmd_queue[CMD_QUEUE_SIZE];
uint8_t cmd_head = 0, cmd_tail = 0;
```

### Behavior

- Commands are **enqueued** when received (from REST API or serial)
- Commands are **dequeued** only at cycle boundaries
- If the queue is full, the oldest unprocessed command is dropped (or the new one is rejected — TBD)
- A STOP command clears the rest of the queue (STOP is terminal)
- A new PLAY command while IDLE starts immediately (no queuing needed)

### Why queue instead of immediate switching

Mid-cycle gait switches cause jerky motion because the servo positions at an arbitrary point in cycle A don't match the start of cycle B. By waiting for the cycle boundary (where all servos are at neutral), transitions are always smooth.

---

## 6. Inverted Mode

For running the robot upside-down (legs flipped over).

### Transform

```c
if (inverted) {
    angle = 270.0f - angle;
}
```

### Details

- Applied **after** interpolation, **before** PWM write
- Assumes all servos use the same 0–270° range
- Works because flipping the robot reverses the direction of gravity relative to the joints — mirroring the angle achieves the same physical motion in the inverted orientation
- The `inverted` flag is set per-command: `{"name": "walk_forward", "inverted": true}`
- May need per-joint inversion logic if yaw axes behave differently when flipped — test on hardware

### When NOT to use

If the inverted gait needs different timing or foot placement (not just mirrored angles), create a separate gait cycle instead.

---

## 7. Speed Multiplier

### How it works

```c
uint32_t elapsed = millis() - cycle_start_ms;
uint32_t scaled_time = (elapsed * speed_x100) / 100;
uint32_t cycle_time = scaled_time % gait.duration_ms;
```

- `speed_x100 = 100` → normal speed (1.0×)
- `speed_x100 = 120` → 20% faster
- `speed_x100 = 80`  → 20% slower

### Limitations

- Useful range: ~80–120 (±20%). Beyond that, the motion looks unnatural because:
  - Faster: servo acceleration may exceed physical capability, causing missed positions
  - Slower: the robot appears to float/hover unnaturally
- For significantly different speeds, create a new gait cycle with appropriate timing
- Speed is set per-command and persists for the gait's lifetime

### Fixed-point storage

```c
uint8_t speed_x100;  // 0–255, where 100 = 1.0×
                      // Max speed: 2.55× (more than enough)
```

---

## 8. Servo PWM Interface

### Configuration (`servo_config.h`)

```c
#define NUM_SERVOS 12

typedef struct {
    uint8_t gpio_pin;
    uint16_t pwm_min_us;   // microseconds for 0°   (typically ~500)
    uint16_t pwm_max_us;   // microseconds for 270° (typically ~2500)
    float angle_min;        // software limit (degrees)
    float angle_max;        // software limit (degrees)
} servo_config_t;

static const servo_config_t SERVO_CONFIG[NUM_SERVOS] = {
    {.gpio_pin = 13, .pwm_min_us = 500, .pwm_max_us = 2500, .angle_min = 10, .angle_max = 260},
    // ... 11 more
};
```

### Angle to PWM conversion

```c
void set_servo_angle(uint8_t servo_id, float angle_deg) {
    servo_config_t* cfg = &SERVO_CONFIG[servo_id];

    // Clamp to software limits
    if (angle_deg < cfg->angle_min) angle_deg = cfg->angle_min;
    if (angle_deg > cfg->angle_max) angle_deg = cfg->angle_max;

    // Map angle to PWM microseconds
    uint16_t pwm_us = cfg->pwm_min_us
        + (uint16_t)((angle_deg / 270.0f) * (cfg->pwm_max_us - cfg->pwm_min_us));

    // Write to hardware (ESP32 LEDC or servo library)
    write_pwm(cfg->gpio_pin, pwm_us);
}
```

### PWM details for ESP32

- Use ESP32 LEDC peripheral or `ESP32Servo` library
- PWM frequency: 50 Hz (standard servo protocol, 20ms period)
- Resolution: 16-bit timer for microsecond precision
- All 12 servos can run on separate LEDC channels (ESP32 has 16)

---

## 9. Performance Notes

| Operation | Time | Notes |
|-----------|------|-------|
| Binary search (10 keyframes) | ~1 µs | 4 comparisons max |
| Interpolation (1 servo) | ~0.5 µs | float multiply + add |
| Full tick (12 servos) | ~20 µs | Well within 20ms budget |
| PWM write (12 channels) | ~50 µs | LEDC register writes |
| Total per tick | ~70 µs | 0.35% CPU usage |

The playback engine is extremely lightweight. The ESP32 has plenty of headroom for the REST API server, WiFi, and any sensor processing.
