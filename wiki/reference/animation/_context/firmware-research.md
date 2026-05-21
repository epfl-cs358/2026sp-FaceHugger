<!--
WIKI-MIGRATION REFERENCE COPY
Original path:  doc/animation-pipeline/firmware-research.md
Original kind:  research
Copied on:      2026-05-21
Status:         unreviewed
Maps to:        reference-only
-->

> **Reference material.** Verbatim copy of `doc/animation-pipeline/firmware-research.md` kept in `_context/`
> as source for the published wiki page. Edit the parent wiki page, not
> this file. The original at `doc/animation-pipeline/firmware-research.md` is still canonical; this file is
> scaffolding the user will delete manually once the wiki pages exist.
# ESP32 12-DOF Quadruped Firmware — Research Synthesis

Consolidated synthesis of three LLM research answers — Claude, Gemini, Perplexity — to the original prompt. Each top-level section corresponds to one part of that prompt.

---

## On-Board IK Feasibility on the ESP32

Synthesis of Claude, Gemini, and Perplexity answers to section 1 of the original prompt.

### TL;DR

- **IK is essentially free.** 12-DOF closed-form IK at 50–100 Hz consumes **<5 % of one core** even with pessimistic timings. The real bottleneck is servo PWM bandwidth (~100 Hz mechanical), not compute.
- **Pin tasks: WiFi → core 0 (PRO_CPU), IK + servo loop → core 1 (APP_CPU)** via `xTaskCreatePinnedToCore`. WiFi tasks live on core 0 by default; pinning your real-time loop to core 1 is the proven pattern.
- **Use single-precision floats** (`atan2f`, `acosf`, `sqrtf`). Double precision is software-emulated and ~10× slower.
- **Multiple open-source ESP32 quadrupeds already do this**: [PingguSoft/esp32_quadruped](https://github.com/PingguSoft/esp32_quadruped) at 100 Hz, [runeharlyk/SpotMicroESP32-Leika](https://github.com/runeharlyk/SpotMicroESP32-Leika) with full kinematics + web UI, [PetoiCamp/OpenCatEsp32](https://github.com/PetoiCamp/OpenCatEsp32) for 12-servo skill playback.

---

### Consensus across the three sources

| Fact | Notes |
|------|-------|
| ESP32 has hardware FPU on Xtensa LX6 | Single-precision only |
| `atan2f`, `acosf`, `sqrtf` are libm software routines | They *use* the FPU for add/mul inside their iterative approximations, but are not single hardware ops |
| Use `float`, never `double` | Double is software-emulated; Espressif explicitly recommends `sinf`/`atanf` etc. ([esp32.com](https://esp32.com/viewtopic.php?t=18765)) |
| WiFi stack is pinned to core 0 by default | Application code (Arduino `loop()`) runs on core 1 by default ([forum.arduino](https://forum.arduino.cc/t/does-arduino-code-and-wifi-run-on-same-processor/1302509)) |
| 50 Hz is generous; 100 Hz is normal | PingguSoft runs `HW_SERVO_UPDATE_FREQ = 100 Hz` on ESP32 |
| FreeRTOS lazy FPU context-switch | First FPU access per task triggers a one-time exception/enable. Don't measure it in benchmarks |

---

### Disagreement: how slow is `atan2f` actually?

This matters because it determines your CPU budget headroom. The three sources give wildly different numbers:

| Operation | Claude | Gemini | Perplexity |
|-----------|--------|--------|------------|
| `sinf` / `cosf` | ~15.8 µs | (not directly stated; implied few cycles) | **8.6 µs** ([Espressif esp-dsp issue #6](https://github.com/espressif/esp-dsp/issues/6)) — measured 20.3 µs on C3 ([groups.google](https://groups.google.com/g/softrf_community/c/XQK3dkG1sYE/m/xenCdCD7AgAJ)) |
| `atan2f` at 240 MHz | 16–25 µs (estimate) | **~1.0 µs** ("150–300 cycles") | 10–20 µs (worst case) |
| `acosf` at 240 MHz | 20–30 µs (estimate) | (not stated) | same envelope as `atan2f` |
| 20 trig ops per IK update | **310–400 µs** total | **~30 µs** total | **172–400 µs** total |
| % of core at 50 Hz | 1.5–2 % | 0.15 % | 0.9–2 % |

#### What's likely true

**Trust Perplexity.** It is the only answer with citations to actual measurements, including Espressif's own statement that `sin`/`cos` ≈ 2060 cycles ≈ 8.6 µs at 240 MHz, and a community measurement of 20.3 µs on the closely related ESP32-C3.

Gemini's "~1.0 µs" estimate is implausible — 150–300 cycles cannot accommodate a polynomial approximation that calls into libm. It would require `atan2f` to be a hardware instruction, which it isn't. **Use 10–25 µs per trig call as your design number.**

The good news: **the conclusion is the same regardless** of which timing you believe. Even at the pessimistic end (400 µs total IK per update, 2 % of one core at 50 Hz), you have ~98 % of core 1 idle. At the optimistic end you have ~99.85 % idle. **IK is not your bottleneck.**

> Note from Claude worth keeping: the actual servo bandwidth limit (~100 Hz mechanical for hobby servos) means running IK faster than 200–500 Hz is wasted compute. 50–100 Hz is the right target.

---

### CPU budget breakdown (using conservative ~20 µs/trig)

Per IK update, per leg:
- 1× `atan2f` (Link1 yaw): ~20 µs
- 1× `acosf` (knee, law of cosines): ~25 µs
- 1× `atan2f` (hip, derived from knee): ~20 µs
- ~5–10 µs arithmetic overhead (mul, add, `sqrtf`)

**Per leg: ~70–80 µs. Four legs: ~300 µs per cycle.**

At 50 Hz (20 ms period): ~1.5 % of one core.
At 100 Hz (10 ms period): ~3 %.
At 200 Hz (5 ms period): ~6 %.

If you ever feel pinched, [Rob Tillaart's FastTrig](https://github.com/RobTillaart/FastTrig) lookup tables trade accuracy for ~10× speedup — but you almost certainly will never need them.

---

### Dual-core architecture

The standard ESP32 split:

```
┌─────────────────────────┐       ┌──────────────────────────┐
│ CORE 0 (PRO_CPU)        │       │ CORE 1 (APP_CPU)         │
│  • WiFi stack           │       │  • Gait scheduler        │
│  • AsyncWebServer       │       │  • Per-leg phase clocks  │
│  • WebSocket / OTA      │  ⇄    │  • IK solver             │
│  • File loading task    │ Queue │  • PCA9685 / LEDC PWM    │
└─────────────────────────┘       │  • 50–100 Hz fixed loop  │
                                  └──────────────────────────┘
```

API:
```c
xTaskCreatePinnedToCore(
    ikServoTask,    // task fn
    "IKServo",
    4096,           // stack
    NULL,
    10,             // priority (above background)
    NULL,
    1               // core ID — APP_CPU
);
```

#### Caveats all three sources flag

- **Don't make blocking calls in WiFi tasks.** WiFiManager / Blynk-style blocking reconnect logic on core 0 has been observed to stall core 1 ([stackoverflow](https://stackoverflow.com/questions/72295242/core0-blocking-code-from-core1-to-run-on-esp32-isnt-supposed-to-run-both-code)).
- **Both cores share the SPI flash bus** for instruction cache refills. Two caches compete for one SPI interface, causing arbitration jitter on cache misses. Mitigation: mark IK hot-loop functions with `IRAM_ATTR` so they live in instruction RAM and don't trigger refills.
- **Use hardware PWM** (LEDC peripheral, or external PCA9685 over I2C) for servo pulses. Software bit-banging at 50 Hz is fine in theory but can be jittered by WiFi interrupts ([reddit](https://www.reddit.com/r/esp32/comments/189aja8/help_wifi_on_core_0_is_interfering_with_time/)). PCA9685 over I2C @ 400 kHz can update 16 channels in ~200 µs — well within budget.
- **FreeRTOS queues** between cores are thread-safe and the right primitive for "gait switch" commands flowing core 0 → core 1.

---

### Real ESP32 quadrupeds running on-board IK

#### [PingguSoft/esp32_quadruped](https://github.com/PingguSoft/esp32_quadruped) — closest architectural match

- 12-DOF SpotMicro/Kangal-style robot.
- **Closed-form 3-DOF leg IK on-board**, using `atan`/`asin`/`acos`/`sqrt` per leg — essentially identical to your scheme.
- `QuadRuped` class with `HW_SERVO_UPDATE_FREQ = 100 Hz`.
- Each 10 ms tick: gait phase update → IK for 4 legs → write angles to PCA9685.
- Same firmware *also* runs SPIFFS web file browser (`FSBrowser`), OTA, BLE joystick handling, and IMU balancing. Direct proof of the dual-core pattern working under realistic load.
- Caveat: gaits are designed in an external Processing simulator, then on-device the firmware uses **analytic gait parameters** (step vectors, swing offsets, `stepsPerSec`) — no big keyframe tables. Less expressive but very memory-efficient.

#### [runeharlyk/SpotMicroESP32-Leika](https://github.com/runeharlyk/SpotMicroESP32-Leika) — most modern stack

- ESP32-CAM quadruped, full body-frame kinematics + per-foot IK on-board.
- **Bezier-curve trot gait + 8-phase crawl**, all parametric.
- Embeds a Svelte web app served from the ESP32 itself — exactly the "WiFi + IK on one chip" pattern you're aiming for.
- FreeRTOS tasks for "sense, plan, act" including video streaming, gait planning, and choreographed animation.
- Gemini calls this the "gold standard"; it's the closest architectural reference for what you're building.

#### [PetoiCamp/OpenCatEsp32](https://github.com/PetoiCamp/OpenCatEsp32) — best skill-library reference

- ESP32 framework for Boston Dynamics-style quadrupeds, supports up to 12 servos.
- **Stores skill frames as `int8_t` arrays** in flash (e.g. `carpetForward`) — compact, expressive, but no runtime IK; pure pre-baked angle playback.
- Standard hardware: up to 23 frames × 20 joints (~467 B). On the BiBoard ESP32 variant: up to 125 frames (~2.5 KB). Mirrored leftward gaits → rightward at runtime to halve storage.
- Best reference for *expressive* / choreographed animation storage if you separate that from locomotion.

#### Other useful references

- [Denis Bujoreanu, Hackster.io](https://www.hackster.io/denis-bujoreanu/quadruped-robot-dog-with-12dof-and-ik-07d047) — direct analogue of your hardware (ESP32-WROOM-CAM + PCA9685 + 12 MG90S + MPU6050), runs IK on-board.
- [Michael Seyoum, Hackster.io](https://www.hackster.io/mikroller/robot-leg-inverse-kinematics-and-generic-gait-with-esp32-e78d1e) — minimal generic IK + gait demo on ESP32.
- [ViolinLee/NodeQuad12-MicroPython](https://github.com/ViolinLee/NodeQuad12-MicroPython) — even MicroPython on a comparable board can do 2-link IK + parameterized gaits, which puts a comfortable lower bound on what's possible.
- [YouTube: "Quadruped robot dog – esp32 use both cores"](https://www.youtube.com/watch?v=3wH0poJt0Mc) — explicit demo of moving services to one core, IK/HAL on the other, 20 ms loop.

---

### Bottom line

| Question | Answer |
|----------|--------|
| Can ESP32 run 12-DOF IK on-board at 50 Hz? | **Yes, trivially.** ~1–3 % of one core. |
| Can it coexist with a WiFi webserver? | **Yes**, with the standard core 0 / core 1 pin pattern. Multiple shipped projects do exactly this. |
| What's the actual bottleneck? | **Servo PWM bandwidth** (~100 Hz hobby-servo mechanical limit) and **flash/WiFi SPI contention** for cache misses on core 1. Mitigate the latter with `IRAM_ATTR` on the hot loop. |
| Should I worry about FPU performance? | **No, but use `float` not `double`.** Don't pre-optimize with FastTrig-style lookup tables until benchmarks prove you need them. |
## Memory Constraints in Practice

Synthesis of Claude, Gemini, and Perplexity answers to section 2 of the original prompt.

### TL;DR

- **You have ~200–280 KB of free heap** with WiFi enabled. Plenty for a full gait library.
- **All 14 gait/animation patterns fit in ≤30 KB** under any reasonable encoding. Just preload everything at boot.
- **Do *not* read gait files from LittleFS during the 50 Hz loop.** Flash and WiFi share the SPI bus; under WiFi load, reads can stall up to 200 ms. This is the strongest "don't" in the entire research.
- **Approach A (servo angles, `int16`) ≈ 29 B/frame.** **Approach B (foot XYZ, `float`) ≈ 48–53 B/frame** — ~2× larger. **Approach B as parametric Bezier curves ≈ a few hundred bytes per gait** — far smaller than either. The right answer depends on what you mean by "Approach B".
- **Recommended encoding: hybrid.** Locomotion as parametric foot-curves (small, terrain-adaptable). Expressive animations as baked `int16` joint frames (compact, no IK risk).

---

### Consensus across the three sources

| Fact | Notes |
|------|-------|
| ESP32 has 520 KB SRAM total | Split across DRAM (~320 KB), IRAM (~192 KB) |
| WiFi stack consumes ~80 KB at runtime | Allocated from heap on init; reclaimable if WiFi deinit'd |
| Realistic free heap with WiFi + webserver = **200–280 KB** | Confirmed by multiple Arduino projects ([esp32.com](https://esp32.com/viewtopic.php?t=4545), [rntlab](https://rntlab.com/question/getting-lost-trying-to-do-the-esp32-web-server-w-arduino-tutorial/)) |
| Approach A frame: 1×u32 ts + 1×u8 interp + 12×i16 angles = **29 bytes** | Your math is right |
| Total gait library is "tens of KB" — fits comfortably | Even worst-case 80 frames × 29 B × 16 patterns ≈ 37 KB |
| Use **fixed-point `int16`** (0.1° units) over `float` for angle storage | OpenCat uses `int8_t`; `int16` gives you 0.1° resolution at the same byte cost as a float halved |
| Allocate runtime buffers statically — no `malloc` in the control loop | Standard MCU practice |

---

### Disagreement: can you load gaits from LittleFS on demand?

This is the most consequential disagreement in the whole research.

| Source | Verdict | Cited evidence |
|--------|---------|----------------|
| **Claude** | "Well under 1 ms per 4 KB block — fine to demand-load in the gait transition window" | Generic LittleFS read benchmarks |
| **Gemini** | **"DO NOT. Flash and WiFi share the SPI bus. Concurrent WiFi traffic causes unpredictable read latency spikes up to 200 ms. Servos will violently stutter."** | No direct citation, but the SPI bus contention is real |
| **Perplexity** | "Few ms per file — safe *if* you preload into a 'next gait' buffer and never block inside the IK loop." Cites measured 274 ms for 557 KB read ≈ 2 MB/s, sub-ms per 4 KB | [arduino-esp32 issue #1360](https://github.com/espressif/arduino-esp32/issues/1360), [esp32.com](https://esp32.com/viewtopic.php?t=8593) |

#### What's likely true

**Trust Gemini's warning, with Perplexity's nuance.** All three are partially right:

- In *isolation*, LittleFS reads of 0.6–4 KB files take <1 ms — Claude and Perplexity are correct on the steady-state number.
- *Under concurrent WiFi traffic*, the shared SPI bus to flash can be arbitrated for tens to hundreds of milliseconds. Gemini is right that this is unpredictable enough to break a 20 ms control loop.
- Perplexity's compromise is the clean engineering answer: **never read flash from inside the IK loop.** If you must demand-load, do it from a separate task on core 0, double-buffered, with the new clip swapped in only after the read completes.

**Default policy: preload everything at boot.** Your full library is <30 KB; there is no reason to incur this risk. Demand-loading only matters at scales (hundreds of KB of animation data) you don't have.

---

### Disagreement: Approach A vs Approach B

The three sources reach different verdicts because they're describing *different versions* of "Approach B".

| Variant | Per-frame size | Where it shines |
|---------|----------------|-----------------|
| **A — servo angles, `int16`** | 29 B | Smallest dense format. Zero-IK at runtime. Simple. |
| **B-dense — foot XYZ as `float`** | 48–53 B | Natural in Blender. Geometry-relative. ~2× A. |
| **B-fixed — foot XYZ as `int16` (0.1 mm)** | ~29 B | Same size as A, with the flexibility of B-dense. |
| **B-parametric — Bezier control points** | **a few hundred B per *whole gait*** | Tiny. Adapts to speed/terrain. What real projects use. |

#### What each source recommended

| Source | Verdict |
|--------|---------|
| **Claude** | **Hybrid.** A for locomotion gaits (fixed, frequent, small). B for expressive animations (geometry-relative, benefit from IK re-solve). |
| **Gemini** | **B is "vastly superior".** "Pre-computed angles cannot adapt to uneven terrain or roll/pitch body targets. Modern platforms use continuous parametric trajectories rather than discrete arrays of joint angles." |
| **Perplexity** | **Hybrid, leaning parametric.** Locomotion as parametric/Bezier curves (matching SpotMicroESP32-Leika's trot). Expressive animations as compact integer joint tables (matching OpenCat's skill library). |

#### What's likely true

**The hybrid is right. Gemini is right about *why* but wrong to dismiss A entirely.**

- For **locomotion** (walk/trot/bound): use **B-parametric**. Bezier control points + phase logic gives you adjustable step length, height, speed at runtime — which is exactly what terrain adaptation and body-pose IMU correction need. This is what [PingguSoft](https://github.com/PingguSoft/esp32_quadruped) and [Leika](https://github.com/runeharlyk/SpotMicroESP32-Leika) both do. Storage: a few hundred bytes per gait.
- For **expressive animations** (sit/wave/idle): use **A** (or **B-fixed** if you want geometry independence). These don't need terrain adaptation — they're choreographed. Bake to compact joint frames. This is what [OpenCat](https://github.com/PetoiCamp/OpenCatEsp32)'s skill library does.

Net: dense XYZ floats (the strawman version of B) are 2× larger than A and not worth doing. The interesting comparison is **A vs parametric**, and parametric wins for locomotion specifically.

---

### SRAM landscape (combining all three sources)

```
ESP32 520 KB total SRAM
├─ DRAM (data, accessible from C):       ~320 KB
│   ├─ WiFi stack (runtime):              ~80 KB
│   ├─ FreeRTOS kernel + task stacks:     ~20–40 KB (grows with tasks)
│   ├─ Webserver (e.g. ESPAsyncWebServer): ~20–40 KB heap
│   ├─ App heap (your gaits, IK state):   ~140–280 KB ← what you have to work with
│   └─ Static globals + .bss
└─ IRAM (instruction cache):              ~192 KB
    └─ Some usable for data via .iram1 section
```

**Practical free-heap measurements from real projects:**
- Empty Arduino sketch + WiFi: ~218 KB free heap ([esp32.com](https://esp32.com/viewtopic.php?t=4545)).
- Arduino WiFi-scan + HTTP server tutorial: ~290 KB free heap ([rntlab](https://rntlab.com/question/getting-lost-trying-to-do-the-esp32-web-server-w-arduino-tutorial/)).

---

### Storage math: how much do my gaits actually cost?

Your stated budget: 4–6 locomotion gaits + 5–10 expressive animations ≈ **16 patterns**, each 1–4 s at 10–20 fps.

#### Approach A (servo angles, 29 B/frame)

| Scenario | Frames per pattern | Total for 16 patterns |
|----------|--------------------|-----------------------|
| Min: 1 s @ 10 fps | 10 | 16 × 290 B = **4.6 KB** |
| Typical: 2 s @ 15 fps | 30 | 16 × 870 B = **13.9 KB** |
| Max: 4 s @ 20 fps | 80 | 16 × 2 320 B = **37 KB** |

#### Approach B-dense (foot XYZ floats, 53 B/frame)

| Scenario | Total for 16 patterns |
|----------|-----------------------|
| Min | 8.5 KB |
| Typical | 25.4 KB |
| Max | 67 KB |

#### Approach B-parametric (Bezier control points)

A few hundred bytes per gait → **~5 KB total for 16 patterns**, regardless of duration.

#### Conclusion

**Even the worst case (37 KB for A or 67 KB for B-dense) fits in your ~200 KB budget with room to spare.** You will never run out of RAM for gaits. You'll run out of RAM for something else (a large WebSocket buffer, a JSON parser allocating eagerly) long before gait storage matters.

This invalidates demand-loading as a memory-saving optimization. The only reason to read from flash at runtime is if you want hundreds of pre-recorded animations (think OpenCat-scale skill library).

---

### LittleFS specifics (read-only playback workload)

If you do load files from LittleFS at boot:

- **LittleFS over SPIFFS**: SPIFFS is no longer maintained. LittleFS has better wear-levelling and crash-resilience.
- **Read throughput**: ~2 MB/s sustained, sub-ms for files under 4 KB ([arduino-esp32 #1360](https://github.com/espressif/arduino-esp32/issues/1360)).
- **File `open` is the bottleneck**, not `read`. A few ms per open. Negligible at boot.
- **Write latency**: 20–350 ms for appends due to journaling. Doesn't apply to your read-only workload — you flash gait files once, read forever.

**Recommended LittleFS layout:**

```
/gaits/walk.bin
/gaits/trot.bin
/gaits/bound.bin
/gaits/turn_l.bin
/gaits/turn_r.bin
/gaits/crab.bin
/anim/sit.bin
/anim/stand.bin
/anim/wave.bin
/anim/idle.bin
/anim/look_around.bin
```

At boot: read every file in `/gaits/` and `/anim/`, parse into pre-allocated SRAM clip buffers, close files, never touch flash again during locomotion.

---

### Bottom line

| Question | Answer |
|----------|--------|
| Can all my gaits fit in RAM with WiFi active? | **Yes, easily.** Even worst-case 37 KB out of ~200 KB available. |
| Should I demand-load from LittleFS? | **No.** Preload at boot. The 200 ms SPI-bus contention risk under WiFi load isn't worth taking when the entire library fits in RAM. |
| Approach A or B? | **Hybrid.** Parametric Bezier foot-curves for locomotion, baked `int16` joint frames for expressive animations. The dense-XYZ-floats version of B is the worst of both worlds — skip it. |
| What size encoding for stored angles? | **`int16` in 0.1° units** — same byte cost as halved float, plenty of resolution for hobby servos. |
## Gait Patterns & Animation Scheduling

Synthesis of Claude, Gemini, and Perplexity answers to section 3 of the original prompt.

### TL;DR

- **Total library cost**: 16 patterns × ≤2 KB each ≈ **5–37 KB total**, well under your ~200 KB budget.
- **The right data structure is FSM + per-leg phase clocks.** Not a priority queue, not a circular buffer. All three sources independently land on the same answer.
- **Per-leg `phase_offset_ms` encodes every quadruped gait pattern** (walk, trot, bound, crab) trivially via `(phase + offset[leg]) % period`.
- **Smooth gait transitions = lerp current and next clip's joint angles over 100–200 ms** — don't try to re-solve IK to match.
- **Looping vs one-shot** is a single boolean on each clip; the scheduler returns the leg to `IDLE` when a one-shot's phase exceeds its period.

---

### Consensus across the three sources

This is the section with the *least* disagreement — the shape of a quadruped gait scheduler is well-established. All three converge on:

1. **A global motion FSM** (`IDLE`, `STANCE_POSE`, `GAIT_WALK`, `GAIT_TROT`, `ANIM_SIT`, ...). Transitions triggered by web/joystick commands. This is exactly what [SpotMicroESP32-Leika](https://github.com/runeharlyk/SpotMicroESP32-Leika) implements.
2. **Per-leg state** with its own phase variable in `[0, period)`. Each leg knows its current clip, optional next clip, and a blend factor.
3. **Per-leg phase offsets** as the single mechanism that encodes gait pattern. Trot → `[0, P/2, P/2, 0]`. Walk → `[0, P/4, P/2, 3P/4]`. Bound → `[0, 0, P/2, P/2]`.
4. **Blend in joint space** between two clips — not in task space, not by re-solving IK.

The disagreements are minor (struct field names, exact transition timing) and resolved easily.

---

### Total storage estimates for your library

Stated targets:
- 4–6 locomotion gaits: walk, trot, bound, turn L/R, crab.
- 5–10 expressive animations: sit, stand, look around, wave, idle breathing.
- Duration: 1–4 s. Keyframe rate: 10–20 fps.

#### Approach A (baked servo angles, 29 B/frame)

| Scenario | Per pattern | 16 patterns total |
|----------|-------------|-------------------|
| Light: 1–2 s @ 10–15 fps | 290–870 B | **~10 KB** |
| Heavy: 4 s @ 20 fps | 2.3 KB | **~37 KB** |

#### Approach B-dense (foot XYZ floats, 53 B/frame)

16 × heavy ≈ **67 KB** — fine but ~2× larger.

#### Approach B-parametric (Bezier control points per leg)

A handful of segment coefficients per leg per gait → a few hundred bytes total per gait → **~5 KB for 16 patterns** regardless of duration.

**All three fit comfortably.** Memory is not a constraint at this scale.

---

### Data structure: FSM + per-leg phase clocks

The problem decomposes cleanly because legs are independent timelines that share a global clock.

#### Recommended struct shape

```c
typedef enum { INTERP_LINEAR, INTERP_CUBIC, INTERP_STEP } InterpType;
typedef enum { CLIP_LOOP, CLIP_ONESHOT } ClipMode;
typedef enum { LEG_IDLE, LEG_PLAYING, LEG_TRANSITIONING } LegState;

typedef struct {
    uint16_t timestamp_ms;
    int16_t  angles[3];      // yaw, hip, knee in 0.1° units
    uint8_t  flags;          // bits 0-1: interp; bit 2: pole sign; bits 3-7: reserved
} Frame;

typedef struct {
    uint8_t   id;
    uint8_t   mode;          // ClipMode
    uint16_t  num_frames;
    uint16_t  period_ms;
    int16_t   phase_offset_ms[4];  // per-leg phase for trot/walk/bound
    Frame*    frames[4];     // per-leg keyframe arrays (or shared if symmetric)
} ClipDef;

typedef struct {
    LegState  state;
    uint8_t   current_clip;
    uint8_t   next_clip;     // valid only when state == TRANSITIONING
    float     phase_ms;
    float     blend_t;       // 0.0 → 1.0 during transition
    int16_t   current_angles[3];
    uint8_t   pole_sign;
} LegRuntime;
```

#### Update loop (50–100 Hz, on core 1)

```c
void ik_servo_tick(float dt_ms) {
    for (int leg = 0; leg < 4; leg++) {
        LegRuntime* lr = &leg_runtime[leg];
        ClipDef* clip = &clip_library[lr->current_clip];

        // 1. Advance phase (per-leg, with offset)
        lr->phase_ms = fmodf(lr->phase_ms + dt_ms, clip->period_ms);
        float effective_phase = fmodf(lr->phase_ms + clip->phase_offset_ms[leg],
                                       clip->period_ms);

        // 2. Sample keyframes (lerp between bracketing frames)
        int16_t target_angles[3];
        sample_clip(clip, leg, effective_phase, target_angles);

        // 3. If transitioning, blend with next clip's sample
        if (lr->state == LEG_TRANSITIONING) {
            int16_t next_angles[3];
            sample_clip(&clip_library[lr->next_clip], leg,
                        effective_phase, next_angles);
            for (int i = 0; i < 3; i++)
                target_angles[i] = lerp(target_angles[i], next_angles[i],
                                        lr->blend_t);
            lr->blend_t += dt_ms / TRANSITION_MS;  // e.g. 150 ms
            if (lr->blend_t >= 1.0f) {
                lr->current_clip = lr->next_clip;
                lr->state = LEG_PLAYING;
                lr->blend_t = 0.0f;
            }
        }

        // 4. One-shot completion
        if (clip->mode == CLIP_ONESHOT &&
            lr->phase_ms >= clip->period_ms - dt_ms) {
            lr->state = LEG_IDLE;
        }

        // 5. Write to servos
        memcpy(lr->current_angles, target_angles, sizeof(target_angles));
        servo_write(leg, target_angles);
    }
}
```

#### Why this beats alternatives

| Alternative | Why not |
|-------------|---------|
| Circular buffer (FIFO) of frames | Suits *streaming* (where frames arrive over the network). Your frames are all in RAM. Adds indirection without benefit. |
| Priority queue of events | Useful if you have many simultaneous events with different priorities. You only ever have one active clip per leg + at most one queued transition. Overkill. |
| One global timeline + frame indices | Loses the "legs run at different phases" property. You'd have to special-case every gait. |
| Per-leg threads / FreeRTOS tasks | Tasks are expensive. Four legs × overhead = wasted RAM for what's a single deterministic loop. |

The **state machine + per-leg phase accumulator** approach is what [OpenCatEsp32](https://github.com/PetoiCamp/OpenCatEsp32) and [PingguSoft/esp32_quadruped](https://github.com/PingguSoft/esp32_quadruped) use, and what [SpotMicroESP32-Leika](https://github.com/runeharlyk/SpotMicroESP32-Leika)'s "motion state controller" is built around. It is the de facto standard.

---

### Phase-offset encoding for gait patterns

This is the elegant part: every standard quadruped gait reduces to four numbers in `phase_offset_ms[4]`. With legs labelled FL, FR, BL, BR and period P:

| Gait | FL | FR | BL | BR | Description |
|------|----|----|----|----|-------------|
| **Trot** (diagonal) | 0 | P/2 | P/2 | 0 | Diagonal pairs in sync |
| **Walk** (4-beat) | 0 | P/2 | 3P/4 | P/4 | One leg lifted at a time |
| **Bound** | 0 | 0 | P/2 | P/2 | Front pair / back pair |
| **Pace** (lateral) | 0 | P/2 | 0 | P/2 | Same-side pairs |
| **Crab** (sideways) | 0 | 0 | P/2 | P/2 | Same as bound but with sideways foot path |

The clip data itself is *the same single-leg trajectory*; the offsets choose the gait. This is a classic locomotion result and matches how Leika's 8-phase crawl and Bezier trot are described.

#### Same-clip-shared-across-legs trick

If your gait is symmetric (most are), you only need *one* `Frame[]` array per gait — shared across all four legs. The runtime cost is one read with the `effective_phase` index. This drops storage by ~4×.

```c
typedef struct {
    // ...
    Frame* shared_frames;     // single timeline
    uint8_t per_leg_mirror;   // bit i: mirror Y axis for leg i (e.g. left vs right)
} ClipDef;
```

OpenCat does exactly this — leftward gaits stored, rightward mirrored at runtime.

---

### Looping vs one-shot

A single field on the clip (`mode: CLIP_LOOP | CLIP_ONESHOT`) plus the phase math handles both:

- **Looping** (locomotion): `phase_ms = fmodf(phase_ms + dt_ms, period_ms)` — naturally cycles forever.
- **One-shot** (sit, wave): when `phase_ms >= period_ms`, transition the leg back to `LEG_IDLE` (or to whatever's queued). Don't wrap.

For the typical use case "interrupt locomotion to play 'sit', then return to standing", the FSM is:

```
GAIT_WALK ──user_cmd("sit")─→ TRANSITIONING (blend to ANIM_SIT start pose, 150 ms)
                              │
                              ▼
                              ANIM_SIT (one-shot, 1.5 s)
                              │ phase >= period
                              ▼
                              POSE_SIT (static pose, looping with period_ms = ∞)
                              │
                              ▼ user_cmd("walk")
                              TRANSITIONING ──→ GAIT_WALK
```

Each transition is a 100–200 ms angle-space lerp. Smooth, no IK required during the blend.

---

### Smooth transitions: blend in joint space, not task space

All three sources agree (Claude explicitly, others implicitly):

- **Joint-space lerp**: `θ_out = (1 - b) · θ_current + b · θ_next` where `b ∈ [0, 1]`. Three multiplies and three adds per leg per tick. Trivial.
- **Task-space blend** (lerp foot positions, then re-solve IK): more "physically correct" but introduces IK singularity risks during the blend. Don't bother for clip-to-clip transitions.

**Phase-aligned transitions** (quietly the right move): start the blend at a phase where all legs are in stance, not mid-swing. Avoids visually jarring "leg moves backward through air to new trajectory" artefacts. This is what Leika does between gait modes.

For clip-to-clip blends in expressive animation, phase alignment doesn't matter; just blend whenever the user commands.

---

### Bottom line

| Question | Answer |
|----------|--------|
| What data structure? | **FSM (global) + per-leg phase clocks (per-leg state)**. Not queues, not threads. |
| How to encode gait patterns? | **`phase_offset_ms[4]` per clip**. One number per leg covers every quadruped gait. |
| How to handle looping vs one-shot? | **A `ClipMode` enum + `phase_ms >= period_ms` check**. Looping wraps; one-shot returns leg to `IDLE`. |
| How to transition smoothly? | **Joint-space lerp over 100–200 ms** between current and next clip's sampled angles. |
| Storage cost for 16 patterns? | **5–37 KB** depending on encoding — all fit easily in your ~200 KB heap. |
| Existing implementations to study? | [SpotMicroESP32-Leika](https://github.com/runeharlyk/SpotMicroESP32-Leika) (most modern), [PingguSoft/esp32_quadruped](https://github.com/PingguSoft/esp32_quadruped) (analytic gaits), [OpenCatEsp32](https://github.com/PetoiCamp/OpenCatEsp32) (skill library). |
## Pole Target & Knee Direction on the ESP32

Synthesis of Claude, Gemini, and Perplexity answers to section 4 of the original prompt.

### TL;DR

- **Encode the pole as `int8_t pole_sign ∈ {+1, -1}` per leg.** That's the entire choice between "elbow up" and "elbow down".
- **In practice, knee direction is *static configuration*, not animation data.** Quadrupeds keep all four knees bending the same way for their entire life. Don't waste a per-keyframe field on it.
- **Pack 4 legs into a single nibble** (4 bits in a `uint8_t`) if you ever do need per-keyframe pole data.
- **Avoid flips** by (a) staying away from the singularity (`|cos θ₂| < 0.9`) by design, and (b) only ever flipping during swing phase, never during stance.
- **The math**: `θ₂ = pole_sign · acosf((D² − L₁² − L₂²) / (2·L₁·L₂))`. One `acosf` per leg, multiply by ±1.

---

### Consensus across the three sources

| Fact | Notes |
|------|-------|
| 2-link planar IK has exactly two solutions: elbow up vs elbow down | Both reach the same foot position |
| The choice manifests as a `±` in front of the `acos` result | Law of cosines gives the magnitude; sign is your pole choice |
| One bit per leg suffices to encode the choice | Total: 4 bits across all 4 legs |
| Switching pole sign while the foot is planted is mechanically impossible | Without lifting the foot, you'd need to break the kinematic chain |
| Singularities at full extension and fully folded must be avoided | At `D = L₁ + L₂` or `D = |L₁ − L₂|`, both solutions converge and small numerical noise can flip them |
| Real quadrupeds keep knee direction constant | Leg geometry is mirrored side-to-side, but the *bend direction* is fixed |

---

### Disagreement: per-frame field or static configuration?

| Source | Verdict |
|--------|---------|
| **Claude** | "Encode it as a single `int8_t pole` field per leg in your frame struct." Per-keyframe in CSV, with hysteresis applied at runtime. |
| **Gemini** | Static per-leg `int8_t knee_dir` variable. Transitions only "during the aerial swing phase" — implies per-leg, not per-frame. |
| **Perplexity** | "**You don't usually want to switch knee direction per keyframe**. In practice this is a per-leg static configuration, not animation data." Concrete recommendation: 4-bit mask in a calibration struct. |

#### What's likely true

**Trust Perplexity.** Static configuration is the right answer for 99 % of robots. Reasoning:

- A real quadruped's knee direction is determined by the physical leg geometry and how the servos are mounted. It doesn't change unless the robot does a stunt that flips it (e.g. front-knee-forward → front-knee-backward to crawl under something).
- Storing pole_sign per keyframe means every animator has to remember to set it correctly — a footgun for no benefit.
- If you ever do want per-clip pole choice (e.g. a stunt animation), put it on the **clip** struct, not the **frame** struct.

**Recommended encoding**:

```c
// In the global robot calibration (set once, never changes for a given physical robot):
typedef struct {
    uint8_t knee_pole_mask;  // 4 bits, one per leg (FL, FR, BL, BR)
    int16_t link_lengths_mm[3];  // L1, L2, L3
    // ...
} RobotCalib;

// Per-clip *override* if you ever need a stunt:
typedef struct {
    // ...
    uint8_t pole_override_mask;  // bit 0: override active for this clip
                                 // bits 4-7: per-leg override values
} ClipDef;
```

For 99 % of clips, `pole_override_mask = 0` and the runtime uses `RobotCalib.knee_pole_mask`. No per-frame storage needed.

---

### The math

For the hip-knee 2-link planar chain with link lengths L₁ (thigh) and L₂ (calf), and a target at distance D from the hip:

```
D² = x² + y²                                  (foot distance from shoulder, in leg plane)
cos(θ₂) = (D² − L₁² − L₂²) / (2 · L₁ · L₂)
θ₂ = pole_sign · acosf(cos_θ2)                ← pole_sign ∈ {+1, -1}
θ₁ = atan2f(y, x) − atan2f(L₂·sinf(θ₂), L₁ + L₂·cosf(θ₂))
```

Per leg per tick: 1× `acosf`, 2× `atan2f`, 1× `sinf`, 1× `cosf`. Adds + multiplies otherwise. About what we costed in [[#On-Board IK Feasibility on the ESP32]].

#### Code sketch

```c
// Per-leg IK (called per tick, per leg)
void ik_solve_2link(float x, float y, int8_t pole_sign,
                    float L1, float L2,
                    float* out_theta1, float* out_theta2) {
    float D2 = x*x + y*y;
    float cos_t2 = (D2 - L1*L1 - L2*L2) / (2.0f * L1 * L2);

    // Clamp to valid acos domain — protects against numerical noise pushing
    // a near-singular pose just outside [-1, 1].
    cos_t2 = fmaxf(-1.0f, fminf(1.0f, cos_t2));

    float t2 = pole_sign * acosf(cos_t2);
    float t1 = atan2f(y, x) - atan2f(L2 * sinf(t2), L1 + L2 * cosf(t2));

    *out_theta1 = t1;
    *out_theta2 = t2;
}
```

The `clamp` on `cos_t2` is the only defensive code that earns its keep — without it, `acosf(1.0000001f)` returns NaN.

---

### Avoiding flips

A flip happens when the IK solution discontinuously jumps between `+pole_sign` and `-pole_sign`. The two scenarios:

#### Scenario 1: numerical noise near singularity

Near `D ≈ L₁ + L₂` (fully extended) or `D ≈ |L₁ − L₂|` (fully folded), `|cos θ₂| → 1` and the two solutions merge. Tiny floating-point noise on `cos_t2` can flip you across the boundary.

**Mitigation: design your foot trajectories so D stays comfortably away from the singularities.**

- Set `D_min = |L₁ − L₂| + margin`, `D_max = L₁ + L₂ − margin` (e.g. 5–10 mm margin).
- Clip foot targets to this range before passing to IK. Better to have the foot fall short of an unreachable target than have a snap.
- Equivalently: keep `|cos θ₂| < 0.95` or so by design.

#### Scenario 2: deliberately changing pole_sign mid-motion

If you ever *do* change `pole_sign` (e.g. switching from a stunt-pose animation back to a normal gait), the leg has to physically lift off the ground first.

**Mitigation: only allow pole_sign changes during swing phase**, never during stance. Two ways to enforce this:

- **State-based**: track an `is_swing` flag per leg (derivable from your gait phase). Block `pole_sign` writes during stance.
- **Trajectory-based**: schedule the pole flip at the moment the leg reaches its swing apex, where it's furthest from the ground and the leg is near full extension — both knee solutions are nearly identical, so the angle change is small.

This is what Gemini calls "mathematically interpolating the multiplier from 1 to −1 as the leg passes through a fully extended state". It's smooth precisely because the two solutions converge at full extension.

#### Hysteresis as a backstop

If you store pole signs in the file (against the static-only recommendation above), apply hysteresis at runtime: only honor a stored pole change when `|cos θ₂| < 0.9`. Inside the dead band, hold the previous sign. This prevents jitter at the boundary.

---

### Bottom line

| Question | Answer |
|----------|--------|
| How to encode pole choice? | **`int8_t` per leg**, packed as a 4-bit nibble in robot calibration. Static, not per-frame. |
| Per-keyframe field? | **No.** Real quadrupeds don't change knee direction during animation. Use a per-clip override only if you have a specific stunt that needs it. |
| How to compute? | `θ₂ = pole_sign · acosf((D² − L₁² − L₂²) / (2·L₁·L₂))`. Clamp `cos_t2` to `[-1, 1]` defensively. |
| How to avoid flips? | (1) Design trajectories to keep `|cos θ₂| < 0.95`. (2) If `pole_sign` ever changes, schedule it during swing phase, near full extension where the two solutions converge. |
| Singularity threshold? | Stay at least 5–10 mm away from `D = L₁ + L₂` and `D = |L₁ − L₂|`. |
## Recommended Architecture

Synthesis of Claude, Gemini, and Perplexity answers to section 5 of the original prompt, plus cross-cutting verdicts from the other four topic docs:

- [[#On-Board IK Feasibility on the ESP32]]
- [[#Memory Constraints in Practice]]
- [[#Gait Patterns & Animation Scheduling]]
- [[#Pole Target & Knee Direction on the ESP32]]

---

### TL;DR

**Build a hybrid pipeline:**

1. **Blender authors animations** (locomotion as foot-curve trajectories in robot frame; expressive animations as joint-angle keyframes).
2. **A Python exporter (run with `uv`)** samples each clip and writes a compact binary blob.
3. **The blob lives on the ESP32's LittleFS partition.** At boot, the ESP32 inflates *every* clip into pre-allocated SRAM buffers. Total RAM cost: <30 KB.
4. **At runtime**: core 0 runs WiFi + AsyncWebServer. Core 1 runs a fixed 50–100 Hz FreeRTOS loop: sample the active clip, blend with any next clip, run on-board IK, write to PCA9685 PWM.
5. **Never touch flash from inside the control loop.** WiFi and flash share the SPI bus; concurrent reads can stall up to 200 ms.

The closest reference implementation is [runeharlyk/SpotMicroESP32-Leika](https://github.com/runeharlyk/SpotMicroESP32-Leika). Read it before writing your own.

---

### Where the three sources agree

This is the foundation everyone converges on:

| Decision | Consensus |
|----------|-----------|
| Dual-core split | WiFi/webserver on core 0, motion control on core 1, communicating via FreeRTOS queue |
| IK location | On-board (closed-form, single-precision) — the CPU cost is trivial |
| Update rate | 50 Hz minimum, 100 Hz typical, more is wasted on hobby-servo bandwidth |
| Servo driver | PCA9685 over I2C @ 400 kHz, or LEDC peripheral directly |
| Gait scheduler | FSM + per-leg phase clocks (not threads, not priority queues) |
| Phase encoding | `phase_offset_ms[4]` per clip handles every standard gait |
| Transitions | Joint-space lerp over 100–200 ms |
| Storage scale | ~5–37 KB total for a full library — preload everything at boot |
| Reference projects | [PingguSoft/esp32_quadruped](https://github.com/PingguSoft/esp32_quadruped), [SpotMicroESP32-Leika](https://github.com/runeharlyk/SpotMicroESP32-Leika), [PetoiCamp/OpenCatEsp32](https://github.com/PetoiCamp/OpenCatEsp32) |

---

### Where they disagree (and what to actually do)

These are the load-bearing conflicts the synthesis must not smooth over. The columns show each source's verdict; the final column is the recommendation grounded in the cited evidence.

| Issue | Claude | Gemini | Perplexity | **Recommendation** |
|-------|--------|--------|------------|--------------------|
| `atan2f` timing on ESP32 | 16–25 µs | ~1.0 µs | 8.6–20 µs (cited) | **Use 10–25 µs.** Espressif's own measurement (~2060 cycles) is the authoritative number. Gemini's 1 µs is implausible — there's no hardware `atan2`. Conclusion is the same either way: IK is <5 % of one core. |
| LittleFS demand-load during control loop | "fine" | "**DO NOT — SPI bus shared with WiFi, 200 ms stutter**" | "few ms IF preloaded into next-gait buffer; never block in IK loop" | **Preload everything at boot.** Gemini's warning is real and the library is small enough that demand-loading buys nothing. |
| Approach A vs B | Hybrid (A for gaits, B for animations) | "B vastly superior" (parametric) | Hybrid (parametric B for locomotion, A for skills) | **Hybrid, parametric for locomotion.** B-dense (foot XYZ floats) is 2× larger than A and not worth doing. B-parametric (Bezier control points) is tiny and adapts to terrain — that's the right shape for locomotion. A-style baked frames are right for choreographed expressive animations. |
| Pole sign storage | Per-keyframe field | Static per-leg | Static per-leg, optional per-clip override | **Static.** Quadruped knee direction doesn't change during normal animation. Put it in robot calibration, not in clip data. |
| Closest reference project | OpenCatEsp32 + PingguSoft | SpotMicroESP32-Leika | All three | **Leika first** for architecture, PingguSoft for analytic IK, OpenCat for skill library patterns. |

---

### The architecture

```
┌─────────────────────────────────────────────────────────────────┐
│ AUTHORING (offline, on your M3 Mac)                             │
│                                                                 │
│  ┌──────────────┐    Python exporter (uv-managed)              │
│  │   Blender    │──→ • Sample foot-curve gaits → Bezier        │
│  │   .blend     │    • Sample expressive anims → joint frames  │
│  │              │    • Pack to compact binary                   │
│  └──────────────┘    • Validate against link-length limits     │
│                                                                 │
│         ↓ Output: gaits.bin, anims.bin (or per-clip files)     │
└─────────────────────────────────────────────────────────────────┘
                              │ USB-serial flash, or WiFi OTA
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ ESP32 — LittleFS partition                                      │
│  /gaits/walk.bin  /gaits/trot.bin  /gaits/bound.bin             │
│  /anim/sit.bin    /anim/wave.bin   /anim/idle.bin               │
└──────────────┬──────────────────────────────────────────────────┘
               │ Loaded into SRAM at boot, then never touched again
               ▼
┌────────────────────────────────┐  ┌──────────────────────────┐
│ CORE 0 (PRO_CPU)               │  │ CORE 1 (APP_CPU)         │
│  • WiFi stack                  │  │  • Gait scheduler (FSM)  │
│  • AsyncWebServer (HTTP/WS)    │  │  • Per-leg phase clocks  │
│  • WebSocket gait commands     │  │  • Closed-form IK        │
│  • OTA update handler          │◄─┤  • Pole-sign clamping    │
│  • Telemetry pushback          │  │  • PCA9685 / LEDC PWM    │
│                                │  │  • IRAM_ATTR hot loop    │
│       (no servos, no IK)       │  │  • 50–100 Hz fixed tick  │
└────────────────────────────────┘  └──────────────────────────┘
                  ↕
            FreeRTOS queue
       (gait switch commands,
        body pose offsets,
        telemetry samples)
```

---

### Binary format on flash

Combining Claude's `GaitFrame` and Perplexity's `ClipDescriptor` shapes:

```c
// File container ─ one file per clip, or a single packed bundle
struct ClipFileHeader {
    uint8_t  magic[4];        // 'F','H','C','1'  (FaceHugger Clip v1)
    uint8_t  type;            // 0=locomotion (Bezier), 1=animation (frames)
    uint8_t  mode;            // 0=loop, 1=oneshot
    uint16_t period_ms;
    uint16_t num_frames;      // for type=1
    int16_t  phase_offset_ms[4];  // per-leg gait offset
    uint16_t reserved;
};
// = 16 bytes header

// For type=1 (baked joint frames):
struct AnimFrame {
    uint16_t timestamp_ms;
    int16_t  angles[12];      // 0.1° units, big-endian or native — pick one
    uint8_t  flags;           // bits 0-1: interp type
                              // bits 2-3: reserved (don't use for pole — static config)
} __attribute__((packed));
// = 27 bytes per frame

// Total per 2-sec × 20 fps animation: 16 + 40×27 = 1 096 bytes
// Full library of 16 patterns: ~17 KB on flash, ~17 KB in RAM
```

```c
// For type=0 (locomotion Bezier):
struct LocoCurve {
    int16_t step_length_mm;
    int16_t step_height_mm;
    int16_t bezier_control_pts[4][3];   // 4 control points × xyz, fixed-point
    int16_t stance_height_mm;
    uint16_t swing_duration_ms;         // remainder is stance
} __attribute__((packed));
// = ~36 bytes per locomotion gait — tiny
```

---

### Build/deploy pipeline

The Python exporter, per the user's `uv`-default convention:

```bash
# In doc/animation-pipeline/exporter/
uv add bpy numpy
uv run python export_gaits.py \
    --blend ../../assets/quadruped.blend \
    --out ../../firmware/data/gaits/ \
    --fps 20 \
    --link-lengths 60,70,75    # mm, validated against IK reach limits
```

The script's responsibilities:

1. Iterate Blender `Action`s tagged with `gait_*` (locomotion) or `anim_*` (expressive).
2. For locomotion: extract foot empties, fit Bezier control points to the foot path, write `LocoCurve` records.
3. For expressive: sample at 20 fps, walk the IK rig, extract servo angles, write `AnimFrame` records.
4. Validate every frame against `D_min` / `D_max` reach (see [[#Pole Target & Knee Direction on the ESP32]]). Reject the build if any frame would force the IK near a singularity.
5. Emit a manifest JSON for the firmware to know what's in `/gaits/` and `/anim/`.

The firmware-side loader at boot:

```c
void load_clip_library(void) {
    DIR* d = opendir("/gaits");
    struct dirent* e;
    while ((e = readdir(d))) {
        if (clip_count >= MAX_CLIPS) break;
        load_clip_into_sram(e->d_name, &clip_library[clip_count++]);
    }
    closedir(d);
    // Same for /anim/
}
```

This runs once at boot. After this, the LittleFS handle is closed and never re-opened during locomotion.

---

### Critical design tradeoffs (decided)

| Choice | Decided as | Reason |
|--------|------------|--------|
| Storage encoding | Hybrid: parametric Bezier (locomotion) + baked `int16` joint frames (expressive) | Smallest total footprint; locomotion stays terrain-adaptable; animations stay choreographed |
| IK location | On-board, single-precision | <5 % of one core; required anyway for body-pose / IMU correction; baking everything off-board would lock you out of reactive control |
| Gait loading | Eager preload at boot | Library is <30 KB; eliminates flash/WiFi SPI contention risk |
| Transition blending | Joint-space lerp, 100–200 ms | Simple, no IK singularity risk during the blend |
| Pole / knee direction | Static in robot calibration | Quadrupeds don't flip knees during normal motion; per-frame storage is a footgun |
| Update rate | 100 Hz on core 1 (with provision for 50–200 Hz) | Matches PingguSoft / Leika; sufficient for hobby servos; leaves headroom for IMU correction |
| Servo driver | PCA9685 over I2C | Industry default; isolates PWM jitter from WiFi interrupts; matches your stated hardware |
| File system | LittleFS (not SPIFFS) | SPIFFS is unmaintained; LittleFS is a strict upgrade |
| Filesystem access | Once at boot, then closed | Avoids 200 ms SPI-bus contention spikes under WiFi load |

---

### What to verify early (before the architecture solidifies)

These are the unknowns that *could* invalidate the plan if reality differs:

1. **Servo PWM bandwidth in your specific setup.** Hobby-servo mechanical bandwidth is ~100 Hz. If you're running cheap MG90S, even 50 Hz commanded updates may be partially absorbed. Run a step-response test on one servo before tuning the loop rate up.
2. **PCA9685 I2C latency.** 16 channels in ~200 µs at 400 kHz is the spec. Confirm with a logic analyzer that you're hitting it under WiFi load — Wi-Fi DMA on core 0 *can* stall I2C ISRs on core 1 in pathological cases.
3. **WiFi/flash SPI contention magnitude.** If you ever do need a runtime flash read (telemetry log, OTA chunking), measure worst-case latency under a WebSocket flood to see if Gemini's 200 ms warning materializes on your hardware.
4. **Free heap with everything loaded.** After WiFi + AsyncWebServer + 30 KB of clips + IK state, run `esp_get_free_heap_size()` and confirm you're above 100 KB. Below that, dynamic allocation in the WebSocket layer starts failing.
5. **`IRAM_ATTR` placement.** Profile the IK hot loop and put any function called >1× per tick into IRAM. Not strictly required, but eliminates cache-miss jitter on core 1.

---

### Reference projects, ranked by usefulness for your build

#### 1. [runeharlyk/SpotMicroESP32-Leika](https://github.com/runeharlyk/SpotMicroESP32-Leika) — **start here**
- ESP32-CAM with full body-frame kinematics + per-foot IK on-board.
- 8-phase crawl + Bezier-curve trot, both parametric — exactly the locomotion model recommended above.
- Serves a Svelte web UI from the ESP32 itself. Direct proof of "WiFi + IK + gait planner on one chip".
- FSM-based motion controller with phase-aligned transitions.
- Closest architectural match to your goals.

#### 2. [PingguSoft/esp32_quadruped](https://github.com/PingguSoft/esp32_quadruped) — **best baseline for IK + analytic gaits**
- 12-DOF closed-form leg IK on ESP32, very similar to your 1-yaw + 2-pitch scheme.
- 100 Hz update rate, PCA9685 driver, IMU-based balancing.
- Same dual-core split as recommended.
- Caveat: gaits are designed in an external Processing simulator and use analytic parameters rather than keyframed clips. Less expressive but smaller footprint.

#### 3. [PetoiCamp/OpenCatEsp32](https://github.com/PetoiCamp/OpenCatEsp32) — **best for the skill-library pattern**
- Compact `int8_t` joint-frame skill encoding; many small expressive behaviors.
- Mirroring trick (left-only stored, right mirrored at runtime) is worth borrowing.
- No on-board IK — pure pre-baked playback. Adopt the storage shape, not the compute model.

#### Honorable mentions
- [Denis Bujoreanu (Hackster)](https://www.hackster.io/denis-bujoreanu/quadruped-robot-dog-with-12dof-and-ik-07d047) — direct hardware analogue (ESP32-WROOM + PCA9685 + 12 MG90S + MPU6050).
- [Michael Seyoum (Hackster)](https://www.hackster.io/mikroller/robot-leg-inverse-kinematics-and-generic-gait-with-esp32-e78d1e) — minimal IK + generic gait demo.
- [ViolinLee/NodeQuad12-MicroPython](https://github.com/ViolinLee/NodeQuad12-MicroPython) — even MicroPython does this; reassuring lower bound.

---

### Bottom line

Your initial intuition was correct on every count: 12-DOF closed-form IK on ESP32 is trivial, dual-core WiFi-coexistence is well-trodden, and the entire gait library fits in RAM with margin. The non-obvious findings from synthesizing the three research answers:

1. **Gemini's flash/WiFi SPI-bus warning is the single most important "don't"** — even though the steady-state read latency looks fine, the worst-case under WiFi load can break a 20 ms control loop. **Preload at boot, never read flash from the IK loop.**
2. **The Approach A vs B debate is wrongly framed.** The answer isn't "A or B" — it's "parametric Bezier for locomotion (terrain-adaptable, tiny) + baked joint frames for expressive animation (compact, choreographed)". This is what shipping projects do.
3. **Pole / knee direction is static configuration, not animation data.** Don't waste a per-frame field on a property your robot will never change.
4. **The actual bottleneck is servo PWM bandwidth, not IK compute.** Don't over-engineer the IK loop until benchmarks tell you to.

Build [SpotMicroESP32-Leika](https://github.com/runeharlyk/SpotMicroESP32-Leika)'s architecture, learn from [PingguSoft](https://github.com/PingguSoft/esp32_quadruped)'s closed-form IK, borrow [OpenCat](https://github.com/PetoiCamp/OpenCatEsp32)'s skill-library encoding, and you'll have a robust 12-DOF quadruped platform.
