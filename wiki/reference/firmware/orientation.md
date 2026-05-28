# Orientation and auto-flip

Why does the robot need to detect upside-down at all? Because the spinal cord's pitch mirror is what makes inverted operation *feel* like upright operation. `applyInvert` does the math — `2 * CALIB - angle` on each pitch joint, shoulder untouched — but it only fires when `isInverted` is set. Something has to tell the firmware which mode it is in. On a quadruped that the user can pick up, drop, or flip in mid-clip, that "something" has to be the IMU.

This page walks through how a single MPU6050 accel sample becomes a steady, latched `isInverted` boolean — and how that boolean reaches the same `applyInvert` call on hardware, in the SIL, and across reconnects.

## The MPU6050 sample loop

`code/firmware/src/brain/sensors.cpp` owns the IMU. `initSensors()` brings up the MPU6050 on the shared I2C bus (`0x68`, no clash with the PCA9685 at `0x40`, OLED at `0x3C`, or VL53L0X at `0x29`), and every `loop()` tick calls `tickImu()`. The driver is Adafruit's `MPU6050` library v2.2.4; we use only the accelerometer (±2 g, 21 Hz bandwidth), normalise it to a unit gravity vector, and compute three quantities:

- `s_tilt_deg` — angle between the measured gravity vector and a hardcoded `UPRIGHT_REF` (the gravity vector the chip should see when the chassis is upright; `(0, 0, -1)` for a Z-up mount).
- `s_pitch_deg` — `atan2(ay, sqrt(ax² + az²)) * 180 / π`, nose-up positive.
- `s_roll_deg` — `atan2(-ax, az) * 180 / π`, right-side-down positive.

These are exposed via `imuTiltDeg()`, `imuPitchDeg()`, `imuRollDeg()`, and the latched `imuIsInverted()`. The MPU is optional: if `begin()` fails on the bus, `imuReady()` stays false and every accessor returns its last value (zero on boot). The rest of the firmware degrades gracefully — there is no MPU on the bench-side test rig.

## 150°/30° hysteresis and the dead zone

`imuIsInverted()` is not a raw threshold on tilt. It runs the live tilt angle through `imuInvertedHysteresis()` (`code/firmware/src/brain/imu_hysteresis.h`):

```cpp
constexpr float IMU_FLIP_UP_DEG   = 150.0f;
constexpr float IMU_FLIP_DOWN_DEG =  30.0f;
```

When the latch is `false` (upright), it only flips to `true` once tilt exceeds 150°. When the latch is `true` (inverted), it only clears once tilt drops below 30°. Everything between 30° and 150° is a *dead zone* where state stays latched, no matter how the robot wobbles. That band is deliberate: at ~90° tilt the gravity vector lies in the body's horizontal plane, the accel-only formula is degenerate, and a single noisy sample can swing tilt by tens of degrees. Without hysteresis, the boolean would chatter every few milliseconds while the robot was being picked up or rolled, and `applyInvert` would mirror the pose back and forth into oscillation. With it, the flip is decisive: the robot has to actually be nearly upside-down (or nearly upright) to change modes. The strict inequalities matter — landing exactly on 30° or 150° leaves the state untouched.

## Boot orientation classifier

The hysteresis solves the steady-state case but not the cold-boot case. If a user powers the robot on while it is sitting upside-down, the firmware's first servo writes would still snap to the upright `NEUTRAL[]` — the legs would flail until the latch caught up. `boot_orientation.h` fixes that.

After `Wire.begin()` and a 50 ms settle, `initSensors()` takes *one* accel sample, normalises it, and compares it to `UPRIGHT_REF` via dot product:

- dot > +0.7 (≈ cos 45°) → `BOOT_UPRIGHT` — leave `isInverted` false.
- dot < −0.7 → `BOOT_INVERTED` — call `sc.setInverted(true)` *before* the first servo write.
- in between → default upright (the safe fallback; `tickImu()` will catch a real inversion on the next loop).

This replaced the older "average 50 samples = upright reference" scheme, which silently assumed the robot was upright at power-on and broke any cold-boot-inverted workflow.

## The auto-flip gate: `tickAutoInvert`

The IMU now tells us *when* the robot is upside-down. Something still has to translate that into a `setInverted()` call. That used to live inline in `main.cpp::loop()`. It now lives on `SpinalCord`:

```cpp
void SpinalCord::tickAutoInvert(bool imuInverted) {
    if (imuInverted == prevImuInverted_) return;   // no edge
    if (!autoInvertEnabled_) return;               // user froze the gate
    setInverted(imuInverted);
    prevImuInverted_ = imuInverted;
}
```

Three properties matter. **It is edge-triggered**: the instance member `prevImuInverted_` latches the last forwarded value, so `setInverted()` fires once per flip, not every tick. **It respects the user toggle**: when `autoInvertEnabled_` is false the call is skipped entirely *and* `prevImuInverted_` is intentionally not advanced, so re-enabling the gate evaluates against the last-forwarded value (the IMU may have moved while the user was frozen, and we want the very next change to fire). **It is called from both runtimes**:

- `main.cpp::loop()` — hardware: `spinalCord.tickAutoInvert(imuIsInverted());`
- `bindings.cpp::FirmwareControl::tick()` — SIL: same call, between `clock_ms` advance and `spinalCord.update()`.

That second call is the critical fact about the new architecture. The PyBullet SIL bypasses `main.cpp::loop()` entirely (the WebSocket server is Python-side; the C++ entry point is `FirmwareControl::tick`). Before this method existed, the IMU latch was set correctly from PyBullet but no one consumed it, so the simulated robot stayed visually upright in inverted mode. Funnelling both runtimes through the same `tickAutoInvert` makes hardware and SIL behave identically by construction. Add a third runtime tomorrow (a unit test, a CI harness) and you call `tickAutoInvert` from it too.

`setInverted` itself does the visible work: flip the flag, switch the face sprite (`EYES_CONFUSED` while inverted), and call `flipPoseInPlace(INVERT_EASE_MS)` so the robot mirrors whatever pose it is currently holding instead of snapping to the inverted neutral. See [motion-engine.md](motion-engine.md) for `applyServos` and the pitch-mirror choke point.

## T:6 — freezing the gate

A user might want auto-flip *off* — for instance, calibrating the robot on its back without the firmware constantly trying to mirror the pose. T:6 (`CMD_SET_AUTO_INVERT`) does exactly that:

```json
{"T": 6, "enabled": false}
```

The handler in `network.cpp` calls `SpinalCord::setAutoInvertEnabled(bool)`. Setting `enabled=false` does *not* clear `isInverted`; it freezes whatever value the flag had. The robot keeps mirroring or not mirroring exactly as it was, but new IMU edges are ignored until either the user re-enables the gate (next IMU edge re-evaluates) or sends an explicit T:9 (`CMD_SET_INVERT`) to override the flag directly. Missing or non-bool `enabled` is a no-op, which lets a client probe the firmware's current state without changing it.

T:6 used to be `CMD_ACTION_SELECTION` (the manual "invert robot" button). That button disappeared from the app once auto-flip landed; the type code was repurposed rather than retired so the wire shape stayed contiguous.

## T:10 — telemetry

The robot broadcasts `T:10` every 100 ms to every connected client (`TELEMETRY_INTERVAL_MS`). Four IMU-related fields piggyback on the existing payload rather than a separate stream:

- `pitch_deg` — `imuPitchDeg()`, signed.
- `roll_deg` — `imuRollDeg()`, signed.
- `upside_down` — `imuIsInverted()`, the latched boolean.
- `auto_invert_enabled` — `SpinalCord::isAutoInvertEnabled()`, so the app's switch state survives reconnects.

The SIL produces the same shape from `ws_sim.py::telemetry()`, reading through the bindings (`fc.imu_pitch_deg()`, `imu_roll_deg()`, `imu_is_inverted()`, `is_auto_invert_enabled()`). Hardware and sim emit byte-identical IMU fields.

## App: orientation tile and auto-flip toggle

The header `RobotStatus.tsx` tile displays the live orientation as `Pitch: +12.3° | Roll: -5.7° | Upright`. The formatters live in `code/remote-control-app/MyApp/api/orientation.ts` and render `—` when the field is null — important for older firmware that predates the IMU fields, so the tile stays graceful instead of crashing on `undefined`.

The auto-flip toggle is a single row on the Actions screen (`screens/Actions.tsx`). Flipping it calls `sendSetAutoInvert(boolean)`, which sends the T:6 above. The on-wire state comes back via T:10's `auto_invert_enabled`, so two phones connected at once stay in sync.

## Cross-references

- [Invert mirror and CALIB](../conventions.md#invert-mirror-and-calib) — the `2 * CALIB - angle` formula `applyInvert` uses, and why it matters that the mirror runs about per-joint flat instead of constant 90.
- [Calibration guide](../../guide/calibration.md) — the workflow that measures the `CALIB_*` values the mirror is anchored to.
- [Motion engine](motion-engine.md) — the `applyServos` / `applyInvert` choke point every motion source funnels through; `setInverted` triggers `flipPoseInPlace`, and subsequent ticks naturally produce the mirrored pose because they all route through `applyServos`.
- [FSM states](fsm-states.md) — auto-flip is *not* a state. The IMU edge can fire in IDLE, WALK, ACTION, or STAND; `isInverted` is orthogonal to `robotState`.
- [WebSocket API](../remote-control/websocket-api.md) — T:6 and T:10 IMU fields on the wire.
- [PyBullet IMU emulation](../simulation/pybullet-control.md#imu-emulation-in-pybullet) — how PyBullet's body orientation drives the same hysteresis on the SIL side.
