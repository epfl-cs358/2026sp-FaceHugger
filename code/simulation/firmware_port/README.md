# pybullet_interpreter

Python module that mirrors the FaceHugger firmware clip pipeline, letting you
validate animation clips in PyBullet before flashing.

## Purpose

The firmware plays clips via `tickClip()` in `spinal_cord.cpp`, which calls
`translateToServo()` and `clampClipServos()` from `motion_math.cpp`. This module
ports that exact math to Python so the same clip data can be driven through a
PyBullet simulation and inspected visually.

EMA smoothing (`CLIP_EMA_ALPHA=0.75` in firmware) is intentionally **not**
reproduced here — the goal is validating clip geometry, not smoothing artifacts.

## Files

| File | Role |
|------|------|
| `servo_convention.py` | Firmware math ported to Python: `translate_to_servo`, `clamp_clip_servos`, `servo_to_radians`, `NEUTRAL` constants |
| `clip_loader.py` | Parses `animation/exported_clips/clips_all.h` into `ClipData` / `ClipFrame` dataclasses |
| `clip_player.py` | Interpolates clip frames and drives PyBullet joints; `ClipPlayer.play_blocking()` runs in real time |
| `gait_interpreter.py` | Stub — not yet implemented |

## Usage

Play a clip from the command line:

```bash
# from code/simulation/
python facehugger.py sim --clip "tiny wiggle"
python facehugger.py sim --clip "tiny wiggle" --headless   # CI / no window
```

Or drive it from Python:

```python
from pybullet_interpreter.clip_loader import DEFAULT_CLIPS_H, get_clip_by_name, load_clips_all_h
from pybullet_interpreter.clip_player import ClipPlayer

clips = load_clips_all_h(DEFAULT_CLIPS_H)
clip  = get_clip_by_name(clips, "tiny wiggle")

player = ClipPlayer(robot_id, joint_map, clip)
player.play_blocking(gui=True)
```

`robot_id` and `joint_map` come from the existing PyBullet setup in `gaits.py`
(`_connect_and_setup` + `build_joint_map`).

## Pipeline

```
clips_all.h  →  load_clips_all_h()  →  ClipData.frames (a[12], ms)
                                              │
                                    _interpolate_frame(elapsed_ms)
                                              │
                                    frame_to_joint_targets(a)
                                      ├── translate_to_servo(leg_id, sh, th, kn)
                                      ├── clamp_clip_servos(servo)
                                      └── servo_to_radians(deg)
                                              │
                                    pybullet.setJointMotorControl2(...)
```

## Tests

Pure-math tests (no PyBullet required):

```bash
# from code/simulation/
uv run --with pytest python -m pytest pybullet_interpreter/tests/ -v
```

53 tests across `servo_convention`, `clip_loader`, and `clip_player`.

## Firmware correspondence

| Python | Firmware |
|--------|----------|
| `translate_to_servo()` | `translateToServo()` — `motion_math.cpp:32` |
| `clamp_clip_servos()` | `clampClipServos()` — `motion_math.cpp:10` |
| `_interpolate_frame()` | `clipPoseAt()` — `motion_math.cpp:62` |
| `ClipPlayer.step()` | inner loop of `tickClip()` — `spinal_cord.cpp:427` |
| `NEUTRAL[]` | `NEUTRAL[LEG_COUNT]` — `neutral_pose.h:13` |
