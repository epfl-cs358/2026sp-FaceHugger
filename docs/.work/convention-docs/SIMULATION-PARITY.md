# Simulation parity — how the PyBullet sim mirrors the firmware

The PyBullet interpreter (`code/simulation/pybullet_interpreter/`) exists to
validate animation clips on a laptop before flashing. For that to be meaningful
it must interpret the Blender export **exactly** as the firmware does. This page
explains the shared contract and the verification chain that proves it.

## The shared contract

Both the firmware and the sim consume the same bundled `clips_all.h`
(pre-scaled, math-space joint degrees) and apply the **same** transform:

```
clips_all.h frame a[12]  --translateToServo (per-leg)-->  servo degrees
                         --clampClipServos-->            --> PCA (firmware)
                                                          --> joint radians (sim)
```

There are three independent implementations of `translateToServo`:

- firmware C++ — `code/firmware/src/nervous_system/motion_math.cpp`
- exporter Python — `_frame_to_servo` in `animation/addons/fh_clip_panel.py`
  (writes the browser `.js`)
- sim Python — `translate_to_servo` in `code/simulation/.../servo_convention.py`

They must stay byte-identical. The convention itself: math-space `+sh` = CCW yaw
(uniform, all legs); pitch (thigh/knee) mirrors on the {FL,BR}↔{FR,BL} diagonal;
every shoulder is `+1` (BR un-mirrored — see DRAFT-delta-conventions.md §6);
`servo 90` = outward/flat for all legs.

## The three export formats, and what each is for

| file | contents | consumer |
|------|----------|----------|
| `clips_all.h` | math-space, all clips bundled | firmware (compiled in); sim (loaded) |
| `<clip>.h` | per-clip, link1-corrected bone angles | legacy/debug |
| `<clip>.js` | servo degrees ([0,180] clamp) | browser live-stream |
| `<clip>.csv` | raw bone angles | debug (only when CSV export is ticked) |

`clips_all.h` math-space = scale-from-NEUTRAL of the bone angles; the `.js`
servos = `translateToServo(math-space)` with a `[0,180]` clamp. The firmware/sim
clip path applies the tighter `clampClipServos` — that is an intentional
extra-safety layer, **not** a convention difference.

## The verification chain (what proves parity)

| check | what it locks | how to run |
|-------|---------------|------------|
| `test_servo_convention.py` | sim `translate_to_servo` == firmware formula | `pytest` (sim tests) |
| `test_servo_parity.py` | exporter `_frame_to_servo` == firmware formula | `pytest` (animation/scripts) |
| `test_clip_parity` (C++) | firmware `translateToServo` == exporter (`clip_parity_reference.h`) | `pio test -e native` |
| `check_export_consistency.py` | per-clip `.h` round-trips to `.js`; convention at NEUTRAL/flat | `python check_export_consistency.py` |
| `verify_export_parity.py` | **sim's reading of `clips_all.h` == the `.js`**, all clips/frames | `python verify_export_parity.py` |

The last two run on the *actual exported clips*. Because the sim's
`translate_to_servo` and the exporter's `_frame_to_servo` are **separate
implementations**, their agreement on real data (`verify_export_parity`, 6/6),
plus `test_clip_parity` tying the firmware C++ to the exporter, closes the loop:
**sim == exporter == firmware**.

## Watching a clip in the sim

- `python facehugger.py sim --clip "wiggle"` — physics: gravity, floor, free
  base; shows dynamic balance (and where a clip collapses).
- `--float` — no gravity/floor, body pinned; shows the **pure joint geometry** as
  authored (use this to confirm yaw/shoulder direction in isolation).
- `--loop` — replay continuously to watch cumulative drift/slip.

## Re-export reminder

The export writes `animation/exported_clips/` only. After any re-export, sync the
firmware copy by hand:

```
cp animation/exported_clips/clips_all.h code/firmware/src/nervous_system/clips_all.h
```
then re-run `pio test -e native` and `verify_export_parity.py`.
