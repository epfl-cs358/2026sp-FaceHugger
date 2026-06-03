# Shoulder limits, inter-leg buffer, and the URDF→firmware→Blender alignment

This page consolidates the joint-limit rework that landed across the URDF generator, the firmware safety net, the Blender rig, and the export-time lint. The motivation was a recurring failure mode in which a clip authored in Blender played past the firmware's clamp on hardware — the four layers in the stack each had a slightly different opinion of what was reachable.

## The four layers that need to agree

| Layer | What it enforces | Source |
|---|---|---|
| **URDF `<limit>`** | Per-joint hard envelope, written into `facehugger.urdf` at generation time. | `code/simulation/urdf_gen/generate_urdf.py` |
| **Blender rig** | `LIMIT_ROTATION` constraint + `use_ik_limit_z` on each pose bone, read from URDF at rig-build time. The IK solver respects these mid-solve, so the animator can't drag a foot target to an out-of-envelope pose. | `animation/scripts/urdf_to_blender_rigged.py:add_limit_rotation_constraints` |
| **Firmware `clampClipServos`** | Per-leg, CALIB-relative servo-space window on the clip playback path only. Symmetric. | `code/firmware/src/nervous_system/motion_math.cpp` |
| **Firmware `enforceShoulderLimits`** | Math-space safety net with the per-side asymmetric shoulder envelope plus the same-side inter-leg gap rule. Currently lives in `motion_math.cpp` but is *not yet wired* into the gait / clip / T:12 tick paths — see below. | `code/firmware/src/nervous_system/motion_math.cpp` |

The export-time **`check_clip_clamp.py` lint** is a fifth surface that doesn't enforce anything but flags clips that would hit the safety net so the animator can act before exporting.

## Current numeric envelope per joint

All three of URDF, Blender rig, and firmware now use the same numbers.

| Joint | Range (URDF θ from rest) | Notes |
|---|---|---|
| Shoulder, LEFT body (FL, BL) | `[-52°, +90°]` | Tight side inward toward the body, loose side sweeps outward away from the body's neighbour leg. |
| Shoulder, RIGHT body (FR, BR) | `[-90°, +52°]` | Mirror of LEFT — the URDF axis is uniform `+Z` for all four shoulders, so the *sign* of the loose side flips between body sides. |
| Thigh (link2), all four legs | `[-75°, +75°]` | Symmetric. CAD-derived from Fusion. |
| Knee (link3), all four legs | `[-90°, +90°]` | Symmetric. CAD-derived from Fusion. |

The shoulder asymmetry arises because the shoulder URDF axis is uniformly `+Z` (post-`dc97133`), but the physical "outward" direction is mirrored across the body-X axis: for LEFT-body legs the outward direction is `+θ` (CCW about `+Z` viewed from above), for RIGHT-body legs it's `-θ`. A uniform `[-52°, +90°]` window would let FR swing `+90°` toward FL's quadrant, which would mechanically collide. The mirrored window puts the loose `+90°` side on the physically-outward direction for every leg.

## Where each constant lives

For the shoulder envelope:

```c
// code/firmware/src/shared/config.h
#define SHOULDER_THETA_NARROW_DEG  52   // tight inward URDF side
#define SHOULDER_THETA_WIDE_DEG    90   // loose outward URDF side
```

`enforceShoulderLimits` reads these and applies the per-side asymmetric clamp:

```c
LEFT  (FL, BL):  θ ∈ [-NARROW, +WIDE]   // outward = +θ
RIGHT (FR, BR):  θ ∈ [-WIDE,   +NARROW] // outward = −θ
```

The URDF generator (`generate_urdf.py`) writes the same per-body-side window into each shoulder joint's `<limit>` tag. Body side is derived from the leg-ID suffix — `r` for right, `l` for left — *not* from the `side` field in `facehugger_config.yaml`, which tracks bracket pairing (FL+BR share one bracket shape, FR+BL the other) and is used for the hip/knee axis flip.

For the thigh/knee clip-playback envelope:

```c
// code/firmware/src/shared/config.h
#define HIP_CLAMP_FROM_NINETY   52   // shoulder (kept as a defensive backup)
#define THIGH_CLAMP_FROM_CALIB  75   // matches URDF ±75°
#define KNEE_CLAMP_FROM_CALIB   90   // matches URDF ±90°
```

`clampClipServos` builds a CALIB-centred window: `thigh ∈ [CALIB_THIGH ± 75]`, `knee ∈ [CALIB_KNEE ± 90]`. Per-leg because the CALIB constants differ per leg; the half-width is uniform.

The shoulder clamp `HIP_CLAMP_FROM_NINETY = 52` is the older symmetric servo-space clamp. It's superseded by `enforceShoulderLimits` for the per-side asymmetric envelope, but kept in place as a defensive backup on the clip playback path — anything that bypasses the new safety net still gets a `±52°` ceiling.

## Same-side inter-leg gap rule

Beyond the per-joint hard envelope, the front and back leg on the same side can collide even when both are inside their individual hard envelopes — a front leg swung most of the way back into its back neighbour's quadrant. The rule:

```
math-space shorter-arc gap between FR and BR ≥ INTER_LEG_BUFFER_DEG
math-space shorter-arc gap between FL and BL ≥ INTER_LEG_BUFFER_DEG
```

Where `INTER_LEG_BUFFER_DEG = 5°` (see `config.h`). The rule is in *math space* (firmware NEUTRAL[] convention, the absolute yaw bearing in the body frame) and uses the shorter arc between two angles, so it's coordinate-free and works trivially at rest — at rest each gap is 90°, well above 5°, no clamping fires.

When the rule does fire, the *back* leg is pushed in whichever direction widens the gap; the front leg leads, the back follows. If the back leg's hard envelope conflicts with the push (the back leg can't move far enough), the hard envelope wins and the gap collapses below the buffer at that frame.

Diagonal pairs (FR↔BL, FL↔BR) can't physically collide on a quadruped — they're on opposite sides of the body — so they're not checked.

## What runs where today

Hard envelope (per-joint):

- **URDF + Blender rig**: enforced as hard IK clamps. An animator dragging a foot target gets the bone clamped at the envelope boundary by the IK solver — no warning needed.
- **Firmware `clampClipServos`**: enforces the symmetric clip-playback envelope on the clip path. Gait output is *not* clamped here (kept bit-for-bit identical for parity).
- **Firmware `enforceShoulderLimits`**: implements the per-side asymmetric shoulder envelope. *Not yet wired* into `spinal_cord.cpp` — the function exists and is unit-tested but no tick calls it. When wired, it will live before `translateToServo` in `applyServos`, so gait + clip + T:12 streaming all flow through it.

Inter-leg gap:

- **Export lint** (`check_clip_clamp.py` and `check_export_consistency.py`): runs after every Blender export. Flags clips that would violate the buffer as a `WARN` line — does not block the export. The clip data on disk still has the violation; the animator can decide whether to re-author or accept the silent firmware truncation on hardware.
- **Firmware `enforceShoulderLimits`**: implements the same rule. Same wiring caveat as above — not yet active.

## What changed and why (changelog summary)

| Commit | Change | Why |
|---|---|---|
| `dc97133` | Shoulder URDF axis fixed to uniform `+Z` for all four legs; downstream sign compensations removed. | The link1 bracket STL is mirrored for FR/BL but the servo itself isn't — the shaft direction does not flip across legs. Every downstream layer (`sil_bridge`, `clip_player`, `servo_math`) had been silently compensating for an axis flip that wasn't real. |
| `cb8b911`, `650646b`, `d2634b1`, `0797c6e` | CALIB decoupling: clip streaming moved from T:4 (per-channel raw servo write, coupled to firmware CALIB) to T:12 (one math-space packet per frame, firmware applies translateToServo + CALIB on receipt). | Exporter no longer needs to know hardware CALIB values; the mobile-app bundle is tagged `wire: "T12"` to catch stale CALIB-baked bundles at app load. |
| `3021483` | `check_export_consistency.py` updated to compare math-space ↔ math-space (matching the new T:12 export); stale T:4 `.js` files skipped with a re-export hint instead of failing noisily. | The post-CALIB-decouple `.js` carries floats; the old integer regex matched zero frames and every clip failed loudly. |
| `83bc65b` | `enforceShoulderLimits` added to `motion_math.cpp` with native unit tests. | Pure-math piece of the future safety net; not yet wired into ticks per the deferred firmware-wiring decision. |
| `24314a2` | `check_inter_leg_gaps` added to both `check_clip_clamp.py` and `check_export_consistency.py`; Blender export now emits `WARN` lines for clips violating the 5° buffer. | Lint surface for the inter-leg rule while firmware wiring is paused. |
| `c5ebbee` | `generate_urdf.py` writes per-body-side asymmetric shoulder limits; Blender rig rebuilt. | Aligns URDF + rig with `enforceShoulderLimits` so all three layers enforce the same window. Animator IK in Blender now refuses to push past the firmware envelope. |
| `3f552f2` | `THIGH_CLAMP_FROM_CALIB` widened from 60 to 75 to match URDF `±75°`. | The firmware clamp was tighter than URDF, so Blender IK could solve to thigh poses the firmware would silently truncate. |

## Re-export checklist

Whenever the URDF or rig limits change, the export pipeline must re-bake:

1. `python facehugger.py urdf` — regenerates `code/simulation/generated/facehugger.urdf` from `fusion_export.json` + `facehugger_config.yaml` + `generate_urdf.py`.
2. `python facehugger.py blender --rigged` — rebuilds `animation/fh_rigged_latest.blend` against the new URDF.
3. Re-bake clips from the rebuilt rig and re-run the export — IK-driven keyframes change wherever the old limits were wider than the new ones. The export's `WARN` lines flag any clip that ends up violating the inter-leg buffer at the new envelope.
4. Update reference traces if the SIL clip suite fails: `python code/facehugger.py update-reference-clips`.

## Related

- [Conventions: joint axes and URDF conventions](../conventions.md#joint-axes-and-urdf-conventions)
- [Motion engine: URDF clip-clamp envelope](motion-engine.md#urdf-clip-clamp-envelope)
- [Servo conventions](servo-conventions.md)
- [Animation: clip panel](../animation/clip-panel.md)
- [Animation: URDF pipeline](../animation/urdf-pipeline.md)
