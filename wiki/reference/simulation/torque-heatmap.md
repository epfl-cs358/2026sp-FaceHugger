# Torque heatmap

The Activity Heatmap is a feature of the FH Clip Panel's Display sub-panel in Blender. It colours servo meshes by the per-frame angle delta (`|angle - prev_angle|`) for each joint, giving a motion-intensity proxy that highlights mechanical shock before a clip is exported.

## What the heatmap shows

Each servo mesh is coloured by how much its joint angle changed since the previous frame. A large delta means the servo is being driven hard or jerked, which is a proxy for high torque load and mechanical shock.

Only 8 of the 12 joints are shown. The 4 hip/link2 joints have no `__servo` mesh (merged away in the CAD combine step), so they are excluded from the display. Joint angles are still read for all 12 joints internally by `_read_bone_angles()`.

## Reading the colour scale

- Green: quiet joint (delta below ~5 degrees)
- Orange: moderate activity (~5 to 20 degrees)
- Red: jerky or high-load movement (delta above ~20 degrees)

Aim to keep high-red frames out of loops that repeat frequently, as they indicate servo stress.

## Generating it

Each frame, `_heatmap_handler()` in `fh_clip_panel.py` reads the IK-solved bone angles, computes the per-frame delta for each joint, maps the delta through the green/orange/red thresholds (`_HEATMAP_MID_DEG = 5`, `_HIGH_DEG = 20`), and applies a material colour to the corresponding `__servo` mesh in the viewport. No external script is needed; just enable the heatmap toggle in the Display sub-panel of the FH Clip Panel while a clip is active.

## Planned: gravity-hold torque mode

A second heatmap mode is scoped but not implemented: instead of motion delta, colour each joint by the **static gravity-hold torque** the servo has to produce to hold the pose against gravity. This is a true torque proxy, not a speed proxy, so it catches different problems (a slowly-held outstretched leg can be near stall without any motion).

The inputs are already in the repo. `code/simulation/generated/facehugger.urdf` carries per-link `<inertial>` (mass + COM origin) refreshed by every CAD export, and `_read_bone_angles()` already provides the 12 IK-solved joint angles per frame. The model is the standard outboard chain sum,

```
τ_j = Σ_{i ∈ outboard(j)} (J_{j,i})ᵀ · (m_i · g)
```

i.e. one recursive Newton–Euler pass with zero velocity and acceleration. Display colour normalizes by a servo stall constant (`τ_stall`, MG-series ≈ 1.8–3.5 kg·cm) and reuses the existing green→orange→red ramp; the panel grows a mode enum (Activity Δ / Gravity torque) rather than a second toggle. The only genuinely new pieces are a URDF inertial parser and the chain torque sum.

The honest caveat is **ground contact**. A gravity-only model is only true for legs in *swing*; a stance leg is dominated by the ground reaction force, which depends on which feet are planted and how body weight is shared across the support polygon. The recommended first cut is to ship the gravity-only mode labelled clearly as "swing-leg / holds only — no ground contact" and treat it as a motion diagnostic. A crude contact heuristic (feet below a Z threshold are planted; distribute `m_body · g` over them) is a reasonable follow-up. Full inverse dynamics with contact is heavier still, and arguably belongs in PyBullet (where the contact solver and URDF inertials are already loaded) rather than Blender — that path computes torque from the exported clip rather than from the rig.

Acceptance for the first cut is a headless test asserting a known lifted pose's hip torque matches a hand-computed `m·g·lever` within tolerance.
