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
