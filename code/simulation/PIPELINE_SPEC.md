# FaceHugger URDF Pipeline — CAD Decisions & Spec

This document captures all grilling answers and design decisions for the
Claude Code plan adaptation. Use this as the authoritative reference when
updating the export script, generator, and config.

---

## 1. Leg assignment and mesh pairing

The base leg model (L mesh) points along **-X** at zero joint angle.
The R mesh is the XZ-plane mirror of L — it naturally points along **+X**.

| Leg | Mesh | Orientation | Quadrant | rpy_z_deg |
| --- | ---- | ----------- | -------- | --------- |
| FL  | L    | base        | -X, +Y   | 0°        |
| BR  | L    | 180° Z flip | +X, -Y   | 180°      |
| FR  | R    | base        | +X, +Y   | 0°        |
| BL  | R    | 180° Z flip | -X, -Y   | 180°      |

**Diagonal pairs:**
- FL + BR share the L mesh
- FR + BL share the R mesh

The 180° Z flip for the back leg of each pair rotates around the leg's own
mount point (`LegMountPointXX`) — the mount point stays fixed, the leg flips
in place.

---

## 2. Mirror plane for R mesh

`Link1R` and `MotorMountR` were mirrored through the **XZ plane of the
FaceHuggerLegAssembly** origin. `BodyToLink1Point` has local position
`(11.97, 0.0, -4.2)` — Y=0, so it lies exactly on the XZ plane. The mirror
is invariant at that point. No separate plane or construction point needed.
Re-origining math works cleanly: subtracting `BodyToLink1Point` from R
vertices gives the same shoulder-axis-at-origin result as for L.

---

## 3. Mount point naming

| File                      | Point name           | Purpose                         |
| ------------------------- | -------------------- | ------------------------------- |
| `FlexibleSkeleton` (body) | `LegMountPointFL`    | Body-side attachment, FL corner |
| `FlexibleSkeleton` (body) | `LegMountPointFR`    | Body-side attachment, FR corner |
| `FlexibleSkeleton` (body) | `LegMountPointBR`    | Body-side attachment, BR corner |
| `FlexibleSkeleton` (body) | `LegMountPointBL`    | Body-side attachment, BL corner |
| `MotorMount` (bracket)    | `LegMountFixedPoint` | Bracket-side attachment point   |

`LegMountFixedPoint` on each bracket coincides with its corresponding
`LegMountPointXX` on the body when assembled. The URDF generator uses
`LegMountPointXX` as the shoulder joint origin. `LegMountFixedPoint` is
for CAD alignment verification only.

All 4 `LegMountPointXX` are at the **same Z height** on the body.

The yaml `mount_point` keys reference `LegMountPointFL` etc. — update the
yaml to use these new names.

---

## 4. Shoulder joint zero and limits

**Zero angle definition:** 0° = leg pointing along -X (FL base orientation).
This comes directly from the Fusion joint definition.

**Fusion limits for FL** (source of truth, L mesh): `lower = -105°, upper = +45°`

From FL's limits, derive the other legs by checking which direction is
"forward" (+Y) for each leg:

| Leg | Zero points  | Forward swing | Backward swing | URDF lower | URDF upper |
| --- | ------------ | ------------- | -------------- | ---------- | ---------- |
| FL  | -X           | +45°          | -105°          | **-105°**  | **+45°**   |
| BR  | +X (flipped) | -45°          | +105°          | **-45°**   | **+105°**  |
| FR  | +X           | -45°          | +105°          | **-45°**   | **+105°**  |
| BL  | -X (flipped) | +45°          | -105°          | **-105°**  | **+45°**   |

**Pattern:** legs pointing -X (FL, BL) keep FL limits as-is.
Legs pointing +X (FR, BR) get limits negated and swapped.

These values come from the Fusion revolute joint on `Link1Revolute`. Once
Phase T (joint capture) lands, the generator reads them directly from the
CAD export — no manual entry in yaml.

---

## 5. Link2 and Link3 orientation

Link2 and Link3 are **shared meshes** (no L/R split yet). Their visual
orientation per leg is handled by `mesh_rpy` in the URDF generator via
the `R_L1` rotation — same as the current pipeline. Do **not** rotate
Link2/Link3 bodies in Fusion CAD. Leave them as-is.

---

## 6. Body components (base_link)

The following are chassis-fixed and bake into `QuadrupedBody.stl` when
`FlexibleSkeleton:1` is exported as a single STL:

- `QuadrupedBody` (main frame)
- `LipoCage` (sub-body, treated as part of chassis)
- All electronics components (hidden, excluded by `VISIBLE_ONLY = True`)

`MotorMount` and `MotorMountR` now live inside `FaceHuggerLegAssembly`,
**not** inside `FlexibleSkeleton`. The old `LegMountXX:1` occurrences in
`FlexibleSkeleton` are removed.

---

## 7. Revolute joints in Fusion

Three revolute joints are now defined inside `FaceHuggerLegAssembly`:

| Joint name      | Links               | Axis             |
| --------------- | ------------------- | ---------------- |
| `Link1Revolute` | MotorMount → Link1L | BodyToLink1Axis  |
| `Link2Revolute` | Link1L → Link2L     | Link1ToLink2Axis |
| `Link3Revolute` | Link2L → Link3L     | Link2ToLink3Axis |

Limits are set in Fusion with rest angles. Phase T (A3 in the plan) captures
these via `collect_joints()` into `fusion_export.json`. The generator prefers
CAD-sourced limits over yaml, with yaml as override fallback.

---

## 8. R mesh base orientation

When `Link1R` is placed in FR position with `rpy_z_deg = 0°`, it naturally
points along **+X** — the XZ mirror flips the X direction. No additional
rotation needed for FR base placement. Same for BL at `rpy_z_deg = 180°`.

---

## 9. Rotation center for 180° flip

The 180° Z rotation for BR and BL rotates **around the leg's own mount
point** (`LegMountPointBR` / `LegMountPointBL`). The mount point stays
fixed in world space — the leg geometry flips in place around that point.

---

## 10. Summary: what changes in each file

| File                     | Change                                                                                                                                                |
| ------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------- |
| `facehugger_config.yaml` | Update `mount_point` keys to `LegMountPointXX`; add `side: L/R` per leg; set `rpy_z_deg` per table above; set per-leg shoulder limits per table above |
| `ExportBodiesToURDF.py`  | Fix `MESH_EXPORTS` for moved bracket; add L/R STL rules; add `collect_joints()`                                                                       |
| `generate_urdf.py`       | Per-side mesh selection; per-leg limits from table; consume CAD joints when present                                                                   |
| `simulate_v2.py`         | `shoulder_neutral_deg = 0°` for all legs; per-leg limits from config                                                                                  |
