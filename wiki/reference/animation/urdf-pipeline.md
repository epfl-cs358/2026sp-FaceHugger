# 3D model to URDF

This is the first stage of the pipeline: a Fusion 360 assembly becomes a single `facehugger.urdf` that PyBullet and Blender both consume. It is a two-program pipeline joined by one JSON intermediate, plus a small declarative config. The URDF is the kinematic source of truth, and everything downstream (the sim, the Blender rig, the IK reference cases) is derived from it.

```mermaid
flowchart TD
    CAD["Fusion 360 assembly"] -->|ExportBodiesToURDF add-in| J["fusion_export.json<br/>(occurrence tree + joints[] + mesh manifest)"]
    CAD -->|re-origined STLs| M["generated/exported_meshes/*.stl"]
    J --> G["generate_urdf.py"]
    CFG["facehugger_config.yaml"] --> G
    M --> G
    G --> U["generated/facehugger.urdf"]
```

## Who owns what

**The Fusion add-in** (`cad/scripts/ExportBodiesToURDF/`) owns everything that needs the live CAD model. It runs inside Fusion and writes three artifacts to `code/simulation/generated/`:

- `fusion_export.json`: the complete, unpruned occurrence tree (every screw, PCB, servo) plus four name-driven whitelists (the mesh manifest, construction points, construction axes, and a `joints[]` array with each revolute joint's axis, origin, and limits). The tree is kept complete on purpose so downstream tools can resolve any occurrence path for a world transform; pruning it would break path resolution.
- `fusion_export.txt`: a human-readable version for sanity checking.
- `exported_meshes/*.stl`: the meshes, selected by an `EXPORT_RULES` whitelist.

Crucially, the add-in owns which CAD bodies become which STL (`EXPORT_RULES`), the re-origin geometry (moving each mesh's local origin onto its URDF joint landmark), and the joint axis/origin/limit capture. Mesh visibility in Fusion has no effect; capture is entirely whitelist-driven.

**`generate_urdf.py`** (`code/simulation/urdf_gen/`) owns the kinematic assembly. It reads the JSON and the config and emits the URDF XML: the CAD-name to URDF-name joint topology, the per-leg world placement, the Convention A shoulder-rest derivation, the right-side axis and limit flips, and the inertial-origin correction. It contains zero hardcoded coordinates; all geometry comes from the JSON.

**`facehugger_config.yaml`** owns only what the CAD genuinely does not know: robot and mesh names, the four leg instances (id, mount point, and L/R side for diagonal mesh sharing), and servo physical fallbacks (`mass_kg`, `effort_nm`, `velocity_rad_s`). After Convention A landed, the per-leg yaw, shoulder limits, and shoulder neutral were removed from the config because they are now derived.

## The key transforms (and why they exist)

- **Leg-assembly normalization.** The leg assembly is placed in CAD with a non-identity world rotation (around 90 degrees about Z). The add-in extracts that rotation and pre-applies it to each leg-internal joint's axis and origin so the JSON lands in a world-aligned frame.
- **STL re-origin.** Each mesh is exported with its vertices in the occurrence's world frame, then the joint-landmark world position is subtracted so the mesh's local origin sits exactly on its URDF joint (`BodyToLink1Point`, `Link1ToLink2Point`, and so on). The amount subtracted is recorded as `origin_shift_mm` in the manifest. This is why the URDF emits `<origin xyz="0 0 0"/>` on every visual: the geometry is already placed.
- **Inertial CoM correction.** Because the visual origin is zeroed but Fusion reported the center of mass in the pre-shift frame, `generate_urdf` subtracts the same `origin_shift_mm` from the CoM so the inertial frame still tracks the geometry. A small placeholder mass and inertia are used when Fusion physics are missing.
- **Convention A shoulder rest.** Joint zero equals the Fusion mechanical rest. The FL shoulder rest defines zero; FR/BL/BR rests are derived by mirror and rotate, and right-side legs get their axis negated and limits negate-swapped so that the same joint angle produces the same physical motion on every leg. See [Conventions](../conventions.md) for the math-space and per-leg servo mapping that this feeds.

## Non-obvious decisions worth knowing

- **The URDF is the source of truth, and everything is derived from it.** PyBullet, the Blender rig, and the IK reference cases are all expected to agree with the URDF, not the other way around. `generate_urdf` even bakes a machine-parseable leg-metadata comment into the URDF so the simulator need not re-open the JSON.
- **The mirror plane is the leg-assembly XZ plane.** The right-side link and mount bodies are XZ mirrors of the left, and `BodyToLink1Point` lies on the plane (Y = 0), so the re-origin math is identical for both sides.
- **Two distinct "flips" must not be confused.** The geometric back-of-pair flip (0 or 180 degrees, derived from leg id, which positions meshes) is separate from the kinematic shoulder rest (Convention A, which goes into the joint origin rpy).
- **Re-exports preserve the servo-role assignment.** The manifest block recording which servo occurrence is shoulder, hip, or knee is read from the prior JSON and carried forward, with stale paths repaired after a CAD rename.

## Gotchas for a builder or extender

1. **Never hand-edit anything in `generated/`** (`facehugger.urdf`, `fusion_export.json`, the STLs). They are all regenerated.
2. **`EXPORT_RULES` and the whitelists live in the add-in, not downstream.** To change which bodies make up the chassis or a link, edit `EXPORT_RULES` (and the point/joint whitelists) in the add-in, then re-export. `generate_urdf` only reads the resulting manifest.
3. **The CoM frame trap.** Fusion reports a center of mass in the immediate parent frame, not the occurrence's own frame, so it must be lifted with the parent-to-world transform or it double-transforms.
4. **The foot tip is currently inferred** from the max +Y vertex cluster of `leg_lower.stl` until a real foot-tip construction point exists in CAD.
5. **Servo numbering is still a proposal.** `servo_mapping.yaml` is pending firmware `SERVO_CONFIG[]` confirmation; flag the gap before baking clips.
6. **Leg naming.** URDF and Python use `fl/fr/bl/br`. Do not propagate the firmware's `fr/fl/rr/rl` names here.

For the CAD-side authoring steps see the [Fusion export how-to](../../guide/toolchain/fusion-export.md). The next stages are [URDF to Blender rig](blender-rig.md) and [Blender to robot clips](clip-panel.md).
