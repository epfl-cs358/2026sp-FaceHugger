<!--
WIKI-MIGRATION REFERENCE COPY
Original path:  animation/scripts/torque-heatmap.md
Original kind:  spec
Copied on:      2026-05-21
Status:         unreviewed
Maps to:        reference/simulation/torque-heatmap.md
-->

> **Reference material.** Verbatim copy of `animation/scripts/torque-heatmap.md` kept in `_context/`
> as source for the published wiki page. Edit the parent wiki page, not
> this file. The original at `animation/scripts/torque-heatmap.md` is still canonical; this file is
> scaffolding the user will delete manually once the wiki pages exist.
<!--
CONSISTENCY-CHECK 2026-05-21
Verified against: feat/wiki-setup @ 0a1a138
- [ok]    Body is byte-identical to original animation/scripts/torque-heatmap.md (only the H1 line differs in diff context). Still "not implemented".
- [ok]    Activity Heatmap colours per-joint servo meshes by per-frame |Δangle|: confirmed in fh_clip_panel.py _heatmap_handler() (L250) using `delta = abs(angle - prev)` and thresholds _HEATMAP_MID_DEG=5/_HIGH_DEG=20 (L103-104).
- [ok]    `_read_bone_angles()` returns all 12 IK-solved joint angles (fh_clip_panel.py L132-151, JOINT_BONES loop). URDF has per-link `<inertial>` mass+COM `<origin>` (13 inertial blocks in generated/facehugger.urdf) — feasibility inputs exist as claimed.
- [drift] Doc implies the heatmap colours all 12 joints; fh_clip_panel.py L106-111 notes only 8 of 12 are shown (hip/link2 has no `__servo` mesh — merged in CAD combine step). Display ≠ angle-read coverage.
- [todo]  Proposed τ_stall constant location ("convention.json or servo_spec", MG-series 1.8-3.5 kg·cm): animation/convention.json exists but has no servo stall field yet — Phase A (this task) does not verify the future-mode plan.
-->
# Torque heatmap — design note (future task)

Status: **not implemented.** The Display panel's Activity Heatmap
currently colours the per-joint servo meshes by per-frame `|Δangle|`
(motion/speed proxy). This note scopes a future *torque* mode so it can
be picked up deliberately. Decided via brainstorming: do the
gravity-only version first, clearly labelled; do **not** fold it into
unrelated work.

## What's feasible now (cheap, useful)

A **static gravity-hold torque** per joint, per frame:

- **Inputs we already have.** `code/simulation/generated/facehugger.urdf`
  carries per-link `<inertial>` `mass` + COM `<origin>` (refreshed by
  the latest CAD export). `_read_bone_angles()` already returns all 12
  IK-solved joint angles every frame (the heatmap handler calls it).
- **Model.** For each joint *j*, the gravity torque is
  `τ_j = Σ_{i ∈ outboard(j)} (J_{j,i})ᵀ · (m_i · g)`, i.e. the static
  load that servo must hold against gravity at that pose. Equivalent to
  one recursive Newton–Euler pass with zero velocity/acceleration.
- **Display.** Colour by `|τ_j| / τ_stall` (servo stall torque, a new
  constant — MG-series ≈ 1.8–3.5 kg·cm; put it in `convention.json` or
  a small `servo_spec`), reusing the existing green→orange→red ramp and
  the existing snapshot/restore viewport plumbing. Add a heatmap-mode
  enum (Activity Δ / Gravity torque) rather than a second toggle.

Effort: moderate. The only new pieces are a URDF inertial parser
(mass + COM per link) and the chain torque sum; everything else
(per-frame angle read, mesh colouring, panel) is reused.

## The honest caveat — ground contact

This gives the **true** torque only for a leg that is **lifted / in
swing** (free, gravity-loaded). For a leg **bearing the robot's weight
on the ground**, the real servo torque is dominated by the ground
reaction force, which depends on:

- which feet are in contact this frame (support polygon), and
- how the body weight is shared across the contacts.

A free-floating gravity model has none of that, so it will *under-read*
stance-leg torque badly. Options when this matters:

1. Label the mode "swing-leg / holds only — no ground contact" and
   accept it as a *motion* diagnostic (recommended first cut).
2. Add a contact heuristic: feet below a Z threshold are "planted",
   distribute `m_body · g` over the planted feet, add the reaction to
   each planted leg's chain. Crude but far closer for stances.
3. Full inverse dynamics with a contact/physics model (PyBullet already
   does this in `code/simulation/` — arguably compute torque there from
   the exported clip rather than in Blender). Heaviest; best accuracy.

Dynamic terms (accelerating the leg's own inertia) need inertia tensors
+ double-differenced joint angles; doable since clips are uniform-fps,
but noisy and lower-priority than the contact question.

## Recommendation

Ship option 1 (gravity-only, clearly labelled) as a standalone change
with its own headless test (assert a known lifted pose's hip torque
≈ hand-computed `m·g·lever`). Treat the contact model as a separate
follow-up, and seriously consider doing the accurate version in the
PyBullet sim instead of Blender — the sim already has the contact
solver and the URDF inertials loaded.
