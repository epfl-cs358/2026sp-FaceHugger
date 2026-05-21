<!--
WIKI-MIGRATION REFERENCE COPY
Original path:  doc/animation-pipeline/README.md
Original kind:  readme
Folder context: Index of the doc/animation-pipeline/ design+research+roadmap docs.
Copied on:      2026-05-21
Status:         unreviewed
Maps to:        reference/animation/index.md
-->

> **Reference material.** Verbatim copy of `doc/animation-pipeline/README.md` kept in `_context/`
> as source for the published wiki page. Edit the parent wiki page, not
> this file. The original at `doc/animation-pipeline/README.md` is still canonical; this file is
> scaffolding the user will delete manually once the wiki pages exist.
<!--
CONSISTENCY-CHECK 2026-05-21
Verified against: feat/wiki-setup @ 0a1a138
- [ok]    All 6 sibling links (firmware-research.md, leg-coordinates.md, urdf-conventions.md, animation-pipeline-roadmap.md, api-surface.md, animation-library-pipeline-blender-setup-for-animation-rig.md) still exist in the ORIGINAL doc/animation-pipeline/ dir, so relative links resolve from the original location. They will break in the published wiki page unless repointed — Phase B.
- [ok]    Cross-dir links resolve from original: code/API_SPEC.md, code/simulation/kinematics.py, code/simulation/docs/API_ANIMATION_SPEC.md, animation/scripts/urdf_to_blender_rigged.py, animation/SERVO_ID_CONVENTION.md all present.
- [ok]    "Implementation: not yet started" matches reality — no T1-T10 firmware artifacts exist (no leg_ik/fhc_loader/bezier_eval/gait_engine in code/firmware/src/). CLAUDE.md concurs.
- [ok]    "replaces legacy .gait + textbook IK in kinematics.cpp, scheduled for removal" matches CLAUDE.md.
- [drift] kinematics.py described as "ported to C in firmware" — the C port (leg_ik.{h,cpp}, roadmap T1) does not exist yet; current firmware kinematics.cpp is the legacy/wrong-rig one. Aspirational, not done.
-->
# Animation Pipeline — Documentation Index

Design + research + roadmap for the FaceHugger's animation pipeline:
how clips are authored in Blender, exported to a binary `.fhc`
format, loaded onto the ESP32, and played back at 100 Hz with
on-board IK + IMU body-pose correction.

This pipeline **replaces** the legacy `.gait` baked-angle format and
the textbook IK in `code/firmware/.../kinematics.cpp`, both of which
are scheduled for removal.

---

## Read in this order

1. [`firmware-research.md`](firmware-research.md) — ESP32 platform
   constraints (memory, dual-core, flash, IK cost). Synthesized from
   a Claude/Gemini/Perplexity research round. Sets the *budget* the
   design must fit in.
2. [`leg-coordinates.md`](leg-coordinates.md) — **the design canon**.
   Coordinate frame, foot-XYZ + Bezier storage, IK math, file format
   spec, runtime pipeline. Read this before writing any code.
3. [`urdf-conventions.md`](urdf-conventions.md) — joint-origin
   convention, RL/sim-handoff requirements, and the coordinate
   contracts that must NOT change without bumping the `.fhc`
   version.
4. [`animation-pipeline-roadmap.md`](animation-pipeline-roadmap.md)
   — task breakdown for the team. Independent tasks (T1–T6),
   sequencing, ownership suggestions, end-to-end test plan (T9).
5. [`api-surface.md`](api-surface.md) — runtime gait engine's C API
   surface; how the existing WebSocket protocol in
   [`code/API_SPEC.md`](../../code/API_SPEC.md) plugs into it.
6. [`animation-library-pipeline-blender-setup-for-animation-rig.md`](animation-library-pipeline-blender-setup-for-animation-rig.md)
   — how the Blender rig is rebuilt from Fusion model iterations
   (the upstream of authoring).

---

## Cross-cutting references outside this directory

- [`code/API_SPEC.md`](../../code/API_SPEC.md) — WebSocket protocol
  between dashboard and firmware.
- [`code/simulation/kinematics.py`](../../code/simulation/kinematics.py)
  — canonical IK implementation, ported to C in firmware.
- [`code/simulation/docs/API_ANIMATION_SPEC.md`](../../code/simulation/docs/API_ANIMATION_SPEC.md)
  — animator-facing reference for the existing rigged Blender scene.
- [`animation/scripts/urdf_to_blender_rigged.py`](../../animation/scripts/urdf_to_blender_rigged.py)
  — the rig builder.
- [`animation/SERVO_ID_CONVENTION.md`](../../animation/SERVO_ID_CONVENTION.md)
  — servo numbering authority (firmware vs URDF translation).

---

## Status

**Design**: locked. Open items tracked explicitly in
`leg-coordinates.md` §9 and `api-surface.md` "Open contract questions".

**Implementation**: not yet started. Roadmap in
`animation-pipeline-roadmap.md`. Tasks T1–T6 are parallelizable; T7
(runtime engine) integrates them; T8 connects to WebSocket; T9 is the
bench-test sequence; T10 (IMU correction) runs on a separate track.

---

## Local-only artifacts (gitignored)

The directories below contain raw research material that fed the
synthesis above. They are **gitignored** to keep this branch clean —
the synthesis docs at the top of this directory are the canonical,
shareable source of truth.

- `llm-answers/` — original LLM responses (Claude/Gemini/Perplexity
  for firmware-research; Claude conversation for leg-coordinates).
- `synthesis/` — per-topic intermediate synthesis files used while
  drafting `firmware-research.md`.

If you need to access them, they're still in your working tree on
your local clone. They simply aren't shared via git.
