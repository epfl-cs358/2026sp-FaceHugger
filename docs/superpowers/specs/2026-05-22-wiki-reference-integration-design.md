# Wiki reference integration — design spec

**Date:** 2026-05-22
**Branch:** `feat/wiki-setup`
**Tracking issue:** [#92](https://github.com/epfl-cs358/2026sp-FaceHugger/issues/92)

## Goal

Fold the six hand-written docs in `proposed-documentation/` into the wiki's
Reference tab as the primary, accurate how-it-works content, replacing the
thin stubs and superseding the stale `_context` material the audit flagged.
The docs were written from the `animation-flow-integration` branch and reflect
current firmware, the WebSocket API, and the Blender clip exporter.

## Scope

**In scope:**
- Port the six proposed docs into Reference pages (placement + restyle).
- Add a build-guide **Toolchain** section for software/exporter install-and-run.
- Extract the embedded verification checklists into one file kept *outside* the wiki.

**Out of scope:**
- The build guide's physical-build pages (design, parts, printing, wiring,
  assembly). A teammate is preparing a separate notes-and-ordering document;
  adapting that into the wiki is a parallel future task using the same
  port-and-restyle pattern as this spec, tracked on its own.
- Authoring net-new reference content where no source exists. Pages without a
  source doc stay as `todo` stubs (Tier 3 below).

## Style

Pages are **prose-like, concise, short, and clear**. The source docs are
table- and checklist-heavy; the wiki versions lead with explanatory prose and
keep tables only for genuinely tabular data (command refs, pin maps, gait
params, NEUTRAL poses). Strip the `Verify:` checklists entirely (see below).

## Target structure

Reference tab, after integration:

```
reference/
  conventions.md         ← conventions.md            (NEW, top-level, listed first)
  api.md                 ← api-reference.md           (NEW, top-level)
  firmware/
    index.md             ← nervous_system README + firmware-main-loop §1–2
    motion-engine.md     ← firmware-main-loop §3–5 (gait + clip mechanics)
    fsm-states.md        ← firmware-states (5 states, checklists stripped)
    kinematics.md        ← (existing stub; Tier 3)
    csv-diagnostics.md   ← (existing stub; Tier 3)
  animation/
    index.md             ← (existing stub; Tier 3)
    clip-panel.md        ← blender-addon.md           (NEW)
    blender-rig.md  urdf-pipeline.md  fhc-format.md  gait-design.md  (existing stubs; Tier 3)
  simulation/
    torque-analysis.md  torque-heatmap.md             (existing stubs; Tier 3)
  remote-control/
    index.md             ← + api-reference "Frontend Integration Summary"
    websocket-api.md     ← thin; points at reference/api.md
  roadmap.md             ← future-considerations.md   (NEW, top-level)
```

Build guide, added section (task-oriented "how to run/build the software"):

```
guide/
  software.md            ← slim to a conceptual overview that links into
                            reference/ and toolchain/
  toolchain/
    index.md             ← the pipeline at a glance: Fusion → URDF → Blender → clips → firmware
    flashing.md          ← PlatformIO build/upload/monitor; join FaceHugger_Net
    fusion-export.md     ← install + run ExportBodiesToURDF / ExportPrintableSTLs
    blender-clips.md     ← install Blender 5.x, open rig, FH Clip Panel basics, export (GUI + headless)
```

`conventions.md` and `api.md` are top-level Reference pages because both are
contracts shared by firmware, animation, and the app.

## Source-to-page mapping and treatment

| Source (`proposed-documentation/`) | Destination | Treatment |
|---|---|---|
| `conventions.md` | `reference/conventions.md` | Restyle to prose; keep the leg-ID, NEUTRAL, `translateToServo`, and servo-channel tables. Fold the old `servo-conventions.md` stub in here. |
| `api-reference.md` | `reference/api.md` (+ frontend summary → `reference/remote-control/index.md`) | Keep command/telemetry tables; drop `Verify:` blocks; move "Frontend Integration Summary" to remote-control. |
| `firmware-main-loop.md` | `reference/firmware/index.md` (§1–2) + `motion-engine.md` (§3–5) | Prose; keep gait-param and NEUTRAL tables and the Mermaid pipeline diagrams. |
| `firmware-states.md` | `reference/firmware/fsm-states.md` | Keep the FSM Mermaid diagram + state descriptions; **strip every `Verify:` checklist**. |
| `blender-addon.md` | `reference/animation/clip-panel.md` | Feature catalogue as prose; strip `Verify:` and integration-test sections. |
| `future-considerations.md` | `reference/roadmap.md` | Light restyle; it is already prose-ish. Keep next-steps tables. |

## Verification-checklist extraction

Collect every `Verify:` block (and the `blender-addon.md` integration-test
sections) from all six files into a single `proposed-documentation/verification-checklists.md`,
grouped by source file then section heading. This file stays in
`proposed-documentation/` and never enters `wiki/` (so it is not built or
published). It is a grouping aid for the user, not wiki content.

## What this supersedes

The proposed docs become the lead source for the firmware, API, conventions,
clip-panel, and roadmap pages. The matching `_context/` copies stay in the repo
as provenance (already excluded from the build via `exclude_docs`) but are no
longer the primary source where a proposed doc covers the same ground. The
`reference/firmware/servo-conventions.md` stub is removed (folded into
`conventions.md`); its `.pages` entry is dropped.

## Priority tiers

1. **Port existing content** (low effort, the proposed docs are already
   written): `conventions`, `api`, firmware `index`/`motion-engine`/`fsm-states`,
   `clip-panel`, `roadmap`, remote-control frontend summary.
2. **Build-guide Toolchain** (`flashing`, `fusion-export`, `blender-clips`,
   `index`): real authoring, but draws on the proposed docs + `CLAUDE.md`
   commands for content.
3. **Leave thin** (`todo` stubs, no source): firmware `kinematics`/`csv-diagnostics`,
   animation `blender-rig`/`urdf-pipeline`/`fhc-format`/`gait-design`, simulation
   torque pages. `fhc-format` may borrow from roadmap §2 if someone picks it up.

## Verification

`mkdocs build --strict` must pass after each tier: new pages reachable via nav,
`.pages` updated (Reference gains `conventions.md`, `api.md`, `roadmap.md`;
firmware `.pages` re-cut; `servo-conventions` removed), `_context/` and
`proposed-documentation/` excluded from the built site, no broken links/anchors.
