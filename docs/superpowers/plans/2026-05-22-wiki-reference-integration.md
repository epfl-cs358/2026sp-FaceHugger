# Wiki Reference Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fold the six `proposed-documentation/` files into the wiki Reference tab as the primary how-it-works content, add a build-guide Toolchain section, and extract the verification checklists out of the wiki.

**Architecture:** Port-and-restyle. Each task adapts one source doc into one (or two) Reference pages as concise prose, wires the page into its `.pages` nav, fixes any links broken by deletions, and gates on `mkdocs build --strict`. `conventions.md` and `api.md` become top-level Reference pages; firmware is re-cut into three concept pages; the old `servo-conventions.md` and `api.md` firmware stubs are removed.

**Tech Stack:** MkDocs Material, awesome-pages (`.pages` nav), `mkdocs build --strict` as the test gate. Build via the worktree venv: `.venv-docs/bin/mkdocs`.

---

## House style (applies to every page-authoring step)

- **Prose-like, concise, short, clear.** Lead with explanatory sentences. Do not paste source docs verbatim — restyle.
- **Keep tables only for genuinely tabular data** (command refs, pin/channel maps, NEUTRAL poses, gait params). Convert bullet-dumps to prose.
- **Strip every `Verify:` block** and Blender integration-test section — those are extracted in Task 1, not published.
- **No em dashes** (use `-`); no decorative arrows in prose (use `->`). Match the existing wiki pages.
- Keep the Mermaid diagrams from the firmware docs (loop, FSM, pipelines) — they are high-value.

## File map

- Create: `proposed-documentation/verification-checklists.md`
- Create: `wiki/reference/conventions.md`, `wiki/reference/api.md`, `wiki/reference/roadmap.md`
- Create: `wiki/reference/firmware/motion-engine.md`, `wiki/reference/firmware/fsm-states.md`
- Create: `wiki/reference/animation/clip-panel.md`
- Create: `wiki/guide/toolchain/.pages`, `wiki/guide/toolchain/index.md`, `wiki/guide/toolchain/flashing.md`, `wiki/guide/toolchain/fusion-export.md`, `wiki/guide/toolchain/blender-clips.md`
- Rewrite: `wiki/reference/firmware/index.md`, `wiki/guide/software.md`, `wiki/reference/remote-control/index.md`
- Delete: `wiki/reference/firmware/servo-conventions.md`, `wiki/reference/firmware/api.md`
- Modify (nav): `wiki/reference/.pages`, `wiki/reference/firmware/.pages`, `wiki/reference/animation/.pages`, `wiki/guide/.pages`
- Modify (links): `wiki/guide/wiring.md:27`, `wiki/guide/design.md:24`, `wiki/reference/remote-control/websocket-api.md:6`

Sources are the six files in `proposed-documentation/`. Existing reference source material is in each domain's `wiki/reference/**/_context/` (excluded from the build).

---

### Task 1: Extract verification checklists (out of the wiki)

**Files:**
- Create: `proposed-documentation/verification-checklists.md`

- [ ] **Step 1: Create the consolidated checklist file**

Read each of the six `proposed-documentation/*.md` files. Copy every `Verify:` block (and `blender-addon.md`'s "Verification Checklist - Integration Tests" section with the `test_*.py` subsections) verbatim into `proposed-documentation/verification-checklists.md`, grouped by source file then by the section heading the block came from. Structure:

```markdown
# FaceHugger verification checklists

Consolidated from the proposed-documentation source files. Grouping aid only - not part of the wiki.

## From api-reference.md
### T:1 - CMD_MOVE
- [ ] ...(verbatim checklist items)...
### T:2 - CMD_FSM_STATE
...

## From firmware-states.md
### IDLE
- [ ] ...
...

## From blender-addon.md
### 1. Clip Management
- [ ] ...
### Integration tests (test_servo_parity.py / test_clips_header.py / test_bake_independence.py)
...
```

Preserve the exact command strings and expected outputs. Do not summarize.

- [ ] **Step 2: Confirm it is outside the build**

Run: `cd /Users/marcushamelink/Documents/EPFL-projects/projects-ba6/Making-Intelligent-Things/FaceHuggerWorktrees/wiki-setup && ls wiki/ | grep -c proposed-documentation`
Expected: `0` (the file lives at repo root under `proposed-documentation/`, not under `wiki/`, so MkDocs never builds it).

- [ ] **Step 3: Commit**

```bash
git add proposed-documentation/verification-checklists.md
git commit -m "docs(wiki): extract proposed-doc verification checklists into one file (kept out of wiki)"
```

---

### Task 2: `reference/conventions.md` (new top-level page)

**Files:**
- Create: `wiki/reference/conventions.md`
- Modify: `wiki/reference/.pages`

- [ ] **Step 1: Write the page**

Port `proposed-documentation/conventions.md` into `wiki/reference/conventions.md`, restyled to concise prose per the house style. Keep these as tables: the leg-naming/`LegId` table, the NEUTRAL pose table, the `translateToServo()` per-leg formulas, and the `LEG_SERVO_CHANNEL[4][3]` channel map. Keep the two ASCII body-frame diagrams. Keep the "Quick Reference: Which Space Am I In?" table. This page also absorbs servo numbering (it makes the old `firmware/servo-conventions.md` stub redundant - that stub is deleted in Task 4). Start the file with `# Conventions` (H1) and no `_context` provenance banner (this is a published page, not a reference copy).

- [ ] **Step 2: Add to nav**

Edit `wiki/reference/.pages` to list `conventions.md` first:

```yaml
title: Reference
nav:
  - conventions.md
  - firmware
  - animation
  - simulation
  - remote-control
```

- [ ] **Step 3: Build strict**

Run: `.venv-docs/bin/mkdocs build --strict 2>&1 | tail -5`
Expected: `Documentation built` with no `WARNING`/`ERROR` lines (ignore the Material "MkDocs 2.0" banner).

- [ ] **Step 4: Commit**

```bash
git add wiki/reference/conventions.md wiki/reference/.pages
git commit -m "docs(wiki): add Conventions reference page (frames, angle spaces, translateToServo)"
```

---

### Task 3: `reference/api.md` (new top-level page)

**Files:**
- Create: `wiki/reference/api.md`
- Modify: `wiki/reference/.pages`

- [ ] **Step 1: Write the page**

Port `proposed-documentation/api-reference.md` into `wiki/reference/api.md`, restyled to concise prose. Keep as tables: the per-command payload/effect tables (T:1-T:7) and the T:10 telemetry field table. **Strip every `Verify:` block.** **Omit the "Frontend Integration Summary" section** - it goes to `remote-control/index.md` in Task 7. Keep the connection example and the `dir`-not-`d` / corrected gait-ID notes (they are the accurate values). H1: `# WebSocket API`.

- [ ] **Step 2: Add to nav**

Edit `wiki/reference/.pages`:

```yaml
title: Reference
nav:
  - conventions.md
  - api.md
  - firmware
  - animation
  - simulation
  - remote-control
```

- [ ] **Step 3: Build strict**

Run: `.venv-docs/bin/mkdocs build --strict 2>&1 | tail -5`
Expected: built, no warnings/errors. (`reference/firmware/api.md` stub still exists and is still navved at this point - that is fine; it is removed in Task 4.)

- [ ] **Step 4: Commit**

```bash
git add wiki/reference/api.md wiki/reference/.pages
git commit -m "docs(wiki): add top-level WebSocket API reference page"
```

---

### Task 4: Re-cut firmware reference into concept pages

**Files:**
- Rewrite: `wiki/reference/firmware/index.md`
- Create: `wiki/reference/firmware/motion-engine.md`, `wiki/reference/firmware/fsm-states.md`
- Delete: `wiki/reference/firmware/servo-conventions.md`, `wiki/reference/firmware/api.md`
- Modify: `wiki/reference/firmware/.pages`
- Modify (links): `wiki/guide/wiring.md`, `wiki/guide/design.md`, `wiki/reference/remote-control/index.md`, `wiki/reference/remote-control/websocket-api.md`

- [ ] **Step 1: Rewrite the firmware index**

Rewrite `wiki/reference/firmware/index.md` as the firmware architecture overview: the brain / nervous_system / shared layering and the "80% hardcoded pose / 20% calibration" philosophy (source: `wiki/reference/firmware/_context/readme-nervous-system.md`), plus the main-loop overview from `proposed-documentation/firmware-main-loop.md` sections 1-2 (single-threaded event loop, execution order, deadman switch, input smoothing). Keep the loop Mermaid diagram. Concise prose. Link onward to `motion-engine.md` and `fsm-states.md`.

- [ ] **Step 2: Write motion-engine.md**

Create `wiki/reference/firmware/motion-engine.md` from `proposed-documentation/firmware-main-loop.md` sections 3-5 (gait mechanics: phase/stance-swing, per-leg angle computation, CRAB/TROT notes, graceful stop, yaw rotation, gait profiles; and the clip player: interpolation, return-to-stand, invertRobot/wall-flip). Keep the gait-profile table, the NEUTRAL table, and the Mermaid pipeline/architecture diagrams. For `translateToServo()`, summarize and link to `../conventions.md` (the formulas live there) rather than duplicating them. H1: `# Motion engine (gaits & clips)`.

- [ ] **Step 3: Write fsm-states.md**

Create `wiki/reference/firmware/fsm-states.md` from `proposed-documentation/firmware-states.md`: the five states (IDLE/WALK/ACTION/REST/FAILSAFE), what enters/exits each, and the signal pipelines. Keep the FSM Mermaid state diagram and the clip/gait pipeline Mermaid diagrams. Keep the timing-constants and gait-params tables. **Strip every `Verify:` block.** H1: `# FSM states`.

- [ ] **Step 4: Delete the superseded stubs**

```bash
cd /Users/marcushamelink/Documents/EPFL-projects/projects-ba6/Making-Intelligent-Things/FaceHuggerWorktrees/wiki-setup
git rm wiki/reference/firmware/servo-conventions.md wiki/reference/firmware/api.md
```

- [ ] **Step 5: Set firmware nav**

Overwrite `wiki/reference/firmware/.pages`:

```yaml
title: Firmware
nav:
  - index.md
  - motion-engine.md
  - fsm-states.md
  - kinematics.md
  - csv-diagnostics.md
```

- [ ] **Step 6: Fix links broken by the deletions**

Edit these four lines (servo-conventions -> conventions page; firmware/api -> top-level api):

- `wiki/guide/wiring.md:27` `[servo conventions](../reference/firmware/servo-conventions.md)` -> `[servo conventions](../reference/conventions.md)`
- `wiki/guide/design.md:24` `[servo conventions](../reference/firmware/servo-conventions.md)` -> `[servo conventions](../reference/conventions.md)`
- `wiki/reference/remote-control/index.md:21` `[Firmware -> API](../firmware/api.md)` -> `[API](../api.md)`
- `wiki/reference/remote-control/websocket-api.md:6` `[Firmware -> API](../firmware/api.md)` -> `[API](../api.md)`

Then confirm no other references remain:

Run: `grep -rn "firmware/api.md\|firmware/servo-conventions.md" wiki --include="*.md" | grep -v _context`
Expected: no output.

- [ ] **Step 7: Build strict**

Run: `.venv-docs/bin/mkdocs build --strict 2>&1 | tail -5`
Expected: built, no warnings/errors.

- [ ] **Step 8: Commit**

```bash
git add -A wiki/reference/firmware wiki/reference/.pages wiki/guide/wiring.md wiki/guide/design.md wiki/reference/remote-control
git commit -m "docs(wiki): re-cut firmware reference into index/motion-engine/fsm-states; drop servo-conventions+api stubs"
```

---

### Task 5: `reference/animation/clip-panel.md`

**Files:**
- Create: `wiki/reference/animation/clip-panel.md`
- Modify: `wiki/reference/animation/.pages`

- [ ] **Step 1: Write the page**

Port `proposed-documentation/blender-addon.md` into `wiki/reference/animation/clip-panel.md` as concise prose describing the FH Clip Panel add-on features (clip management, pose library, selection sets, export formats, activity heatmap, Bezier/Linear toggle, authoring warnings, headless export). **Strip every `Verify:` block and the entire "Verification Checklist - Integration Tests" section.** Keep the "Limitations & Known Issues" as a short list. Fix the "Related Documents" links to point at wiki pages where they exist (e.g. `../../guide/toolchain/blender-clips.md` for how-to, `fhc-format.md` for the Phase-2 design) or drop links to repo-internal paths. H1: `# Blender clip panel (FH Clip Panel)`.

- [ ] **Step 2: Add to nav**

Edit `wiki/reference/animation/.pages` to insert `clip-panel.md` after `index.md`:

```yaml
title: Animation pipeline
nav:
  - index.md
  - clip-panel.md
  - blender-rig.md
  - urdf-pipeline.md
  - fhc-format.md
  - gait-design.md
```

- [ ] **Step 3: Build strict**

Run: `.venv-docs/bin/mkdocs build --strict 2>&1 | tail -5`
Expected: built, no warnings/errors.

- [ ] **Step 4: Commit**

```bash
git add wiki/reference/animation/clip-panel.md wiki/reference/animation/.pages
git commit -m "docs(wiki): add Blender clip-panel reference page"
```

---

### Task 6: `reference/roadmap.md` (new top-level page)

**Files:**
- Create: `wiki/reference/roadmap.md`
- Modify: `wiki/reference/.pages`

- [ ] **Step 1: Write the page**

Port `proposed-documentation/future-considerations.md` into `wiki/reference/roadmap.md`. It is already prose-ish - light restyle, keep the "next steps" and "design documents" tables. Keep the project-state summary, the `.fhc` foot-space section, runtime adaptability, mobile-app clips UI, servo-numbering alignment, and the rough-edges/quick-wins. Drop the closing signature line. Fix internal doc references to wiki links where a wiki page exists (e.g. `fhc-format.md`, `conventions.md`, `api.md`), otherwise refer to the repo path in prose without a markdown link. H1: `# Roadmap & future work`.

- [ ] **Step 2: Add to nav**

Edit `wiki/reference/.pages` to append `roadmap.md` last:

```yaml
title: Reference
nav:
  - conventions.md
  - api.md
  - firmware
  - animation
  - simulation
  - remote-control
  - roadmap.md
```

- [ ] **Step 3: Build strict**

Run: `.venv-docs/bin/mkdocs build --strict 2>&1 | tail -5`
Expected: built, no warnings/errors.

- [ ] **Step 4: Commit**

```bash
git add wiki/reference/roadmap.md wiki/reference/.pages
git commit -m "docs(wiki): add Roadmap & future-work reference page"
```

---

### Task 7: Remote-control frontend summary

**Files:**
- Rewrite: `wiki/reference/remote-control/index.md`

- [ ] **Step 1: Enrich the page**

Rewrite `wiki/reference/remote-control/index.md` to incorporate the "Frontend Integration Summary" from `proposed-documentation/api-reference.md` (the WebSocket service, Zustand store fields, API helpers, screens, hooks, pager-to-FSM mapping), as concise prose. Keep the existing "What it does / Tech stack / Connection model" framing. Link to `../api.md` for the protocol (the `../firmware/api.md` link was already repointed in Task 4). Keep it shorter than the source - this is an overview, not the full app source map.

- [ ] **Step 2: Build strict**

Run: `.venv-docs/bin/mkdocs build --strict 2>&1 | tail -5`
Expected: built, no warnings/errors.

- [ ] **Step 3: Commit**

```bash
git add wiki/reference/remote-control/index.md
git commit -m "docs(wiki): flesh out remote-control overview with app integration summary"
```

---

### Task 8: Build-guide Toolchain section

**Files:**
- Create: `wiki/guide/toolchain/.pages`, `wiki/guide/toolchain/index.md`, `wiki/guide/toolchain/flashing.md`, `wiki/guide/toolchain/fusion-export.md`, `wiki/guide/toolchain/blender-clips.md`
- Modify: `wiki/guide/.pages`
- Rewrite: `wiki/guide/software.md`

- [ ] **Step 1: Create the toolchain nav**

Create `wiki/guide/toolchain/.pages`:

```yaml
title: Toolchain
nav:
  - index.md
  - flashing.md
  - fusion-export.md
  - blender-clips.md
```

- [ ] **Step 2: Write toolchain/index.md**

Create `wiki/guide/toolchain/index.md`: a short prose overview of the pipeline (Fusion CAD -> URDF export -> Blender rig + clip authoring -> exported clips -> firmware flash), one short paragraph per stage, each linking to the relevant page (`flashing.md`, `fusion-export.md`, `blender-clips.md`, and the reference pages for the "why"). H1: `# Toolchain`.

- [ ] **Step 3: Write flashing.md**

Create `wiki/guide/toolchain/flashing.md`: how to flash the ESP32 with PlatformIO and connect. Use the commands from `CLAUDE.md` and `proposed-documentation/firmware-states.md` "Testing Workflow":

```bash
cd code/firmware
pio run -e upesy_wroom      # build
pio run -t upload           # flash
pio device monitor          # 115200 baud
```

Then: join Wi-Fi `FaceHugger_Net` (password `12345678`), WebSocket at `ws://192.168.4.1:81`. Link to `../../reference/api.md` for the command protocol. Concise prose. H1: `# Flashing the firmware`.

- [ ] **Step 4: Write fusion-export.md**

Create `wiki/guide/toolchain/fusion-export.md`: how to install and run the two Fusion 360 add-ins, `cad/scripts/ExportBodiesToURDF/` (produces `code/simulation/generated/fusion_export.json` + meshes consumed by `generate_urdf.py`) and `cad/scripts/ExportPrintableSTLs/` (print-ready STLs). Cover: where the add-ins live, how to load a script/add-in in Fusion 360 (Utilities -> Add-Ins -> Scripts and Add-Ins), and what each emits. Source detail: `CLAUDE.md` repo-layout section and `code/simulation/README.md` (in `wiki/reference/simulation/_context/readme-simulation.md`). Note these run inside Fusion, not the host Python env. H1: `# Fusion 360 export add-ins`.

- [ ] **Step 5: Write blender-clips.md**

Create `wiki/guide/toolchain/blender-clips.md`: how to install Blender 5.x, open the rig (`animation/fh_rigged_latest.blend`), enable/use the FH Clip Panel (the `N`-panel), author a clip, and export (GUI + the headless `export_all_clips.py` command). Pull the install/run specifics from `proposed-documentation/blender-addon.md` (Prerequisites + Headless Export sections) and the `BLENDER_BIN` note from `CLAUDE.md`. Keep the headless command block. Link to `../../reference/animation/clip-panel.md` for the full feature reference. H1: `# Blender clip authoring & export`.

- [ ] **Step 6: Slim software.md and add toolchain to guide nav**

Rewrite `wiki/guide/software.md` so its "Setup" and "Running" sections become a short pointer to the new Toolchain pages (keep the conceptual "Overview" and "How it works" sections). Then edit `wiki/guide/.pages` to add `toolchain` after `software.md`:

```yaml
title: Build Guide
nav:
  - design.md
  - parts.md
  - printing.md
  - wiring.md
  - assembly.md
  - software.md
  - toolchain
```

- [ ] **Step 7: Build strict**

Run: `.venv-docs/bin/mkdocs build --strict 2>&1 | tail -5`
Expected: built, no warnings/errors.

- [ ] **Step 8: Commit**

```bash
git add wiki/guide/toolchain wiki/guide/.pages wiki/guide/software.md
git commit -m "docs(wiki): add build-guide Toolchain section (flashing, Fusion export, Blender clips)"
```

---

### Task 9: Final verification

**Files:** none (verification only)

- [ ] **Step 1: Strict build is clean**

Run: `.venv-docs/bin/mkdocs build --strict 2>&1 | grep -iE "warning|error|not found|anchor" | grep -vE "Material for MkDocs|plugin system|theme overrides|migration path|contribution model|unlicensed|backward-incompat|MkDocs 2.0" || echo "clean"`
Expected: `clean`.

- [ ] **Step 2: New pages built, stubs gone**

Run: `ls site/reference/conventions/ site/reference/api/ site/reference/roadmap/ site/reference/firmware/motion-engine/ site/reference/firmware/fsm-states/ site/reference/animation/clip-panel/ site/guide/toolchain/ >/dev/null 2>&1 && echo "new pages OK"; ls site/reference/firmware/servo-conventions 2>/dev/null && echo "STALE servo-conventions still built" || echo "servo-conventions gone"`
Expected: `new pages OK` and `servo-conventions gone`.

- [ ] **Step 3: `_context/` and checklists not in the site**

Run: `find site -path "*_context*" -o -name "verification-checklists*" | head; echo "---"; test -f proposed-documentation/verification-checklists.md && echo "checklist file present in repo"`
Expected: first command empty; `checklist file present in repo`.

- [ ] **Step 4: Serve sanity (manual)**

Run: `.venv-docs/bin/mkdocs serve` and open http://127.0.0.1:8000 — confirm the Reference tab shows Conventions, WebSocket API, Firmware (Overview / Motion engine / FSM states / Kinematics / CSV diagnostics), Animation (with Clip panel), Simulation, Remote control, Roadmap; and Build Guide shows the Toolchain section. Stop the server when done.

- [ ] **Step 5: No commit needed** (verification only). If any check failed, return to the relevant task and fix.

---

## Notes for the executor

- Run all commands from the worktree root: `/Users/marcushamelink/Documents/EPFL-projects/projects-ba6/Making-Intelligent-Things/FaceHuggerWorktrees/wiki-setup`.
- The build venv already exists at `.venv-docs/`. If missing: `python -m venv .venv-docs && .venv-docs/bin/pip install -r requirements-docs.txt`.
- Do **not** push (a git hook blocks `git push`); the user pushes manually.
- Tier-3 stubs (firmware kinematics/csv-diagnostics, animation blender-rig/urdf-pipeline/fhc-format/gait-design, simulation torque pages) are intentionally left as-is. Do not author them.
- Anchor links into `conventions.md` are avoided in favor of page-level links to stay robust against heading edits; if you do use a `#anchor`, verify it against the heading you wrote (strict build validates anchors).
```
