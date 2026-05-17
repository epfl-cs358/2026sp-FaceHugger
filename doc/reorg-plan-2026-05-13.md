# Reorganization plan — code/simulation, animation/, doc/

> **Status:** drafted 2026-05-13, deferred for later execution. Reflects the
> repo state on that date. Re-check the file inventory before executing if
> significant time has passed.

## Context

The repo grew organically across CAD, sim, firmware, and animation tracks, and the directory shape no longer matches the conceptual pipeline. `code/simulation/` flatly mixes the Fusion → URDF generator with the PyBullet sim and the CLI dispatcher; `animation/scripts/` mixes pipeline scene builders (called by `facehugger.py`) with a Blender add-on and a one-shot migration tool. Two broken stubs (`teleop.py`, `terrain.py`) and a fully-superseded design folder (`doc/gait-design/`) still take up space. The wiki to be written needs a directory layout that the section headings can reference cleanly.

This plan also pins down how to use the two new artifacts (`fh_clip_panel.py`, `fh_rename_actions.py`) since their integration with the rest of the workflow isn't yet documented.

Intended outcome: a flatter conceptual story (URDF pipeline / sim / animation pipeline / animation tooling), zero dead files, and a `doc/README.md` wiki outline that anchors each chapter to existing authoritative spec docs so narrative copy can be written without searching.

---

## Decisions (locked)

1. **`code/simulation/` → subfolder split.** `urdf_pipeline/` + `sim/`. No `pyproject.toml`.
2. **`animation/` → three-way split.** `pipeline/` + `addons/` + `migrations/`.
3. **`doc/gait-design/` → delete outright.** Superseded by `doc/animation-pipeline/`. git history is the archive.
4. **`view_urdf.py` + `facehugger.py view` subcommand → remove.** Redundant with `facehugger.py sim`.

---

## Target layout

```
code/simulation/
  facehugger.py              CLI dispatcher (unchanged user-facing interface)
  facehugger_config.yaml     user-facing config — stays at top
  requirements.txt
  README.md                  update CLI section to drop `view`, fix paths
  generated/                 unchanged
  docs/                      unchanged
  urdf_pipeline/
    __init__.py
    generate_urdf.py
  sim/
    __init__.py
    simulate.py
    kinematics.py
    gaits.py
    helpers.py
    constants.py

animation/
  README.md                  new — top-level overview of the three kinds
  fh_rigged_latest.blend     unchanged
  blend-iterations/          unchanged (gitignored)
  SERVO_ID_CONVENTION.md     unchanged
  pipeline/
    README.md                moved from animation/scripts/README.md, trimmed to scene-builder content
    urdf_to_blender_rigged.py
    visualize_urdf.py
    visualize_fusion_export.py
  addons/
    README.md                new — install paths + panel walkthrough
    fh_clip_panel.py
  migrations/
    README.md                new — when to run, what it changes
    fh_rename_actions.py

doc/
  README.md                  rewritten as wiki outline + links to authoritative spec docs
  animation-pipeline/        unchanged (new authority)
  torque-analysis/           unchanged
  (doc/gait-design/ deleted)
```

---

## Execution steps (ordered — earlier steps unblock later ones)

### Step 1 — delete dead files (zero-risk, do first)

```
git rm code/simulation/teleop.py
git rm code/simulation/terrain.py
git rm code/simulation/terrain_report.md
git rm code/simulation/view_urdf.py
git rm -r doc/gait-design/
```

Also update `code/simulation/docs/MERGE_AND_CONVENTION.md §8` to drop the "teleop / terrain incomplete port" TODO (no longer needed — the stubs are gone).

### Step 2 — strip the `view` subcommand from `facehugger.py`

In `code/simulation/facehugger.py`:
- Remove the `VIEW_URDF` constant.
- Remove `cmd_view`.
- Remove the `pv = sub.add_parser("view", ...)` block.
- Update the module docstring example list and `Subcommands:` section.

### Step 3 — move `code/simulation/` into subfolders

```
mkdir code/simulation/urdf_pipeline code/simulation/sim
git mv code/simulation/generate_urdf.py code/simulation/urdf_pipeline/generate_urdf.py
git mv code/simulation/simulate.py     code/simulation/sim/simulate.py
git mv code/simulation/kinematics.py   code/simulation/sim/kinematics.py
git mv code/simulation/gaits.py        code/simulation/sim/gaits.py
git mv code/simulation/helpers.py      code/simulation/sim/helpers.py
git mv code/simulation/constants.py    code/simulation/sim/constants.py
touch code/simulation/urdf_pipeline/__init__.py code/simulation/sim/__init__.py
```

Then fix path math and imports:

- **`sim/constants.py`** — any `SCRIPT_DIR = Path(__file__).parent` followed by `SCRIPT_DIR / "generated"` needs to walk one level higher. Change to `SCRIPT_DIR = Path(__file__).resolve().parent.parent` (so `SCRIPT_DIR` still resolves to `code/simulation/`). Verify all path constants (`CONFIG_YAML`, `FUSION_JSON`, `MESH_DIR`, `URDF_PATH`) point at `code/simulation/{facehugger_config.yaml, generated/...}`.
- **`urdf_pipeline/generate_urdf.py`** — same fix: `SCRIPT_DIR = Path(__file__).resolve().parent.parent` so `DEFAULT_JSON`, `DEFAULT_CFG`, `DEFAULT_OUT` resolve correctly.
- **`sim/simulate.py`** — `from gaits import ...` → `from .gaits import ...`; `from kinematics import build_config` → `from .kinematics import build_config`.
- **`sim/kinematics.py`** — `from constants import ...` → `from .constants import ...`; `from helpers import ...` → `from .helpers import ...`.
- **`sim/gaits.py`** — `from constants import TIMESTEP` → `from .constants import TIMESTEP`; `from helpers import ...` → `from .helpers import ...`.
- **`code/simulation/facehugger.py`** — update path constants:
  - `GENERATE_URDF = HERE / "urdf_pipeline" / "generate_urdf.py"`
  - `SIMULATE = HERE / "sim" / "simulate.py"`
- **`code/firmware/test/gen_ik_reference.py`** — `SIM_DIR = Path(__file__).resolve().parent.parent.parent / "simulation"` → `... / "simulation" / "sim"`. Confirm `from kinematics import NEUTRAL_FOOT, leg_ik` still resolves.

When invoking moved modules via subprocess, `simulate.py` and `generate_urdf.py` are still run as scripts (`python <path>`), so the `from .x import` relative imports require launching them as modules instead. Recommended: switch `facehugger.py` to `[sys.executable, "-m", "sim.simulate", ...]` invocation, set `cwd=HERE`. Cleaner long-term, no shim.

```python
def cmd_sim(args):
    cli = [sys.executable, "-m", "sim.simulate", ...]
def cmd_urdf(args):
    cli = [sys.executable, "-m", "urdf_pipeline.generate_urdf", ...]
```

### Step 4 — move `animation/scripts/` into subfolders

```
mkdir animation/pipeline animation/addons animation/migrations
git mv animation/scripts/urdf_to_blender_rigged.py    animation/pipeline/
git mv animation/scripts/visualize_urdf.py            animation/pipeline/
git mv animation/scripts/visualize_fusion_export.py   animation/pipeline/
git mv animation/scripts/fh_clip_panel.py             animation/addons/
git mv animation/scripts/fh_rename_actions.py         animation/migrations/
git mv animation/scripts/README.md                    animation/pipeline/README.md
```

Then:
- **`animation/pipeline/README.md`** — trim to scene-builder content only (drop the addon + migration sections that were just added).
- **`animation/addons/README.md`** — new file with the `fh_clip_panel.py` section copied from the old README (install paths, panel UI, operator IDs).
- **`animation/migrations/README.md`** — new file with the `fh_rename_actions.py` section (rename map, slot-identifier verification, CLI invocation).
- **`animation/README.md`** — new top-level: one-paragraph overview + table linking to each subfolder.
- **`animation/migrations/fh_rename_actions.py`** — fix `REPO_ROOT = HERE.parent.parent` → `HERE.parent.parent.parent` (one more level deep now).
- **`code/simulation/facehugger.py`** — update VISUALIZE / VISUALIZE_RIGGED paths:
  - `VISUALIZE = REPO_ROOT / "animation" / "pipeline" / "visualize_urdf.py"`
  - `VISUALIZE_RIGGED = REPO_ROOT / "animation" / "pipeline" / "urdf_to_blender_rigged.py"`
- **`code/simulation/README.md`** — fix path links in the `blender` subcommand section (now point at `animation/pipeline/...`).
- **`CLAUDE.md`** (uncommitted) — sweep paths: `code/simulation/{generate_urdf,simulate,kinematics,gaits,helpers,constants}.py` → `sim/...` or `urdf_pipeline/...`; `animation/scripts/*` → `animation/{pipeline,addons,migrations}/*`.

### Step 5 — rewrite `doc/README.md` as wiki outline

Replace the current minimal index with a section-by-section outline whose body is empty (one or two TODO-line per section) and a "see also" link to the authoritative spec. This gives a place to write narrative copy without inventing structure or searching for what already exists.

Proposed section order (prose to be filled in by the author):

```markdown
# FaceHugger — project wiki

## 1. CAD (Fusion 360)
   → see also: cad/scripts/ExportBodiesToURDF/, code/simulation/docs/PIPELINE_SPEC.md
   TODO: how the Fusion assembly is structured; the ExportBodiesToURDF add-in.

## 2. URDF generation
   → see also: code/simulation/docs/URDF_PIPELINE.md, code/simulation/docs/MERGE_AND_CONVENTION.md
   TODO: how generate_urdf.py consumes the Fusion JSON + mesh manifest.

## 3. PyBullet simulation
   → see also: code/simulation/docs/SIM_PIPELINE.md, code/simulation/README.md
   TODO: facehugger.py sim / --walk / --trot / --headless.

## 4. Blender pipeline
   → see also: animation/pipeline/README.md, code/simulation/docs/API_ANIMATION_SPEC.md
   TODO: rigged vs placement-only; visualize_fusion_export.py for CAD cross-check.

## 5. Animation authoring (clip panel)
   → see also: animation/addons/README.md
   TODO: clip naming convention, layered Action API, duplicate/rename workflow.

## 6. Firmware playback
   → see also: code/firmware/src/nervous_system/README.md, doc/animation-pipeline/leg-coordinates.md
   TODO: ESP32 80/20 hardcode-pose philosophy; upcoming .fhc on-board engine.

## 7. Mobile remote control
   → see also: code/API_SPEC.md, code/remote-control-app/MyApp/

## 8. Conventions reference
   → see also: code/simulation/docs/MERGE_AND_CONVENTION.md, animation/SERVO_ID_CONVENTION.md
```

The author writes the connective tissue; the executor's edits stop at this skeleton.

### Step 6 — verification (must all pass before commit)

```bash
conda activate facehugger

# Pipeline still works end-to-end
python code/simulation/facehugger.py urdf
python code/simulation/facehugger.py sim --headless
python code/simulation/facehugger.py sim --walk          # GUI smoke; close after ~5s
python code/simulation/facehugger.py blender --rigged --headless --save /tmp/fh.blend
test -s /tmp/fh.blend && echo "blender export OK"

# Firmware test still resolves sim.kinematics
cd code/firmware
python test/gen_ik_reference.py
pio test -e native -f test_kinematics

# Migration script still resolves its blend path
blender --background --python animation/migrations/fh_rename_actions.py   # should report no-op (already renamed)

# Manual: load animation/fh_rigged_latest.blend → Text Editor → run
# animation/addons/fh_clip_panel.py → press N → "FaceHugger" tab visible
```

### Step 7 — commit strategy

Split into four commits so reverts are surgical:

1. `chore: delete stale stubs (teleop, terrain, view_urdf) + superseded doc/gait-design`
2. `refactor(sim): split code/simulation into urdf_pipeline/ and sim/ subpackages`
3. `refactor(animation): split scripts into pipeline/ addons/ migrations/`
4. `docs: rewrite doc/README.md as wiki outline + update CLAUDE.md / READMEs for new paths`

---

## Critical files to be modified

| Path | Change |
|---|---|
| `code/simulation/facehugger.py` | Drop `view` subcommand; update script paths; switch to `python -m` invocation |
| `code/simulation/sim/constants.py` (moved) | `SCRIPT_DIR` walk +1 |
| `code/simulation/urdf_pipeline/generate_urdf.py` (moved) | `SCRIPT_DIR` walk +1 |
| `code/simulation/sim/{simulate,kinematics,gaits}.py` (moved) | Imports → relative |
| `code/firmware/test/gen_ik_reference.py` | `SIM_DIR` walk +1 to land in `sim/` |
| `animation/migrations/fh_rename_actions.py` (moved) | `REPO_ROOT` walk +1 |
| `animation/pipeline/README.md` (moved + trimmed) | Drop addon/migration sections |
| `animation/{addons,migrations}/README.md` (new) | Extracted from old README |
| `animation/README.md` (new) | Top-level index |
| `code/simulation/README.md` | Drop `view` block; update animation script paths |
| `CLAUDE.md` (uncommitted, will commit with reorg) | Path sweep |
| `doc/README.md` | Rewrite as wiki outline |
| `code/simulation/docs/MERGE_AND_CONVENTION.md` | Drop §8 teleop/terrain TODO |

## Files reused (no change, but worth noting)

- `code/simulation/docs/{PIPELINE_SPEC,URDF_PIPELINE,SIM_PIPELINE,MERGE_AND_CONVENTION,API_ANIMATION_SPEC}.md` — already authoritative; the wiki outline links to them rather than duplicating.
- `code/simulation/sim/kinematics.py` (after move) — still the firmware IK reference source via `gen_ik_reference.py`.
- `animation/SERVO_ID_CONVENTION.md` — already the bridge spec between URDF and firmware servo numbering.

## How to use the new clip panel + migration (documentation source)

This content lands in `animation/addons/README.md` and `animation/migrations/README.md` after the move; included here so the integration is documented in the plan.

**Migration (one-shot):**
```bash
blender --background --python animation/migrations/fh_rename_actions.py
```
Renames the 5 legacy Actions on `animation/fh_rigged_latest.blend` to `base_anim__<target>` and saves in place. Idempotent.

**Clip panel (dev iteration):**
1. Open `animation/fh_rigged_latest.blend`.
2. Text Editor > Open > `animation/addons/fh_clip_panel.py` > Alt+P.
3. Press N in the 3D viewport, switch to the "FaceHugger" tab.

**Clip panel (persistent install):**
Edit > Preferences > Add-ons > Install... → pick `animation/addons/fh_clip_panel.py` → enable "FH Clip Panel".

**Authoring flow:**
- Apply `base_anim` (the migrated legacy clip).
- Type a new clip name in the panel, click **Duplicate Active Clip** — gets 5 fresh Actions named `<new>__<target>`, applied to the rig.
- Edit the new clip in pose mode / dope sheet without touching `base_anim`.
- Switch between clips with the per-clip buttons.

## Verification recap

The verification block in Step 6 is the contract. If any of those commands fails after the reorg, roll back the offending commit. Manual Blender check is in the same block (open .blend, run addon, see panel) — there's no automated way to verify a GUI-only Blender addon short of a screenshot test, which is out of scope.