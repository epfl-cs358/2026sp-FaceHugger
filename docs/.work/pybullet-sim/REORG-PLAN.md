# `code/simulation/` reorganization — migration plan

**Status:** ✅ EXECUTED (2026-05-26, commit `e787788`) via TDD — a subprocess
regression net (`test_pipeline_regression.py`) stayed green across the move and
4 structure tests (`test_package_structure.py`) flipped red→green. 70 tests pass
under conda env `facehugger`. The plan below is kept as the record of what was
done; one deviation: it shipped as a single atomic commit (the cross-file import
rewrites don't bisect cleanly) rather than the 5-commit sequence in §6.

**Goal:** split the flat `code/simulation/` directory into two clear concerns —
**URDF generation** (the build step that *produces* the URDF from CAD) and the
**PyBullet runtime** (everything that *consumes* the URDF) — without changing any
behavior.

**Non-goals:** no logic changes, no new features, no gait/IK/convention changes.
Pure move + import/path rewiring. The CLI surface (`facehugger.py <subcommand>`)
stays identical for users.

---

## 1. Target structure

```
code/simulation/
  facehugger.py            # stays — the single CLI entry point
  facehugger_config.yaml   # stays — read by BOTH urdf_gen and the runtime
  requirements.txt         # stays
  README.md                # stays (update paths)
  generated/               # stays — derived artifacts (urdf, meshes, json)
  docs/                    # stays

  pybullet_sim/            # the runtime  ⚠ underscore, NOT hyphen (Python pkg)
    __init__.py
    simulate.py
    gaits.py
    kinematics.py
    helpers.py
    constants.py
    sim_monitor.py
    test_sim_monitor.py
    interpreter/           # today's pybullet_interpreter/ moves here
      __init__.py
      clip_loader.py
      clip_player.py
      servo_convention.py
      gait_interpreter.py
      tests/

  urdf_gen/
    __init__.py
    generate_urdf.py
    verify_export_parity.py    # export↔sim parity; imports the interpreter
```

> **Naming caveat.** The original idea was a folder called `pybullet-sim/`.
> A hyphen is illegal in a Python module/package name, so it must be
> `pybullet_sim/` (underscore) for `import`/`-m` to work.

---

## 2. Key decision: package-relative imports (Approach A)

Today the modules use **flat, cwd-based imports** (`from gaits import …`,
`import sim_monitor`) that only resolve because `simulate.py` is run with the
working directory set to `code/simulation/`. Two ways to reorganize:

- **Approach A (recommended):** make `pybullet_sim` a real package with
  `__init__.py`, convert sibling imports to package-relative
  (`from .gaits import …`), and run entry points as modules
  (`python -m pybullet_sim.simulate`). Correct and robust; more edits.
- **Approach B (rejected):** move files into folders but keep flat imports and
  set `cwd`/`sys.path` per folder. Minimal import edits but keeps the fragile
  cwd dependency and `sys.path` hacking. Not worth carrying forward.

This plan follows **Approach A**.

---

## 3. File-by-file changes

### 3a. Moves (`git mv`, preserves history)

| From | To |
|---|---|
| `simulate.py gaits.py kinematics.py helpers.py constants.py sim_monitor.py test_sim_monitor.py` | `pybullet_sim/` |
| `pybullet_interpreter/` (whole dir) | `pybullet_sim/interpreter/` |
| `generate_urdf.py verify_export_parity.py` | `urdf_gen/` |

Add empty `pybullet_sim/__init__.py` and `urdf_gen/__init__.py`. (The interpreter
already has `__init__.py`.)

### 3b. Import rewrites (Approach A)

| File | Current | New |
|---|---|---|
| `pybullet_sim/kinematics.py` | `from constants import …` / `from helpers import …` | `from .constants import …` / `from .helpers import …` |
| `pybullet_sim/gaits.py` | `from constants import TIMESTEP` / `from helpers import …` / `import sim_monitor` | `from .constants import TIMESTEP` / `from .helpers import …` / `from . import sim_monitor` |
| `pybullet_sim/simulate.py` | `from gaits import …` / `from kinematics import …` | `from .gaits import …` / `from .kinematics import …` |
| `pybullet_sim/gaits.py` (clip import) | `from pybullet_interpreter.clip_loader import …` | `from .interpreter.clip_loader import …` |
| `pybullet_sim/test_sim_monitor.py` | `import sim_monitor as m` (+ `sys.path.insert`) | `from pybullet_sim import sim_monitor as m`; drop the `sys.path.insert` |
| `urdf_gen/verify_export_parity.py` | `from pybullet_interpreter.X import …` (+ `sys.path.insert(parent)`) | `from pybullet_sim.interpreter.X import …`; set `sys.path` to the `simulation/` root, not the file's own dir |

`generate_urdf.py` has **no sibling-module imports** (self-contained) — only its
`__file__`-relative paths change (§3c).

### 3c. `__file__`-relative path fixes (the subtle, breakage-prone part)

| File | Symbol | Today | After move | Fix |
|---|---|---|---|---|
| `constants.py` | `HERE`, `GENERATED_DIR`, `CONFIG_YAML` | `HERE = dirname(__file__)`; `generated/` & yaml are siblings | now one level *below* the files (in `pybullet_sim/`) | point at the parent: `SIM_ROOT = os.path.dirname(HERE)`; resolve `generated/`, `facehugger_config.yaml` against `SIM_ROOT` |
| `interpreter/clip_loader.py` | `_REPO_ROOT` | `Path(__file__).resolve().parents[3]` | moved one level deeper (`…/pybullet_sim/interpreter/…`) | `parents[3]` → `parents[4]` |
| `generate_urdf.py` | `SCRIPT_DIR` | `Path(__file__).parent` (= `simulation/`) | now `urdf_gen/`; `generated/` & yaml are one level up | resolve outputs against `SCRIPT_DIR.parent` |
| `facehugger.py` | `GENERATE_URDF`, `SIMULATE` | `HERE / "generate_urdf.py"`, `HERE / "simulate.py"` | files moved into subfolders | invoke as modules instead (§3d) |

> **These four are the highest-risk edits.** A wrong `parents[N]` or a stale
> sibling-path assumption fails *silently* (wrong file found or `FileNotFound`),
> not at import time. Verify each by running the smoke checks in §5.

### 3d. Entry-point invocation (`facehugger.py`)

`facehugger.py` currently shells out with `subprocess.run([python, SIMULATE], cwd=HERE)`.
After the move, run the package modules instead, with `cwd` = `code/simulation/`
(so `pybullet_sim` / `urdf_gen` are importable):

- `sim`  → `python -m pybullet_sim.simulate …`, `cwd=HERE`
- `urdf` → `python -m urdf_gen.generate_urdf …`, `cwd=HERE`

(`HERE` is still `code/simulation/`, the package parent — no change to `HERE`.)
The `view` subcommand is already removed.

---

## 4. Documentation references to update

Grep the repo for the old flat paths and fix prose/commands:

- **`CLAUDE.md`** (root) — the pipeline diagram and "Common commands" reference
  `code/simulation/generate_urdf.py`, `simulate.py`, `view_urdf.py`,
  `teleop.py`/`terrain.py` (now deleted), and `pybullet_interpreter`.
- **`code/simulation/README.md`** — command walkthrough + troubleshooting table.
- **`code/simulation/docs/*.md`** — `SIM_PIPELINE.md`, `URDF_PIPELINE.md`,
  `MERGE_AND_CONVENTION.md` reference module paths.
- **`docs/.work/pybullet-sim/REPORT.md`** (this report) — the file map and all
  `file:line` references shift; regenerate or note the move.
- **`code/firmware/test/gen_clip_parity_reference.py`** and any tooling that
  imports `animation/addons/fh_clip_panel.py` — check it doesn't reach into
  `code/simulation/pybullet_interpreter` by path.

---

## 5. Verification checklist (run after the move, before committing)

```bash
cd code/simulation
python -m py_compile pybullet_sim/*.py pybullet_sim/interpreter/*.py urdf_gen/*.py facehugger.py
python -m pytest pybullet_sim/test_sim_monitor.py pybullet_sim/interpreter/tests/ -q
python -m pytest urdf_gen/ -q                      # if any parity tests live there
python -m urdf_gen.generate_urdf                   # regenerates generated/facehugger.urdf — diff must be empty
python facehugger.py urdf                          # same, via the CLI wrapper
python facehugger.py sim --headless                # smoke: connects, loads, steps, exits 0
python facehugger.py sim --headless --log          # smoke: writes sim_log.csv (+ png if matplotlib)
python -m pybullet_sim.simulate --headless --clip "wave"   # interpreter path resolves clips_all.h (parents[4])
python urdf_gen/verify_export_parity.py            # parity still passes (interpreter import + clips path)
```

The **must-not-change** signals: `generated/facehugger.urdf` is byte-identical
after regeneration, all tests green, and `verify_export_parity.py` exits 0.

---

## 6. Suggested commit sequence

Keep it bisectable; one logical step per commit:

1. `git mv` the runtime modules into `pybullet_sim/` (+ `__init__.py`); rewrite
   their sibling imports to relative. Fix `constants.py` paths.
2. `git mv pybullet_interpreter pybullet_sim/interpreter`; bump
   `clip_loader.py` `parents[3]→[4]`; fix the gaits/parity import paths.
3. `git mv generate_urdf.py verify_export_parity.py urdf_gen/` (+ `__init__.py`);
   fix `generate_urdf.py` `SCRIPT_DIR` and `verify_export_parity.py` sys.path.
4. Rewire `facehugger.py` to `python -m …` invocations.
5. Update all docs (§4).

Run the §5 checklist before *each* commit so a break is localized.

---

## 7. Risks & rollback

- **Silent path breakage** (`parents[N]`, sibling paths) — caught only by the
  §5 smoke runs, not by import/compile. This is why every commit re-runs the
  checklist.
- **External path coupling** — firmware test tooling or Blender scripts that
  reach into `code/simulation/pybullet_interpreter` by hardcoded path would
  break; §4 covers the grep, but double-check `animation/` and
  `code/firmware/test/`.
- **`clips_all.h` lookup** — the interpreter finds it via repo-root walk; an
  off-by-one `parents[]` points it at the wrong directory and fails at runtime,
  not import. The `--clip` smoke run in §5 is the guard.
- **Rollback** is clean: the whole change is moves + mechanical import edits on
  one branch; `git revert` the range or reset the branch.

---

## 8. Open question for later

`facehugger_config.yaml` is read by **both** `urdf_gen` (drives generation) and
the runtime (`constants.CONFIG_YAML`). Keeping it at the `simulation/` top level
(as planned) keeps it neutral. If a future change makes it generation-only,
revisit whether it should live in `urdf_gen/`.
