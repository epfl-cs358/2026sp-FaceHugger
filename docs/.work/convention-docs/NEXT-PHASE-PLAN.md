# Next-phase plan — flashing, discoverability, invert flag, rig limits

Created 2026-05-23. Follows the convention work (CONVENTIONS.md) and the five
committed changes C / 3a / 3b / B / D. This is the plan for what comes after the
first hardware flash. Personal todo context: `~/Downloads/what-to-do-next.md`.

## Status recap

Commits on `feat/animation-flow-integration` (pre-reorder order):
`C 4c3741a → 3a 9bc1e1d → 3b 5f6ee28 → B 8936b04 → D 13413a0`.
Safe to flash: C, 3a, 3b, D. **B held** until the FL shoulder horn is physically
remounted and the FL-keying clips are re-exported.

## Decisions (2026-05-23)

1. **Reorder** `feat/animation-flow-integration` so B is last: `C → 3a → 3b → D → B`.
   `feat/safe-flash` = the branch point just before B (i.e. `…→ D`).
2. **Branch naming:** temporary, pre-hardware-test branches use `wip/…`; promote to
   `feat/…` once validated on hardware.
3. **Discoverability:** new command **`T:8 CMD_LIST_CLIPS`**.
4. **Invert — simplified, full feature deferred:**
   - **Defer** the keyframe-able Blender invert toggle and the on-board `.h`
     clip-format invert events entirely.
   - **Add `T:9 {"inverted": true|false}`** — sets the `isInverted` flag ONLY: no
     servo movement, no reset to NEUTRAL. The next motion tick (gait, clip, stand)
     applies the mirrored pose through the `applyServos` choke from Change D.
   - **Fix `T:6` `invertRobot()`:** currently (post-D) it re-poses to NEUTRAL when
     toggled, which fights animations. Change it to flag-only (toggle), same as T:9.
   - The `.js` streaming exporter MAY emit `{"T":9,"inverted":true}` at a frame for a
     mid-clip flip — a small, separate exporter addition, NOT a new clip format.
5. **On-board invert events:** fully deferred. No header format change. T:9 suffices.

---

## Branch & worktree map

```
feat/animation-flow-integration   reordered → C 3a 3b D B   (canonical; build features on this)
  └── feat/safe-flash              = …→ D  (no B)            worktree: ../safe-flash   ← flash/test here
  └── wip/invert-flag              T:9 + T:6 fix             off feat/animation-flow-integration
  └── wip/clip-discoverability     T:8 + flash workflow + consistency tool
  └── wip/rig-shoulder-limits      3c + asymmetric limits    scratch blend fh_rigged_limits.blend
```

Each `wip/` branch gets its own worktree when started (`git worktree add ../<name> -b wip/<name> feat/animation-flow-integration`). **Hold all `wip/` branches until safe-flash is hardware-tested.**

### Reordering main (run yourself — see note)
The reorder rewrites unpushed history. Interactive rebase (`-i`) and `reset --hard`
are blocked in the agent environment, and the working tree currently carries
unrelated uncommitted changes (exported clips, the rig `.blend`, settings, the
2026-05-21 changelog), so the agent will NOT rewrite this branch in place. Do it
yourself once the tree is clean (commit or stash the pending changes first):

```bash
git rebase -i 4c3741a~1     # reorder the pick lines to: C, 3a, 3b, D, B
# resolve any small spinal_cord.cpp / motion_math.cpp conflict (D vs B touch nearby code)
```

The agent instead creates `feat/safe-flash` (= C,3a,3b,D) directly via cherry-pick in
its own worktree, which is the artifact you actually flash from — independent of
whether/when you reorder main.

---

## Workstream 1 — Flash & test (now)

Worktree `../safe-flash` on `feat/safe-flash` (C,3a,3b,D). Follow the test checklist in
`~/Downloads/what-to-do-next.md`. Key gates: gaits must feel bit-for-bit identical
(if not, D regressed — roll back); `{"T":2,"s":4}` → flat X, `{"T":2,"s":5}` → stand;
clip motion smoother; flip + gait works upside-down. Tune `CLIP_EMA_ALPHA` (0.75) down
if clips feel laggy.

FL horn remount + B flash + clip re-export is a separate hands-on session (steps in the
personal todo).

## Workstream 2 — `wip/invert-flag` (small firmware)

Goal: animation- and client-driven invert that doesn't fight motion.

- **T9-1** Add `T:9 CMD_SET_INVERT` to `data.h` enum + `network.cpp` handler:
  `{"T":9,"inverted":<bool>}` sets `isInverted` directly, no motion. Bounds: require the
  `inverted` key be a bool.
- **T9-2** Fix `T:6` `invertRobot()` (`spinal_cord.cpp`): drop the NEUTRAL re-pose; just
  toggle `isInverted` and reset `gaitPhaseStartMs_`. Next tick applies the mirror via
  `applyServos`. (This supersedes the re-pose introduced in Change D.)
- **T9-3** Document T:9 + the new T:6 semantics in `code/API_SPEC.md`.
- **T9-4 (optional, exporter)** `.js` export emits `{"T":9,"inverted":…}` at frames where
  an (future) invert property changes — defer with the Blender toggle; not now.
- Tests: native test that the handler accepts only bool; the flag→mirror behaviour is
  already covered by `test_invert_mirror`. `pio run` clean; gaits unchanged.

## Workstream 3 — `wip/clip-discoverability` (firmware + tooling)

Goal: flash a header with every Blender clip; client discovers and plays by id.

- **D8-1** `T:8 CMD_LIST_CLIPS` in `data.h` + `network.cpp`: firmware replies over the
  socket with `{"clips":[{"id":i,"name":FH_CLIPS[i].name,"ms":FH_CLIPS[i].duration_ms},…]}`
  built from `FH_CLIPS[]` / `FH_CLIP_COUNT`. Document in `API_SPEC.md`.
- **D8-2** Client flow (app/JS): `T:8` → list names → user picks → `T:7 {"c":<id>}` (existing).
- **D8-3** Flash workflow doc (one place): re-export all clips from Blender → bundled export
  regenerates `clips_all.h` + `clips_manifest.json` → run the consistency check (below) →
  `pio run -t upload`. The on-board player (`playClip`/`tickClip`) already exists.
- **D8-4 Consistency check** `animation/scripts/check_export_consistency.py`: for every
  exported clip, parse the `.js` `CLIP[]` servo arrays and the `.h` `FhClipFrame`s, apply
  the firmware `translateToServo` to the `.h` math-space frames, and assert
  `round(translateToServo(h_frame)) == js_servo` per frame/channel. This is the missing
  end-to-end `.js`↔`.h`↔firmware gate (today's parity tests only check the formula, not the
  baked artifacts). Wire it into the export step and CI.

## Workstream 4 — `wip/rig-shoulder-limits` (Blender, 3c) + asymmetric limits

Work on a **scratch blend** `animation/fh_rigged_limits.blend`; promote to
`fh_rigged_latest.blend` only after the headless rig test passes.

The asymmetric shoulder-yaw ranges are permanent physical reality (mirrored mounting):
one side ≈ `servo 38..180`, the mirrored side ≈ `servo 0..142` — a 142°-wide window placed
asymmetrically in `[0,180]`. Exact numbers must be derived from the URDF `link1` limits
**after B** (FL's mapping moved 75→135). Layered handling:

- **R-1 (primary) rig limits:** per-leg `LIMIT_ROTATION` on each shoulder bone, set to the
  bone-space equivalent of that leg's range. Method: URDF `link1` lower/upper → express
  relative to the post-B outward zero → map through each leg's `translateToServo` (servo
  window) and the rig `REST_ANGLE` (bone-space limit). Author can't drag past the stop.
- **R-2 bake-time net:** extend the exporter clamp to also WARN when a shoulder frame
  exceeds that leg's joint limit (today it only guards `[0,180]`). Catches motion authored
  outside the rig.
- **R-3 runtime:** leave firmware at `constrain(0,180)`. No per-leg shoulder clamp — if
  authoring is constrained, runtime clamping would silently distort rather than flag.

## Workstream 5 — doc promotion (after review)

Promote `docs/.work/convention-docs/CONVENTIONS.md` into a real repo/wiki location
(`doc/conventions/servo-and-control-conventions.md`), cross-link from CLAUDE.md and
`code/simulation/docs/`. Rename the two convention PNGs (filenames swapped vs content).
Optional: consolidate the per-leg neutral values to one source + a convention-vs-rig test.

## Deferred (explicitly not now)

- Keyframe-able Blender invert toggle (full feature).
- On-board `.h` clip-format invert events.
- T:3 body-pose static IK.
- T10/T11/T12 (configurable IP in exporter, heatmap rework, PyBullet clip validator) —
  tracked in the personal todo.
