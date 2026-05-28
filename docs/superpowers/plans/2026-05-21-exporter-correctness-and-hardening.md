# Exporter Correctness & Packet Hardening — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the Blender clip exporter produce *correct, distinct, safe* output — fix the
bake-clip mislabel bug, warn on dangerous fast keyframes, verify the JS wire format, cut packet
load, and add a robot-motion preview toggle.

**Architecture:** All changes are in the Blender add-on `animation/scripts/fh_clip_panel.py`
plus a new headless-Blender regression test. The root bug: `bake_clip()` reads the *live* rig
pose from the depsgraph instead of binding the target clip first, so multi-clip "Export Selected"
bakes the active clip under every name. Fix = bind the clip (autocomplete → assign →
`view_layer.update()`) before sampling, restore the previous clip after. The other items are
additive (delta warning, FPS warning, delta-encoded JS, preview toggle).

**Tech Stack:** Python 3.12, Blender 5.1 `bpy` (run headless via `blender --background
--factory-startup`), `uv run` for pure-Python tests, git-lfs for the rig blend.

**Conventions:** Leg naming `fl/fr/bl/br` (never firmware `rr/rl`). Code is source of truth over
docs. Use context7 for `bpy` API. Don't commit `tmp/`. End commit messages with the
`Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>` trailer. No `git push`.

**Test fixture (verified to exist):** `animation/fh_rigged_latest.blend` — armature
`FaceHuggerRig`, anchor empty `body_ctrl`, 5 clips: `lie down and stand up` (frames 0-72, the
ACTIVE clip on load), `one leg lift` (0-49), `tiny wiggle` (0-72), `wave` (0-29), `wiggle`
(0-72). Each clip defines all 5 `CLIP_TARGETS`.

**Key existing API in `fh_clip_panel.py` (read before editing):**
- `bake_clip(clip_name, context)` :1653 — the buggy function. Returns list of row dicts
  `{"frame", "time_ms", "<bone>": angle, ...}` for the 12 `JOINT_BONES`.
- `active_clip()` :326 — name of the clip whose `body_ctrl` Action is bound, or `None`.
- `assign_clip(clip)` :389 — binds all 5 target Actions; returns `(assigned, missing_list)`.
- `_autocomplete_clip(clip, context)` :418 — creates any missing target Actions (idempotent).
- `clip_action(clip, target)` :322 — `bpy.data.actions.get(f"{clip}__{target}")`.
- `_frame_to_servo(row, convention)` :1756 — one row → `{leg: [hip,thigh,knee] int}` (servo deg).
- `_LEG_ID` :1700 = `{"fr":0,"fl":1,"br":2,"bl":3}`. `JOINT_BONES` :72. `_LEGS` :1691.
- `to_js(frames, clip_name, convention, loop=False, dry_run=False)` :1792 — emits the streaming
  `.js`. `LEG_IDS` :1927, `clamp` :1935, `playFrame()` :1948, `FRAME_MS` :1921.
- `_load_convention()` / `convention.json` — `neutral_joint_deg`, `scale` (≈2/3), `channels`.
- Blender resolution: reuse `_resolve_blender_bin(version)` from
  `code/simulation/facehugger.py:62` (`$BLENDER_BIN` → macOS app → `$PATH` → fail loud).

---

## Task 1: Headless test harness + bake-independence regression (RED)

**Files:**
- Create: `animation/scripts/test_bake_independence.py`

The invariant the bug violates: **baking a clip must depend only on the clip name, not on which
clip is currently active.** Today, with `lie down and stand up` active, `bake_clip("wave")`
samples the active pose, so baking the same clip under two different active clips yields
different rows.

- [ ] **Step 1: Write the failing test**

```python
#!/usr/bin/env python3
"""Headless-Blender regression: baking a clip must be independent of the
currently-active clip. Guards the bake_clip() mislabel bug (export-review
findings 2026-05-21 §2). Run via:

    BLENDER_BIN=/Applications/Blender-5.1.app/Contents/MacOS/Blender \
      "$BLENDER_BIN" --background --factory-startup \
      animation/fh_rigged_latest.blend \
      --python animation/scripts/test_bake_independence.py

Exit code 0 = GREEN, 1 = RED. --factory-startup avoids unrelated user
add-ons (e.g. ThreeMF_io) erroring on quit.
"""
import os
import sys
import importlib.util

import bpy

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCRIPT = os.path.join(REPO_ROOT, "animation/scripts/fh_clip_panel.py")


def _load_module():
    spec = importlib.util.spec_from_file_location("fh_clip_panel", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _servo_payload(mod, rows, conv):
    """Reduce baked rows to the wire payload the robot actually receives."""
    return [mod._frame_to_servo(r, conv) for r in rows]


def main() -> int:
    mod = _load_module()
    conv = mod._load_convention()
    ctx = bpy.context

    target = "wave"          # the clip we bake twice
    other = "wiggle"         # a clearly different clip to make active

    # Bake `target` while `other` is the active clip.
    mod.assign_clip(other)
    ctx.view_layer.update()
    rows_a = mod.bake_clip(target, ctx)

    # Bake `target` again while `target` itself is active.
    mod.assign_clip(target)
    ctx.view_layer.update()
    rows_b = mod.bake_clip(target, ctx)

    pay_a = _servo_payload(mod, rows_a, conv)
    pay_b = _servo_payload(mod, rows_b, conv)

    ok = pay_a == pay_b
    print(f"bake('{target}') with active='{other}' vs active='{target}': "
          f"{'MATCH' if ok else 'DIFFER'} ({len(pay_a)} vs {len(pay_b)} frames)")
    print("RESULT " + ("GREEN" if ok else "RED: bake_clip depends on active clip"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Run it to verify it FAILS against the current bug**

Run:
```bash
cd <repo-root>
BLENDER_BIN="${BLENDER_BIN:-/Applications/Blender-5.1.app/Contents/MacOS/Blender}"
"$BLENDER_BIN" --background --factory-startup animation/fh_rigged_latest.blend \
  --python animation/scripts/test_bake_independence.py 2>&1 | grep -E "^(bake\(|RESULT)"
```
Expected: `RESULT RED: bake_clip depends on active clip` (exit 1). If it prints GREEN, the bug is
already not reproducing — STOP and report (the fixture or assumptions changed).

- [ ] **Step 3: Commit the failing test**

```bash
git add animation/scripts/test_bake_independence.py
git commit -m "test(animation): headless regression for bake_clip active-clip bleed

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Fix `bake_clip` to bind the target clip (GREEN)

**Files:**
- Modify: `animation/scripts/fh_clip_panel.py:1653-1684` (`bake_clip`)

- [ ] **Step 1: Replace the body of `bake_clip` with the binding version**

```python
def bake_clip(clip_name, context):
    """Step through every frame of `clip_name`, evaluate the depsgraph and
    read the IK-solved joint angles. Returns a list of row dicts:

        [{"frame": 1, "time_ms": 0, "fl_link1": 0.0, "fl_link2": 0.0, ...},
         ...]

    Binds `clip_name`'s Actions onto the rig BEFORE sampling so the
    depsgraph reflects this clip's motion — not whatever clip happens to
    be active (the multi-clip "Export Selected" mislabel bug). Restores
    the previously-active clip + frame afterward. Pure data extraction —
    knows nothing about servos, scaling or channels. Raises ValueError if
    the rig/clip is not exportable."""
    scene = context.scene
    arm_obj = _find_arm_obj()
    if arm_obj is None:
        raise ValueError(f"Armature '{_ARM_OBJ_NAME}' not found in scene")
    action = clip_action(clip_name, "body_ctrl")
    if action is None:
        raise ValueError(f"No body_ctrl action for clip '{clip_name}'")

    # Bind the target clip so the depsgraph samples ITS motion. Autocomplete
    # first (mirrors FH_OT_apply_clip) so a clip missing a target doesn't
    # inherit the previous clip's Action on that target. view_layer.update()
    # forces the IK constraint stack to re-solve to the rebind before frame 0.
    prev_clip = active_clip()
    _autocomplete_clip(clip_name, context)
    _assigned, missing = assign_clip(clip_name)
    if missing:
        raise ValueError(
            f"Clip '{clip_name}' incomplete — missing: {', '.join(missing)}"
        )
    context.view_layer.update()

    frame_start = int(action.frame_range[0])
    frame_end = int(action.frame_range[1])
    fps = scene.render.fps / scene.render.fps_base

    original_frame = scene.frame_current
    rows = []
    for frame in range(frame_start, frame_end + 1):
        scene.frame_set(frame)
        depsgraph = bpy.context.evaluated_depsgraph_get()
        arm_eval = arm_obj.evaluated_get(depsgraph)
        angles = _read_bone_angles(arm_eval)
        time_ms = round((frame - frame_start) / fps * 1000)
        rows.append({"frame": frame, "time_ms": time_ms, **angles})
    scene.frame_set(original_frame)

    # Restore whatever clip was active before, so baking doesn't leave the
    # rig rebound to the last-baked clip (matters for "Export Selected").
    if prev_clip is not None and prev_clip != clip_name:
        _autocomplete_clip(prev_clip, context)
        assign_clip(prev_clip)
        context.view_layer.update()
    return rows
```

- [ ] **Step 2: Run the regression test to verify it PASSES**

Run the same command as Task 1 Step 2.
Expected: `RESULT GREEN` (exit 0).

- [ ] **Step 3: Commit**

```bash
git add animation/scripts/fh_clip_panel.py
git commit -m "fix(animation): bind target clip in bake_clip before sampling

bake_clip read the live depsgraph pose using only the target clip's
frame_range, so multi-clip Export Selected baked the active clip under
every name (export-review findings §2). Autocomplete + assign_clip +
view_layer.update() before the frame loop; restore the previously-active
clip + frame after. Regression test now GREEN.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Frame-delta warning (torque guard at authoring time)

**Files:**
- Modify: `animation/scripts/fh_clip_panel.py` — add module constant near the other constants
  (after `JOINT_BONES`, ~line 85) and a check inside `bake_clip` after the frame loop is built.
- Modify: `animation/scripts/test_bake_independence.py` — add a check that the warning fires.

- [ ] **Step 1: Add the constant + warning helper**

Add near the top constants:
```python
# Authoring-time torque guard: warn if any joint moves more than this many
# degrees between consecutive baked frames (fast keyframe = mechanical shock
# on hardware). Tunable; not a hard limit — the firmware clamp is the
# electrical backstop, this catches bad animation before it ships.
FRAME_DELTA_WARN_DEG = 20.0


def _warn_frame_deltas(rows):
    """Print a console WARNING for each joint that moves more than
    FRAME_DELTA_WARN_DEG between consecutive frames. Returns the number of
    warnings emitted (for tests)."""
    n = 0
    for prev, cur in zip(rows, rows[1:]):
        for bone in JOINT_BONES:
            delta = abs(cur[bone] - prev[bone])
            if delta > FRAME_DELTA_WARN_DEG:
                print(
                    f"WARNING: {bone} moves {delta:.0f}° between frame "
                    f"{prev['frame']} and {cur['frame']} "
                    f"(threshold: {FRAME_DELTA_WARN_DEG:.0f}°)"
                )
                n += 1
    return n
```

- [ ] **Step 2: Call it from `bake_clip` just before `return rows`**

```python
    _warn_frame_deltas(rows)
    return rows
```
(Place before the existing `return rows`; if the restore block precedes the return, call it
before the restore or after — order doesn't matter, it only reads `rows`.)

- [ ] **Step 3: Add a test that the warning fires on a synthetic fast jump**

Append to `test_bake_independence.py`'s `main()` before the final RESULT print:
```python
    # Frame-delta warning fires on a synthetic > threshold jump.
    fake = [
        {"frame": 0, "time_ms": 0, **{b: 0.0 for b in mod.JOINT_BONES}},
        {"frame": 1, "time_ms": 33, **{b: 0.0 for b in mod.JOINT_BONES}},
    ]
    fake[1]["fl_link1"] = mod.FRAME_DELTA_WARN_DEG + 5.0
    nwarn = mod._warn_frame_deltas(fake)
    print(f"delta-warning fired {nwarn} time(s) on synthetic jump")
    if nwarn != 1:
        print("RESULT RED: frame-delta warning did not fire as expected")
        return 1
```

- [ ] **Step 4: Run the test — expect GREEN + the WARNING line printed**

Run the Task 1 Step 2 command. Expected: a `WARNING: fl_link1 moves 25° ...` line and
`RESULT GREEN`.

- [ ] **Step 5: Commit**

```bash
git add animation/scripts/fh_clip_panel.py animation/scripts/test_bake_independence.py
git commit -m "feat(animation): warn on per-frame joint deltas above threshold

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: JS export verification assertions

**Files:**
- Modify: `animation/scripts/test_bake_independence.py` — add a section that bakes a clip, builds
  the `.js` via `to_js(..., dry_run=True)`, and asserts the wire contract.

The pure-Python `_frame_to_servo` ranges are already locked by `test_servo_parity.py`; this adds
an end-to-end check on the *generated JS string* for `LEG_IDS`, the 0-180 clamp, and `FRAME_MS`.

- [ ] **Step 1: Add the JS-verification block to `main()` — assert SEMANTICS, not formatting**

Do NOT match exact whitespace/literal declarations (template spacing may change). Parse the
values out with regex.

```python
    import re
    # JS export wire-contract checks (semantic — robust to template formatting).
    rows = mod.bake_clip("wave", ctx)
    js = mod.to_js(rows, "wave", conv, dry_run=True)
    fps = ctx.scene.render.fps / ctx.scene.render.fps_base
    expected_frame_ms = round(1000.0 / fps)

    def _leg_ids_ok(s):
        m = re.search(r"LEG_IDS\s*=\s*\{([^}]*)\}", s)
        if not m:
            return False
        body = m.group(1)
        return all(
            re.search(rf"\b{leg}\s*:\s*{idx}\b", body)
            for leg, idx in (("fr", 0), ("fl", 1), ("br", 2), ("bl", 3))
        )

    def _frame_ms_ok(s, expected):
        m = re.search(r"FRAME_MS\s*=\s*(\d+)", s)
        return m is not None and int(m.group(1)) == expected

    checks = {
        "LEG_IDS contains fr:0 fl:1 br:2 bl:3": _leg_ids_ok(js),
        "defensive upper clamp present": "Math.min(180" in js,
        f"FRAME_MS == round(1000/fps) ({expected_frame_ms})": _frame_ms_ok(js, expected_frame_ms),
    }
    for name, ok in checks.items():
        print(("PASS " if ok else "FAIL ") + name)
        if not ok:
            print("RESULT RED: JS export contract violated")
            return 1
```

- [ ] **Step 2: Run — expect all PASS + GREEN.** Task 1 Step 2 command.

- [ ] **Step 3: Commit**

```bash
git add animation/scripts/test_bake_independence.py
git commit -m "test(animation): verify JS export wire contract (LEG_IDS, clamp, FRAME_MS)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: FPS-too-high warning (recommend ≤12 fps for robot export)

**Files:**
- Modify: `animation/scripts/fh_clip_panel.py` — warn in `bake_clip` (console) when scene fps > 12.

- [ ] **Step 1: Add the constant + warning in `bake_clip` after `fps` is computed**

Constant near the others:
```python
# Recommended max authoring fps for robot export. Higher = more WS packets
# per second with no motion benefit for these slow clips. Not enforced —
# the animator sets fps in Output Properties; we only nudge.
RECOMMENDED_MAX_FPS = 12
```
In `bake_clip`, right after `fps = scene.render.fps / scene.render.fps_base`:
```python
    if fps > RECOMMENDED_MAX_FPS:
        print(
            f"WARNING: scene fps is {fps:.0f}; consider lowering to "
            f"{RECOMMENDED_MAX_FPS} in Output Properties for robot export "
            f"(fewer WebSocket packets, same motion)."
        )
```

- [ ] **Step 2: Run the regression test — fixture is 24 fps, so expect the fps WARNING line.**
Confirm `WARNING: scene fps is 24; consider lowering to 12 ...` appears and `RESULT GREEN`.

- [ ] **Step 3: Commit**

```bash
git add animation/scripts/fh_clip_panel.py
git commit -m "feat(animation): warn when scene fps exceeds recommended robot-export rate

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: Delta encoding in the JS template

**Files:**
- Modify: `animation/scripts/fh_clip_panel.py` `to_js` template — the `playFrame()` send loop
  (~lines 1948-1960) and add a `_last` per-channel cache.

Goal: only `ws.send()` a servo message when its value changed from the previous frame. First
frame sends all 12. Also kills the hold-at-end flood (held pose = unchanged = nothing sent).

- [ ] **Step 1: Inspect the live template first**

Run:
```bash
cd <repo-root>
python3 - <<'PY'
import importlib.util, types, sys
# (reuse the bpy stub from test_servo_parity.py to import the module)
PY
```
Actually simplest: open `fh_clip_panel.py` around line 1900-1965 and read the exact `to_js`
template f-string. The template currently builds `msg = {"T":4, "id":..., "servo_id":j,
"a":clamp(angles[j])}` and sends every j unconditionally in `playFrame()`.

- [ ] **Step 2: Modify the JS template so `playFrame` skips unchanged channels**

In the emitted JS, add a module-level cache and guard the send:
```javascript
// last-sent servo angle per "leg:servo_id" — delta encoding: only send on change.
const _last = {};
function sendServo(leg, j, a) {
  const key = leg + ":" + j;
  if (_last[key] === a) return;     // unchanged since last frame → skip
  _last[key] = a;
  ws.send(JSON.stringify({ "T": 4, "id": LEG_IDS[leg], "servo_id": j, "a": a }));
}
```
and in `playFrame()` replace the per-`j` `ws.send(...)` with `sendServo(leg, j, clamp(angles[j]))`.
Reset `_last = {}` wherever playback restarts (clip start / loop wrap) so the first frame
re-sends all channels.

- [ ] **Step 3: Add a connection-failure alert to the JS template**

The robot IP varies by network (currently `192.168.4.2`), so a wrong/unreachable IP must be
visible to the user instead of silently doing nothing. Where the template creates the socket
(`new WebSocket(<url>)`), attach handlers (adapt to the template's actual URL variable name):
```javascript
ws.onerror = () => {
  alert("FaceHugger: could not connect to " + ws.url +
        " — check the robot IP / Wi-Fi network and reload.");
};
ws.onclose = (e) => { if (!e.wasClean) console.warn("FaceHugger WS closed", e.code); };
```

- [ ] **Step 4: Re-run the JS-verification test (Task 4) — assertions still pass + add two**

Add to the Task 4 checks (semantic, not literal):
```python
        "delta-encode skip-on-unchanged present": "_last" in js,
        "connection-failure alert present": ("onerror" in js and "alert(" in js),
```

- [ ] **Step 5: Commit**

```bash
git add animation/scripts/fh_clip_panel.py animation/scripts/test_bake_independence.py
git commit -m "perf(animation): delta-encode JS servo stream (skip unchanged channels)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 7: F-curve interpolation preview toggle (N-panel)

**Files:**
- Modify: `animation/scripts/fh_clip_panel.py` — add an `FH_OT_toggle_preview` operator + a panel
  button. Follow the existing operator/panel registration pattern in this file (other
  `FH_OT_*` classes + the `register()`/`unregister()` lists).

Non-destructive: flipping `kp.interpolation` between `'BEZIER'` and `'LINEAR'` leaves
`handle_left/right` + `handle_*_type` intact, so toggling back restores Bezier exactly.

- [ ] **Step 1: Add the operator (use context7 to confirm fcurve/keyframe_point API for 5.1)**

```python
class FH_OT_toggle_preview(bpy.types.Operator):
    """Toggle the active clip's F-curves between BEZIER (authoring) and
    LINEAR (exactly what the robot plays). Non-destructive: Bezier handles
    are preserved on the keyframes and restored when toggled back."""
    bl_idname = "fh.toggle_preview"
    bl_label = "Preview Robot Motion"

    def execute(self, context):
        clip = active_clip()
        if clip is None:
            self.report({'WARNING'}, "No active clip")
            return {'CANCELLED'}
        # Determine current mode from the first keyframe we see.
        going_linear = None
        touched = 0
        for target in CLIP_TARGETS:
            action = clip_action(clip, target)
            if action is None:
                continue
            for fcurve in action.fcurves:
                for kp in fcurve.keyframe_points:
                    if going_linear is None:
                        going_linear = kp.interpolation != 'LINEAR'
                    kp.interpolation = 'LINEAR' if going_linear else 'BEZIER'
                    touched += 1
        mode = "LINEAR (robot preview)" if going_linear else "BEZIER (authoring)"
        self.report({'INFO'}, f"{clip}: {touched} keyframes -> {mode}")
        _redraw_view3d(context)
        return {'FINISHED'}
```

- [ ] **Step 2: Add the button to the N-panel `draw()` and register the class**

Add `layout.operator("fh.toggle_preview")` in the panel `draw()` near the other clip controls,
and add `FH_OT_toggle_preview` to the `register()`/`unregister()` class lists. (Match the exact
registration idiom already in the file — note the `bl_idname` panel-key gotcha.)

- [ ] **Step 3: Smoke-test headless that the toggle flips interpolation and preserves handles**

Add a function to `test_bake_independence.py`:
```python
    # Preview toggle: LINEAR flip is non-destructive to Bezier handles.
    act = mod.clip_action("wave", "body_ctrl")
    kp0 = act.fcurves[0].keyframe_points[0]
    h_left_before = tuple(kp0.handle_left)
    bpy.ops.fh.toggle_preview()
    assert kp0.interpolation == 'LINEAR', kp0.interpolation
    assert tuple(kp0.handle_left) == h_left_before, "handles must be preserved"
    bpy.ops.fh.toggle_preview()
    assert kp0.interpolation == 'BEZIER', kp0.interpolation
    print("PASS preview toggle flips interpolation, preserves handles")
```
NOTE: `bpy.ops.fh.toggle_preview()` requires the operator be registered; the test imports the
module then calls `mod.register()` once. If registration in `--background` hits the panel
`bl_idname` gotcha (see memory), call only the operator's `execute` via a minimal context, or
register just the operator class.

- [ ] **Step 4: Run — expect PASS + GREEN.** Task 1 Step 2 command.

- [ ] **Step 5: Commit**

```bash
git add animation/scripts/fh_clip_panel.py animation/scripts/test_bake_independence.py
git commit -m "feat(animation): add Preview Robot Motion (BEZIER<->LINEAR) toggle

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Task 8: VCS — track the rig (git-lfs) + un-ignore exported clips

**Files:**
- Create: `.gitattributes`
- Modify: `.gitignore:857` (un-ignore `fh_rigged_latest.blend`), `.gitignore:863` (un-ignore
  `animation/exported_gaits/`)

- [ ] **Step 1: Install + configure git-lfs for the blend**

```bash
git lfs install
printf '*.blend filter=lfs diff=lfs merge=lfs -text\n' > .gitattributes
```

- [ ] **Step 2: Un-ignore the canonical blend and the exported clips**

Remove the `animation/fh_rigged_latest.blend` line (keep `animation/blend-iterations/` ignored)
and the `animation/exported_gaits/` line from `.gitignore`. Keep `*.blend1` ignored.

- [ ] **Step 3: Track the blend + commit**

```bash
git add .gitattributes .gitignore animation/fh_rigged_latest.blend
git lfs ls-files   # expect: fh_rigged_latest.blend listed
git commit -m "chore(animation): track canonical rig via git-lfs; un-ignore exported_gaits

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```
(Teammates run `git lfs install` once locally or they get pointer files. No GitHub UI setup.)

---

## Task 9: Re-export all clips and verify distinct + correctly named

**Files:** none (verification). Produces `animation/exported_gaits/<clip>/{<clip>.js,.h,.csv}`.

- [ ] **Step 1: Re-export every clip from the fixed exporter**

Drive `FH_OT_export_selected` (or per-clip `FH_OT_export_clip`) for all 5 clips headless, e.g.:
```bash
"$BLENDER_BIN" --background --factory-startup animation/fh_rigged_latest.blend \
  --python animation/scripts/<export-all-helper>.py
```
If no headless export entry point exists, add a tiny `if __name__` driver that imports the
module, registers it, and calls the export operator for `list_clip_names()`.

- [ ] **Step 2: Verify each clip's `.js` payload is distinct + has a valid robot IP**

Do NOT assert a specific IP — the robot IP varies by network (currently `192.168.4.2`). Assert
only that each clip embeds *some* valid `192.168.x.x` address (the connection-failure alert from
Task 6 handles a wrong/unreachable one at runtime).

```bash
for d in animation/exported_gaits/*/; do
  name="$(basename "$d")"; f="$d$name.js"
  hash="$(md5 -q "$f" 2>/dev/null || md5sum "$f" | cut -d' ' -f1)"
  ip="$(grep -oE '192\.168\.[0-9]+\.[0-9]+' "$f" | head -1)"
  echo "$hash  ip=${ip:-MISSING}  $f"
done
```
Expected: 5 *different* hashes (no two clips identical), and every clip's `ip=` is a non-empty
`192.168.x.x`. Spot-check that `lie down and stand up` differs from `wiggle`.

- [ ] **Step 3: Commit the re-exported clips**

```bash
git add animation/exported_gaits
git commit -m "chore(animation): re-export all clips with fixed bake_clip (verified distinct)

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>"
```

---

## Deferred tasks (tracked, NOT yet implemented)

These were added after T1–T9 landed. Note the ordering: **T13 → T12 → T11**, and **T10** is
independent. Each gets fully fleshed (bite-sized TDD steps) before implementation.

### T10 — Robot IP configurability
The WS template hardcodes `192.168.4.1`, but the robot IP varies by network (currently `.4.2`).
- Add `bpy.context.scene.fh_robot_ip` (`StringProperty`, default `"192.168.4.1"`); expose as a
  text field in the N-panel.
- `to_js` reads the IP from this property instead of the hardcoded literal.
- Update T9's IP verification to check the `.js` against the scene-property value, not a
  hardcoded string.

### T11 — Heatmap rework (delta + static torque)
Replace the existing torque approximation with two honest layers (drop mass/gravity torque math —
it gave false precision):
- **Red:** frame-to-frame delta exceeds `FRAME_DELTA_WARN_DEG` (reuse the *same* constant as the
  console warning so they're always consistent).
- **Orange:** joint angle within 10% of its URDF limit.
- **Green:** neither.
- If a `torque_report.json` exists for the clip (produced by T12), use the **real PyBullet
  torques** for colouring instead, against the **2.94 Nm** servo limit from the URDF.

### T12 — PyBullet clip validator
- Add `code/simulation/validate_clip.py` + a `facehugger.py` subcommand:
  `python facehugger.py validate --clip <name> --out <json>`.
- Loads the URDF into PyBullet with full physics, feeds joint angles from the exported CSV frame
  by frame, reads `appliedJointMotorTorque` from `p.getJointState()` after each `stepSimulation()`,
  writes a per-joint per-frame torque report as JSON. Flags frames exceeding `servo_force`
  (2.94 Nm from config).
- The Blender add-on gets a **Validate** button that runs this as a subprocess using the **conda**
  env Python (not `uv` — PyBullet needs conda for C deps). Conda Python path read from
  `bpy.context.scene.fh_conda_python` (`StringProperty`, default `""`, UI hint: "path to conda env
  Python, e.g. `~/miniconda3/envs/facehugger/bin/python`"). After the subprocess completes, the
  add-on reads the JSON and updates the heatmap.
- **Blocking unknown (resolve in T13 first):** the CSV angle space. PyBullet needs joint-space
  **radians**; if the CSV stores servo-space **degrees**, a reverse-`translateToServo` step is
  required. Confirm before implementing T12.

### T13 — Information gathering for T12 (DO BEFORE T12 — read-only, no code)
- Read the current CSV export format in `fh_clip_panel.py` (`to_csv` ~1802 / `_bake_and_write`).
  Confirm whether exported angles are joint-space radians or servo-space degrees.
- `validate_clip.py` does not exist yet — confirm how the existing `simulate.py` / `gaits.py`
  feed joint angles to PyBullet so T12 follows the same conventions.
- Write findings to `tmp/validate-clip-findings.md` (local scratch).

## Self-review checklist (run before handoff)
- Spec coverage: bake fix (T2) ✓, headless regression (T1) ✓, delta warning (T3) ✓, JS verify
  (T4) ✓, fps warn (T5) ✓, delta encode (T6) ✓, preview toggle (T7) ✓, VCS (T8) ✓, re-export
  (T9) ✓.
- The `to_js` literal substrings in T4/T6 MUST be checked against the live template — adjust if
  spacing differs (flagged inline).
- T7 registration may hit the `bl_idname` headless gotcha — fallback noted inline.
