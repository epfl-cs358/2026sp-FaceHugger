# FaceHugger verification checklists

Consolidated from the proposed-documentation source files. Grouping aid only - not part of the wiki.

## From api-reference.md

### T:1 — CMD_MOVE (Manual Movement)

- [ ] Connect to the robot and navigate to **Remote control** tab in the React Native app
- [ ] Touch the joystick and move it forward; robot should walk forward
- [ ] Release joystick; robot should stop and return to idle
- [ ] In the browser console, send `ws.send(JSON.stringify({T:1,dir:"FW"}))` and confirm robot walks forward
- [ ] Stop sending movement for 2+ seconds while walking; robot should revert to idle (no manual `STOP` needed)

### T:2 — CMD_FSM_STATE (Finite State Machine Transition)

- [ ] Open the React Native app; default is `STATE_IDLE`
- [ ] Tap **Remote control** tab; app should send `T:2` with `s:1` (`STATE_WALK`)
- [ ] Robot should become responsive to joystick movement
- [ ] Tap **Actions** tab; app should send `T:2` with `s:2` (`STATE_ACTION`)
- [ ] In the console, send `ws.send(JSON.stringify({T:2,s:0}))` and confirm robot enters idle

### T:3 — CMD_POSE (Body Pose / Static IK)

- [ ] Open the browser console
- [ ] Send `ws.send(JSON.stringify({T:2,s:2}))` to enter `STATE_ACTION`
- [ ] Send `ws.send(JSON.stringify({T:3,h:150,p:10,r:0}))` to tilt forward
- [ ] Robot should lean forward (nose down)
- [ ] Send `ws.send(JSON.stringify({T:3,h:150,p:-10,r:0}))` to tilt back
- [ ] Robot should lean back (nose up)

### T:4 — CMD_CALIBRATE (Servo Calibration)

- [ ] Open the React Native app and navigate to **Individual control**
- [ ] Pick a leg (e.g., Front-right) and a servo (e.g., Hip)
- [ ] Tap an angle button (e.g., "90°"); robot servo should move
- [ ] Confirm in the browser console: `ws.send(JSON.stringify({T:4,id:0,servo_id:0,a:90}))`
- [ ] Front-right hip should rotate to 90°

### T:5 — CMD_GAIT_MODE (Gait Selection)

- [ ] Open the React Native app and navigate to **Remote control**
- [ ] Ensure you are in the WALK state (if not, transition to it)
- [ ] Tap **TROT** button; robot should adopt trotting gait
- [ ] Move joystick forward; robot walks using diagonal-pair stepping
- [ ] Tap **CRAB** button; robot should switch to crab gait mid-walk
- [ ] Move joystick to the side; robot side-steps using crab gait

### T:6 — CMD_ACTION_SELECTION (Invert Robot / Wall Flip)

- [ ] Open the React Native app and navigate to **Actions**
- [ ] Tap **Invert robot** button (appears on `STATE_ACTION`)
- [ ] A confirmation prompt should appear
- [ ] Tap **Confirm invert**; robot should execute the flip maneuver
- [ ] In the browser console: `ws.send(JSON.stringify({T:6,a:0}))`
- [ ] Robot should flip and right itself

### T:7 — CMD_PLAY_CLIP (Play Animation Clip)

- [ ] Open a non-HTTPS page (e.g., `about:blank` in the browser console)
- [ ] Connect: `const ws = new WebSocket("ws://192.168.4.1:81");`
- [ ] Transition to ACTION state: `ws.send(JSON.stringify({T:2,s:2}))`
- [ ] Play clip 4 ("wiggle"): `ws.send(JSON.stringify({T:7,c:4}))`
- [ ] Open the **serial monitor** on the robot and confirm you see `[clip] play wiggle (73 frames)`
- [ ] Robot should wiggle once, then smoothly return to standing pose (~500 ms), then go idle
- [ ] Send another clip without waiting: `ws.send(JSON.stringify({T:7,c:3}))` (wave)
- [ ] Robot should interrupt wiggle and start waving immediately
- [ ] Confirm in the app's telemetry (`T:10` field `pc`) that playback progress updates each cycle
- [ ] Send an out-of-range clip ID: `ws.send(JSON.stringify({T:7,c:99}))` — robot should ignore it (no error)

### T:10 — System Status (Telemetry)

- [ ] Connect to the robot: `const ws = new WebSocket("ws://192.168.4.1:81");`
- [ ] Open the browser console and log incoming messages:
  ```js
  ws.onmessage = (e) => {
    const msg = JSON.parse(e.data);
    if (msg.T === 10) console.log("Telemetry:", msg);
  };
  ```
- [ ] Confirm you receive a telemetry message every ~500 ms
- [ ] Move the robot forward and confirm `a[0]` (speed) increases
- [ ] Rotate the robot and confirm `a[1]`, `a[2]`, `a[3]` (gyro) change
- [ ] Play a clip (`T:7`) and confirm `pc` increments from 0.0 to 1.0 as the clip plays
- [ ] Trigger an error (e.g., send an invalid servo angle) and confirm `e` contains an error message

### Verification Checklist

- [ ] **Connection**: Robot at `192.168.4.1:81` is reachable over `ws://` from HTTP page
- [ ] **Heartbeat**: Robot sends `T:10` telemetry every ~500 ms
- [ ] **T:1 Movement**: Joystick in app or `{T:1, dir:"FW"}` in console makes robot walk forward
- [ ] **T:2 FSM**: Switching tabs in app or `{T:2, s:1}` transitions FSM state; correct state appears in next telemetry
- [ ] **T:3 Pose**: `{T:3, h:150, p:10, r:0}` leans robot forward (nose down)
- [ ] **T:4 Calibrate**: Servo buttons in **Individual control** move specific joints; `{T:4, id:0, servo_id:0, a:90}` works
- [ ] **T:5 Gait**: Switching TROT/CRAB in app or `{T:5, g:0}` changes gait; robot uses new gait in next walk cycle
- [ ] **T:6 Action**: **Actions** tab → **Invert robot** → confirm → executes flip; `{T:6, a:0}` works
- [ ] **T:7 Clip** (NEW): Transition to `STATE_ACTION`, send `{T:7, c:4}`, robot plays "wiggle" once, returns to stand, goes idle
- [ ] **T:7 Clip IDs**: Verify all 5 clips (0–4) play as expected (lie down, one leg, tiny wiggle, wave, wiggle)
- [ ] **T:7 Interrupt**: Send `{T:7, c:4}` then immediately `{T:7, c:3}` → clip 3 starts, clip 4 is interrupted
- [ ] **T:7 Out-of-range**: Send `{T:7, c:99}` → robot ignores silently (no error in telemetry)
- [ ] **Telemetry Fields**: All fields in `T:10` (s, d, a, g, pc, e) are populated and update in real time
- [ ] **Safety Timeout**: Walk, then stop sending `T:1` for 2+ seconds; robot should revert to idle (no manual STOP)
- [ ] **Servo Clamp**: Deliberately send out-of-range angle; serial monitor should log `[WARN] servo <ch> clamped`
- [ ] **App State**: App pages correctly reflect robot FSM state; switching tabs sends correct state transitions
- [ ] **Queue on Connect**: Send commands before WebSocket is fully open; verify they queue and flush on `onopen`

---

## From blender-addon.md

### 1. Clip Management

- [ ] Load `animation/fh_rigged_latest.blend` in Blender 5.1+.
- [ ] Open the 3D Viewport, press `N`, switch to the **FaceHugger** tab.
- [ ] In the **Clips** sub-panel, enter a clip name (e.g., "test_wave") in the text field.
- [ ] Click **New Clip** — verify a new clip appears in the list below, with an active (filled) button.
- [ ] Verify the parent panel shows "Clip 'test_wave' [applied]" in the active-clip header.
- [ ] Rotate the shoulder (`fl_link1`) in pose mode, drag a foot target — verify the clip's Actions record those edits.
- [ ] In the **Clips** list, click the **⟳** (overwrite) icon next to the clip — a confirmation dialog appears; click **Overwrite**. Verify it re-binds the live animation onto the clip.
- [ ] Click **Rename** and enter a new name. Verify the clip name updates in the list and on the active-clip header.
- [ ] Click **Duplicate** and enter a new name. Verify a copy of the clip appears with distinct Actions.

### 2. Pose Library

- [ ] In the **Poses** sub-panel, click **Apply** next to `flat` — the rig splays into rest pose.
- [ ] Verify the parent panel shows a yellow ⚠ notice: `Pose 'flat' applied — not keyed`.
- [ ] Click **Apply** next to `standing` — the rig lifts and tucks. The notice updates to `Pose 'standing' applied — not keyed`.
- [ ] In the **Poses** sub-panel, enter a new pose name (e.g., "my_pose") and click **Save Current Pose** — the rig's current configuration is snapshotted to `poses.json`.
- [ ] Verify the name appears in the poses list with Apply/Rename/Delete buttons.
- [ ] Click **Rename** next to `my_pose`, enter a new name, and verify the rename propagates immediately (after panel redraw).
- [ ] Click **Apply** next to the renamed pose, then click **Key into 'active_clip' @ frame N** in the notice — verify 5 keyframes are inserted into the active clip's Actions at the recorded frame.
- [ ] Delete `my_pose` via **Delete** — verify it is removed from the library and `poses.json` is updated.

### 3. Selection Sets

- [ ] In the **Selection** sub-panel, click **Legs** — verify the 4 foot_target objects are selected (outlined white in viewport).
- [ ] Click **Front** — verify only `foot_target_fl` and `foot_target_fr` are selected.
- [ ] Click **FL** — verify only `foot_target_fl` is selected.
- [ ] Click **Body** — verify only `body_ctrl` is selected.
- [ ] Click **All** — verify all 5 controls are selected.
- [ ] Select and move `body_ctrl` by hand, then click **Legs** to re-select just the legs for keyframing.

### 4. Export Formats

- [ ] Apply a clip and add a few keyframes (e.g., frame 1 at rest pose, frame 10 at a different pose) by hand or via **Key into Clip**.
- [ ] In the **Export** sub-panel, tick **CSV**, **`.h`**, and **`.js`** format checkboxes.
- [ ] Click **Export Active Clip** — verify a dialog reports success and files appear in `animation/exported_clips/<clip>/`.
- [ ] Check the generated files exist:
  - `animation/exported_clips/<clip>/<clip>.csv` with 12 columns (joints) + frame/time_ms.
  - `animation/exported_clips/<clip>/<clip>.h` with a C struct.
  - `animation/exported_clips/<clip>/<clip>.js` with a WebSocket player script.
- [ ] **Browser `.js` verification:** Open a **non-HTTPS** page (file://, http://, or about:blank) in a web console. Paste the `.js` content. Verify the script logs `"Connecting to ws://192.168.4.1:81"` and does **not** execute (no robot needed; this is a dry-run check).
  - Optional: if `.js` dry-run toggle is enabled (under Browser JS), check that the script logs each frame's `{T:4, id, servo_id, a}` command instead of sending it.
- [ ] **Bundled export:** tick all export checkboxes, select **multiple clips** in the "Clips to export" checklist, then click **Export Selected Clips**. Verify `animation/exported_clips/clips_all.h` and `animation/exported_clips/clips_manifest.json` are created (next to the per-clip folders).

### 5. Activity Heatmap

- [ ] Apply a clip with smooth keyframed motion (e.g., 10 frames, 2° per frame).
- [ ] Scroll down to the **Display** sub-panel (collapsed by default) and click to expand.
- [ ] Toggle **Activity Heatmap ON** — verify the servo meshes appear in the viewport and a console message notes: "Activity Heatmap: link2/hip joints (fl_link2, fr_link2, bl_link2, br_link2) have no `__servo` mesh and are NOT shown — only the 8 shoulder/knee servos are coloured."
- [ ] Verify the 8 servo meshes change colour as the timeline scrubs (green for small frame deltas, red for large).
- [ ] Toggle the heatmap **OFF** — verify the meshes return to their prior colour and the viewport shading is restored to its pre-toggle state.
- [ ] Create a synthetic jerky motion: keyframe at frame 1, then key a very different pose at frame 2 (e.g., rotate shoulder by 60°). Toggle the heatmap **ON** again — verify the shoulder servo flashes red between those frames.

### 6. Preview Robot Motion (BEZIER ↔ LINEAR Interpolation Toggle)

- [ ] Apply a clip with 3+ keyframes on the body_ctrl or a foot target.
- [ ] In the **Export** sub-panel, click **Preview Robot Motion** — verify the console reports "active_clip: N keyframes -> LINEAR (robot preview)".
- [ ] Scrub the timeline and observe the motion: it is now linear (no acceleration/deceleration).
- [ ] Inspect the keyframes in the Graph Editor: check that a keyframe's interpolation is `LINEAR` (not `BEZIER`).
- [ ] Click **Preview Robot Motion** again — verify console reports "active_clip: N keyframes -> BEZIER (authoring)".
- [ ] The Bezier handles are still visible and intact in the Graph Editor.
- [ ] Scrub again: motion is now smooth (Bezier eased).

### 7. Authoring-Time Warnings

- [ ] Create a clip with a smooth motion (no large jumps). Bake and verify no frame-delta warnings appear.
- [ ] Create a second clip with an intentionally jerky frame: frame 1 at rest, frame 2 with `fl_link1` rotated by 30° (well above 20° threshold).
- [ ] Apply this clip and click **Export Active Clip**. Check the Blender console for: `WARNING: fl_link1 moves 30° between frame 1 and 2 (threshold: 20°)`.
- [ ] In Output Properties, set fps to 24. Apply a clip and export. Verify console shows: `WARNING: scene fps is 24; consider lowering to 12 in Output Properties...`.
- [ ] Lower fps to 12 and re-export: no fps warning.

### 8. Headless Export (`export_all_clips.py`)

- [ ] Open a terminal in the repo root.
- [ ] Export a subset of clips to `animation/fh_rigged_latest.blend` via the GUI (so clips exist).
- [ ] Save the `.blend`.
- [ ] Run the headless export command above.
- [ ] Verify exit code 0 (success): `echo $?`.
- [ ] Verify console output lists each clip: `Re-exporting N clip(s): [...]` and per-clip lines: `exported 'clip_name': M frames -> ...`.
- [ ] Verify `animation/exported_clips/clips_all.h` and `clips_manifest.json` exist.
- [ ] Verify `clips_manifest.json` is valid JSON with a `clips` array, each entry having `id`, `name`, `frame_count`, `duration_ms`.

### Integration tests (test_servo_parity.py / test_clips_header.py / test_bake_independence.py)

#### `test_servo_parity.py`

**Run:**
```bash
uv run python animation/scripts/test_servo_parity.py
```

- [ ] Exit code 0 (GREEN) — all assertions pass.
- [ ] Console output shows `PASS` for at least:
  - `_LEGS order matches firmware LegId`
  - Per-leg tests: `{leg} @ N == firmware translateToServo(N)` for all 4 legs
  - Scale contract: `convention.scale ≈ 2/3`
  - Wire-shape contract: `_LEG_ID` maps Blender leg names to firmware IDs correctly
  - Servo angle clamping: all output angles are int and 0–180

#### `test_clips_header.py`

**Run:**
```bash
uv run python animation/scripts/test_clips_header.py
```

- [ ] Exit code 0 (GREEN).
- [ ] Console output includes:
  - `PASS guard + count` (header has `#define FH_CLIP_COUNT N`)
  - `PASS struct typedefs present` (FhClipFrame, FhClip, FH_CLIPS array)
  - `PASS manifest ids 0..n-1` (clips are numbered sequentially)
  - `PASS clip array present` (e.g., `fh_clip_<name>[]` struct)
  - `PASS FR shoulder scaled-once == X.XXX` (pre-scale rule verified)
  - `PASS leg reorder FR,FL,RR,RL with neutral fixed point` (firmware LegId order)
  - `PASS deterministic header` and `PASS deterministic manifest`
  - `PASS empty clip refused`, `PASS non-monotonic t_ms refused`, `PASS C-symbol collision refused`, `PASS uint16_t overflow refused`

#### `test_bake_independence.py`

**Run:**
```bash
BLENDER_BIN=/Applications/Blender-5.1.app/Contents/MacOS/Blender \
  "$BLENDER_BIN" --background --factory-startup \
  animation/fh_rigged_latest.blend \
  --python animation/scripts/test_bake_independence.py
```

- [ ] Exit code 0 (GREEN).
- [ ] Console output shows:
  - `bake('wave') with active='wiggle' vs active='wave': MATCH`
  - `delta-warning fired 1 time(s) on synthetic jump` (the delta-warning function was tested)
  - `PASS LEG_IDS contains fr:0 fl:1 br:2 bl:3`
  - `PASS defensive upper clamp present` (`.js` includes `Math.min(180)`)
  - `PASS FRAME_MS == round(1000/fps)...`
  - `PASS delta-encode skip-on-unchanged present` (skip optimization in `.js`)
  - `PASS connection-failure alert present` (user-facing error in `.js`)
  - `preview toggle: BEZIER -> LINEAR -> BEZIER, handles_preserved=True` (toggle non-destructive)
  - `RESULT GREEN` (all checks passed)

---

## From firmware-states.md

### IDLE

- [ ] Power on; confirm robot is standing in neutral pose (shoulder/thigh/knee angles match `NEUTRAL[]` in math-space).
- [ ] Issue no commands; watch serial monitor (`pio device monitor`) for 500+ ms and confirm no `[gait]` or `[clip]` output.
- [ ] Use browser console (`ws.send(JSON.stringify({ T: 2, s: 0 }))`); robot remains stationary.
- [ ] Observe `robot_state = 0` in the CSV snapshot output if logging is enabled.

### WALK (STATE_WALK)

1. **Select and start gait:**
   - Browser console: `ws.send(JSON.stringify({ T: 5, g: 2 }))` (select trot).
   - Serial monitor shows: `[gait] Trot`.
   - Robot remains standing (gait selected but no movement command yet).

2. **Issue movement and observe motion:**
   - `ws.send(JSON.stringify({ T: 1, dir: "FW" }))` (forward).
   - Robot walks/trots forward with legs in sync.
   - Serial monitor shows target vector smoothly ramping up.

3. **Test graceful stop:**
   - Let gait run for 2+ cycles.
   - Stop sending movement commands.
   - After ~500 ms (deadman switch), robot slows and stops at a clean pose (not mid-stride).
   - FSM returns to IDLE.

4. **Test yaw rotation (in-place spin):**
   - While trotting: `ws.send(JSON.stringify({ T: 1, d: 4 }))` (rotate clockwise).
   - Front/rear hips adjust; body rotates in place without sliding sideways.

5. **Test clip interruption:**
   - While trotting: `ws.send(JSON.stringify({ T: 7, c: 4 }))` (play clip).
   - Gait stops immediately; clip playback begins.
   - On clip end, robot returns to standing pose and IDLE.

6. **Inspect angle ranges:**
   - Check serial output for `[WARN] servo <ch> clamped: <raw> -> <clamped>`.
   - A few warnings are OK; a flood (>1 per frame) means the gait is overextending and should be tuned.

### ACTION (STATE_ACTION)

1. **Basic clip playback:**
   - Browser console: `ws.send(JSON.stringify({ T: 7, c: 4 }))` (play clip 4, "wiggle").
   - Serial monitor shows: `[clip] play wiggle (73 frames)`.
   - Robot executes the wiggle motion once (no loop).
   - After ~1.5 s (clip duration + 500 ms ease), serial shows clip returning to idle.
   - Robot settles into standing pose.

2. **Return-to-stand behavior:**
   - Watch the serial output or CSV snapshot for `robot_state`.
   - Confirm state is ACTION while clip plays, then transitions to IDLE after ease.
   - Visually: robot does not freeze mid-pose; it smoothly eases back.

3. **Interrupt clip with gait:**
   - Start a clip: `ws.send(JSON.stringify({ T: 7, c: 4 }))`.
   - Before it ends, send a gait command: `ws.send(JSON.stringify({ T: 5, g: 2 }))` then `ws.send(JSON.stringify({ T: 1, dir: "FW" }))`.
   - Gait should immediately take over; clip is discarded.

4. **Clip without servo warnings:**
   - Play each clip in the manifest.
   - Monitor serial output for `[WARN] servo <ch> clamped` lines.
   - Expected: zero or very few (≤1 per clip). If a constant stream appears, the clip is pushing joints out of range and should be toned down in Blender.

5. **Invert-robot (wall-flip):**
   - `ws.send(JSON.stringify({ T: 6, a: 0 }))` (toggle invert ON).
   - Robot flips all legs; observes hard-coded wall-contact pose.
   - Serial shows no servo clamping (the angles are pre-validated).
   - `ws.send(JSON.stringify({ T: 6, a: 0 }))` again (toggle OFF).
   - Robot returns to standing pose.

6. **Clip end behavior (key test):**
   - Use CLIP_PLAYER_TESTING.md steps 1–3 (build, flash, connect).
   - Step 3, code example: send `{ T: 7, c: 4 }`.
   - Verify the robot does **not** hold at the clip's final pose; it eases back.
   - Check serial monitor for the absence of `[clip] play …` message after the ease ends (indicating STATE_IDLE).

### REST (STATE_REST)

- [ ] Send `ws.send(JSON.stringify({ T: 2, s: 4 }))` to enter REST.
- [ ] Observe all four legs move to the same joint angles (hip=90, thigh=90, knee=90 in servo-space).
- [ ] Serial monitor shows no motion output.
- [ ] Confirm power draw stabilizes (all joints at mid-point, minimal torque).
- [ ] Send a movement command; robot transitions back to IDLE and accepts motion.
- [ ] Use REST before powering down the robot for safety.

### FAILSAFE (STATE_FAILSAFE)

- [ ] Observe that FAILSAFE is not easily triggered under normal operation.
- [ ] If a servo hangs or goes unresponsive during testing, check the I²C lines (pull-ups, wire continuity, PCA9685 address).
- [ ] Confirm that the serial monitor does not show repeated `[WARN]` lines from the error handler while in normal states.
- [ ] If needed, trigger a controlled test by temporarily disconnecting a servo wire and confirming the robot detects the I²C fault and parks in FAILSAFE.
