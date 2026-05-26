# PyBullet simulation docs — folder index

A technical report of the FaceHugger PyBullet simulation as it stands on
`feat/pybullet-sim-interpreter`: the files involved, the exact PyBullet API used
under the hood, the gait/kinematics and clip-interpreter drivers, the
servo/joint conventions, and the sim↔firmware parity chain.

Companion to `../convention-docs/` — that folder is the servo-*convention* work;
this one is the *simulation* as a whole (engine setup, gaits, the clip
interpreter, parity). The convention material here cites the same canon.

## Wiki-ready

Reader-facing outputs, candidates to promote into the real repo docs / wiki.

- **`REPORT.md`** — the full technical report. Prose-first, with file map,
  per-aspect PyBullet API walkthrough (quoted `file:line` snippets), conventions
  reference table, and the parity guarantee. The §7 "doc-vs-code drift" list is
  the actionable bit for adapting the firmware and updating older docs.
- **`report.html`** — the same report rendered as a self-contained styled page
  (sticky TOC, syntax-highlighted code). Open directly in a browser; no build.
- **`REORG-PLAN.md`** — executable migration plan to split `code/simulation/`
  into `pybullet_sim/` (runtime) + `urdf_gen/` (build). ✅ executed.
- **`GAIT-AND-PARITY-PLAN.md`** — design (plan only, not implemented) for a
  firmware-faithful gait interpreter and a unified firmware↔sim control path;
  weighs Python re-port vs compile-and-call-firmware (SIL/FFI) vs
  hardware-in-the-loop, and analyses how strongly parity can be guaranteed.
  Backed by `research/fw-control-surface.md` + `research/sim-parity-infra.md`.
- **`EXACT-FIRMWARE-SIL-PLAN.md`** + **`.html`** — the focused plan (for review)
  for running the *exact flashed firmware C++* in the sim via SIL: how to compile
  the hardware-coupled control code against a mock/HAL layer, bind it to Python,
  drive PyBullet, test every firmware/clip change, simulate the JS app over the
  WebSocket API entrypoint, and tune masses. The headline option from the
  parity plan, made concrete. Open with the `.html` for a styled read.

## Evidence

Read-only research the report is built on. One focused study per subsystem,
deeper `file:line` detail than the synthesized report.

- `research/01-engine-setup.md` — connect/load/physics/step loop, the monitor.
- `research/02-gaits-kinematics.md` — IK/FK math, the `GAITS` table, foot trajectories.
- `research/03-interpreter.md` — `pybullet_interpreter/`, the servo-convention engine.
- `research/04-conventions-parity.md` — conventions, joint axes, the parity chain.

## Status

Generated 2026-05-26. The report flags seven places where prose docs lag the
live code/URDF (joint names `link1/2/3` not `shoulder/hip/knee`, asymmetric
shoulder ROM, per-leg shoulder axis sign, the 2026-05-25 BR un-mirror, etc.) —
see `REPORT.md §7`.

### What this branch adds — `feat/pybullet-sim-interpreter` since `feat/animation-flow-integration`

This branch grew the simulation from a gait viewer into a firmware-faithful
animation and control rig. The work came in a few waves; what follows is the whole
arc in prose, the report snapshot folded in (`REPORT.md` is a point-in-time
snapshot, so where it and this list disagree, this list is newer).

**A clip interpreter, then the exact firmware behind it.** The first wave ported the
firmware's clip pipeline into Python — `firmware_port/` (originally
`pybullet_interpreter/`) with `servo_convention.py` (a line-for-line port of
`translateToServo`), `clip_loader.py` (parses `clips_all.h`), and `clip_player.py`
(frame interpolation → joint targets) — and wired it into
`facehugger.py sim --clip NAME`, alongside two viewing aids: `--loop` to replay a
clip continuously and `--float` to pin the body weightless so joint geometry can be
inspected without gravity or a floor. The second wave went further and made the sim
run the *exact flashed firmware C++* rather than a re-port: `firmware_sil/` compiles
the real `SpinalCord` unchanged (via `hal/` shims + CMake/pybind11) into an `fh_sim`
module, and `sil_bridge.py` drives PyBullet from the firmware's own servo angles.
Clip playback now **defaults to that firmware**, with `--python` forcing the re-port
as a fallback and parity reference. The compiled module **auto-rebuilds** when
firmware sources change, and `python -m firmware_sil.sil_bridge --check` is a CI
freshness gate so a stale build can't silently mislead.

**Parity is enforced, not assumed.** `gen_golden.py` bakes a per-clip servo-angle
trace into `firmware_sil/golden/`, and `test_sil_clip_suite.py` replays every clip
and asserts an exact match — any firmware or clip change that shifts a servo angle
fails CI, which is the exporter guard. That sits on top of an end-to-end regression
net (`test_pipeline_regression.py`) and a package-structure test, plus the PoC and
WebSocket suites; 95 tests pass in all.

**Driving the sim like the real robot.** The firmware gained `T:8` (CMD_LIST_CLIPS)
for clip discovery and the mobile app a matching clip-discovery UI (T:8 request,
`ClipList`, store, Jest coverage). `facehugger.py serve` then exposed the full
API_SPEC `T:` protocol over WebSocket (default :8081) — and crucially the command
dispatch *is* the compiled firmware (`network.cpp::handleParsedMessage` + ArduinoJson,
routed to a shared `spinalCord`), so any change to the firmware's API handling
reflects automatically with no Python mirror to drift. `tools/robot_control_panel.html`
is a single-file, no-build panel that drives both the sim (`ws://localhost:8081`) and
the real robot (`ws://<ip>:81`); the unmodified app can connect to either too.

**Observability of forces and limits.** A physics-realism pass (plus a foot-friction
fix that had silently never fired) made the sim's loads meaningful, and `--monitor`
prints a live torque/current readout. `SimLogger` + `--log` capture per-step
torque/current to a summary, `sim_log.csv`, and a `sim_log.png` plot. The servo model
was corrected to the actual hardware, the QYRC DSS-230MG (30 kg·cm → 2.94 N·m, ~2 A);
the monitor's stall model and `facehugger_config.yaml`'s `effort_nm` now both read
2.94, matching the URDF (`mass_kg: 0.300` remains an unverified placeholder). Most
recently the firmware itself learned to warn before it clamps: `Leg::setJointAngles`
and `Servo::setServoAngle` print `[OOR] servo <ch> requested <deg>` when an angle
leaves [0,180] — a real bench feature on the serial monitor, captured by the SIL's
Serial mock. `serve` streams a per-joint telemetry frame over **SSE on :8082** at
~20 Hz, carrying both the servo-space command (what the real servos receive) and the
same command in URDF-joint degrees, the measured joint angle, a true tracking delta,
torque/current, and the pre-clamp request; the panel renders it with Δ/τ colours, a
Clamp column, and a stale banner, and `--gui` torque-tints the PyBullet links.

**Conventions corrected at the source.** The shoulder/yaw convention was made uniform
across all four legs — most notably the BR shoulder un-mirror (Change E), propagated
in lockstep through the three parity-locked `translateToServo` twins (firmware,
exporter, sim) so robot motion is unchanged — along with fixing an inverted FL
shoulder yaw direction and switching yaw export to uniform math-space. On the exporter
side, link1's delta-to-absolute conversion was corrected, the clip export path
repaired, and clips were re-exported with `clips_all.h` re-bundled and a new
cross-format verifier added to keep the `.h`/`.js`/sim representations in agreement.
The Blender N-panel gained a live servo-angle readout and per-clip frame-range sync
(clip-wide span, honouring the Custom-range selector).

**Structure and documentation.** `code/simulation/` was split into `pybullet_sim/`
(runtime) + `urdf_gen/` (build), the clip re-port relocated to `firmware_port/`, and
the dead `teleop.py` / `terrain.py` / `view_urdf.py` stubs removed (there is no `view`
subcommand — use `sim`). This docs folder is the paper trail: the technical
`REPORT.md`/`report.html`, the per-subsystem `research/` studies, the executed
`REORG-PLAN.md`, and the `EXACT-FIRMWARE-SIL-PLAN.md` that the SIL work followed.
