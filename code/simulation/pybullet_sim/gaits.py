"""Gait registry, foot trajectories, debug overlay, run_stand / run_gait."""

import math
import os
import time

import pybullet as p
import pybullet_data

from .constants import TIMESTEP
from .helpers import (
    _wrap_pi,
    apply_joint_targets,
    apply_leg_pose,
    build_joint_map,
    reset_to_stance,
)

# --------------------------------------------------------------------------- #
# Physics realism (applied in _connect_and_setup)
# --------------------------------------------------------------------------- #
FOOT_LATERAL_FRICTION = 2.5  # high grip — models the rubber/elastic bands on the
# real feet (PyBullet multiplies foot × plane, so this is the grippy end)
FOOT_SPINNING_FRICTION = 0.3  # rubber tips resist the foot pivoting in place
SOLVER_ITERATIONS = 150  # stiffer, less-jittery contact resolution


# --------------------------------------------------------------------------- #
# Gait registry
# --------------------------------------------------------------------------- #

GAITS = {
    "walk": {
        "period": 2.4,
        "step_length": 0.04,
        "step_height": 0.02,
        "duty": 0.25,
        "offsets": {"fl": 0.00, "br": 0.25, "fr": 0.50, "bl": 0.75},
        "label": "Static walk",
    },
    "trot": {
        "period": 0.8,
        "step_length": 0.05,
        "step_height": 0.025,
        "duty": 0.5,
        "offsets": {"fl": 0.0, "br": 0.0, "fr": 0.5, "bl": 0.5},
        "label": "Trot (diagonal pairs)",
    },
}

_TRAJ_COLORS = {
    "fl": (1.0, 0.30, 0.30),
    "fr": (0.30, 1.0, 0.30),
    "bl": (0.30, 0.50, 1.0),
    "br": (1.0, 1.0, 0.30),
}


# --------------------------------------------------------------------------- #
# Foot trajectory + per-tick joint targets
# --------------------------------------------------------------------------- #


def foot_target(
    neutral_foot, leg_id, phase, step_length, step_height, duty, swing_axis="y"
):
    """Body-frame foot target. swing_axis selects which body axis steps forward.
    Body +Y is forward, so the default swing_axis="y" steps in the forward direction."""
    nx, ny, nz = neutral_foot[leg_id]
    if phase < duty:
        s = phase / duty
        d = -step_length * 0.5 + s * step_length
        dz = step_height * math.sin(math.pi * s)
    else:
        s = (phase - duty) / (1.0 - duty)
        d = step_length * 0.5 - s * step_length
        dz = 0.0
    if swing_axis == "y":
        return (nx, ny + d, nz + dz)
    return (nx + d, ny, nz + dz)


def gait_joint_targets(cfg, gait, t):
    offsets = gait["offsets"]
    global_phase = (t / gait["period"]) % 1.0
    targets = {}
    for leg_id, off in offsets.items():
        phase = (global_phase - off) % 1.0
        foot = foot_target(
            cfg.neutral_foot,
            leg_id,
            phase,
            gait["step_length"],
            gait["step_height"],
            gait["duty"],
        )
        s, h, k = cfg.leg_ik(cfg, foot, leg_id)
        targets[f"{leg_id}_link1_joint"] = s
        targets[f"{leg_id}_link2_joint"] = h
        targets[f"{leg_id}_link3_joint"] = k
    return targets


# --------------------------------------------------------------------------- #
# Debug overlay
# --------------------------------------------------------------------------- #

_TRAJ_SAMPLES = 40
_LINE_IDS: dict = {}
_MARK_IDS: dict = {}


def _body_to_world(pos_body, base_pos, base_orn):
    wp, _ = p.multiplyTransforms(base_pos, base_orn, pos_body, [0, 0, 0, 1])
    return wp


def _precompute_cycle(cfg, gait):
    offsets = gait["offsets"]
    cycles = {}
    for leg_id in offsets:
        pts = [
            foot_target(
                cfg.neutral_foot,
                leg_id,
                k / _TRAJ_SAMPLES,
                gait["step_length"],
                gait["step_height"],
                gait["duty"],
            )
            for k in range(_TRAJ_SAMPLES)
        ]
        pts.append(pts[0])
        cycles[leg_id] = pts
    return cycles


def _draw_overlay(robot_id, cycles, current_targets):
    base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
    for leg_id, pts in cycles.items():
        color = _TRAJ_COLORS.get(leg_id, (1, 1, 1))
        prev = _body_to_world(pts[0], base_pos, base_orn)
        for k in range(1, len(pts)):
            cur = _body_to_world(pts[k], base_pos, base_orn)
            key = (leg_id, k - 1)
            lid = _LINE_IDS.get(key)
            _LINE_IDS[key] = p.addUserDebugLine(
                prev,
                cur,
                lineColorRGB=color,
                lineWidth=1.5,
                replaceItemUniqueId=lid if lid is not None else -1,
            )
            prev = cur
        tw = _body_to_world(current_targets[leg_id], base_pos, base_orn)
        s = 0.012
        axes = [
            ((-s, 0, 0), (s, 0, 0)),
            ((0, -s, 0), (0, s, 0)),
            ((0, 0, -s), (0, 0, s)),
        ]
        for i, (a, b) in enumerate(axes):
            p0 = tuple(tw[j] + a[j] for j in range(3))
            p1 = tuple(tw[j] + b[j] for j in range(3))
            mkey = (leg_id, i)
            mid = _MARK_IDS.get(mkey)
            _MARK_IDS[mkey] = p.addUserDebugLine(
                p0,
                p1,
                lineColorRGB=color,
                lineWidth=3.0,
                replaceItemUniqueId=mid if mid is not None else -1,
            )


# --------------------------------------------------------------------------- #
# Simulation entry points
# --------------------------------------------------------------------------- #


def _body_height_for_gait(cfg, gait, samples_per_period=100):
    """Maximum foot-depth below body origin sampled over one gait period.
    Covers swing + stance feet across all 4 legs. Used in place of the
    neutral-stance depth so the spawn Z accommodates gait trajectories
    where foot-contact Z differs from neutral_foot."""
    offsets = gait["offsets"]
    worst = 0.0
    for k in range(samples_per_period):
        global_phase = k / samples_per_period
        for leg_id, off in offsets.items():
            phase = (global_phase - off) % 1.0
            foot = foot_target(
                cfg.neutral_foot,
                leg_id,
                phase,
                gait["step_length"],
                gait["step_height"],
                gait["duty"],
            )
            if -foot[2] > worst:
                worst = -foot[2]
    return max(1e-3, worst)


def _settle(robot_id, joint_map, cfg, duration_s):
    """Step the sim for `duration_s` while holding stance targets. Lets
    gravity resolve any initial overlap before gait/stand loops begin.
    No visible debug draws here — just physics."""
    n_steps = int(duration_s / TIMESTEP)
    for _ in range(n_steps):
        apply_leg_pose(
            robot_id, joint_map, cfg.stance_rad, cfg.servo_force, cfg.servo_velocity
        )
        p.stepSimulation()


def _make_step_monitor(robot_id, joint_map, every=30, logger=None):
    """Return an on_step(i) callback for the sim loops.

    --monitor: prints a torque + estimated-current status line every `every`
    sim steps (~8 Hz at 240 Hz) — per-leg angles, peak joint torque, total
    estimated current ([WARN >10A]) and any stalling joint ([STALL]).

    --log: if `logger` (a sim_monitor.SimLogger) is given, records every step
    (not just every `every`) for the end-of-run summary/CSV/plot. Reads torques
    once per step and shares them with the periodic print. See sim_monitor.py.
    """
    from . import sim_monitor

    def on_step(i):
        do_print = i % every == 0
        if logger is None and not do_print:
            return
        torques = sim_monitor.read_joint_torques(robot_id, joint_map)
        if logger is not None:
            logger.record(i * TIMESTEP, torques)
        if do_print:
            pos = sim_monitor.read_joint_pos_deg(robot_id, joint_map)
            print(sim_monitor.format_status(i * TIMESTEP, torques, pos))

    return on_step


def setup_step_hook(robot_id, joint_map, monitor, log):
    """Build the (on_step, logger) pair shared by the run_* loops.

    Returns (None, None) when neither --monitor nor --log is set. When --log,
    the SimLogger is returned too so the caller can _finalize_log() it after
    the loop. See _make_step_monitor / sim_monitor.SimLogger.
    """
    if not (monitor or log):
        return None, None
    from . import sim_monitor

    logger = sim_monitor.SimLogger(list(joint_map.keys())) if log else None
    on_step = _make_step_monitor(robot_id, joint_map, logger=logger)
    return on_step, logger


def _finalize_log(logger):
    """End-of-run output for --log: summary table, CSV, and 3-panel plot.

    Called from each run_* finally block so it runs even on Ctrl+C / p.error.
    No-op when logging is disabled.
    """
    if logger is None:
        return
    logger.summary()
    logger.save_csv("sim_log.csv")
    logger.plot("sim_log.png")


def _connect_and_setup(cfg, gui, float_mode=False):
    """Connect PyBullet and load the plane + robot.

    float_mode=True: no gravity, no floor, and the body is pinned in the air
    (useFixedBase). Use it to watch a clip's pure joint geometry — each leg
    articulates exactly as authored, with no falling/slipping/collapse from
    physics. (Normal mode loads the ground plane, real gravity, and a free
    floating base so you see dynamic balance.)
    """
    p.connect(p.GUI if gui else p.DIRECT)
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0 if float_mode else -9.81)
    p.setTimeStep(TIMESTEP)
    # More solver iterations -> stiffer, less-jittery contacts (helps planted-feet
    # moves resolve cleanly). Harmless in float mode (no contacts).
    p.setPhysicsEngineParameter(numSolverIterations=SOLVER_ITERATIONS)
    if not float_mode:
        p.loadURDF("plane.urdf")

    robot_id = p.loadURDF(
        cfg.urdf_path,
        basePosition=[0, 0, cfg.body_height + 0.02],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=float_mode,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
    )
    joint_map = build_joint_map(robot_id)
    # stance_rad is already per-leg; reset + motor-command from the same dict.
    reset_to_stance(robot_id, joint_map, cfg.stance_rad)
    apply_leg_pose(
        robot_id, joint_map, cfg.stance_rad, cfg.servo_force, cfg.servo_velocity
    )

    # Foot contact (link3 = the knee joint's child = lower leg/foot). NB: joint_map
    # keys are URDF joint names (`*_link3_joint`) — they contain "link3", not
    # "knee"; the old "knee" match never fired, so feet sat at PyBullet's default
    # 0.5 friction and slipped during planted-feet moves. Grip + a little spin
    # resistance so the feet hold; no bounce.
    for name, idx in joint_map.items():
        if "link3" in name:
            p.changeDynamics(
                robot_id,
                idx,
                lateralFriction=FOOT_LATERAL_FRICTION,
                spinningFriction=FOOT_SPINNING_FRICTION,
                restitution=0.0,
            )

    if gui:
        p.resetDebugVisualizerCamera(
            cameraDistance=0.55,
            cameraYaw=45,
            cameraPitch=-25,
            cameraTargetPosition=[0, 0, 0.1],
        )
    return robot_id, joint_map


def _print_banner(cfg):
    print("\n=== FaceHugger sim ===")
    print(f"  URDF: {os.path.basename(cfg.urdf_path)}")
    print(f"  legs: {list(cfg.legs.keys())}")
    print(f"  servo: force={cfg.servo_force} N*m  vel={cfg.servo_velocity} rad/s")
    print(f"  body_height: {cfg.body_height * 1000:.1f} mm")
    print("  stance (deg, per leg):")
    for leg_id, s in cfg.stance_rad.items():
        print(
            f"    {leg_id}: shoulder={math.degrees(s['shoulder']):+.1f}  "
            f"hip={math.degrees(s['hip']):+.1f}  knee={math.degrees(s['knee']):+.1f}"
        )
    for leg_id, foot in cfg.neutral_foot.items():
        print(
            f"    {leg_id}: foot = "
            f"({foot[0] * 1000:+6.1f}, {foot[1] * 1000:+6.1f}, {foot[2] * 1000:+6.1f}) mm"
        )

    # IK round-trip check: recover stance angles from neutral foot.
    tol = math.radians(2.0)
    for leg_id, foot in cfg.neutral_foot.items():
        s, h, k = cfg.leg_ik(cfg, foot, leg_id)
        target = cfg.stance_rad[leg_id]
        ds = abs(_wrap_pi(s - target["shoulder"]))
        dh = abs(h - target["hip"])
        dk = abs(k - target["knee"])
        ok = max(ds, dh, dk) < tol
        tag = "OK" if ok else "FAIL"
        print(
            f"    IK[{leg_id}]: ds={math.degrees(ds):+.2f} dh={math.degrees(dh):+.2f} "
            f"dk={math.degrees(dk):+.2f} [{tag}]"
        )


def run_stand(cfg, gui=True, settle_s=0.5, float_mode=False, monitor=False, log=False):
    robot_id, joint_map = _connect_and_setup(cfg, gui, float_mode=float_mode)
    _print_banner(cfg)
    if float_mode:
        print("[float] no gravity/floor, body pinned — showing the stance pose")
    elif settle_s > 0:
        print(f"\n[settle] holding stance for {settle_s:.2f}s before idle loop")
        _settle(robot_id, joint_map, cfg, settle_s)
    print("\nStanding - Ctrl+C to exit.")
    on_step, logger = setup_step_hook(robot_id, joint_map, monitor, log)
    step = 0
    try:
        while p.isConnected():
            p.stepSimulation()
            if on_step is not None:
                on_step(step)
            if gui:
                time.sleep(TIMESTEP)
            step += 1
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if p.isConnected():
            p.disconnect()
        _finalize_log(logger)


def run_clip(
    cfg,
    clip_name,
    gui=True,
    settle_s=0.5,
    loop=False,
    float_mode=False,
    monitor=False,
    log=False,
    python_port=False,
):
    """Load and play an animation clip by name in PyBullet.

    By DEFAULT the joints are driven by the EXACT firmware code compiled to the
    host (firmware_sil) — i.e. what the robot would actually command — and the
    fh_sim module is auto-rebuilt if firmware sources changed. This requires a
    C++ toolchain; if it is unavailable the run errors (use --python instead).

    python_port=True instead uses the Python re-port (firmware_port.ClipPlayer),
    which needs no toolchain. The re-port reads animation/exported_clips/
    clips_all.h; the firmware (default) reads the firmware's own clips_all.h.
    loop=True (GUI, re-port only) replays continuously; float_mode pins the body;
    monitor/log add the torque/current readout + capture.
    """
    if not python_port:
        return _run_clip_sil(
            cfg,
            clip_name,
            gui=gui,
            settle_s=settle_s,
            float_mode=float_mode,
            monitor=monitor,
            log=log,
        )

    from firmware_port.clip_loader import (
        DEFAULT_CLIPS_H,
        get_clip_by_name,
        load_clips_all_h,
    )
    from firmware_port.clip_player import ClipPlayer

    clips = load_clips_all_h(DEFAULT_CLIPS_H)
    clip = get_clip_by_name(clips, clip_name)

    robot_id, joint_map = _connect_and_setup(cfg, gui, float_mode=float_mode)
    _print_banner(cfg)
    if float_mode:
        print("[float] no gravity/floor, body pinned — showing joint geometry")
    elif settle_s > 0:
        print(f"\n[settle] holding stance for {settle_s:.2f}s before clip")
        _settle(robot_id, joint_map, cfg, settle_s)

    loop_note = " (looping)" if loop and gui else ""
    mon_note = "  [monitor: torque/current]" if monitor else ""
    print(
        f"\n[clip] playing '{clip.name}' ({clip.duration_ms} ms){loop_note}{mon_note}"
    )
    on_step, logger = setup_step_hook(robot_id, joint_map, monitor, log)
    player = ClipPlayer(robot_id, joint_map, clip, cfg.servo_force, cfg.servo_velocity)
    try:
        player.play_blocking(gui=gui, loop=loop, on_step=on_step)
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if p.isConnected():
            p.disconnect()
        _finalize_log(logger)


def _run_clip_sil(
    cfg, clip_name, gui=True, settle_s=0.5, float_mode=False, monitor=False, log=False
):
    """Play a clip through the compiled firmware (software-in-the-loop)."""
    from firmware_sil.sil_bridge import FirmwareSILDriver

    try:
        driver = FirmwareSILDriver()  # auto-builds fh_sim if stale/missing
    except ImportError as e:
        raise SystemExit(
            f"{e}\n\nThe firmware driver is the default. Without a C++ toolchain, "
            "play clips with the Python re-port instead:\n"
            f"  python facehugger.py sim --clip {clip_name!r} --python"
        ) from e
    if clip_name not in driver.clip_names():
        raise KeyError(
            f"clip {clip_name!r} not in firmware clips {driver.clip_names()}"
        )

    robot_id, joint_map = _connect_and_setup(cfg, gui, float_mode=float_mode)
    _print_banner(cfg)
    if float_mode:
        print("[float] no gravity/floor, body pinned — showing joint geometry")
    elif settle_s > 0:
        print(f"\n[settle] holding stance for {settle_s:.2f}s before clip")
        _settle(robot_id, joint_map, cfg, settle_s)

    mon_note = "  [monitor: torque/current]" if monitor else ""
    print(f"\n[clip][SIL] playing '{clip_name}' via exact firmware code{mon_note}")
    on_step, logger = setup_step_hook(robot_id, joint_map, monitor, log)
    try:
        driver.play_clip_blocking(
            robot_id,
            joint_map,
            clip_name,
            cfg.servo_force,
            cfg.servo_velocity,
            gui=gui,
            on_step=on_step,
        )
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if p.isConnected():
            p.disconnect()
        _finalize_log(logger)


def _run_gait_sil(
    cfg, gait_name, gui=True, settle_s=0.5, float_mode=False, monitor=False, log=False
):
    """Run a gait through the compiled firmware (software-in-the-loop) — the EXACT
    tickGait/tickTrot. Spawns at the neutral-stance body height (like clips), which
    is stable since the gait oscillates around NEUTRAL[]; no Python body-height solve."""
    from firmware_sil.sil_bridge import FirmwareSILDriver

    try:
        driver = FirmwareSILDriver()  # auto-builds fh_sim if stale/missing
    except ImportError as e:
        raise SystemExit(
            f"{e}\n\nThe firmware driver is the default. Without a C++ toolchain, "
            "run the gait with the Python re-port instead:\n"
            f"  python facehugger.py sim --{gait_name} --python"
        ) from e

    robot_id, joint_map = _connect_and_setup(cfg, gui, float_mode=float_mode)
    _print_banner(cfg)
    if float_mode:
        print("[float] no gravity/floor, body pinned")
    elif settle_s > 0:
        print(f"\n[settle] holding stance for {settle_s:.2f}s before gait")
        _settle(robot_id, joint_map, cfg, settle_s)

    mon_note = "  [monitor: torque/current]" if monitor else ""
    print(f"\n[gait][SIL] running '{gait_name}' (FW) via exact firmware code{mon_note}")
    on_step, logger = setup_step_hook(robot_id, joint_map, monitor, log)
    try:
        driver.run_gait_blocking(
            robot_id,
            joint_map,
            gait_name,
            cfg.servo_force,
            cfg.servo_velocity,
            gui=gui,
            on_step=on_step,
            duration_s=None if gui else 3.0,  # headless: finite smoke run
        )
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if p.isConnected():
            p.disconnect()
        _finalize_log(logger)


def run_gait(
    cfg,
    gait_name,
    gui=True,
    settle_s=0.5,
    float_mode=False,
    monitor=False,
    log=False,
    python_port=False,
):
    """Run a gait. By DEFAULT drives the joints with the EXACT compiled firmware
    (firmware_sil) — the same tickGait/tickTrot the robot runs. python_port=True uses
    the Python IK gait below instead (no C++ toolchain; also the body-height reference)."""
    if gait_name not in GAITS:
        raise ValueError(f"Unknown gait: {gait_name}")
    if not python_port:
        return _run_gait_sil(
            cfg,
            gait_name,
            gui=gui,
            settle_s=settle_s,
            float_mode=float_mode,
            monitor=monitor,
            log=log,
        )
    gait = GAITS[gait_name]

    # Gait-aware spawn height: worst foot Z across a full period, not just
    # neutral_foot. Prevents the body from sinking into the floor when a
    # gait's stance-phase Z differs from the neutral Z used at build time.
    gait_depth_m = _body_height_for_gait(cfg, gait)
    if gait_depth_m > cfg.body_height:
        print(
            f"[body_height] lifting spawn from {cfg.body_height * 1000:.1f} mm "
            f"to {gait_depth_m * 1000:.1f} mm for {gait_name} trajectory"
        )
        cfg.body_height = gait_depth_m

    robot_id, joint_map = _connect_and_setup(cfg, gui, float_mode=float_mode)
    _print_banner(cfg)
    if float_mode:
        print("[float] no gravity/floor, body pinned")
    elif settle_s > 0:
        print(f"\n[settle] holding stance for {settle_s:.2f}s before gait")
        _settle(robot_id, joint_map, cfg, settle_s)
    print(
        f"\n{gait['label']}: period={gait['period']:.2f}s  "
        f"len={gait['step_length'] * 1000:.0f}mm  h={gait['step_height'] * 1000:.0f}mm  "
        f"duty={gait['duty']:.2f}"
    )

    draw_overlay = gui
    cycles = _precompute_cycle(cfg, gait) if draw_overlay else None
    draw_every = 4

    on_step, logger = setup_step_hook(robot_id, joint_map, monitor, log)
    t = 0.0
    step = 0
    try:
        while p.isConnected():
            targets = gait_joint_targets(cfg, gait, t)
            apply_joint_targets(
                robot_id, joint_map, targets, cfg.servo_force, cfg.servo_velocity
            )
            if draw_overlay and step % draw_every == 0:
                offsets = gait["offsets"]
                global_phase = (t / gait["period"]) % 1.0
                cur_targets = {
                    leg_id: foot_target(
                        cfg.neutral_foot,
                        leg_id,
                        (global_phase - off) % 1.0,
                        gait["step_length"],
                        gait["step_height"],
                        gait["duty"],
                    )
                    for leg_id, off in offsets.items()
                }
                _draw_overlay(robot_id, cycles, cur_targets)

            p.stepSimulation()
            if on_step is not None:
                on_step(step)
            if gui:
                time.sleep(TIMESTEP)
            t += TIMESTEP
            step += 1
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if p.isConnected():
            p.disconnect()
        _finalize_log(logger)
