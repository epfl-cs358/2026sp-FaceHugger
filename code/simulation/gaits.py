"""Gait registry, foot trajectories, debug overlay, run_stand / run_gait."""

import math
import os
import time

import pybullet as p
import pybullet_data

from constants import TIMESTEP
from helpers import (
    _wrap_pi,
    apply_joint_targets,
    apply_leg_pose,
    build_joint_map,
    reset_to_stance,
)


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


def _connect_and_setup(cfg, gui):
    p.connect(p.GUI if gui else p.DIRECT)
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(TIMESTEP)
    p.loadURDF("plane.urdf")

    robot_id = p.loadURDF(
        cfg.urdf_path,
        basePosition=[0, 0, cfg.body_height + 0.02],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=False,
    )
    joint_map = build_joint_map(robot_id)
    # stance_rad is already per-leg; reset + motor-command from the same dict.
    reset_to_stance(robot_id, joint_map, cfg.stance_rad)
    apply_leg_pose(
        robot_id, joint_map, cfg.stance_rad, cfg.servo_force, cfg.servo_velocity
    )

    # Friction on the foot (link3 = the knee joint's child = lower leg/foot).
    # NB: joint_map keys are URDF joint names (`*_link3_joint`), which contain
    # "link3", not "knee" — the old "knee" match never fired, so the feet sat at
    # PyBullet's default lateralFriction 0.5 and slipped during planted-feet
    # moves (e.g. the body wiggle). Match "link3" so the intended grip applies.
    for name, idx in joint_map.items():
        if "link3" in name:
            p.changeDynamics(robot_id, idx, lateralFriction=1.5, restitution=0.0)

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


def run_stand(cfg, gui=True, settle_s=0.5):
    robot_id, joint_map = _connect_and_setup(cfg, gui)
    _print_banner(cfg)
    if settle_s > 0:
        print(f"\n[settle] holding stance for {settle_s:.2f}s before idle loop")
        _settle(robot_id, joint_map, cfg, settle_s)
    print("\nStanding - Ctrl+C to exit.")
    try:
        while p.isConnected():
            p.stepSimulation()
            if gui:
                time.sleep(TIMESTEP)
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if p.isConnected():
            p.disconnect()


def run_clip(cfg, clip_name, gui=True, settle_s=0.5, loop=False):
    """Load and play an animation clip by name in PyBullet.

    Looks up clip_name in animation/exported_clips/clips_all.h,
    settles the robot to stance, then plays the clip via ClipPlayer.
    loop=True (GUI only) replays the clip continuously so you can watch
    cumulative behaviour over time; physics state carries across loops.
    """
    from pybullet_interpreter.clip_loader import (
        DEFAULT_CLIPS_H,
        get_clip_by_name,
        load_clips_all_h,
    )
    from pybullet_interpreter.clip_player import ClipPlayer

    clips = load_clips_all_h(DEFAULT_CLIPS_H)
    clip = get_clip_by_name(clips, clip_name)

    robot_id, joint_map = _connect_and_setup(cfg, gui)
    _print_banner(cfg)
    if settle_s > 0:
        print(f"\n[settle] holding stance for {settle_s:.2f}s before clip")
        _settle(robot_id, joint_map, cfg, settle_s)

    loop_note = " (looping)" if loop and gui else ""
    print(f"\n[clip] playing '{clip.name}' ({clip.duration_ms} ms){loop_note}")
    player = ClipPlayer(robot_id, joint_map, clip, cfg.servo_force, cfg.servo_velocity)
    try:
        player.play_blocking(gui=gui, loop=loop)
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if p.isConnected():
            p.disconnect()


def run_gait(cfg, gait_name, gui=True, settle_s=0.5):
    if gait_name not in GAITS:
        raise ValueError(f"Unknown gait: {gait_name}")
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

    robot_id, joint_map = _connect_and_setup(cfg, gui)
    _print_banner(cfg)
    if settle_s > 0:
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
            if gui:
                time.sleep(TIMESTEP)
            t += TIMESTEP
            step += 1
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if p.isConnected():
            p.disconnect()
