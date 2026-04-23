"""Gait registry + run_stand / run_gait with foot-trajectory debug overlay."""

import math
import time

import pybullet as p
import pybullet_data

from constants import (
    HIP_ANGLE, KNEE_ANGLE, SHOULDER_ANGLE, STANCE,
    SERVO_FORCE, SERVO_VELOCITY, TIMESTEP, URDF_PATH,
)
from helpers import (
    all_legs, apply_per_leg_pose, build_joint_map, reset_to_stance,
)
from kinematics import (
    LEG_INFO, NEUTRAL_FOOT, foot_target, gait_joint_targets, leg_ik,
)


def pre_orient_splay(robot_id, joint_map, cfg, gui=True):
    """Ramp shoulder joints from 0 to splay_signs[leg] * shoulder_splay over
    `pre_orient` seconds, keeping hip/knee at STANCE. No-op if either the
    splay or pre_orient duration is zero."""
    pre_orient = cfg.get("pre_orient", 0.0)
    splay = cfg.get("shoulder_splay", 0.0)
    splay_signs = cfg.get("splay_signs", {})
    if pre_orient <= 0.0 or splay == 0.0:
        return
    print(f"  [pre-orient] splaying shoulders to "
          f"{math.degrees(splay):+.0f} deg over {pre_orient:.1f}s")
    n_steps = int(pre_orient / TIMESTEP)
    for k in range(n_steps):
        if not p.isConnected():
            break
        alpha = (k + 1) / n_steps
        for leg_id in LEG_INFO:
            target = alpha * splay_signs.get(leg_id, 0) * splay
            idx = joint_map.get(f"{leg_id}_shoulder_joint")
            if idx is not None:
                p.setJointMotorControl2(
                    robot_id, idx, p.POSITION_CONTROL,
                    targetPosition=target,
                    force=SERVO_FORCE, maxVelocity=SERVO_VELOCITY,
                )
            for joint in ("hip", "knee"):
                jidx = joint_map.get(f"{leg_id}_{joint}_joint")
                if jidx is not None:
                    p.setJointMotorControl2(
                        robot_id, jidx, p.POSITION_CONTROL,
                        targetPosition=STANCE[joint],
                        force=SERVO_FORCE, maxVelocity=SERVO_VELOCITY,
                    )
        p.stepSimulation()
        if gui:
            time.sleep(TIMESTEP)


# Gait registry. Each entry is a self-contained scheduler:
#   - period / step_length / step_height / duty: trajectory shape
#   - offsets: per-leg phase in [0, 1)
#   - label: printed in the banner
GAITS = {
    # Tuned 2026-04: doubled/quadrupled ground speeds while keeping the robot
    # within servo spec (force 1.47 N.m, velocity 5 rad/s). Raising the servo
    # velocity cap actually hurt trot speed in sim (legs chase targets faster
    # than the physics step resolves contacts, so the foot slips), so only
    # the trajectory parameters were changed.
    "walk": {
        "period":      1.2,       # was 2.4  -> 2x cycle rate
        "step_length": 0.05,      # was 0.04
        "step_height": 0.025,     # was 0.02
        "duty":        0.25,
        "offsets":     {"fl": 0.00, "rr": 0.25, "fr": 0.50, "rl": 0.75},
        # Default 30 deg outward splay widens the Y footprint from 100 -> 180 mm
        # to compensate for the smaller FlexibleSkeleton body mounts.
        "shoulder_splay":   math.radians(30.0),
        "splay_signs":      {"fl": -1, "fr": +1, "rl": +1, "rr": -1},
        "pre_orient":       0.8,
        "label":       "Static walk (FL -> RR -> FR -> RL)",
    },
    "trot": {
        # Keep step_height at the old 0.025 so the foot doesn't overshoot
        # small obstacles -- over-tall steps caused the fast trot to bounce
        # off the terrain. Shorter period + slightly longer stride gives
        # ~13 cm/s on flat ground without destabilising bumps/step traversal.
        "period":      0.50,      # was 0.8   -> 1.6x cycle rate
        "step_length": 0.065,     # was 0.05  -> 1.3x stride
        "step_height": 0.025,
        "duty":        0.5,
        "offsets":     {"fl": 0.0, "rr": 0.0, "fr": 0.5, "rl": 0.5},
        "shoulder_splay":   math.radians(30.0),
        "splay_signs":      {"fl": -1, "fr": +1, "rl": +1, "rr": -1},
        "pre_orient":       0.8,
        "label":       "Trot (diagonal pairs: FL+RR | FR+RL)",
    },
    "bound": {
        "period":      0.5,
        # Negative step_length flips the swing/stance direction so the push-
        # off actually propels the body forward. With positive step_length
        # the synchronous front/rear pair timing made the robot drift in -Y.
        "step_length": -0.055,
        "step_height": 0.035,
        "duty":        0.4,
        "offsets":     {"fl": 0.0, "fr": 0.0, "rl": 0.5, "rr": 0.5},
        "shoulder_splay":   math.radians(30.0),
        "splay_signs":      {"fl": -1, "fr": +1, "rl": +1, "rr": -1},
        "pre_orient":       0.8,
        "label":       "Bound (front pair, then rear pair)",
    },
    "crab": {
        # Sideways gait driven by HIP PITCH on splayed legs. With shoulders
        # yawed 45 deg, the hip swing plane is rotated 45 deg -- a fore/aft
        # stride in that plane puts each foot on a diagonal in body frame.
        # Trot-style diagonal pairs keep two feet on the ground at all times.
        "period":      0.55,
        "step_length": 0.07,
        "step_height": 0.028,
        "duty":        0.55,
        "axis":        "x",
        "offsets":     {"fl": 0.0, "rr": 0.0, "fr": 0.5, "rl": 0.5},
        # Legs splay 45 deg outward (X pattern). The splay branch rotates the
        # rest foot around the shoulder mount; regular leg_ik then resolves
        # shoulder yaw so the splay is kinematically honoured.
        "shoulder_splay":   math.radians(45.0),
        "splay_signs":      {"fl": -1, "fr": +1, "rl": +1, "rr": -1},
        # Warmup: ramp shoulders from 0 to splay over this many seconds before
        # starting the stride cycle.
        "pre_orient":       1.2,
        "label":       "Crab walk (sideways, 45 deg splay)",
    },
}

# Debug overlay of planned foot trajectories in the GUI
SHOW_FOOT_TRAJECTORIES = True
_TRAJ_COLORS = {
    "fl": (1.0, 0.30, 0.30),
    "fr": (0.30, 1.0, 0.30),
    "rl": (0.30, 0.50, 1.0),
    "rr": (1.0, 1.0, 0.30),
}
_TRAJ_SAMPLES = 40


def _body_to_world(pos_body, base_pos, base_orn):
    """Transform a point from the robot's body frame into world frame."""
    world_pos, _ = p.multiplyTransforms(base_pos, base_orn, pos_body, [0, 0, 0, 1])
    return world_pos


def _precompute_foot_cycle(step_length, step_height, duty, axis="y",
                           samples=_TRAJ_SAMPLES):
    """Pre-sample one full cycle of body-frame foot positions per leg."""
    cycles = {}
    for leg_id in LEG_INFO:
        pts = []
        for k in range(samples):
            phase = k / samples
            pts.append(foot_target(leg_id, phase, step_length, step_height,
                                   duty, axis))
        pts.append(pts[0])  # close the loop
        cycles[leg_id] = pts
    return cycles


def _draw_foot_trajectories(robot_id, cycles, line_ids, marker_ids, current_targets):
    """Redraw trajectory loops + current-target crosses in world frame."""
    base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
    for leg_id, pts in cycles.items():
        color = _TRAJ_COLORS[leg_id]
        prev_world = _body_to_world(pts[0], base_pos, base_orn)
        for k in range(1, len(pts)):
            cur_world = _body_to_world(pts[k], base_pos, base_orn)
            key = (leg_id, k - 1)
            existing = line_ids.get(key)
            if existing is None:
                line_ids[key] = p.addUserDebugLine(prev_world, cur_world,
                                                  lineColorRGB=color,
                                                  lineWidth=1.5)
            else:
                p.addUserDebugLine(prev_world, cur_world,
                                   lineColorRGB=color, lineWidth=1.5,
                                   replaceItemUniqueId=existing)
            prev_world = cur_world

        target_world = _body_to_world(current_targets[leg_id], base_pos, base_orn)
        s = 0.012
        axes = [((-s, 0, 0), (s, 0, 0)),
                ((0, -s, 0), (0, s, 0)),
                ((0, 0, -s), (0, 0, s))]
        for i, (a, b) in enumerate(axes):
            p0 = (target_world[0] + a[0], target_world[1] + a[1], target_world[2] + a[2])
            p1 = (target_world[0] + b[0], target_world[1] + b[1], target_world[2] + b[2])
            mkey = (leg_id, i)
            existing = marker_ids.get(mkey)
            if existing is None:
                marker_ids[mkey] = p.addUserDebugLine(p0, p1,
                                                     lineColorRGB=color,
                                                     lineWidth=3.0)
            else:
                p.addUserDebugLine(p0, p1, lineColorRGB=color, lineWidth=3.0,
                                   replaceItemUniqueId=existing)


def _verify_neutral_ik():
    """Sanity check: IK of neutral foot should reproduce STANCE within 1 deg."""
    tol = math.radians(1.5)
    for leg_id, foot in NEUTRAL_FOOT.items():
        s, h, k = leg_ik(foot, leg_id)
        errs = (abs(s - SHOULDER_ANGLE), abs(h - HIP_ANGLE), abs(k - KNEE_ANGLE))
        ok = all(e < tol for e in errs)
        tag = "OK" if ok else "FAIL"
        print(f"  IK check {leg_id}: s={math.degrees(s):+6.2f}  "
              f"h={math.degrees(h):+6.2f}  k={math.degrees(k):+6.2f}  [{tag}]")


def run_stand(gui=True):
    p.connect(p.GUI if gui else p.DIRECT)
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    robot_id = p.loadURDF(
        URDF_PATH,
        basePosition=[0, 0, 0.20],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=False,
    )
    joint_map = build_joint_map(robot_id)

    reset_to_stance(robot_id, joint_map, STANCE)
    apply_per_leg_pose(robot_id, joint_map, all_legs(STANCE))

    for joint_name, joint_idx in joint_map.items():
        if "knee" in joint_name:
            p.changeDynamics(robot_id, joint_idx,
                             lateralFriction=1.0, restitution=0.1)

    p.resetDebugVisualizerCamera(
        cameraDistance=0.5, cameraYaw=45, cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.1],
    )
    p.setTimeStep(TIMESTEP)

    print("\nSimulation running - press Ctrl+C to exit")
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


def run_gait(gait_name, gui=True):
    """Run a named gait from the GAITS registry."""
    if gait_name not in GAITS:
        raise ValueError(
            f"Unknown gait '{gait_name}'. Known: {sorted(GAITS)}"
        )
    cfg = GAITS[gait_name]

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

    # Body height = distance from foot tip to body origin given stance.
    body_height = -NEUTRAL_FOOT["fl"][2]
    robot_id = p.loadURDF(
        URDF_PATH,
        basePosition=[0, 0, body_height],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=False,
    )
    joint_map = build_joint_map(robot_id)

    reset_to_stance(robot_id, joint_map, STANCE)
    apply_per_leg_pose(robot_id, joint_map, all_legs(STANCE))

    for joint_name, joint_idx in joint_map.items():
        if "knee" in joint_name:
            p.changeDynamics(robot_id, joint_idx,
                             lateralFriction=1.0, restitution=0.1)

    p.resetDebugVisualizerCamera(
        cameraDistance=0.65, cameraYaw=45, cameraPitch=-25,
        cameraTargetPosition=[0, 0, 0.1],
    )

    print(f"\n=== FaceHugger {cfg['label']} ===")
    print(f"  period={cfg['period']:.2f}s  step_len={cfg['step_length']*1000:.0f}mm  "
          f"step_h={cfg['step_height']*1000:.0f}mm  duty={cfg['duty']:.2f}")
    _verify_neutral_ik()

    cycles = _precompute_foot_cycle(cfg["step_length"], cfg["step_height"],
                                    cfg["duty"], cfg.get("axis", "y"))
    line_ids = {}
    marker_ids = {}
    draw_overlay = gui and SHOW_FOOT_TRAJECTORIES
    draw_every = 4  # refresh overlay every N sim steps to keep GUI snappy

    pre_orient_splay(robot_id, joint_map, cfg, gui=gui)

    t = 0.0
    step_count = 0
    try:
        while p.isConnected():
            targets = gait_joint_targets(t, cfg)
            for joint_name, angle in targets.items():
                idx = joint_map.get(joint_name)
                if idx is None:
                    continue
                p.setJointMotorControl2(
                    robot_id, idx, p.POSITION_CONTROL,
                    targetPosition=angle,
                    force=SERVO_FORCE, maxVelocity=SERVO_VELOCITY,
                )

            if draw_overlay and step_count % draw_every == 0:
                current_targets = {}
                global_phase = (t / cfg["period"]) % 1.0
                for leg_id, offset in cfg["offsets"].items():
                    phase = (global_phase - offset) % 1.0
                    current_targets[leg_id] = foot_target(
                        leg_id, phase,
                        cfg["step_length"], cfg["step_height"], cfg["duty"],
                    )
                _draw_foot_trajectories(robot_id, cycles, line_ids,
                                        marker_ids, current_targets)

            p.stepSimulation()
            if gui:
                time.sleep(TIMESTEP)
            t += TIMESTEP
            step_count += 1
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if p.isConnected():
            p.disconnect()
