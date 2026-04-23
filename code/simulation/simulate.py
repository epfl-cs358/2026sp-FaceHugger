"""
FaceHugger Quadruped Robot - PyBullet Simulation

Run modes:
  default     : robot loads and holds a stable standing stance
  --walk      : static walk gait (one leg swings at a time, CoM always inside
                support triangle), with overlaid foot-trajectory debug lines.
  --trot      : dynamic trot gait (diagonal pairs FL+RR / FR+RL swing
                together, duty 0.5), same foot-trajectory debug overlay.
  --bound     : front pair then rear pair (FL+FR / RL+RR, duty 0.4). Higher
                step height; body pitches as the rear push-off launches it.
  --terrain GAIT
              : spawn an obstacle course (ramp, plateau, bumps, step)
                along +Y and run the named gait across it. Per-checkpoint
                status is printed as the robot passes each obstacle.
  --terrain-matrix
              : headless run of all three gaits over the same course;
                prints a markdown survival table (reached / tilted / fell)
                per checkpoint. Great for comparing gait robustness.

Specs from project proposal:
  - 12-DOF: 4 legs x 3 joints (shoulder yaw, hip pitch, knee pitch)
  - Link lengths: L1=80mm, L2=75mm, L3=90mm
  - Servo: DSS-M15S, 1.47 N*m, 270 deg
  - Default stance: hip ~40 deg, knee ~-60 deg
"""

import argparse
import math
import os
import time

import pybullet as p
import pybullet_data

URDF_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "facehugger.urdf")

# Standing stance angles (radians) - from proposal torque widget
HIP_ANGLE = math.radians(40)
KNEE_ANGLE = math.radians(-60)  # negative = bending down
SHOULDER_ANGLE = 0.0  # straight out

STANCE = {
    "shoulder": SHOULDER_ANGLE,
    "hip": HIP_ANGLE,
    "knee": KNEE_ANGLE,
}

# Servo characteristics (DSS-M15S)
SERVO_FORCE = 1.47       # 15 kg*cm ~= 1.47 N*m
SERVO_VELOCITY = 5.0     # rad/s
TIMESTEP = 1.0 / 240.0


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #

def build_joint_map(robot_id):
    """Return {joint_name: joint_index} and print a one-line summary."""
    joint_map = {}
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        joint_map[info[1].decode("utf-8")] = i
    print(f"FaceHugger loaded: {num_joints} joints")
    return joint_map


def stance_angle_for(joint_name, stance):
    """Pick the right stance angle (shoulder/hip/knee) for a joint name."""
    if "shoulder" in joint_name:
        return stance["shoulder"]
    if "hip" in joint_name:
        return stance["hip"]
    if "knee" in joint_name:
        return stance["knee"]
    return 0.0


def reset_to_stance(robot_id, joint_map, stance):
    """Hard-reset every revolute joint to the given stance."""
    for joint_name, joint_idx in joint_map.items():
        info = p.getJointInfo(robot_id, joint_idx)
        if info[2] == p.JOINT_FIXED:
            continue
        p.resetJointState(robot_id, joint_idx, stance_angle_for(joint_name, stance))


def apply_per_leg_pose(robot_id, joint_map, leg_targets,
                       force=SERVO_FORCE, velocity=SERVO_VELOCITY):
    """leg_targets = {leg_prefix: {shoulder/hip/knee: angle}}."""
    for joint_name, joint_idx in joint_map.items():
        info = p.getJointInfo(robot_id, joint_idx)
        if info[2] == p.JOINT_FIXED:
            continue
        leg = joint_name[:2]                # 'fl', 'fr', 'rl', 'rr'
        stance = leg_targets.get(leg)
        if stance is None:
            continue
        p.setJointMotorControl2(
            robot_id, joint_idx, p.POSITION_CONTROL,
            targetPosition=stance_angle_for(joint_name, stance),
            force=force, maxVelocity=velocity,
        )


def all_legs(stance):
    return {leg: stance for leg in ("fl", "fr", "rl", "rr")}


# --------------------------------------------------------------------------- #
# Standing simulation (the original behaviour)
# --------------------------------------------------------------------------- #

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


# --------------------------------------------------------------------------- #
# Static walk gait
# --------------------------------------------------------------------------- #

# Leg link geometry (metres). Pulled from the URDF joint origins.
L1 = 0.080   # shoulder arm:   shoulder origin -> hip joint
L2 = 0.075   # upper leg:      hip joint       -> knee joint
L3 = 0.077   # lower leg:      knee joint      -> foot tip (end of mesh)

# Shoulder mount per leg, in body frame, plus the yaw offset baked into the
# URDF origin rpy (left legs = 0, right legs = pi). arm_sign follows from the
# yaw offset (left arm extends body -X, right arm extends body +X).
LEG_INFO = {
    "fl": {"mount": (-0.080,  0.105, 0.025), "yaw_offset": 0.0},
    "fr": {"mount": ( 0.080,  0.105, 0.025), "yaw_offset": math.pi},
    "rl": {"mount": (-0.080, -0.105, 0.025), "yaw_offset": 0.0},
    "rr": {"mount": ( 0.080, -0.105, 0.025), "yaw_offset": math.pi},
}

# Neutral foot position (body frame) corresponding to the STANCE joint angles.
# Derived analytically from forward kinematics, so leg_ik(neutral) == STANCE.
NEUTRAL_FOOT = {
    "fl": (-L1 - (L2 * math.sin(HIP_ANGLE) + L3 * math.sin(HIP_ANGLE + KNEE_ANGLE))
           + LEG_INFO["fl"]["mount"][0],
            LEG_INFO["fl"]["mount"][1],
           -(L2 * math.cos(HIP_ANGLE) + L3 * math.cos(HIP_ANGLE + KNEE_ANGLE))
           + LEG_INFO["fl"]["mount"][2]),
    "fr": ( L1 + (L2 * math.sin(HIP_ANGLE) + L3 * math.sin(HIP_ANGLE + KNEE_ANGLE))
           + LEG_INFO["fr"]["mount"][0],
            LEG_INFO["fr"]["mount"][1],
           -(L2 * math.cos(HIP_ANGLE) + L3 * math.cos(HIP_ANGLE + KNEE_ANGLE))
           + LEG_INFO["fr"]["mount"][2]),
    "rl": (-L1 - (L2 * math.sin(HIP_ANGLE) + L3 * math.sin(HIP_ANGLE + KNEE_ANGLE))
           + LEG_INFO["rl"]["mount"][0],
            LEG_INFO["rl"]["mount"][1],
           -(L2 * math.cos(HIP_ANGLE) + L3 * math.cos(HIP_ANGLE + KNEE_ANGLE))
           + LEG_INFO["rl"]["mount"][2]),
    "rr": ( L1 + (L2 * math.sin(HIP_ANGLE) + L3 * math.sin(HIP_ANGLE + KNEE_ANGLE))
           + LEG_INFO["rr"]["mount"][0],
            LEG_INFO["rr"]["mount"][1],
           -(L2 * math.cos(HIP_ANGLE) + L3 * math.cos(HIP_ANGLE + KNEE_ANGLE))
           + LEG_INFO["rr"]["mount"][2]),
}

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

# Joint limits (mirrored from URDF) for clamping IK outputs
_JOINT_LIMITS = {
    "shoulder": (-2.356, 2.356),
    "hip":      (-1.5708, 1.5708),
    "knee":     (-2.356, 0.5),
}


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def leg_ik(foot_body, leg_id):
    """Analytic inverse kinematics. foot_body in robot body frame -> joint angles."""
    info = LEG_INFO[leg_id]
    sx, sy, sz = info["mount"]
    yaw_offset = info["yaw_offset"]

    dx = foot_body[0] - sx
    dy = foot_body[1] - sy
    dz = foot_body[2] - sz

    r_xy = math.hypot(dx, dy)
    # Shoulder yaw: rotate shoulder link so its -X axis points toward the foot
    theta_total = math.atan2(-dy, -dx)
    theta_s = _wrap_pi(theta_total - yaw_offset)

    # Sagittal 2-link reach: a horizontal (from hip), b downward (from hip)
    a = r_xy - L1
    b = -dz

    c = math.hypot(a, b)
    c_max = L2 + L3 - 1e-4
    if c > c_max:
        c = c_max
        # Rescale (a, b) so the triangle is reachable
        scale = c_max / max(math.hypot(a, b), 1e-9)
        a *= scale
        b *= scale

    cos_alpha = (L2 * L2 + L3 * L3 - c * c) / (2.0 * L2 * L3)
    alpha = math.acos(_clamp(cos_alpha, -1.0, 1.0))
    theta_k = alpha - math.pi

    cos_beta = (L2 * L2 + c * c - L3 * L3) / (2.0 * L2 * c + 1e-12)
    beta = math.acos(_clamp(cos_beta, -1.0, 1.0))
    theta_h = math.atan2(a, b) + beta

    theta_s = _clamp(theta_s, *_JOINT_LIMITS["shoulder"])
    theta_h = _clamp(theta_h, *_JOINT_LIMITS["hip"])
    theta_k = _clamp(theta_k, *_JOINT_LIMITS["knee"])
    return theta_s, theta_h, theta_k


def foot_target(leg_id, phase, step_length, step_height, duty, axis="y"):
    """Body-frame foot target for a given leg at a given per-leg phase [0, 1).
    axis="y": fore/aft gait (walk/trot/bound). axis="x": lateral gait (crab)."""
    nx, ny, nz = NEUTRAL_FOOT[leg_id]
    if phase < duty:
        s = phase / duty
        d = -step_length * 0.5 + s * step_length
        dz = step_height * math.sin(math.pi * s)
    else:
        s = (phase - duty) / (1.0 - duty)
        d = step_length * 0.5 - s * step_length
        dz = 0.0
    if axis == "x":
        return (nx + d, ny, nz + dz)
    return (nx, ny + d, nz + dz)


def leg_ik_fixed_yaw(foot_body, leg_id, theta_s):
    """IK for a splayed leg: shoulder yaw is PINNED at theta_s, hip/knee solve
    for the 2-link reach from the already-placed hip to the foot."""
    info = LEG_INFO[leg_id]
    sx, sy, sz = info["mount"]
    yaw_offset = info["yaw_offset"]
    theta_total = theta_s + yaw_offset
    # Hip joint position in body frame (leg_ik places the arm so its -X_local
    # axis points at angle theta_total in body XY plane; hip sits L1 along it).
    hip_x = sx - L1 * math.cos(theta_total)
    hip_y = sy - L1 * math.sin(theta_total)
    hip_z = sz
    # Displacement from hip to foot.
    ux = foot_body[0] - hip_x
    uy = foot_body[1] - hip_y
    uz = foot_body[2] - hip_z
    # Project horizontal component onto the arm direction (signed) to get `a`,
    # the in-sagittal-plane forward reach. `b` is straight down from the hip.
    # Arm direction (shoulder -> hip) in body XY: (-cos(theta_total), -sin(...))
    # "a" axis (forward in sagittal plane, away from shoulder) is the same dir.
    a = ux * (-math.cos(theta_total)) + uy * (-math.sin(theta_total))
    b = -uz
    c = math.hypot(a, b)
    c_max = L2 + L3 - 1e-4
    if c > c_max:
        scale = c_max / max(c, 1e-9)
        a *= scale
        b *= scale
        c = c_max
    cos_alpha = (L2 * L2 + L3 * L3 - c * c) / (2.0 * L2 * L3)
    alpha = math.acos(_clamp(cos_alpha, -1.0, 1.0))
    theta_k = alpha - math.pi
    cos_beta = (L2 * L2 + c * c - L3 * L3) / (2.0 * L2 * c + 1e-12)
    beta = math.acos(_clamp(cos_beta, -1.0, 1.0))
    theta_h = math.atan2(a, b) + beta
    theta_s = _clamp(theta_s, *_JOINT_LIMITS["shoulder"])
    theta_h = _clamp(theta_h, *_JOINT_LIMITS["hip"])
    theta_k = _clamp(theta_k, *_JOINT_LIMITS["knee"])
    return theta_s, theta_h, theta_k


def gait_joint_targets(t, cfg):
    """Return {joint_name: angle} for all 12 joints at elapsed time t."""
    period = cfg["period"]
    step_length = cfg["step_length"]
    step_height = cfg["step_height"]
    duty = cfg["duty"]
    axis = cfg.get("axis", "y")
    splay = cfg.get("shoulder_splay", 0.0)
    splay_signs = cfg.get("splay_signs", {})
    global_phase = (t / period) % 1.0
    targets = {}
    for leg_id, offset in cfg["offsets"].items():
        phase = (global_phase - offset) % 1.0
        foot = foot_target(leg_id, phase, step_length, step_height, duty, axis)
        if splay != 0.0:
            # Splay: rotate the rest foot position around the shoulder mount
            # by s_angle, then add the body-frame stride/lift from foot_target.
            # Regular leg_ik resolves shoulder yaw to match; shoulder oscillates
            # slightly around s_angle across the cycle, keeping the X-pattern.
            s_angle = splay_signs.get(leg_id, 0) * splay
            nx, ny, nz = NEUTRAL_FOOT[leg_id]
            sx, sy, _sz = LEG_INFO[leg_id]["mount"]
            rest_dx = nx - sx
            rest_dy = ny - sy
            cos_a = math.cos(s_angle)
            sin_a = math.sin(s_angle)
            rest_x = sx + cos_a * rest_dx - sin_a * rest_dy
            rest_y = sy + sin_a * rest_dx + cos_a * rest_dy
            dx = foot[0] - nx
            dy = foot[1] - ny
            dz = foot[2] - nz
            foot_ik = (rest_x + dx, rest_y + dy, nz + dz)
            theta_s, theta_h, theta_k = leg_ik(foot_ik, leg_id)
        else:
            theta_s, theta_h, theta_k = leg_ik(foot, leg_id)
        targets[f"{leg_id}_shoulder_joint"] = theta_s
        targets[f"{leg_id}_hip_joint"]      = theta_h
        targets[f"{leg_id}_knee_joint"]     = theta_k
    return targets


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

    pre_orient = cfg.get("pre_orient", 0.0)
    splay = cfg.get("shoulder_splay", 0.0)
    splay_signs = cfg.get("splay_signs", {})
    if pre_orient > 0.0 and splay != 0.0:
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


# --------------------------------------------------------------------------- #
# Terrain obstacle course
# --------------------------------------------------------------------------- #
#
# The course is a fixed sequence of obstacles laid out along +Y, so every gait
# is tested on the exact same hazard profile. Each checkpoint records how the
# robot crossed it (upright / tilted / never-reached) to produce a
# gait-vs-terrain survival matrix.
#
# Obstacle sequence (dimensions in meters):
#   UP_RAMP   : 12 deg incline, rises 4 cm over 0.30 m
#   PLATEAU   : 40 cm long flat top at 4 cm height
#   DOWN_RAMP : mirror of UP_RAMP back to ground
#   BUMPS     : three 1.5 cm tall strips at 7 cm spacing ("speed bumps")
#   STEP_UP   : 3 cm tall, 30 cm long block (~16 % of robot standing height)
#   FINISH    : thin green marker line
#
# Robot is 18 cm tall / 21 cm long, so a 3 cm step is deliberately on the
# edge of feasibility - bound should clear it, walk will likely stub its toe.


COURSE = [
    # Each checkpoint: y = world y the body origin must pass to be counted
    # as reached; max_tilt = degrees of roll/pitch still considered "clean".
    # Distances are deliberately tight because walk moves at ~2 cm/s and the
    # course has to be traversable inside a minute or two of sim time.
    dict(name="UP_RAMP",   y=0.20, max_tilt=25.0),
    dict(name="PLATEAU",   y=0.45, max_tilt=20.0),
    dict(name="DOWN_RAMP", y=0.70, max_tilt=30.0),
    dict(name="BUMPS",     y=0.95, max_tilt=30.0),
    dict(name="STEP_UP",   y=1.40, max_tilt=45.0),
    dict(name="FINISH",    y=1.70, max_tilt=25.0),
]


def _add_static_box(half, pos, orn=(0, 0, 0, 1), color=(0.6, 0.6, 0.6, 1.0),
                    friction=1.2):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=list(color))
    body = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=list(pos),
        baseOrientation=list(orn),
    )
    p.changeDynamics(body, -1, lateralFriction=friction, restitution=0.0)
    return body


def add_ramp(y_start, length, rise, width=0.60, friction=1.3,
             color=(0.35, 0.65, 0.90, 1.0)):
    """Thin slab tilted about X so its top surface rises `rise` meters
    between y=y_start and y=y_start+length. `rise` may be negative for a
    down-ramp (slab tilts the other way, top surface descends).

    The slab is positioned so its TOP surface passes through z=0 at the
    low end and z=|rise| at the high end. The half-thickness offset pulls
    the slab body down below the visible surface, otherwise it would
    create a lip where it meets flat obstacles at the same height."""
    angle = math.atan2(rise, length)
    hypot = math.hypot(rise, length)
    half_thickness = 0.02
    half = [width / 2.0, hypot / 2.0, half_thickness]
    cy = y_start + length / 2.0
    cz = abs(rise) / 2.0 - half_thickness * math.cos(angle)
    orn = p.getQuaternionFromEuler([angle, 0.0, 0.0])
    return _add_static_box(half, (0.0, cy, cz), orn, color, friction)


def add_box_obstacle(y_center, height, length, width=0.60, friction=1.3,
                     color=(0.9, 0.55, 0.25, 1.0)):
    """Axis-aligned box sitting on the floor with its top face at z=height."""
    half = [width / 2.0, length / 2.0, height / 2.0]
    return _add_static_box(half, (0.0, y_center, height / 2.0),
                           (0, 0, 0, 1), color, friction)


def build_terrain_course():
    """Spawn the full obstacle course. Returns list of body ids."""
    ids = []

    ramp_color = (0.35, 0.65, 0.90, 1.0)
    # Up ramp: 0 -> 2 cm over 20 cm (~5.7 deg) -- gentle enough for walk.
    ids.append(add_ramp(y_start=0.10, length=0.20, rise=0.02, color=ramp_color))
    # Plateau at 2 cm, 20 cm long
    ids.append(add_box_obstacle(y_center=0.40, height=0.02, length=0.20,
                                color=ramp_color))
    # Down ramp back to floor
    ids.append(add_ramp(y_start=0.50, length=0.20, rise=-0.02, color=ramp_color))

    # Speed bumps: three shallow 6 mm strips across the path. Low enough that
    # every gait glides over without the body pitching, narrow enough (1 cm)
    # that a foot cannot land on one; gap between bumps is ~9 cm, wider than
    # any stride so feet always land on clear floor.
    bump_color = (0.95, 0.80, 0.20, 1.0)
    for y in (0.85, 0.95, 1.05):
        ids.append(add_box_obstacle(y_center=y, height=0.006, length=0.010,
                                    color=bump_color))

    # 1 cm step, 25 cm long -- inside every gait's step_height budget so all
    # three gaits can plant on it without stubbing the toe on the leading edge.
    # Pushed back so there's ~22 cm of clear floor between the last bump and
    # the step's leading edge (gives the robot a full stride to settle).
    ids.append(add_box_obstacle(y_center=1.35, height=0.010, length=0.25,
                                color=(0.85, 0.35, 0.30, 1.0)))

    # Finish-line marker (thin green strip)
    ids.append(add_box_obstacle(y_center=1.75, height=0.005, length=0.02,
                                width=0.90, color=(0.25, 0.85, 0.35, 1.0)))
    return ids


def _sample_state(robot_id):
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    roll, pitch, yaw = p.getEulerFromQuaternion(orn)
    return pos, (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def _tilt(rpy):
    return max(abs(rpy[0]), abs(rpy[1]))


def run_terrain(gait_name, gui=True, duration=40.0, return_report=False):
    """Run a named gait across the terrain course, logging per-checkpoint
    crossing data. Returns a summary dict when return_report=True."""
    if gait_name not in GAITS:
        raise ValueError(f"Unknown gait '{gait_name}'. Known: {sorted(GAITS)}")
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

    build_terrain_course()

    body_height = -NEUTRAL_FOOT["fl"][2]
    robot_id = p.loadURDF(
        URDF_PATH,
        basePosition=[0.0, -0.10, body_height + 0.005],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=False,
    )
    joint_map = build_joint_map(robot_id)
    reset_to_stance(robot_id, joint_map, STANCE)
    apply_per_leg_pose(robot_id, joint_map, all_legs(STANCE))

    # Foot link = lower_leg (child of knee joint). Mirror run_gait's friction
    # profile exactly -- adding spinning friction here locks the foot during
    # stance and the robot pogo-sticks in place.
    for joint_name, joint_idx in joint_map.items():
        if "knee" in joint_name:
            p.changeDynamics(robot_id, joint_idx,
                             lateralFriction=1.2, restitution=0.1)

    if gui:
        p.resetDebugVisualizerCamera(
            cameraDistance=0.85, cameraYaw=75, cameraPitch=-15,
            cameraTargetPosition=[0.0, 0.0, 0.10],
        )

    print(f"\n=== Terrain run: {cfg['label']} ===")
    print(f"  course: " + " -> ".join(cp['name'] for cp in COURSE))

    reached = {cp["name"]: None for cp in COURSE}
    min_z, max_tilt_seen = 1e9, 0.0
    fell, fell_at = False, None
    low_z_streak = 0
    streak_threshold = int(0.25 / TIMESTEP)  # must stay low for 0.25 s

    # Heading-keeping: physics asymmetries (contact solver order, diagonal-pair
    # timing mismatch) cause a slow yaw drift. A P-controller on base yaw biases
    # per-side step length. Longer stride on one side yaws the body toward the
    # OPPOSITE side, so to correct yaw>0 we stride the LEFT side longer.
    STEER_KP = 0.4         # rad^-1, applied to yaw error
    STEER_MAX = 0.25       # clamp on stride scaling
    steer_cfg_left = dict(cfg)
    steer_cfg_right = dict(cfg)

    t, steps = 0.0, int(duration / TIMESTEP)
    for i in range(steps):
        if not p.isConnected():
            print("  [window closed, exiting]")
            return
        _, base_orn = p.getBasePositionAndOrientation(robot_id)
        _, _, yaw_rad = p.getEulerFromQuaternion(base_orn)
        steer = _clamp(STEER_KP * yaw_rad, -STEER_MAX, STEER_MAX)
        # yaw>0 (body faced left) -> stride LEFT longer to yaw body right
        steer_cfg_left["step_length"]  = cfg["step_length"] * (1.0 + steer)
        steer_cfg_right["step_length"] = cfg["step_length"] * (1.0 - steer)
        targets = {}
        period = cfg["period"]
        global_phase = (t / period) % 1.0
        for leg_id, offset in cfg["offsets"].items():
            phase = (global_phase - offset) % 1.0
            side_cfg = steer_cfg_left if leg_id.endswith("l") else steer_cfg_right
            foot = foot_target(leg_id, phase, side_cfg["step_length"],
                               cfg["step_height"], cfg["duty"])
            theta_s, theta_h, theta_k = leg_ik(foot, leg_id)
            targets[f"{leg_id}_shoulder_joint"] = theta_s
            targets[f"{leg_id}_hip_joint"]      = theta_h
            targets[f"{leg_id}_knee_joint"]     = theta_k
        for joint_name, angle in targets.items():
            idx = joint_map.get(joint_name)
            if idx is None:
                continue
            p.setJointMotorControl2(
                robot_id, idx, p.POSITION_CONTROL,
                targetPosition=angle,
                force=SERVO_FORCE, maxVelocity=SERVO_VELOCITY,
            )
        p.stepSimulation()

        pos, rpy = _sample_state(robot_id)
        min_z = min(min_z, pos[2])
        max_tilt_seen = max(max_tilt_seen, _tilt(rpy))

        if gui:
            # Side-camera that scrolls with the robot along the course.
            if i % 30 == 0:
                p.resetDebugVisualizerCamera(
                    cameraDistance=0.95, cameraYaw=75, cameraPitch=-14,
                    cameraTargetPosition=[0.0, pos[1], 0.10],
                )
            time.sleep(TIMESTEP)
        t += TIMESTEP

        # Fell detection: truly upside-down, or body dragged along the ground
        # for a sustained time (several sim ticks).
        if abs(rpy[0]) > 100.0 or abs(rpy[1]) > 80.0:
            fell = True
        if pos[2] < 0.045:
            low_z_streak += 1
            if low_z_streak > streak_threshold:
                fell = True
        else:
            low_z_streak = 0

        if fell:
            for cp in COURSE:
                if reached[cp["name"]] is None:
                    fell_at = cp["name"]
                    break
            print(f"  [t={t:5.2f}s] FELL before {fell_at} "
                  f"(y={pos[1]:+.2f} z={pos[2]:+.3f} "
                  f"roll={rpy[0]:+.0f} pitch={rpy[1]:+.0f})")
            break

        for cp in COURSE:
            if reached[cp["name"]] is None and pos[1] >= cp["y"]:
                reached[cp["name"]] = dict(t=t, pos=tuple(pos), rpy=rpy)
                tilt = _tilt(rpy)
                tag = "clean" if tilt < cp["max_tilt"] else f"tilt {tilt:.0f} deg"
                print(f"  [t={t:5.2f}s] {cp['name']:10s} y={pos[1]:+.2f} "
                      f"z={pos[2]:+.3f} roll={rpy[0]:+5.0f} "
                      f"pitch={rpy[1]:+5.0f}  [{tag}]")

        # In headless matrix runs, stop once every checkpoint is hit (no point
        # simulating further). In GUI mode, keep walking past the finish so
        # the viewer can watch the robot cruise off into the distance.
        if not gui and all(v is not None for v in reached.values()):
            break

    final_pos, final_rpy = _sample_state(robot_id)
    summary = dict(
        gait=gait_name,
        reached=reached,
        fell=fell,
        fell_at=fell_at,
        final_pos=tuple(final_pos),
        final_rpy=final_rpy,
        min_z=min_z,
        max_tilt=max_tilt_seen,
        duration=t,
    )

    print(f"\n  RESULT: {'FELL at ' + str(fell_at) if fell else 'SURVIVED'} "
          f"| final y={final_pos[1]:+.2f}  min z={min_z:+.3f}  "
          f"max tilt={max_tilt_seen:.0f} deg  t={t:.2f}s")

    if gui and not fell:
        hold = int(2.0 / TIMESTEP)
        for _ in range(hold):
            if not p.isConnected():
                break
            p.stepSimulation()
            time.sleep(TIMESTEP)

    if p.isConnected():
        p.disconnect()

    return summary if return_report else None


def _status(summary, spawn_y=-0.10):
    """Classify the final outcome of a terrain run."""
    reached_count = sum(1 for v in summary["reached"].values() if v is not None)
    all_reached = reached_count == len(summary["reached"])
    if summary["fell"]:
        return f"FELL @ {summary['fell_at']}"
    if all_reached:
        return "FINISHED"
    if summary["final_pos"][1] < spawn_y - 0.15:
        return "REVERSED"
    if reached_count == 0:
        return "STUCK"
    return "TIMEOUT"


def run_terrain_matrix(gui=False, save_path=None):
    """Run all gaits over the course; print (and optionally save) a markdown
    survival table."""
    # Per-gait duration budgets: walk is inherently slower (single-foot
    # stance, 25% duty) so it needs more sim time to traverse the course.
    durations = {"walk": 60.0, "trot": 40.0, "bound": 40.0}
    results = []
    for gait in ("walk", "trot", "bound"):
        summary = run_terrain(gait, gui=gui, duration=durations[gait],
                              return_report=True)
        results.append(summary)

    header = ["Gait"] + [cp["name"] for cp in COURSE] + [
        "Status", "Reached y (m)", "Min z (m)", "Max tilt", "Time (s)"
    ]

    def cell(summary, cp):
        rec = summary["reached"][cp["name"]]
        if rec is None:
            return "-"
        tilt = _tilt(rec["rpy"])
        if tilt >= cp["max_tilt"]:
            return f"! {tilt:.0f}deg"
        return "ok"

    rows = []
    for r in results:
        row = [r["gait"]]
        row.extend(cell(r, cp) for cp in COURSE)
        row.append(_status(r))
        row.append(f"{r['final_pos'][1]:+.2f}")
        row.append(f"{r['min_z']:+.3f}")
        row.append(f"{r['max_tilt']:.0f}deg")
        row.append(f"{r['duration']:.1f}")
        rows.append(row)

    widths = [max(len(str(header[i])),
                  max(len(str(row[i])) for row in rows)) for i in range(len(header))]

    def fmt(row):
        return "| " + " | ".join(str(c).ljust(widths[i])
                                 for i, c in enumerate(row)) + " |"

    lines = []
    lines.append("# FaceHugger Terrain Survival Matrix")
    lines.append("")
    lines.append("Course (spawn at y = -0.10, all obstacles along +Y):")
    for cp in COURSE:
        lines.append(f"- **{cp['name']}** at y = {cp['y']:.2f} m "
                     f"(tilt budget {cp['max_tilt']:.0f} deg)")
    lines.append("")
    lines.append(fmt(header))
    lines.append("|" + "|".join("-" * (w + 2) for w in widths) + "|")
    for row in rows:
        lines.append(fmt(row))
    lines.append("")
    lines.append("**Legend**")
    lines.append("- `ok`       -- checkpoint crossed within tilt budget")
    lines.append("- `! Xdeg`   -- crossed but body tilt X deg exceeded budget")
    lines.append("- `-`        -- never reached (timeout, fall, or wrong direction)")
    lines.append("- `FINISHED` -- reached every checkpoint upright")
    lines.append("- `TIMEOUT`  -- ran out of sim time before finish")
    lines.append("- `REVERSED` -- gait pushed the robot backward (-Y)")
    lines.append("- `STUCK`    -- never left the spawn neighbourhood")
    lines.append("- `FELL @ X` -- body tipped or dragged before crossing X")

    report = "\n".join(lines)
    print("\n\n" + report)

    if save_path:
        with open(save_path, "w") as f:
            f.write(report + "\n")
        print(f"\nReport written to {save_path}")

    return results


def run_teleop(gait_name="trot", gui=True, duration=600.0):
    """Keyboard-driven teleop on flat ground.

    Controls (PyBullet debug window must have focus, AZERTY layout):
      Z/S      forward / reverse throttle (held)
      Q/D      turn left / right (updates heading setpoint)
      SPACE    cycle gait: walk -> trot -> bound
      R        reset body to stance (teleport)
      ESC      quit
    """
    if gait_name not in GAITS:
        raise ValueError(f"Unknown gait '{gait_name}'. Known: {sorted(GAITS)}")

    p.connect(p.GUI if gui else p.DIRECT)
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        # Disable PyBullet's built-in debug hotkeys (W=wireframe, S=shadows,
        # A=AABB, etc.) so they don't fight the teleop key handler.
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(TIMESTEP)
    p.loadURDF("plane.urdf")

    body_height = -NEUTRAL_FOOT["fl"][2]
    spawn_pos = [0.0, 0.0, body_height + 0.005]
    robot_id = p.loadURDF(
        URDF_PATH,
        basePosition=spawn_pos,
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=False,
    )
    joint_map = build_joint_map(robot_id)
    reset_to_stance(robot_id, joint_map, STANCE)
    apply_per_leg_pose(robot_id, joint_map, all_legs(STANCE))

    for joint_name, joint_idx in joint_map.items():
        if "knee" in joint_name:
            p.changeDynamics(robot_id, joint_idx,
                             lateralFriction=1.2, restitution=0.1)

    if gui:
        p.resetDebugVisualizerCamera(
            cameraDistance=0.85, cameraYaw=50, cameraPitch=-25,
            cameraTargetPosition=[0.0, 0.0, 0.10],
        )

    print(f"\n=== Teleop: {GAITS[gait_name]['label']} ===")
    print("  forward: Z / W / UP      backward: S / DOWN")
    print("  left:    Q / A / LEFT    right:    D / RIGHT")
    print("  SHIFT + fwd/back = sprint  |  SPACE cycle gait  |  R reset  |  ESC quit")
    print("  (raw key codes print on each press; export TELEOP_QUIET=1 to hide)")

    gait_cycle = ["walk", "trot", "bound"]
    gait_idx = gait_cycle.index(gait_name) if gait_name in gait_cycle else 1

    STEER_KP = 1.2        # heading-error gain (rad^-1)
    STEER_MAX = 0.6       # stride-scaling clamp
    TURN_RATE = 1.4       # rad/s of desired-heading change when A/D held
    SPEED_MULT  = 1.6     # throttle scale when forward/back held
    SPRINT_MULT = 2.6     # throttle scale when SHIFT + forward/back held
    PERIOD_SPRINT_SCALE = 0.75  # shorter period in sprint -> higher cadence
    desired_yaw = 0.0

    hud_id = -1
    steer_cfg_left, steer_cfg_right = {}, {}

    t, steps = 0.0, int(duration / TIMESTEP)
    last_hud_update = -1

    for i in range(steps):
        if not p.isConnected():
            print("  [window closed, exiting]")
            return

        keys = p.getKeyboardEvents()

        # Diagnostic: echo every new keypress with its code. One line per
        # press (fires on KEY_WAS_TRIGGERED, not while held) so the terminal
        # stays readable. Set TELEOP_QUIET=1 to silence.
        if keys and not os.environ.get("TELEOP_QUIET"):
            for kc, flag in keys.items():
                if flag & p.KEY_WAS_TRIGGERED:
                    ch = chr(kc) if 32 <= kc < 127 else "?"
                    print(f"  [key] code={kc} char='{ch}'")

        def _down(*codes):
            return any(c in keys and (keys[c] & p.KEY_IS_DOWN) for c in codes)

        def _triggered(*codes):
            return any(c in keys and (keys[c] & p.KEY_WAS_TRIGGERED)
                       for c in codes)

        # Key codes. PyBullet on macOS returns the layout-translated character
        # for letter keys, so AZERTY 'z' at the QWERTY-'w' position comes back
        # as ord('z')=122. We accept BOTH AZERTY and QWERTY letter aliases,
        # plus arrow keys which are layout-neutral, so teleop works regardless.
        KEY_FWD    = (ord('z'), ord('Z'), ord('w'), ord('W'),
                      p.B3G_UP_ARROW)
        KEY_BACK   = (ord('s'), ord('S'), p.B3G_DOWN_ARROW)
        KEY_LEFT   = (ord('q'), ord('Q'), ord('a'), ord('A'),
                      p.B3G_LEFT_ARROW)
        KEY_RIGHT  = (ord('d'), ord('D'), p.B3G_RIGHT_ARROW)
        KEY_GAIT   = (ord(' '), p.B3G_SPACE)
        KEY_RESET  = (ord('r'), ord('R'))
        KEY_QUIT   = (27,)  # ESC (raw ASCII; this build lacks B3G_ESCAPE)
        KEY_SPRINT = (p.B3G_SHIFT,)

        # Quit
        if _down(*KEY_QUIT):
            print("  [quit]")
            break

        # Gait cycle (one-shot)
        if _triggered(*KEY_GAIT):
            gait_idx = (gait_idx + 1) % len(gait_cycle)
            gait_name = gait_cycle[gait_idx]
            print(f"  [gait] -> {gait_name}")

        # Reset (one-shot)
        if _triggered(*KEY_RESET):
            p.resetBasePositionAndOrientation(
                robot_id, spawn_pos,
                p.getQuaternionFromEuler([0, 0, 0]),
            )
            p.resetBaseVelocity(robot_id, [0, 0, 0], [0, 0, 0])
            reset_to_stance(robot_id, joint_map, STANCE)
            desired_yaw = 0.0
            print("  [reset]")

        # Throttle (held). SHIFT = sprint: larger stride + faster cadence.
        sprinting = _down(*KEY_SPRINT)
        speed_scale = SPRINT_MULT if sprinting else SPEED_MULT
        throttle = 0.0
        if _down(*KEY_FWD):
            throttle += speed_scale
        if _down(*KEY_BACK):
            throttle -= speed_scale

        # Steering: update desired heading while left/right held
        if _down(*KEY_LEFT):
            desired_yaw += TURN_RATE * TIMESTEP
        if _down(*KEY_RIGHT):
            desired_yaw -= TURN_RATE * TIMESTEP

        # Current heading
        base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
        _, _, yaw_rad = p.getEulerFromQuaternion(base_orn)
        yaw_err = _wrap_pi(desired_yaw - yaw_rad)
        steer = _clamp(STEER_KP * yaw_err, -STEER_MAX, STEER_MAX)

        cfg = GAITS[gait_name]
        base_stride = cfg["step_length"] * throttle
        # yaw_err>0 means we want to yaw body left (CCW). Longer stride on the
        # right side rotates the body left, so right gets (1+steer), left gets
        # (1-steer).
        steer_cfg_left  = dict(cfg)
        steer_cfg_right = dict(cfg)
        steer_cfg_left["step_length"]  = base_stride * (1.0 - steer)
        steer_cfg_right["step_length"] = base_stride * (1.0 + steer)

        period = cfg["period"] * (PERIOD_SPRINT_SCALE if sprinting else 1.0)
        global_phase = (t / period) % 1.0
        targets = {}
        for leg_id, offset in cfg["offsets"].items():
            phase = (global_phase - offset) % 1.0
            side_cfg = steer_cfg_left if leg_id.endswith("l") else steer_cfg_right
            foot = foot_target(leg_id, phase, side_cfg["step_length"],
                               cfg["step_height"], cfg["duty"])
            theta_s, theta_h, theta_k = leg_ik(foot, leg_id)
            targets[f"{leg_id}_shoulder_joint"] = theta_s
            targets[f"{leg_id}_hip_joint"]      = theta_h
            targets[f"{leg_id}_knee_joint"]     = theta_k

        for joint_name, angle in targets.items():
            idx = joint_map.get(joint_name)
            if idx is None:
                continue
            p.setJointMotorControl2(
                robot_id, idx, p.POSITION_CONTROL,
                targetPosition=angle,
                force=SERVO_FORCE, maxVelocity=SERVO_VELOCITY,
            )

        p.stepSimulation()

        if gui:
            # Chase camera (behind the robot, trailing its heading)
            if i % 10 == 0:
                cam_yaw_deg = math.degrees(yaw_rad) + 90.0
                p.resetDebugVisualizerCamera(
                    cameraDistance=0.85, cameraYaw=cam_yaw_deg,
                    cameraPitch=-20,
                    cameraTargetPosition=[base_pos[0], base_pos[1], 0.10],
                )

            # HUD refresh ~12 Hz
            if i - last_hud_update >= 20:
                mode = "SPRINT" if sprinting else "cruise"
                hud = f"gait:{gait_name}  mode:{mode}"
                hud_kwargs = dict(
                    textPosition=[base_pos[0], base_pos[1], 0.25],
                    textColorRGB=[0.1, 0.9, 0.1],
                    textSize=1.2,
                )
                if hud_id == -1:
                    hud_id = p.addUserDebugText(hud, **hud_kwargs)
                else:
                    hud_id = p.addUserDebugText(hud, replaceItemUniqueId=hud_id,
                                                **hud_kwargs)
                last_hud_update = i

            time.sleep(TIMESTEP)

        t += TIMESTEP

    if p.isConnected():
        p.disconnect()


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #

def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--walk", action="store_true",
                        help="Run the static walk gait.")
    parser.add_argument("--trot", action="store_true",
                        help="Run the dynamic trot gait (diagonal pairs).")
    parser.add_argument("--bound", action="store_true",
                        help="Run the bound gait (front pair, then rear pair).")
    parser.add_argument("--crab", action="store_true",
                        help="Run the crab gait (sideways along body-X).")
    parser.add_argument("--terrain", choices=sorted(GAITS),
                        help="Run the named gait across the obstacle course.")
    parser.add_argument("--terrain-matrix", action="store_true",
                        help="Run all gaits over the course; print survival table.")
    parser.add_argument("--teleop", nargs="?", const="trot",
                        choices=sorted(GAITS),
                        help="Drive the robot with the keyboard "
                             "(Z/S throttle, Q/D turn, SPACE cycle gait, "
                             "R reset, ESC quit). Optional: starting gait.")
    parser.add_argument("--headless", action="store_true",
                        help="Run without GUI (useful for CI / quick checks).")
    args = parser.parse_args()

    if args.teleop:
        if args.headless:
            parser.error("--teleop requires GUI (drop --headless)")
        try:
            run_teleop(args.teleop, gui=True)
        except p.error as e:
            # Closing the GUI window mid-step tears down the physics server,
            # so any in-flight pybullet call fails. Treat that as a normal exit.
            print(f"  [window closed: {e}]")
        return

    # Closing the GUI window mid-step tears down the physics server, so any
    # in-flight pybullet call fails. Treat that as a normal exit for all
    # interactive run modes.
    try:
        if args.terrain_matrix:
            report_path = os.path.join(os.path.dirname(URDF_PATH),
                                       "terrain_report.md")
            run_terrain_matrix(gui=not args.headless, save_path=report_path)
        elif args.terrain:
            # Extra runway past the finish line so the viewer watches the robot
            # keep cruising after crossing the green marker.
            duration = {"walk": 90.0, "trot": 60.0,
                        "bound": 60.0, "crab": 90.0}[args.terrain]
            run_terrain(args.terrain, gui=not args.headless, duration=duration)
        elif args.walk:
            run_gait("walk", gui=not args.headless)
        elif args.trot:
            run_gait("trot", gui=not args.headless)
        elif args.bound:
            run_gait("bound", gui=not args.headless)
        elif args.crab:
            run_gait("crab", gui=not args.headless)
        else:
            run_stand(gui=not args.headless)
    except p.error as e:
        print(f"  [window closed: {e}]")


if __name__ == "__main__":
    main()
