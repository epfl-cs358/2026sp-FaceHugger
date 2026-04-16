"""
FaceHugger Quadruped Robot - PyBullet Simulation

Run modes:
  default     : robot loads and holds a stable standing stance
  --walk      : static walk gait (one leg swings at a time, CoM always inside
                support triangle), with overlaid foot-trajectory debug lines.
  --trot      : dynamic trot gait (diagonal pairs FL+RR / FR+RL swing
                together, duty 0.5), same foot-trajectory debug overlay.
  --wall-flip : robot puts its front legs against a wall, vaults over it,
                lands on the other side upside-down, and reconfigures its
                legs (hip/knee pitches negated) so the inverted pose looks
                identical to the upright one (the robot is bilaterally
                symmetric).

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

# Inverted stance: hip and knee pitches negated. After the body rotates 180°
# around its X-axis (forward somersault), negating both pitches makes each
# leg point back toward the ground in the world frame. The knee upper limit
# is +0.5 rad in the URDF, so the mirrored knee target is clamped there.
INVERTED_STANCE = {
    "shoulder": SHOULDER_ANGLE,
    "hip": -HIP_ANGLE,
    "knee": min(0.5, -KNEE_ANGLE),
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
# Wall flip
# --------------------------------------------------------------------------- #

def add_wall(y_position, height, length=0.50, thickness=0.02):
    """Create a static red wall at +y in front of the robot."""
    half = [length / 2, thickness / 2, height / 2]
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half,
                              rgbaColor=[0.75, 0.25, 0.25, 1.0])
    wall_id = p.createMultiBody(
        baseMass=0,                          # static
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=[0.0, y_position, height / 2],
    )
    p.changeDynamics(wall_id, -1, lateralFriction=1.5, restitution=0.0)
    return wall_id


# --- Pose presets used by the flip phases --------------------------------- #

def pose_stand():
    return all_legs(STANCE)


def pose_reach():
    """Front legs lift off the ground and reach forward to touch the wall."""
    front = {"shoulder": 0.0,
             "hip":      math.radians(-85),   # rotate leg toward +Y
             "knee":     math.radians(-25)}
    return {"fl": front, "fr": front, "rl": STANCE, "rr": STANCE}


def pose_plant():
    """Front feet pressed flat against the wall; rear legs squat to load."""
    front = {"shoulder": 0.0,
             "hip":      math.radians(-90),
             "knee":     math.radians(-5)}    # almost straight, push into wall
    rear  = {"shoulder": 0.0,
             "hip":      math.radians(60),
             "knee":     math.radians(-80)}   # squat, ready to extend
    return {"fl": front, "fr": front, "rl": rear, "rr": rear}


def pose_push():
    """Rear legs extend, driving body up and forward against the wall."""
    front = {"shoulder": 0.0,
             "hip":      math.radians(-90),
             "knee":     math.radians(-5)}
    rear  = {"shoulder": 0.0,
             "hip":      math.radians(20),
             "knee":     math.radians(-15)}   # extend
    return {"fl": front, "fr": front, "rl": rear, "rr": rear}


def pose_tuck():
    """All four legs tuck close to the body during the airborne flip."""
    tuck = {"shoulder": 0.0,
            "hip":      math.radians(-70),
            "knee":     0.5}                  # max +knee
    return all_legs(tuck)


def pose_inverted():
    return all_legs(INVERTED_STANCE)


# --- Flip driver ---------------------------------------------------------- #

def run_wall_flip(gui=True):
    """
    Sequence:
      WARMUP  - settle in standing pose
      REACH   - front legs lift forward to touch the wall
      PLANT   - front feet pinned to wall, rear legs squat
      PUSH    - rear legs extend, driving CoM up over the wall
      FLIP    - external torque around X tips the body forward over wall edge
      TUCK    - legs pull in for clean rotation through 180 deg
      LAND    - hip/knee angles negated to support the now-inverted body
      HOLD    - keep the inverted stance and let physics settle
    """
    p.connect(p.GUI if gui else p.DIRECT)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(TIMESTEP)

    p.loadURDF("plane.urdf")

    wall_y      = 0.14
    wall_height = 0.09
    wall_id     = add_wall(y_position=wall_y, height=wall_height)

    robot_id = p.loadURDF(
        URDF_PATH,
        basePosition=[0, -0.02, 0.18],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=False,
    )
    joint_map = build_joint_map(robot_id)

    # High friction on feet/lower legs so they grip the wall
    for joint_name, joint_idx in joint_map.items():
        if "foot" in joint_name:
            p.changeDynamics(robot_id, joint_idx,
                             lateralFriction=2.0, restitution=0.0,
                             spinningFriction=0.5)
        elif "lower_leg" in joint_name:
            p.changeDynamics(robot_id, joint_idx, lateralFriction=2.0)
    p.changeDynamics(robot_id, -1, lateralFriction=1.0)

    reset_to_stance(robot_id, joint_map, STANCE)
    apply_per_leg_pose(robot_id, joint_map, pose_stand())

    # Camera: side angle so the wall and the flip are both visible
    p.resetDebugVisualizerCamera(
        cameraDistance=0.75,
        cameraYaw=-35,
        cameraPitch=-12,
        cameraTargetPosition=[0.0, wall_y, 0.10],
    )

    # (name, duration_seconds, pose_fn, flip_assist)
    #   flip_assist:  None       -> no body-level forces
    #                 "kick"     -> P-controller torques body around X axis to
    #                               -180 deg + lift/shove to clear the wall
    #                 "snap_inv" -> kinematic snap to inverted on far side (ran
    #                               for one tick at start of phase to lock the
    #                               landing pose; physics resumes after)
    phases = [
        ("WARMUP", 0.8, pose_stand,    None),
        ("REACH",  1.2, pose_reach,    None),
        ("PLANT",  1.0, pose_plant,    None),
        ("PUSH",   0.6, pose_push,     None),
        ("FLIP",   1.8, pose_tuck,     "kick"),
        ("LAND",   0.2, pose_inverted, "snap_inv"),
        ("HOLD",   3.5, pose_inverted, None),
    ]

    print("\n=== FaceHugger Wall Flip ===")
    print("Sequence: " + " -> ".join(name for name, *_ in phases))

    # Flip-controller gains. Roll goes from 0 -> -pi as the body tips forward
    # (right-hand rule around +X with front of body in +Y).
    FLIP_TARGET_ROLL = -math.pi
    K_TORQUE  = 1.2     # N*m per rad of roll error
    D_TORQUE  = 0.18    # N*m per rad/s damping
    MAX_TORQ  = 3.5     # N*m clamp
    FWD_FORCE = 1.5     # N -- shoves CoM toward and over the wall
    LIFT_FORCE = 5.0    # N -- short upward lift while body is still upright

    # Where we want to deposit the inverted robot: just past the wall
    landing_pos = [0.0, wall_y + 0.18, 0.06]
    landing_orn = p.getQuaternionFromEuler([math.pi, 0.0, 0.0])

    inversion_announced = False
    elapsed = 0.0

    for name, duration, pose_fn, assist in phases:
        steps = int(duration / TIMESTEP)
        apply_per_leg_pose(robot_id, joint_map, pose_fn())
        print(f"[{elapsed:5.2f}s] -> {name:6s} ({duration:.1f}s)")

        if assist == "snap_inv":
            # Lock the landing pose: place the inverted body just past the
            # wall, zero out linear/angular velocities, and reset every joint
            # to the mirrored stance. Physics takes over from this clean
            # state for the HOLD phase.
            p.resetBasePositionAndOrientation(robot_id, landing_pos, landing_orn)
            p.resetBaseVelocity(robot_id, [0, 0, 0], [0, 0, 0])
            reset_to_stance(robot_id, joint_map, INVERTED_STANCE)

        for _ in range(steps):
            if assist == "kick":
                _, orn = p.getBasePositionAndOrientation(robot_id)
                roll, _, _ = p.getEulerFromQuaternion(orn)
                ang_vel = p.getBaseVelocity(robot_id)[1]

                err = FLIP_TARGET_ROLL - roll
                tx  = K_TORQUE * err - D_TORQUE * ang_vel[0]
                tx  = max(-MAX_TORQ, min(MAX_TORQ, tx))

                p.applyExternalTorque(robot_id, -1,
                                      torqueObj=[tx, 0, 0],
                                      flags=p.WORLD_FRAME)

                # While body is still upright-ish: lift up and shove forward
                # so the CoM clears the wall edge instead of plowing through.
                if roll > -math.radians(90):
                    p.applyExternalForce(robot_id, -1,
                                         forceObj=[0, FWD_FORCE, LIFT_FORCE],
                                         posObj=[0, 0, 0],
                                         flags=p.WORLD_FRAME)

            try:
                p.stepSimulation()
            except p.error:
                print("Simulation disconnected.")
                return
            if not p.isConnected():
                return
            if gui:
                time.sleep(TIMESTEP)
            elapsed += TIMESTEP

            if not inversion_announced:
                _, orn = p.getBasePositionAndOrientation(robot_id)
                roll, _, _ = p.getEulerFromQuaternion(orn)
                if abs(roll) > math.radians(150):
                    print(f"   >>> body inverted (roll = "
                          f"{math.degrees(roll):+.1f} deg)")
                    inversion_announced = True

    pos, orn = p.getBasePositionAndOrientation(robot_id)
    roll, pitch, yaw = p.getEulerFromQuaternion(orn)
    print(f"\nFinal: pos=({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})  "
          f"roll={math.degrees(roll):+.1f}  "
          f"pitch={math.degrees(pitch):+.1f}  "
          f"yaw={math.degrees(yaw):+.1f}")

    if gui:
        print("\nFlip complete - close the window or press Ctrl+C to exit.")
        try:
            while p.isConnected():
                p.stepSimulation()
                time.sleep(TIMESTEP)
        except (KeyboardInterrupt, p.error):
            pass

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
    "walk": {
        "period":      2.4,
        "step_length": 0.04,
        "step_height": 0.02,
        "duty":        0.25,
        "offsets":     {"fl": 0.00, "rr": 0.25, "fr": 0.50, "rl": 0.75},
        "label":       "Static walk (FL -> RR -> FR -> RL)",
    },
    "trot": {
        "period":      0.8,
        "step_length": 0.05,
        "step_height": 0.025,
        "duty":        0.5,
        "offsets":     {"fl": 0.0, "rr": 0.0, "fr": 0.5, "rl": 0.5},
        "label":       "Trot (diagonal pairs: FL+RR | FR+RL)",
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


def foot_target(leg_id, phase, step_length, step_height, duty):
    """Body-frame foot target for a given leg at a given per-leg phase [0, 1)."""
    nx, ny, nz = NEUTRAL_FOOT[leg_id]
    if phase < duty:
        s = phase / duty
        dy = -step_length * 0.5 + s * step_length
        dz = step_height * math.sin(math.pi * s)
    else:
        s = (phase - duty) / (1.0 - duty)
        dy = step_length * 0.5 - s * step_length
        dz = 0.0
    return (nx, ny + dy, nz + dz)


def gait_joint_targets(t, cfg):
    """Return {joint_name: angle} for all 12 joints at elapsed time t."""
    period = cfg["period"]
    step_length = cfg["step_length"]
    step_height = cfg["step_height"]
    duty = cfg["duty"]
    global_phase = (t / period) % 1.0
    targets = {}
    for leg_id, offset in cfg["offsets"].items():
        phase = (global_phase - offset) % 1.0
        foot = foot_target(leg_id, phase, step_length, step_height, duty)
        theta_s, theta_h, theta_k = leg_ik(foot, leg_id)
        targets[f"{leg_id}_shoulder_joint"] = theta_s
        targets[f"{leg_id}_hip_joint"]      = theta_h
        targets[f"{leg_id}_knee_joint"]     = theta_k
    return targets


def _body_to_world(pos_body, base_pos, base_orn):
    """Transform a point from the robot's body frame into world frame."""
    world_pos, _ = p.multiplyTransforms(base_pos, base_orn, pos_body, [0, 0, 0, 1])
    return world_pos


def _precompute_foot_cycle(step_length, step_height, duty, samples=_TRAJ_SAMPLES):
    """Pre-sample one full cycle of body-frame foot positions per leg."""
    cycles = {}
    for leg_id in LEG_INFO:
        pts = []
        for k in range(samples):
            phase = k / samples
            pts.append(foot_target(leg_id, phase, step_length, step_height, duty))
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
                                    cfg["duty"])
    line_ids = {}
    marker_ids = {}
    draw_overlay = gui and SHOW_FOOT_TRAJECTORIES
    draw_every = 4  # refresh overlay every N sim steps to keep GUI snappy

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
# Entry point
# --------------------------------------------------------------------------- #

def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--wall-flip", action="store_true",
                        help="Run the wall-flip sequence instead of standing.")
    parser.add_argument("--walk", action="store_true",
                        help="Run the static walk gait.")
    parser.add_argument("--trot", action="store_true",
                        help="Run the dynamic trot gait (diagonal pairs).")
    parser.add_argument("--headless", action="store_true",
                        help="Run without GUI (useful for CI / quick checks).")
    args = parser.parse_args()

    if args.wall_flip:
        run_wall_flip(gui=not args.headless)
    elif args.walk:
        run_gait("walk", gui=not args.headless)
    elif args.trot:
        run_gait("trot", gui=not args.headless)
    else:
        run_stand(gui=not args.headless)


if __name__ == "__main__":
    main()
