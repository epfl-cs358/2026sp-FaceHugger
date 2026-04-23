"""Leg kinematics: link lengths, IK, foot trajectory, gait joint targets."""

import math

from constants import HIP_ANGLE, KNEE_ANGLE
from helpers import _clamp, _wrap_pi

# Leg link geometry (metres). Pulled from the URDF joint origins.
L1 = 0.080   # shoulder arm:   shoulder origin -> hip joint
L2 = 0.075   # upper leg:      hip joint       -> knee joint
L3 = 0.077   # lower leg:      knee joint      -> foot tip (end of mesh)

# Shoulder mount per leg, in body frame, plus the yaw offset baked into the
# URDF origin rpy (left legs = 0, right legs = pi). arm_sign follows from the
# yaw offset (left arm extends body -X, right arm extends body +X).
LEG_INFO = {
    "fl": {"mount": (-0.040,  0.050, 0.025), "yaw_offset": 0.0},
    "fr": {"mount": ( 0.040,  0.050, 0.025), "yaw_offset": math.pi},
    "rl": {"mount": (-0.040, -0.050, 0.025), "yaw_offset": 0.0},
    "rr": {"mount": ( 0.040, -0.050, 0.025), "yaw_offset": math.pi},
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

# Joint limits (mirrored from URDF) for clamping IK outputs
_JOINT_LIMITS = {
    "shoulder": (-2.356, 2.356),
    "hip":      (-1.5708, 1.5708),
    "knee":     (-2.356, 0.5),
}


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
