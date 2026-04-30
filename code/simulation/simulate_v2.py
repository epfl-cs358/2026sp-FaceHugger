"""
FaceHugger Quadruped Simulation

Modes: default stand, --walk, --trot.

Geometry sourcing (no duplication of constants in Python):
  - Mount XYZ, joint origins, joint axes, joint limits, link inertials
      -> parsed from facehugger.urdf via xml.etree (not pybullet.getJointInfo,
         because pybullet silently shifts link frames to COM which skews the
         parent-frame origins when inertial <origin> is non-zero).
  - Leg IDs, per-leg yaw_z, servo effort/velocity, joint limits (fallback)
      -> facehugger_config.yaml
  - Foot-tip offset in Link3 frame
      -> ASSUMPTION: lower-leg STL vertices live in the leg-assembly root
         frame (where the shoulder joint sits at origin). The foot tip is
         taken as the centroid of vertices at max +Y of leg_lower.stl. This
         falls back from a missing "FootTipPoint" construction point in
         fusion_export.json -- if one is added later, read it preferentially.
"""

import argparse
import math
import os
import struct
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Tuple

import pybullet as p
import pybullet_data
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
GENERATED_DIR = os.path.join(HERE, "generated")
URDF_PATH = os.path.join(GENERATED_DIR, "facehugger.urdf")
CONFIG_YAML = os.path.join(HERE, "facehugger_config.yaml")
FUSION_JSON = os.path.join(GENERATED_DIR, "fusion_export.json")
MESH_DIR = os.path.join(GENERATED_DIR, "exported_meshes")

TIMESTEP = 1.0 / 240.0

# Hip/knee stance is shared across legs (R_L1 = I because yaml rpy_z_deg = 90
# for all legs). Shoulder stance is per-leg and lives in the yaml as each leg's
# `shoulder_neutral_deg` — the middle of its quadrant's ±90° range. Standing
# pose = each leg at its neutral shoulder, so the legs splay into their four
# corners naturally.
STANCE_DEG = {
    "hip":  -40.0,
    "knee": -60.0,
}


def _stance_for(leg_id, legs_cfg):
    """Resolve the per-leg standing stance {shoulder/hip/knee} dict.
    Shoulder comes from the yaml's `shoulder_neutral_deg`; hip/knee are
    shared across all legs via STANCE_DEG."""
    entry = next(l for l in legs_cfg if l["id"] == leg_id)
    shoulder_deg = entry.get("shoulder_neutral_deg", 0.0)
    return {
        "shoulder": shoulder_deg,
        "hip":      STANCE_DEG["hip"],
        "knee":     STANCE_DEG["knee"],
    }


# --------------------------------------------------------------------------- #
# RobotConfig: everything the sim needs, derived once at startup.
# --------------------------------------------------------------------------- #

@dataclass
class LegGeom:
    """Per-leg kinematic data. Values are in metres / radians, body frame."""
    leg_id: str
    mount: Tuple[float, float, float]      # shoulder joint origin in body frame
    yaw_offset: float                      # shoulder joint rpy around Z
    L1_vec: Tuple[float, float, float]     # shoulder -> hip offset (in link1 frame)
    L2_vec: Tuple[float, float, float]     # hip -> knee offset      (in link2 frame)
    foot_L3: Tuple[float, float, float]    # knee -> foot tip        (in link3 frame)


@dataclass
class RobotConfig:
    urdf_path: str
    legs: Dict[str, LegGeom]               # insertion order == leg iteration order
    joint_limits: Dict[str, Tuple[float, float]]  # shoulder/hip/knee -> (lo, hi) rad
    servo_force: float
    servo_velocity: float
    stance_rad: Dict[str, Dict[str, float]]  # leg_id -> {shoulder, hip, knee} in radians
    leg_ik: Callable[["RobotConfig", Tuple[float, float, float], str], Tuple[float, float, float]]
    leg_fk: Callable[["RobotConfig", str, float, float, float], Tuple[float, float, float]]
    neutral_foot: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)
    body_height: float = 0.12              # spawn height; recomputed from neutral foot


# --------------------------------------------------------------------------- #
# URDF / STL / YAML parsing helpers (v2 introspection)
# --------------------------------------------------------------------------- #

def _parse_xyz(s):
    return tuple(float(x) for x in s.strip().split())


def _parse_leg_points_from_urdf(urdf_path):
    """Read the LEG ASSEMBLY METADATA comment block the generator writes near
    the top of the URDF. Returns {point_name: [x, y, z]} in mm, leg-assembly-
    local frame. Raises if the comment block is missing so we fail loud
    instead of silently falling back to a stale hardcoded value.
    """
    import re
    text = open(urdf_path).read()
    pts = {}
    # Each metadata line has the shape:
    #   <Name> [optional descriptor that may contain digits, like "link3"]: <x> <y> <z>
    # Match non-greedily up to the first colon on the line, then the three
    # whitespace-separated signed-decimal numbers. The earlier [^0-9\-]*
    # bridge broke on descriptors containing digits (e.g. "link3 frame").
    pattern = re.compile(
        r"(BodyToLink1Point|Link1ToLink2Point|Link2ToLink3Point|FootTip)"
        r"[^\n]*?:\s*"
        r"(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)"
    )
    for m in pattern.finditer(text):
        name = m.group(1)
        pts[name] = [float(m.group(2)), float(m.group(3)), float(m.group(4))]
    required = ("BodyToLink1Point", "Link1ToLink2Point", "Link2ToLink3Point", "FootTip")
    missing = [n for n in required if n not in pts]
    if missing:
        raise ValueError(
            f"URDF at {urdf_path} is missing LEG ASSEMBLY METADATA comment "
            f"for points: {missing}. Regenerate with generate_urdf.py."
        )
    return pts


def _load_urdf_joints(urdf_path):
    """Return {joint_name: {parent, child, xyz, rpy, axis, limits}} from the raw XML."""
    root = ET.parse(urdf_path).getroot()
    out = {}
    for j in root.findall("joint"):
        name = j.get("name")
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        org = j.find("origin")
        xyz = _parse_xyz(org.get("xyz", "0 0 0")) if org is not None else (0, 0, 0)
        rpy = _parse_xyz(org.get("rpy", "0 0 0")) if org is not None else (0, 0, 0)
        ax = j.find("axis")
        axis = _parse_xyz(ax.get("xyz")) if ax is not None else (1, 0, 0)
        lim = j.find("limit")
        if lim is not None:
            limits = (float(lim.get("lower", -math.pi)),
                      float(lim.get("upper",  math.pi)),
                      float(lim.get("effort", 0.0)),
                      float(lim.get("velocity", 0.0)))
        else:
            limits = (-math.pi, math.pi, 0.0, 0.0)
        out[name] = dict(parent=parent, child=child, xyz=xyz, rpy=rpy,
                         axis=axis, limits=limits)
    return out


def _stl_foot_tip_m(stl_path, y_tol_mm=2.0):
    """Centroid (in metres) of leg_lower STL vertices within y_tol_mm of max Y.

    ASSUMPTION: STL vertices are in the leg-assembly root frame (shoulder at
    origin). Verified by stacking alignment: leg_shoulder.stl y in [-13, 111],
    leg_upper.stl y in [51, 172] (matches hip origin at 51), leg_lower.stl y in
    [144, 232] (matches knee origin at 146). If the pipeline ever re-centres
    meshes into their own link frames this function will silently return the
    wrong tip -- sanity-check against NEUTRAL_FOOT z after any pipeline change.
    """
    with open(stl_path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        verts = []
        y_max = float("-inf")
        for _ in range(n):
            f.read(12)
            for _ in range(3):
                v = struct.unpack("<fff", f.read(12))
                verts.append(v)
                if v[1] > y_max:
                    y_max = v[1]
            f.read(2)
    tip_pts = [v for v in verts if v[1] >= y_max - y_tol_mm]
    cx = sum(v[0] for v in tip_pts) / len(tip_pts) / 1000.0
    cy = sum(v[1] for v in tip_pts) / len(tip_pts) / 1000.0
    cz = sum(v[2] for v in tip_pts) / len(tip_pts) / 1000.0
    return (cx, cy, cz)


def _foot_tip_from_fusion(fusion_json_path):
    """Return the FootTipPoint from fusion_export.json if present, else None.

    Prefer this over the STL heuristic whenever the CAD designer adds an
    explicit FootTipPoint construction point under FaceHuggerLegAssembly:1.
    """
    import json
    try:
        with open(fusion_json_path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None

    def _walk(node):
        if isinstance(node, dict):
            if node.get("name") == "FootTipPoint" and "position_mm" in node:
                return node["position_mm"]
            for v in node.values():
                r = _walk(v)
                if r is not None:
                    return r
        elif isinstance(node, list):
            for v in node:
                r = _walk(v)
                if r is not None:
                    return r
        return None

    pos = _walk(data)
    if pos is None:
        return None
    return tuple(x / 1000.0 for x in pos)


# --------------------------------------------------------------------------- #
# v2 forward / inverse kinematics
# --------------------------------------------------------------------------- #
# Chain (per leg, after shoulder yaw offset rpy_z around Z):
#   T = Rz(yaw_offset) * Rz(theta_s) *
#       Ttrans(L1) *
#       Rx(theta_h) * Ttrans(L2) *
#       Rx(theta_k) * Ttrans(foot_L3)
# At all zeros the leg chain extends along +Y (because L2_vec and foot_L3 are
# both mostly +Y). Hip and knee pitch around +X -- positive angle lifts the
# chain toward +Z, negative drops it toward -Z.
#
# ASSUMPTIONS (flag if URDF ever violates them):
#   - shoulder joint axis is +Z (URDF has `0 0 1`)
#   - hip and knee axes are +X (URDF has `1 0 -0` which is X within 1e-9)
#   - shoulder yaw offset is a pure Z rotation (rpy = 0 0 q)
#   - no roll/pitch component in L1/L2/foot transforms (no <origin rpy=...>
#     on hip/knee joints in v2 URDF -- verified 2026-04-23)

def _rx(v, c, s):
    x, y, z = v
    return (x, y * c - z * s, y * s + z * c)


def _rz(v, c, s):
    x, y, z = v
    return (x * c - y * s, x * s + y * c, z)


def fk_v2(cfg, leg_id, theta_s, theta_h, theta_k):
    leg = cfg.legs[leg_id]
    ch, sh = math.cos(theta_h), math.sin(theta_h)
    ck, sk = math.cos(theta_k), math.sin(theta_k)

    # foot in Link2 frame: Rx(k) * foot_L3 + L2
    f_l2 = _rx(leg.foot_L3, ck, sk)
    v_l2 = (leg.L2_vec[0] + f_l2[0],
            leg.L2_vec[1] + f_l2[1],
            leg.L2_vec[2] + f_l2[2])
    # foot in Link1 frame: Rx(h) * v_l2 + L1
    v_l1 = _rx(v_l2, ch, sh)
    w = (leg.L1_vec[0] + v_l1[0],
         leg.L1_vec[1] + v_l1[1],
         leg.L1_vec[2] + v_l1[2])
    # foot in body frame: mount + Rz(yaw + s) * w
    q = leg.yaw_offset + theta_s
    cq, sq = math.cos(q), math.sin(q)
    foot = _rz(w, cq, sq)
    return (leg.mount[0] + foot[0],
            leg.mount[1] + foot[1],
            leg.mount[2] + foot[2])


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def ik_v2(cfg, foot_body, leg_id):
    leg = cfg.legs[leg_id]
    # delta = Rz(yaw + s) * w, where w = L1 + Rx(h) * (L2 + Rx(k) * foot_L3).
    dx = foot_body[0] - leg.mount[0]
    dy = foot_body[1] - leg.mount[1]
    dz = foot_body[2] - leg.mount[2]

    # w_x is independent of theta_h, theta_k because Rx preserves X.
    w_x = leg.L1_vec[0] + leg.L2_vec[0] + leg.foot_L3[0]

    # Rz preserves Z, so w_z == dz.
    w_z = dz

    # |delta_xy|^2 == w_x^2 + w_y^2  -> w_y
    rxy2 = dx * dx + dy * dy
    w_y2 = rxy2 - w_x * w_x
    if w_y2 < 0.0:
        w_y2 = 0.0     # target laterally closer than w_x can reach; snap in
    # Branch selection: the leg chain extends along +Y in Link1 frame for
    # fl/bl (template orientation) but along -Y for fr/br (generator applied
    # a 180-deg R_L1 rotation, flipping the chain's natural direction). Pick
    # the sign of w_y based on where the neutral chain points so the
    # round-trip is self-consistent. The neutral y-extent is
    #   L1.y + L2.y + foot_L3.y
    # which is + for +Y-extending legs and - for flipped legs.
    chain_y_sign = 1.0 if (
        leg.L1_vec[1] + leg.L2_vec[1] + leg.foot_L3[1]
    ) >= 0 else -1.0
    w_y = chain_y_sign * math.sqrt(w_y2)

    # Shoulder yaw: q = atan2(dy, dx) - atan2(w_y, w_x)
    q = math.atan2(dy, dx) - math.atan2(w_y, w_x)
    theta_s = _wrap_pi(q - leg.yaw_offset)

    # 2-link planar solve in the (Y, Z) plane *relative to L1*.
    u = w_y - leg.L1_vec[1]
    v = w_z - leg.L1_vec[2]

    # L2 and foot_L3 planar lengths (ignore X, which is baked into w_x above).
    l2 = math.hypot(leg.L2_vec[1], leg.L2_vec[2])
    l3 = math.hypot(leg.foot_L3[1], leg.foot_L3[2])
    d = math.hypot(u, v)

    d = min(d, l2 + l3 - 1e-6)  # keep triangle reachable
    d = max(d, abs(l2 - l3) + 1e-6)

    # Law of cosines on the 2-link chain. |V|^2 expands to
    #   l2^2 + l3^2 + 2 l2 l3 cos(alpha_rel + theta_k)
    # where alpha_rel is the angle of foot_L3 in the Y-Z plane MEASURED FROM
    # L2's direction (not from the +Y axis). This removes the 180-deg offset
    # for flipped chains where L2 and foot_L3 both point along -Y.
    cos_kprime = (d * d - l2 * l2 - l3 * l3) / (2.0 * l2 * l3)
    kprime = math.acos(_clamp(cos_kprime, -1.0, 1.0))  # [0, pi]
    alpha_rel = _wrap_pi(
        math.atan2(leg.foot_L3[2], leg.foot_L3[1])
        - math.atan2(leg.L2_vec[2], leg.L2_vec[1])
    )
    # Knee branch: fold the foot AGAINST the chain direction so the leg
    # supports the body. For normal (+Y) chains this means theta_k < 0; for
    # flipped (-Y) chains the equivalent fold is theta_k > 0. chain_y_sign
    # (computed above) selects the correct branch so FK/IK round-trip.
    theta_k = chain_y_sign * (-kprime) - alpha_rel

    # theta_h = angle of (u, v) in Y-Z - angle offset of the bent chain in Y-Z.
    # Bent chain Y-Z vector (before hip rotation):
    #     V_yz = (l2 + l3*cos(k), l3*sin(k))       if foot_L3 points along +Y.
    # Using the actual L2 / foot_L3 components (including Z) is more general:
    ck, sk = math.cos(theta_k), math.sin(theta_k)
    V_y = leg.L2_vec[1] + leg.foot_L3[1] * ck - leg.foot_L3[2] * sk
    V_z = leg.L2_vec[2] + leg.foot_L3[1] * sk + leg.foot_L3[2] * ck
    theta_h = math.atan2(v, u) - math.atan2(V_z, V_y)
    theta_h = _wrap_pi(theta_h)

    lo_s, hi_s = cfg.joint_limits["shoulder"]
    lo_h, hi_h = cfg.joint_limits["hip"]
    lo_k, hi_k = cfg.joint_limits["knee"]
    return (_clamp(theta_s, lo_s, hi_s),
            _clamp(theta_h, lo_h, hi_h),
            _clamp(theta_k, lo_k, hi_k))


# --------------------------------------------------------------------------- #
# Config builder
# --------------------------------------------------------------------------- #

def build_config():
    if not os.path.exists(URDF_PATH):
        raise FileNotFoundError(f"Missing URDF: {URDF_PATH}")
    if not os.path.exists(CONFIG_YAML):
        raise FileNotFoundError(f"Missing config yaml: {CONFIG_YAML}")
    with open(CONFIG_YAML) as f:
        yaml_cfg = yaml.safe_load(f)

    joints = _load_urdf_joints(URDF_PATH)

    # Prefer an explicit FootTipPoint, else fall back to STL heuristic.
    tip_assembly = _foot_tip_from_fusion(FUSION_JSON)
    if tip_assembly is None:
        tip_assembly = _stl_foot_tip_m(os.path.join(MESH_DIR, "leg_lower.stl"))
        print(f"[sim] foot tip from STL (no FootTipPoint in fusion_export.json): "
              f"{tuple(round(v, 4) for v in tip_assembly)} m")
    else:
        print(f"[sim] foot tip from FootTipPoint: "
              f"{tuple(round(v, 4) for v in tip_assembly)} m")

    # Per-leg: pull mount + yaw offset from URDF shoulder joint.
    servo = yaml_cfg.get("servo", {})
    servo_force = float(servo.get("effort_nm", 2.94))
    servo_velocity = float(servo.get("velocity_rad_s", 5.0))

    # Joint limits: trust URDF (authoritative) but fall back to config if needed.
    lim_shoulder = joints["fr_shoulder_joint"]["limits"][:2]
    lim_hip = joints["fr_hip_joint"]["limits"][:2]
    lim_knee = joints["fr_knee_joint"]["limits"][:2]

    # Foot tip in Link3 frame, per leg. The generator now rotates each leg's
    # internal geometry from LAL into its Link1 frame by R_L1 (different per
    # leg depending on yaml's rpy_z_deg vs. the CAD's leg-assembly rotation).
    # So L1_vec, L2_vec, foot_L3 are per-leg now; the LegGeom dataclass
    # already supports that, we just have to read them per leg.
    #
    # STL tip and LAL knee point both come from the URDF comment (single
    # source of truth) and are in LAL frame. The per-leg foot_L3 in Link3
    # frame is R_L1 @ (tip_LAL - knee_LAL). We recover R_L1 per leg by
    # comparing the URDF's hip xyz (in Link1 frame) to the LAL delta from
    # the comment block -- if they match, r_l1_angle = 0; if they're xy-
    # flipped, r_l1_angle = pi. Cleaner than parsing each link's visual rpy.
    leg_pts = _parse_leg_points_from_urdf(URDF_PATH)
    body_to_link1_mm = leg_pts["BodyToLink1Point"]
    link1_to_link2_mm = leg_pts["Link1ToLink2Point"]
    link2_to_link3_mm = leg_pts["Link2ToLink3Point"]
    foot_tip_mm = leg_pts["FootTip"]
    # LAL-frame deltas (shared across all legs).
    L1_lal_m = tuple((link1_to_link2_mm[i] - body_to_link1_mm[i]) / 1000.0 for i in range(3))
    foot_from_knee_lal_m = tuple(
        (foot_tip_mm[i] - link2_to_link3_mm[i]) / 1000.0 for i in range(3)
    )

    def _rot_z(v, theta):
        c, s = math.cos(theta), math.sin(theta)
        return (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])

    def _r_l1_angle(leg_id):
        """Recover the R_L1 rotation (pure Z angle) applied by the generator
        for this leg, by comparing URDF's hip xyz (Link1-frame) against the
        shared LAL delta."""
        l1_in_link1 = joints[f"{leg_id}_hip_joint"]["xyz"]
        # Find theta such that R_z(theta) @ L1_lal_m ≈ l1_in_link1.
        # Two unknowns via x, y components; solve with atan2.
        src_angle = math.atan2(L1_lal_m[1], L1_lal_m[0])
        dst_angle = math.atan2(l1_in_link1[1], l1_in_link1[0])
        return _wrap_pi(dst_angle - src_angle)

    # Per-leg stance: hip/knee shared; shoulder is each leg's
    # `shoulder_neutral_deg` from the yaml (the center of its corner's
    # ±90° quadrant).
    stance_per_leg = {
        leg["id"]: {
            k: math.radians(v)
            for k, v in _stance_for(leg["id"], yaml_cfg["legs"]).items()
        }
        for leg in yaml_cfg["legs"]
    }

    cfg = RobotConfig(
        urdf_path=URDF_PATH,
        legs={},
        joint_limits={"shoulder": lim_shoulder, "hip": lim_hip, "knee": lim_knee},
        servo_force=servo_force,
        servo_velocity=servo_velocity,
        stance_rad=stance_per_leg,
        leg_ik=ik_v2,
        leg_fk=fk_v2,
    )

    for leg in yaml_cfg["legs"]:
        leg_id = leg["id"]
        sh_joint = joints[f"{leg_id}_shoulder_joint"]
        mount = sh_joint["xyz"]
        yaw_offset = sh_joint["rpy"][2]
        # Per-leg vectors from the (now rotated) URDF.
        L1_vec = joints[f"{leg_id}_hip_joint"]["xyz"]
        L2_vec = joints[f"{leg_id}_knee_joint"]["xyz"]
        theta_l1 = _r_l1_angle(leg_id)
        foot_L3 = _rot_z(foot_from_knee_lal_m, theta_l1)
        cfg.legs[leg_id] = LegGeom(
            leg_id=leg_id,
            mount=mount,
            yaw_offset=yaw_offset,
            L1_vec=L1_vec,
            L2_vec=L2_vec,
            foot_L3=foot_L3,
        )

    cfg.neutral_foot = {
        leg_id: fk_v2(
            cfg, leg_id,
            cfg.stance_rad[leg_id]["shoulder"],
            cfg.stance_rad[leg_id]["hip"],
            cfg.stance_rad[leg_id]["knee"],
        )
        for leg_id in cfg.legs
    }
    # Body spawn height = highest foot Z below body origin.
    cfg.body_height = max(1e-3, -min(f[2] for f in cfg.neutral_foot.values()))
    return cfg


# --------------------------------------------------------------------------- #
# Shared helpers
# --------------------------------------------------------------------------- #

def _joint_type_from_name(name):
    if "shoulder" in name: return "shoulder"
    if "hip" in name:      return "hip"
    if "knee" in name:     return "knee"
    return None


def build_joint_map(robot_id):
    out = {}
    n = p.getNumJoints(robot_id)
    for i in range(n):
        info = p.getJointInfo(robot_id, i)
        out[info[1].decode()] = i
    return out


def reset_to_stance(robot_id, joint_map, stance):
    # `stance` is now per-leg: {leg_id: {shoulder/hip/knee: rad}}.
    for name, idx in joint_map.items():
        info = p.getJointInfo(robot_id, idx)
        if info[2] == p.JOINT_FIXED:
            continue
        jtype = _joint_type_from_name(name)
        leg_id = name[:2]
        if jtype is None or leg_id not in stance:
            continue
        p.resetJointState(robot_id, idx, stance[leg_id][jtype])


def apply_leg_pose(robot_id, joint_map, per_leg_stance, force, velocity):
    for name, idx in joint_map.items():
        info = p.getJointInfo(robot_id, idx)
        if info[2] == p.JOINT_FIXED:
            continue
        leg = name[:2]
        jtype = _joint_type_from_name(name)
        if jtype is None or leg not in per_leg_stance:
            continue
        p.setJointMotorControl2(
            robot_id, idx, p.POSITION_CONTROL,
            targetPosition=per_leg_stance[leg][jtype],
            force=force, maxVelocity=velocity,
        )


def apply_joint_targets(robot_id, joint_map, targets, force, velocity):
    """targets = {joint_name: angle}"""
    for name, angle in targets.items():
        idx = joint_map.get(name)
        if idx is None:
            continue
        p.setJointMotorControl2(
            robot_id, idx, p.POSITION_CONTROL,
            targetPosition=angle, force=force, maxVelocity=velocity,
        )


# --------------------------------------------------------------------------- #
# Gait scheduler
# --------------------------------------------------------------------------- #

GAITS = {
    "walk": {
        "period":      2.4,
        "step_length": 0.04,
        "step_height": 0.02,
        "duty":        0.25,
        "offsets":     {"fl": 0.00, "br": 0.25, "fr": 0.50, "bl": 0.75},
        "label":       "Static walk",
    },
    "trot": {
        "period":      0.8,
        "step_length": 0.05,
        "step_height": 0.025,
        "duty":        0.5,
        "offsets":     {"fl": 0.0, "br": 0.0, "fr": 0.5, "bl": 0.5},
        "label":       "Trot (diagonal pairs)",
    },
}

_TRAJ_COLORS = {
    "fl": (1.0, 0.30, 0.30),
    "fr": (0.30, 1.0, 0.30),
    "bl": (0.30, 0.50, 1.0),
    "br": (1.0, 1.0, 0.30),
}


def foot_target(neutral_foot, leg_id, phase, step_length, step_height, duty,
                swing_axis="y"):
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
        foot = foot_target(cfg.neutral_foot, leg_id, phase,
                           gait["step_length"], gait["step_height"], gait["duty"])
        s, h, k = cfg.leg_ik(cfg, foot, leg_id)
        targets[f"{leg_id}_shoulder_joint"] = s
        targets[f"{leg_id}_hip_joint"]      = h
        targets[f"{leg_id}_knee_joint"]     = k
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
        pts = [foot_target(cfg.neutral_foot, leg_id, k / _TRAJ_SAMPLES,
                           gait["step_length"], gait["step_height"], gait["duty"])
               for k in range(_TRAJ_SAMPLES)]
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
                prev, cur, lineColorRGB=color, lineWidth=1.5,
                replaceItemUniqueId=lid if lid is not None else -1,
            )
            prev = cur
        tw = _body_to_world(current_targets[leg_id], base_pos, base_orn)
        s = 0.012
        axes = [((-s, 0, 0), (s, 0, 0)),
                ((0, -s, 0), (0, s, 0)),
                ((0, 0, -s), (0, 0, s))]
        for i, (a, b) in enumerate(axes):
            p0 = tuple(tw[j] + a[j] for j in range(3))
            p1 = tuple(tw[j] + b[j] for j in range(3))
            mkey = (leg_id, i)
            mid = _MARK_IDS.get(mkey)
            _MARK_IDS[mkey] = p.addUserDebugLine(
                p0, p1, lineColorRGB=color, lineWidth=3.0,
                replaceItemUniqueId=mid if mid is not None else -1,
            )


# --------------------------------------------------------------------------- #
# Simulation entry points
# --------------------------------------------------------------------------- #

def _body_height_for_gait(cfg, gait, period_s, samples_per_period=100):
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
                cfg.neutral_foot, leg_id, phase,
                gait["step_length"], gait["step_height"], gait["duty"],
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
        apply_leg_pose(robot_id, joint_map, cfg.stance_rad,
                       cfg.servo_force, cfg.servo_velocity)
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
    apply_leg_pose(robot_id, joint_map, cfg.stance_rad,
                   cfg.servo_force, cfg.servo_velocity)

    # Friction on the distal link (knee joint's child = lower leg).
    for name, idx in joint_map.items():
        if "knee" in name:
            p.changeDynamics(robot_id, idx, lateralFriction=1.0, restitution=0.1)

    if gui:
        p.resetDebugVisualizerCamera(
            cameraDistance=0.55, cameraYaw=45, cameraPitch=-25,
            cameraTargetPosition=[0, 0, 0.1],
        )
    return robot_id, joint_map


def _print_banner(cfg):
    print(f"\n=== FaceHugger sim ===")
    print(f"  URDF: {os.path.basename(cfg.urdf_path)}")
    print(f"  legs: {list(cfg.legs.keys())}")
    print(f"  servo: force={cfg.servo_force} N*m  vel={cfg.servo_velocity} rad/s")
    print(f"  body_height: {cfg.body_height*1000:.1f} mm")
    print(f"  stance (deg, per leg):")
    for leg_id, s in cfg.stance_rad.items():
        print(f"    {leg_id}: shoulder={math.degrees(s['shoulder']):+.1f}  "
              f"hip={math.degrees(s['hip']):+.1f}  knee={math.degrees(s['knee']):+.1f}")
    for leg_id, foot in cfg.neutral_foot.items():
        print(f"    {leg_id}: foot = "
              f"({foot[0]*1000:+6.1f}, {foot[1]*1000:+6.1f}, {foot[2]*1000:+6.1f}) mm")

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
        print(f"    IK[{leg_id}]: ds={math.degrees(ds):+.2f} dh={math.degrees(dh):+.2f} "
              f"dk={math.degrees(dk):+.2f} [{tag}]")


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


def run_gait(cfg, gait_name, gui=True, settle_s=0.5):
    if gait_name not in GAITS:
        raise ValueError(f"Unknown gait: {gait_name}")
    gait = GAITS[gait_name]

    # Gait-aware spawn height: worst foot Z across a full period, not just
    # neutral_foot. Prevents the body from sinking into the floor when a
    # gait's stance-phase Z differs from the neutral Z used at build time.
    gait_depth_m = _body_height_for_gait(cfg, gait, gait["period"])
    if gait_depth_m > cfg.body_height:
        print(f"[body_height] lifting spawn from {cfg.body_height*1000:.1f} mm "
              f"to {gait_depth_m*1000:.1f} mm for {gait_name} trajectory")
        cfg.body_height = gait_depth_m

    robot_id, joint_map = _connect_and_setup(cfg, gui)
    _print_banner(cfg)
    if settle_s > 0:
        print(f"\n[settle] holding stance for {settle_s:.2f}s before gait")
        _settle(robot_id, joint_map, cfg, settle_s)
    print(f"\n{gait['label']}: period={gait['period']:.2f}s  "
          f"len={gait['step_length']*1000:.0f}mm  h={gait['step_height']*1000:.0f}mm  "
          f"duty={gait['duty']:.2f}")

    draw_overlay = gui
    cycles = _precompute_cycle(cfg, gait) if draw_overlay else None
    draw_every = 4

    t = 0.0
    step = 0
    try:
        while p.isConnected():
            targets = gait_joint_targets(cfg, gait, t)
            apply_joint_targets(robot_id, joint_map, targets,
                                cfg.servo_force, cfg.servo_velocity)
            if draw_overlay and step % draw_every == 0:
                offsets = gait["offsets"]
                global_phase = (t / gait["period"]) % 1.0
                cur_targets = {
                    leg_id: foot_target(
                        cfg.neutral_foot, leg_id, (global_phase - off) % 1.0,
                        gait["step_length"], gait["step_height"], gait["duty"],
                    ) for leg_id, off in offsets.items()
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


# --------------------------------------------------------------------------- #

def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--walk", action="store_true")
    parser.add_argument("--trot", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--settle", type=float, default=0.5,
                        help="Seconds to hold stance before the main loop "
                             "begins (lets gravity resolve initial overlap). "
                             "Default 0.5.")
    args = parser.parse_args()

    cfg = build_config()
    gui = not args.headless
    if args.walk:
        run_gait(cfg, "walk", gui=gui, settle_s=args.settle)
    elif args.trot:
        run_gait(cfg, "trot", gui=gui, settle_s=args.settle)
    else:
        run_stand(cfg, gui=gui, settle_s=args.settle)


if __name__ == "__main__":
    main()
