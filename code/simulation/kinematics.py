"""LegGeom + RobotConfig dataclasses, FK/IK math, build_config from URDF/yaml.

Chain (per leg, after shoulder yaw offset rpy_z around Z):
  T = Rz(yaw_offset) * Rz(theta_s) *
      Ttrans(L1) *
      Rx(theta_h) * Ttrans(L2) *
      Rx(theta_k) * Ttrans(foot_L3)

At all zeros the leg chain extends along +Y (because L2_vec and foot_L3 are
both mostly +Y). Hip and knee pitch around +X -- positive angle lifts the
chain toward +Z, negative drops it toward -Z.

ASSUMPTIONS (flag if URDF ever violates them):
  - shoulder joint axis is +Z (URDF has `0 0 1`)
  - hip and knee axes are +X (URDF has `1 0 -0` which is X within 1e-9)
  - shoulder yaw offset is a pure Z rotation (rpy = 0 0 q)
  - no roll/pitch component in L1/L2/foot transforms (no <origin rpy=...>
    on hip/knee joints in v2 URDF -- verified 2026-04-23)
"""

import math
import os
from dataclasses import dataclass, field
from typing import Callable, Dict, Tuple

import yaml

from constants import CONFIG_YAML, FUSION_JSON, MESH_DIR, STANCE_DEG, URDF_PATH
from helpers import (
    _clamp, _wrap_pi,
    _foot_tip_from_fusion, _load_urdf_joints,
    _parse_leg_points_from_urdf, _stl_foot_tip_m,
)


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
# FK / IK
# --------------------------------------------------------------------------- #

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
