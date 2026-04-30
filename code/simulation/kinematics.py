"""LegGeom + RobotConfig dataclasses, FK/IK math, build_config from URDF/yaml.

Chain (per leg, after factoring shoulder yaw + back-of-pair Rz(rpy_z)):
  T = Rz(yaw_offset + theta_s) *
      Ttrans(L1) *
      Ry(s_h) * Ttrans(L2) *
      Ry(s_k) * Ttrans(foot_L3)

where
  s_h = leg.hip_axis_sign  * theta_h
  s_k = leg.knee_axis_sign * theta_k

User-facing angles (theta_h, theta_k) follow Phase H's uniform convention:
the same numerical value drops every leg into the same physical pose. The
URDF-internal angles (s_h, s_k) include the per-leg axis sign so the FK
matches what PyBullet applies when given the user-facing target.

ASSUMPTIONS (flag if URDF ever violates them):
  - shoulder joint axis is +Z (URDF has `0 0 1`)
  - hip and knee axes are +Y for L pair, -Y for R pair (Phase H sign flip)
  - shoulder yaw offset is a pure Z rotation (rpy = 0 0 q)
  - no roll/pitch on hip/knee joint origins (`<origin rpy="0 0 0"/>`)
  - foot tip in link3 frame for L pair = mesh-local FootTip from URDF metadata
  - foot tip in link3 frame for R pair = (-x, y, -z) of the L-pair value
    (link2/link3 visuals carry mesh_rpy=(0, π, 0) for R pair, which is an
    Ry(π) rotation of the mesh; the kinematic foot tip in link3 frame
    follows the rotation).
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
    """Per-leg kinematic data. Values in metres / radians, body frame."""
    leg_id: str
    mount: Tuple[float, float, float]      # shoulder joint origin in body frame
    yaw_offset: float                      # shoulder joint rpy around Z (0 or ±π)
    L1_vec: Tuple[float, float, float]     # shoulder -> hip offset (in link1 frame)
    L2_vec: Tuple[float, float, float]     # hip -> knee offset      (in link2 frame)
    foot_L3: Tuple[float, float, float]    # knee -> foot tip        (in link3 frame)
    hip_axis_sign: int                     # +1 if URDF hip axis is +Y, -1 if -Y
    knee_axis_sign: int                    # +1 if URDF knee axis is +Y, -1 if -Y
    joint_limits: Dict[str, Tuple[float, float]]  # per-leg, user-facing range


@dataclass
class RobotConfig:
    urdf_path: str
    legs: Dict[str, LegGeom]               # insertion order = leg iteration order
    servo_force: float
    servo_velocity: float
    stance_rad: Dict[str, Dict[str, float]]  # leg_id -> {shoulder, hip, knee} in radians
    leg_ik: Callable[["RobotConfig", Tuple[float, float, float], str], Tuple[float, float, float]]
    leg_fk: Callable[["RobotConfig", str, float, float, float], Tuple[float, float, float]]
    neutral_foot: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)
    body_height: float = 0.12              # spawn height; recomputed from neutral foot


# --------------------------------------------------------------------------- #
# Rotation helpers
# --------------------------------------------------------------------------- #

def _ry(v, c, s):
    """Apply Ry(theta) to vector v, given c = cos(theta), s = sin(theta).
    Right-hand-rule rotation around +Y; rotates (X, Z) plane clockwise as
    seen from +Y for positive theta."""
    x, y, z = v
    return (x * c + z * s, y, -x * s + z * c)


def _rz(v, c, s):
    x, y, z = v
    return (x * c - y * s, x * s + y * c, z)


# --------------------------------------------------------------------------- #
# Forward kinematics
# --------------------------------------------------------------------------- #

def fk_v2(cfg, leg_id, theta_s, theta_h, theta_k):
    """User-facing (theta_s, theta_h, theta_k) -> foot position in body frame.

    Phase H made theta_h, theta_k uniform across legs (same value -> same
    physical pose). Internally we multiply by the URDF axis sign to get the
    actual rotation matrix that PyBullet applies."""
    leg = cfg.legs[leg_id]
    s_h = leg.hip_axis_sign * theta_h
    s_k = leg.knee_axis_sign * theta_k
    ch, sh = math.cos(s_h), math.sin(s_h)
    ck, sk = math.cos(s_k), math.sin(s_k)

    # foot in link2 frame: Ry(s_k) * foot_L3 + L2
    f_l2 = _ry(leg.foot_L3, ck, sk)
    v_l2 = (leg.L2_vec[0] + f_l2[0],
            leg.L2_vec[1] + f_l2[1],
            leg.L2_vec[2] + f_l2[2])
    # foot in link1 frame: Ry(s_h) * v_l2 + L1
    v_l1 = _ry(v_l2, ch, sh)
    w = (leg.L1_vec[0] + v_l1[0],
         leg.L1_vec[1] + v_l1[1],
         leg.L1_vec[2] + v_l1[2])
    # foot in body frame: mount + Rz(yaw_offset + theta_s) * w
    q = leg.yaw_offset + theta_s
    cq, sq = math.cos(q), math.sin(q)
    foot = _rz(w, cq, sq)
    return (leg.mount[0] + foot[0],
            leg.mount[1] + foot[1],
            leg.mount[2] + foot[2])


# --------------------------------------------------------------------------- #
# Inverse kinematics
# --------------------------------------------------------------------------- #

def ik_v2(cfg, foot_body, leg_id):
    """Foot position (body frame) -> user-facing (theta_s, theta_h, theta_k).

    Strategy:
      1. Y component of the link1-frame foot is invariant under Ry(s_h) and
         Ry(s_k), so it's a per-leg constant. Use it + |delta_xy|^2 to
         recover X (the chain-extension axis).
      2. Shoulder yaw drops out via atan2.
      3. 2-link planar solve in (X, Z) plane relative to L1 gives s_h, s_k.
      4. Convert URDF-internal (s_h, s_k) to user-facing (theta_h, theta_k)
         via the per-leg axis sign.
      5. Per-leg limit clamping (limits differ between L and R pair).
    """
    leg = cfg.legs[leg_id]

    dx = foot_body[0] - leg.mount[0]
    dy = foot_body[1] - leg.mount[1]
    dz = foot_body[2] - leg.mount[2]

    # w_y is invariant under Ry — equals L1.y + L2.y + foot_L3.y.
    w_y = leg.L1_vec[1] + leg.L2_vec[1] + leg.foot_L3[1]

    # Rz preserves XY norm: |delta_xy|^2 == w_x^2 + w_y^2.
    rxy2 = dx * dx + dy * dy
    w_x2 = rxy2 - w_y * w_y
    if w_x2 < 0.0:
        w_x2 = 0.0     # target laterally closer than w_y alone can reach

    # Sign of w_x: chain in link1 frame extends along -X for L pair,
    # +X for R pair. Pick the sign that matches the per-leg geometry so
    # FK/IK round-trip is stable.
    chain_x_sign = 1.0 if (
        leg.L1_vec[0] + leg.L2_vec[0] + leg.foot_L3[0]
    ) >= 0 else -1.0
    w_x = chain_x_sign * math.sqrt(w_x2)

    # Rz preserves Z, so w_z == dz.
    w_z = dz

    # Shoulder yaw: q = atan2(dy, dx) - atan2(w_y, w_x); theta_s = q - yaw_offset.
    q = math.atan2(dy, dx) - math.atan2(w_y, w_x)
    theta_s = _wrap_pi(q - leg.yaw_offset)

    # 2-link planar solve in (X, Z) plane, relative to L1.
    u = w_x - leg.L1_vec[0]
    v = w_z - leg.L1_vec[2]

    # L2 and foot_L3 planar lengths in (X, Z); Y is folded into w_y above.
    l2 = math.hypot(leg.L2_vec[0], leg.L2_vec[2])
    l3 = math.hypot(leg.foot_L3[0], leg.foot_L3[2])
    d = math.hypot(u, v)

    d = min(d, l2 + l3 - 1e-6)              # keep triangle reachable
    d = max(d, abs(l2 - l3) + 1e-6)

    # Law of cosines on the bent chain. |V|^2 expands to
    #   l2^2 + l3^2 + 2 l2 l3 cos(s_k - alpha_rel)
    # where alpha_rel is the angle of foot_L3 in the X-Z plane MEASURED
    # FROM L2's direction (not from +X).
    cos_kprime = (d * d - l2 * l2 - l3 * l3) / (2.0 * l2 * l3)
    kprime = math.acos(_clamp(cos_kprime, -1.0, 1.0))   # [0, pi]
    alpha_rel = _wrap_pi(
        math.atan2(leg.foot_L3[2], leg.foot_L3[0])
        - math.atan2(leg.L2_vec[2], leg.L2_vec[0])
    )
    # Knee branch: fold the lower-leg so the foot supports the body. Ry
    # rotates (X, Z) by -theta (opposite of Rx in the old +Y-chain code),
    # so the supporting branch is s_k = +chain_x_sign * kprime — for L
    # pair (chain at -X) that's s_k < 0, for R pair (chain at +X with
    # axis flipped) it's s_k > 0. chain_x_sign picks the right branch so
    # FK/IK round-trips.
    s_k = chain_x_sign * kprime - alpha_rel

    # Recover s_h: the "bent chain in (X, Z)" sits at angle phi_V; (u, v) sits
    # at angle phi_uv. Ry(s_h) rotates (X, Z) by -s_h, so phi_uv = phi_V - s_h
    # → s_h = phi_V - phi_uv.
    ck, sk = math.cos(s_k), math.sin(s_k)
    V_x = leg.L2_vec[0] + leg.foot_L3[0] * ck + leg.foot_L3[2] * sk
    V_z = leg.L2_vec[2] - leg.foot_L3[0] * sk + leg.foot_L3[2] * ck
    s_h = _wrap_pi(math.atan2(V_z, V_x) - math.atan2(v, u))

    # Convert URDF-internal angles back to user-facing (Phase H).
    theta_h = leg.hip_axis_sign * s_h
    theta_k = leg.knee_axis_sign * s_k

    lo_s, hi_s = leg.joint_limits["shoulder"]
    lo_h, hi_h = leg.joint_limits["hip"]
    lo_k, hi_k = leg.joint_limits["knee"]
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


def _axis_sign_y(axis_xyz):
    """+1 if axis points along +Y, -1 if -Y. Tolerates the tiny float dust
    the URDF carries (e.g. (1.18e-16, 1.0, 2.43e-17) or its negation)."""
    return 1 if axis_xyz[1] >= 0.0 else -1


def build_config():
    if not os.path.exists(URDF_PATH):
        raise FileNotFoundError(f"Missing URDF: {URDF_PATH}")
    if not os.path.exists(CONFIG_YAML):
        raise FileNotFoundError(f"Missing config yaml: {CONFIG_YAML}")
    with open(CONFIG_YAML) as f:
        yaml_cfg = yaml.safe_load(f)

    joints = _load_urdf_joints(URDF_PATH)

    # FootTip (in link3-mesh frame) from the LEG ASSEMBLY METADATA comment.
    # Prefer an explicit FootTipPoint construction point if/when the user
    # adds one to Fusion; else fall back to the metadata block; else fall
    # back to the STL heuristic.
    leg_pts = _parse_leg_points_from_urdf(URDF_PATH)
    foot_tip_link3_mm = leg_pts["FootTip"]                  # in link3 mesh frame
    tip_assembly = _foot_tip_from_fusion(FUSION_JSON)
    if tip_assembly is None:
        # Sanity: keep the STL heuristic available for diagnostics; the
        # foot_L3 the FK actually uses comes from the URDF comment block.
        _ = _stl_foot_tip_m(os.path.join(MESH_DIR, "leg_lower.stl"))
        print(f"[sim] foot tip from URDF metadata (link3 frame, mm): "
              f"{tuple(round(v, 3) for v in foot_tip_link3_mm)}")
    else:
        print(f"[sim] foot tip from FootTipPoint: "
              f"{tuple(round(v, 4) for v in tip_assembly)} m")

    # foot_L3 in link3 frame, per side.
    # L pair: link3 visual rpy = identity, so link3 frame ≡ mesh frame.
    # R pair: link3 visual rpy = (0, π, 0), so the mesh is rotated 180°
    # around Y inside link3. The foot tip in link3 frame is then the
    # mesh-local tip rotated by Ry(π) → (-x, y, -z).
    foot_L3_L = tuple(v / 1000.0 for v in foot_tip_link3_mm)
    foot_L3_R = (-foot_L3_L[0], foot_L3_L[1], -foot_L3_L[2])

    servo = yaml_cfg.get("servo", {})
    servo_force = float(servo.get("effort_nm", 2.94))
    servo_velocity = float(servo.get("velocity_rad_s", 5.0))

    # Per-leg stance: hip/knee shared via STANCE_DEG; shoulder from yaml's
    # shoulder_neutral_deg.
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
        servo_force=servo_force,
        servo_velocity=servo_velocity,
        stance_rad=stance_per_leg,
        leg_ik=ik_v2,
        leg_fk=fk_v2,
    )

    for leg in yaml_cfg["legs"]:
        leg_id = leg["id"]
        side = leg.get("side", "L")
        sh_joint = joints[f"{leg_id}_shoulder_joint"]
        hip_joint = joints[f"{leg_id}_hip_joint"]
        knee_joint = joints[f"{leg_id}_knee_joint"]

        mount = sh_joint["xyz"]
        yaw_offset = sh_joint["rpy"][2]
        L1_vec = hip_joint["xyz"]
        L2_vec = knee_joint["xyz"]
        foot_L3 = foot_L3_R if side == "R" else foot_L3_L

        # Convert URDF user-facing limits (already in the post-Phase-H frame)
        # into our LegGeom. Each leg's URDF limits differ; per-leg storage.
        joint_limits = {
            "shoulder": sh_joint["limits"][:2],
            "hip":      hip_joint["limits"][:2],
            "knee":     knee_joint["limits"][:2],
        }

        cfg.legs[leg_id] = LegGeom(
            leg_id=leg_id,
            mount=mount,
            yaw_offset=yaw_offset,
            L1_vec=L1_vec,
            L2_vec=L2_vec,
            foot_L3=foot_L3,
            hip_axis_sign=_axis_sign_y(hip_joint["axis"]),
            knee_axis_sign=_axis_sign_y(knee_joint["axis"]),
            joint_limits=joint_limits,
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
    # Body spawn height = max foot depth below body origin.
    cfg.body_height = max(1e-3, -min(f[2] for f in cfg.neutral_foot.values()))
    return cfg
