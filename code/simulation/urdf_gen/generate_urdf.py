"""
generate_urdf.py  —  FaceHugger URDF generator

Reads:
  generated/fusion_export.json  (from ExportBodiesToURDF Fusion script)
  facehugger_config.yaml        (robot hierarchy definition)

Writes:
  generated/facehugger.urdf     (or --out path)

Usage:
  python -m urdf_gen.generate_urdf
  python -m urdf_gen.generate_urdf --export some/other/fusion_export.json
  python -m urdf_gen.generate_urdf --config facehugger_config.yaml --out generated/facehugger.urdf
"""

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

try:
    import yaml
except ImportError:
    raise SystemExit("PyYAML missing — run: uv add pyyaml")

from urdf_gen.lib.construction_points import resolve_leg_construction_points
from urdf_gen.lib.foot_tip import _foot_tip_from_export, _foot_tip_from_stl
from urdf_gen.lib.fusion_export import (
    _find_occ_rot,
    find_occurrence,
    find_point_in_tree,
    load_export,
)
from urdf_gen.lib.joint_defs import build_joint_definitions, flip_joint_for_r_side
from urdf_gen.lib.leg_rest import _back_of_pair_rpy_z_deg, _shoulder_rest_for
from urdf_gen.lib.mesh_servo import _mesh_shift, _primary_link_occurrence
from urdf_gen.lib.physics_fallback import get_physics
from urdf_gen.lib.urdf_math import (
    _mat_mul_3x3,
    _mat_transpose_3x3,
    _rot_to_urdf_rpy,
    sub,
)
from urdf_gen.lib.urdf_writer import CollisionPrimitive, URDF


# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------

# This module lives in urdf_gen/; generated/ and the config yaml sit one level
# up in code/simulation/ (shared with the pybullet_sim runtime).
SIM_ROOT = Path(__file__).resolve().parent.parent
GENERATED_DIR = SIM_ROOT / "generated"
DEFAULT_JSON = GENERATED_DIR / "fusion_export.json"
DEFAULT_CFG = SIM_ROOT / "facehugger_config.yaml"
DEFAULT_OUT = GENERATED_DIR / "facehugger.urdf"

# Module-level audit accumulator for the per-link mass table.
_audit_rows = []


# ---------------------------------------------------------------------------
# Point-mass inertia helpers (parallel axis theorem)
# ---------------------------------------------------------------------------


def _point_mass_inertia(mass_kg, pos_mm, com_mm):
    """Inertia tensor of a point mass at `pos_mm` about `com_mm`.

    I_ij = m * (|d|² δ_ij - d_i d_j)  where d = pos - com.
    pos_mm, com_mm in mm; returns {ixx, iyy, izz, ixy, ixz, iyz} in kg·m².
    """
    dx = (pos_mm[0] - com_mm[0]) / 1000.0
    dy = (pos_mm[1] - com_mm[1]) / 1000.0
    dz = (pos_mm[2] - com_mm[2]) / 1000.0
    return {
        "ixx": mass_kg * (dy * dy + dz * dz),
        "iyy": mass_kg * (dx * dx + dz * dz),
        "izz": mass_kg * (dx * dx + dy * dy),
        "ixy": -mass_kg * dx * dy,
        "ixz": -mass_kg * dx * dz,
        "iyz": -mass_kg * dy * dz,
    }


def _shift_inertia_to_com(inertia, mass_kg, old_com_mm, new_com_mm):
    """Shift inertia tensor from old COM to new COM (parallel axis theorem).

    I_new = I_old + m * (|d|² δ - d⊗d)  where d = old_com - new_com.
    All moments in kg·m²; positions in mm.
    """
    d_mm = [old_com_mm[i] - new_com_mm[i] for i in range(3)]
    d_sq = sum(x * x for x in d_mm) / 1e6  # (mm→m)²
    dx = d_mm[0] / 1000.0
    dy = d_mm[1] / 1000.0
    dz = d_mm[2] / 1000.0
    m = mass_kg
    return {
        "ixx": inertia["ixx"] + m * (d_sq - dx * dx),
        "iyy": inertia["iyy"] + m * (d_sq - dy * dy),
        "izz": inertia["izz"] + m * (d_sq - dz * dz),
        "ixy": inertia.get("ixy", 0) - m * dx * dy,
        "ixz": inertia.get("ixz", 0) - m * dx * dz,
        "iyz": inertia.get("iyz", 0) - m * dy * dz,
    }


# ---------------------------------------------------------------------------
# Generator state
# ---------------------------------------------------------------------------


@dataclass
class _GenState:
    """Everything `_emit_*` helpers need that is shared across base_link and
    the per-leg loop. Built once by `_build_state` from the export + cfg."""

    # config
    cfg: dict
    base_cfg: dict
    leg_tmpl: dict
    mesh_dir: str
    servo_cfg: dict
    effort: float
    vel: float
    # export
    export: dict
    occs: list
    # joint kinematics
    joint_defs: list
    fl_rest_rad: float
    shoulder_lower_deg: float
    shoulder_upper_deg: float
    # leg-assembly construction points (resolved once)
    L_axis_offset: list
    R_axis_offset: list
    # per-role servo offsets in their parent frames
    shoulder_servo_L_offset: list | None
    shoulder_servo_R_offset: list | None
    hip_servo_offset_in_link1: list | None
    knee_servo_offset_in_link3: list | None
    # per-role servo orientations
    hip_servo_rpy: tuple
    knee_servo_rpy: tuple
    # discovered servo mesh filename (re-origined to ServoMountPoint)
    servo_mesh_name: str | None
    # foot tip in link3-mesh-local frame (mm), computed once
    foot_tip_link3_mm: list | None


def _build_state(export: dict, cfg: dict) -> _GenState:
    """Resolve everything that's shared across the base_link emit and the
    per-leg loop: joint definitions, construction-point offsets, per-role
    servo placements + orientations, and the servo mesh filename."""
    occs = export["occurrences"]
    servo_cfg = cfg.get("servo", {})
    leg_tmpl = cfg["leg_template"]

    # Sanity check the configured leg assembly exists in the export.
    leg_occ = find_occurrence(occs, leg_tmpl["leg_assembly_occurrence"])
    if not leg_occ:
        raise ValueError(
            f"Leg assembly occurrence not found: {leg_tmpl['leg_assembly_occurrence']}"
        )

    # Joint definitions — axes, origins, limits, FL rest pose.
    jd_result = build_joint_definitions(export, cfg)

    # Per-side offsets in normalized (world-aligned) frame. See
    # `resolve_leg_construction_points` for the geometry.
    cpts = resolve_leg_construction_points(occs)

    # Standalone-servo design (no bake-in): each leg gets a shoulder, hip,
    # and knee servo emitted as separate <visual> blocks. The mesh
    # `servo.stl` is re-origined to ServoMountPoint; the visual's xyz
    # is the world position where ServoMountPoint should land.
    shoulder_servo_L_offset = (
        sub(cpts["shoulder_servo_L"], cpts["mount_L"])
        if cpts["shoulder_servo_L"]
        else None
    )
    shoulder_servo_R_offset = (
        sub(cpts["shoulder_servo_R"], cpts["mount_R"])
        if cpts["shoulder_servo_R"]
        else None
    )
    hip_servo_offset_in_link1 = (
        sub(cpts["hip_servo"], cpts["body_to_link1"]) if cpts["hip_servo"] else None
    )
    knee_servo_offset_in_link3 = (
        sub(cpts["knee_servo"], cpts["link2_to_link3"]) if cpts["knee_servo"] else None
    )

    # Per-role servo orientations. The shared `servo.stl` was exported via
    # combined-rule from `LegBaseServoEnclosure:1` (the shoulder servo),
    # which bakes vertices in WORLD frame using its `world_transform_rm_cm`.
    # So mesh-local axes equal world axes for the SHOULDER placement —
    # shaft along +Z. The hip and knee servos in CAD have different world
    # rotations (shaft along +Y), so reusing the same mesh on link1 / link3
    # needs a per-role rpy that takes the mesh's shoulder-orientation back
    # to the role's CAD orientation:
    #
    #     M_role = R_role · R_shoulder^T
    #
    # Computed from JSON, this gives:
    #     M_hip  = Rx(-π/2)            →  rpy = (-π/2, 0, 0)
    #     M_knee = Rz(-π/2)·Ry(-π/2)   →  rpy = (0, -π/2, -π/2)
    R_servo1 = _find_occ_rot(occs, "LegBaseServoEnclosure:1")  # shoulder
    R_servo2 = _find_occ_rot(occs, "LegBaseServoEnclosure:2")  # hip
    R_servo3 = _find_occ_rot(occs, "LegBaseServoEnclosure:3")  # knee

    def _relative_rpy(R_target):
        if R_target is None or R_servo1 is None:
            return (0.0, 0.0, 0.0)
        return _rot_to_urdf_rpy(_mat_mul_3x3(R_target, _mat_transpose_3x3(R_servo1)))

    # The servo mesh is the manifest entry whose origin_landmark is
    # ServoMountPoint. Works for both body-rule and combined-rule entries
    # (the latter only has `parts`, no `source_body`).
    servo_mesh_name = None
    for mesh_name, entry in (export.get("mesh_files") or {}).items():
        if entry.get("origin_landmark") == "ServoMountPoint":
            servo_mesh_name = mesh_name
            break

    # Foot tip in link3-mesh-local frame (mm). Prefer the explicit Fusion
    # construction point; fall back to STL distance-from-origin heuristic.
    foot_tip_mm = _foot_tip_from_export(export)
    if foot_tip_mm is None:
        foot_tip_mm = _foot_tip_from_stl(
            GENERATED_DIR / cfg["mesh_dir"].rstrip("/\\") / "leg_lower.stl"
        )

    return _GenState(
        cfg=cfg,
        base_cfg=cfg["base_link"],
        leg_tmpl=leg_tmpl,
        mesh_dir=cfg["mesh_dir"],
        servo_cfg=servo_cfg,
        effort=servo_cfg.get("effort_nm", 1.47),
        vel=servo_cfg.get("velocity_rad_s", 5.0),
        export=export,
        occs=occs,
        joint_defs=jd_result.joints,
        fl_rest_rad=jd_result.fl_rest_rad,
        shoulder_lower_deg=jd_result.shoulder_lower_deg,
        shoulder_upper_deg=jd_result.shoulder_upper_deg,
        L_axis_offset=cpts["L_axis_offset"],
        R_axis_offset=cpts["R_axis_offset"],
        shoulder_servo_L_offset=shoulder_servo_L_offset,
        shoulder_servo_R_offset=shoulder_servo_R_offset,
        hip_servo_offset_in_link1=hip_servo_offset_in_link1,
        knee_servo_offset_in_link3=knee_servo_offset_in_link3,
        hip_servo_rpy=_relative_rpy(R_servo2),
        knee_servo_rpy=_relative_rpy(R_servo3),
        servo_mesh_name=servo_mesh_name,
        foot_tip_link3_mm=foot_tip_mm,
    )


def _rotate_z(v, angle_deg):
    if not v:
        return v
    c = math.cos(math.radians(angle_deg))
    s = math.sin(math.radians(angle_deg))
    return [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]]


# ---------------------------------------------------------------------------
# Emitters
# ---------------------------------------------------------------------------


def _emit_base_link(urdf: URDF, state: _GenState) -> None:
    """Emit the chassis <link> plus, per leg, a bracket mesh and the
    chassis-fixed shoulder servo."""
    urdf.comment("BASE LINK")
    base_occ = find_occurrence(state.occs, "FlexibleSkeleton:1")
    mass, com, inertia = get_physics(
        base_occ, fallback_mass=0.5, comp_name="FlexibleSkeleton:1"
    )
    # Fusion only knows about the printed chassis; the electronics live in
    # config (see base_link.extra_mass_kg). COM/inertia stay Fusion's: the
    # electronics are roughly co-located with the chassis centroid, so the
    # static-torque error from skipping a COM shift is negligible for the
    # standstill check this is sized for.
    extra_mass_kg = float(state.base_cfg.get("extra_mass_kg", 0.0) or 0.0)
    if extra_mass_kg:
        mass = mass + extra_mass_kg

    # base_link gets two kinds of chassis-fixed visuals per leg, both placed
    # at the leg's LegMountPointXX in body frame:
    #
    #   1. Bracket mesh (leg_mount_L.stl for FL/BR, leg_mount_R.stl for
    #      FR/BL). Re-origined to LegMountFixedPoint in mesh-local frame, so
    #      xyz=mount_world, rpy=identity drops it on the body's mating
    #      point exactly. Diagonal-pair flip — same mesh on opposite-
    #      diagonal corners — needs the back-of-pair leg's bracket to
    #      rotate 180° around its own mount Z so its outer face points
    #      outward; rpy=(0,0,π) does that.
    #
    #   2. Shoulder servo (servo.stl, re-origined to ServoMountPoint).
    #      Chassis-fixed too — bolted to the bracket, doesn't rotate with
    #      the leg. Uniform orientation per the source FL servo rotation so
    #      all four servos look identical instead of inheriting the per-
    #      bracket Rx/Ry/Rz(180°) flip-flopping that put two of them below
    #      the chassis.
    base_extra_visuals = []
    for leg in state.cfg["legs"]:
        mount_mm = find_point_in_tree(state.occs, leg["mount_point"])
        if not mount_mm:
            print(
                f"warning: mount point {leg['mount_point']!r} not found; "
                f"skipping bracket visual for {leg['id']}"
            )
            continue
        side = leg.get("side", "L")
        rpy_z_deg = _back_of_pair_rpy_z_deg(leg["id"])
        bracket_rpy = (0.0, 0.0, math.radians(rpy_z_deg))

        bracket_mesh = f"leg_mount_{side}.stl"
        if bracket_mesh in (state.export.get("mesh_files") or {}):
            base_extra_visuals.append((bracket_mesh, list(mount_mm), bracket_rpy))
        else:
            print(
                f"warning: {bracket_mesh} not in mesh_files; "
                f"skipping bracket visual for {leg['id']}"
            )

        # Shoulder servo. Position = mount tab + rotated side-offset.
        # No extra "mirror" rotation — the CAD has the L-bracket and
        # R-bracket nested servos at slightly different world rotations
        # (R_R · R_L^T = Ry(π)), but applying that would flip the servo's
        # Z-direction (shaft) which is geometrically wrong for an
        # unflippable physical part.
        ss_offset = (
            state.shoulder_servo_L_offset
            if side == "L"
            else state.shoulder_servo_R_offset
        )
        if state.servo_mesh_name and ss_offset is not None:
            rotated = _rotate_z(ss_offset, rpy_z_deg)
            ss_xyz = [mount_mm[i] + rotated[i] for i in range(3)]
            base_extra_visuals.append((state.servo_mesh_name, ss_xyz, bracket_rpy))
        elif state.servo_mesh_name:
            print(
                f"warning: shoulder-servo ServoMountPoint not found for "
                f"{side}-side bracket; skipping shoulder servo visual for "
                f"{leg['id']}"
            )

    urdf.link(
        state.base_cfg["name"],
        state.base_cfg["mesh"],
        state.mesh_dir,
        mass,
        com,
        inertia,
        origin_shift_mm=_mesh_shift(state.export, state.base_cfg["mesh"]),
        extra_visuals=base_extra_visuals,
    )
    # Audit: base_link — count servo.stl visuals on it (shoulder servos, chassis-fixed)
    base_servo_count = sum(
        1 for ev in base_extra_visuals
        if ev[0] == (state.servo_mesh_name or "servo.stl")
    )
    _audit_rows.append((
        state.base_cfg["name"],
        mass * 1000.0,  # final URDF mass in g
        mass * 1000.0,  # CAD mass pre-servo = final for base_link
        base_servo_count,
        0.0,            # servo mass gap g
        mass * 1000.0,  # total expected = final
    ))


def _emit_leg_assembly_metadata(urdf: URDF, state: _GenState) -> None:
    """Emit the machine-parseable <!-- LEG ASSEMBLY METADATA --> comment
    block. Downstream tools (simulate) read this to avoid re-opening
    fusion_export.json — the URDF remains the single source of truth for
    the leg chain geometry. Prefer the explicit Fusion construction point
    (`Link3TipPoint` relative to `Link2ToLink3Axis`, rotated by Rz(90°));
    fall back to the STL distance-from-origin heuristic when absent."""
    tip_mm = _foot_tip_from_export(state.export)
    if tip_mm is not None:
        tip_source = "Link3TipPoint construction point, in link3 frame"
    else:
        tip_mm = _foot_tip_from_stl(
            GENERATED_DIR / state.mesh_dir.rstrip("/\\") / "leg_lower.stl"
        )
        tip_source = "leg_lower.stl max-distance-from-origin centroid, in link3 frame"
    jd = state.joint_defs
    pts_comment = [
        "LEG ASSEMBLY METADATA (mm, leg-assembly-local frame)",
        f"  BodyToLink1Point : {jd[0].origin_local_mm[0]:.3f} "
        f"{jd[0].origin_local_mm[1]:.3f} {jd[0].origin_local_mm[2]:.3f}",
        f"  Link1ToLink2Point: {jd[1].origin_local_mm[0]:.3f} "
        f"{jd[1].origin_local_mm[1]:.3f} {jd[1].origin_local_mm[2]:.3f}",
        f"  Link2ToLink3Point: {jd[2].origin_local_mm[0]:.3f} "
        f"{jd[2].origin_local_mm[1]:.3f} {jd[2].origin_local_mm[2]:.3f}",
        f"  FootTip ({tip_source}): {tip_mm[0]:.3f} {tip_mm[1]:.3f} {tip_mm[2]:.3f}",
    ]
    urdf.comment("\n       ".join(pts_comment))


def _emit_leg(urdf: URDF, leg: dict, state: _GenState) -> None:
    """Emit one leg's joint chain + three links.

    Per leg:
      * Shoulder joint origin = LegMountPointXX_world + Rz(rpy_z) ·
        side_axis_offset, where side_axis_offset is the world-frame vector
        from the mounting tab to the rotation axis (BodyToLink1Point) for
        that side's bracket.
      * Hip/knee joint origins are leg-assembly-local deltas in the
        normalized frame. For the R pair they're X-mirrored from the L
        values (the R bracket and Link1R are mirrored about the leg-
        assembly XZ plane, which after R_la maps to a world-X mirror).
      * Link2/Link3 are shared meshes; for the R pair we apply
        mesh_rpy = (0, π, 0) so the leg geometry extends in +X (mesh-local)
        instead of -X.
      * Hip servo (servo₂) is rigid with link1 per Link1RigidGroup;
        emitted as a standalone <visual> on link1 at the source-CAD
        hip-servo offset (X-flipped for R pair).
      * Knee servo (servo₃) same idea on link3.
    """
    leg_id = leg["id"]
    side = leg.get("side", "L")

    mount_mm = find_point_in_tree(state.occs, leg["mount_point"])
    if not mount_mm:
        raise ValueError(f"Mount point not found in export: {leg['mount_point']}")

    # Geometric back-of-pair rotation (0° front, 180° back) — used to
    # position the shoulder joint origin in world. Distinct from the
    # kinematic shoulder rest, which goes into the URDF joint's <origin rpy>.
    geometric_rpy_z_deg = _back_of_pair_rpy_z_deg(leg_id)

    # Per-side axis offset (mounting-tab → rotation-axis), then rotate by
    # the back-of-pair Rz so the shoulder pivot lands on BodyToLink1Point
    # for this corner.
    axis_offset = state.L_axis_offset if side == "L" else state.R_axis_offset
    rotated_axis_offset = _rotate_z(axis_offset, geometric_rpy_z_deg)
    shoulder_origin_xyz = [mount_mm[i] + rotated_axis_offset[i] for i in range(3)]

    # Shoulder rest baked into the URDF rpy comes from FL's Fusion-export
    # rest, mirrored per-leg via _shoulder_rest_for. URDF θ=0 then equals
    # each leg's mechanical zero (the splayed-out resting pose).
    sj = state.joint_defs[0]
    shoulder_rest_rad = _shoulder_rest_for(leg_id, state.fl_rest_rad)
    shoulder_rest_deg = math.degrees(shoulder_rest_rad)

    # Shoulder axis is uniform across all four legs: the servo shaft points
    # in the same direction (+Z in body frame) regardless of which corner.
    # The link1 bracket STL is mirrored for FR/BL, but the servo itself is
    # not — the shaft direction does not flip. Hip/knee DO get the R-side
    # flip because those servos are physically rotated 180° for FR/BL.
    shoulder_axis = list(sj.axis_dir)
    # Shoulder limit window is per-body-side asymmetric. With the uniform +Z
    # axis the URDF +θ direction is the same right-hand-rule rotation for
    # every leg, but the physical "outward" (loose) side is mirrored across
    # the body-X axis: for LEFT-body legs the loose side is +θ (URDF upper),
    # for RIGHT-body legs it's −θ (URDF lower). Mirror the L-derived window
    # for right-side legs so each shoulder's URDF <limit> reflects the actual
    # mechanical envelope on that side. Pinned by the same constants the
    # firmware safety net (config.h SHOULDER_THETA_NARROW/WIDE_DEG) and the
    # Blender rig builder (LIMIT_ROTATION + use_ik_limit_z) consume.
    #
    # NB: the `side` field in facehugger_config.yaml is the BRACKET pairing
    # (L-bracket = FL+BR, R-bracket = FR+BL), used for hip/knee axis flipping.
    # Body-side (which is what the shoulder asymmetry actually depends on)
    # comes from the leg_id suffix: 'r' → right body, 'l' → left body.
    is_left_body = leg_id.endswith("l")
    if is_left_body:
        sh_lo, sh_hi = state.shoulder_lower_deg, state.shoulder_upper_deg
    else:
        sh_lo, sh_hi = -state.shoulder_upper_deg, -state.shoulder_lower_deg

    urdf.comment(
        f"LEG: {leg_id.upper()}  (side={side}, "
        f"shoulder_rest={shoulder_rest_deg:+.1f}°, "
        f"limits=[{sh_lo:+.1f}°, {sh_hi:+.1f}°])"
    )

    # Shoulder joint: origin = world position of the rotation axis for this
    # corner (= mount + rotated side-offset). rpy_z carries the leg's
    # mechanical rest so URDF θ=0 lands at the Fusion mechanical zero.
    urdf.joint(
        name=f"{leg_id}_{sj.urdf_name}_joint",
        jtype="revolute",
        parent=state.base_cfg["name"],
        child=f"{leg_id}_link1",
        origin_mm=shoulder_origin_xyz,
        axis=shoulder_axis,
        lower_deg=sh_lo,
        upper_deg=sh_hi,
        effort=state.effort,
        velocity=state.vel,
        rpy_z_deg=shoulder_rest_deg,
    )

    # Hip + knee joints. For the R pair we apply two flips so that the SAME
    # stance-angle value produces the SAME physical motion across all four
    # legs (no per-side hacks needed in simulate / IK / gait controllers):
    #
    #   1. X-flip the joint origin (geometry: link2/link3 are on the +X side
    #      of link1 for R pair, -X for L pair).
    #   2. Negate the joint axis (sign convention: in L-pair conventions,
    #      hip=-40° drops the leg; in R-pair the same physical motion needs
    #      hip=+40° because the mesh and offset flipped together. Negating
    #      the axis flips the sign convention, so hip=-40° drops the R pair
    #      too).
    #   3. Negate-and-swap the joint limits (same physical range, expressed
    #      in the flipped sign convention).
    for i, jd in enumerate(state.joint_defs[1:], start=1):
        offset = list(jd.offset_from_parent_mm)
        axis_dir = list(jd.axis_dir)
        lim_lo, lim_hi = jd.limits_deg
        if side == "R":
            axis_dir, lim_lo, lim_hi, offset = flip_joint_for_r_side(
                axis_dir, lim_lo, lim_hi, offset
            )
        urdf.joint(
            name=f"{leg_id}_{jd.urdf_name}_joint",
            jtype="revolute",
            parent=f"{leg_id}_link{i}",
            child=f"{leg_id}_link{i + 1}",
            origin_mm=offset,
            axis=axis_dir,
            lower_deg=lim_lo,
            upper_deg=lim_hi,
            effort=state.effort,
            velocity=state.vel,
        )

    # Standalone servo visuals on link1 (hip) and link3 (knee). The shared
    # servo.stl is the same physical part on every leg — we don't apply a
    # mirror approximation; only:
    #   1. X-flip the mounting position for R pair so the servo sits at the
    #      mirrored attach point.
    #   2. Apply the per-role rpy (hip_servo_rpy / knee_servo_rpy) so the
    #      mesh's shoulder-baked orientation rotates to the hip/knee CAD
    #      orientation (shaft along +Y instead of +Z).
    # Any per-leg orientation difference (back-of-pair Rz(π)) cascades
    # automatically from the parent link's frame.
    link_extra_servos = {"link1": [], "link2": [], "link3": []}
    if state.servo_mesh_name:
        if state.hip_servo_offset_in_link1 is not None:
            hip_xyz = list(state.hip_servo_offset_in_link1)
            if side == "R":
                hip_xyz[0] = -hip_xyz[0]
            link_extra_servos["link1"].append(
                (state.servo_mesh_name, hip_xyz, state.hip_servo_rpy)
            )
        if state.knee_servo_offset_in_link3 is not None:
            knee_xyz = list(state.knee_servo_offset_in_link3)
            if side == "R":
                knee_xyz[0] = -knee_xyz[0]
            link_extra_servos["link3"].append(
                (state.servo_mesh_name, knee_xyz, state.knee_servo_rpy)
            )

    # Per-link visual rpy: link2 / link3 mesh is shared (L-flavor); rotate
    # 180° about Y for R pair so leg extends in +X.
    link_mesh_rpy = {
        "link1": None,
        "link2": (0.0, math.pi, 0.0) if side == "R" else None,
        "link3": (0.0, math.pi, 0.0) if side == "R" else None,
    }

    for link_key, link_cfg in state.leg_tmpl["links"].items():
        link_num = link_key.replace("link", "")
        link_name = f"{leg_id}_link{link_num}"
        # Substitute {side} in the mesh template (e.g. leg_shoulder_L.stl
        # for FL/BR, leg_shoulder_R.stl for FR/BL). Templates without the
        # token (leg_upper.stl, leg_lower.stl) format unchanged.
        mesh_name = link_cfg["mesh"].format(side=side)
        link_occ_path = _primary_link_occurrence(
            (state.export.get("mesh_files") or {}).get(mesh_name)
        )
        link_occ = (
            find_occurrence(state.occs, link_occ_path.split("/")[-1])
            if link_occ_path
            else None
        )
        mass, com, inertia = get_physics(
            link_occ, fallback_mass=0.05, comp_name=mesh_name
        )
        cad_mass_g = mass * 1000.0  # before servo addition

        # Servo mass: if include_servo_mass is true, treat each servo on this
        # link as a point mass and add its contribution to both mass and
        # inertia via the parallel axis theorem.
        include_servo = bool(state.servo_cfg.get("include_servo_mass", False))
        servo_mass_kg = float(state.servo_cfg.get("mass_kg", 0.060))
        servo_positions = []  # (xyz_mm) positions in link frame
        if link_key in ("link1", "link3") and include_servo:
            if link_key == "link1" and state.hip_servo_offset_in_link1 is not None:
                sp = list(state.hip_servo_offset_in_link1)
                if side == "R":
                    sp[0] = -sp[0]
                servo_positions.append(sp)
            if link_key == "link3" and state.knee_servo_offset_in_link3 is not None:
                sp = list(state.knee_servo_offset_in_link3)
                if side == "R":
                    sp[0] = -sp[0]
                servo_positions.append(sp)

            for sp in servo_positions:
                # New mass
                new_mass = mass + servo_mass_kg
                # New COM: weighted average
                new_com = [
                    (mass * com[i] + servo_mass_kg * sp[i]) / new_mass
                    for i in range(3)
                ]
                # Shift original inertia to new COM, then add point-mass contribution
                inertia = _shift_inertia_to_com(inertia, mass, com, new_com)
                pm = _point_mass_inertia(servo_mass_kg, sp, new_com)
                for k in inertia:
                    inertia[k] = inertia[k] + pm.get(k, 0)
                mass = new_mass
                com = new_com
        elif link_key in ("link1", "link3"):
            # Legacy: add mass without COM/inertia adjustment (kept for backward
            # compat when include_servo_mass is false).
            mass += servo_mass_kg

        # Collision override: analytic primitives replace STL meshes.
        # - link1, link2: box along the bone direction
        # - link3: sphere at foot tip (single clean contact point)
        collision_override = None
        if link_key == "link3" and state.foot_tip_link3_mm is not None:
            ft = state.foot_tip_link3_mm
            if side == "R":
                ft = [-ft[0], ft[1], -ft[2]]
            collision_override = CollisionPrimitive(
                shape="sphere",
                origin_xyz_mm=list(ft),
                radius=0.005,
            )
        elif link_key in ("link1", "link2"):
            # Bone vector from this link's origin to the next joint.
            # link1: joint_defs[1] = Link2Revolute offset from link1 frame
            # link2: joint_defs[2] = Link3Revolute offset from link2 frame
            link_idx = int(link_key[-1])  # "1" or "2"
            bone = list(state.joint_defs[link_idx].offset_from_parent_mm)
            if side == "R":
                bone[0] = -bone[0]
            bx, by, bz = bone
            bone_len = math.hypot(bx, math.hypot(by, bz))
            if bone_len > 1e-6:
                # Box centered at midpoint, +X aligned with bone direction.
                # URDF RPY: Rz(yaw)*Ry(pitch)*Rx(0) maps +X to bone unit vector.
                mid = [bx / 2, by / 2, bz / 2]
                nx, ny, nz = bx / bone_len, by / bone_len, bz / bone_len
                pitch = math.asin(max(-1.0, min(1.0, -nz)))
                cp = math.cos(pitch)
                yaw = math.atan2(ny, nx) if abs(cp) > 1e-9 else 0.0
                collision_override = CollisionPrimitive(
                    shape="box",
                    origin_xyz_mm=mid,
                    origin_rpy=(0.0, pitch, yaw),
                    size_xyz_mm=[bone_len, 15.0, 15.0],
                )

        urdf.link(
            link_name,
            mesh_name,
            state.mesh_dir,
            mass,
            com,
            inertia,
            origin_shift_mm=_mesh_shift(state.export, mesh_name),
            extra_visuals=link_extra_servos.get(link_key, []),
            mesh_rpy=link_mesh_rpy.get(link_key),
            collision_override=collision_override,
        )
        # Audit tracking
        sv_count = len(link_extra_servos.get(link_key, []))
        servo_gap_g = servo_mass_kg * 1000.0 * sv_count
        total_expected_g = cad_mass_g + servo_gap_g
        _audit_rows.append((link_name, mass * 1000.0, cad_mass_g, sv_count, servo_gap_g, total_expected_g))


def generate(export: dict, cfg: dict, out_path: Path):
    global _audit_rows
    _audit_rows = []
    state = _build_state(export, cfg)
    urdf = URDF(cfg["robot_name"])
    _emit_base_link(urdf, state)
    _emit_leg_assembly_metadata(urdf, state)
    for leg in cfg["legs"]:
        _emit_leg(urdf, leg, state)
    urdf.save(out_path)
    _print_mass_audit_table(state)
    _print_invariant_check(state.occs, cfg, state.joint_defs)
    _print_limits_summary(state)


def _print_mass_audit_table(state: _GenState):
    """Print the servo mass audit table after URDF generation."""
    servo_mass_g = float(state.servo_cfg.get("servo_mass_g",
                                               state.servo_cfg.get("mass_kg", 0.060) * 1000.0))
    include_servo = bool(state.servo_cfg.get("include_servo_mass", False))
    print()
    print("=== URDF mass audit (servo mass per-link) ===")
    print(f"  servo_mass_g = {servo_mass_g:.1f} g")
    print(f"  include_servo_mass = {include_servo}")
    header = ("  {:<14s} {:>14s} {:>14s} {:>18s} {:>14s}"
              .format("Link", "CAD mass (g)", "Servo visuals", "Servo mass gap (g)", "Total URDF (g)"))
    print(header)
    total_cad = 0.0
    total_gap = 0.0
    total_urdf = 0.0
    for row in _audit_rows:
        link_name, urdf_mass_g, cad_mass_g, sv_count, servo_gap_g, total_expected_g = row
        total_cad += cad_mass_g
        total_gap += servo_gap_g
        total_urdf += urdf_mass_g
        print("  {:<14s} {:>14.1f} {:>14d} {:>18.1f} {:>14.1f}"
              .format(link_name, cad_mass_g, int(sv_count), servo_gap_g, urdf_mass_g))
    print("  " + "-" * 76)
    print("  {:<14s} {:>14.1f} {:>14s} {:>18.1f} {:>14.1f}"
          .format("TOTAL", total_cad, "", total_gap, total_urdf))
    print("=" * 80)


def _print_invariant_check(occs: list, cfg: dict, joint_defs: list):
    """Post-generation diagnostic: print where each leg's shoulder axis
    lands in world frame (= its LegMountPointXX construction point)."""
    print("\n=== Shoulder axis world positions (mm) ===")
    for leg in cfg["legs"]:
        leg_id = leg["id"]
        mount_pos = find_point_in_tree(occs, leg["mount_point"])
        if mount_pos is None:
            print(f"  {leg_id:>2}: missing mount point {leg['mount_point']!r}")
            continue
        print(
            f"  {leg_id:>2}: ({mount_pos[0]:+7.2f}, {mount_pos[1]:+7.2f}, "
            f"{mount_pos[2]:+7.2f})"
        )


# ---------------------------------------------------------------------------
# Limits summary — per-leg, per-joint across three angle spaces
# ---------------------------------------------------------------------------

def _print_limits_summary(state: _GenState):
    """Print per-leg, per-joint limits across three angle spaces.

    Columns:
      Fusion   — absolute CAD joint angles from the Fusion export (degrees)
      URDF     — <limit lower/upper> as written to facehugger.urdf (displacement from rest)
      Math     — absolute math-space angle = rest + URDF limit (body frame, CCW +)
    """
    joint_defs = state.joint_defs
    fl_rest_rad = state.fl_rest_rad

    def _norm(a):
        a = math.fmod(a, 360.0)
        if a > 180.0:
            a -= 360.0
        elif a <= -180.0:
            a += 360.0
        return a

    fusion_lo = [
        math.degrees(joint_defs[0].limits_rad["min"]),
        joint_defs[1].limits_deg[0],
        joint_defs[2].limits_deg[0],
    ]
    fusion_hi = [
        math.degrees(joint_defs[0].limits_rad["max"]),
        joint_defs[1].limits_deg[1],
        joint_defs[2].limits_deg[1],
    ]

    print()
    print("=== Joint limits — Fusion → URDF → Math-space ===")
    print("    Fusion = absolute CAD joint angles (FL reference for link1)")
    print("    URDF   = <limit lower/upper> displacement from rest")
    print("    Math   = rest + URDF limit  (absolute, body frame, CCW +)")

    joint_names = ["link1 (shoulder)", "link2 (thigh)", "link3 (knee)"]

    for joint_idx, jname in enumerate(joint_names):
        print()
        print(f"  ── {jname} ──")
        hdr = f"  {'Leg':>4s}  {'Fusion':>16s}  {'URDF':>16s}  {'Math':>16s}"
        print(hdr)
        print(f"  {'':4s}  {'lo':>7s} {'hi':>7s}   {'lo':>7s} {'hi':>7s}   {'lo':>7s} {'hi':>7s}")
        print("  " + "─" * (len(hdr) - 2))

        for leg in ("fr", "fl", "br", "bl"):
            side = "R" if leg in ("fr", "bl") else "L"
            is_left_body = leg.endswith("l")
            rest_deg = math.degrees(_shoulder_rest_for(leg, fl_rest_rad))

            if joint_idx == 0:
                f_lo, f_hi = fusion_lo[0], fusion_hi[0]
                if is_left_body:
                    u_lo, u_hi = state.shoulder_lower_deg, state.shoulder_upper_deg
                else:
                    u_lo, u_hi = -state.shoulder_upper_deg, -state.shoulder_lower_deg
                m_lo = _norm(rest_deg + u_lo)
                m_hi = _norm(rest_deg + u_hi)
            else:
                f_lo, f_hi = fusion_lo[joint_idx], fusion_hi[joint_idx]
                if side == "R":
                    u_lo, u_hi = -f_hi, -f_lo
                else:
                    u_lo, u_hi = f_lo, f_hi
                m_lo, m_hi = u_lo, u_hi

            print(
                f"  {leg.upper():>4s}  "
                f"{f_lo:>+7.0f}° {f_hi:>+7.0f}°   "
                f"{u_lo:>+7.0f}° {u_hi:>+7.0f}°   "
                f"{m_lo:>+7.0f}° {m_hi:>+7.0f}°"
            )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Generate FaceHugger URDF from Fusion export"
    )
    parser.add_argument(
        "--export",
        type=Path,
        default=DEFAULT_JSON,
        help=f"Path to fusion_export.json (default: {DEFAULT_JSON})",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CFG,
        help=f"Path to facehugger_config.yaml (default: {DEFAULT_CFG})",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=DEFAULT_OUT,
        help=f"Output URDF path (default: {DEFAULT_OUT})",
    )
    args = parser.parse_args()

    print(f"Loading export : {args.export}")
    print(f"Loading config : {args.config}")

    export = load_export(args.export)
    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    generate(export, cfg, args.out)


if __name__ == "__main__":
    main()
