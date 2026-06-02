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
from urdf_gen.lib.urdf_writer import URDF


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


# ---------------------------------------------------------------------------
# Generator
# ---------------------------------------------------------------------------


def generate(export: dict, cfg: dict, out_path: Path):
    occs = export["occurrences"]
    robot = cfg["robot_name"]
    mesh_dir = cfg["mesh_dir"]
    servo_cfg = cfg.get("servo", {})
    effort = servo_cfg.get("effort_nm", 1.47)
    vel = servo_cfg.get("velocity_rad_s", 5.0)
    # `visual_flip_rpy_deg` was a workaround for when servos were emitted
    # with CAD-source orientation that pointed the wrong way. With the
    # standalone-servos design (post-Phase G) the mesh is re-origined to
    # ServoMountPoint and placed at world ServoMountPoint per leg, so no
    # blanket orientation hack is needed. Field is read for backward yaml
    # compat but not applied.
    _ = servo_cfg.get("visual_flip_rpy_deg", [0.0, 0.0, 0.0])

    urdf = URDF(robot)

    # --- joint kinematics: CAD-sourced from export["joints"] ---
    # The Fusion exporter walks every Joint / AsBuiltJoint and emits an
    # entry with axis_dir_local_unit, axis_origin_local_mm, and limits_rad
    # (rest/min/max). The cad-name → urdf-name + parent/child mapping is
    # fixed by convention (Link1Revolute = shoulder, etc.) so it lives
    # here rather than in the yaml.
    # `urdf_name` here is the joint-name suffix (e.g. urdf_name="link1"
    # → joint name "fl_link1_joint"), aligned with the Fusion CAD
    # `LinkNRevolute` naming so the URDF, the JSON export, and the CAD
    # all share one mental model. The semantic mapping
    # link1=shoulder, link2=hip, link3=knee lives in
    # helpers._joint_type_from_name.
    leg_tmpl = cfg["leg_template"]
    leg_occ = find_occurrence(occs, leg_tmpl["leg_assembly_occurrence"])
    if not leg_occ:
        raise ValueError(
            f"Leg assembly occurrence not found: {leg_tmpl['leg_assembly_occurrence']}"
        )

    jd_result = build_joint_definitions(export, cfg)
    joint_defs = jd_result.joints
    fl_rest_rad = jd_result.fl_rest_rad
    shoulder_lower_deg = jd_result.shoulder_lower_deg
    shoulder_upper_deg = jd_result.shoulder_upper_deg

    # --- per-side offsets in normalized (world-aligned) frame ---
    # The leg-assembly normalization in the exporter (E1-E3) aligned the
    # leg-assembly's local axes with world. JSON construction-point
    # `pos_world_mm` values are still genuine world positions; in the
    # source CAD, the L bracket is at LegMountPointFL and the R bracket
    # is sitting at the mirror position. We derive the constant offsets
    # (mount-tab → axis, mount-tab → shoulder-servo, etc.) from the
    # source-CAD positions and reuse them per leg.
    cpts = resolve_leg_construction_points(occs)
    body_to_link1_world = cpts["body_to_link1"]
    link2_to_link3_world = cpts["link2_to_link3"]
    mount_L_world = cpts["mount_L"]
    mount_R_world = cpts["mount_R"]
    shoulder_servo_L_world = cpts["shoulder_servo_L"]
    shoulder_servo_R_world = cpts["shoulder_servo_R"]
    hip_servo_world = cpts["hip_servo"]
    knee_servo_world = cpts["knee_servo"]
    L_axis_offset = cpts["L_axis_offset"]
    R_axis_offset = cpts["R_axis_offset"]

    # Standalone-servo design (no bake-in): each leg gets a shoulder,
    # hip, and knee servo emitted as separate <visual> blocks. The mesh
    # `servo.stl` is re-origined to ServoMountPoint, so the visual's xyz
    # is the world position where ServoMountPoint should land.
    shoulder_servo_L_offset = (
        sub(shoulder_servo_L_world, mount_L_world) if shoulder_servo_L_world else None
    )
    shoulder_servo_R_offset = (
        sub(shoulder_servo_R_world, mount_R_world) if shoulder_servo_R_world else None
    )
    hip_servo_offset_in_link1 = (
        sub(hip_servo_world, body_to_link1_world) if hip_servo_world else None
    )
    knee_servo_offset_in_link3 = (
        sub(knee_servo_world, link2_to_link3_world) if knee_servo_world else None
    )

    # Per-role servo orientations. The shared `servo.stl` was exported
    # via combined-rule from `LegBaseServoEnclosure:1` (the shoulder
    # servo), which bakes vertices in WORLD frame using
    # `LegBaseServoEnclosure:1`'s `world_transform_rm_cm`. So mesh-local
    # axes equal world axes for the SHOULDER placement — shaft along
    # `+Z`. The hip and knee servos in CAD have different world
    # rotations (shaft along `+Y`), so reusing the same mesh on link1
    # / link3 needs a per-role rpy that takes the mesh's
    # shoulder-orientation back to the role's CAD orientation:
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

    hip_servo_rpy = _relative_rpy(R_servo2)
    knee_servo_rpy = _relative_rpy(R_servo3)

    # The servo mesh is the manifest entry whose origin_landmark is
    # ServoMountPoint. Works for both body-rule and combined-rule
    # entries (the latter only has `parts`, no `source_body`).
    servo_mesh_name = None
    for mesh_name, entry in (export.get("mesh_files") or {}).items():
        if entry.get("origin_landmark") == "ServoMountPoint":
            servo_mesh_name = mesh_name
            break

    def _rotate_z(v, angle_deg):
        if not v:
            return v
        c = math.cos(math.radians(angle_deg))
        s = math.sin(math.radians(angle_deg))
        return [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]]

    # --- base_link ---
    urdf.comment("BASE LINK")
    base_cfg = cfg["base_link"]
    base_occ = find_occurrence(occs, "FlexibleSkeleton:1")
    mass, com, inertia = get_physics(
        base_occ, fallback_mass=0.5, comp_name="FlexibleSkeleton:1"
    )
    # Fusion only knows about the printed chassis; the electronics live in
    # config (see base_link.extra_mass_kg). COM/inertia stay Fusion's: the
    # electronics are roughly co-located with the chassis centroid, so the
    # static-torque error from skipping a COM shift is negligible for the
    # standstill check this is sized for.
    extra_mass_kg = float(base_cfg.get("extra_mass_kg", 0.0) or 0.0)
    if extra_mass_kg:
        mass = mass + extra_mass_kg

    # base_link gets two kinds of chassis-fixed visuals per leg, both
    # placed at the leg's LegMountPointXX in body frame:
    #
    #   1. Bracket mesh (leg_mount_L.stl for FL/BR, leg_mount_R.stl for
    #      FR/BL). Re-origined to LegMountFixedPoint in mesh-local frame,
    #      so xyz=mount_world, rpy=identity drops it on the body's
    #      mating point exactly. Diagonal-pair flip — same mesh on
    #      opposite-diagonal corners — needs the back-of-pair leg's
    #      bracket to rotate 180° around its own mount Z so its outer
    #      face points outward; rpy=(0,0,π) does that.
    #
    #   2. Shoulder servo (servo.stl, re-origined to ServoMountPoint).
    #      Chassis-fixed too — bolted to the bracket, doesn't rotate
    #      with the leg. Uniform orientation per the source FL servo
    #      rotation (post-multiplied by visual_flip_rpy_deg) so all
    #      four servos look identical instead of inheriting the per-
    #      bracket Rx/Ry/Rz(180°) flip-flopping that put two of them
    #      below the chassis.
    base_extra_visuals = []

    # Bracket meshes + chassis-fixed shoulder servos — one of each per
    # leg. Bracket visual at LegMountPointXX (in chassis frame), with
    # rpy=(0,0,rpy_z_deg) for the back-of-pair flip. Shoulder servo at
    # mount + Rz(rpy_z) · side_offset (the offset from mounting tab to
    # the bracket's nested-servo ServoMountPoint, in world frame).
    for leg in cfg["legs"]:
        mount_mm = find_point_in_tree(occs, leg["mount_point"])
        if not mount_mm:
            print(
                f"warning: mount point {leg['mount_point']!r} not found; "
                f"skipping bracket visual for {leg['id']}"
            )
            continue
        side = leg.get("side", "L")
        # Geometric back-of-pair flip (chassis-fixed bracket + servo). Derived
        # from leg_id; not the kinematic shoulder rest (that's applied to the
        # URDF joint <origin rpy> only).
        rpy_z_deg = _back_of_pair_rpy_z_deg(leg["id"])
        bracket_rpy = (0.0, 0.0, math.radians(rpy_z_deg))

        bracket_mesh = f"leg_mount_{side}.stl"
        if bracket_mesh in (export.get("mesh_files") or {}):
            base_extra_visuals.append((bracket_mesh, list(mount_mm), bracket_rpy))
        else:
            print(
                f"warning: {bracket_mesh} not in mesh_files; "
                f"skipping bracket visual for {leg['id']}"
            )

        # Shoulder servo. Position = mount tab + rotated side-offset.
        # The shared servo.stl is one physical part placed at each
        # corner; we rotate it only by the back-of-pair flip
        # (rpy = (0, 0, rpy_z_deg)). No extra "mirror" rotation — the
        # CAD has the L-bracket and R-bracket nested servos at slightly
        # different world rotations (R_R · R_L^T = Ry(π)), but applying
        # that would flip the servo's Z-direction (shaft) which is
        # geometrically wrong for an unflippable physical part.
        ss_offset = shoulder_servo_L_offset if side == "L" else shoulder_servo_R_offset
        if servo_mesh_name and ss_offset is not None:
            rotated = _rotate_z(ss_offset, rpy_z_deg)
            ss_xyz = [mount_mm[i] + rotated[i] for i in range(3)]
            base_extra_visuals.append((servo_mesh_name, ss_xyz, bracket_rpy))
        elif servo_mesh_name:
            print(
                f"warning: shoulder-servo ServoMountPoint not found for "
                f"{side}-side bracket; skipping shoulder servo visual for "
                f"{leg['id']}"
            )

    urdf.link(
        base_cfg["name"],
        base_cfg["mesh"],
        mesh_dir,
        mass,
        com,
        inertia,
        origin_shift_mm=_mesh_shift(export, base_cfg["mesh"]),
        extra_visuals=base_extra_visuals,
    )

    # Emit a machine-parseable comment block with the raw leg-assembly-local
    # Points. Downstream tools (simulate) read this to avoid re-opening
    # fusion_export.json — the URDF remains the single source of truth for
    # the leg chain geometry.  Prefer the explicit Fusion construction point
    # (Link3TipPoint relative to Link2ToLink3Axis, rotated by Rz(90°));
    # fall back to the STL distance-from-origin heuristic when absent.
    tip_mm = _foot_tip_from_export(export)
    if tip_mm is not None:
        tip_source = "Link3TipPoint construction point, in link3 frame"
    else:
        tip_mm = _foot_tip_from_stl(
            GENERATED_DIR / mesh_dir.rstrip("/\\") / "leg_lower.stl"
        )
        tip_source = "leg_lower.stl max-distance-from-origin centroid, in link3 frame"
    pts_comment = [
        "LEG ASSEMBLY METADATA (mm, leg-assembly-local frame)",
        f"  BodyToLink1Point : {joint_defs[0].origin_local_mm[0]:.3f} "
        f"{joint_defs[0].origin_local_mm[1]:.3f} "
        f"{joint_defs[0].origin_local_mm[2]:.3f}",
        f"  Link1ToLink2Point: {joint_defs[1].origin_local_mm[0]:.3f} "
        f"{joint_defs[1].origin_local_mm[1]:.3f} "
        f"{joint_defs[1].origin_local_mm[2]:.3f}",
        f"  Link2ToLink3Point: {joint_defs[2].origin_local_mm[0]:.3f} "
        f"{joint_defs[2].origin_local_mm[1]:.3f} "
        f"{joint_defs[2].origin_local_mm[2]:.3f}",
        f"  FootTip ({tip_source}): {tip_mm[0]:.3f} {tip_mm[1]:.3f} {tip_mm[2]:.3f}",
    ]
    urdf.comment("\n       ".join(pts_comment))

    # --- 4 leg instances ---
    # Per leg:
    #   * Shoulder joint origin = LegMountPointXX_world + Rz(rpy_z) ·
    #     side_axis_offset, where side_axis_offset is the world-frame
    #     vector from the mounting tab to the rotation axis
    #     (BodyToLink1Point) for that side's bracket.
    #   * Hip/knee joint origins are leg-assembly-local deltas in the
    #     normalized frame. For the R pair, they're X-mirrored from the
    #     L values (the R bracket and Link1R are mirrored about the
    #     leg-assembly XZ plane, which after R_la maps to a world-X
    #     mirror in the normalized frame).
    #   * Link2/Link3 are shared meshes; for the R pair we apply
    #     mesh_rpy = (0, π, 0) so the leg geometry extends in +X
    #     (mesh-local) instead of -X.
    #   * Hip servo (servo₂) is rigid with link1 per Link1RigidGroup;
    #     emitted as a standalone <visual> on link1 at the source-CAD
    #     hip-servo offset (X-flipped + (0, π, 0) for R pair).
    #   * Knee servo (servo₃) same idea on link3.
    for leg in cfg["legs"]:
        leg_id = leg["id"]
        mount_key = leg["mount_point"]
        side = leg.get("side", "L")

        mount_mm = find_point_in_tree(occs, mount_key)
        if not mount_mm:
            raise ValueError(f"Mount point not found in export: {mount_key}")

        # Geometric back-of-pair rotation (0° front, 180° back) — used to
        # position the shoulder joint origin in world. Distinct from the
        # kinematic shoulder rest, which goes into the URDF joint's
        # <origin rpy>.
        geometric_rpy_z_deg = _back_of_pair_rpy_z_deg(leg_id)

        # Per-side axis offset (mounting-tab → rotation-axis), then rotate
        # by the back-of-pair Rz so the shoulder pivot lands on
        # BodyToLink1Point for this corner.
        axis_offset = L_axis_offset if side == "L" else R_axis_offset
        rotated_axis_offset = _rotate_z(axis_offset, geometric_rpy_z_deg)
        shoulder_origin_xyz = [mount_mm[i] + rotated_axis_offset[i] for i in range(3)]

        # Shoulder rest baked into the URDF rpy comes from FL's Fusion-export
        # rest, mirrored per-leg via _shoulder_rest_for. URDF θ=0 then equals
        # each leg's mechanical zero (the splayed-out resting pose).
        sj = joint_defs[0]
        shoulder_rest_rad = _shoulder_rest_for(leg_id, fl_rest_rad)
        shoulder_rest_deg = math.degrees(shoulder_rest_rad)

        # R-side three-flip on the shoulder (matches hip/knee logic
        # below). The bracket+Link1 are CAD-mirrored about the body's
        # YZ plane for FR/BL, which reverses the servo-shaft direction
        # in body frame. Keeping the same axis vector on every leg
        # would make positive θ rotate FR/BL the wrong physical way
        # and apply asymmetric limits on the wrong half of the sweep.
        # Negating the axis and negate-swapping the limits restores
        # "same θ → same physical motion" across all four legs and
        # lets the FL Fusion limits be set arbitrarily-asymmetric
        # without breaking the right side. The shoulder rpy_z (rest)
        # does NOT change with the axis flip — it is a static rotation
        # in body frame, independent of axis sign.
        shoulder_axis = list(sj.axis_dir)
        sh_lo, sh_hi = shoulder_lower_deg, shoulder_upper_deg
        if side == "R":
            shoulder_axis, sh_lo, sh_hi, _ = flip_joint_for_r_side(
                shoulder_axis, sh_lo, sh_hi
            )

        urdf.comment(
            f"LEG: {leg_id.upper()}  (side={side}, "
            f"shoulder_rest={shoulder_rest_deg:+.1f}°, "
            f"limits=[{sh_lo:+.1f}°, {sh_hi:+.1f}°])"
        )

        # Shoulder joint: origin = world position of the rotation axis for
        # this corner (= mount + rotated side-offset). rpy_z carries the
        # leg's mechanical rest so URDF θ=0 lands at the Fusion mechanical zero.
        urdf.joint(
            name=f"{leg_id}_{sj.urdf_name}_joint",
            jtype="revolute",
            parent=base_cfg["name"],
            child=f"{leg_id}_link1",
            origin_mm=shoulder_origin_xyz,
            axis=shoulder_axis,
            lower_deg=sh_lo,
            upper_deg=sh_hi,
            effort=effort,
            velocity=vel,
            rpy_z_deg=shoulder_rest_deg,
        )

        # Hip + knee joints. For the R pair we apply two flips so that
        # the SAME stance-angle value produces the SAME physical motion
        # across all four legs (no per-side hacks needed in
        # simulate / IK / gait controllers):
        #
        #   1. X-flip the joint origin (geometry: link2/link3 are on
        #      the +X side of link1 for R pair, -X for L pair).
        #   2. Negate the joint axis (sign convention: in L-pair
        #      conventions, hip=-40° drops the leg; in R-pair the same
        #      physical motion needs hip=+40° because the mesh and
        #      offset flipped together. Negating the axis flips the
        #      sign convention, so hip=-40° drops the R pair too).
        #   3. Negate-and-swap the joint limits (same physical range,
        #      expressed in the flipped sign convention).
        for i, jd in enumerate(joint_defs[1:], start=1):
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
                effort=effort,
                velocity=vel,
            )

        # Standalone servo visuals on link1 (hip) and link3 (knee).
        # The shared servo.stl is the same physical part on every leg —
        # we don't apply a mirror approximation; only:
        #   1. X-flip the mounting position for R pair so the servo
        #      sits at the mirrored attach point.
        #   2. Apply the per-role rpy (hip_servo_rpy / knee_servo_rpy)
        #      so the mesh's shoulder-baked orientation rotates to the
        #      hip/knee CAD orientation (shaft along +Y instead of +Z).
        # Any per-leg orientation difference (back-of-pair Rz(π))
        # cascades automatically from the parent link's frame.
        link_extra_servos = {"link1": [], "link2": [], "link3": []}
        if servo_mesh_name:
            if hip_servo_offset_in_link1 is not None:
                hip_xyz = list(hip_servo_offset_in_link1)
                if side == "R":
                    hip_xyz[0] = -hip_xyz[0]
                link_extra_servos["link1"].append(
                    (servo_mesh_name, hip_xyz, hip_servo_rpy)
                )
            if knee_servo_offset_in_link3 is not None:
                knee_xyz = list(knee_servo_offset_in_link3)
                if side == "R":
                    knee_xyz[0] = -knee_xyz[0]
                link_extra_servos["link3"].append(
                    (servo_mesh_name, knee_xyz, knee_servo_rpy)
                )

        # Per-link visual rpy: link2 / link3 mesh is shared (L-flavor);
        # rotate 180° about Y for R pair so leg extends in +X.
        link_mesh_rpy = {
            "link1": None,
            "link2": (0.0, math.pi, 0.0) if side == "R" else None,
            "link3": (0.0, math.pi, 0.0) if side == "R" else None,
        }

        for link_key, link_cfg in leg_tmpl["links"].items():
            link_num = link_key.replace("link", "")
            link_name = f"{leg_id}_link{link_num}"
            # Substitute {side} in the mesh template (e.g. leg_shoulder_L.stl
            # for FL/BR, leg_shoulder_R.stl for FR/BL). Templates without
            # the token (leg_upper.stl, leg_lower.stl) format unchanged.
            mesh_name = link_cfg["mesh"].format(side=side)
            link_occ_path = _primary_link_occurrence(
                (export.get("mesh_files") or {}).get(mesh_name)
            )
            link_occ = (
                find_occurrence(occs, link_occ_path.split("/")[-1])
                if link_occ_path
                else None
            )
            mass, com, inertia = get_physics(
                link_occ, fallback_mass=0.05, comp_name=mesh_name
            )
            # link1 hosts the hip servo; link3 hosts the knee servo.
            # Add the physical servo mass so dynamics match the real robot.
            # CoM stays at the structural centroid (good enough for sim fidelity).
            if link_key in ("link1", "link3"):
                mass += servo_cfg.get("mass_kg", 0.060)
            urdf.link(
                link_name,
                mesh_name,
                mesh_dir,
                mass,
                com,
                inertia,
                origin_shift_mm=_mesh_shift(export, mesh_name),
                extra_visuals=link_extra_servos.get(link_key, []),
                mesh_rpy=link_mesh_rpy.get(link_key),
            )

    urdf.save(out_path)

    _print_invariant_check(occs, cfg, joint_defs)


def _print_invariant_check(occs: list, cfg: dict, joint_defs: list):
    """Post-generation diagnostic: print where each leg's shoulder axis
    lands in world frame (= its LegMountPointXX construction point).
    """
    print("\n=== Shoulder axis world positions (mm) ===")
    for leg in cfg["legs"]:
        leg_id = leg["id"]
        mount_key = leg["mount_point"]
        mount_pos = find_point_in_tree(occs, mount_key)
        if mount_pos is None:
            print(f"  {leg_id:>2}: missing mount point {mount_key!r}")
            continue
        print(
            f"  {leg_id:>2}: ({mount_pos[0]:+7.2f}, {mount_pos[1]:+7.2f}, "
            f"{mount_pos[2]:+7.2f})"
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
