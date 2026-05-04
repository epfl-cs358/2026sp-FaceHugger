"""
urdf_to_blender_rigged.py — Armature-based URDF importer with IK

Sibling of `visualize_urdf.py`. Same URDF parsing + scene bootstrap, but
builds a real Blender Armature so the rig can be posed/animated. The
animator-facing import path (placement-only `visualize_urdf.py` stays as
the cross-check against PyBullet).

Architecture:
  1. Parse URDF, compute link world matrices at rest pose (theta=0).
  2. Build a single Armature in edit mode:
       - One bone per URDF link (12 leg bones + base_link), named to
         match the URDF link names exactly (`fl_link1`, `fr_link2`, …).
       - Bone head = parent joint pivot world.
       - Bone tail = next joint pivot (or for `*_link3`, the foot tip),
         **projected onto the plane perpendicular to that joint's URDF
         axis** so bone-Y ⊥ joint axis. The projection is the
         precondition for `EditBone.align_roll(axis)` to land bone-local
         Z exactly on the joint axis. Without it, bone-local Z drifts
         from the joint axis (≈13° on `*_link1`, ≈23° on `*_link3`),
         breaking the spec's "rotation_euler[2] = joint angle" contract.
         The bone is a rotation control; mesh geometry is set
         independently in `attach_visuals` via `matrix_world`, so the
         visible limb is unaffected.
       - Bone roll = `EditBone.align_roll(joint_axis_world)`. After the
         tail projection above, **bone-local Z = the URDF joint axis**
         for all 12 joints uniformly.
       - Parent in chain via `EditBone.parent`, `use_connect=False`.

Constraints, applied in this order so the constraint stack evaluates as
[IK / DAMPED_TRACK first, then LIMIT_ROTATION] (LIMIT_ROTATION clamps
the IK-or-track-solved pose, never the other way around):

  3. Per leg, two foot Empties + IK + Damped Track:
       - `foot_target_{leg}` (visible, sphere): at the actual CAD foot
         tip in world. Foot tip per leg: `Link3TipAxis` in
         `fusion_export.json` (falls back to
         `FOOT_TIP_FALLBACK_IN_LINK3_FRAME_M`). For R-side legs
         (FR, BL), the X component is negated to match the mirrored
         `leg_lower.stl` visual (`<visual rpy="0 π 0">`). Animator
         drags this.
       - `foot_ik_{leg}` (hidden, plain axes): `COPY_LOCATION` from
         `foot_target` with `use_offset=False` — tracks foot_target
         exactly. Kept as a separate Empty so the animator-facing
         handle is decoupled from the IK target.
       - IK constraint on `*_link3` pose bone: `target=foot_ik`,
         `chain_count=2` (covers link3 + link2), `use_tail=True`,
         `use_rotation=False`, `use_stretch=False`.
       - Damped Track on `*_link1` pose bone: `target=foot_target`,
         `track_axis='TRACK_Y'`. Combined with `LIMIT_ROTATION`'s X/Y
         locks (below), this becomes a yaw-only aim — link1 yaws around
         bone-local Z (= world +Z) so its bone-Y horizontal projection
         points at the foot.

     Known limitation: bone tail is the projected foot tip (drops the
     joint-axis component of the foot tip in link3 frame, ≈ −Y in link3
     frame). Foot_ik / foot_target sit at the actual CAD foot tip. The
     two differ by ~13 mm along the joint axis at rest, so IK has a
     constant ~13 mm residual it cannot close — the gap is along the
     joint axis, perpendicular to every plane the IK can rotate the
     foot through. Cosmetic only; the visible foot mesh tip remains at
     foot_target's location.
  4. `lock_ik_x = lock_ik_y = True; lock_ik_z = False` plus
     `ik_stiffness_x/y = 1.0` plus `use_ik_limit_z` with `ik_min_z` /
     `ik_max_z` from URDF on `*_link2` and `*_link3` pose bones.
     Restricts the IK solver to bone-local Z rotation (= URDF joint
     axis) within the URDF range. link1 isn't in the IK chain so it
     doesn't get IK locks.
  5. `LIMIT_ROTATION` on every joint bone: `use_limit_x=use_limit_y=True`
     with `min=max=0` (forces 1-DOF around bone-local Z), and
     `use_limit_z=True` with `min_z/max_z` from URDF `<limit>`. On link1
     this enforces yaw-only motion under Damped Track; on link2/link3
     it's belt-and-suspenders for the IK locks.
  6. Mesh attach: `<visual>` STLs imported, parented to their link's
     bone, then `obj.matrix_world = link_world @ visual_origin` to
     match `visualize_urdf.py` byte-for-byte at rest.
  7. Joint-pivot/axis markers — `hide_viewport=True` by default.

Usage (GUI — recommended):
    /Applications/Blender-5.1.app/Contents/MacOS/Blender \
        --python animation/scripts/urdf_to_blender_rigged.py

Usage (headless — smoke test):
    /Applications/Blender-5.1.app/Contents/MacOS/Blender --background \
        --python animation/scripts/urdf_to_blender_rigged.py \
        -- --save /tmp/fh_rigged.blend

CLI flags (after the `--` separator per Blender convention):
    --urdf PATH      path to facehugger.urdf
    --meshes PATH    directory containing exported STLs
    --json PATH      fusion_export.json (foot tip from `Link3TipPoint`)
    --save PATH      write a .blend here after building the scene

Targets Blender 5.x.
"""

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import bpy
from mathutils import Matrix, Vector


# ---------------------------------------------------------------------------
# Constants — match visualize_urdf.py for cross-script overlay
# ---------------------------------------------------------------------------

M_TO_MM = 1000.0  # URDF metres → mm scene

SPHERE_RADIUS_MM = 3.0
AXIS_LENGTH_MM = 20.0

ROOT_COLLECTION = "URDFRigged"
ARMATURE_COLLECTION = "Armature"
MESHES_COLLECTION = "Meshes"
JOINTS_COLLECTION = "Joint Origins"
AXES_COLLECTION = "Joint Axes"
TARGETS_COLLECTION = "IK Targets"
CONTROLS_COLLECTION = "Controls"

COLOR_JOINT = (1.0, 0.0, 0.0, 1.0)  # red
COLOR_AXIS = (1.0, 0.4, 0.0, 1.0)  # orange

ARMATURE_NAME = "FaceHuggerRig"

# Foot tip in link3 frame (mm) — leg_lower.stl max-+Y centroid, copied
# from the URDF metadata comment block. This is the AUTHORITATIVE value
# used to place each leg's foot_target Empty and the link3 bone tail
# (after axis-parallel projection). The JSON `Link3TipPoint` is read
# only as a cross-check: if it differs from this constant by > 5 mm,
# main() prints a warning so the CAD construction point gets moved to
# coincide with the mesh tip.
FOOT_TIP_FALLBACK_IN_LINK3_FRAME_M = Vector((-0.030946, -0.013000, 0.000024))

# Naming patterns
LEG_IDS = ("fl", "fr", "bl", "br")
R_SIDE_LEGS = ("fr", "bl")  # legs whose link3 visual has rpy=(0, π, 0)
FOOT_TARGET_FMT = "foot_target_{leg}"
FOOT_IK_FMT = "foot_ik_{leg}"
SHOULDER_BONE_FMT = "{leg}_link1"
KNEE_BONE_FMT = "{leg}_link3"


def _foot_tip_for_leg(leg, foot_tip_in_link3_m):
    """Return the foot tip in link3 frame, accounting for the L/R-side
    mesh mirroring. The leg_lower.stl mesh is shared between sides;
    L-side legs (FL, BR) use it with `<visual rpy="0 0 0">` and R-side
    legs (FR, BL) with `<visual rpy="0 π 0">` — a Y-axis 180° rotation.
    The foot tip, defined in the STL frame, therefore lives at -X in
    link3 frame for R-side legs (relative to the L-side reference value
    loaded from `Link3TipAxis`). Without this flip, FR/BL foot Empties
    are placed on the wrong side of the leg in world.

    The Z component of the foot tip mirrors too in principle, but its
    URDF magnitude is ~24μm — below visual precision, ignored.
    """
    if leg in R_SIDE_LEGS:
        return Vector(
            (-foot_tip_in_link3_m.x, foot_tip_in_link3_m.y, foot_tip_in_link3_m.z)
        )
    return foot_tip_in_link3_m


# ---------------------------------------------------------------------------
# URDF parsing — copy of visualize_urdf.py's parser plus _parse_limit
# ---------------------------------------------------------------------------


def _parse_origin(elem):
    o = elem.find("origin")
    if o is None:
        return Matrix.Identity(4)
    xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
    rx = Matrix.Rotation(rpy[0], 3, "X")
    ry = Matrix.Rotation(rpy[1], 3, "Y")
    rz = Matrix.Rotation(rpy[2], 3, "Z")
    M = (rz @ ry @ rx).to_4x4()
    M.translation = Vector(xyz)
    return M


def _parse_axis(elem):
    a = elem.find("axis")
    if a is None:
        return Vector((1.0, 0.0, 0.0))
    xyz = [float(v) for v in a.get("xyz").split()]
    v = Vector(xyz)
    return v.normalized() if v.length > 0 else Vector((1.0, 0.0, 0.0))


def _parse_limit(elem):
    """Return (lower_rad, upper_rad) from <joint><limit>, or (None, None)
    for fixed joints / missing limits."""
    lim = elem.find("limit")
    if lim is None:
        return (None, None)
    try:
        return (float(lim.get("lower")), float(lim.get("upper")))
    except (TypeError, ValueError):
        return (None, None)


def _parse_visual(v):
    g = v.find("geometry/mesh")
    if g is None:
        return None
    fname = g.get("filename")
    scale = [float(s) for s in (g.get("scale") or "1 1 1").split()]
    return (_parse_origin(v), fname, scale)


def parse_urdf(path):
    """Same shape as visualize_urdf.parse_urdf but joints carry limits too."""
    tree = ET.parse(path)
    root_elem = tree.getroot()
    if root_elem.tag != "robot":
        sys.exit(f"{path}: root tag is <{root_elem.tag}>, expected <robot>")

    links = {}
    for link_elem in root_elem.findall("link"):
        name = link_elem.get("name")
        visuals = []
        for v in link_elem.findall("visual"):
            parsed = _parse_visual(v)
            if parsed is not None:
                visuals.append(parsed)
        links[name] = visuals

    joints = {}
    for j in root_elem.findall("joint"):
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        lower, upper = _parse_limit(j)
        joints[child] = {
            "parent": parent,
            "origin": _parse_origin(j),
            "axis": _parse_axis(j),
            "name": j.get("name"),
            "type": j.get("type"),
            "lower": lower,
            "upper": upper,
        }

    children = set(joints.keys())
    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        sys.exit(f"{path}: expected exactly one root link, got {roots}")
    return {"root": roots[0], "links": links, "joints": joints}


# ---------------------------------------------------------------------------
# Forward kinematics at rest pose (all joint angles 0)
# ---------------------------------------------------------------------------


def compute_link_world(robot):
    """{link_name: 4x4 matrix in metres, rest pose}."""
    link_world = {robot["root"]: Matrix.Identity(4)}
    remaining = dict(robot["joints"])
    while remaining:
        placed = []
        for child, j in remaining.items():
            if j["parent"] in link_world:
                link_world[child] = link_world[j["parent"]] @ j["origin"]
                placed.append(child)
        if not placed:
            sys.exit(
                f"Dangling joint chain — could not place: {list(remaining.keys())}"
            )
        for c in placed:
            del remaining[c]
    return link_world


def load_foot_tip_in_link3_frame(json_path):
    """Read `Link2ToLink3Axis` (construction axis) and `Link3TipPoint`
    (construction point) from `fusion_export.json` and return the foot
    tip position in link3-local frame (metres), or `None` if either is
    missing.

    `Link2ToLink3Axis` lives in each occurrence's `axes[]` (key
    `origin_mm`); `Link3TipPoint` lives in `points[]` (key `pos_mm`).
    Both are in leg-assembly-source frame; the foot-relative-to-knee
    delta lives in that same frame, so apply `R_la = Rz(90°)` — the
    same rotation `ExportBodiesToURDF` pre-applies to joint axes — to
    convert to link3-local.
    """
    if not json_path or not Path(json_path).exists():
        return None
    try:
        data = json.loads(Path(json_path).read_text())
    except (OSError, json.JSONDecodeError) as e:
        print(f"[warn] could not read {json_path}: {e}")
        return None

    knee_source_mm = [None]
    foot_source_mm = [None]

    def _walk(node):
        for axis in node.get("axes", []) or []:
            xyz = axis.get("origin_mm")
            if (
                axis.get("name") == "Link2ToLink3Axis"
                and knee_source_mm[0] is None
                and xyz
                and len(xyz) >= 3
            ):
                knee_source_mm[0] = Vector(xyz[:3])
        for point in node.get("points", []) or []:
            xyz = point.get("pos_mm")
            if (
                point.get("name") == "Link3TipPoint"
                and foot_source_mm[0] is None
                and xyz
                and len(xyz) >= 3
            ):
                foot_source_mm[0] = Vector(xyz[:3])
        for child in node.get("children", []) or []:
            if knee_source_mm[0] is not None and foot_source_mm[0] is not None:
                return
            _walk(child)

    for occ in data.get("occurrences", []) or []:
        _walk(occ)
        if knee_source_mm[0] is not None and foot_source_mm[0] is not None:
            break

    if knee_source_mm[0] is None or foot_source_mm[0] is None:
        return None

    delta_source_mm = foot_source_mm[0] - knee_source_mm[0]
    import math as _math

    R_la = Matrix.Rotation(_math.pi / 2.0, 4, "Z")
    return (R_la @ delta_source_mm) / M_TO_MM


def matrix_m_to_mm(M):
    out = M.copy()
    out.translation = out.translation * M_TO_MM
    return out


# ---------------------------------------------------------------------------
# Scene bootstrap (same idiom as visualize_urdf.py)
# ---------------------------------------------------------------------------


def clear_scene():
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    for coll in list(bpy.data.collections):
        bpy.data.collections.remove(coll)
    for mat in list(bpy.data.materials):
        bpy.data.materials.remove(mat)
    for mesh in list(bpy.data.meshes):
        bpy.data.meshes.remove(mesh)
    for arm in list(bpy.data.armatures):
        bpy.data.armatures.remove(arm)
    _configure_units()


def _configure_units():
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 0.001  # 1 BU = 1 mm
    scene.unit_settings.length_unit = "MILLIMETERS"
    for area in bpy.context.screen.areas if bpy.context.screen else []:
        if area.type == "VIEW_3D":
            for space in area.spaces:
                if space.type == "VIEW_3D":
                    space.clip_start = 0.1
                    space.clip_end = 10000.0


def make_collections():
    scene_coll = bpy.context.scene.collection
    root = bpy.data.collections.new(ROOT_COLLECTION)
    scene_coll.children.link(root)
    children = {ROOT_COLLECTION: root}
    for name in (
        ARMATURE_COLLECTION,
        MESHES_COLLECTION,
        TARGETS_COLLECTION,
        CONTROLS_COLLECTION,
        JOINTS_COLLECTION,
        AXES_COLLECTION,
    ):
        c = bpy.data.collections.new(name)
        root.children.link(c)
        children[name] = c
    # Joint markers off by default; toggle in outliner when debugging.
    children[JOINTS_COLLECTION].hide_viewport = True
    children[AXES_COLLECTION].hide_viewport = True
    return children


def make_material(name, rgba):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    principled = next((n for n in nodes if n.type == "BSDF_PRINCIPLED"), None)
    if principled is not None:
        principled.inputs["Base Color"].default_value = rgba
    mat.diffuse_color = rgba
    return mat


# ---------------------------------------------------------------------------
# Armature construction
# ---------------------------------------------------------------------------


def _bone_endpoints_world_mm(robot, link_world, link_name, foot_tip_in_link3_m):
    """Return (head_mm, tail_mm) for the bone representing `link_name`.

    head = link's frame origin in world (where the parent joint pivot sits).
    tail = next joint's origin world (= child link's frame origin) when the
           link has exactly one child joint, else falls back to a per-link
           heuristic (`foot_tip_in_link3_m` for `*_link3`, +Y bump for
           base_link).

    `foot_tip_in_link3_m` MUST match the value used to place each leg's
    `foot_target` Empty so that `*_link3` bone tail and the foot Empty
    coincide at rest (zero IK error). Caller responsibility — main
    threads the same value through `place_ik_targets_and_constraints`.
    """
    head_m = link_world[link_name].to_translation()

    # Find a child joint (link N+1) whose parent is this link, if any.
    child_joint_origin_m = None
    for child_link, j in robot["joints"].items():
        if j["parent"] == link_name:
            child_joint_origin_m = link_world[child_link].to_translation()
            break

    if link_name.endswith("_link1"):
        # Aim link1 bone-Y at the foot tip (instead of at the hip pivot)
        # so Damped Track on link1 is identity at the URDF rest pose.
        # If bone-Y aimed at the hip (the natural child-joint origin),
        # shoulder→hip and shoulder→foot would differ by ~19° in the
        # horizontal plane (the leg has bends at hip and knee at rest),
        # and Damped Track would yaw link1 by that angle when the
        # dep-graph evaluates — visually "contorting" the rest pose by
        # ~88 mm at the foot. With the foot-tip aim, Damped Track is
        # zero at rest and only fires when the animator drags
        # foot_target away from its rest position.
        #
        # Joint axis for link1 is world +Z, so the perpendicular-to-
        # axis projection step below drops the Z component into the
        # horizontal plane. bone-local Z stays = world +Z, alignment
        # invariant unchanged. _foot_tip_for_leg applies the R-side
        # X-flip so FR/BL legs aim correctly toward their own feet.
        leg = link_name.split("_", 1)[0]
        link3_name = KNEE_BONE_FMT.format(leg=leg)
        tail_m = link_world[link3_name] @ _foot_tip_for_leg(leg, foot_tip_in_link3_m)
    elif child_joint_origin_m is not None:
        tail_m = child_joint_origin_m
    elif link_name.endswith("_link3"):
        leg = link_name.split("_", 1)[0]
        tail_m = link_world[link_name] @ _foot_tip_for_leg(leg, foot_tip_in_link3_m)
    else:
        # base_link or any leaf without a foot definition — 50 mm bump in
        # +Y world so the bone is visible.
        tail_m = head_m + Vector((0.0, 0.050, 0.0))

    # Bone-Y MUST be perpendicular to the URDF joint axis so that
    # EditBone.align_roll(axis) lands exactly on bone-local Z = axis.
    # Where the link-to-link offset has a component along the joint axis
    # (link1 has a Z lift; link3 foot tip has a Y component), project that
    # component out — the bone becomes a guide rod for rotation, not a
    # visual limb. Mesh geometry is set independently via matrix_world in
    # attach_visuals(), so the limb still looks correct.
    j = robot["joints"].get(link_name)
    if j is not None and j["axis"].length > 1e-6:
        axis_world = (link_world[link_name].to_3x3() @ j["axis"]).normalized()
        bone_dir = tail_m - head_m
        proj = bone_dir - bone_dir.dot(axis_world) * axis_world
        if proj.length > 1e-6:
            tail_m = head_m + proj

    return head_m * M_TO_MM, tail_m * M_TO_MM


def _joint_axis_world(robot, link_world, link_name):
    """The URDF joint axis for the joint that drives `link_name`, expressed
    in world frame at rest pose. None if `link_name` has no incoming joint
    (i.e., the root)."""
    j = robot["joints"].get(link_name)
    if j is None:
        return None
    # joint axis is in joint frame == child link frame at rest.
    return link_world[link_name].to_3x3() @ j["axis"]


def build_armature(robot, link_world, collection, foot_tip_in_link3_m):
    """Create the armature, bones, parenting, and roll. Returns the
    armature object. `foot_tip_in_link3_m` is forwarded to
    `_bone_endpoints_world_mm` for `*_link3` tail placement; pass the
    same value used for IK target placement so bone tail and IK target
    coincide at rest.
    """
    bpy.ops.object.armature_add(
        enter_editmode=False,
        align="WORLD",
        location=(0.0, 0.0, 0.0),
    )
    arm_obj = bpy.context.object
    arm_obj.name = ARMATURE_NAME
    arm_obj.data.name = ARMATURE_NAME + "_data"

    # Move into the armature collection.
    for c in list(arm_obj.users_collection):
        c.objects.unlink(arm_obj)
    collection.objects.link(arm_obj)

    # Iterate-until-stable so parents always exist before children.
    bpy.ops.object.mode_set(mode="EDIT")
    edit_bones = arm_obj.data.edit_bones

    # Remove the default bone armature_add gives us.
    for b in list(edit_bones):
        edit_bones.remove(b)

    placed = set()
    pending = set(robot["links"].keys())
    while pending:
        added_this_pass = []
        for link_name in list(pending):
            j = robot["joints"].get(link_name)
            if j is not None and j["parent"] not in placed:
                continue
            head_mm, tail_mm = _bone_endpoints_world_mm(
                robot, link_world, link_name, foot_tip_in_link3_m
            )
            bone = edit_bones.new(link_name)
            bone.head = head_mm
            bone.tail = tail_mm
            axis_world = _joint_axis_world(robot, link_world, link_name)
            if axis_world is not None and axis_world.length > 1e-6:
                bone.align_roll(axis_world)
            if j is not None:
                bone.parent = edit_bones[j["parent"]]
                bone.use_connect = False
            placed.add(link_name)
            added_this_pass.append(link_name)
        if not added_this_pass:
            sys.exit(f"Dangling links — could not place: {list(pending)}")
        for n in added_this_pass:
            pending.discard(n)

    bpy.ops.object.mode_set(mode="POSE")
    for pb in arm_obj.pose.bones:
        pb.rotation_mode = "XYZ"
    bpy.ops.object.mode_set(mode="OBJECT")
    bpy.context.view_layer.update()
    return arm_obj


def add_limit_rotation_constraints(robot, arm_obj):
    """`LIMIT_ROTATION` per joint bone:
    - X and Y rotation locked to 0 — forces 1-DOF around bone-local Z
      (= URDF joint axis after the bone-tail projection in
      `_bone_endpoints_world_mm`).
    - Z rotation limited to URDF `<limit lower upper>`.

    On link1 this enforces yaw-only motion under Damped Track. On
    link2/link3 it's belt-and-suspenders for the IK locks; the IK
    solver should only produce Z rotation anyway, but the constraint
    guarantees nothing slips through.

    Added LAST in the constraint stack so it clamps after IK / Damped
    Track. Owner space = LOCAL throughout (bone-local axes).
    """
    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode="POSE")
    n = 0
    for child_link, j in robot["joints"].items():
        if j["lower"] is None or j["upper"] is None:
            continue
        pb = arm_obj.pose.bones.get(child_link)
        if pb is None:
            print(f"[warn] no pose bone for joint child link {child_link!r}")
            continue
        c = pb.constraints.new("LIMIT_ROTATION")
        c.use_limit_x = True
        c.min_x = 0.0
        c.max_x = 0.0
        c.use_limit_y = True
        c.min_y = 0.0
        c.max_y = 0.0
        c.use_limit_z = True
        c.min_z = j["lower"]
        c.max_z = j["upper"]
        c.owner_space = "LOCAL"
        n += 1
    bpy.ops.object.mode_set(mode="OBJECT")
    return n


def lock_ik_axes(robot, arm_obj):
    """IK-side joint constraints on `*_link2` and `*_link3` pose bones
    (the bones in the IK chain — `chain_count=2` from the constraint on
    link3 covers link3 + link2):

    - `lock_ik_x = lock_ik_y = True` plus `ik_stiffness_x/y = 1.0`:
      restrict the IK solver to bone-local Z = URDF joint axis.
    - `use_ik_limit_z = True` with `ik_min_z`/`ik_max_z` from URDF
      `<limit lower upper>`: solver respects URDF range during the
      iterative solve, not just the post-hoc `LIMIT_ROTATION` clamp.
      This keeps the solver from oscillating against an out-of-range
      target.

    link1 is not in any IK chain so it has no IK locks; its yaw-only
    motion is enforced by Damped Track + LIMIT_ROTATION instead.
    """
    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode="POSE")
    n = 0
    for pb in arm_obj.pose.bones:
        if not (pb.name.endswith("_link2") or pb.name.endswith("_link3")):
            continue
        j = robot["joints"].get(pb.name)
        if j is None:
            continue
        pb.lock_ik_x = True
        pb.lock_ik_y = True
        pb.lock_ik_z = False
        pb.ik_stiffness_x = 1.0
        pb.ik_stiffness_y = 1.0
        pb.ik_stiffness_z = 0.0
        if j["lower"] is not None and j["upper"] is not None:
            pb.use_ik_limit_z = True
            pb.ik_min_z = j["lower"]
            pb.ik_max_z = j["upper"]
        n += 1
    bpy.ops.object.mode_set(mode="OBJECT")
    return n


# ---------------------------------------------------------------------------
# Mesh attachment
# ---------------------------------------------------------------------------


def _import_stl(stl_path, name):
    before = set(bpy.data.objects)
    try:
        bpy.ops.wm.stl_import(filepath=str(stl_path), global_scale=1.0)
    except Exception as e:
        print(f"[warn] STL import failed for {stl_path}: {e}")
        return None
    new_objs = [o for o in bpy.data.objects if o not in before]
    if not new_objs:
        print(f"[warn] STL import created no objects: {stl_path}")
        return None
    obj = new_objs[0]
    obj.name = name
    return obj


def attach_visuals(robot, link_world, arm_obj, meshes_dir, target_collection):
    """Import every <visual> STL and parent it to its link's bone via
    parent_type='BONE'. After parenting, set matrix_world to the
    rest-pose target so geometry overlays visualize_urdf.py byte-for-byte."""
    placed = 0
    for link_name, visuals in robot["links"].items():
        link_W = link_world[link_name]
        for visual_origin, mesh_rel, _scale in visuals:
            mesh_path = (meshes_dir / Path(mesh_rel).name).resolve()
            if not mesh_path.exists():
                mesh_path = Path(mesh_rel)
                if not mesh_path.exists():
                    print(f"[warn] mesh missing: {mesh_rel}")
                    continue
            obj_name = f"{link_name}__{Path(mesh_rel).stem}"
            obj = _import_stl(mesh_path, obj_name)
            if obj is None:
                continue
            for c in list(obj.users_collection):
                c.objects.unlink(obj)
            target_collection.objects.link(obj)
            obj.parent = arm_obj
            obj.parent_type = "BONE"
            obj.parent_bone = link_name
            obj.matrix_parent_inverse = Matrix.Identity(4)
            bpy.context.view_layer.update()
            obj.matrix_world = matrix_m_to_mm(link_W @ visual_origin)
            placed += 1
    return placed


# ---------------------------------------------------------------------------
# IK targets + IK + Damped Track
# ---------------------------------------------------------------------------


def _add_empty(name, location_mm, target_collection, kind="SPHERE", size_mm=8.0):
    bpy.ops.object.empty_add(type=kind, radius=size_mm, location=tuple(location_mm))
    obj = bpy.context.object
    obj.name = name
    for c in list(obj.users_collection):
        c.objects.unlink(obj)
    target_collection.objects.link(obj)
    return obj


def place_ik_targets_and_constraints(
    robot, link_world, arm_obj, collection, foot_tip_in_link3_m
):
    """Per leg, set up the two-empty foot target + IK + Damped Track:

    `foot_target_{leg}` (visible, blue ball) — placed at the actual CAD
    foot tip in world. Per-leg foot tip from `_foot_tip_for_leg(leg, …)`
    so R-side legs (FR, BL) get the X-flipped value to match their
    mirrored leg_lower.stl visual. Animator drags this.

    `foot_ik_{leg}` (hidden) — `COPY_LOCATION` from `foot_target` with
    `use_offset=False` and zero stored offset, so foot_ik tracks
    foot_target exactly. Kept as a separate Empty for clarity (animator
    handle vs IK target are decoupled — easy to retarget later).

    Known limitation: bone tail is the projected foot tip (= drop the
    joint-axis component of `foot_tip_in_link3_m`), which sits ~13 mm
    away from the actual CAD foot tip along the joint axis (the
    component the projection removed, ≈ link3-frame Y). foot_ik sits
    at the actual CAD foot tip, so at rest IK has a constant ~13 mm
    residual it cannot close — the gap is along the joint axis,
    perpendicular to every plane the IK can rotate the foot through.
    Cosmetic; the visible foot mesh tip remains at foot_target's
    location to within numerical tolerance.

    IK on `*_link3` pose bone: `target=foot_ik`, `chain_count=2`,
    `use_tail=True`, `use_rotation=False`, `use_stretch=False`.

    Damped Track on `*_link1` pose bone: `target=foot_target`,
    `track_axis='TRACK_Y'`. With LIMIT_ROTATION's X/Y locks added
    afterwards, this becomes yaw-only — link1 rotates around
    bone-local Z (= world +Z) so its bone-Y horizontal projection
    aims at the foot. `TRACK_Y` works for all four legs because each
    link1's bone-Y after the tail projection points outward from the
    chassis (toward its own foot direction) at rest, regardless of
    L/R-side mesh mirroring.
    """
    bpy.ops.object.mode_set(mode="OBJECT")
    n_targets = 0
    n_ik = 0
    n_track = 0

    for leg in LEG_IDS:
        link3 = KNEE_BONE_FMT.format(leg=leg)
        link1 = SHOULDER_BONE_FMT.format(leg=leg)
        if link3 not in link_world:
            continue

        foot_tip_per_leg = _foot_tip_for_leg(leg, foot_tip_in_link3_m)
        actual_foot_world_m = link_world[link3] @ foot_tip_per_leg
        actual_foot_world_mm = actual_foot_world_m * M_TO_MM

        # Visible foot target at actual CAD foot tip.
        foot_target = _add_empty(
            FOOT_TARGET_FMT.format(leg=leg),
            actual_foot_world_mm,
            collection,
            kind="SPHERE",
            size_mm=8.0,
        )
        n_targets += 1

        # Hidden foot_ik that exactly tracks foot_target.
        foot_ik = _add_empty(
            FOOT_IK_FMT.format(leg=leg),
            actual_foot_world_mm,  # initial position; constraint will override
            collection,
            kind="PLAIN_AXES",
            size_mm=4.0,
        )
        foot_ik.hide_viewport = True
        cop = foot_ik.constraints.new("COPY_LOCATION")
        cop.target = foot_target
        cop.target_space = "WORLD"
        cop.owner_space = "WORLD"
        cop.use_offset = False

        # IK on link3, Damped Track on link1.
        bpy.context.view_layer.objects.active = arm_obj
        bpy.ops.object.mode_set(mode="POSE")
        knee_pb = arm_obj.pose.bones.get(link3)
        if knee_pb is not None:
            ik = knee_pb.constraints.new("IK")
            ik.target = foot_ik
            ik.chain_count = 2
            ik.use_rotation = False
            ik.use_tail = True
            ik.use_stretch = False
            n_ik += 1
        shoulder_pb = arm_obj.pose.bones.get(link1)
        if shoulder_pb is not None:
            tk = shoulder_pb.constraints.new("DAMPED_TRACK")
            tk.target = foot_target
            tk.track_axis = "TRACK_Y"
            n_track += 1
        bpy.ops.object.mode_set(mode="OBJECT")

    return n_targets, n_ik, n_track


# ---------------------------------------------------------------------------
# Body control (single Empty drives the whole rig)
# ---------------------------------------------------------------------------


def add_body_control(arm_obj, controls_collection):
    """Single `body_ctrl` Empty (CUBE, 80 mm) at world origin. Reparents
    the armature OBJECT to body_ctrl so moving body_ctrl translates and
    rotates the whole rig. Foot Empties are unparented (world space),
    so as body_ctrl moves, IK on link3 + Damped Track on link1 keep the
    legs reaching back to the planted feet.

    The armature object is hidden afterward — animator's primary handle
    becomes the visible cube. Bones still evaluate (hide_viewport on
    the object hides display, not constraint evaluation), and the
    armature can be unhidden temporarily for per-bone pose-mode work.

    User spec wording was "parented to base_link bone"; in practice the
    parenting is inverted (armature parent = body_ctrl) so that moving
    body_ctrl drives the rig rather than following it. body_ctrl sits
    at base_link's rest position (world origin), satisfying the spec's
    intent of a chassis-anchored handle.
    """
    bpy.ops.object.empty_add(type="CUBE", radius=80.0, location=(0.0, 0.0, 0.0))
    body_ctrl = bpy.context.object
    body_ctrl.name = "body_ctrl"
    for c in list(body_ctrl.users_collection):
        c.objects.unlink(body_ctrl)
    controls_collection.objects.link(body_ctrl)

    arm_obj.parent = body_ctrl
    arm_obj.parent_type = "OBJECT"
    arm_obj.matrix_parent_inverse = Matrix.Identity(4)
    arm_obj.hide_viewport = True
    return body_ctrl


def _diagnose_foot_target_placement(robot, link_world, arm_obj):
    """Print, per leg, the actual mesh foot tip in world (max-+Y
    centroid of `leg_lower` STL) versus `foot_target` Empty's world
    position — both at rest pose.

    Uses URDF rest matrices (`link_world[link3] @ visual_origin`) to
    compute the rest-pose mesh world; Blender's `mesh_obj.matrix_world`
    reflects the LIVE pose after Damped Track + IK fire on the
    placeholder rest, which is misleading for placement diagnosis.

    Expected: mesh tip and foot_target agree to within sub-mm. If they
    don't, the JSON `Link3TipAxis` was placed at a different reference
    in CAD than the mesh max-+Y centroid (which the URDF metadata
    comment `FOOT_TIP_FALLBACK_IN_LINK3_FRAME_M` captures). Fix is
    either to move the construction axis in CAD, or switch the rigger
    to use the fallback constant.

    NOTE: this diagnostic does NOT measure the ~13 mm bone-tail-vs-
    foot_target gap (projection that preserves align_roll exactness) —
    that's rig-internal and documented in
    `place_ik_targets_and_constraints`.
    """
    # Damped Track rest-identity check: per leg, compare each link1's
    # bone-Y (in armature space, from matrix_local) to the direction
    # from link1's head to foot_target. If they align at rest, Damped
    # Track applies zero rotation at the URDF rest pose; only when the
    # animator drags foot_target does it rotate link1 — preserving
    # auto-yaw on movement while eliminating the 88 mm rest contortion.
    print("[urdf_to_blender_rigged] Damped Track rest-identity check (link1):")
    import math as _math2

    for leg in LEG_IDS:
        link1_name = SHOULDER_BONE_FMT.format(leg=leg)
        target = bpy.data.objects.get(FOOT_TARGET_FMT.format(leg=leg))
        bone = arm_obj.data.bones.get(link1_name)
        if bone is None or target is None:
            continue
        ml = bone.matrix_local
        bone_y = Vector(ml.col[1][:3]).normalized()
        head_world = Vector(ml.col[3][:3])
        target_world = target.matrix_world.translation
        dir_to_target = (target_world - head_world).normalized()
        # Full 3D angle includes elevation; LIMIT_ROTATION X/Y locks
        # block elevation rotation, so it doesn't matter for rest pose.
        cos_3d = max(-1.0, min(1.0, bone_y.dot(dir_to_target)))
        angle_3d_deg = _math2.degrees(_math2.acos(cos_3d))
        # Yaw-only angle (horizontal projection) IS what Damped Track
        # can actually apply through the X/Y locks. ~0° at rest means
        # no rest-pose contortion; auto-yaw still fires when the
        # animator drags foot_target horizontally.
        bone_y_h = Vector((bone_y.x, bone_y.y, 0.0))
        target_h = Vector((dir_to_target.x, dir_to_target.y, 0.0))
        if bone_y_h.length > 1e-9 and target_h.length > 1e-9:
            cos_yaw = max(
                -1.0,
                min(1.0, bone_y_h.normalized().dot(target_h.normalized())),
            )
            angle_yaw_deg = _math2.degrees(_math2.acos(cos_yaw))
        else:
            angle_yaw_deg = float("nan")
        print(
            f"  [{leg}_link1] 3D={angle_3d_deg:6.3f}° "
            f"yaw-only={angle_yaw_deg:6.3f}° "
            f"(yaw-only is what passes through LIMIT_ROTATION's X/Y locks; "
            f"~0° = no rest contortion)"
        )

    # Spot-check the FL Empties before iterating: parent (should be None),
    # location vs matrix_world.translation (should match — foot_target is
    # unparented and unmoved), COPY_LOCATION settings on foot_ik.
    fl_target = bpy.data.objects.get(FOOT_TARGET_FMT.format(leg="fl"))
    fl_ik = bpy.data.objects.get(FOOT_IK_FMT.format(leg="fl"))
    if fl_target is not None:
        loc = fl_target.location
        mw = fl_target.matrix_world.translation
        print(
            f"[urdf_to_blender_rigged] foot_target_fl: "
            f"location={tuple(round(v, 3) for v in loc)} mm, "
            f"matrix_world.translation={tuple(round(v, 3) for v in mw)} mm, "
            f"parent={fl_target.parent.name if fl_target.parent else None}"
        )
    if fl_ik is not None:
        cop = next((c for c in fl_ik.constraints if c.type == "COPY_LOCATION"), None)
        if cop is not None:
            print(
                f"[urdf_to_blender_rigged] foot_ik_fl: "
                f"COPY_LOCATION target={cop.target.name if cop.target else None} "
                f"target_space={cop.target_space} owner_space={cop.owner_space} "
                f"use_offset={cop.use_offset}"
            )

    print("[urdf_to_blender_rigged] foot_target vs mesh tip (rest pose):")
    for leg in LEG_IDS:
        link3 = KNEE_BONE_FMT.format(leg=leg)
        mesh_obj = bpy.data.objects.get(f"{leg}_link3__leg_lower")
        target_obj = bpy.data.objects.get(FOOT_TARGET_FMT.format(leg=leg))
        if mesh_obj is None or target_obj is None:
            print(
                f"  [{leg}] missing object — "
                f"mesh={mesh_obj is not None}, target={target_obj is not None}"
            )
            continue
        verts = mesh_obj.data.vertices
        if not verts:
            print(f"  [{leg}] empty mesh data")
            continue
        max_y = max(v.co.y for v in verts)
        tol_y = 0.01  # mesh-local mm
        tip_verts = [v.co for v in verts if v.co.y >= max_y - tol_y]
        n = len(tip_verts)
        centroid_local = sum(tip_verts, Vector((0.0, 0.0, 0.0))) / n

        # Rest-pose mesh world matrix from URDF: link_world[link3] @
        # visual_origin (for the leg_lower visual specifically).
        visual_origin = Matrix.Identity(4)
        for vis_origin, mesh_rel, _scale in robot["links"][link3]:
            if Path(mesh_rel).stem == "leg_lower":
                visual_origin = vis_origin
                break
        mesh_rest_world_m = link_world[link3] @ visual_origin
        mesh_rest_world_mm = matrix_m_to_mm(mesh_rest_world_m)
        centroid_world = mesh_rest_world_mm @ centroid_local

        target_world = target_obj.matrix_world.translation
        delta_mm = (target_world - centroid_world).length
        print(
            f"  [{leg}] mesh tip = "
            f"({centroid_world.x:+8.2f}, {centroid_world.y:+8.2f}, {centroid_world.z:+7.2f}) mm "
            f"({n} verts at max Y={max_y:.3f}); "
            f"foot_target = ({target_world.x:+8.2f}, {target_world.y:+8.2f}, {target_world.z:+7.2f}) mm; "
            f"|delta| = {delta_mm:6.3f} mm"
        )


# ---------------------------------------------------------------------------
# Joint markers (parented to bones; hidden by default)
# ---------------------------------------------------------------------------


def _add_marker(
    world_pos_mm, name, material, target_collection, radius_mm=SPHERE_RADIUS_MM
):
    before = set(bpy.data.objects)
    bpy.ops.mesh.primitive_uv_sphere_add(
        radius=radius_mm,
        location=(world_pos_mm[0], world_pos_mm[1], world_pos_mm[2]),
        segments=16,
        ring_count=8,
    )
    new_objs = [o for o in bpy.data.objects if o not in before]
    if not new_objs:
        return None
    obj = new_objs[0]
    obj.name = name
    for c in list(obj.users_collection):
        c.objects.unlink(obj)
    target_collection.objects.link(obj)
    if obj.data.materials:
        obj.data.materials[0] = material
    else:
        obj.data.materials.append(material)
    return obj


def place_joint_markers(
    robot, link_world, arm_obj, mat_joint, mat_axis, joints_collection, axes_collection
):
    """Red sphere at each joint pivot, orange sphere along the axis tip.
    Both parented to the joint's child bone with parent_type='BONE' so
    they follow pose-mode rotations."""
    n_joints = 0
    n_axes = 0
    bpy.ops.object.mode_set(mode="OBJECT")
    for child_link, j in robot["joints"].items():
        child_W = link_world[child_link]
        pivot_mm = child_W.translation * M_TO_MM
        pivot = _add_marker(
            pivot_mm, f"{j['name']}__pivot", mat_joint, joints_collection
        )
        if pivot is not None:
            pivot.parent = arm_obj
            pivot.parent_type = "BONE"
            pivot.parent_bone = child_link
            pivot.matrix_parent_inverse = Matrix.Identity(4)
            bpy.context.view_layer.update()
            pivot.matrix_world = Matrix.Translation(pivot_mm)
            n_joints += 1

        axis_world = child_W.to_3x3() @ j["axis"]
        tip_mm = pivot_mm + axis_world * AXIS_LENGTH_MM
        tip = _add_marker(
            tip_mm, f"{j['name']}__axis_tip", mat_axis, axes_collection, radius_mm=2.0
        )
        if tip is not None:
            tip.parent = arm_obj
            tip.parent_type = "BONE"
            tip.parent_bone = child_link
            tip.matrix_parent_inverse = Matrix.Identity(4)
            bpy.context.view_layer.update()
            tip.matrix_world = Matrix.Translation(tip_mm)
            n_axes += 1
    return n_joints, n_axes


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def _script_dir():
    try:
        return Path(__file__).resolve().parent
    except NameError:
        return Path.cwd()


def parse_args():
    default_urdf = (
        _script_dir()
        / ".."
        / ".."
        / "code"
        / "simulation"
        / "generated"
        / "facehugger.urdf"
    )
    default_meshes = (
        _script_dir()
        / ".."
        / ".."
        / "code"
        / "simulation"
        / "generated"
        / "exported_meshes"
    )
    default_json = (
        _script_dir()
        / ".."
        / ".."
        / "code"
        / "simulation"
        / "generated"
        / "fusion_export.json"
    )

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=default_urdf.resolve())
    parser.add_argument("--meshes", type=Path, default=default_meshes.resolve())
    parser.add_argument(
        "--json",
        type=Path,
        default=default_json.resolve(),
        help="fusion_export.json path. Used to cross-check the foot tip "
        "from the Link3TipPoint construction point against "
        "FOOT_TIP_FALLBACK_IN_LINK3_FRAME_M (URDF metadata, mesh-derived). "
        "Warns if they disagree by more than 5 mm.",
    )
    parser.add_argument("--save", type=Path, default=None)

    argv = sys.argv
    argv = argv[argv.index("--") + 1 :] if "--" in argv else []
    return parser.parse_args(argv)


def main():
    args = parse_args()
    print(f"[urdf_to_blender_rigged] urdf   : {args.urdf}")
    print(f"[urdf_to_blender_rigged] meshes : {args.meshes}")
    print(f"[urdf_to_blender_rigged] json   : {args.json}")

    if not args.urdf.exists():
        raise SystemExit(f"URDF not found: {args.urdf}")

    robot = parse_urdf(args.urdf)
    print(
        f"[urdf_to_blender_rigged] parsed — {len(robot['links'])} links, "
        f"{len(robot['joints'])} joints, root={robot['root']}"
    )
    link_world = compute_link_world(robot)

    # Prefer the URDF metadata (mesh max-+Y centroid) over the JSON
    # Link3TipAxis. The JSON read remains available for diagnostic
    # comparison: if the values disagree by more than the tolerance,
    # warn — that means the CAD construction axis is misplaced
    # relative to the leg_lower.stl tip.
    foot_tip_in_link3_m = FOOT_TIP_FALLBACK_IN_LINK3_FRAME_M
    foot_tip_from_json = load_foot_tip_in_link3_frame(args.json)
    if foot_tip_from_json is not None:
        delta_mm = (foot_tip_in_link3_m - foot_tip_from_json).length * M_TO_MM
        if delta_mm > 5.0:
            print(
                f"[urdf_to_blender_rigged] WARNING: JSON Link3TipPoint "
                f"{tuple(round(v * M_TO_MM, 3) for v in foot_tip_from_json)} mm "
                f"differs from FOOT_TIP_FALLBACK "
                f"{tuple(round(v * M_TO_MM, 3) for v in foot_tip_in_link3_m)} mm "
                f"by {delta_mm:.2f} mm — move the construction point in CAD to "
                f"match the leg_lower.stl max-+Y centroid. Using FOOT_TIP_FALLBACK."
            )
        else:
            print(
                f"[urdf_to_blender_rigged] foot tip: JSON Link3TipPoint matches "
                f"FOOT_TIP_FALLBACK to {delta_mm:.3f} mm — using metadata."
            )
    print(
        f"[urdf_to_blender_rigged] foot tip: link3-local "
        f"{tuple(round(v * M_TO_MM, 3) for v in foot_tip_in_link3_m)} mm "
        "(FOOT_TIP_FALLBACK = URDF metadata, mesh-derived)"
    )

    clear_scene()
    collections = make_collections()
    mat_joint = make_material("JointPivotRed", COLOR_JOINT)
    mat_axis = make_material("JointAxisOrange", COLOR_AXIS)

    arm_obj = build_armature(
        robot, link_world, collections[ARMATURE_COLLECTION], foot_tip_in_link3_m
    )
    n_bones = len(arm_obj.data.bones)

    n_meshes = attach_visuals(
        robot, link_world, arm_obj, args.meshes, collections[MESHES_COLLECTION]
    )
    # Constraint stack order: IK / Damped Track first, LIMIT_ROTATION
    # last (so it clamps the IK-or-track-solved pose, not the other way).
    # New constraints append to the stack, so call order = stack order.
    n_targets, n_ik, n_track = place_ik_targets_and_constraints(
        robot,
        link_world,
        arm_obj,
        collections[TARGETS_COLLECTION],
        foot_tip_in_link3_m,
    )
    n_lim = add_limit_rotation_constraints(robot, arm_obj)
    n_locked = lock_ik_axes(robot, arm_obj)
    n_joints, n_axes = place_joint_markers(
        robot,
        link_world,
        arm_obj,
        mat_joint,
        mat_axis,
        collections[JOINTS_COLLECTION],
        collections[AXES_COLLECTION],
    )

    _diagnose_foot_target_placement(robot, link_world, arm_obj)
    _verify_auto_yaw(arm_obj)
    body_ctrl = add_body_control(arm_obj, collections[CONTROLS_COLLECTION])

    max_err_deg = _check_bone_z_alignment(robot, link_world, arm_obj)
    print(
        f"[urdf_to_blender_rigged] armature: {n_bones} bones, "
        f"{n_lim} LIMIT_ROTATION (X/Y locked, Z = URDF range), "
        f"{n_ik} IK (chain=2, no stretch), {n_track} Damped Track, "
        f"{n_locked} bones with IK X/Y locked"
    )
    print(
        f"[urdf_to_blender_rigged] targets : {n_targets} foot_target "
        f"+ {n_targets} foot_ik (hidden, COPY_LOCATION exact)"
    )
    print(
        f"[urdf_to_blender_rigged] controls: {body_ctrl.name} "
        f"({body_ctrl.empty_display_type}, armature parented to it, "
        f"armature.hide_viewport=True)"
    )
    print(
        f"[urdf_to_blender_rigged] geometry: {n_meshes} visuals, "
        f"{n_joints} joint pivots, {n_axes} axis tips (markers hidden)"
    )
    print(
        f"[urdf_to_blender_rigged] check   : "
        f"max bone-Z vs joint-axis error = {max_err_deg:.4f}° "
        f"(invariant should survive IK addition)"
    )

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=str(args.save))
        print(f"[urdf_to_blender_rigged] saved: {args.save}")


def _verify_auto_yaw(arm_obj):
    """Dynamic check: translate foot_target_fl by +50 mm in world X,
    eval dep graph, measure how much fl_link1's resolved bone-Y
    direction rotated in the world XY plane (yaw). Constraint-resolved
    pose lives in `pose_bone.matrix`, not `rotation_euler` (which is the
    pre-constraint keyed value). A non-zero yaw delta confirms Damped
    Track is firing on foot_target movement. Restores foot_target
    afterward so the saved blend keeps the rest pose.
    """
    import math as _math3

    target = bpy.data.objects.get(FOOT_TARGET_FMT.format(leg="fl"))
    pb = arm_obj.pose.bones.get(SHOULDER_BONE_FMT.format(leg="fl"))
    if target is None or pb is None:
        return

    def _yaw_from_matrix():
        bpy.context.view_layer.update()
        m = pb.matrix
        return _math3.atan2(m.col[1].y, m.col[1].x)

    saved = target.location.copy()
    yaw_before = _yaw_from_matrix()
    target.location = saved + Vector((50.0, 0.0, 0.0))
    yaw_after = _yaw_from_matrix()
    target.location = saved
    bpy.context.view_layer.update()

    delta_deg = _math3.degrees(yaw_after - yaw_before)
    print(
        f"[urdf_to_blender_rigged] auto-yaw : foot_target_fl +50 mm in X → "
        f"fl_link1 yaw changed by {delta_deg:+.3f}° "
        f"(non-zero = Damped Track firing through LIMIT_ROTATION X/Y locks)"
    )


def _check_bone_z_alignment(robot, link_world, arm_obj):
    """Return the worst-case angle (degrees) between each bone's local Z
    (armature-space, read from `bone.matrix_local`) and the URDF joint
    axis it should equal. A clean rig prints < 1e-3°.

    NOTE: `bone.y_axis` / `bone.z_axis` are in parent-bone-local space,
    not armature space — use `bone.matrix_local.col[1]` (Y) and
    `.col[2]` (Z) for armature-space directions.
    """
    import math as _math

    worst = 0.0
    worst_bone = None
    per_bone = []
    for child_link, j in robot["joints"].items():
        bone = arm_obj.data.bones.get(child_link)
        if bone is None:
            continue
        axis_world = (link_world[child_link].to_3x3() @ j["axis"]).normalized()
        ml = bone.matrix_local
        z_world = Vector(ml.col[2][:3]).normalized()
        cos_t = max(-1.0, min(1.0, abs(z_world.dot(axis_world))))
        err_deg = _math.degrees(_math.acos(cos_t))
        per_bone.append((child_link, err_deg))
        if err_deg > worst:
            worst = err_deg
            worst_bone = child_link
    print(
        "[urdf_to_blender_rigged] bone-Z alignment per bone: "
        + ", ".join(f"{n}={e:.4f}°" for n, e in per_bone)
    )
    return worst


if __name__ == "__main__":
    main()
