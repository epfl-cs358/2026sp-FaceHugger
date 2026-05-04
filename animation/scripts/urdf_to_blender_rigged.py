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
    --json PATH      fusion_export.json (foot tip from `Link3TipAxis`)
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

COLOR_JOINT = (1.0, 0.0, 0.0, 1.0)  # red
COLOR_AXIS = (1.0, 0.4, 0.0, 1.0)  # orange

ARMATURE_NAME = "FaceHuggerRig"

# Fallback foot tip in link3 frame (mm) — used if `Link3TipAxis` is
# missing from `fusion_export.json`. Same value as the URDF metadata
# comment block. The CAD defines the foot tip via a `Link3TipAxis`
# construction axis (whitelisted in
# ExportBodiesToURDF.CONSTRUCTION_AXES); the importer prefers that
# JSON value when available.
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
    """Read `Link2ToLink3Axis` and `Link3TipAxis` from `fusion_export.json`
    and return the foot tip position in link3-local frame (metres), or
    `None` if either axis is missing.

    The JSON stores ONE construction axis instance per name (from the
    leg-assembly source occurrence), so we use `origin_mm` (in
    leg-assembly-source frame) rather than `origin_world_mm` (which
    reflects the CAD construction pose, not the URDF rest pose). The
    foot-relative-to-knee delta lives in the source frame; apply
    `R_la = Rz(90°)` — the same rotation `ExportBodiesToURDF` pre-applies
    to joint axes — to convert to link3-local.
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
            name = axis.get("name")
            xyz = axis.get("origin_mm")
            if xyz and len(xyz) >= 3:
                if name == "Link2ToLink3Axis" and knee_source_mm[0] is None:
                    knee_source_mm[0] = Vector(xyz[:3])
                elif name == "Link3TipAxis" and foot_source_mm[0] is None:
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

    if child_joint_origin_m is not None:
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
        help="fusion_export.json path. Used to read each leg's "
        "Link3TipAxis origin for foot-target placement; falls back to "
        "FOOT_TIP_FALLBACK_IN_LINK3_FRAME_M if missing.",
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

    foot_tip_in_link3_m = load_foot_tip_in_link3_frame(args.json)
    if foot_tip_in_link3_m is not None:
        src = f"Link3TipAxis in {args.json.name}"
    else:
        foot_tip_in_link3_m = FOOT_TIP_FALLBACK_IN_LINK3_FRAME_M
        src = "hardcoded fallback (Link3TipAxis missing — re-export Fusion)"
    print(
        f"[urdf_to_blender_rigged] foot tip: link3-local "
        f"{tuple(round(v * M_TO_MM, 3) for v in foot_tip_in_link3_m)} mm ({src})"
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
    for child_link, j in robot["joints"].items():
        bone = arm_obj.data.bones.get(child_link)
        if bone is None:
            continue
        axis_world = (link_world[child_link].to_3x3() @ j["axis"]).normalized()
        ml = bone.matrix_local
        y_world = Vector(ml.col[1][:3]).normalized()
        z_world = Vector(ml.col[2][:3]).normalized()
        cos_t = max(-1.0, min(1.0, abs(z_world.dot(axis_world))))
        err_deg = _math.degrees(_math.acos(cos_t))
        worst = max(worst, err_deg)
    return worst


if __name__ == "__main__":
    main()
