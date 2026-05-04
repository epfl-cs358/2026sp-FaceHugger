"""
urdf_to_blender_rigged.py — Armature-based URDF importer with IK + joint limits

Sibling of `visualize_urdf.py`. Same URDF parsing + scene bootstrap, but
builds a real Blender Armature so the rig can be posed/animated. The
animator-facing import path (placement-only `visualize_urdf.py` stays as
the cross-check against PyBullet).

Architecture (per RIGGING_PLAN.md, Option B — Armature):
  1. Parse URDF (parse_urdf, same as visualize_urdf).
  2. Compute link world matrices at rest pose (all joint angles 0°).
  3. Build a single Armature in edit mode:
       - One bone per URDF link (12 leg bones + base_link), named to
         match the URDF link names exactly (`fl_link1`, `fr_link2`, …).
       - Bone head = parent joint pivot world; tail = next joint pivot
         (or for `*_link3`, the foot tip in link3 frame, projected to
         world).
       - Bone roll = `EditBone.align_roll(joint_axis_world)`. After
         roll alignment, **bone-local Z = the URDF joint axis** for all
         12 joints uniformly. The per-side ±Y sign on hip/knee falls
         out automatically.
       - Parent in chain via `EditBone.parent`, `use_connect=False` (so
         the parent's tail need not coincide with the child's head — for
         our robot the head sits at the joint pivot, which is offset from
         the parent's tail by the joint origin).
  4. Pose mode: `LIMIT_ROTATION` on every joint bone (use_limit_z =
     True, min_z/max_z from URDF `<limit lower upper>`).
  5. Attach `<visual>` STLs via `parent_type='BONE'` to their link's
     bone. After parenting, set `obj.matrix_world = link_world @
     visual_origin` so rest-pose geometry matches `visualize_urdf.py`
     byte-for-byte.
  6. Per leg: place a `{leg}_foot_target` Empty at the rest-pose foot
     tip, add an IK constraint to the knee bone targeting that empty.
     `chain_count` is set by `--ik-chain` (default 2 = hip+knee solve;
     experimental 3 also includes shoulder).
  7. Joint-pivot/axis markers placed but `hide_viewport=True` by default
     (toggle in outliner for debugging).

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
    --save PATH      write a .blend here after building the scene
    --ik-chain N     IK chain length per leg (2 default = hip+knee solve;
                     3 = shoulder+hip+knee solve, experimental)

Targets Blender 5.x.
"""

import argparse
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
COLOR_TARGET = (0.0, 0.6, 1.0, 1.0)  # blue

ARMATURE_NAME = "FaceHuggerRig"

# Foot tip in link3 frame, in mm. Documented in the URDF comment block
# (leg_lower.stl max +Y centroid). Same for all 4 legs by symmetry. If
# the CAD changes such that this drifts, update this constant — the
# URDF generator emits it as a comment in facehugger.urdf for reference.
FOOT_TIP_IN_LINK3_FRAME_M = Vector((-0.030946, -0.013000, 0.000024))

# Naming patterns
FOOT_TARGET_FMT = "{leg}_foot_target"
LEG_IDS = ("fl", "fr", "bl", "br")
KNEE_BONE_FMT = "{leg}_link3"


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


def _bone_endpoints_world_mm(robot, link_world, link_name):
    """Return (head_mm, tail_mm) for the bone representing `link_name`.

    head = link's frame origin in world (where the parent joint pivot sits).
    tail = next joint's origin world (= child link's frame origin) when the
           link has exactly one child joint, else falls back to a per-link
           heuristic (foot tip for *_link3, +Y bump for base_link).
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
        # Foot tip in link3 frame → world.
        tail_m = link_world[link_name] @ FOOT_TIP_IN_LINK3_FRAME_M
    else:
        # Fallback for base_link or any leaf link without a foot definition.
        # 50 mm bump in +Y world so the bone is visible.
        tail_m = head_m + Vector((0.0, 0.050, 0.0))

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


def build_armature(robot, link_world, collection):
    """Create the armature, bones, parenting, and roll. Returns the
    armature object."""
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
            head_mm, tail_mm = _bone_endpoints_world_mm(robot, link_world, link_name)
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
    """For each joint with finite (lower, upper), add a LIMIT_ROTATION
    constraint to the child link's pose bone. After align_roll, the joint
    axis is bone-local Z, so use_limit_z + min_z/max_z is uniform."""
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
        c.use_limit_z = True
        c.min_z = j["lower"]
        c.max_z = j["upper"]
        c.owner_space = "LOCAL"
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
# Foot targets + IK
# ---------------------------------------------------------------------------


def _add_empty(
    name, location_mm, target_collection, color_mat=None, kind="SPHERE", size_mm=8.0
):
    bpy.ops.object.empty_add(type=kind, radius=size_mm, location=tuple(location_mm))
    obj = bpy.context.object
    obj.name = name
    for c in list(obj.users_collection):
        c.objects.unlink(obj)
    target_collection.objects.link(obj)
    return obj


def place_foot_targets_and_ik(robot, link_world, arm_obj, ik_chain, collection):
    """One Empty per leg at the rest-pose foot tip. IK on each *_link3
    pose bone targeting that empty. chain_count = ik_chain (2 default)."""
    bpy.ops.object.mode_set(mode="OBJECT")
    n_targets = 0
    n_ik = 0
    for leg in LEG_IDS:
        link3 = f"{leg}_link3"
        if link3 not in link_world:
            continue
        foot_world_m = link_world[link3] @ FOOT_TIP_IN_LINK3_FRAME_M
        foot_world_mm = foot_world_m * M_TO_MM
        empty = _add_empty(
            FOOT_TARGET_FMT.format(leg=leg),
            foot_world_mm,
            collection,
            kind="SPHERE",
            size_mm=8.0,
        )
        n_targets += 1

        bpy.context.view_layer.objects.active = arm_obj
        bpy.ops.object.mode_set(mode="POSE")
        knee_pb = arm_obj.pose.bones.get(KNEE_BONE_FMT.format(leg=leg))
        if knee_pb is None:
            bpy.ops.object.mode_set(mode="OBJECT")
            continue
        ik = knee_pb.constraints.new("IK")
        ik.target = empty
        ik.chain_count = ik_chain
        ik.use_rotation = False
        ik.use_tail = True
        bpy.ops.object.mode_set(mode="OBJECT")
        n_ik += 1
    return n_targets, n_ik


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

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=default_urdf.resolve())
    parser.add_argument("--meshes", type=Path, default=default_meshes.resolve())
    parser.add_argument("--save", type=Path, default=None)
    parser.add_argument(
        "--ik-chain",
        type=int,
        choices=(2, 3),
        default=2,
        help="IK chain length per leg. 2 (default) = hip+knee solve, "
        "shoulder stays FK. 3 = shoulder+hip+knee all solve "
        "(experimental).",
    )

    argv = sys.argv
    argv = argv[argv.index("--") + 1 :] if "--" in argv else []
    return parser.parse_args(argv)


def main():
    args = parse_args()
    print(f"[urdf_to_blender_rigged] urdf      : {args.urdf}")
    print(f"[urdf_to_blender_rigged] meshes    : {args.meshes}")
    print(f"[urdf_to_blender_rigged] ik_chain  : {args.ik_chain}")

    if not args.urdf.exists():
        raise SystemExit(f"URDF not found: {args.urdf}")

    robot = parse_urdf(args.urdf)
    print(
        f"[urdf_to_blender_rigged] parsed — {len(robot['links'])} links, "
        f"{len(robot['joints'])} joints, root={robot['root']}"
    )
    link_world = compute_link_world(robot)

    clear_scene()
    collections = make_collections()
    mat_joint = make_material("JointPivotRed", COLOR_JOINT)
    mat_axis = make_material("JointAxisOrange", COLOR_AXIS)

    arm_obj = build_armature(robot, link_world, collections[ARMATURE_COLLECTION])
    n_bones = len(arm_obj.data.bones)

    n_meshes = attach_visuals(
        robot, link_world, arm_obj, args.meshes, collections[MESHES_COLLECTION]
    )
    # IK BEFORE LIMIT_ROTATION: the LIMIT_ROTATION constraint must sit
    # below IK in the stack so it clamps the IK-solved pose. Adding it
    # second guarantees the order without bpy.ops.constraint.move_up.
    n_targets, n_ik = place_foot_targets_and_ik(
        robot,
        link_world,
        arm_obj,
        args.ik_chain,
        collections[TARGETS_COLLECTION],
    )
    n_lim = add_limit_rotation_constraints(robot, arm_obj)
    n_joints, n_axes = place_joint_markers(
        robot,
        link_world,
        arm_obj,
        mat_joint,
        mat_axis,
        collections[JOINTS_COLLECTION],
        collections[AXES_COLLECTION],
    )

    print(
        f"[urdf_to_blender_rigged] armature: {n_bones} bones, "
        f"{n_lim} LIMIT_ROTATION, {n_ik} IK (chain={args.ik_chain}), "
        f"{n_targets} foot targets"
    )
    print(
        f"[urdf_to_blender_rigged] geometry: {n_meshes} visuals, "
        f"{n_joints} joint pivots, {n_axes} axis tips (markers hidden)"
    )

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=str(args.save))
        print(f"[urdf_to_blender_rigged] saved: {args.save}")


if __name__ == "__main__":
    main()
