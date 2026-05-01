"""
visualize_urdf.py — Blender debug scene builder driven by the URDF (no rigging)

Sibling of visualize_fusion_export.py — same scene conventions (1 Blender unit
= 1 mm, mm-scale viewport), same marker colours, same "no Empties / no
parenting" placement-only philosophy. The difference is the data source:

    visualize_fusion_export.py   -> reads fusion_export.json
                                    (CAD landmarks already in world frame)
    visualize_urdf.py (this one) -> reads facehugger.urdf
                                    (kinematic chain, must walk to get world)

This is what PyBullet does on `loadURDF`: parse the joint chain, compose
parent→child transforms at rest pose (all joint angles = 0), then for each
<visual> apply visual_origin in the link's frame. Because PyBullet's interp
is the gold standard, reproducing it here lets us cross-check the URDF
generator end-to-end: any disagreement between this script's output and the
fusion_export visualizer is either a URDF generation bug or a chain-walk bug
in this script.

The "sequential" intuition (build L and R legs in their natural pose, then
rotate and translate to corners) IS what the URDF encodes:
  • per-side mesh choice           — leg_shoulder_{L,R}.stl in <visual>
  • L-mesh-on-R-leg Y-flip          — visual rpy=(0, π, 0) on FR/BL link2/link3
  • back-of-pair 180° rotation      — joint rpy=(0,0,π) on BR/BL shoulder joint
  • per-side hip/knee axis sign     — joint axis="0 ±1 0"
The chain walk applies all four automatically.

Reads:
  code/simulation/generated/facehugger.urdf
  code/simulation/generated/exported_meshes/*.stl

Builds:
  - One mesh per <visual> at link_world @ visual_origin (29 visuals total).
  - Red spheres in "Joint Origins" sub-collection at each joint's world pivot.
  - Orange spheres in "Joint Axes" sub-collection — axis tip = pivot +
    20·(unit axis vector), so you can eyeball the rotation axis direction.

Usage (GUI — recommended for iterative debugging):
    blender --python animation/scripts/visualize_urdf.py

Usage (headless — smoke test / CI):
    blender --background --python animation/scripts/visualize_urdf.py \\
        -- --save /tmp/fh_urdf_viz.blend

CLI flags (after the `--` separator per Blender convention):
    --urdf   PATH   path to facehugger.urdf
    --meshes PATH   directory containing exported STLs (URDF mesh paths
                    are relative to the URDF file, this overrides that)
    --save   PATH   write a .blend here after building the scene

Targets Blender 5.x.
"""

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import bpy
from mathutils import Matrix, Vector


# ---------------------------------------------------------------------------
# Constants — match visualize_fusion_export.py so the two scenes overlay
# ---------------------------------------------------------------------------

# URDF is in METRES, our scene is in MILLIMETRES (1 BU = 1 mm). Apply this
# scale to every translation that comes out of the chain walk before handing
# it to Blender. Rotations are unitless and pass through unchanged.
M_TO_MM = 1000.0

SPHERE_RADIUS_MM = 3.0
AXIS_LENGTH_MM = 20.0   # how far past the joint pivot to draw the axis tip

ROOT_COLLECTION = "URDFViz"
MESHES_COLLECTION = "Meshes"
JOINTS_COLLECTION = "Joint Origins"
AXES_COLLECTION = "Joint Axes"

COLOR_JOINT = (1.0, 0.0, 0.0, 1.0)   # red — same as fusion_export's points
COLOR_AXIS = (1.0, 0.4, 0.0, 1.0)    # orange — same as fusion_export's axes


# ---------------------------------------------------------------------------
# URDF parsing (stdlib XML only — no rospkg / urdfpy dependency)
# ---------------------------------------------------------------------------

def _parse_origin(elem):
    """Return a 4×4 mathutils.Matrix from an <origin xyz=... rpy=.../> child
    of `elem`. Identity if absent. URDF rpy is the extrinsic XYZ convention,
    which composes as R = Rz(yaw) @ Ry(pitch) @ Rx(roll). Translation is in
    metres."""
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
    """Return the joint axis as a normalized 3-vector. Default (1,0,0)."""
    a = elem.find("axis")
    if a is None:
        return Vector((1.0, 0.0, 0.0))
    xyz = [float(v) for v in a.get("xyz").split()]
    v = Vector(xyz)
    return v.normalized() if v.length > 0 else Vector((1.0, 0.0, 0.0))


def _parse_visual(v):
    """(origin_matrix, mesh_filename, scale_xyz) or None if no <mesh>."""
    g = v.find("geometry/mesh")
    if g is None:
        return None
    fname = g.get("filename")
    scale = [float(s) for s in (g.get("scale") or "1 1 1").split()]
    return (_parse_origin(v), fname, scale)


def parse_urdf(path):
    """Parse the URDF into {root, links, joints}.

    links[name]              -> [(visual_origin_4x4, mesh_filename, scale_xyz), ...]
    joints[child_link_name]  -> {parent, origin, axis, name}
    root                     -> the unique link with no parent in joints
    """
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
        joints[child] = {
            "parent": parent,
            "origin": _parse_origin(j),
            "axis":   _parse_axis(j),
            "name":   j.get("name"),
        }

    children = set(joints.keys())
    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        sys.exit(f"{path}: expected exactly one root link, got {roots}")
    return {"root": roots[0], "links": links, "joints": joints}


# ---------------------------------------------------------------------------
# Forward kinematics (rest pose — joint angles all zero, like loadURDF)
# ---------------------------------------------------------------------------

def compute_link_world(robot):
    """Walk the joint tree from root and return {link_name: 4x4 matrix in m}.

    Joints aren't necessarily in topological order in the URDF, so we
    iterate-until-stable: each pass places every link whose parent is
    already known. Bails out loud if the URDF has a dangling chain."""
    link_world = {robot["root"]: Matrix.Identity(4)}
    remaining = dict(robot["joints"])
    while remaining:
        placed = []
        for child, j in remaining.items():
            if j["parent"] in link_world:
                # Rest pose: child = parent @ joint_origin (no R(axis, theta)
                # because all joint angles are 0). Add joint angle support
                # here later if you ever want a stance-pose snapshot.
                link_world[child] = link_world[j["parent"]] @ j["origin"]
                placed.append(child)
        if not placed:
            sys.exit(f"Dangling joint chain — could not place: "
                     f"{list(remaining.keys())}")
        for c in placed:
            del remaining[c]
    return link_world


def matrix_m_to_mm(M):
    """Return a copy of 4x4 matrix M with its translation scaled m → mm.
    Rotation passes through (rotations are unitless)."""
    out = M.copy()
    out.translation = out.translation * M_TO_MM
    return out


# ---------------------------------------------------------------------------
# Scene bootstrap (same idiom as visualize_fusion_export.py)
# ---------------------------------------------------------------------------

def clear_scene():
    """Wipe objects/collections/orphans so re-runs produce identical output.
    See visualize_fusion_export.py for why we don't use read_factory_settings."""
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    for coll in list(bpy.data.collections):
        bpy.data.collections.remove(coll)
    for mat in list(bpy.data.materials):
        bpy.data.materials.remove(mat)
    for mesh in list(bpy.data.meshes):
        bpy.data.meshes.remove(mesh)
    _configure_units()


def _configure_units():
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 0.001   # 1 BU = 1 mm
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
    children = {}
    for name in (MESHES_COLLECTION, JOINTS_COLLECTION, AXES_COLLECTION):
        c = bpy.data.collections.new(name)
        root.children.link(c)
        children[name] = c
    children[ROOT_COLLECTION] = root
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
# Placement primitives (mm-scale; identical signature to the JSON sibling)
# ---------------------------------------------------------------------------

def import_stl(stl_path, name, matrix_world_mm, target_collection):
    before = set(bpy.data.objects)
    try:
        # Scene is mm; STL vertices are mm; global_scale=1.0 keeps them.
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
    for c in list(obj.users_collection):
        c.objects.unlink(obj)
    target_collection.objects.link(obj)
    obj.matrix_world = matrix_world_mm
    return obj


def add_marker(world_pos_mm, name, material, target_collection,
               radius_mm=SPHERE_RADIUS_MM):
    before = set(bpy.data.objects)
    bpy.ops.mesh.primitive_uv_sphere_add(
        radius=radius_mm,
        location=(world_pos_mm[0], world_pos_mm[1], world_pos_mm[2]),
        segments=16, ring_count=8,
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


# ---------------------------------------------------------------------------
# Mesh + marker placement from URDF
# ---------------------------------------------------------------------------

def place_visuals(robot, link_world, meshes_dir, collections):
    """For each <visual>: import the STL and set obj.matrix_world =
    matrix_m_to_mm(link_world[link] @ visual_origin). Object name is
    `{link_name}__{stl_stem}` so the outliner stays grep-friendly."""
    placed = 0
    for link_name, visuals in robot["links"].items():
        link_W = link_world[link_name]
        for visual_origin, mesh_rel, _scale in visuals:
            mesh_path = (meshes_dir / Path(mesh_rel).name).resolve()
            if not mesh_path.exists():
                # Fallback: maybe the URDF path was relative to the URDF dir,
                # already resolved by the caller via meshes_dir. Try the raw
                # path too.
                mesh_path = Path(mesh_rel)
                if not mesh_path.exists():
                    print(f"[warn] mesh missing: {mesh_rel}")
                    continue
            mesh_world_mm = matrix_m_to_mm(link_W @ visual_origin)
            obj_name = f"{link_name}__{Path(mesh_rel).stem}"
            if import_stl(mesh_path, obj_name, mesh_world_mm,
                          collections[MESHES_COLLECTION]):
                placed += 1
    return placed


def place_joint_markers(robot, link_world, mat_joint, mat_axis, collections):
    """One red sphere at every joint's world pivot, plus an orange sphere
    20mm out along the joint axis so you can see which way the joint rotates.

    The joint pivot is the child link's frame origin (because joint origin
    is what places the child relative to its parent — see
    `compute_link_world`)."""
    n_joints = 0
    n_axes = 0
    for child, j in robot["joints"].items():
        child_W = link_world[child]
        pivot_mm = child_W.translation * M_TO_MM
        add_marker(pivot_mm, f"{j['name']}__pivot", mat_joint,
                   collections[JOINTS_COLLECTION])
        n_joints += 1

        # Axis tip in world: rotate the joint-axis unit vector into world via
        # the child link's rotation, scale by AXIS_LENGTH_MM, add to pivot.
        axis_world = child_W.to_3x3() @ j["axis"]
        tip_mm = pivot_mm + axis_world * AXIS_LENGTH_MM
        add_marker(tip_mm, f"{j['name']}__axis_tip", mat_axis,
                   collections[AXES_COLLECTION], radius_mm=2.0)
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
    default_urdf = (_script_dir() / ".." / ".."
                    / "code" / "simulation" / "generated" / "facehugger.urdf")
    default_meshes = (_script_dir() / ".." / ".."
                      / "code" / "simulation" / "generated" / "exported_meshes")

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=default_urdf.resolve())
    parser.add_argument("--meshes", type=Path, default=default_meshes.resolve())
    parser.add_argument("--save", type=Path, default=None)

    argv = sys.argv
    argv = argv[argv.index("--") + 1:] if "--" in argv else []
    return parser.parse_args(argv)


def main():
    args = parse_args()
    print(f"[visualize_urdf] urdf   : {args.urdf}")
    print(f"[visualize_urdf] meshes : {args.meshes}")

    if not args.urdf.exists():
        raise SystemExit(f"URDF not found: {args.urdf}")

    robot = parse_urdf(args.urdf)
    print(f"[visualize_urdf] parsed — {len(robot['links'])} links, "
          f"{len(robot['joints'])} joints, root={robot['root']}")

    link_world = compute_link_world(robot)

    clear_scene()
    collections = make_collections()
    mat_joint = make_material("JointPivotRed", COLOR_JOINT)
    mat_axis = make_material("JointAxisOrange", COLOR_AXIS)

    n_meshes = place_visuals(robot, link_world, args.meshes, collections)
    n_joints, n_axes = place_joint_markers(robot, link_world,
                                            mat_joint, mat_axis, collections)

    print(f"[visualize_urdf] placed {n_meshes} visuals, "
          f"{n_joints} joint pivots, {n_axes} axis tips")

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=str(args.save))
        print(f"[visualize_urdf] saved: {args.save}")


if __name__ == "__main__":
    main()
