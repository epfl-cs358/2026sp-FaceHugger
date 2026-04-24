"""
visualize_fusion_export.py  —  Blender 3.3 LTS debug scene builder

Reads:
  code/simulation/fusion_export.json  (produced by ExportBodiesToURDF.py)
  code/simulation/meshes/*.stl

Builds:
  - Chassis + one leg + servos at their exported world poses, each piece
    placed according to the `mesh_files` manifest (or a hardcoded fallback
    if the manifest isn't present yet).
  - Red spheres in "Construction Points" sub-collection, one per
    `points[]` entry on every occurrence in the tree.
  - Orange spheres in "Axis Origins" sub-collection, one per `axes[]` entry.

The difference between a construction Point and an Axis origin is exactly
what we want to debug — seeing both side by side exposes joint-frame bugs
that are otherwise invisible in generated URDFs.

Usage (GUI — recommended for iterative debugging):
    blender --python animation/scripts/visualize_fusion_export.py

Usage (headless — smoke test / CI):
    blender --background --python animation/scripts/visualize_fusion_export.py \\
        -- --save /tmp/fh_debug.blend

CLI flags (after the `--` separator per Blender convention):
    --export PATH   path to fusion_export.json
    --meshes PATH   directory containing exported STLs
    --save PATH     write a .blend here after building the scene

Targets Blender 3.3 LTS specifically — uses `bpy.ops.import_mesh.stl`. On
4.x swap that one call to `bpy.ops.wm.stl_import`.
"""

import argparse
import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Matrix, Vector


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Scene is configured so 1 Blender unit = 1 mm (see `_configure_units` below).
# Fusion's STL export writes vertex values in mm, so importing with
# global_scale=1.0 already lands in the right scale with no conversion. All
# positions in the JSON are in mm too, so we use them at face value.
MM_SCALE = 1.0   # mm → Blender units (no conversion; kept for readability)
CM_TO_MM = 10.0  # world_transform_rm_cm's translation column is in cm

SPHERE_RADIUS_MM = 3.0   # 3 mm markers

ROOT_COLLECTION = "FusionExport"
MESHES_COLLECTION = "Meshes"
POINTS_COLLECTION = "Construction Points"
AXES_COLLECTION = "Axis Origins"

COLOR_POINT = (1.0, 0.0, 0.0, 1.0)   # red
COLOR_AXIS = (1.0, 0.4, 0.0, 1.0)    # orange

# Label prefix for the source leg assembly (the one physical leg in the
# CAD — the other three are virtual and only exist in URDF output).
LEG_SOURCE_PREFIX = "FL"

# Fallback mesh rules used when `mesh_files` isn't in the JSON yet (i.e.
# you haven't re-run the updated ExportBodiesToURDF.py). Format:
#   path_suffix: (stl_filename, origin_shift_mm or None)
# Longer suffixes win via endswith matching, evaluated in dict order.
FALLBACK_MESH_RULES = {
    "FlexibleSkeleton:1":                           ("QuadrupedBody.stl", None),
    "FaceHuggerLegAssembly:1/Link1:1":              ("leg_shoulder.stl",  None),
    "FaceHuggerLegAssembly:1/Link2:1":              ("leg_upper.stl",     None),
    "FaceHuggerLegAssembly:1/Link3:1":              ("leg_lower.stl",     None),
}


# ---------------------------------------------------------------------------
# JSON helpers (self-contained — no imports from code/simulation)
# ---------------------------------------------------------------------------


def load_export(path):
    with open(path) as f:
        return json.load(f)


def iter_occ(nodes, parent_path=""):
    """Yield (occ, path) for every occurrence at every depth."""
    for n in nodes:
        path = f"{parent_path}/{n['name']}" if parent_path else n["name"]
        yield n, path
        yield from iter_occ(n.get("children", []), path)


def find_occ_by_path(nodes, path):
    for occ, p in iter_occ(nodes):
        if p == path:
            return occ
    return None


# ---------------------------------------------------------------------------
# Scene bootstrap
# ---------------------------------------------------------------------------


def clear_scene():
    """Factory-reset the scene so re-runs produce identical output."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    _configure_units()


def _configure_units():
    """Set the scene to millimetre scale: 1 Blender unit = 1 mm, display
    shows values in mm. Grid and viewport clipping stay sensible for a
    ~200mm-wide robot."""
    scene = bpy.context.scene
    scene.unit_settings.system = 'METRIC'
    scene.unit_settings.scale_length = 0.001   # 1 Blender unit = 1 mm
    scene.unit_settings.length_unit = 'MILLIMETERS'
    # Nudge viewport clip so the 200mm-scale robot is visible without
    # near-clipping artifacts at default settings.
    for area in bpy.context.screen.areas if bpy.context.screen else []:
        if area.type == 'VIEW_3D':
            for space in area.spaces:
                if space.type == 'VIEW_3D':
                    space.clip_start = 0.1
                    space.clip_end = 10000.0


def make_collections():
    """Create the FusionExport/{Meshes, Construction Points, Axis Origins}
    hierarchy and return a dict keyed by collection name."""
    scene_coll = bpy.context.scene.collection
    root = bpy.data.collections.new(ROOT_COLLECTION)
    scene_coll.children.link(root)
    children = {}
    for name in (MESHES_COLLECTION, POINTS_COLLECTION, AXES_COLLECTION):
        c = bpy.data.collections.new(name)
        root.children.link(c)
        children[name] = c
    children[ROOT_COLLECTION] = root
    return children


# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------


def make_material(name, rgba):
    """Principled BSDF material with a given base color. Used for the
    construction-point + axis-origin markers."""
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    principled = next((n for n in nodes if n.type == "BSDF_PRINCIPLED"), None)
    if principled is not None:
        principled.inputs["Base Color"].default_value = rgba
    mat.diffuse_color = rgba   # fallback viewport color (solid shading)
    return mat


# ---------------------------------------------------------------------------
# Placement primitives
# ---------------------------------------------------------------------------


def _world_matrix_from_occ(occ, shift_mm=None):
    """Build a Blender `matrix_world` for the mesh imported at this occurrence,
    with positions in mm (Blender units here = 1 mm — see `_configure_units`).

    shift_mm: the value from mesh_files[...].origin_shift_mm — the landmark
    position (in the mesh's original local frame) that was subtracted from
    vertices during STL export. When None/zero, no shift (mesh not re-origined).

    For a re-origined mesh, the mesh's local (0,0,0) corresponds to the
    landmark at `shift_mm` in the original local frame. Applying the
    occurrence's world transform to `shift_mm` gives that landmark's world
    position — which is where we want the mesh's new origin to sit.
    """
    rm_cm = occ.get("world_transform_rm_cm")
    if not rm_cm:
        # Fall back to just placing at world_origin with identity rotation.
        origin_mm = occ.get("world_origin_mm", [0.0, 0.0, 0.0])
        return Matrix.Translation(Vector(origin_mm))

    # Rotation: upper-left 3x3 (dimensionless).
    rot_3x3 = [row[:3] for row in rm_cm[:3]]
    # Translation from the matrix: right column is in cm → mm.
    tx = rm_cm[0][3] * CM_TO_MM
    ty = rm_cm[1][3] * CM_TO_MM
    tz = rm_cm[2][3] * CM_TO_MM

    if shift_mm is not None and any(v != 0 for v in shift_mm):
        # Compute R @ shift (mm), add to translation so the mesh's new (0,0,0)
        # origin lands at landmark_world = T + R @ shift.
        sx, sy, sz = shift_mm
        rx = rot_3x3[0][0] * sx + rot_3x3[0][1] * sy + rot_3x3[0][2] * sz
        ry = rot_3x3[1][0] * sx + rot_3x3[1][1] * sy + rot_3x3[1][2] * sz
        rz = rot_3x3[2][0] * sx + rot_3x3[2][1] * sy + rot_3x3[2][2] * sz
        tx += rx
        ty += ry
        tz += rz

    return Matrix((
        (rot_3x3[0][0], rot_3x3[0][1], rot_3x3[0][2], tx),
        (rot_3x3[1][0], rot_3x3[1][1], rot_3x3[1][2], ty),
        (rot_3x3[2][0], rot_3x3[2][1], rot_3x3[2][2], tz),
        (0.0,           0.0,           0.0,           1.0),
    ))


def import_stl(stl_path, name, matrix_world, target_collection):
    """Import an STL file and place it with the given world matrix inside
    `target_collection`. Returns the created object, or None on failure."""
    before = set(bpy.data.objects)
    try:
        # Blender 3.3: bpy.ops.import_mesh.stl (built-in STL addon). Swap to
        # bpy.ops.wm.stl_import on 4.x.
        bpy.ops.import_mesh.stl(filepath=str(stl_path))
    except Exception as e:
        print(f"[warn] STL import failed for {stl_path}: {e}")
        return None
    new_objs = [o for o in bpy.data.objects if o not in before]
    if not new_objs:
        print(f"[warn] STL import created no objects: {stl_path}")
        return None
    obj = new_objs[0]
    obj.name = name

    # Move into target collection.
    for c in list(obj.users_collection):
        c.objects.unlink(obj)
    target_collection.objects.link(obj)

    obj.matrix_world = matrix_world
    return obj


def add_marker(world_pos_mm, name, material, target_collection, radius_mm=SPHERE_RADIUS_MM):
    """Drop a UV sphere at `world_pos_mm` (mm, world frame) into the target
    collection, assign it the given material. Uses a pre/post object-set diff
    to find the newly-created sphere because `bpy.context.active_object` is
    not always available in `blender --python` GUI mode."""
    before = set(bpy.data.objects)
    bpy.ops.mesh.primitive_uv_sphere_add(
        radius=radius_mm,
        location=(world_pos_mm[0] * MM_SCALE,
                  world_pos_mm[1] * MM_SCALE,
                  world_pos_mm[2] * MM_SCALE),
        segments=16,
        ring_count=8,
    )
    new_objs = [o for o in bpy.data.objects if o not in before]
    if not new_objs:
        print(f"[warn] sphere creation failed: {name}")
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
# Mesh rule resolution (manifest + fallback)
# ---------------------------------------------------------------------------


def build_mesh_rule_lookup(export):
    """Return a dict `{occurrence_path: (stl_filename, origin_shift_mm)}`.

    Primary source: `mesh_files` manifest written by the updated
    ExportBodiesToURDF. Each entry's `source_occurrences` list enumerates
    every occurrence that uses that STL (e.g. 3 Servo_Mouser_Model:N
    instances all pointing at servo.stl).

    Fallback: the hardcoded FALLBACK_MESH_RULES suffix map, matched via
    endswith. Kept so the script is useful before the user re-runs Fusion.
    """
    lookup = {}

    manifest = export.get("mesh_files") or {}
    if manifest:
        for stl_name, entry in manifest.items():
            if not isinstance(entry, dict):
                continue   # skip _servo_role_assignment (a sub-dict)
            if "source_occurrences" not in entry:
                continue   # skip anything not describing a mesh
            shift = entry.get("origin_shift_mm") or [0.0, 0.0, 0.0]
            for path in entry["source_occurrences"]:
                lookup[path] = (stl_name, shift)
        return lookup

    # No manifest — synthesize rules by matching path suffixes against the tree.
    # Longest suffixes first so more specific rules win.
    rules = sorted(FALLBACK_MESH_RULES.items(), key=lambda kv: -len(kv[0]))
    for occ, path in iter_occ(export.get("occurrences", [])):
        for suffix, (stl, shift) in rules:
            if path == suffix or path.endswith("/" + suffix):
                lookup[path] = (stl, shift)
                break
    return lookup


# ---------------------------------------------------------------------------
# Visitor walk
# ---------------------------------------------------------------------------


class VisitContext:
    def __init__(self, meshes_dir, collections, mesh_lookup, mat_point, mat_axis):
        self.meshes_dir = Path(meshes_dir)
        self.collections = collections
        self.mesh_lookup = mesh_lookup
        self.mat_point = mat_point
        self.mat_axis = mat_axis
        self.meshes_imported = 0
        self.points_placed = 0
        self.axes_placed = 0
        self.missing_meshes = []


def _label_name(path, suffix):
    """Prefix the source-leg assembly with LEG_SOURCE_PREFIX, leave other
    paths alone. Produces stable, grep-friendly names in the Blender outliner.
    """
    if path.startswith("FaceHuggerLegAssembly:1"):
        return f"{LEG_SOURCE_PREFIX}_{suffix}"
    return suffix


def visit_occurrence(occ, path, ctx):
    # Mesh (if a rule matches this path)
    if path in ctx.mesh_lookup:
        stl_name, shift_mm = ctx.mesh_lookup[path]
        stl_path = ctx.meshes_dir / stl_name
        if not stl_path.exists():
            if stl_name not in ctx.missing_meshes:
                print(f"[warn] mesh missing: {stl_path}")
                ctx.missing_meshes.append(stl_name)
        else:
            matrix_world = _world_matrix_from_occ(occ, shift_mm)
            obj_name = _label_name(path, f"{occ['name']}_{stl_name}")
            if import_stl(stl_path, obj_name, matrix_world, ctx.collections[MESHES_COLLECTION]):
                ctx.meshes_imported += 1

    # Construction points → red spheres
    for pt in occ.get("points", []):
        if pt.get("pos_world_mm") is None:
            continue
        label = _label_name(path, f"{occ['name']}/{pt['name']}")
        add_marker(pt["pos_world_mm"], label,
                   ctx.mat_point, ctx.collections[POINTS_COLLECTION])
        ctx.points_placed += 1

    # Axis origins → orange spheres
    for ax in occ.get("axes", []):
        if ax.get("origin_world_mm") is None:
            continue
        label = _label_name(path, f"{occ['name']}/{ax['name']}")
        add_marker(ax["origin_world_mm"], label,
                   ctx.mat_axis, ctx.collections[AXES_COLLECTION])
        ctx.axes_placed += 1

    for child in occ.get("children", []):
        child_path = f"{path}/{child['name']}"
        visit_occurrence(child, child_path, ctx)


def visit_root(export, ctx):
    # Root-level axes / points, if any
    for pt in export.get("root_points", []):
        if pt.get("pos_world_mm") is None and pt.get("pos_mm") is None:
            continue
        pos = pt.get("pos_world_mm") or pt.get("pos_mm")
        add_marker(pos, f"root/{pt['name']}",
                   ctx.mat_point, ctx.collections[POINTS_COLLECTION])
        ctx.points_placed += 1
    for ax in export.get("root_axes", []):
        pos = ax.get("origin_world_mm") or ax.get("origin_mm")
        if pos is None:
            continue
        add_marker(pos, f"root/{ax['name']}",
                   ctx.mat_axis, ctx.collections[AXES_COLLECTION])
        ctx.axes_placed += 1

    for occ, path in ((o, o["name"]) for o in export.get("occurrences", [])):
        visit_occurrence(occ, path, ctx)


# ---------------------------------------------------------------------------
# 4-leg instancing post-pass
# ---------------------------------------------------------------------------


def instance_four_legs(export, collections, ctx):
    """After the single source leg is loaded, duplicate it at the 4 chassis
    corners as independent objects (with deep-copied mesh data) so each
    leg can be rigged and animated independently.

    FL + BL (same body-side as source FL): translation-only duplication —
        the leg keeps its source orientation, just moved to the target
        LegMountXX. Same local frame as the source, rigging-friendly.

    FR + BR (opposite body-side): same translation PLUS a 180° rotation
        around Z centered on each corner's LegMountXX point. The pivot
        sits on the shoulder rotation axis (LegMountXX = BodyToLink1Point
        are colocated on the axis), so the whole leg — including the
        shoulder servo — swings to face the opposite body-side. Every
        duplicated object gets the rotation: link1/link2/link3 + all
        three servos.

    Matrix composition for each duplicate at corner N:
        M_new = T(mount_N) @ Rz(Δ) @ T(-mount_FL) @ M_src
      where Δ = 180° for FR/BR, 0° for BL (translation-only).

    Collection tree:
      FusionExport/Legs/
        FL/      source leg (Link1/2/3 + 3 servos), kept in place
        BL/      translation-only duplicate
        FR/      translation + 180° Z rotation (pivot = LegMountFR)
        BR/      translation + 180° Z rotation (pivot = LegMountBR)
    """
    # 1. Pull the 4 LegMountXX points from FlexibleSkeleton:1's construction points.
    mounts = {}
    fs = next(
        (o for o in export.get("occurrences", [])
         if o.get("name") == "FlexibleSkeleton:1"),
        None,
    )
    if fs is None:
        print("[instance_legs] FlexibleSkeleton:1 not in export; skipping")
        return
    for pt in fs.get("points", []):
        name = pt.get("name", "")
        if name.startswith("LegMount") and len(name) > len("LegMount"):
            corner = name[len("LegMount"):].upper()  # FR/FL/BR/BL
            pos = pt.get("pos_world_mm") or pt.get("pos_mm")
            if pos:
                mounts[corner] = pos

    if not all(c in mounts for c in ("FR", "FL", "BR", "BL")):
        print(f"[instance_legs] need LegMountFR/FL/BR/BL, found {sorted(mounts)}; skipping")
        return

    # 2. Create Legs root + 4 per-corner sub-collections.
    root_coll = collections[ROOT_COLLECTION]
    legs_coll = bpy.data.collections.new("Legs")
    root_coll.children.link(legs_coll)

    corner_colls = {}
    for corner in ("FL", "FR", "BR", "BL"):
        c = bpy.data.collections.new(corner)
        legs_coll.children.link(c)
        corner_colls[corner] = c

    # 3. Move source-leg objects (prefixed with LEG_SOURCE_PREFIX="FL_") from
    # the Meshes collection into Legs/FL.
    meshes_coll = collections[MESHES_COLLECTION]
    source_prefix = LEG_SOURCE_PREFIX + "_"
    source_objs = [o for o in list(meshes_coll.objects)
                   if o.name.startswith(source_prefix)]
    for obj in source_objs:
        meshes_coll.objects.unlink(obj)
        corner_colls["FL"].objects.link(obj)

    # 4. Duplicate the source leg for FR/BR/BL. FR/BR are rotated 180°
    # around Z centered on their own LegMountXX point; BL is translation-
    # only. Deep-copy mesh data so rig weights on each leg are independent.
    fl_mount_v = Vector(mounts["FL"])
    rot_by_corner = {
        "FR": Matrix.Rotation(math.pi, 4, "Z"),
        "BR": Matrix.Rotation(math.pi, 4, "Z"),
        "BL": Matrix.Identity(4),
    }
    n_dup_per_leg = 0
    for corner in ("FR", "BR", "BL"):
        m_v = Vector(mounts[corner])
        # M_new = T(mount_N) @ R @ T(-mount_FL) @ M_src
        leg_xform = (
            Matrix.Translation(m_v)
            @ rot_by_corner[corner]
            @ Matrix.Translation(-fl_mount_v)
        )
        count = 0
        for src in source_objs:
            dup = src.copy()
            if dup.data is not None:
                dup.data = src.data.copy()    # independent mesh for per-leg rigging
            # Rename: strip the "FL_" prefix, prepend the target corner prefix.
            if src.name.startswith(source_prefix):
                dup.name = corner + "_" + src.name[len(source_prefix):]
            else:
                dup.name = f"{corner}_{src.name}"
            dup.matrix_world = leg_xform @ src.matrix_world
            corner_colls[corner].objects.link(dup)
            count += 1
        n_dup_per_leg = count

    n_src = len(source_objs)
    print(f"[instance_legs] Legs/FL has {n_src} source objects; "
          f"duplicated to {n_dup_per_leg} objects each in FR/BR/BL "
          f"(FR/BR rotated 180° around Z about their LegMount)")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def _script_dir():
    # __file__ is reliable under `blender --python`; falls back to CWD.
    try:
        return Path(__file__).resolve().parent
    except NameError:
        return Path.cwd()


def parse_args():
    """Argparse splits Blender's argv at `--`. Before the `--` are Blender's
    own args; after, our script's args."""
    default_export = _script_dir() / ".." / ".." / "code" / "simulation" / "fusion_export.json"
    default_meshes = _script_dir() / ".." / ".." / "code" / "simulation" / "exported_meshes"

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--export", type=Path, default=default_export.resolve())
    parser.add_argument("--meshes", type=Path, default=default_meshes.resolve())
    parser.add_argument("--save", type=Path, default=None)

    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    else:
        argv = []
    return parser.parse_args(argv)


def main():
    args = parse_args()
    print(f"[visualize] export : {args.export}")
    print(f"[visualize] meshes : {args.meshes}")

    if not args.export.exists():
        raise SystemExit(f"fusion_export.json not found: {args.export}")

    export = load_export(args.export)

    clear_scene()
    collections = make_collections()
    mat_point = make_material("ConstructionPointRed", COLOR_POINT)
    mat_axis = make_material("AxisOriginOrange", COLOR_AXIS)

    mesh_lookup = build_mesh_rule_lookup(export)
    if not (export.get("mesh_files") or {}):
        print("[visualize] no mesh_files manifest — using fallback hardcoded rules")

    ctx = VisitContext(args.meshes, collections, mesh_lookup, mat_point, mat_axis)
    visit_root(export, ctx)

    # Replicate the source (FL) leg at the other 3 corners via collection
    # instances, each in its own Legs/{FR,FL,BR,BL} sub-collection.
    instance_four_legs(export, collections, ctx)

    print(
        f"[visualize] imported {ctx.meshes_imported} meshes, "
        f"placed {ctx.points_placed} points, {ctx.axes_placed} axis origins"
    )
    if ctx.missing_meshes:
        print(f"[visualize] missing STLs: {', '.join(ctx.missing_meshes)}")

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=str(args.save))
        print(f"[visualize] saved: {args.save}")


if __name__ == "__main__":
    main()
