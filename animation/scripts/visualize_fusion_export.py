"""
visualize_fusion_export.py  —  Blender 3.3 LTS debug scene builder

Reads:
  code/simulation/generated/fusion_export.json  (produced by ExportBodiesToURDF.py)
  code/simulation/generated/exported_meshes/*.stl

Builds:
  - Chassis + one leg + servos at their exported world poses. Each STL
    in the `mesh_files` manifest is placed at its `origin_landmark`'s
    world position (combined-rule meshes are pre-baked in world frame
    relative to that landmark, so no occurrence-transform math is
    needed at import time).
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

# Path prefix that flags an occurrence as "inside the source leg assembly".
# Objects whose backing occurrence lives under this path get the FL_ name
# prefix so `instance_four_legs` can find them as duplicate sources.
LEG_ASSEMBLY_OCC_PREFIX = "FaceHuggerLegAssembly:1"


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
# Mesh import (manifest-driven, post-Phase-E exporter)
# ---------------------------------------------------------------------------


def _find_landmark_world_pos(export, landmark_name):
    """Find the first construction point with this name anywhere in the
    occurrence tree, return its `pos_world_mm`. None if not found."""
    if landmark_name is None:
        return None
    for occ, _ in iter_occ(export.get("occurrences", [])):
        for pt in occ.get("points", []):
            if pt.get("name") == landmark_name:
                pos = pt.get("pos_world_mm")
                if pos is not None:
                    return pos
    return None


def import_meshes_from_manifest(export, ctx):
    """Place each STL once at its `origin_landmark`'s world position.

    Background: post-Phase-E, `EXPORT_RULES` for everything inside the
    leg assembly use the combined-rule, which bakes vertices into world
    frame and subtracts `landmark_world` so mesh-local (0,0,0) lands on
    the landmark in world coords. Restoring the world placement is then
    just `Translation(landmark_world)` — no rotation, since vertices
    are already world-aligned.

    This replaces the old per-occurrence walk that consumed
    `mesh_files[*].source_occurrences`. After Phase A that field became
    `["FlexibleSkeleton:1"]` for every STL (the top-level chassis
    occurrence the combined rule walks down from), so the old logic
    collapsed all 8 STLs onto a single key and only one rendered.
    """
    manifest = export.get("mesh_files") or {}
    if not manifest:
        print("[visualize] no mesh_files manifest — meshes will not import")
        return

    chassis_occ = next(
        (o.get("name") for o in export.get("occurrences", []) if o.get("name")),
        "scene",
    )

    for stl_name, entry in manifest.items():
        if not isinstance(entry, dict) or "parts" not in entry:
            continue   # skip _servo_role_assignment etc.

        landmark = entry.get("origin_landmark")
        if landmark is None:
            # Chassis case (no re-origin): vertices are already in world frame.
            world_pos = [0.0, 0.0, 0.0]
        else:
            world_pos = _find_landmark_world_pos(export, landmark)
            if world_pos is None:
                print(f"[warn] landmark {landmark!r} not found in tree; "
                      f"skipping {stl_name}")
                continue

        stl_path = ctx.meshes_dir / stl_name
        if not stl_path.exists():
            if stl_name not in ctx.missing_meshes:
                print(f"[warn] mesh missing: {stl_path}")
                ctx.missing_meshes.append(stl_name)
            continue

        # Naming: prefix with FL_ if the mesh's primary part lives inside
        # the leg assembly, so instance_four_legs picks it up as a source-leg
        # object. Otherwise (chassis / brackets) leave the name plain.
        parts = entry.get("parts") or []
        first_occ_path = parts[0].get("occurrence", "") if parts else ""
        is_leg_internal = first_occ_path.startswith(LEG_ASSEMBLY_OCC_PREFIX)
        occ_basename = (first_occ_path.split("/")[-1]
                        if first_occ_path else chassis_occ)
        prefix = f"{LEG_SOURCE_PREFIX}_" if is_leg_internal else ""
        obj_name = f"{prefix}{occ_basename}_{stl_name}"

        matrix = Matrix.Translation(Vector(world_pos))
        if import_stl(stl_path, obj_name, matrix,
                      ctx.collections[MESHES_COLLECTION]):
            ctx.meshes_imported += 1


# ---------------------------------------------------------------------------
# Visitor walk
# ---------------------------------------------------------------------------


class VisitContext:
    def __init__(self, meshes_dir, collections, mat_point, mat_axis):
        self.meshes_dir = Path(meshes_dir)
        self.collections = collections
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


# Per-corner rotation: diagonal-pair scheme (see docs/PIPELINE_SPEC.md §1).
# FL/FR use 0°; BR/BL use 180° about Z. Same table the URDF generator uses
# (yaml legs[].rpy_z_deg). With the new CAD the bracket occurrences moved
# inside the leg assembly, so reading per-corner rotations from the export
# tree (the old _legmount_rot_3x3 path) no longer works — this hardcoded
# table replaces it.
_PER_CORNER_RPY_Z_DEG = {"FL": 0.0, "FR": 0.0, "BR": 180.0, "BL": 180.0}


def _rz_4x4(deg):
    import math
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return Matrix((
        (c, -s, 0.0, 0.0),
        (s,  c, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    ))


def _shoulder_servo_occ_basename(export):
    """Return the shoulder servo's occurrence basename (e.g.
    'Servo_Mouser_Model:1') from mesh_files._servo_role_assignment, or
    None if the manifest doesn't flag it."""
    mf = export.get("mesh_files") or {}
    sra = mf.get("_servo_role_assignment") or {}
    path = sra.get("shoulder")
    if not path:
        return None
    return path.split("/")[-1]


def instance_four_legs(export, collections, ctx):
    """After the single source leg is loaded, replicate it at the four
    chassis corners as independent objects (with deep-copied mesh data) so
    each leg can be rigged and animated independently.

    The shoulder servo is treated as *chassis-fixed*, not leg-owned — it's
    bolted to the chassis bracket and doesn't rotate with the leg. So:
      • It sits in a `Shoulders` sub-collection next to `Legs/`, not under
        the per-corner leg collections.
      • All four shoulder-servo copies inherit the source (FL) orientation
        with translation-only to the other three mounts — no per-corner
        bracket rotation. This matches the URDF's chassis-uniform shoulder
        treatment.

    Each corner's leg objects get a per-corner Z-rotation from the
    diagonal-pair scheme (yaml's `rpy_z_deg`):

        FL/FR: Rz(0°)        BR/BL: Rz(180°)

    Same table the URDF generator uses. Each corner's leg objects are
    parented to a `{corner}_root` Empty placed at `LegMountPointXX` with
    that rotation baked in — rotating the Empty pivots the whole leg
    around the shoulder axis.

    KNOWN LIMITATION: this routine duplicates the L-source leg meshes for
    every corner, so the FR / BL legs (which should use the R-side
    `leg_shoulder_R.stl` / `leg_mount_R.stl` / mirrored link2/link3) will
    look L-handed in Blender even though their position/rotation are
    correct. Use `view_urdf.py` for a kinematically-faithful 4-leg view;
    this script remains useful for debugging the export tree (chassis,
    construction points, axis origins, source leg).

    Collection tree:
      FusionExport/
        Legs/
          FL/  {FL_root, 3 links + hip servo + knee servo}   (R_rel = I)
          FR/  {FR_root, duplicated 5 leg objects}           (R_rel = Ry(180°))
          BR/  {BR_root, duplicated 5 leg objects}           (R_rel = Rz(180°))
          BL/  {BL_root, duplicated 5 leg objects}           (R_rel = Rx(180°))
        Shoulders/  {4 shoulder servos, all with FL orientation}
    """
    # 1. Pull the 4 LegMountPointXX construction points from FlexibleSkeleton:1.
    # New CAD names them LegMountPointFL/FR/BR/BL (with the "Point" infix);
    # the older LegMountXX names are gone.
    mounts = {}
    fs = next(
        (o for o in export.get("occurrences", [])
         if o.get("name") == "FlexibleSkeleton:1"),
        None,
    )
    if fs is None:
        print("[instance_legs] FlexibleSkeleton:1 not in export; skipping")
        return
    PREFIX = "LegMountPoint"
    for pt in fs.get("points", []):
        name = pt.get("name", "")
        if name.startswith(PREFIX) and len(name) > len(PREFIX):
            corner = name[len(PREFIX):].upper()      # FR/FL/BR/BL
            pos = pt.get("pos_world_mm") or pt.get("pos_mm")
            if pos:
                mounts[corner] = pos

    if not all(c in mounts for c in ("FR", "FL", "BR", "BL")):
        print(f"[instance_legs] need {PREFIX}FR/FL/BR/BL, found {sorted(mounts)}; skipping")
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

    # 3. Pull the source-leg objects out of the Meshes collection. Split
    # into (a) the shoulder servo (chassis-fixed, goes to Shoulders) and
    # (b) the remaining 5 leg objects (go to Legs/FL).
    meshes_coll = collections[MESHES_COLLECTION]
    source_prefix = LEG_SOURCE_PREFIX + "_"
    all_source_objs = [o for o in list(meshes_coll.objects)
                       if o.name.startswith(source_prefix)]

    shoulder_basename = _shoulder_servo_occ_basename(export)
    shoulder_src = None
    leg_source_objs = []
    for obj in all_source_objs:
        # Object name pattern: "FL_<occurrence_name>_<stl_filename>"
        # e.g. "FL_Servo_Mouser_Model:1_servo.stl"
        if shoulder_basename and f"_{shoulder_basename}_" in obj.name:
            shoulder_src = obj
        else:
            leg_source_objs.append(obj)

    # Create Shoulders sub-collection and move source shoulder there.
    shoulders_coll = bpy.data.collections.new("Shoulders")
    root_coll.children.link(shoulders_coll)
    if shoulder_src is not None:
        meshes_coll.objects.unlink(shoulder_src)
        shoulders_coll.objects.link(shoulder_src)

    # Move the 5 remaining leg objects into Legs/FL.
    for obj in leg_source_objs:
        meshes_coll.objects.unlink(obj)
        corner_colls["FL"].objects.link(obj)

    # 4. Per-corner rotations from the diagonal-pair scheme. The old code
    # derived these from the LegMountXX:1 occurrences' world_transform_rm_cm,
    # but those occurrences moved into the leg assembly in the new CAD and
    # the lookup no longer resolves. The diagonal-pair table from yaml is
    # what the URDF generator uses anyway, so it's the source of truth.
    rot_by_corner = {c: _rz_4x4(_PER_CORNER_RPY_Z_DEG[c])
                     for c in ("FL", "FR", "BR", "BL")}

    # 5. Create a root Empty per corner at LegMountPointXX, carrying the per-corner
    # rotation. FL's root is placed but the source leg already lives in FL's
    # sub-collection at its source poses, so we parent the children without
    # moving them (matrix_parent_inverse).
    fl_mount_v = Vector(mounts["FL"])
    roots = {}
    for corner in ("FL", "FR", "BR", "BL"):
        m_v = Vector(mounts[corner])
        root = bpy.data.objects.new(f"{corner}_root", None)  # Empty
        root.empty_display_type = "ARROWS"
        root.empty_display_size = 20.0  # mm scene; 20 mm is readable without dominating
        root.matrix_world = Matrix.Translation(m_v) @ rot_by_corner[corner]
        corner_colls[corner].objects.link(root)
        roots[corner] = root

    # FL leg children: already at their source poses — parent without
    # moving them.
    for obj in leg_source_objs:
        obj.parent = roots["FL"]
        obj.matrix_parent_inverse = roots["FL"].matrix_world.inverted()

    # 6. Duplicate the 5 leg objects for FR/BR/BL. Each duplicate's world
    # matrix is `T(mount_N) @ rot_by_corner[N] @ T(-mount_FL) @ M_src`,
    # then we parent to the corner's root Empty without altering world
    # placement. Deep-copy mesh data so rig weights on each leg stay
    # independent.
    n_dup_per_leg = 0
    for corner in ("FR", "BR", "BL"):
        m_v = Vector(mounts[corner])
        leg_xform = (
            Matrix.Translation(m_v)
            @ rot_by_corner[corner]
            @ Matrix.Translation(-fl_mount_v)
        )
        count = 0
        for src in leg_source_objs:
            dup = src.copy()
            if dup.data is not None:
                dup.data = src.data.copy()
            if src.name.startswith(source_prefix):
                dup.name = corner + "_" + src.name[len(source_prefix):]
            else:
                dup.name = f"{corner}_{src.name}"
            dup.parent = None
            dup.matrix_world = leg_xform @ src.matrix_world
            dup.parent = roots[corner]
            dup.matrix_parent_inverse = roots[corner].matrix_world.inverted()
            corner_colls[corner].objects.link(dup)
            count += 1
        n_dup_per_leg = count

    # 7. Duplicate the shoulder servo for FR/BR/BL with translation only
    # (chassis-fixed orientation = FL's orientation). All four live in
    # Shoulders/, unparented — they don't move with any leg.
    n_shoulders = 0
    if shoulder_src is not None:
        n_shoulders = 1  # count the source
        for corner in ("FR", "BR", "BL"):
            m_v = Vector(mounts[corner])
            offset = m_v - fl_mount_v
            dup = shoulder_src.copy()
            if dup.data is not None:
                dup.data = shoulder_src.data.copy()
            if shoulder_src.name.startswith(source_prefix):
                dup.name = corner + "_" + shoulder_src.name[len(source_prefix):]
            else:
                dup.name = f"{corner}_{shoulder_src.name}"
            dup.matrix_world = Matrix.Translation(offset) @ shoulder_src.matrix_world
            shoulders_coll.objects.link(dup)
            n_shoulders += 1

    rels = ", ".join(f"{c}=Rz({_PER_CORNER_RPY_Z_DEG[c]:+.0f}°)"
                     for c in ("FR", "BR", "BL"))
    print(f"[instance_legs] Legs/FL has {len(leg_source_objs)} source leg objects; "
          f"duplicated to {n_dup_per_leg} objects each in FR/BR/BL; "
          f"Shoulders/ has {n_shoulders} shoulder servos (chassis-fixed)")
    print(f"[instance_legs] per-corner rotation — {rels}; "
          f"each leg parented to a {{FL,FR,BR,BL}}_root Empty at LegMountPointXX")


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
    default_export = _script_dir() / ".." / ".." / "code" / "simulation" / "generated" / "fusion_export.json"
    default_meshes = _script_dir() / ".." / ".." / "code" / "simulation" / "generated" / "exported_meshes"

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

    ctx = VisitContext(args.meshes, collections, mat_point, mat_axis)

    # Import the 8 STLs from the manifest, each placed at its
    # origin_landmark's world position (combined-rule meshes are
    # already in world frame relative to that landmark).
    import_meshes_from_manifest(export, ctx)

    # Walk the occurrence tree for construction-point + axis-origin markers.
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
