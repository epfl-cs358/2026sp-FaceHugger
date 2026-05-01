"""
visualize_fusion_export.py  —  Blender debug scene builder (no rigging)

Reads:
  code/simulation/generated/fusion_export.json  (produced by ExportBodiesToURDF.py)
  code/simulation/generated/exported_meshes/*.stl

Builds (placement only — no Empties / armatures / parenting):
  - Chassis + one source leg + chassis-fixed servos at their exported
    world poses. Each STL in the `mesh_files` manifest is placed at its
    `origin_landmark`'s `pos_world_mm` from the occurrence tree.
    Combined-rule meshes are pre-baked in world frame relative to that
    landmark, so no occurrence-transform math is needed at import time.
  - 4 brackets + 4 shoulder servos + 4 leg copies at the chassis corners
    via `instance_four_legs`. Pure mesh duplication; no rig.
  - Red spheres in "Construction Points" sub-collection, one per
    `points[]` entry on every occurrence in the tree.
  - Orange spheres in "Axis Origins" sub-collection, one per `axes[]` entry.

Usage (GUI — recommended for iterative debugging):
    blender --python animation/scripts/visualize_fusion_export.py

Usage (headless — smoke test / CI):
    blender --background --python animation/scripts/visualize_fusion_export.py \\
        -- --save /tmp/fh_debug.blend

CLI flags (after the `--` separator per Blender convention):
    --export PATH   path to fusion_export.json
    --meshes PATH   directory containing exported STLs
    --save PATH     write a .blend here after building the scene

Targets Blender 5.x. (The original 3.3 LTS version of this script used
`bpy.ops.import_mesh.stl`; 5.x renamed it to `bpy.ops.wm.stl_import`.)
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
    """Wipe objects/collections/orphans so re-runs produce identical output.

    Avoid `bpy.ops.wm.read_factory_settings(use_empty=True)` (the original
    3.3-LTS version of this clear) — on Blender 5.x it leaves the
    `bpy.ops.wm.stl_import` operator's poll context invalid until the GUI
    has fully redrawn, so every subsequent STL import fails with
    "context is incorrect"."""
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
        # Blender 5.x. Scene scale is 1 BU = 1 mm and STL vertices are in mm,
        # so global_scale=1.0 lands geometry at the right size.
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


def _find_occ_at_path(nodes, path):
    """Resolve a slash-delimited occurrence path to the occurrence node
    (or None if any segment is missing)."""
    if not path:
        return None
    parts = path.split("/")
    cur = nodes
    node = None
    for seg in parts:
        node = next((c for c in (cur or []) if c.get("name") == seg), None)
        if node is None:
            return None
        cur = node.get("children", [])
    return node


def _find_landmark_world_pos(export, landmark_name, scope_occ_path=None):
    """Look up a construction point's `pos_world_mm`.

    If `scope_occ_path` is given, search there first (then walk up to the
    parent / grandparent / ... of that occurrence) — this disambiguates
    landmark names that exist on multiple occurrences (e.g. both
    `MotorMount:1` and `MotorMountR:1` carry a `LegMountFixedPoint`).
    Falls back to a tree-wide first-match if scoped search misses.
    """
    if landmark_name is None:
        return None

    occurrences = export.get("occurrences", [])

    # Scoped search: leaf → ancestors along the occurrence path.
    if scope_occ_path:
        segments = scope_occ_path.split("/")
        for i in range(len(segments), 0, -1):
            anc_path = "/".join(segments[:i])
            occ = _find_occ_at_path(occurrences, anc_path)
            if occ is None:
                continue
            for pt in occ.get("points", []):
                if pt.get("name") == landmark_name:
                    pos = pt.get("pos_world_mm")
                    if pos is not None:
                        return pos

    # Fallback: first match anywhere in the tree.
    for occ, _ in iter_occ(occurrences):
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

        # parts[0].occurrence is the body-bearing occurrence; we use it both
        # to scope the landmark lookup (so e.g. LegMountFixedPoint resolves
        # to MotorMountR:1's instance for leg_mount_R, not MotorMount:1's)
        # and to derive the object name.
        parts = entry.get("parts") or []
        first_occ_path = parts[0].get("occurrence", "") if parts else ""

        landmark = entry.get("origin_landmark")
        if landmark is None:
            # Chassis case (no re-origin): vertices are already in world frame.
            world_pos = [0.0, 0.0, 0.0]
        else:
            world_pos = _find_landmark_world_pos(
                export, landmark, scope_occ_path=first_occ_path
            )
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


# Per-leg config. Mirrors yaml legs[] in facehugger_config.yaml — same table
# the URDF generator uses for placement.
_LEGS = {
    "FL": {"side": "L", "rpy_z_deg":   0.0},
    "FR": {"side": "R", "rpy_z_deg":   0.0},
    "BR": {"side": "L", "rpy_z_deg": 180.0},
    "BL": {"side": "R", "rpy_z_deg": 180.0},
}


def _categorize_pool(pool):
    """Map source-leg object pool to its semantic role. Returns a dict with
    keys: bracket_L/R, link1_L/R, link2, link3, shoulder_servo (any may be
    None if the corresponding mesh wasn't imported).

    The classification keys off the occurrence basename baked into the
    object name during import: `FL_<occ_basename>_<stl>`.
    """
    def first_match(substr):
        return next((o for o in pool if substr in o.name), None)

    return {
        "bracket_L":      first_match("_MotorMount:1_"),
        "bracket_R":      first_match("_MotorMountR:1_"),
        "shoulder_servo": first_match("_Servo_Mouser_Model:1_"),
        "link1_L":        first_match("_Link1L:1_"),
        "link1_R":        first_match("_Link1R:1_"),
        "link2":          first_match("_Link2L:1_"),
        "link3":          first_match("_Link3L:1_"),
    }


def _copy_obj(src, new_name, target_collection, world_matrix):
    """Deep-copy a Blender object (with its mesh data) into a target
    collection at the given world matrix."""
    dup = src.copy()
    if dup.data is not None:
        dup.data = src.data.copy()
    dup.name = new_name
    dup.matrix_world = world_matrix
    target_collection.objects.link(dup)
    return dup


def instance_four_legs(export, collections, ctx):
    """Place 4 legs + 4 brackets + 4 shoulder servos at the chassis corners.

    The placement math mirrors `generate_urdf.py`:

      shoulder_world[c] = LegMountPoint[c] + Rz(rpy_z[c]) · side_offset[s]
      bracket_world[c]  = LegMountPoint[c]                          (chassis-fixed)
      shoulder_servo[c] = LegMountPoint[c] + (chassis-fixed shoulder offset)

    where `s` is "L" for FL/BR and "R" for FR/BL (diagonal-pair scheme),
    and `side_offset[L]` is the source-FL leg's BodyToLink1Point offset
    from LegMountPointFL. `side_offset[R]` is its X-mirror.

    Each leg is built by copying the appropriate-handedness source meshes
    (`leg_shoulder_{L,R}.stl` plus the shared `leg_upper.stl`/`leg_lower.stl`)
    and applying `T(shoulder_world) @ Rz(rpy_z) @ T(-source_anchor)` to each
    source's world matrix. `source_anchor = BodyToLink1Point world` since
    that's where `import_meshes_from_manifest` placed the leg meshes.

    KNOWN LIMITATIONS:
      • Link2 / Link3 are shared L-body meshes; for R-pair legs (FR/BL)
        the URDF emits them with `mesh_rpy = (0, π, 0)` to mirror across
        the leg-assembly XZ plane. The visualizer doesn't replicate that
        rpy, so the lower-leg geometry on FR/BL will look L-handed.
        Position is correct; orientation is approximate.
      • Hip / knee servos aren't in the manifest as separate STL entries
        (the URDF emits them programmatically using `servo.stl` plus a
        per-role rpy). The visualizer shows only the shoulder servo.

    Collection tree:
      FusionExport/
        Meshes/      {chassis only}
        Brackets/    {4 leg_mount_{L,R}.stl visuals at LegMountPointXX}
        Shoulders/   {4 servo.stl visuals at the 4 shoulder mounts}
        Legs/
          FL/{FL_root, link1_L + link2 + link3}
          FR/{FR_root, link1_R + link2 + link3}
          BR/{BR_root, link1_L + link2 + link3} (Rz(180°))
          BL/{BL_root, link1_R + link2 + link3} (Rz(180°))
    """
    # 1. Pull the 4 LegMountPointXX world positions from FlexibleSkeleton:1.
    fs = next(
        (o for o in export.get("occurrences", [])
         if o.get("name") == "FlexibleSkeleton:1"),
        None,
    )
    if fs is None:
        print("[instance_legs] FlexibleSkeleton:1 not in export; skipping")
        return
    PREFIX = "LegMountPoint"
    mounts = {}
    for pt in fs.get("points", []):
        name = pt.get("name", "")
        if name.startswith(PREFIX) and len(name) > len(PREFIX):
            corner = name[len(PREFIX):].upper()
            pos = pt.get("pos_world_mm") or pt.get("pos_mm")
            if pos:
                mounts[corner] = Vector(pos)
    if not all(c in mounts for c in _LEGS):
        print(f"[instance_legs] need {PREFIX}FR/FL/BR/BL, found {sorted(mounts)}; "
              f"skipping")
        return

    # 2. Source anchor = where leg meshes were imported (BodyToLink1Point world).
    source_anchor = _find_landmark_world_pos(export, "BodyToLink1Point")
    if source_anchor is None:
        print("[instance_legs] BodyToLink1Point not in tree; skipping")
        return
    source_anchor_v = Vector(source_anchor)

    # Side offsets: (-X mirror) of L_offset gives R_offset.
    L_offset = source_anchor_v - mounts["FL"]
    R_offset = Vector((-L_offset.x, L_offset.y, L_offset.z))
    side_offset = {"L": L_offset, "R": R_offset}

    # 3. Categorize the imported FL_-prefixed source pool.
    meshes_coll = collections[MESHES_COLLECTION]
    pool = [o for o in list(meshes_coll.objects)
            if o.name.startswith(LEG_SOURCE_PREFIX + "_")]
    sources = _categorize_pool(pool)

    # Pull every source out of Meshes/ — each one is a template that gets
    # replicated below, never displayed standalone.
    for obj in sources.values():
        if obj is None:
            continue
        for c in list(obj.users_collection):
            c.objects.unlink(obj)

    # 4. Build the new collection tree.
    root_coll = collections[ROOT_COLLECTION]
    legs_coll = bpy.data.collections.new("Legs")
    root_coll.children.link(legs_coll)
    corner_colls = {}
    for c in _LEGS:
        col = bpy.data.collections.new(c)
        legs_coll.children.link(col)
        corner_colls[c] = col

    brackets_coll = bpy.data.collections.new("Brackets")
    root_coll.children.link(brackets_coll)

    shoulders_coll = bpy.data.collections.new("Shoulders")
    root_coll.children.link(shoulders_coll)

    # 5. Place 4 brackets at LegMountPointXX (chassis-fixed, no rotation).
    n_brackets = 0
    for corner, conf in _LEGS.items():
        src = sources["bracket_L" if conf["side"] == "L" else "bracket_R"]
        if src is None:
            continue
        target_pos = mounts[corner]
        # Translate from source world position to corner's mount.
        src_pos = src.matrix_world.translation.copy()
        delta = target_pos - src_pos
        _copy_obj(
            src,
            f"{corner}_bracket_{conf['side']}",
            brackets_coll,
            Matrix.Translation(delta) @ src.matrix_world,
        )
        n_brackets += 1

    # 6. Place 4 shoulder servos at LegMountPointXX (chassis-fixed, no rotation).
    n_shoulders = 0
    if sources["shoulder_servo"] is not None:
        servo_src = sources["shoulder_servo"]
        servo_src_pos = servo_src.matrix_world.translation.copy()
        # Servo source's mesh-local origin is at ServoMountPoint world,
        # which sits directly above LegMountPointFL by ~36 mm in Z. The
        # delta from FL_mount to source position is the chassis-uniform
        # shoulder offset.
        servo_offset = servo_src_pos - mounts["FL"]
        for corner in _LEGS:
            target = mounts[corner] + servo_offset
            delta = target - servo_src_pos
            _copy_obj(
                servo_src,
                f"{corner}_shoulder_servo",
                shoulders_coll,
                Matrix.Translation(delta) @ servo_src.matrix_world,
            )
            n_shoulders += 1

    # 7. Place 4 legs (per-leg shoulder origin + Rz(rpy_z) rotation).
    leg_pool_by_side = {
        "L": [sources["link1_L"], sources["link2"], sources["link3"]],
        "R": [sources["link1_R"], sources["link2"], sources["link3"]],
    }
    n_legs_total = 0
    for corner, conf in _LEGS.items():
        side = conf["side"]
        rpy_z_deg = conf["rpy_z_deg"]
        rz = _rz_4x4(rpy_z_deg)
        # Rotate the (X, Y) components of side_offset by rpy_z (Z is invariant).
        target_shoulder = mounts[corner] + (rz.to_3x3() @ side_offset[side])

        # Per-leg world transform applied to each source mesh:
        #   M = T(target_shoulder) @ Rz(rpy_z) @ T(-source_anchor)
        leg_xform = (
            Matrix.Translation(target_shoulder)
            @ rz
            @ Matrix.Translation(-source_anchor_v)
        )

        # No rigging Empty here — pure mesh placement. The duplicates land
        # in their corner sub-collection at world positions; a future rig
        # script can parent them onto bones / Empties as needed.
        for src in leg_pool_by_side[side]:
            if src is None:
                continue
            stl_part = src.name.split("_", 2)[-1]   # strip "FL_<occ>_" → keep "<stl>"
            _copy_obj(
                src,
                f"{corner}_{stl_part}",
                corner_colls[corner],
                leg_xform @ src.matrix_world,
            )
            n_legs_total += 1

    print(f"[instance_legs] Brackets/ has {n_brackets}, "
          f"Shoulders/ has {n_shoulders}, "
          f"Legs/ has {n_legs_total} parts across 4 corners")
    print(f"[instance_legs] per-corner rpy_z — "
          f"FL/FR=0°, BR/BL=180°; side offsets L=({L_offset.x:+.1f}, "
          f"{L_offset.y:+.1f}, {L_offset.z:+.1f}) mm, R=X-mirror")


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
