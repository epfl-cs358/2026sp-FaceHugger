"""
ExportBodiesToURDF.py  —  FaceHugger Fusion assembly exporter

Outputs to code/simulation/ (relative to this script's repo location):
  fusion_export.json     machine-readable, consumed by generate_urdf.py +
                         the Blender visualizer. Includes a `mesh_files`
                         manifest describing each produced STL and its
                         source occurrences + re-origin info.
  fusion_export.txt      human-readable tree for sanity checking
  exported_meshes/*.stl  STLs driven by EXPORT_RULES (below). Link meshes
                         are re-origined so their local (0,0,0) coincides
                         with the URDF joint landmark (BodyToLink1Point,
                         Link1ToLink2Point, Link2ToLink3Point) — this
                         lets the URDF generator emit <visual><origin
                         xyz="0 0 0"/> for each link.

Re-exports preserve user edits to mesh_files._servo_role_assignment.

Run via: Shift+S → Scripts and Add-Ins → ExportBodiesToURDF → Run
"""

import json
import os
import struct
import traceback

import adsk.core  # ty:ignore[unresolved-import]
import adsk.fusion  # ty:ignore[unresolved-import]

app = adsk.core.Application.get()
ui = app.userInterface

# ---------------------------------------------------------------------------
# Paths — relative to this script's location inside the repo.
# Script lives at:  cad/scripts/ExportBodiesToURDF/ExportBodiesToURDF/
# Repo root is 4 levels up, then down to code/simulation/.
# ---------------------------------------------------------------------------

_HERE = os.path.dirname(os.path.abspath(__file__))
_SIM_DIR = os.path.normpath(
    os.path.join(_HERE, "..", "..", "..", "..", "code", "simulation")
)
_MESH_DIR = os.path.join(_SIM_DIR, "exported_meshes")

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

VISIBLE_ONLY = False  # skip hidden geometry
COLLECT_PHYSICS = True  # mass/CoM/inertia — slow, set False to skip
CM_TO_MM = 10.0
CM2_TO_M2 = 1e-4  # kg·cm² → kg·m²

# Rules driving STL export. Three rule types supported:
#
#   {"type": "occurrence", "match": "FlexibleSkeleton:1", "stl": "X.stl"}
#     Export the full subtree of a matching occurrence as one STL. Fusion
#     bakes all visible descendants at their placements — everything that
#     happens to be visible at export time ends up baked in. Use only when
#     you actually want everything; otherwise prefer `combined`.
#
#   {"type": "body", "match": "Link1", "stl": "X.stl",
#    "origin_landmark": "BodyToLink1Point"}  # optional
#     Export a single body by name, using the FIRST occurrence containing
#     a body with that name. If `origin_landmark` is set, STL vertices are
#     translated so that landmark's local position becomes the mesh origin
#     — downstream URDF can then use <visual><origin xyz="0 0 0"/>.
#
#   {"type": "combined", "stl": "X.stl", "parts": [
#       {"occurrence": "A:1/B:1", "body": "BodyName"},
#       {"occurrence": "A:1/C:1", "body": "*"},  # "*" = first body in the occ
#   ]}
#     For each part: export just that body, then transform its vertices by
#     the occurrence's world_transform_rm_cm, then concatenate all parts
#     into one binary STL. Output vertices land in root/world frame. Use
#     when you need a precise subset of bodies stitched into one mesh, e.g.
#     chassis plate + 4 LegMount brackets without the electronics.
#
# Downstream consumers (URDF generator, Blender visualizer) read the
# `mesh_files` manifest section of fusion_export.json to know what was
# produced and how to place each mesh — they don't hardcode these rules.
EXPORT_RULES = [
    {"type": "combined", "stl": "QuadrupedBody.stl", "parts": [
        {"occurrence": "FlexibleSkeleton:1/QuadrupedBody:1", "body": "QuadrupedBody"},
        {"occurrence": "FlexibleSkeleton:1/LegMountFR:1",    "body": "*"},
        {"occurrence": "FlexibleSkeleton:1/LegMountFL:1",    "body": "*"},
        {"occurrence": "FlexibleSkeleton:1/LegMountBR:1",    "body": "*"},
        {"occurrence": "FlexibleSkeleton:1/LegMountBL:1",    "body": "*"},
    ]},
    {"type": "body", "match": "Link1",
     "stl": "leg_shoulder.stl", "origin_landmark": "BodyToLink1Point"},
    {"type": "body", "match": "Link2",
     "stl": "leg_upper.stl",    "origin_landmark": "Link1ToLink2Point"},
    {"type": "body", "match": "Link3",
     "stl": "leg_lower.stl",    "origin_landmark": "Link2ToLink3Point"},
    {"type": "body", "match": "ServoBase",
     "stl": "servo.stl"},
]


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def pt_mm(p):
    return [
        round(p.x * CM_TO_MM, 3),
        round(p.y * CM_TO_MM, 3),
        round(p.z * CM_TO_MM, 3),
    ]


def vec3(v):
    return [round(v.x, 4), round(v.y, 4), round(v.z, 4)]


def bbox_center_mm(body):
    bb = body.boundingBox
    return [
        round((bb.minPoint.x + bb.maxPoint.x) / 2.0 * CM_TO_MM, 3),
        round((bb.minPoint.y + bb.maxPoint.y) / 2.0 * CM_TO_MM, 3),
        round((bb.minPoint.z + bb.maxPoint.z) / 2.0 * CM_TO_MM, 3),
    ]


def occ_parent_origin_mm(occ):
    """Translation of `occ.transform`, in mm. This is PARENT-relative (Fusion's
    `Occurrence.transform` returns the placement of the occurrence within its
    immediate assembly context, not within the root document). To get world
    coordinates, multiply by the accumulated parent transform during traversal.
    """
    t = occ.transform
    return [
        round(t.translation.x * CM_TO_MM, 3),
        round(t.translation.y * CM_TO_MM, 3),
        round(t.translation.z * CM_TO_MM, 3),
    ]


def matrix_as_row_major_cm(m):
    """Fusion `Matrix3D.asArray()` returns a 16-element list in row-major
    order. Return the matrix as a 4x4 list of floats in cm (unmodified)."""
    a = m.asArray()
    return [list(a[i * 4 : (i + 1) * 4]) for i in range(4)]


def matrix_translation_mm(m):
    a = m.asArray()
    return [
        round(a[3] * CM_TO_MM, 3),
        round(a[7] * CM_TO_MM, 3),
        round(a[11] * CM_TO_MM, 3),
    ]


def apply_matrix_cm(m, p_cm):
    """Apply a 4x4 Matrix3D (row-major, cm units) to a 3-vector (cm). Returns
    the transformed 3-vector in cm."""
    a = m.asArray()
    x, y, z = p_cm
    return (
        a[0] * x + a[1] * y + a[2] * z + a[3],
        a[4] * x + a[5] * y + a[6] * z + a[7],
        a[8] * x + a[9] * y + a[10] * z + a[11],
    )


def apply_matrix_to_dir_cm(m, d_cm):
    """Apply only the rotational part of a 4x4 matrix to a direction vector."""
    a = m.asArray()
    x, y, z = d_cm
    return (
        a[0] * x + a[1] * y + a[2] * z,
        a[4] * x + a[5] * y + a[6] * z,
        a[8] * x + a[9] * y + a[10] * z,
    )


def point_world_mm(p, world_transform):
    """Lift a Point3D (Fusion cm units) to world-frame mm using a 4x4 matrix."""
    wx, wy, wz = apply_matrix_cm(world_transform, (p.x, p.y, p.z))
    return [round(wx * CM_TO_MM, 3), round(wy * CM_TO_MM, 3), round(wz * CM_TO_MM, 3)]


def dir_world(d, world_transform):
    """Lift a Vector3D direction to world-frame (unitless, just rotate)."""
    dx, dy, dz = apply_matrix_to_dir_cm(world_transform, (d.x, d.y, d.z))
    return [round(dx, 4), round(dy, 4), round(dz, 4)]


def mat_multiply(parent, local):
    """Return a new adsk.core.Matrix3D = parent * local. Computed explicitly
    from the underlying 4x4 row-major arrays so we don't depend on the exact
    Fusion semantics of Matrix3D.transformBy (pre- vs post-multiply, which is
    annoying to verify without an interactive kernel)."""
    pa = parent.asArray()
    la = local.asArray()
    out = [0.0] * 16
    for i in range(4):
        for j in range(4):
            s = 0.0
            for k in range(4):
                s += pa[i * 4 + k] * la[k * 4 + j]
            out[i * 4 + j] = s
    result = adsk.core.Matrix3D.create()
    result.setWithArray(out)
    return result


def identity_matrix():
    return adsk.core.Matrix3D.create()


# ---------------------------------------------------------------------------
# Collectors
# ---------------------------------------------------------------------------


def collect_bodies(comp):
    out = []
    for body in comp.bRepBodies:
        if VISIBLE_ONLY and not body.isLightBulbOn:
            continue
        out.append({
            "name": body.name,
            "visible": body.isLightBulbOn,
            "bbox_center_mm": bbox_center_mm(body),
        })
    return out


def collect_axes(comp, world_transform):
    """`world_transform` is the 4x4 matrix mapping this component's local frame
    into world (root document) frame. Axes are stored in BOTH local frame
    (legacy `origin_mm`/`dir`) and world frame (`origin_world_mm`/`dir_world`)
    so downstream tools can pick whichever they need without re-walking the
    transform tree.
    """
    out = []
    for axis in comp.constructionAxes:
        if VISIBLE_ONLY and not axis.isLightBulbOn:
            continue
        try:
            geom = axis.geometry  # Line3D
            out.append({
                "name": axis.name,
                "visible": axis.isLightBulbOn,
                "origin_mm": pt_mm(geom.origin),
                "dir": vec3(geom.direction),
                "origin_world_mm": point_world_mm(geom.origin, world_transform),
                "dir_world": dir_world(geom.direction, world_transform),
            })
        except Exception as e:
            out.append({"name": axis.name, "error": str(e)})
    return out


def collect_points(comp, world_transform):
    out = []
    for point in comp.constructionPoints:
        if VISIBLE_ONLY and not point.isLightBulbOn:
            continue
        try:
            geom = point.geometry
            out.append({
                "name": point.name,
                "visible": point.isLightBulbOn,
                "pos_mm": pt_mm(geom),
                "pos_world_mm": point_world_mm(geom, world_transform),
            })
        except Exception as e:
            out.append({"name": point.name, "error": str(e)})
    return out


def collect_physics(occ, parent_to_world):
    """Report CoM in both the occurrence's parent frame (`com_mm`, as Fusion
    returns it) and root/world (`com_world_mm`).

    Important: `Occurrence.getPhysicalProperties().centerOfMass` returns the
    CoM in the occurrence's IMMEDIATE PARENT assembly frame, not the
    occurrence's own local frame. To lift it to world we therefore apply
    `parent_to_world`, NOT `this_to_world = parent_to_world * occ.transform`.
    Previously the code applied `this_to_world`, which double-transformed the
    CoM for every occurrence with a non-identity `occ.transform` (e.g.
    MotorMountBR: raw com (45.3, -36.9, 9.0) got reported as world
    (84.3, -72.9, 19.0) — off by exactly the occurrence's own translation).

    Inertia tensor is left in its Fusion form; principal-axis rotation under
    a world transform is an application-level concern (most URDF consumers
    want it about the CoM anyway, not about the link origin)."""
    try:
        acc = adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy
        prop = occ.getPhysicalProperties(acc)
        vals = prop.getXYZMomentsOfInertia()
        ixx, iyy, izz, ixy, iyz, ixz = [round(v * CM2_TO_M2, 9) for v in vals[1:]]
        com = prop.centerOfMass
        return {
            "mass_kg": round(prop.mass, 6),
            "com_mm": pt_mm(com),
            "com_world_mm": point_world_mm(com, parent_to_world),
            "inertia_kg_m2": {
                "ixx": ixx,
                "iyy": iyy,
                "izz": izz,
                "ixy": ixy,
                "iyz": iyz,
                "ixz": ixz,
            },
        }
    except Exception as e:
        return {"error": str(e)}


# ---------------------------------------------------------------------------
# STL export
# ---------------------------------------------------------------------------


def _read_binary_stl(path):
    """Return a list of (normal, v0, v1, v2) tuples where each vertex/normal
    is a 3-tuple of floats. Units match whatever Fusion wrote (mm for this
    CAD)."""
    with open(path, "rb") as f:
        data = f.read()
    ntri = struct.unpack_from("<I", data, 80)[0]
    tris = []
    off = 84
    for _ in range(ntri):
        nx, ny, nz = struct.unpack_from("<fff", data, off)
        v0 = struct.unpack_from("<fff", data, off + 12)
        v1 = struct.unpack_from("<fff", data, off + 24)
        v2 = struct.unpack_from("<fff", data, off + 36)
        tris.append(((nx, ny, nz), v0, v1, v2))
        off += 50
    return tris


def _write_binary_stl(path, triangles):
    """Write a standard binary STL: 80-byte header, uint32 triangle count,
    per-triangle 12+36+2 bytes (normal + 3 verts + attr)."""
    buf = bytearray()
    buf.extend(b"\x00" * 80)  # header
    buf.extend(struct.pack("<I", len(triangles)))
    for n, v0, v1, v2 in triangles:
        buf.extend(struct.pack("<fff", *n))
        buf.extend(struct.pack("<fff", *v0))
        buf.extend(struct.pack("<fff", *v1))
        buf.extend(struct.pack("<fff", *v2))
        buf.extend(b"\x00\x00")
    with open(path, "wb") as f:
        f.write(buf)


def _transform_triangle(tri, rot_3x3, translation_mm):
    """Apply `rot @ v + translation` to each vertex; rotate the normal by
    `rot` only (no translation). Assumes rigid rotation (det ±1, no scale)."""
    (nx, ny, nz), v0, v1, v2 = tri
    r = rot_3x3
    tx, ty, tz = translation_mm

    def rv(v):
        x, y, z = v
        return (
            r[0][0] * x + r[0][1] * y + r[0][2] * z + tx,
            r[1][0] * x + r[1][1] * y + r[1][2] * z + ty,
            r[2][0] * x + r[2][1] * y + r[2][2] * z + tz,
        )

    return (
        (
            r[0][0] * nx + r[0][1] * ny + r[0][2] * nz,
            r[1][0] * nx + r[1][1] * ny + r[1][2] * nz,
            r[2][0] * nx + r[2][1] * ny + r[2][2] * nz,
        ),
        rv(v0),
        rv(v1),
        rv(v2),
    )


def _translate_binary_stl(path, shift_mm):
    """Translate every vertex in a binary STL file by `-shift_mm` in-place so
    the point previously at `shift_mm` becomes the new origin. Fusion's STL
    export writes in millimeters regardless of the design's native units, so
    the shift vector here is also in mm.

    Binary STL layout: 80-byte header, uint32 triangle count, then per
    triangle: 3 floats (normal) + 9 floats (v0,v1,v2 xyz) + uint16 attr = 50
    bytes. We don't touch the normals or attributes — only the 9 vertex
    floats of each triangle get `shift_mm` subtracted.
    """
    sx, sy, sz = shift_mm
    with open(path, "rb") as f:
        data = bytearray(f.read())
    if len(data) < 84:
        raise RuntimeError(f"STL too short: {path}")
    ntri = struct.unpack_from("<I", data, 80)[0]
    expected = 84 + ntri * 50
    if len(data) != expected:
        raise RuntimeError(
            f"STL size mismatch: {path} has {len(data)} bytes, expected {expected}"
        )
    off = 84
    for _ in range(ntri):
        # skip 3 normal floats (12 bytes); 9 vertex floats follow
        v_off = off + 12
        verts = list(struct.unpack_from("<9f", data, v_off))
        for i in range(3):
            verts[i * 3 + 0] -= sx
            verts[i * 3 + 1] -= sy
            verts[i * 3 + 2] -= sz
        struct.pack_into("<9f", data, v_off, *verts)
        off += 50
    with open(path, "wb") as f:
        f.write(data)


def _iter_occ_tree(occurrences):
    """Yield (occ, path) for every occurrence reachable from `occurrences`,
    depth-first. Path is `Name:1/Child:1/...` using Fusion occurrence names."""
    for occ in occurrences:
        yield occ, occ.name
        for child, sub_path in _iter_occ_tree(occ.component.occurrences):
            yield child, f"{occ.name}/{sub_path}"


def _find_body_in_occ(occ, body_name):
    """Return the first BRepBody in `occ.component` whose name matches, else None."""
    for b in occ.component.bRepBodies:
        if b.name == body_name:
            return b
    return None


def _find_occ_and_world_transform(tree, occurrences_json, path):
    """Return `(live_occ, world_transform_rm_cm)` for the occurrence at `path`
    (e.g. 'FlexibleSkeleton:1/LegMountFR:1'), or `(None, None)` if not found.

    `tree` is the pre-walked list from `_iter_occ_tree(root.occurrences)` —
    a sequence of (live_occ, path) tuples. `occurrences_json` is the output
    of `traverse(...)`, carrying the accumulated `world_transform_rm_cm` per
    occurrence so we don't have to recompute transforms.
    """
    live_occ = None
    for occ, p in tree:
        if p == path:
            live_occ = occ
            break
    if live_occ is None:
        return None, None

    parts = path.split("/")
    nodes = occurrences_json
    node = None
    for part in parts:
        node = next((n for n in nodes if n.get("name") == part), None)
        if node is None:
            return live_occ, None
        nodes = node.get("children", [])
    if node is None:   # empty path case
        return live_occ, None
    return live_occ, node.get("world_transform_rm_cm")


def _find_landmark_pos_mm(root, landmark_name):
    """Search all construction points in the tree by name; return its local-frame
    position in mm (its `geometry` scaled from cm). Returns None if not found."""
    # Root component's own construction points
    for p in root.constructionPoints:
        if p.name == landmark_name:
            try:
                return pt_mm(p.geometry)
            except Exception:
                return None
    # Descend into each occurrence
    for occ, _ in _iter_occ_tree(root.occurrences):
        for p in occ.component.constructionPoints:
            if p.name == landmark_name:
                try:
                    return pt_mm(p.geometry)
                except Exception:
                    return None
    return None


def _export_one(mgr, entity, filename):
    """Common STL export call. `entity` is an Occurrence or BRepBody."""
    opts = mgr.createSTLExportOptions(entity, filename)
    opts.sendToPrintUtility = False
    opts.isBinaryFormat = True
    opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
    mgr.execute(opts)


def export_stls(design, root, occurrences_json):
    """Execute EXPORT_RULES. Returns (exported_entries, failed_list) where
    `exported_entries` is a list of dicts describing what was produced (used
    by the `mesh_files` manifest), one entry per successfully-written STL.

    `occurrences_json` is the output of `traverse()`, needed so the
    `combined` rule can look up each part's `world_transform_rm_cm`.
    """
    os.makedirs(_MESH_DIR, exist_ok=True)
    mgr = design.exportManager
    exported = []
    failed = []

    # Pre-walk tree once so we can match rules efficiently.
    tree = list(_iter_occ_tree(root.occurrences))

    for rule in EXPORT_RULES:
        stl_name = rule["stl"]
        filename = os.path.join(_MESH_DIR, stl_name)
        rtype = rule["type"]
        try:
            if rtype == "occurrence":
                match = rule["match"]
                # First occurrence whose LEAF name matches (ignoring path).
                target_occ = None
                source_path = None
                for occ, path in tree:
                    if occ.name == match:
                        target_occ = occ
                        source_path = path
                        break
                if target_occ is None:
                    failed.append(f"{stl_name}: no occurrence named {match}")
                    continue
                _export_one(mgr, target_occ, filename)
                exported.append({
                    "stl": stl_name,
                    "source_type": "occurrence",
                    "source_occurrences": [source_path],
                    "origin_landmark": None,
                    "origin_shift_mm": [0.0, 0.0, 0.0],
                })
            elif rtype == "body":
                match = rule["match"]
                # Every occurrence whose component contains a body with this name.
                # Used to populate `source_occurrences` so consumers know all the
                # places this STL is instanced (e.g. 3 Servo_Mouser_Model:N for
                # one leg).
                matching_paths = []
                target_body = None
                for occ, path in tree:
                    body = _find_body_in_occ(occ, match)
                    if body is not None:
                        matching_paths.append(path)
                        if target_body is None:
                            target_body = body  # export the first one
                if target_body is None:
                    failed.append(f"{stl_name}: no body named {match}")
                    continue
                _export_one(mgr, target_body, filename)
                # Re-origin step
                landmark = rule.get("origin_landmark")
                shift_mm = [0.0, 0.0, 0.0]
                if landmark:
                    pos = _find_landmark_pos_mm(root, landmark)
                    if pos is None:
                        failed.append(
                            f"{stl_name}: origin_landmark {landmark} not found"
                        )
                        # Still produced an un-re-origined STL; record as-is.
                    else:
                        _translate_binary_stl(filename, pos)
                        shift_mm = pos
                exported.append({
                    "stl": stl_name,
                    "source_type": "body",
                    "source_body": match,
                    "source_occurrences": matching_paths,
                    "origin_landmark": landmark,
                    "origin_shift_mm": shift_mm,
                })
            elif rtype == "combined":
                # Explicit list of (occurrence_path, body_name) parts. For each,
                # export the body alone (native component body), then transform
                # its vertices by the occurrence's world_transform so everything
                # lands in root/world frame. Concatenate all transformed tris
                # into one binary STL.
                parts = rule.get("parts", [])
                out_tris = []
                fatal = False
                for i, part in enumerate(parts):
                    occ_path = part["occurrence"]
                    body_name = part["body"]
                    live_occ, wtf_cm = _find_occ_and_world_transform(
                        tree, occurrences_json, occ_path
                    )
                    if live_occ is None or wtf_cm is None:
                        failed.append(
                            f"{stl_name}: occurrence {occ_path} not found "
                            f"(or missing world transform)"
                        )
                        fatal = True
                        break
                    # Resolve body: "*" → first body in component; else by name.
                    if body_name == "*":
                        bodies = live_occ.component.bRepBodies
                        body = bodies[0] if bodies.count > 0 else None
                    else:
                        body = _find_body_in_occ(live_occ, body_name)
                    if body is None:
                        failed.append(
                            f"{stl_name}: body {body_name!r} in {occ_path} not found"
                        )
                        fatal = True
                        break
                    # Export to a temp file, read triangles, transform, accumulate.
                    # Must end in .stl — Fusion's STL exporter silently no-ops
                    # on other extensions.
                    tmp_path = filename + f".part{i}.stl"
                    _export_one(mgr, body, tmp_path)
                    try:
                        part_tris = _read_binary_stl(tmp_path)
                    finally:
                        try:
                            os.remove(tmp_path)
                        except OSError:
                            pass
                    rot = [[wtf_cm[r][c] for c in range(3)] for r in range(3)]
                    tx_mm = [wtf_cm[r][3] * CM_TO_MM for r in range(3)]
                    for t in part_tris:
                        out_tris.append(_transform_triangle(t, rot, tx_mm))
                if fatal:
                    continue
                _write_binary_stl(filename, out_tris)
                exported.append({
                    "stl": stl_name,
                    "source_type": "combined",
                    # Place at world origin — vertices are already in world frame.
                    # Using FlexibleSkeleton:1 (which is at origin) so the Blender
                    # consumer's world_transform lookup yields identity.
                    "source_occurrences": ["FlexibleSkeleton:1"],
                    "parts": list(parts),
                    "origin_landmark": None,
                    "origin_shift_mm": [0.0, 0.0, 0.0],
                })
            else:
                failed.append(f"{stl_name}: unknown rule type {rtype!r}")
        except Exception as e:
            failed.append(f"{stl_name}: {e}")

    return exported, failed


def build_mesh_files_manifest(exported, preserved_role_assignment=None):
    """Turn the list of `exported` dicts from export_stls into the keyed
    `mesh_files` manifest section written to fusion_export.json.

    `preserved_role_assignment` is the `_servo_role_assignment` block from a
    prior JSON (if it exists), so user edits survive re-export. If none is
    passed, defaults are written: the first 3 occurrences with a ServoBase
    body are mapped to shoulder/hip/knee by list order.
    """
    manifest = {}
    servo_paths = []
    for entry in exported:
        stl_name = entry["stl"]
        # Copy everything except "stl" (it's the key).
        manifest[stl_name] = {k: v for k, v in entry.items() if k != "stl"}
        if entry.get("source_body") == "ServoBase":
            servo_paths = list(entry.get("source_occurrences", []))

    if preserved_role_assignment is not None:
        manifest["_servo_role_assignment"] = preserved_role_assignment
    else:
        roles = ["shoulder", "hip", "knee"]
        assignment = {
            "comment": (
                "USER-EDIT: map servo occurrence paths to kinematic roles. "
                "Defaults follow the order servos appear in the export; "
                "adjust after inspecting meshes in Blender or the sim. "
                "Re-exports preserve edits to this block."
            ),
        }
        for i, role in enumerate(roles):
            assignment[role] = servo_paths[i] if i < len(servo_paths) else None
        manifest["_servo_role_assignment"] = assignment

    return manifest


# ---------------------------------------------------------------------------
# Recursive traversal
# ---------------------------------------------------------------------------


def traverse(occurrences, parent_to_world=None):
    """Recursive walk that accumulates the world transform.

    `occ.transform` is PARENT-RELATIVE in Fusion — it gives the placement of
    the occurrence within its immediate assembly context, not within the root
    document. To get each node's world (root-frame) transform, we pre-multiply
    by the parent's world transform. Passing this accumulator down lets every
    nested occurrence — including arbitrarily deep xref subtrees — report its
    points/axes/CoM in a single consistent frame.

    We still emit the raw local-frame values so downstream code that already
    understands the local convention keeps working.
    """
    if parent_to_world is None:
        parent_to_world = identity_matrix()
    result = []
    for occ in occurrences:
        if VISIBLE_ONLY and not occ.isLightBulbOn:
            continue
        comp = occ.component
        # world = parent_to_world * occ.transform
        this_to_world = mat_multiply(parent_to_world, occ.transform)
        node = {
            "name": occ.name,
            "xref": occ.isReferencedComponent,
            "visible": occ.isLightBulbOn,
            # Parent-relative translation (old misnamed "world_origin_mm").
            "parent_origin_mm": occ_parent_origin_mm(occ),
            # True root-frame translation.
            "world_origin_mm": matrix_translation_mm(this_to_world),
            # Full 4x4 row-major matrix mapping this occurrence's local frame
            # into world frame (cm units, Fusion native). Useful for consumers
            # that need to lift arbitrary local vectors into world.
            "world_transform_rm_cm": matrix_as_row_major_cm(this_to_world),
            "bodies": collect_bodies(comp),
            "axes": collect_axes(comp, this_to_world),
            "points": collect_points(comp, this_to_world),
            "children": [],
        }
        if COLLECT_PHYSICS:
            # CoM from Fusion is in the occurrence's PARENT frame, so apply
            # parent_to_world here, not this_to_world (which would add the
            # occurrence's own transform a second time).
            node["physics"] = collect_physics(occ, parent_to_world)
        if comp.occurrences.count > 0:
            node["children"] = traverse(comp.occurrences, this_to_world)
        result.append(node)
    return result


# ---------------------------------------------------------------------------
# Text formatter
# ---------------------------------------------------------------------------


def fmt_tree(nodes, depth=0):
    lines = []
    pad = "  " * depth
    for n in nodes:
        xref_tag = " [xref]" if n["xref"] else ""
        vis_tag = "" if n["visible"] else " [hidden]"
        lines.append(f"{pad}{n['name']}{xref_tag}{vis_tag}")
        lines.append(
            f"{pad}  parent_origin_mm: {tuple(n['parent_origin_mm'])}  "
            f"world_origin_mm: {tuple(n['world_origin_mm'])}"
        )
        for b in n["bodies"]:
            v = "" if b["visible"] else " [hidden]"
            lines.append(
                f"{pad}  body: {b['name']}{v} | bbox_center_mm: {tuple(b['bbox_center_mm'])}"
            )
        for a in n["axes"]:
            if "error" in a:
                lines.append(f"{pad}  caxis: {a['name']} | ERROR: {a['error']}")
            else:
                v = "" if a["visible"] else " [hidden]"
                lines.append(
                    f"{pad}  caxis: {a['name']}{v} "
                    f"| local origin_mm: {tuple(a['origin_mm'])} dir: {tuple(a['dir'])} "
                    f"| world origin_mm: {tuple(a['origin_world_mm'])} dir: {tuple(a['dir_world'])}"
                )
        for p in n["points"]:
            if "error" in p:
                lines.append(f"{pad}  cpoint: {p['name']} | ERROR: {p['error']}")
            else:
                v = "" if p["visible"] else " [hidden]"
                lines.append(
                    f"{pad}  cpoint: {p['name']}{v} "
                    f"| local pos_mm: {tuple(p['pos_mm'])} "
                    f"| world pos_mm: {tuple(p['pos_world_mm'])}"
                )
        if COLLECT_PHYSICS and "physics" in n and "error" not in n["physics"]:
            ph = n["physics"]
            lines.append(
                f"{pad}  mass: {ph['mass_kg']} kg | com local: {tuple(ph['com_mm'])} "
                f"| com world: {tuple(ph['com_world_mm'])}"
            )
        if n["children"]:
            lines += fmt_tree(n["children"], depth + 1)
    return lines


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def run(_context: str):
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("No active Fusion design.")
            return

        root = design.rootComponent

        os.makedirs(_SIM_DIR, exist_ok=True)
        json_path = os.path.join(_SIM_DIR, "fusion_export.json")

        # Preserve user edits to _servo_role_assignment across re-exports.
        preserved_roles = None
        if os.path.exists(json_path):
            try:
                with open(json_path, "r") as f:
                    prior = json.load(f)
                prior_manifest = prior.get("mesh_files") or {}
                preserved_roles = prior_manifest.get("_servo_role_assignment")
            except Exception:
                preserved_roles = None

        # Root component's frame IS world; pass identity.
        root_world = identity_matrix()

        # Traverse FIRST — export_stls's `combined` rule needs per-occurrence
        # world_transform_rm_cm values to bake vertices into world frame.
        occurrences_json = traverse(root.occurrences, root_world)

        exported, failed = export_stls(design, root, occurrences_json)
        mesh_files = build_mesh_files_manifest(exported, preserved_roles)

        export = {
            "document": root.name,
            "visible_only": VISIBLE_ONLY,
            "collect_physics": COLLECT_PHYSICS,
            "mesh_files": mesh_files,
            "root_axes": collect_axes(root, root_world),
            "root_points": collect_points(root, root_world),
            "occurrences": occurrences_json,
        }

        # JSON
        with open(json_path, "w") as f:
            json.dump(export, f, indent=2)

        # TXT
        txt_path = os.path.join(_SIM_DIR, "fusion_export.txt")
        header = [
            "=" * 60,
            "FaceHugger Fusion Export",
            f"Document   : {export['document']}",
            f"Visible only: {VISIBLE_ONLY}  |  Physics: {COLLECT_PHYSICS}",
            "=" * 60,
            "",
        ]
        if export["root_axes"] or export["root_points"]:
            header.append("[root]")
            for a in export["root_axes"]:  # ty:ignore[not-iterable]
                header.append(
                    f"  caxis: {a['name']} | origin_mm: {tuple(a['origin_mm'])} | dir: {tuple(a['dir'])}"
                )
            for p in export["root_points"]:  # ty:ignore[not-iterable]
                header.append(f"  cpoint: {p['name']} | pos_mm: {tuple(p['pos_mm'])}")
            header.append("")
        with open(txt_path, "w") as f:
            f.write("\n".join(header + fmt_tree(export["occurrences"])))

        # Summary
        msg = f"Export complete.\n\nJSON + TXT → {_SIM_DIR}\n"
        if exported:
            msg += f"\nSTLs exported ({len(exported)}):\n" + "\n".join(
                f"  {e['stl']}" for e in exported
            )
        if failed:
            msg += f"\n\nFailed ({len(failed)}):\n" + "\n".join(
                f"  {s}" for s in failed
            )
        ui.messageBox(msg)

    except:  # pylint: disable=bare-except  # noqa: E722
        app.log(f"Failed:\n{traceback.format_exc()}")
