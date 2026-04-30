"""
ExportBodiesToURDF.py  —  FaceHugger Fusion assembly exporter

Design philosophy: **the export is the raw material, not the curated
output**. The JSON `occurrences` tree contains every occurrence in the
design — PCBs, OLEDs, capacitors, every screw — because downstream
tools (the URDF generator, the Blender visualizer) navigate that tree
by occurrence path to look up world transforms, joint origins, mount
points, etc. Pruning the tree would break path resolution. So we keep
it complete and filter at consumption time.

Filtering happens via four explicit whitelists, each shaping a
DIFFERENT slice of the output:

    EXPORT_RULES         bodies → STL files (8 files in exported_meshes/)
    CONSTRUCTION_POINTS  named cpoints land in each occurrence's points[]
    CONSTRUCTION_AXES    named caxes  land in each occurrence's axes[]
    JOINTS               named joints land in the top-level joints[] array

The OCCURRENCE TREE itself is not filtered: every component shows up,
with its full bodies[] (name + bbox center as metadata), its
whitelisted points[]/axes[], and its own children[]. This means
generate_urdf.py can resolve `FaceHuggerLegAssembly:1/MotorMount:1`
and find its world_transform_rm_cm without us having to anticipate
which paths it'll need.

Visibility (light-bulb state) in Fusion has NO effect on what gets
captured — toggle visibility however you want in the CAD browser, it's
purely a presentation concern. The JSON records each entity's
`visible` flag as informational metadata only.

Outputs to code/simulation/ (relative to this script's repo location):
  fusion_export.json     machine-readable, consumed by generate_urdf.py +
                         the Blender visualizer. Includes a `mesh_files`
                         manifest (one entry per STL with source occurrences
                         + re-origin info), the three whitelists, and a
                         `joints` array (axis, origin, limits, parent/child
                         for each whitelisted joint).
  fusion_export.txt      human-readable tree for sanity checking, with a
                         trailing `=== Joints ===` block.
  exported_meshes/*.stl  STLs driven by EXPORT_RULES (below). Link meshes
                         are re-origined so their local (0,0,0) coincides
                         with the URDF joint landmark (BodyToLink1Point,
                         Link1ToLink2Point, Link2ToLink3Point) — this
                         lets the URDF generator emit <visual><origin
                         xyz="0 0 0"/> for each link. Combined-rule STLs
                         (link + rigidly-attached servo) re-origin in
                         world frame around the same landmarks.

The post-export message box prints a checklist (✓/✗) verifying the
assembly matches code/simulation/docs/ASSEMBLY_HIERARCHY.md, so a CAD edit
that breaks an expected name surfaces immediately.

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

COLLECT_PHYSICS = True  # mass/CoM/inertia — slow, set False to skip
CM_TO_MM = 10.0
CM2_TO_M2 = 1e-4  # kg·cm² → kg·m²

# Leg-assembly normalization: the FaceHuggerLegAssembly:1 occurrence is
# placed in CAD with a non-identity world rotation (currently 90° about
# Z). Mesh vertices and per-joint axis_dir / axis_origin live in that
# rotated frame, which trips up the URDF generator. We capture that
# rotation as `R_la` and apply it to leg-assembly-internal joint data
# in collect_joints() so the JSON output is in a world-aligned frame.
# Mesh STLs are produced via combined-rule which already bakes
# world-frame vertices, so no extra mesh math is needed.
LEG_ASSEMBLY_OCCURRENCE = "FaceHuggerLegAssembly:1"
LEG_ASSEMBLY_COMPONENT = "FaceHuggerLegAssembly"

# ---------------------------------------------------------------------------
# Authoritative content whitelists. Visibility (light-bulb state) in Fusion
# has NO effect on what gets captured — the export is fully driven by these
# explicit lists. Toggle visibility however you want in CAD; it's a
# presentation concern, not a data filter.
#
# Each list matches by NAME (not occurrence path). When multiple components
# expose a same-named entity (e.g. LegMountFixedPoint in both MotorMount
# and MotorMountR; ServoMountPoint in 3 servo instances), every match is
# captured.
# ---------------------------------------------------------------------------

CONSTRUCTION_POINTS = [
    # Body-side mating points (FlexibleSkeleton:1).
    "LegMountPointFL", "LegMountPointFR", "LegMountPointBR", "LegMountPointBL",
    # Joint origins (FaceHuggerLegAssembly:1).
    "BodyToLink1Point", "Link1ToLink2Point", "Link2ToLink3Point",
    # Bracket-side mating point (MotorMount + MotorMountR).
    "LegMountFixedPoint",
    # Servo seat (each Servo_Mouser_Model occurrence).
    "ServoMountPoint",
]

CONSTRUCTION_AXES = [
    "BodyToLink1Axis", "Link1ToLink2Axis", "Link2ToLink3Axis",
]

JOINTS = [
    "Link1Revolute", "Link2Revolute", "Link3Revolute",
]

# Rules driving STL export. Three rule types supported:
#
#   {"type": "occurrence", "match": "FlexibleSkeleton:1", "stl": "X.stl"}
#     Export the full subtree of a matching occurrence as one STL. Fusion
#     bakes every descendant at its placement, so this is mostly useful
#     when you actually want everything in the subtree. Prefer `combined`
#     when you need a precise body subset.
#
#   {"type": "body", "match": "Link1", "stl": "X.stl",
#    "component": "Link1L",                 # optional, disambiguates same-named bodies
#    "origin_landmark": "BodyToLink1Point"} # optional
#     Export a single body by name. Without `component`, uses the FIRST
#     occurrence whose component contains a matching body. With `component`,
#     restricts to occurrences whose immediate parent component has that
#     name (e.g. `Link1L` body lives in `Link1L:1` occurrence — needed
#     because the R-side `Link1R` body shares the assembly with `Link1`).
#     If `origin_landmark` is set, STL vertices are translated so that
#     landmark's local-frame position becomes the mesh origin — downstream
#     URDF can then use <visual><origin xyz="0 0 0"/>.
#
#   {"type": "combined", "stl": "X.stl", "parts": [
#       {"occurrence": "A:1/B:1", "body": "BodyName"},
#       {"occurrence": "A:1/C:1", "body": "*"},  # "*" = first body in the occ
#   ], "origin_landmark": "BodyToLink1Point"}    # optional, world-frame re-origin
#     For each part: export just that body, then transform its vertices by
#     the occurrence's world_transform_rm_cm, then concatenate all parts
#     into one binary STL. Output vertices land in root/world frame. If
#     `origin_landmark` is set, the landmark's WORLD position is subtracted
#     from every vertex so the mesh's local origin coincides with the
#     landmark in world space — same effective convention as the body
#     rule's re-origin. Use when one URDF link's visual is several CAD
#     bodies (e.g. link1 + the hip servo body that's rigid with link1 per
#     `Link1RigidGroup`), or when you want a precise body subset for a
#     chassis STL without dragging in everything visible.
#
# Downstream consumers (URDF generator, Blender visualizer) read the
# `mesh_files` manifest section of fusion_export.json to know what was
# produced and how to place each mesh — they don't hardcode these rules.
#
# Per docs/ASSEMBLY_HIERARCHY.md and docs/PIPELINE_SPEC.md: shoulder servo
# (Servo_Mouser_Model:1) is chassis-fixed (bolted to the bracket); hip
# servo (Servo_Mouser_Model:2) is rigid with link1 via `Link1RigidGroup`;
# knee servo (Servo_Mouser_Model:3) is rigid with link3 via
# `Link3RigidGroup`.
EXPORT_RULES = [
    # Chassis: explicit body list. QuadrupedBody (main frame) + LipoCage
    # only — electronics (PCBs, OLED, MPU6050, …) are intentionally
    # excluded by NOT being in this list, regardless of CAD visibility.
    # Brackets (MotorMount{,R}) live in the leg assembly and are exported
    # separately as leg_mount_{L,R}.stl.
    {"type": "combined", "stl": "QuadrupedBody.stl", "parts": [
        {"occurrence": "FlexibleSkeleton:1/QuadrupedBody:1",
         "body": "QuadrupedBody"},
        {"occurrence": "FlexibleSkeleton:1/LipoCage:1",
         "body": "LipoCage"},
    ]},

    # Brackets. Combined-rule (single part each) so the output is in
    # world-frame vertices — the leg-assembly's CAD-local rotation gets
    # absorbed for free by `_transform_triangle`. `landmark_occurrence`
    # scopes the LegMountFixedPoint lookup to the right bracket; without
    # it the tree walk could pick the wrong one (both brackets share
    # the same landmark name).
    {"type": "combined", "stl": "leg_mount_L.stl",
     "origin_landmark": "LegMountFixedPoint",
     "landmark_occurrence": "FaceHuggerLegAssembly:1/MotorMount:1",
     "parts": [
         {"occurrence": "FaceHuggerLegAssembly:1/MotorMount:1",
          "body": "LegMountL"},
     ]},
    {"type": "combined", "stl": "leg_mount_R.stl",
     "origin_landmark": "LegMountFixedPoint",
     "landmark_occurrence": "FaceHuggerLegAssembly:1/MotorMountR:1",
     "parts": [
         {"occurrence": "FaceHuggerLegAssembly:1/MotorMountR:1",
          "body": "LegMountR"},
     ]},

    # Shoulder links — L and R variants. NO servo bake-in: the URDF
    # generator emits standalone servo visuals on each link from
    # servo.stl, with per-leg position/rpy. This avoids needing a R-side
    # servo in CAD and a per-rule mirror flag in the exporter.
    {"type": "combined", "stl": "leg_shoulder_L.stl",
     "origin_landmark": "BodyToLink1Point",
     "parts": [
         {"occurrence": "FaceHuggerLegAssembly:1/Link1L:1",
          "body": "Link1"},
     ]},
    {"type": "combined", "stl": "leg_shoulder_R.stl",
     "origin_landmark": "BodyToLink1Point",
     "parts": [
         {"occurrence": "FaceHuggerLegAssembly:1/Link1R:1",
          "body": "Link1R"},
     ]},

    # Upper / lower leg: shared (no mirror in CAD). The URDF generator
    # applies a (0, π, 0) visual rpy on the R-pair link2/link3 so the
    # mesh's knee/foot end up on the correct side.
    {"type": "combined", "stl": "leg_upper.stl",
     "origin_landmark": "Link1ToLink2Point",
     "parts": [
         {"occurrence": "FaceHuggerLegAssembly:1/Link2L:1",
          "body": "Link2"},
     ]},
    {"type": "combined", "stl": "leg_lower.stl",
     "origin_landmark": "Link2ToLink3Point",
     "parts": [
         {"occurrence": "FaceHuggerLegAssembly:1/Link3L:1",
          "body": "Link3"},
     ]},

    # Single shared servo mesh — instanced 12× by the URDF generator
    # (4 shoulder + 4 hip + 4 knee). Re-origined to ServoMountPoint so
    # the URDF can place each instance at its servo seat.
    {"type": "combined", "stl": "servo.stl",
     "origin_landmark": "ServoMountPoint",
     "parts": [
         {"occurrence": "FaceHuggerLegAssembly:1/Servo_Mouser_Model:1",
          "body": "ServoBase"},
     ]},
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
    """Every body in the component, with `visible` recorded as informational
    metadata only — visibility no longer filters output."""
    out = []
    for body in comp.bRepBodies:
        out.append({
            "name": body.name,
            "visible": body.isLightBulbOn,
            "bbox_center_mm": bbox_center_mm(body),
        })
    return out


def collect_axes(comp, world_transform):
    """Filtered to the names in CONSTRUCTION_AXES. `world_transform` is the
    4x4 matrix mapping this component's local frame into world frame.
    Axes are stored in BOTH local frame (`origin_mm`/`dir`) and world frame
    (`origin_world_mm`/`dir_world`) so downstream tools can pick whichever
    they need without re-walking the transform tree."""
    out = []
    for axis in comp.constructionAxes:
        if axis.name not in CONSTRUCTION_AXES:
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
    """Filtered to the names in CONSTRUCTION_POINTS."""
    out = []
    for point in comp.constructionPoints:
        if point.name not in CONSTRUCTION_POINTS:
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


def _find_landmark_in_component(component, landmark_name):
    """Look up a construction point by name in `component.constructionPoints`
    only (not descendants). Used by the body rule's re-origin step when a
    `component` filter is set, so a per-component landmark like
    `LegMountFixedPoint` (which exists once per bracket) resolves to the
    matching component's copy rather than picking the first occurrence in
    a tree-wide walk. Returns mm in component-local frame, or None.
    """
    for p in component.constructionPoints:
        if p.name == landmark_name:
            try:
                return pt_mm(p.geometry)
            except Exception:
                return None
    return None


def _find_landmark_world_pos_mm(occurrences_json, landmark_name):
    """Walk the JSON occurrence tree (already carries `pos_world_mm` for every
    construction point) and return the WORLD position of the first matching
    landmark. Used by the combined rule's re-origin step, where vertices are
    in world frame so the shift must also be world. Returns None if not found.
    """
    def walk(nodes):
        for n in nodes or []:
            for p in n.get("points") or []:
                if p.get("name") == landmark_name:
                    return p.get("pos_world_mm")
            r = walk(n.get("children"))
            if r is not None:
                return r
        return None
    return walk(occurrences_json)


def _find_landmark_world_pos_at_occurrence(occurrences_json, occ_path, landmark_name):
    """Like `_find_landmark_world_pos_mm`, but scope the lookup to a specific
    occurrence path (e.g. 'FaceHuggerLegAssembly:1/MotorMount:1'). Use this
    when a landmark name appears in multiple occurrences (e.g. both
    MotorMount:1 and MotorMountR:1 carry a `LegMountFixedPoint`) and you
    need the one belonging to a specific bracket. Returns None if the
    occurrence or landmark isn't found.
    """
    parts = occ_path.split("/")
    nodes = occurrences_json
    node = None
    for part in parts:
        node = next((n for n in (nodes or []) if n.get("name") == part), None)
        if node is None:
            return None
        nodes = node.get("children", [])
    if node is None:
        return None
    for p in node.get("points") or []:
        if p.get("name") == landmark_name:
            return p.get("pos_world_mm")
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
                component_filter = rule.get("component")
                # Every occurrence whose component contains a body with this
                # name. Used to populate `source_occurrences` so consumers
                # know all the places this STL is instanced (e.g. 3
                # Servo_Mouser_Model:N for one leg). With `component`
                # specified, restrict to occurrences whose immediate parent
                # component name matches — needed when two bodies share a
                # name across components (Link1 in Link1L vs Link1R in
                # Link1R won't collide here since the body names also
                # differ, but the pattern is general).
                matching_paths = []
                target_body = None
                target_occ = None
                for occ, path in tree:
                    if component_filter and occ.component.name != component_filter:
                        continue
                    body = _find_body_in_occ(occ, match)
                    if body is not None:
                        matching_paths.append(path)
                        if target_body is None:
                            target_body = body  # export the first one
                            target_occ = occ
                if target_body is None:
                    detail = f"no body named {match}"
                    if component_filter:
                        detail += f" in component {component_filter!r}"
                    failed.append(f"{stl_name}: {detail}")
                    continue
                _export_one(mgr, target_body, filename)
                # Re-origin step. When `component` is specified, prefer the
                # landmark inside that component (handles cases like
                # LegMountFixedPoint, which exists once per bracket
                # component); fall back to global tree-walk lookup.
                landmark = rule.get("origin_landmark")
                shift_mm = [0.0, 0.0, 0.0]
                if landmark:
                    pos = None
                    if target_occ is not None:
                        pos = _find_landmark_in_component(
                            target_occ.component, landmark
                        )
                    if pos is None:
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
                    "source_component": component_filter,
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
                # Optional re-origin: subtract the landmark's WORLD position
                # from every vertex so the resulting STL has its local
                # origin at the landmark in world space — same effective
                # convention as the body-rule re-origin (where the shift is
                # expressed in the body's local frame). Downstream URDF can
                # still use <visual><origin xyz="0 0 0"/>.
                #
                # `landmark_occurrence` (optional) scopes the landmark
                # lookup to a specific occurrence path. Needed when the
                # landmark name appears in multiple occurrences and the
                # tree-wide walk would pick the wrong one — e.g.
                # LegMountFixedPoint exists in both MotorMount:1 and
                # MotorMountR:1.
                landmark = rule.get("origin_landmark")
                landmark_occ = rule.get("landmark_occurrence")
                shift_mm = [0.0, 0.0, 0.0]
                if landmark:
                    if landmark_occ:
                        pos = _find_landmark_world_pos_at_occurrence(
                            occurrences_json, landmark_occ, landmark
                        )
                    else:
                        pos = _find_landmark_world_pos_mm(occurrences_json, landmark)
                    if pos is None:
                        scope = f" at {landmark_occ}" if landmark_occ else ""
                        failed.append(
                            f"{stl_name}: origin_landmark {landmark}{scope} "
                            f"not found in occurrence tree"
                        )
                    else:
                        sx, sy, sz = pos
                        shifted_tris = []
                        for tri in out_tris:
                            n, v0, v1, v2 = tri
                            shifted_tris.append((
                                n,
                                (v0[0] - sx, v0[1] - sy, v0[2] - sz),
                                (v1[0] - sx, v1[1] - sy, v1[2] - sz),
                                (v2[0] - sx, v2[1] - sy, v2[2] - sz),
                            ))
                        out_tris = shifted_tris
                        shift_mm = list(pos)
                _write_binary_stl(filename, out_tris)
                exported.append({
                    "stl": stl_name,
                    "source_type": "combined",
                    # Place at world origin — vertices are already in world frame
                    # (and re-origined to landmark world position if specified).
                    # Using FlexibleSkeleton:1 (which is at origin) so the Blender
                    # consumer's world_transform lookup yields identity.
                    "source_occurrences": ["FlexibleSkeleton:1"],
                    "parts": list(parts),
                    "origin_landmark": landmark,
                    "origin_shift_mm": shift_mm,
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
# Joint capture
# ---------------------------------------------------------------------------


def _find_leg_assembly_R_la(occurrences_json):
    """Find FaceHuggerLegAssembly:1 in the JSON occurrence tree and return
    its 3x3 world rotation matrix as a row-major list of lists. Returns
    None if not found or if the transform is missing.
    """
    for occ in occurrences_json or []:
        if occ.get("name") == LEG_ASSEMBLY_OCCURRENCE:
            wtf = occ.get("world_transform_rm_cm")
            if not wtf:
                return None
            return [
                [wtf[0][0], wtf[0][1], wtf[0][2]],
                [wtf[1][0], wtf[1][1], wtf[1][2]],
                [wtf[2][0], wtf[2][1], wtf[2][2]],
            ]
    return None


def _apply_R_3x3(R, v):
    """3x3 rotation applied to a 3-vector. R is row-major list-of-lists,
    v is a list/tuple of 3 floats. Returns a list of 3 floats. Tolerates
    None for either argument by returning the input unchanged.
    """
    if R is None or v is None:
        return v
    return [
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    ]


def collect_joints(design, R_la=None):
    """Walk every joint in the design (component-owned) and return a list of
    dicts describing each one. Schema per joint:

        {
          "name": "Link1Revolute",
          "owner_component": "FaceHuggerLegAssembly",
          "kind": "joint" | "asbuilt",         # which Fusion command made it
          "type": "revolute" | "rigid" | ... ,  # urdf-ish motion class
          "axis_dir_local_unit": [x, y, z] or None,
          "axis_origin_local_mm": [x, y, z] or None,
          "axis_construction_name": "BodyToLink1Axis" or None,
          "origin_construction_name": "BodyToLink1Point" or None,
          "limits_rad": {
              "rest": float,
              "min_enabled": bool, "min": float or None,
              "max_enabled": bool, "max": float or None,
          } or None,
          "parent_occurrence_path": "...:1/...:1" or None,
          "parent_body": str or None,
          "child_occurrence_path":  "...:1/...:1" or None,
          "child_body":  str or None,
        }

    Per the Fusion forum thread on joint discovery, `Component.joints`
    contains only joints created via the Joint command, while
    `Component.asBuiltJoints` carries those created via As-Built Joint —
    these are two separate collections, and a complete walk has to visit
    both. `Design.allComponents` enumerates every component definition in
    the design including xref'd documents (the FaceHuggerLegAssembly is
    an xref, and Link1Revolute / Link2Revolute / Link3Revolute live on
    that component's joints collection).

    Local frames are the OWNER COMPONENT's local frame. Defensive: every
    Fusion API call is wrapped in try/except so a single weird joint
    doesn't kill the whole export.

    Refs: https://forums.autodesk.com/t5/fusion-api-and-scripts/joints-where-are-you/td-p/8584358
          https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/JointSample_Sample.htm
          https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/AsBuiltJointSample_Sample.htm
    """
    joints_data = []
    try:
        all_components = design.allComponents
    except Exception:
        return joints_data

    for component in all_components:
        # Regular joints — created via the Joint command, with explicit
        # joint origins on each side.
        try:
            joints = component.joints
        except Exception:
            joints = None
        if joints:
            for joint in joints:
                if not _whitelisted_joint_name(joint):
                    continue
                entry = _extract_joint(joint, component, kind="joint", R_la=R_la)
                if entry is not None:
                    joints_data.append(entry)

        # As-built joints — created by selecting two faces/edges on
        # already-positioned occurrences. Same JointMotion subclasses,
        # but the geometry attribute is `.geometry` (a JointGeometry)
        # rather than `.geometryOrOriginOne`.
        try:
            as_built = component.asBuiltJoints
        except Exception:
            as_built = None
        if as_built:
            for joint in as_built:
                if not _whitelisted_joint_name(joint):
                    continue
                entry = _extract_joint(joint, component, kind="asbuilt", R_la=R_la)
                if entry is not None:
                    joints_data.append(entry)

    return joints_data


def _whitelisted_joint_name(joint):
    """Return True if `joint.name` is in the JOINTS whitelist. Defensive
    against AttributeError so a malformed joint can't crash the walk."""
    try:
        return joint.name in JOINTS
    except Exception:
        return False


def _extract_joint(joint, owner_component, kind="joint", R_la=None):
    """Pull data for one Fusion Joint or AsBuiltJoint into the JSON-friendly
    schema. `kind` distinguishes the two collections (Component.joints vs
    Component.asBuiltJoints). When `R_la` is provided AND this joint is
    owned by FaceHuggerLegAssembly, axis_dir_local_unit and
    axis_origin_local_mm are pre-rotated by R_la so downstream consumers
    see them in a world-aligned (normalized) frame. Returns None if the
    joint is malformed."""
    try:
        name = joint.name
    except Exception:
        return None

    motion = None
    motion_type = "unknown"
    try:
        motion = joint.jointMotion
        # objectType returns e.g. "adsk::fusion::RevoluteJointMotion"
        motion_type = motion.objectType.split("::")[-1] if motion else "unknown"
    except Exception:
        pass

    # Map Fusion motion class → URDF-ish type tag.
    if "Revolute" in motion_type:
        type_tag = "revolute"
    elif "Slider" in motion_type:
        type_tag = "prismatic"
    elif "Rigid" in motion_type:
        type_tag = "rigid"
    elif "Pin" in motion_type or "Cylindrical" in motion_type:
        type_tag = "cylindrical"
    elif "Ball" in motion_type:
        type_tag = "ball"
    else:
        type_tag = motion_type.lower()

    # Axis (revolute / prismatic only).
    axis_dir = None
    axis_construction_name = None
    if motion is not None and type_tag in ("revolute", "prismatic", "cylindrical"):
        axis_dir, axis_construction_name = _joint_axis(motion)

    # Origin point — comes from geometryOrOriginOne (regular Joint) or
    # geometry (AsBuiltJoint).
    origin_local_mm, origin_construction_name = _joint_origin(joint)

    # Limits — only meaningful for revolute / prismatic.
    limits_rad = None
    if motion is not None and type_tag in ("revolute", "prismatic"):
        limits_rad = _joint_limits(motion, type_tag)

    # Parent / child paths. Fusion's `occurrenceOne` is the *moving* side
    # (URDF child); `occurrenceTwo` is the *held* side (URDF parent). This
    # matches both fusion2urdf and the ACDC4Robot fork — getting it backwards
    # produces a URDF tree that's inverted relative to the CAD kinematics.
    parent_path = _safe_full_path(getattr(joint, "occurrenceTwo", None))
    child_path = _safe_full_path(getattr(joint, "occurrenceOne", None))
    parent_body = _first_body_name(getattr(joint, "occurrenceTwo", None))
    child_body = _first_body_name(getattr(joint, "occurrenceOne", None))

    owner_name = ""
    try:
        owner_name = owner_component.name or ""
    except Exception:
        pass

    # Normalization: rotate axis_dir and axis_origin by R_la so the JSON
    # values are in the world-aligned ("normalized") frame, when this
    # joint is owned by the leg-assembly. The mesh STLs are already
    # produced in world frame by the combined-rule export, so this
    # rotation is the only piece needed to make the URDF generator's
    # job a straight read.
    normalized = False
    if R_la is not None and owner_name == LEG_ASSEMBLY_COMPONENT:
        if axis_dir is not None:
            axis_dir = _apply_R_3x3(R_la, axis_dir)
        if origin_local_mm is not None:
            origin_local_mm = _apply_R_3x3(R_la, origin_local_mm)
        normalized = True

    return {
        "name": name,
        "owner_component": owner_name,
        "kind": kind,
        "type": type_tag,
        "axis_dir_local_unit": axis_dir,
        "axis_origin_local_mm": origin_local_mm,
        "axis_construction_name": axis_construction_name,
        "origin_construction_name": origin_construction_name,
        "limits_rad": limits_rad,
        "parent_occurrence_path": parent_path,
        "parent_body": parent_body,
        "child_occurrence_path": child_path,
        "child_body": child_body,
        "normalized_by_R_la": normalized,
    }


def _joint_axis(motion):
    """Return (axis_dir_unit, construction_axis_name) for a revolute/prismatic
    motion. Falls back to None if the axis isn't expressible as a unit vector
    (e.g., custom entity that's not a construction axis)."""
    # Standard X/Y/Z axes
    try:
        axis_kind = motion.rotationAxis  # may not exist on prismatic
    except AttributeError:
        try:
            axis_kind = motion.slideDirection
        except AttributeError:
            axis_kind = None
    except Exception:
        axis_kind = None

    # adsk.fusion.JointDirections enum values: X=0, Y=1, Z=2, Custom=3 (typical).
    # Don't import the enum — compare via the matching custom-entity getter.
    custom_entity = None
    try:
        custom_entity = motion.customRotationAxisEntity
    except AttributeError:
        try:
            custom_entity = motion.customSlideDirectionEntity
        except AttributeError:
            custom_entity = None
    except Exception:
        custom_entity = None

    if custom_entity is not None:
        # Construction axis: .geometry returns InfiniteLine3D with .direction.
        name = getattr(custom_entity, "name", None)
        try:
            geom = custom_entity.geometry  # InfiniteLine3D
            d = geom.direction             # Vector3D
            # Normalize defensively.
            mag = (d.x * d.x + d.y * d.y + d.z * d.z) ** 0.5
            if mag > 0:
                return [d.x / mag, d.y / mag, d.z / mag], name
        except Exception:
            return None, name
        return None, name

    # Fall back to principal axes (kind 0/1/2 → X/Y/Z).
    if axis_kind is None:
        return None, None
    try:
        kind_int = int(axis_kind)
    except Exception:
        return None, None
    if kind_int == 0:
        return [1.0, 0.0, 0.0], None
    if kind_int == 1:
        return [0.0, 1.0, 0.0], None
    if kind_int == 2:
        return [0.0, 0.0, 1.0], None
    return None, None


def _joint_origin(joint):
    """Return (origin_local_mm, construction_point_name) using the joint's
    held-side geometry reference. Regular Joints expose
    `geometryOrOriginOne` (either a JointGeometry or a JointOrigin);
    AsBuiltJoints expose `.geometry` (a JointGeometry). Both have `.origin`
    yielding a construction-point-like reference."""
    geo_one = None
    for attr in ("geometryOrOriginOne", "geometry"):
        try:
            geo_one = getattr(joint, attr, None)
        except Exception:
            geo_one = None
        if geo_one is not None:
            break
    if geo_one is None:
        return None, None

    name = None
    pos_mm = None
    try:
        # JointOrigin has .geometry (JointGeometry); JointGeometry has .origin.
        origin_entity = geo_one
        # Drill down to a construction-point-like object that exposes .geometry
        # returning a Point3D.
        if hasattr(origin_entity, "geometry"):
            inner = origin_entity.geometry
            if inner is not None and hasattr(inner, "origin"):
                origin_entity = inner.origin
        elif hasattr(origin_entity, "origin"):
            origin_entity = origin_entity.origin

        if hasattr(origin_entity, "name"):
            name = origin_entity.name or None

        # Try to pull a Point3D out.
        pt = None
        if hasattr(origin_entity, "geometry") and origin_entity.geometry is not None:
            g = origin_entity.geometry
            if hasattr(g, "x") and hasattr(g, "y") and hasattr(g, "z"):
                pt = g
        if pt is None and hasattr(origin_entity, "x") and hasattr(origin_entity, "y"):
            pt = origin_entity
        if pt is not None:
            pos_mm = [
                round(pt.x * CM_TO_MM, 3),
                round(pt.y * CM_TO_MM, 3),
                round(pt.z * CM_TO_MM, 3),
            ]
    except Exception:
        pass

    return pos_mm, name


def _joint_limits(motion, type_tag):
    """Return a `limits_rad` dict (radians) or None on failure."""
    try:
        if type_tag == "revolute":
            lim = motion.rotationLimits
            rest = motion.restValue if hasattr(motion, "restValue") else 0.0
        else:
            lim = motion.slideLimits
            rest = motion.restValue if hasattr(motion, "restValue") else 0.0
    except Exception:
        return None
    out = {"rest": float(rest)}
    try:
        out["min_enabled"] = bool(lim.isMinimumValueEnabled)
        out["min"] = float(lim.minimumValue) if lim.isMinimumValueEnabled else None
    except Exception:
        out["min_enabled"] = False
        out["min"] = None
    try:
        out["max_enabled"] = bool(lim.isMaximumValueEnabled)
        out["max"] = float(lim.maximumValue) if lim.isMaximumValueEnabled else None
    except Exception:
        out["max_enabled"] = False
        out["max"] = None
    return out


def _safe_full_path(occ):
    if occ is None:
        return None
    try:
        return occ.fullPathName
    except Exception:
        return None


def _first_body_name(occ):
    """Best-effort 'representative' body name for an occurrence — used to
    show parent/child geometry in the joints log without committing to any
    URDF link mapping (the URDF generator decides that from the rule
    structure + RigidGroups)."""
    if occ is None:
        return None
    try:
        bodies = occ.component.bRepBodies
        if bodies.count > 0:
            return bodies[0].name
    except Exception:
        pass
    return None


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
        # No visibility filter — every occurrence shows up in the JSON tree.
        # `visible` lands in the node as informational metadata for human
        # inspection, but is not used to gate anything.
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


def fmt_joints(joints):
    """Render the joints array as a human-readable block for fusion_export.txt.

    Strict-ASCII (no degree sign / em-dash) so the file write succeeds on
    Fusion's default encoding without us having to remember encoding="utf-8"
    at every call site.
    """
    if not joints:
        return ["=== Joints ===", "  (none)"]
    lines = ["=== Joints ==="]
    for j in joints:
        name = j.get("name", "?")
        type_tag = j.get("type", "?")
        kind = j.get("kind", "joint")
        kind_suffix = "" if kind == "joint" else f" [{kind}]"
        lines.append(f"{name} ({type_tag}){kind_suffix}")

        axis_name = j.get("axis_construction_name") or "(principal axis)"
        origin_name = j.get("origin_construction_name") or "(implicit origin)"
        axis_dir = j.get("axis_dir_local_unit")
        origin = j.get("axis_origin_local_mm")
        bits = [f"axis: {axis_name}"]
        if axis_dir is not None:
            bits.append(
                f"dir=({axis_dir[0]:+.3f}, {axis_dir[1]:+.3f}, {axis_dir[2]:+.3f})"
            )
        bits.append(f"origin: {origin_name}")
        if origin is not None:
            bits.append(
                f"local_mm=({origin[0]:+.2f}, {origin[1]:+.2f}, {origin[2]:+.2f})"
            )
        lines.append("  " + "  ".join(bits))

        lim = j.get("limits_rad") or {}
        if lim:
            min_d = (
                f"{_rad_to_deg(lim['min']):+.1f}deg" if lim.get("min") is not None else "--"
            )
            max_d = (
                f"{_rad_to_deg(lim['max']):+.1f}deg" if lim.get("max") is not None else "--"
            )
            rest = lim.get("rest", 0.0)
            rest_d = _rad_to_deg(rest) if rest is not None else 0.0
            lines.append(f"  limits: rest={rest_d:+.1f}deg  [{min_d}, {max_d}]")

        parent = j.get("parent_occurrence_path") or "?"
        child = j.get("child_occurrence_path") or "?"
        parent_body = j.get("parent_body") or "?"
        child_body = j.get("child_body") or "?"
        lines.append(f"  parent: {parent} ({parent_body})")
        lines.append(f"  child:  {child} ({child_body})")
    return lines


def _summarize_joint(j):
    """One-line summary used in the post-export message box. Pure ASCII for
    the same reason as fmt_joints — Fusion's UI is fine with unicode but
    we don't pay to keep both code paths consistent."""
    name = j.get("name", "?")
    type_tag = j.get("type", "?")
    lim = j.get("limits_rad") or {}
    range_txt = ""
    if lim and lim.get("min") is not None and lim.get("max") is not None:
        range_txt = (
            f"  [{_rad_to_deg(lim['min']):+.0f}deg, "
            f"{_rad_to_deg(lim['max']):+.0f}deg]"
        )
    axis_name = j.get("axis_construction_name") or "?"
    return f"  {name} ({type_tag}, axis={axis_name}){range_txt}"


def _rad_to_deg(rad):
    import math
    try:
        return math.degrees(float(rad))
    except Exception:
        return 0.0


# A re-export immediately tells the user whether the CAD still matches
# ASSEMBLY_HIERARCHY.md.
_REQUIRED_FS_POINTS = (
    "LegMountPointFL", "LegMountPointFR", "LegMountPointBR", "LegMountPointBL",
)
_REQUIRED_FHLA_POINTS = (
    "BodyToLink1Point", "Link1ToLink2Point", "Link2ToLink3Point",
)
_REQUIRED_FHLA_OCCS = {
    "Link1L:1":              {"body": "Link1"},
    "Link1R:1":              {"body": "Link1R"},
    "Link2L:1":              {"body": "Link2"},
    "Link3L:1":              {"body": "Link3"},
    "MotorMount:1":          {"body": "LegMountL", "point": "LegMountFixedPoint"},
    "MotorMountR:1":         {"body": "LegMountR", "point": "LegMountFixedPoint"},
    "Servo_Mouser_Model:1":  {"body": "ServoBase"},
    "Servo_Mouser_Model:2":  {"body": "ServoBase"},
    "Servo_Mouser_Model:3":  {"body": "ServoBase"},
}
_REQUIRED_JOINTS = ("Link1Revolute", "Link2Revolute", "Link3Revolute")


def _verify_against_assembly_hierarchy(occurrences_json, joints, mesh_files):
    """Lightweight diagnostic against ASSEMBLY_HIERARCHY.md. Returns a list of
    human-readable lines (a few ✓/✗ rows) for the message box. Doesn't fail
    the export — just reports.

    `mesh_files` is consulted as a secondary existence check: if an
    occurrence shows up in ANY entry's `source_occurrences`, the export
    rules also found it. With the visibility filter removed (every
    occurrence lands in the JSON tree), this fallback is mostly
    redundant — kept defensively in case a future export rule references
    a path that traverse() decides to skip for an unrelated reason."""
    fs = next(
        (o for o in occurrences_json if o.get("name") == "FlexibleSkeleton:1"),
        None,
    )
    fhla = next(
        (o for o in occurrences_json
         if o.get("name") == "FaceHuggerLegAssembly:1"),
        None,
    )

    def _has_point(node, name):
        return any(p.get("name") == name for p in (node.get("points") or []))

    def _has_body(node, body_name):
        bodies = node.get("bodies") or []
        return any(
            (b.get("name") if isinstance(b, dict) else b) == body_name
            for b in bodies
        )

    def _mark(ok):
        return "✓" if ok else "✗"

    rows = []

    # 1. FlexibleSkeleton points
    fs_ok = bool(fs) and all(_has_point(fs, n) for n in _REQUIRED_FS_POINTS)
    rows.append(f"  {_mark(fs_ok)} FlexibleSkeleton points "
                f"({', '.join(_REQUIRED_FS_POINTS)})")

    # 2. FaceHuggerLegAssembly points
    fhla_pts_ok = (
        bool(fhla) and all(_has_point(fhla, n) for n in _REQUIRED_FHLA_POINTS)
    )
    rows.append(f"  {_mark(fhla_pts_ok)} FaceHuggerLegAssembly points "
                f"({', '.join(_REQUIRED_FHLA_POINTS)})")

    # 3. Bracket alignment cross-check
    cross_ok = False
    if fs and fhla:
        fs_pts = {p.get("name"): p for p in (fs.get("points") or [])}
        fl = fs_pts.get("LegMountPointFL")
        mm = next(
            (c for c in (fhla.get("children") or [])
             if c.get("name") == "MotorMount:1"),
            None,
        )
        if fl and mm:
            fp = next(
                (p for p in (mm.get("points") or [])
                 if p.get("name") == "LegMountFixedPoint"),
                None,
            )
            if fp:
                a = fl.get("pos_world_mm") or [0, 0, 0]
                b = fp.get("pos_world_mm") or [0, 0, 0]
                d = sum((a[i] - b[i]) ** 2 for i in range(3)) ** 0.5
                cross_ok = d <= 1.0
    rows.append(f"  {_mark(cross_ok)} MotorMount LegMountFixedPoint "
                "≈ LegMountPointFL (≤ 1mm)")

    # 4. FaceHuggerLegAssembly occurrences + bodies. Visibility-tolerant:
    # an occurrence counts as "found" if it's either in the (visibility-
    # filtered) JSON tree OR mentioned in any mesh_files entry's
    # source_occurrences (which the export rules populate from the
    # unfiltered live tree). Construction points only resolve through
    # the JSON tree, so they're checked separately and surface as a
    # softer "info" row when the parent occurrence is hidden.
    occ_ok = bool(fhla)
    point_warnings = []
    if fhla:
        children = {c.get("name"): c for c in (fhla.get("children") or [])}
        seen_paths = set()
        for entry in (mesh_files or {}).values():
            if not isinstance(entry, dict):
                continue
            for p in entry.get("source_occurrences") or []:
                seen_paths.add(p)
                # Also add the leaf component:N for matching by occurrence
                # name (covers e.g. "FaceHuggerLegAssembly:1/MotorMountR:1"
                # → "MotorMountR:1").
                seen_paths.add(p.rsplit("/", 1)[-1])
        for occ_name, expect in _REQUIRED_FHLA_OCCS.items():
            child = children.get(occ_name)
            in_mesh = occ_name in seen_paths
            if not child and not in_mesh:
                occ_ok = False
                break
            # Body presence is implied when the occurrence shows up in
            # mesh_files (the rule already matched against it). When the
            # JSON tree has the occurrence, double-check the body name.
            if child and "body" in expect and not _has_body(child, expect["body"]):
                occ_ok = False
                break
            # Construction point only resolves via the JSON tree; if the
            # occurrence is hidden, we can't see its points — flag as a
            # warning rather than a hard fail.
            if "point" in expect:
                if child:
                    if not _has_point(child, expect["point"]):
                        occ_ok = False
                        break
                else:
                    point_warnings.append(
                        f"{occ_name} is hidden — can't verify {expect['point']}"
                    )
    rows.append(f"  {_mark(occ_ok)} LegAssembly occurrences + bodies "
                "(via JSON tree or mesh_files)")
    for w in point_warnings:
        rows.append(f"    note: {w}")

    # 5. Joints
    joint_names = {j.get("name") for j in joints}
    j_ok = all(n in joint_names for n in _REQUIRED_JOINTS)
    rows.append(f"  {_mark(j_ok)} Joints array "
                f"({', '.join(_REQUIRED_JOINTS)})")

    return rows


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
                with open(json_path, "r", encoding="utf-8") as f:
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

        # Leg-assembly normalization rotation. The leg-assembly is placed
        # in CAD with a non-identity world rotation; we rotate joint
        # axis_dir / axis_origin by this matrix so the JSON values are in
        # a world-aligned frame. Fail loudly if missing — downstream URDF
        # generation depends on it.
        R_la = _find_leg_assembly_R_la(occurrences_json)
        if R_la is None:
            raise RuntimeError(
                f"Could not find {LEG_ASSEMBLY_OCCURRENCE} world transform; "
                "leg-assembly normalization cannot proceed."
            )

        # Capture Fusion joints (axis, origin, limits, parent/child). The
        # URDF generator prefers this over yaml when present; downstream
        # readers ignore the array if they don't know about it.
        joints = collect_joints(design, R_la=R_la)

        # Bind to typed locals so the TXT iteration below doesn't fight the
        # type checker over `export[...]` heterogeneity.
        root_axes = collect_axes(root, root_world)
        root_points = collect_points(root, root_world)

        export = {
            "document": root.name,
            "collect_physics": COLLECT_PHYSICS,
            "mesh_files": mesh_files,
            "construction_points_whitelist": list(CONSTRUCTION_POINTS),
            "construction_axes_whitelist": list(CONSTRUCTION_AXES),
            "joints_whitelist": list(JOINTS),
            "leg_assembly_normalization": {
                "applied": True,
                "occurrence": LEG_ASSEMBLY_OCCURRENCE,
                "owner_component": LEG_ASSEMBLY_COMPONENT,
                "rotation_3x3_row_major": R_la,
            },
            "joints": joints,
            "root_axes": root_axes,
            "root_points": root_points,
            "occurrences": occurrences_json,
        }

        # JSON. Explicit UTF-8 so any unicode in CAD names / point names
        # doesn't get mangled by Fusion's platform-default encoding.
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(export, f, indent=2)

        # TXT
        txt_path = os.path.join(_SIM_DIR, "fusion_export.txt")
        header = [
            "=" * 60,
            "FaceHugger Fusion Export",
            f"Document        : {export['document']}",
            f"Physics         : {COLLECT_PHYSICS}",
            f"Cpoint whitelist: {', '.join(CONSTRUCTION_POINTS)}",
            f"Caxis whitelist : {', '.join(CONSTRUCTION_AXES)}",
            f"Joint whitelist : {', '.join(JOINTS)}",
            "=" * 60,
            "",
            "[Leg-assembly normalization]",
            f"  Owner: {LEG_ASSEMBLY_COMPONENT}",
            "  R_la (3x3, row-major):",
            f"    [{R_la[0][0]:+.4f}, {R_la[0][1]:+.4f}, {R_la[0][2]:+.4f}]",
            f"    [{R_la[1][0]:+.4f}, {R_la[1][1]:+.4f}, {R_la[1][2]:+.4f}]",
            f"    [{R_la[2][0]:+.4f}, {R_la[2][1]:+.4f}, {R_la[2][2]:+.4f}]",
            "  Applied to: joint axis_dir_local_unit and "
            "axis_origin_local_mm (leg-assembly-internal joints only).",
            "  Mesh STLs are produced by combined-rule, which already bakes "
            "world-frame vertices.",
            "",
        ]
        if root_axes or root_points:
            header.append("[root]")
            for a in root_axes:
                header.append(
                    f"  caxis: {a['name']} | origin_mm: {tuple(a['origin_mm'])} | dir: {tuple(a['dir'])}"
                )
            for p in root_points:
                header.append(f"  cpoint: {p['name']} | pos_mm: {tuple(p['pos_mm'])}")
            header.append("")

        joints_lines = fmt_joints(joints)

        # Explicit UTF-8 so the joints/diagnostic blocks and any unicode in
        # CAD names / construction-point names don't blow up on a default
        # cp1252-style write encoding inside Fusion.
        with open(txt_path, "w", encoding="utf-8") as f:
            f.write("\n".join(
                header + fmt_tree(export["occurrences"]) + [""] + joints_lines
            ))

        # Diagnostic against ASSEMBLY_HIERARCHY checklist. Lets the user see
        # immediately whether the export lines up with the spec.
        diag_lines = _verify_against_assembly_hierarchy(
            occurrences_json, joints, mesh_files
        )

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
        if joints:
            msg += f"\n\nJoints captured ({len(joints)}):\n" + "\n".join(
                _summarize_joint(j) for j in joints
            )
        if diag_lines:
            msg += "\n\nAssembly check:\n" + "\n".join(diag_lines)
        ui.messageBox(msg)

    except:  # pylint: disable=bare-except  # noqa: E722
        app.log(f"Failed:\n{traceback.format_exc()}")
