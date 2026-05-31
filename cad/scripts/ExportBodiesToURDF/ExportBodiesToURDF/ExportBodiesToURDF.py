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
import traceback

import adsk.core  # ty:ignore[unresolved-import]
import adsk.fusion  # ty:ignore[unresolved-import]

from config import (
    CM2_TO_M2,
    COLLECT_PHYSICS,
    CONSTRUCTION_AXES,
    CONSTRUCTION_POINTS,
    CM_TO_MM,
    EXPORT_RULES,
    JOINTS,
    LEG_ASSEMBLY_COMPONENT,
    LEG_ASSEMBLY_OCCURRENCE,
)
from math_utils import (
    _apply_R_3x3,
    _rad_to_deg,
    bbox_center_mm,
    dir_world,
    mat_multiply_arrays,
    matrix_as_row_major_cm,
    matrix_translation_mm,
    point_world_mm,
    pt_mm,
    vec3,
)
from stl_utils import read_binary_stl, translate_stl_in_place, write_binary_stl
from landmark_utils import (
    find_landmark_world_pos,
    find_landmark_world_pos_at_occurrence,
)
from manifest import build_mesh_files_manifest
from verify import verify_against_assembly_hierarchy

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
_GENERATED_DIR = os.path.join(_SIM_DIR, "generated")
_MESH_DIR = os.path.join(_GENERATED_DIR, "exported_meshes")


# ---------------------------------------------------------------------------
# adsk-aware matrix helpers (stay here — only callers are in this file)
# ---------------------------------------------------------------------------


def mat_multiply(parent, local):
    """Return a new adsk.core.Matrix3D = parent * local."""
    out = mat_multiply_arrays(parent.asArray(), local.asArray())
    result = adsk.core.Matrix3D.create()
    result.setWithArray(out)
    return result


def identity_matrix():
    return adsk.core.Matrix3D.create()


def occ_parent_origin_mm(occ):
    """Translation of `occ.transform`, in mm (parent-relative)."""
    t = occ.transform
    return [
        round(t.translation.x * CM_TO_MM, 3),
        round(t.translation.y * CM_TO_MM, 3),
        round(t.translation.z * CM_TO_MM, 3),
    ]


# ---------------------------------------------------------------------------
# Collectors
# ---------------------------------------------------------------------------


def collect_bodies(comp):
    """Every body in the component, with `visible` as informational metadata."""
    out = []
    for body in comp.bRepBodies:
        out.append(
            {
                "name": body.name,
                "visible": body.isLightBulbOn,
                "bbox_center_mm": bbox_center_mm(body),
            }
        )
    return out


def collect_axes(comp, world_transform):
    """Filtered to names in CONSTRUCTION_AXES. Stored in local and world frame."""
    out = []
    for axis in comp.constructionAxes:
        if axis.name not in CONSTRUCTION_AXES:
            continue
        try:
            geom = axis.geometry  # Line3D
            out.append(
                {
                    "name": axis.name,
                    "visible": axis.isLightBulbOn,
                    "origin_mm": pt_mm(geom.origin),
                    "dir": vec3(geom.direction),
                    "origin_world_mm": point_world_mm(geom.origin, world_transform),
                    "dir_world": dir_world(geom.direction, world_transform),
                }
            )
        except Exception as e:
            out.append({"name": axis.name, "error": str(e)})
    return out


def collect_points(comp, world_transform):
    """Filtered to names in CONSTRUCTION_POINTS."""
    out = []
    for point in comp.constructionPoints:
        if point.name not in CONSTRUCTION_POINTS:
            continue
        try:
            geom = point.geometry
            out.append(
                {
                    "name": point.name,
                    "visible": point.isLightBulbOn,
                    "pos_mm": pt_mm(geom),
                    "pos_world_mm": point_world_mm(geom, world_transform),
                }
            )
        except Exception as e:
            out.append({"name": point.name, "error": str(e)})
    return out


def collect_physics(occ, parent_to_world):
    """Report CoM in both parent frame (com_mm) and world (com_world_mm).

    `Occurrence.getPhysicalProperties().centerOfMass` is in the PARENT frame,
    so we apply `parent_to_world`, NOT `this_to_world`.
    """
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


def _iter_occ_tree(occurrences):
    """Yield (occ, path) for every occurrence reachable from `occurrences`, depth-first."""
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
    """Return (live_occ, world_transform_rm_cm) for the occurrence at `path`."""
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
    if node is None:
        return live_occ, None
    return live_occ, node.get("world_transform_rm_cm")


def _find_landmark_pos_mm(root, landmark_name):
    """Search all construction points in the tree by name; return local-frame mm."""
    for p in root.constructionPoints:
        if p.name == landmark_name:
            try:
                return pt_mm(p.geometry)
            except Exception:
                return None
    for occ, _ in _iter_occ_tree(root.occurrences):
        for p in occ.component.constructionPoints:
            if p.name == landmark_name:
                try:
                    return pt_mm(p.geometry)
                except Exception:
                    return None
    return None


def _find_landmark_in_component(component, landmark_name):
    """Look up a construction point by name in `component.constructionPoints` only."""
    for p in component.constructionPoints:
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
    """Execute EXPORT_RULES. Returns (exported_entries, failed_list)."""
    os.makedirs(_MESH_DIR, exist_ok=True)
    mgr = design.exportManager
    exported = []
    failed = []

    tree = list(_iter_occ_tree(root.occurrences))

    for rule in EXPORT_RULES:
        stl_name = rule["stl"]
        filename = os.path.join(_MESH_DIR, stl_name)
        rtype = rule["type"]
        try:
            if rtype == "occurrence":
                match = rule["match"]
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
                exported.append(
                    {
                        "stl": stl_name,
                        "source_type": "occurrence",
                        "source_occurrences": [source_path],
                        "origin_landmark": None,
                        "origin_shift_mm": [0.0, 0.0, 0.0],
                    }
                )
            elif rtype == "body":
                match = rule["match"]
                component_filter = rule.get("component")
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
                            target_body = body
                            target_occ = occ
                if target_body is None:
                    detail = f"no body named {match}"
                    if component_filter:
                        detail += f" in component {component_filter!r}"
                    failed.append(f"{stl_name}: {detail}")
                    continue
                _export_one(mgr, target_body, filename)
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
                    else:
                        translate_stl_in_place(filename, pos)
                        shift_mm = pos
                exported.append(
                    {
                        "stl": stl_name,
                        "source_type": "body",
                        "source_body": match,
                        "source_component": component_filter,
                        "source_occurrences": matching_paths,
                        "origin_landmark": landmark,
                        "origin_shift_mm": shift_mm,
                    }
                )
            elif rtype == "combined":
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
                    tmp_path = filename + f".part{i}.stl"
                    _export_one(mgr, body, tmp_path)
                    try:
                        part_tris = read_binary_stl(tmp_path)
                    finally:
                        try:
                            os.remove(tmp_path)
                        except OSError:
                            pass
                    rot = [[wtf_cm[r][c] for c in range(3)] for r in range(3)]
                    tx_mm = [wtf_cm[r][3] * CM_TO_MM for r in range(3)]
                    from stl_utils import transform_triangles

                    out_tris.extend(transform_triangles(part_tris, rot, tx_mm))
                if fatal:
                    continue
                landmark = rule.get("origin_landmark")
                landmark_occ = rule.get("landmark_occurrence")
                shift_mm = [0.0, 0.0, 0.0]
                if landmark:
                    if landmark_occ:
                        pos = find_landmark_world_pos_at_occurrence(
                            occurrences_json, landmark_occ, landmark
                        )
                    else:
                        pos = find_landmark_world_pos(occurrences_json, landmark)
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
                            shifted_tris.append(
                                (
                                    n,
                                    (v0[0] - sx, v0[1] - sy, v0[2] - sz),
                                    (v1[0] - sx, v1[1] - sy, v1[2] - sz),
                                    (v2[0] - sx, v2[1] - sy, v2[2] - sz),
                                )
                            )
                        out_tris = shifted_tris
                        shift_mm = list(pos)
                write_binary_stl(filename, out_tris)
                exported.append(
                    {
                        "stl": stl_name,
                        "source_type": "combined",
                        "source_occurrences": ["FlexibleSkeleton:1"],
                        "parts": list(parts),
                        "origin_landmark": landmark,
                        "origin_shift_mm": shift_mm,
                    }
                )
            else:
                failed.append(f"{stl_name}: unknown rule type {rtype!r}")
        except Exception as e:
            failed.append(f"{stl_name}: {e}")

    return exported, failed


# ---------------------------------------------------------------------------
# Joint capture
# ---------------------------------------------------------------------------


def _find_leg_assembly_R_la(occurrences_json):
    """Return 3x3 world rotation of FaceHuggerLegAssembly:1, or None."""
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


def collect_joints(design, R_la=None):
    """Walk every joint in the design and return a list of dicts."""
    joints_data = []
    try:
        all_components = design.allComponents
    except Exception:
        return joints_data

    for component in all_components:
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
    try:
        return joint.name in JOINTS
    except Exception:
        return False


def _extract_joint(joint, owner_component, kind="joint", R_la=None):
    """Pull data for one Fusion Joint or AsBuiltJoint into JSON-friendly schema."""
    try:
        name = joint.name
    except Exception:
        return None

    motion = None
    motion_type = "unknown"
    try:
        motion = joint.jointMotion
        motion_type = motion.objectType.split("::")[-1] if motion else "unknown"
    except Exception:
        pass

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

    axis_dir = None
    axis_construction_name = None
    if motion is not None and type_tag in ("revolute", "prismatic", "cylindrical"):
        axis_dir, axis_construction_name = _joint_axis(motion)

    origin_local_mm, origin_construction_name = _joint_origin(joint)

    limits_rad = None
    if motion is not None and type_tag in ("revolute", "prismatic"):
        limits_rad = _joint_limits(motion, type_tag)

    parent_path = _safe_full_path(getattr(joint, "occurrenceTwo", None))
    child_path = _safe_full_path(getattr(joint, "occurrenceOne", None))
    parent_body = _first_body_name(getattr(joint, "occurrenceTwo", None))
    child_body = _first_body_name(getattr(joint, "occurrenceOne", None))

    owner_name = ""
    try:
        owner_name = owner_component.name or ""
    except Exception:
        pass

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
    """Return (axis_dir_unit, construction_axis_name) for a revolute/prismatic motion."""
    try:
        axis_kind = motion.rotationAxis
    except AttributeError:
        try:
            axis_kind = motion.slideDirection
        except AttributeError:
            axis_kind = None
    except Exception:
        axis_kind = None

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
        name = getattr(custom_entity, "name", None)
        try:
            geom = custom_entity.geometry
            d = geom.direction
            mag = (d.x * d.x + d.y * d.y + d.z * d.z) ** 0.5
            if mag > 0:
                return [d.x / mag, d.y / mag, d.z / mag], name
        except Exception:
            return None, name
        return None, name

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
    """Return (origin_local_mm, construction_point_name)."""
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
        origin_entity = geo_one
        if hasattr(origin_entity, "geometry"):
            inner = origin_entity.geometry
            if inner is not None and hasattr(inner, "origin"):
                origin_entity = inner.origin
        elif hasattr(origin_entity, "origin"):
            origin_entity = origin_entity.origin

        if hasattr(origin_entity, "name"):
            name = origin_entity.name or None

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
        else:
            lim = motion.slideLimits
        rest = lim.restValue if hasattr(lim, "restValue") else 0.0
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
    """Recursive walk that accumulates the world transform."""
    if parent_to_world is None:
        parent_to_world = identity_matrix()
    result = []
    for occ in occurrences:
        comp = occ.component
        this_to_world = mat_multiply(parent_to_world, occ.transform)
        node = {
            "name": occ.name,
            "xref": occ.isReferencedComponent,
            "visible": occ.isLightBulbOn,
            "parent_origin_mm": occ_parent_origin_mm(occ),
            "world_origin_mm": matrix_translation_mm(this_to_world),
            "world_transform_rm_cm": matrix_as_row_major_cm(this_to_world),
            "bodies": collect_bodies(comp),
            "axes": collect_axes(comp, this_to_world),
            "points": collect_points(comp, this_to_world),
            "children": [],
        }
        if COLLECT_PHYSICS:
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
    """Render the joints array as a human-readable block for fusion_export.txt."""
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
                f"{_rad_to_deg(lim['min']):+.1f}deg"
                if lim.get("min") is not None
                else "--"
            )
            max_d = (
                f"{_rad_to_deg(lim['max']):+.1f}deg"
                if lim.get("max") is not None
                else "--"
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
    """One-line summary for the post-export message box."""
    name = j.get("name", "?")
    type_tag = j.get("type", "?")
    lim = j.get("limits_rad") or {}
    range_txt = ""
    if lim and lim.get("min") is not None and lim.get("max") is not None:
        range_txt = (
            f"  [{_rad_to_deg(lim['min']):+.0f}deg, {_rad_to_deg(lim['max']):+.0f}deg]"
        )
    axis_name = j.get("axis_construction_name") or "?"
    return f"  {name} ({type_tag}, axis={axis_name}){range_txt}"


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

        os.makedirs(_GENERATED_DIR, exist_ok=True)
        json_path = os.path.join(_GENERATED_DIR, "fusion_export.json")

        preserved_roles = None
        if os.path.exists(json_path):
            try:
                with open(json_path, "r", encoding="utf-8") as f:
                    prior = json.load(f)
                prior_manifest = prior.get("mesh_files") or {}
                preserved_roles = prior_manifest.get("_servo_role_assignment")
            except Exception:
                preserved_roles = None

        root_world = identity_matrix()

        occurrences_json = traverse(root.occurrences, root_world)

        exported, failed = export_stls(design, root, occurrences_json)
        mesh_files = build_mesh_files_manifest(exported, preserved_roles)

        R_la = _find_leg_assembly_R_la(occurrences_json)
        if R_la is None:
            raise RuntimeError(
                f"Could not find {LEG_ASSEMBLY_OCCURRENCE} world transform; "
                "leg-assembly normalization cannot proceed."
            )

        joints = collect_joints(design, R_la=R_la)

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

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(export, f, indent=2)

        txt_path = os.path.join(_GENERATED_DIR, "fusion_export.txt")
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

        with open(txt_path, "w", encoding="utf-8") as f:
            f.write(
                "\n".join(
                    header + fmt_tree(export["occurrences"]) + [""] + joints_lines
                )
            )

        diag_lines = verify_against_assembly_hierarchy(
            occurrences_json, joints, mesh_files
        )

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
