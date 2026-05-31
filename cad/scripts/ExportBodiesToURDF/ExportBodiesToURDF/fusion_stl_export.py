# fusion_stl_export.py — Fusion-dependent STL orchestration.
# Imports adsk (unavoidable — calls exportManager and walks bRepBodies).
# The local-frame landmark finders (_find_landmark_pos_mm,
# _find_landmark_in_component) stay here because they walk the live Fusion
# root object, not the JSON.

import os

import adsk.fusion  # ty:ignore[unresolved-import]

from config import CM_TO_MM, EXPORT_RULES
from landmark_utils import (
    find_landmark_world_pos,
    find_landmark_world_pos_at_occurrence,
)
from math_utils import pt_mm
from stl_utils import (
    read_binary_stl,
    transform_triangles,
    translate_stl_in_place,
    write_binary_stl,
)


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
    """Search all construction points in the live Fusion tree by name.
    Returns local-frame position in mm, or None."""
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


def export_stls(design, root, occurrences_json, mesh_dir):
    """Execute EXPORT_RULES. Returns (exported_entries, failed_list).

    `occurrences_json` is the output of traverse(), needed so the `combined`
    rule can look up each part's world_transform_rm_cm.
    `mesh_dir` is the absolute path to the output directory.
    """
    os.makedirs(mesh_dir, exist_ok=True)
    mgr = design.exportManager
    exported = []
    failed = []

    tree = list(_iter_occ_tree(root.occurrences))

    for rule in EXPORT_RULES:
        stl_name = rule["stl"]
        filename = os.path.join(mesh_dir, stl_name)
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
