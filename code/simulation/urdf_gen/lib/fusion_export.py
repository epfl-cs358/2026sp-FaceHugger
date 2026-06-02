"""Helpers that walk the Fusion `fusion_export.json` occurrence tree.

No URDF math, no XML — pure dict-and-list traversal. The export schema is
defined by the `ExportBodiesToURDF` Fusion add-in
(`cad/scripts/FaceHuggerExport/`).
"""

import json
from pathlib import Path


def load_export(path: Path) -> dict:
    with open(path) as f:
        return json.load(f)


def find_occurrence(nodes: list, name: str) -> dict | None:
    """Recursively find an occurrence by name in the export tree."""
    for n in nodes:
        if n["name"] == name:
            return n
        found = find_occurrence(n.get("children", []), name)
        if found:
            return found
    return None


def find_axis(occ: dict, key: str) -> dict | None:
    return next((a for a in occ.get("axes", []) if a["name"] == key), None)


def find_point(occ: dict, key: str) -> dict | None:
    return next((p for p in occ.get("points", []) if p["name"] == key), None)


def find_point_in_tree(nodes: list, key: str) -> list | None:
    """Find a named construction point anywhere in the tree, return pos_mm."""
    for n in nodes:
        for p in n.get("points", []):
            if p["name"] == key:
                return p["pos_mm"]
        result = find_point_in_tree(n.get("children", []), key)
        if result:
            return result
    return None


def find_point_world_in_tree(nodes: list, key: str) -> list | None:
    """Find a named construction point anywhere in the tree, return its
    `pos_world_mm` (world position) instead of the component-local
    `pos_mm`. The first match wins — caller should use
    `find_point_world_at_occurrence` when the name is ambiguous (e.g.
    LegMountFixedPoint exists in both MotorMount:1 and MotorMountR:1)."""
    for n in nodes:
        for p in n.get("points", []):
            if p["name"] == key:
                return p.get("pos_world_mm")
        result = find_point_world_in_tree(n.get("children", []), key)
        if result is not None:
            return result
    return None


def find_point_world_at_occurrence(nodes: list, occ_path: str, key: str) -> list | None:
    """Look up a construction point's `pos_world_mm` scoped to a specific
    occurrence path (e.g. 'FaceHuggerLegAssembly:1/MotorMount:1'). Use
    this when a point name appears multiple times in the tree — both
    bracket components carry a `LegMountFixedPoint` and the unscoped
    walk would pick whichever comes first."""
    parts = occ_path.split("/")
    cur = nodes
    node = None
    for part in parts:
        node = next((c for c in (cur or []) if c.get("name") == part), None)
        if node is None:
            return None
        cur = node.get("children", [])
    if node is None:
        return None
    for p in node.get("points", []):
        if p.get("name") == key:
            return p.get("pos_world_mm")
    return None


def _find_occ_by_path(occs: list, path: str) -> dict | None:
    """Path like 'A:1/B:1/C:1'. Returns the occurrence dict or None."""
    parts = path.split("/")
    current = occs
    node = None
    for part in parts:
        node = next((c for c in current if c.get("name") == part), None)
        if node is None:
            return None
        current = node.get("children", [])
    return node


def _iter_occ(nodes: list):
    for n in nodes:
        yield n
        yield from _iter_occ(n.get("children", []))


def _find_occ_rot(occs: list, name: str) -> list | None:
    """Find an occurrence by leaf name anywhere in the tree and return its
    3x3 world rotation (from world_transform or world_transform_rm_cm)."""
    for n in _iter_occ(occs):
        if n.get("name") == name:
            wtf = n.get("world_transform") or n.get("world_transform_rm_cm")
            if wtf:
                return [row[:3] for row in wtf[:3]]
    return None
