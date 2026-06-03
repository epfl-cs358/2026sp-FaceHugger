"""Mesh-manifest and servo-occurrence lookups against `fusion_export.json`.

Pulls origin shifts, the link-body primary occurrence (for inertial
physics), and the per-role servo placements (shoulder/hip/knee) that
the URDF generator places as extra visuals on each link.
"""

from .fusion_export import _find_occ_by_path


def _primary_link_occurrence(mesh_entry: dict | None) -> str | None:
    """Pull the *main link body* occurrence path out of a mesh_files entry.

    For `body` rules, that's just `source_occurrences[0]`. For `combined`
    rules it's `parts[0].occurrence` — the rule convention is that the
    first part is the link body and any subsequent parts are rigidly-
    attached extras (e.g. the hip servo welded into leg_shoulder_L.stl).
    Used for inertial physics lookup; the mesh placement itself doesn't
    need this.
    """
    if not mesh_entry:
        return None
    if mesh_entry.get("source_type") == "combined":
        parts = mesh_entry.get("parts") or []
        if parts:
            return parts[0].get("occurrence")
        return None
    occs = mesh_entry.get("source_occurrences") or []
    return occs[0] if occs else None


def _mesh_shift(export: dict, mesh_name: str) -> list:
    """Return the origin_shift_mm that ExportBodiesToURDF applied to this
    STL during re-origin (or [0,0,0] if the mesh wasn't re-origined or the
    manifest is missing). This is the exact value to pass as
    URDF.link(..., origin_shift_mm=...) so the inertial CoM gets shifted
    back into the URDF link frame while the visual mesh sits at (0,0,0).
    """
    mf = export.get("mesh_files") or {}
    entry = mf.get(mesh_name) or {}
    return list(entry.get("origin_shift_mm", [0.0, 0.0, 0.0]))


def _servo_world_by_role(export: dict, role: str) -> list | None:
    """Look up the world_origin_mm of the servo instance assigned to `role`
    (shoulder/hip/knee) via mesh_files._servo_role_assignment. Returns None
    if no assignment or occurrence missing — caller should skip that visual.
    """
    mf = export.get("mesh_files") or {}
    roles = mf.get("_servo_role_assignment") or {}
    path = roles.get(role)
    if not path:
        return None
    occ = _find_occ_by_path(export["occurrences"], path)
    if occ is None:
        return None
    return occ.get("world_origin_mm")


def _servo_rot_by_role(export: dict, role: str) -> list | None:
    """3x3 rotation of the servo occurrence assigned to `role`, from its
    world_transform (or legacy world_transform_rm_cm) — rotation part of a 4x4."""
    mf = export.get("mesh_files") or {}
    roles = mf.get("_servo_role_assignment") or {}
    path = roles.get(role)
    if not path:
        return None
    occ = _find_occ_by_path(export["occurrences"], path)
    if occ is None:
        return None
    wtf = occ.get("world_transform") or occ.get("world_transform_rm_cm")
    if wtf is None:
        return None
    return [row[:3] for row in wtf[:3]]
